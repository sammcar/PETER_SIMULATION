# Plan — Ablation Study de la Red de Arbitración de Ganglios Basales

> **Objetivo:** responder a `R3-01`/`R3-02` de `paper-revision/PROGRESS.md` (referee de
> *Robotics and Autonomous Systems*, manuscrito `ROBOT-D-26-00122R1`): un ablation analysis
> comparativo, removiendo capas neuronales / conexiones inhibitorias, que justifique el uso de
> **Winner-Take-All (WTA)** frente a **lógica de umbral (threshold logic)** en la arbitración
> de comportamientos de PETER.
>
> **Deadline real: 2026-09-01** (dato para el referee, no la fecha límite del manuscrito).
> **Estado:** plan v2, refinado tras revisar `paper-revision/sections/` — pendiente de
> aprobación final antes de tocar código.
> **Rama de trabajo: `Deiv`** (todo el código de este plan vive ahí, nunca en `main`).
> **Repo de ejecución:** `PETER_SIMULATION` (este repo). La redacción de la respuesta al
> referee y las figuras/tablas finales del paper se hacen después, en `paper-revision/`,
> siguiendo su propio `README.md`/`CLAUDE.md` — no se mezclan aquí (solo lectura ya hecha
> de `sections/methodology.tex`, `results.tex` y `PROGRESS.md`/`OUTLINE.md` para alinear
> terminología y escenarios).

## 0. Alineación con el paper (hallazgos de `paper-revision/sections/`)

- **`R3-01`/`R3-02`** (`PROGRESS.md`) apuntan específicamente al **módulo de Ganglios
  Basales STN/GPe/GPi/STR** (`methodology.tex` línea 154, Ecs. `eq:stn`–`eq:str`) — **no** al
  WTA lateral del submódulo LIDAR (`Response`/`Aux`), que es un módulo distinto ("Obstacle
  Sensory Module"). Corrección respecto al plan v1.
- El paper **ya documenta** un contraste "lógica de umbral" propio, en el Módulo de
  Locomoción: umbral de 20° en `X5`/`X6` (Ecs. `eq:Z_5`/`eq:Z_6`) y `Ar=28.0` gateando `X17`
  (Ec. `eq:Z_17`) — `PROGRESS.md` lo señala como "natural in-paper contrast point". El modo
  `threshold_only` de la ablación debe ser conceptualmente equivalente a esa lógica ya
  existente, no una arquitectura nueva inventada.
- `results.tex` §"Quantitative Validation" ya define **4 escenarios con nombre fijo**:
  *Appetitive Targeting*, *Aversive Escape*, *Obstacle Evasion*, *Complex Navigation*
  (`tab:consolidated_simulation_metrics`: columnas Suite / Success % / $T_{sim}$ / Roll /
  Pitch, `N=15` réplicas/escenario, barrido de ruido $\sigma\in[0,4]$ — mapea 1:1 al
  parámetro `nl` de `red_neuronal.py`). *Complex Navigation* es explícitamente donde se
  arbitra entre 3 canales simultáneos — el escenario natural para el ablation de arbitración.
  *Appetitive Targeting* es el control sin conflicto.
- `results.tex` §Limitations ya admite que la fusión sensorial "emerges implicitly... through
  Winner-Take-All competition... has not been formally compared against standard fusion
  methods" — confirma que el gap es real y que el ablation es la pieza que falta.
- Consecuencia práctica: la ablación se redacta como una subsubsección nueva justo después de
  "Quantitative Validation: Multi-Modal Simulation Performance", reusando el mismo formato de
  tabla (+ columna de variante + λ), en vez de introducir métricas o escenarios nuevos.

---

## 1. Diagnóstico del sistema actual

Archivo de referencia: `src/red_neuronal.py` (nodo `network_publisher`).

| Mecanismo | Ubicación | Naturaleza |
|---|---|---|
| Circuito de ganglios basales (Gpi/Gpe/StN/StR, 3 canales: hostil/rojo, obstáculo/verde, apetente/azul) | `initctes()` L.80-83, dinámica L.348-362 | Continuo, con inhibición cruzada explícita entre canales (`-Gpe[j]`, `-StR[i]`) |
| WTA lateral clásico (16 neuronas `Response`/`Aux` del submódulo LIDAR) | `initctes()` L.145-150, dinámica L.332-336 | Inhibición lateral pura: `weights_r_r = -1` fuera de diagonal |
| Selección final de comando motor (giro / lateral / avance) | `run_network()` L.476-502 | **`if/elif` secuencial con prioridad fija**, comparado contra `epsilem` — lógica de umbral estática, no WTA dinámico |
| Histéresis de cambio de modo (C/H/X) | `run_network()` L.517-542 | Umbrales fijos (`z16>0.4`, `z15>0.6`...) + tiempo mínimo de permanencia (`min_dwell_time`) |

**Conclusión del diagnóstico:** el sistema mezcla dos paradigmas de arbitración en el mismo
nodo — un WTA dinámico (ganglios basales + inhibición lateral del LIDAR) y una lógica de
umbral estática en la capa de salida motora y de modos. El ablation debe aislar el aporte de
cada uno para poder afirmar cuantitativamente que el WTA es superior.

---

## 1bis. Bug de índices en `scripts/test_manager.py` (a corregir antes de correr nada)

`/neuron_activity` (publicado por `red_neuronal.py::publish_data()`) concatena, en orden fijo:
`Gpi(3) + Gpe(3) + StN(3) + StR(3) + z(20) + lidar(5) + activaciones_totales(16) + Response(16)
+ Aux(16)`. El bloque `z[]` (donde viven `X0`, `X14`, `X15`, `X17` del paper) **empieza en el
índice 12**, no en 0. `test_manager.py` lee 4 neuronas del bloque `z` sin ese offset:

| Uso en `test_manager.py` | Índice leído hoy | Índice correcto | Corresponde a (paper) |
|---|---|---|---|
| `_eval_appetitive`: `x17 = neurons[17]` | 17 | **29** | $X_{17}$ (stop, gatea `Ar=28.0`) |
| `_eval_evasive`: `x14_active = neurons[14]>0.3` | 14 | **26** | $X_{14}$ (modo X, evasión) |
| `_eval_terrain_c1`: `x0 = neurons[0]` | 0 | **12** | $X_0$ (rugosidad IMU) |
| `_eval_terrain_c1`: `x15 = neurons[15]` | 15 | **27** | $X_{15}$ (modo C, cuadrúpedo) |

Es un único bug sistemático (falta `Z_OFFSET = 12`), no cuatro bugs independientes. Se corrige
con una constante de módulo y `neurons[Z_OFFSET + k]` en los 4 sitios. Esto afecta también a
las suites ya existentes (`familia_a_apetitivo`, `familia_a_obstaculo`, `familia_c1_...`), no
solo a la ablación — se corrige una vez, beneficia a todo el orquestador.

---

## 2. Variantes de ablation (prioridad 3+1, vía parámetro ROS `ablation_mode`)

Mismo patrón que el parámetro `nl` (nivel de ruido) ya existente en `red_neuronal.py`
(`declare_parameter` + rama condicional), para no duplicar nodos ni launch files:

```bash
ros2 run peter_robot red_neuronal --ros-args -p ablation_mode:=<modo>
```

| Modo | Prioridad | Qué se desactiva (dentro de STN/GPe/GPi/STR, Ecs. `eq:stn`–`eq:str`) | Qué prueba |
|---|---|---|---|
| `full` (control) | — | Nada — sistema actual íntegro | Línea base |
| `no_lateral_inhibition` | 1 | Los términos de inhibición cruzada entre canales (`-Σ_j Gpe[j]` en `StN`, `-Gpe[i]`/`-StR[i]` en `Gpi`) se ponen a 0 — los 3 canales evolucionan desacoplados | Rol de los lazos inhibitorios en producir un único ganador (vs. co-activación/parálisis) |
| `threshold_only` | 1 | Las ecuaciones diferenciales de `STN`/`Gpi`/`Gpe`/`STR` se saltan por completo; el canal ganador se decide con un `argmax`/umbral estático sobre `Stimuli_i` — la misma familia de lógica que ya usa el Módulo de Locomoción (umbral 20° en `X5`/`X6`, `Ar=28.0` en `X17`) | Comparación directa WTA dinámico (ganglios basales) vs. lógica de umbral estática — el núcleo de R3-02 |
| `no_stn_str` | stretch | Retroalimentación subtalámica/estriatal (`STN`, `STR`) removida de las ecuaciones de `Gpi`/`Gpe` | Aporte específico de esa vía en la resolución de conflictos — solo si sobra tiempo |

Los modos se implementan como ramas dentro de `initctes()` (construcción de matrices/pesos)
y `run_network()` (ecuaciones), controladas por un único `if self.ablation_mode == ...`,
para que el resto del pipeline (launch, recorder, test_manager) no necesite saber qué modo
está activo — solo lo etiqueta. El WTA lateral del submódulo LIDAR (`Response`/`Aux`,
`weights_r_r`) **no se toca** — no es el objetivo de R3-01/R3-02 (ver §0).

---

## 3. Métricas (ya instrumentadas — no requieren nodos nuevos)

De `src/neural_recorder.py` (tópico `/experiment/metrics`):
- **λ — eficiencia de resolución de conflictos** (métrica central: 1.0 = decisión clara, →0 = parálisis).
- Latencia de decisión (estímulo → primer `cmd_vel` significativo).
- Varianza de disparo neuronal (ventana deslizante).
- Consistencia temporal (correlación coseno entre ciclos consecutivos).

De `src/red_neuronal.py` (tópico `/Metrics`) y `scripts/test_manager.py`:
- `Tresponse`, `Tswitch`, `roll_rms`, `pitch_rms`.
- Veredicto por corrida: `SUCCESS` / `FAILURE_TIPOVER` / `FAILURE_TIMEOUT` / `FAILURE_CRASH`.

---

## 4. Diseño experimental

- **Escenarios (nombres y suites ya existentes en `config/experiments_config.yaml` y en
  `results.tex`):**
  - **Complex Navigation** (`familia_b_compleja`, `multiple_stimuli.launch.py`) — 3 estímulos
    simultáneos (rojo/verde/azul). Es el escenario donde el paper ya reporta que se arbitra
    entre canales; el banco de pruebas natural para el ablation.
  - **Appetitive Targeting** (`familia_a_apetitivo`, `single_stimulus.launch.py`) — un solo
    estímulo, sin conflicto. Control: verifica que las ablations no rompen el caso simple.
  - *Obstacle Evasion* / *Aversive Escape* quedan fuera de esta primera pasada (agregar solo
    si sobra tiempo) — no aportan conflicto multi-canal adicional.
- **Réplicas: N=12** por combinación (variante × escenario) — ver justificación estadística
  acordada (potencia de Mann-Whitney con n=12 vs. n=5). `nl:=0` (sin ruido artificial) en
  todas las corridas de ablation, para no confundir el efecto estructural con la robustez a
  ruido — esa ya está cubierta por separado en `tab:consolidated_simulation_metrics`
  ($\sigma\in[0,4]$) para el modelo `full`.
- **Total de corridas:** 3 modos prioritarios × 2 escenarios × 12 réplicas = **72 runs**
  (+24 si se añade `no_stn_str` = 96 runs).
- **Estimado de tiempo:** Complex Navigation timeout 150s + Appetitive 45s + ~25s de
  warmup/teardown cada uno ⇒ ~170s y ~65s por corrida. 72 runs ≈ 2.4 h de simulación
  secuencial (96 runs ≈ 3.1 h).
- **Análisis:** medias ± IC por métrica y variante; test no paramétrico (Mann-Whitney U) de
  `full` vs. cada ablation en λ y latencia; tasa de éxito (`SUCCESS`) por variante en `test_manager`.
- **Tabla de salida:** mismo formato que `tab:consolidated_simulation_metrics`
  (Suite / Success % / $T_{sim}$ / Roll / Pitch) + columna `Ablation Mode` + columna λ
  (solo relevante en Complex Navigation) — para que se inserte en el paper sin reinventar
  el estilo de tabla.
- **Hipótesis a confirmar:** `full` (y `no_stn_str` si se corre) mantienen λ alto y latencia
  baja en Complex Navigation; `threshold_only` y `no_lateral_inhibition` degradan λ y/o
  aumentan latencia u oscilación (mayor `roll_rms`/`pitch_rms`) bajo conflicto — evidencia
  cuantitativa de que el WTA aporta sobre la lógica de umbral. En Appetitive Targeting se
  espera que todas las variantes se comporten similar (sin conflicto no debería haber
  diferencia grande) — eso mismo es parte del argumento: el WTA importa *específicamente*
  cuando hay conflicto.

---

## 5. Cambios de código requeridos (implementación — pendiente de aprobación), en orden

1. **`scripts/test_manager.py`** (bugfix, primero — desbloquea evaluación correcta de todo):
   constante `Z_OFFSET = 12` + corregir los 4 accesos `neurons[k]` → `neurons[Z_OFFSET + k]`
   (§1bis).
2. **`src/red_neuronal.py`**: parámetro `ablation_mode` (default `full`, patrón igual a `nl`)
   + ramas condicionales en `initctes()`/`run_network()` para `no_lateral_inhibition` y
   `threshold_only` (`no_stn_str` solo si sobra tiempo tras lo anterior).
3. **`config/experiments_config.yaml`**: descomentar `familia_a_apetitivo` y
   `familia_b_compleja` (ya existen, solo comentadas), duplicar cada una × 3 modos con
   `ablation_mode` en `dynamic_parameters`, `repetitions: 12`. No se crean suites nuevas de
   cero — se reusa lo ya validado.
4. **`scripts/test_manager.py`** (`_build_launch_args`): pasar `ablation_mode` como
   `--ros-args -p` igual que ya hace con `noise_level_idx`.
5. **Launch files** `single_stimulus.launch.py` / `multiple_stimuli.launch.py`: exponer el
   argumento `ablation_mode` y reenviarlo al nodo `red_neuronal`.
6. **`src/neural_recorder.py`**: sin cambios de lógica — solo etiquetar `experiment_type` con
   `<suite_name>_<ablation_mode>` para separar los CSV de salida.
7. Nuevo script **`scripts/analyze_ablation.py`**: agrega los CSV/JSON de
   `Findings/<suite>/`, calcula medias/IC + Mann-Whitney, genera la tabla en el formato de
   `tab:consolidated_simulation_metrics` (insumo directo para `paper-revision/`).
8. `docs/README_experiments.md`: añadir sección "Ablation — Basal Ganglia (R3-01/R3-02)"
   siguiendo el mismo formato que familias A/B ya documentadas.

---

## 6bis. Hallazgos adicionales durante la implementación

- **Bug de overrides de debug en `red_neuronal.py`** (encontrado comparando con la rama
  `fsm`, `fsm_gait_arbitration.py`): `R` estaba fijado a la constante `3.652`, `G`
  (obstáculo) forzado siempre a `0`, y `ang_s` fijado a `90°` — overrides de una prueba de
  terreno inclinado, nunca revertidos. Rompían por completo la arbitración multi-estímulo
  real (el canal verde nunca se activaba). Revertido (commit `4b52536`), confirmado contra
  el estado ya corregido en la rama `fsm`.
- **`fsm_gait_arbitration.py`** (rama `fsm`) es la implementación de **`R2-02`** ("FSM gait
  arbitration baseline"), un reviewer request distinto a R3-01/R3-02: reemplaza la
  arbitración del **Módulo de Decisión de Marcha** (modo C/H/X) por un FSM de prioridad
  fija — no toca STN/GPi/GPe/STR. No es reusable directamente para `threshold_only`, pero
  su patrón (umbral + prioridad fija sobre R/G/B) sirvió de referencia de estilo para
  implementarlo.

---

## 6. Checklist de ejecución

- [x] Confirmar que se está en rama `Deiv`
- [x] Corregir `Z_OFFSET` en `test_manager.py` (commit `5a5133f`)
- [x] Revertir overrides de debug de inclinación en `red_neuronal.py` (commit `4b52536`)
- [x] Implementar `ablation_mode` en `red_neuronal.py` (`full`, `no_lateral_inhibition`,
      `threshold_only`; commit `e47ca4f`) — `no_stn_str` pendiente, solo si sobra tiempo
- [ ] Smoke-test en Gazebo de los 3 modos (requiere contenedor `peter_simulation`, no
      disponible en este entorno de desarrollo — correr antes del barrido completo)
- [ ] Descomentar/duplicar suites en `experiments_config.yaml` (2 escenarios × 3 modos)
- [ ] Propagar `ablation_mode` por `test_manager.py` + launch files
- [ ] Compilar (`colcon build --symlink-install`) y smoke-test manual de cada modo (1 corrida corta cada uno)
- [ ] Ejecutar barrido (72 runs, ~2.4 h) vía `test_manager.py`/Makefile
- [ ] Exportar resultados (`make export-results`) a `docs/resultados/`
- [ ] Correr `scripts/analyze_ablation.py` → tabla/figura comparativa
- [ ] Pasar a `paper-revision/`: nueva subsubsección tras "Quantitative Validation: Multi-Modal
      Simulation Performance", tabla en formato `tab:consolidated_simulation_metrics`,
      respuesta a R3-01/R3-02 en `PROGRESS.md`
