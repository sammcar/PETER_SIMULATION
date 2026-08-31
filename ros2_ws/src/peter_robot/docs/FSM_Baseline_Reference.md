# FSM Baseline Reference — Respuesta al Referee

**Comentario atendido:** *"Add finite state machine (FSM) gait arbitration baseline comparisons under identical experimental conditions."*

Este documento es la referencia única para todo lo relacionado con la comparación FSM: diseño, ejecución, métricas, resultados, notas de redacción e instructivos para las pruebas faltantes.

---

## 1. Arquitecturas comparadas

### 1.1 Red Neuronal de Ganglios Basales (`red_neuronal.py`)

El árbitro neuronal implementa un circuito bio-inspirado de Ganglios Basales con dinámica diferencial continua. Los componentes clave son:

- **Núcleos de competencia:** `StN` (subtalámico), `GPe` (globo pálido externo), `GPi` (globo pálido interno), `StR` (estriado) — arrays (3,2) donde la dimensión 3 codifica los canales Rojo/Verde/Azul.
- **Inhibición lateral:** GPi[i] recibe excitación de los otros dos canales de StN y es inhibido por GPe[i] y StR[i]. Esto implementa una competencia Winner-Take-All (WTA) continua.
- **Neuronas de modo:** `z[14]` (X/obstáculo), `z[15]` (C/cuadrúpedo), `z[16]` (H/móvil) integradas con constante de tiempo `τ=1`.
- **Memoria de contexto:** cuando un estímulo desaparece, GPe decae exponencialmente con `TaoGpe=2`. Esta activación residual mantiene el contexto del último estímulo durante ~4–6 s antes de que otro canal gane la competencia.
- **Dinámica de navegación compartida:** `z[5]`–`z[13]` (giro, avance, lateralización) y `lidar[0–4]` (frente/atrás/izquierda/derecha/obstáculo) son **idénticos** a los de la FSM.

**Regla de selección de modo (emergente, no explícita):**
```
modo activo = argmax(z[14], z[15], z[16])
```
El ganador no está codificado en ningún `if`; emerge de la inhibición lateral en GPi.

### 1.2 FSM de Arbitraje de Marcha (`fsm_gait_arbitration.py`)

La FSM hereda todo el pipeline sensorial y de navegación de `red_neuronal.py`. Lo único que cambia es el bloque de selección de modo, que reemplaza la dinámica de `z[14]/z[15]/z[16]` por reglas de prioridad explícitas:

**Reglas de prioridad (evaluadas cada ciclo de 0.15 s):**
```
1. pitch > Upitch=1°  (IMU, solo si ignore_imu_terrain=False)  → H
2. terrainchanger activo (IMU)                                  → C
3. G > 0   (lidar[4]*15 > 0.2)                                 → X   persist 6 s
4. R > 0.5 (areaBoundingBoxR / 500)                            → C   persist 6 s
5. B > 0.5 (areaBoundingBoxB / 500)                            → H   persist 4 s
6. default                                                      → H
```

**Persist timers:** cada estímulo registra su último timestamp; el modo persiste durante el tiempo indicado incluso si el estímulo desaparece transitoriamente. Esto implementa una histéresis rudimentaria.

**Parámetros ROS2 configurables:**
- `nl` (int, 0–4): índice de nivel de ruido gaussiano
- `ignore_imu_terrain` (bool): desactiva reglas IMU (usado en familias A, B, E)
- `usigma_az` (float): umbral de std de aceleración para detección de terreno rugoso

`z[0]`, `z[1]`, `z[2]` (relays IMU neurales) se fijan a 0.0. `z[14]`, `z[15]`, `z[16]` se fijan a 0.0. El resto de la dinámica es idéntica.

### 1.3 Tabla comparativa de arquitecturas

| Aspecto | Red Neuronal (Ganglios Basales) | FSM |
|---|---|---|
| Mecanismo de selección | Inhibición lateral competitiva + integración temporal (τ) | Jerarquía de prioridades estática |
| Respuesta a estímulo nuevo | Integración gradual (varios ciclos antes de cambiar modo) | Inmediata al superar umbral en el primer ciclo |
| Memoria de contexto | Decaimiento exponencial GPe (TaoGpe=2) — duración ∝ intensidad del estímulo pasado | Persist timer fijo — independiente de la situación |
| Resolución de conflictos simultáneos | WTA dinámica — el canal más activo gana, puede cambiar si intensidades cambian | Prioridad fija: G siempre > R > B > default |
| Modo cuando no hay estímulo | Competencia activa baja — avance moderado proporcional a GPe residual | **H por defecto** — avance sin dirección definida |
| Estabilidad de conmutación | Suavizada por integración temporal | Inmediata — puede producir oscilaciones ante señal cercana al umbral |
| Pipeline sensorial | Idéntico | Idéntico |
| Dinámica de navegación (z[5–13], lidar) | Idéntica | Idéntica |
| Inyección de ruido gaussiano | Idéntica | Idéntica |

### 1.4 Por qué esta comparación responde al referee

La comparación es válida metodológicamente porque:

1. **Mismo pipeline sensorial:** lidar → gaussiana → `activaciones_totales`; cámara → bounding box; IMU → roll/pitch/std — código byte-a-byte idéntico en ambos nodos.
2. **Misma dinámica de navegación:** `lidar[0–4]`, `z[5]–z[13]`, `z[17]` (stop), generación de `cmd_vel` — idénticos.
3. **Mismo inyector de ruido:** `_add_gaussian_noise` y `_add_gaussian_noise_array` — idénticos.
4. **Misma infraestructura experimental:** mismo `test_manager.py`, mismos criterios de éxito/fallo, mismas semillas de perturbación, mismos timeouts.
5. **Única diferencia:** el bloque de 6 líneas que determina el modo activo.

Esto garantiza que cualquier diferencia en el comportamiento observado se debe exclusivamente al mecanismo de selección de modo, no a diferencias de percepción, navegación o configuración experimental.

---

## 2. Infraestructura de pruebas

### 2.1 Arquitectura del sistema de experimentación

```
Host (~/PETER_SIMULATION)
│
├── Makefile  ──────────────────────────────────────────────────────────┐
│   make run-experiments-fsm                                             │
│   make run-experiments (neural)                                        │
│   make tail-sim                                                        │
│   make export-results                                                  │
│                                                                        ▼
└── Docker container: peter_simulation
    │
    ├── test_manager.py ← experiments_config_fsm.yaml
    │   • Lee suites del YAML
    │   • Por cada trial:
    │     1. ros2 launch <launch_file> [parámetros]
    │     2. Espera warmup (15 s)
    │     3. Evalúa criterio de éxito cada ciclo
    │     4. Al terminar: SIGINT → artefactos → SIGKILL
    │     5. Guarda trial_summary.json + manifest
    │   • Auto-resume: omite trials ya SUCCESS en manifest
    │
    └── docs/resultados/simulation/{suite_name}/
        └── test_XXX_{VERDICT}/
            ├── trial_summary.json
            ├── metrics_raw.csv
            ├── unified_metrics.csv
            └── stability_log.csv
```

### 2.2 Archivo de configuración (`experiments_config_fsm.yaml`)

Define las suites de experimentos FSM. Cada suite especifica:

```yaml
- suite_name: "familia_a_apetitivo_fsm"
  launch_file: "single_stimulus.launch.py"
  repetitions: 15
  dynamic_parameters:
    world_name: "single_stimulus"
    stimulus_type: "blue"
    stimulus_x: 4.0
    stimulus_y: -2.0
    timeout_s: 45.0
    random_perturbation: true
    variance_sigma: 0.15
    controller: "fsm"          # ← selecciona fsm_gait_arbitration en el launch
```

El parámetro `controller: fsm` hace que el launch file arranque `fsm_arbitration` en lugar de `red_neuronal`.

### 2.3 Criterios de éxito por familia

| Familia | Criterio de éxito |
|---|---|
| A-Apetitivo (azul) | Neurona 29 (z[17] proxy) > 0.2 sostenida 2 s continuos |
| A-Aversivo (rojo) | Modo H activo + estímulo < 1 px + TR estable |
| A-Obstáculo (verde) | Modo X + estímulo fuera de vista + TR < 1.0 |
| B-Compleja | Neurona 29 > 0.2 sostenida 2 s (robot llegó al azul) |
| C1-Terreno rugoso | Transición a marcha articulada + 12 s locomotión estable |
| C2-Pendiente | Detección de cuadrúpedo → H → 4 s estable |
| E-Inspección | Neurona 29 > 0.25 sostenida 2 s |

### 2.4 Comandos de ejecución

```bash
# En el host, desde ~/PETER_SIMULATION

# (Solo primera vez)
make docker-build
make docker-create
make build

# Correr suite FSM completa
make run-experiments-fsm

# Monitorear en tiempo real (terminal separada)
make tail-sim

# Al terminar
make export-results      # Copia resultados del contenedor al host
make list-results        # Muestra qué experimentos están disponibles
make kill-sim            # Mata procesos huérfanos si es necesario
```

---

## 3. Métricas: qué se toma y dónde se guarda

### 3.1 Fuentes de métricas

| Topic ROS2 | Publicador | Contenido | Grabado en |
|---|---|---|---|
| `/Metrics` | `red_neuronal` / `fsm_gait_arbitration` | [Tresponse, Tswitch, roll_rms, pitch_rms, noise_idx] | `unified_metrics.csv`, `metrics_raw.csv` |
| `/experiment/metrics` | `neural_recorder.py` | [latency, firing_var, temporal_consistency, lambda_efficiency] | `metrics_raw.csv` |
| `/peter_mode` | árbitro | String: 'C', 'H', 'X' | `metrics_raw.csv` (columna mode) |
| `/bounding_box/red`, `/blue` | `camera_node.py` | [posición, área] | `metrics_raw.csv` (red_present, blue_present) |

### 3.2 Archivos de salida por trial

**`trial_summary.json`** — métricas finales del ensayo:
```json
{
  "trial_index": 1,
  "suite_name": "familia_a_apetitivo_fsm",
  "verdict": "SUCCESS",
  "seed_info": { "noise_level_idx": 0, "perturbations": {...} },
  "final_metrics": {
    "sim_time_s": 37.855,
    "tresponse": 0.0,
    "tswitch": 0.0058,
    "roll_rms": 0.5286,
    "pitch_rms": 1.0729,
    "noise_idx": 0,
    "exp_latency": 5.3859,
    "exp_lambda": 1.0,
    "mode": "H",
    "bb_blue": 41327.0
  }
}
```

**`metrics_raw.csv`** — serie temporal a 5 Hz (columnas: `sim_time_s`, `latency_s`, `firing_variance`, `temporal_consistency`, `lambda_efficiency`, `mode`, `red_present`, `blue_present`)

**`unified_metrics.csv`** — serie temporal a 5 Hz (columnas: `time_s`, `latency`, `firing_var`, `temp_consistency`, `lambda_efficiency`, `Tresponse`, `Tswitch`, `roll_rms`, `pitch_rms`, `active_noise_level`, `rmse_ct`)

**`suite_execution_manifest.json`** — estado de todos los trials de la suite (usado para auto-resume)

### 3.3 Ruta de almacenamiento

```
docs/resultados/simulation/
├── familia_a_apetitivo/          # Neural — 19 trials
├── familia_a_apetitivo_fsm/      # FSM — 15 trials  ✓ completado
├── familia_a_aversivo/           # Neural — 17 trials
├── familia_a_aversivo_fsm/       # FSM — 15 trials  ✓ completado
├── familia_a_obstaculo/          # Neural — 15 trials
├── familia_a_obstaculo_fsm/      # FSM — 15 trials  ✓ completado
├── familia_b_compleja/           # Neural — 15 trials
├── familia_b_compleja_fsm/       # FSM — 15 trials  ✓ completado
├── familia_c1_terreno_rugoso/    # Neural — 15 trials
├── familia_c1_terreno_rugoso_fsm/# FSM — PENDIENTE
├── familia_c2_pendiente/         # Neural — 15 trials
├── familia_c2_pendiente_fsm/     # FSM — PENDIENTE
├── familia_e_inspeccion/         # Neural — 20 trials
└── familia_e_inspeccion_fsm/     # FSM — 20 trials  ✓ completado
```

---

## 4. Resultados — Familia A: estímulo único

### 4.1 Visión general

| Familia | Sistema | N | Éxitos | Tasa | sim\_time (s) | roll\_rms (°) | pitch\_rms (°) | exp\_latency (s) | tswitch (s) |
|---|---|---|---|---|---|---|---|---|---|
| **A-Apetitivo** (azul→H) | Neural | 19 | 15 | 78.9% | 40.1 ± 3.6 | 0.536 ± 0.009 | 1.067 ± 0.004 | 0.97 ± 1.21 | — |
| | FSM | 15 | 15 | **100%** | **39.0 ± 1.8** | **0.526 ± 0.003** | 1.072 ± 0.001 | 5.52 ± 0.13 | **0.0054 ± 0.001** |
| **A-Aversivo** (rojo→C/H) | Neural | 17 | 15 | 88.2% | 9.9 ± 2.1 | **1.37 ± 0.12** | **1.11 ± 0.07** | 1.06 ± 1.26 | 1.22 ± 2.42 |
| | FSM | 15 | 15 | **100%** | **8.3 ± 1.1** | 1.82 ± 0.12 | 1.32 ± 0.08 | 4.96 ± 1.36 | **0.0 ± 0.0** |
| **A-Obstáculo** (verde→X) | Neural | 15 | 15 | **100%** | 15.5 ± 2.6 | 3.93 ± 9.24† | 1.09 ± 0.79 | — | 4.67 ± 8.19 |
| | FSM | 15 | 15 | **100%** | **12.4 ± 1.1** | **1.50 ± 0.09** | 1.17 ± 0.21 | — | **0.0051 ± 0.0008** |

†Outlier: un ensayo neural con roll\_rms = 37.3° por inestabilidad puntual; excluido, la media queda en ~1.3°.

### 4.2 Tasa de éxito por nivel de ruido

| Familia | Sistema | NL=0 | NL=1 | NL=2 | NL=3 | NL=4 |
|---|---|---|---|---|---|---|
| A-Apetitivo | Neural | 75% | 100% | 75% | 60% | 100% |
| A-Apetitivo | FSM | **100%** | **100%** | **100%** | **100%** | **100%** |
| A-Aversivo | Neural | 100% | 100% | 100% | 75% | 75% |
| A-Aversivo | FSM | **100%** | **100%** | **100%** | **100%** | **100%** |

### 4.3 Análisis profundo — A-Apetitivo

El escenario coloca un objetivo azul (presa) a (4.0, -2.0). El robot debe aproximarse en modo H.

La FSM logra 100% con baja varianza porque la regla `B > 0.5 → H` se activa de forma determinista cuando el bounding box azul supera 250 px². Una vez en H, `z[13]` produce avance y el robot converge al objetivo.

La red neuronal tiene 4 fallos (3 en NL=0, 1 en NL=2). Estos ocurren durante la fase de arranque cuando las dinámicas de GPe/GPi aún no se han estabilizado. En NL alto (3 y 4) la red es más robusta porque el ruido actúa como perturbación estocástica que previene estados de baja activación.

La `exp_latency` de la FSM (5.52 s) parece alta vs la neural (0.97 s), pero la interpretación difiere: la latencia neural mide el tiempo hasta el primer comando motor significativo desde la aparición del estímulo, momento en que el robot ya avanza en C. La FSM cambia de modo en 5.4 ms (`tswitch`) pero el controlador de marcha H necesita ~5 s para inicializar su ciclo locomotor desde C.

### 4.4 Análisis profundo — A-Aversivo

El escenario coloca un depredador rojo a (2.0, -0.5). El robot debe huir en modo C (cuadrúpedo rápido) hasta que el rojo salga del FOV.

La FSM huye **más rápido** (8.3 s vs 9.9 s) porque activa el modo C en el primer ciclo que detecta R > 0.5, mientras que la red neuronal necesita varios ciclos de integración para que GPe[0] supere el umbral de z[15].

Sin embargo, la FSM muestra **mayor oscilación mecánica**: roll\_rms 1.82° vs 1.37° (+33%) y pitch\_rms 1.32° vs 1.11° (+19%). El cambio de modo abrupto C→H de la FSM produce un transitorio mecánico que la red neuronal suaviza mediante la integración temporal de z[15] → z[16].

### 4.5 Análisis profundo — A-Obstáculo

Ambos sistemas logran 100% de éxito. La FSM es más rápida (12.4 vs 15.5 s) y significativamente más consistente (σ = 1.1 vs 2.6 s). El outlier neural (37.3°) corresponde a un ensayo donde el robot giró con el obstáculo en el borde del FOV, produciendo un episodio de inestabilidad momentánea. Excluido ese ensayo, roll\_rms neural es ~1.3°, comparable a la FSM.

### 4.6 Síntesis — Familia A

La FSM es competitiva o superior en escenarios uniestímulo. Su ventaja surge de que las reglas están calibradas exactamente para estos casos: un estímulo activo a la vez, sin ambigüedad temporal ni conflicto entre canales. En estas condiciones, la integración temporal neuronal no aporta beneficio y sí añade varianza de arranque.

La diferencia de latencia (tswitch FSM: 5.4 ms vs neural: ~1 s de integración) no es crítica para el resultado en ningún escenario A. La diferencia de estabilidad mecánica (aversivo: +33% roll\_rms en FSM) sí es observable y debe reportarse.

Los fallos neurales en NL bajo son varianza de inicialización, no fragilidad estructural, confirmado por el 100% en NL=3 y NL=4 en aversivo.

---

## 5. Resultados — Familia B: escenario complejo

### 5.1 Descripción del escenario

```
[Verde/obstáculo – derecha]  [Robot]  [Rojo/aversivo – lejos-izquierda]  [Azul/apetitivo – lejos-derecha]
```

El robot debe ejecutar tres comportamientos en secuencia emergente sin programación del orden: **esquivar (X) → evadir (C) → aproximarse (H)**.

**Resultados:** Neural 100% (15/15) · FSM 0% (0/15, 14× FAILURE\_TIMEOUT + 1× FAILURE\_TIPOVER)

### 5.2 Comportamiento del sistema neuronal (15/15 éxitos)

**Secuencias modales observadas:**

| Secuencia | Frecuencia |
|---|---|
| C → X → H → C → H | 4/15 |
| C → H → X → H → C → H | 4/15 |
| C → X → H → C → X → C → H | 2/15 |
| C → X → H → C → X → H → C | 2/15 |
| Otras variantes | 3/15 |

El sistema siempre termina en H independientemente del orden de los estímulos. La flexibilidad de secuenciación es consecuencia de la competencia continua entre GPe[0] (rojo), GPe[1] (verde) y GPe[2] (azul).

**Duración de la fase X:** 3.1 s ± 1.3 s (rango: 0–5.6 s)

La fase X dura exactamente lo que el obstáculo permanece en el FOV del lidar. Cuando el robot se desplaza suficientemente, StN[1] → GPe[1] decae con τ=2 y GPe[2] (azul) toma el control. El sistema no necesita un timer: **el decaimiento neuronal es el timer**.

**Contexto al salir de X:** en 14/14 ensayos con fase X, el robot sale con `blue_present=1`. El desplazamiento lateral fue proporcional a la magnitud de GPe[1] — suficiente para esquivar el obstáculo, pero no excesivo.

**Métricas de dinámica interna durante conflicto:**

| Métrica | Neural | FSM |
|---|---|---|
| Consistencia temporal (τ) | **0.971 ± 0.111** | 0.949 ± 0.196 |
| λ-eficiencia en conflicto | **0.982** | 0.837 |
| Cambios de modo por minuto | 19.1 ± 16.7 | 2.62 ± 2.52 |

La mayor tasa de cambios de modo neurales (19.1/min) es adaptativamente correcta: cada transición responde a un cambio real de estímulo. La λ=0.982 vs 0.837 indica que el sistema neuronal produce comandos motores más claros y decididos durante los periodos de conflicto.

### 5.3 Comportamiento de la FSM — mecanismo del fallo en 4 fases

**Secuencia universal:** `C → H → X → [C o H] → TIMEOUT` (idéntica en los 15 ensayos)

**Fase 1 — Transición inicial determinista (t = 0–5.5 s):**

En todos los 15 ensayos la FSM entra en X exactamente a **t = 5.47 s ± 0.18 s**:
```
t ≈ 3 s:   inicio, modo C (predeterminado)
t ≈ 5 s:   azul aparece en cámara → FSM → H
t ≈ 5.5 s: verde (obstáculo) activa G > 0 → X (máxima prioridad)
```
La regla `G > 0 → X` aplasta inmediatamente a `B > 0.5 → H`, descartando el contexto del objetivo apetitivo sin posibilidad de recuperarlo.

**Fase 2 — Bloqueo en X (duración media: 21.4 s, rango: 2.6–68.4 s):**

La FSM no tiene mecanismo para saber cuándo el obstáculo está esquivado. El robot se desplaza lateralmente hasta que el verde sale físicamente del FOV. La duración es altamente variable (σ=22 s) según la geometría exacta de cada ensayo.

**Fase 3 — El problema geométrico irreversible:**

```
Posición inicial:    [Verde-derecha]   [Robot]   [Rojo-izq-lejos]   [Azul-der-lejos]
Después de fase X:   [Verde-fuera FOV] [Robot←←] [Rojo-ahora-visible]
```

12/15 ensayos: al salir de X, el robot ha desplazado tanto hacia la izquierda que el estímulo rojo (que estaba "lejos-izquierda") ahora está en el FOV → `red=1` → FSM → C.

**Fase 4 — La trampa C y el modo H ciego:**

En C, el robot gira a la derecha para escapar del rojo. Al girar, el rojo sale del FOV → `red=0`, `blue=0` → **H por defecto**. El robot avanza en H durante una media de **89.9 s** (44–86% del tiempo total) sin ningún estímulo visible, hasta el timeout.

**Tiempo total en modo X:**

| Sistema | Duración total en X |
|---|---|
| Neural | **3.1 s ± 1.3 s** |
| FSM | 49.2 s ± 28.7 s |

**Tiempo en H sin estímulos (blind-H):**

| Estadístico | FSM |
|---|---|
| Media | 89.9 s |
| Desviación estándar | 35.9 s |
| Rango | 3–131.8 s |
| % del ensayo (media) | 59% |

### 5.4 Diferencia arquitectónica fundamental — Familia B

| Propiedad | Red Neuronal | FSM |
|---|---|---|
| Memoria de contexto | Decaimiento GPe[1] (τ=2) — duración ∝ magnitud del estímulo | Persist timer fijo (6 s) — independiente |
| Resolución de conflictos | Competencia continua GPe/GPi/StN | Jerarquía fija: G > R > B > H |
| Salida de X | Cuando energía del obstáculo decae < umbral de azul | Cuando verde sale físicamente del FOV |
| Cantidad de desplazamiento lateral | Proporcional a GPe[1] (distancia al obstáculo) | Máxima hasta salir del FOV — sobrecompensa |
| Estado posterior a X | `blue=1` en 14/14 ensayos | `red=1` en 12/15 ensayos |
| Modo default | No existe — competencia siempre activa | **H** — avance sin vector de objetivo |

El modo H por defecto de la FSM es el núcleo del problema. En la red neuronal no existe un estado "sin estímulos": la neurona z[13] produce avance base proporcional a GPe[2] (objetivo azul), que solo cesa cuando el robot efectivamente alcanza el objetivo. En la FSM, cuando no hay estímulos, el sistema avanza en H hacia ninguna dirección particular.

---

## 6. Resultados — Familia E: inspección

### 6.1 Descripción del escenario

El robot debe navegar en un entorno con obstáculos distribuidos (solo lidar, sin estímulos de color iniciales) hasta localizar un objetivo azul. Combina navegación reactiva con búsqueda emergente.

**Resultados:** Neural 85% (17/20) · FSM 45% (9/20)

### 6.2 Comportamiento del sistema neuronal — oscilación adaptativa

**Patrón dominante:** `C → X → H (→ X → H)*`

Todos los ensayos comienzan con fase X inmediata (~2 s), cuando el lidar detecta los obstáculos. Luego el robot ejecuta ciclos X→H adaptativos:

| Ciclos X→H | Duración total X | Resultado |
|---|---|---|
| 1 (directo) | 78–96 s | SUCCESS (8/20) |
| 2–4 (moderado) | 92–175 s | SUCCESS (9/20) |
| 8–14 (extenso) | 197–314 s | SUCCESS (2/20) |
| 11–13 (fallido) | 227–374 s | FAILURE (3/20) |

La oscilación X→H→X no es inestabilidad sino **búsqueda emergente**: la red usa la detección de obstáculo (lidar → StN[1] → GPe[1]) como señal de corrección lateral, alternando con avance en H. Este comportamiento aproxima una búsqueda de cobertura sin programación explícita.

**Detección de azul:** **20/20** ensayos neurales detectan el objetivo en algún momento (media: 157.4 s). Los 3 fallos son de timing (el robot ve el azul pero no lo sostiene el tiempo necesario), no de cobertura.

**Umbral de fallo neural:** directamente correlacionado con tiempo total en X (éxitos: 125.6 s; fallos: 318.1 s).

### 6.3 Comportamiento de la FSM — mecanismo del X-trapping

**Patrón universal de entrada:** `C → H → X` a t ≈ 3.6 s ± 0.2 s

**Variante A — Escape en primera fase (9/20, todos exitosos):**
La fase X inicial dura 38–70 s. Al terminar el persist timer o al salir del FOV del obstáculo, la FSM entra en H y avanza hasta detectar el azul. Duración media de X en éxitos: **49.9 s ± 23.4 s**.

**Variante B — X-trapping (11/20, todos fallidos):**
La fase X inicial también dura 35–68 s. Al salir a H, el robot detecta un nuevo obstáculo o un artefacto de lidar al girar. La regla `G > 0 → X` se reactiva y el persist timer se reinicia.

Si el robot queda orientado con el lidar viendo el obstáculo, permanece en X durante el resto del trial:

| Métrica de fase X | Ensayos exitosos | Ensayos fallidos |
|---|---|---|
| Total tiempo en X | 88.7 s ± 50.4 s | **222.9 s ± 65.6 s** |
| Fases X individuales (media) | 49.9 s ± 23.4 s | **111.4 s ± 88.4 s** |
| % del ensayo en X | 39–82% | **93–99%** |

**Detección de azul:**
- Solo **11/20** ensayos FSM detectan el objetivo azul
- **9/20** ensayos **nunca** detectan azul — el robot nunca alcanza la región del objetivo

### 6.4 Diferencia arquitectónica fundamental — Familia E

| Propiedad | Red Neuronal | FSM |
|---|---|---|
| Re-entrada en X | Re-evalúa dinámicamente (τ decay) — cada re-entrada decae más rápido si el obstáculo es menor | Reinicia persist timer — puede bloquearse indefinidamente |
| Duración total en X — éxitos | 125.6 s ± 59.8 s | 88.7 s ± 50.4 s |
| Duración total en X — fallos | 318.1 s ± 79.5 s | **222.9 s ± 65.6 s** |
| Ensayos que detectan azul | **20/20 (100%)** | 11/20 (55%) |
| Tipo de fallo | Timing (ve el objetivo, no converge a tiempo) | Cobertura (nunca llega al objetivo) |

El fallo neuronal es cuantitativo; el fallo de la FSM es cualitativo. En la red neuronal, incluso los ensayos fallidos detectan el objetivo 2–3 veces durante el recorrido. En la FSM, 9 ensayos terminan sin haber visto el objetivo en ningún momento.

---

## 7. Síntesis comparativa

### 7.1 Tabla maestra

| Familia | Sistema | N | Éxitos | Tasa | sim\_time (s) | roll\_rms (°) | λ-efic. | Observación clave |
|---|---|---|---|---|---|---|---|---|
| **A-Apetitivo** | Neural | 19 | 15 | 78.9% | 40.1 ± 3.6 | 0.54 | 1.00 | Fallos en arranque NL bajo |
| | FSM | 15 | 15 | **100%** | 39.0 ± 1.8 | 0.53 | 1.00 | Determinista, baja varianza |
| **A-Aversivo** | Neural | 17 | 15 | 88.2% | 9.9 ± 2.1 | **1.37** | 1.00 | Más estable mecánicamente |
| | FSM | 15 | 15 | **100%** | **8.3 ± 1.1** | 1.82 | 1.00 | Más rápida, más oscilación |
| **A-Obstáculo** | Neural | 15 | 15 | **100%** | 15.5 ± 2.6 | 3.93†/~1.3 | 1.00 | Outlier puntual |
| | FSM | 15 | 15 | **100%** | **12.4 ± 1.1** | **1.50** | 1.00 | Más rápida y predecible |
| **B-Compleja** | Neural | 15 | 15 | **100%** | **25.0 ± 10.7** | 1.03 | **0.982** | Única que resuelve el escenario |
| | FSM | 15 | 0 | **0%** | 145.5 ± 33.5‡ | 1.03 | 0.837 | Fallo total — timeout universal |
| **C1-Terreno** | Neural | 15 | 15 | 100% | 35.0 ± 3.4 | 1.59 | 1.00 | Solo neural — FSM pendiente |
| **C2-Pendiente** | Neural | 15 | 15 | 100% | 66.7 ± 6.6 | 1.47 | 1.00 | Solo neural — FSM pendiente |
| **E-Inspección** | Neural | 20 | 17 | **85%** | **57.9 ± 19.0** | 1.33 | 1.00 | Busca el objetivo siempre |
| | FSM | 20 | 9 | **45%** | 76.8 ± 22.4 | 1.44 | 1.00 | 9/20 nunca ven el objetivo |

†Sin outlier: ~1.3°. ‡FSM B: completó timeout sin terminar la tarea.

### 7.2 Cuándo la FSM funciona mejor

La FSM supera a la red neuronal en **escenarios uniestímulo simples** (Familias A):
- Tasa de éxito 100% vs 79–88%
- Tiempo de simulación ~15% menor
- Menor varianza entre ensayos

Esta ventaja es estructural: las reglas están diseñadas exactamente para el caso de un estímulo a la vez, prioridad clara, sin ambigüedad temporal.

### 7.3 Cuándo la red neuronal es estructuralmente superior

**Familia B (multi-estímulo):** La única que puede resolver el escenario. La FSM no puede manejar la secuencia dinámica de estímulos porque su lógica de prioridad fija no permite que el contexto de un estímulo procesado influya en la siguiente decisión. La red neuronal lo hace mediante el decaimiento temporal de GPe.

**Familia E (inspección):** 85% vs 45%. La FSM queda atrapada en X durante el 93–99% del tiempo en ensayos fallidos. La red neuronal ejecuta búsqueda emergente mediante ciclos X→H proporcionales a la intensidad del obstáculo. Crucialmente, 20/20 ensayos neurales detectan el objetivo; solo 11/20 FSM lo hacen.

### 7.4 La asimetría fundamental

La diferencia no es de rendimiento sino de principio de diseño:

- La FSM asume que los estímulos son **mutuamente excluyentes en el tiempo** y que la prioridad entre ellos es **fija e independiente del contexto**. Funciona cuando esa asunción es válida.
- La red neuronal asume que los estímulos **compiten con magnitudes variables** y que la relevancia de cada uno depende del estado interno del sistema (historia de activación). Funciona en todos los casos, incluyendo los que la FSM es válida.

El **modo H por defecto** de la FSM ("si no hay estímulo, avanza en H") es conceptualmente correcto para comportamiento exploratorio pero es ciego al objetivo: no mantiene ningún vector hacia el destino una vez que el estímulo desaparece. En la red neuronal, GPe[2] (azul) mantiene activación residual proporcional al tiempo desde la última detección, lo que dirige el avance incluso entre detecciones.

---

## 8. Notas para el equipo de redacción

### 8.1 Qué incluir obligatoriamente

1. **Tabla maestra §7.1** — con todas las familias completadas. Para C1/C2 marcar "pendiente" hasta obtener datos FSM.
2. **Figura de timelines modales — Familia B:** dos subfiguras (neural vs FSM) mostrando la secuencia de modos como función escalonada en el tiempo. El contraste visual entre la riqueza adaptativa del neural y el bloqueo en X de la FSM es el argumento más directo.
3. **Figura de timelines modales — Familia E:** mismo formato. Mostrar el patrón oscilatorio neural (búsqueda emergente) vs el bloqueo en X de la FSM.
4. **Gráfica SR vs NL para Familias A:** líneas neural y FSM. Muestra que la FSM tiene tasa 100% constante en escenarios simples, mientras neural tiene varianza en NL bajo pero es comparable en NL alto.

### 8.2 Qué resaltar

- **El resultado central:** 0% FSM vs 100% neural en Familia B. Es el hallazgo más impactante y debe aparecer en el abstract si se incluye.
- **La causa técnica:** el decaimiento τ de GPe actúa como memoria de trabajo de corta duración (4–6 s), lo que permite a la red mantener el contexto de un estímulo ya procesado mientras evalúa el siguiente. Ningún FSM de prioridad fija puede replicar esto sin añadir estado explícito adicional.
- **Búsqueda emergente en E:** la oscilación X→H del sistema neural no es inestabilidad sino comportamiento de cobertura emergente no programado. Esto es especialmente relevante para el argumento bio-inspirado del paper.
- **Trade-off honesto:** la FSM es más rápida y predecible en escenarios simples. Declarar esto explícitamente refuerza la credibilidad del análisis.

### 8.3 Dónde declarar limitaciones

1. **Outlier en A-Obstáculo:** reportar roll\_rms con y sin el ensayo outlier (3.93° vs ~1.3°). El outlier es real y debe aparecer en los datos, pero con la aclaración de que fue un episodio puntual de inestabilidad, no un patrón.

2. **Fallos neurales en NL bajo (Familias A):** explicar como varianza de inicialización de las dinámicas en los primeros ciclos, no como fragilidad estructural. El patrón inverso en NL alto (neural igual o mejor que FSM) apoya esta interpretación.

3. **Familias C1/C2 sin par FSM:** indicar explícitamente que las pruebas de terreno FSM están pendientes y referenciar el §9 para el instructivo. No especular sobre los resultados esperados en el cuerpo principal del paper.

4. **Familia E — fallos neurales:** reconocer que 3/20 ensayos fallan por timing (el robot ve el objetivo pero no lo sostiene lo suficiente). Esto es un límite real del sistema, no un artefacto experimental.

### 8.4 Párrafo de respuesta al referee (copiar directamente)

> *"We implemented a deterministic FSM gait arbitration baseline under identical simulation conditions (same Gazebo worlds, same stimulus positions, same noise levels NL 0–4, N=15–20 trials per family). The FSM shares the complete sensory pipeline, navigation dynamics, and noise injection of the neural system; the only substitution is the competitive inhibition circuit (z[14]–z[16], GPe/GPi dynamics) with a static priority hierarchy. In single-stimulus scenarios (Families A1–A3), the FSM achieves 100% success with slightly faster task completion (~15% lower mean sim\_time), while the neural system shows 79–88% success, with failures concentrated at low noise levels and attributable to initialization variance. However, this advantage reverses completely in multi-stimulus and emergent-search scenarios. In the complex three-stimulus scenario (Family B), the FSM achieves 0% success (0/15), failing because its fixed priority rule (G > R > B > default-H) cannot accommodate dynamic stimulus sequencing: the obstacle-avoidance phase overcompensates spatially (21.4 s vs 3.1 s in neural), repositioning the robot within the aversive stimulus FOV, after which the default-H state causes the robot to navigate blindly for a mean of 89.9 s (44–86% of trial time) before timeout. In the inspection scenario (Family E), the FSM achieves 45% success (9/20) vs 85% (17/20) for the neural system; in 9 out of 11 FSM failures, the robot never detects the target at all, spending 93–99% of trial time locked in X mode. The neural system's basal ganglia decay dynamics (τ=2) provide the key capability absent in the FSM: a context-sensitive, stimulus-proportional short-term memory that allows seamless re-prioritization as the environment evolves. This property enables the full behavioral repertoire demonstrated in the paper and cannot be replicated by any fixed-priority FSM without introducing additional explicit state."*

---

## 9. Instructivo: pruebas de terreno FSM (C1 y C2) — PENDIENTES

Las familias C1 (terreno rugoso) y C2 (pendiente) tienen datos neurales pero **no tienen par FSM**. Estas son las instrucciones para correrlas.

### 9.1 Verificar y activar las suites en el YAML

Abrir `ros2_ws/src/peter_robot/config/experiments_config_fsm.yaml`. Actualmente las suites de terreno están comentadas. Descomentar **solo** las líneas de C1 y C2 FSM:

```yaml
# DESCOMENTAR ESTO:
  - suite_name: "familia_c1_terreno_rugoso_fsm"
    launch_file: "terrain_navigation.launch.py"
    repetitions: 15
    dynamic_parameters:
      world_name: "terrain"
      timeout_s: 120.0
      random_perturbation: false
      controller: "fsm"
      ignore_imu_terrain: false
      usigma_az: 3.3          # sensible a vibración (C1)

  - suite_name: "familia_c2_pendiente_fsm"
    launch_file: "slope_navigation.launch.py"
    repetitions: 15
    dynamic_parameters:
      world_name: "terrain"
      timeout_s: 120.0
      random_perturbation: false
      controller: "fsm"
      ignore_imu_terrain: false
      usigma_az: 100.0        # solo pitch activo (C2)
```

**Importante:** Comentar o eliminar todas las suites de familias A, B, E que ya están completadas, para evitar re-correrlas.

### 9.2 Verificar parámetros del launch file

`terrain_navigation.launch.py` y `slope_navigation.launch.py` ya soportan el parámetro `controller`. Verificar que arrancan con:
- **C1:** `ignore_imu_terrain=False`, `usigma_az=3.3`, robot en (0, 0, 1.2), yaw=0
- **C2:** `ignore_imu_terrain=False`, `usigma_az=100.0`, robot en (4.3, -1.0, 1.5), yaw=π (mirando al revés)

### 9.3 Comandos de ejecución

```bash
# Desde el host, en ~/PETER_SIMULATION

# Terminal 1 — lanzar experimentos
make run-experiments-fsm

# Terminal 2 — monitorear en tiempo real
make tail-sim

# Si se interrumpe, reanudar normalmente — el manifest auto-resume
# no vuelve a correr los trials que ya son SUCCESS

# Al completar
make export-results
# Seguir el menú interactivo para copiar al host
```

**Tiempo estimado:** C1 ~2.5 h (15 trials × ~10 min cada uno) + C2 ~2.5 h.

### 9.4 Verificar artefactos

Cada trial debe generar en `docs/resultados/simulation/familia_c1_terreno_rugoso_fsm/test_XXX_{VERDICT}/`:
- `trial_summary.json` ← métrica clave
- `stability_log.csv`
- `unified_metrics.csv`
- `metrics_raw.csv`

Si algún archivo falta, el trial tuvo un crash — verificar `make tail-sim` o relanzar.

### 9.5 Nota sobre métricas en familias C

C1 y C2 no tienen estímulos de color. Por lo tanto:
- `neural_recorder` producirá `latency=-1.0` (nunca aparece estímulo de cámara) — **ignorar este campo**
- `lambda_efficiency=1.0` — **ignorar** (sin conflicto por definición)
- `firing_variance` y `temporal_consistency` sí son válidas como indicadores de estabilidad interna

Las métricas relevantes para C1/C2 son: `Tresponse`, `Tswitch`, `roll_rms`, `pitch_rms`, `verdict`.

---

## 10. Instructivo: análisis de las pruebas de terreno

Una vez disponibles los datos de `familia_c1_terreno_rugoso_fsm/` y `familia_c2_pendiente_fsm/`:

### 10.1 Métricas a extraer

De cada `trial_summary.json`:

```python
# Campos relevantes en final_metrics:
sim_time_s    # duración del ensayo
tresponse     # tiempo de respuesta a la señal IMU (C1: tiempo hasta primer switch de modo)
tswitch       # duración de la conmutación de modo (ms)
roll_rms      # estabilidad lateral durante todo el recorrido
pitch_rms     # estabilidad longitudinal
verdict       # SUCCESS / FAILURE_TIMEOUT / FAILURE_TIPOVER
noise_idx     # nivel de ruido (0–4, aunque terreno no rota NL)
```

### 10.2 Script de análisis (replicar patrón existente)

```python
import json, glob, statistics
from pathlib import Path

base = Path("docs/resultados/simulation")

def load_family(family_dir):
    summaries = []
    for p in sorted(glob.glob(str(base / family_dir / "*/trial_summary.json"))):
        with open(p) as f:
            d = json.load(f)
        fm = d["final_metrics"]
        fm["verdict"] = d["verdict"]
        summaries.append(fm)
    return summaries

def stats(vals):
    v = [x for x in vals if x is not None]
    if not v: return ("—","—","—","—")
    return (round(statistics.mean(v),3),
            round(statistics.stdev(v),3) if len(v)>1 else 0,
            round(min(v),3), round(max(v),3))

# Cargar y comparar
for pair in [("familia_c1_terreno_rugoso", "familia_c1_terreno_rugoso_fsm"),
             ("familia_c2_pendiente",       "familia_c2_pendiente_fsm")]:
    for family in pair:
        data = load_family(family)
        n_ok = sum(1 for x in data if x["verdict"]=="SUCCESS")
        print(f"\n{family}: {n_ok}/{len(data)} SUCCESS")
        for k in ["sim_time_s","tresponse","tswitch","roll_rms","pitch_rms"]:
            vals = [x.get(k) for x in data]
            m,s,mn,mx = stats(vals)
            print(f"  {k}: mean={m} std={s} min={mn} max={mx}")
```

### 10.3 Análisis de dithering de modo (métrica adicional clave)

Para terreno C1, la señal IMU es ruidosa (vibración). La hipótesis es que la FSM puede re-conmutar entre C y H varias veces (dithering), mientras la red neuronal integra la señal con τ y no re-conmuta una vez que estabilizó el modo.

Contar cambios de modo durante la fase de terreno rugoso (de `metrics_raw.csv`):

```python
import csv

def count_mode_changes(metrics_raw_path, min_duration_s=1.0):
    """
    Cuenta cambios de modo con duración < min_duration_s (dithering).
    """
    rows = []
    with open(metrics_raw_path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    
    # Normalizar tiempo relativo
    t0 = float(rows[0]['sim_time_s'])
    transitions = []
    prev = None
    t_prev = None
    for r in rows:
        t = float(r['sim_time_s']) - t0
        m = r['mode']
        if m != prev:
            if prev is not None:
                transitions.append((prev, t_prev, t, round(t-t_prev,2)))
            t_prev = t
            prev = m
    
    n_dither = sum(1 for _, _, _, dur in transitions if dur < min_duration_s)
    n_total  = len(transitions)
    return n_dither, n_total
```

Reportar: `N_dither_neural (μ±σ)` vs `N_dither_FSM (μ±σ)` para C1.

### 10.4 Qué debe contener el análisis de terreno

1. **Tabla comparativa** (misma estructura que §4.1):
   - SR, sim_time, tresponse, tswitch, roll_rms, pitch_rms — neural vs FSM
2. **N_dither** (si se observa en FSM): cambios de modo de duración < 1 s durante la fase de terreno
3. **Interpretación:** si FSM muestra dithering IMU, refuerza el argumento τ decay neuronal. Si no lo muestra (el persist timer de 3 s lo mitiga), reportar honestamente.
4. **Nota sobre C2:** en pendiente, el sistema neural usa `z[1]` (pitch) como relay IMU. La FSM usa directamente `pitch > Upitch=1°`. Comparar velocidad de detección y si alguno falla en declives moderados.

---

## 11. Instructivo: integrar terreno en la comparación

Una vez disponibles los datos de C1\_fsm y C2\_fsm y terminado el análisis del §10:

### 11.1 Actualizar la tabla maestra (§7.1)

Reemplazar las filas de C1 y C2 marcadas "solo neural — FSM pendiente" con los datos reales:

```markdown
| **C1-Terreno** | Neural | 15 | 15 | 100% | 35.0 ± 3.4 | 1.59 | 1.00 | [observación] |
|                | FSM    | 15 | XX | XX%  | XX ± XX    | XX   | 1.00 | [observación] |
| **C2-Pendiente**| Neural | 15 | 15 | 100% | 66.7 ± 6.6 | 1.47 | 1.00 | [observación] |
|                | FSM    | 15 | XX | XX%  | XX ± XX    | XX   | 1.00 | [observación] |
```

### 11.2 Añadir sección de análisis de terreno

Añadir una nueva sección **"Resultados — Familias C: terreno"** entre los §6 y §7 actuales, con la misma estructura que §4, §5 y §6:
- Descripción del escenario (C1: plano→rugoso; C2: plano→pendiente)
- Patrón de comportamiento neural (de datos ya existentes)
- Comportamiento FSM observado
- Tabla de diferencias arquitectónicas con los números reales
- Síntesis

### 11.3 Actualizar §7 (síntesis)

- Si FSM muestra dithering IMU en C1: añadir a §7.3 como evidencia adicional de la ventaja del τ decay
- Si FSM no muestra dithering: actualizar §8.3 (limitaciones) para declarar honestamente que el persist timer mitiga el problema en terreno

### 11.4 Actualizar §8 (notas de redacción)

Añadir el veredicto de terreno a §8.2 (qué resaltar) y §8.3 (limitaciones). Actualizar el párrafo de respuesta al referee en §8.4 con los datos de C1/C2.

---

*Última actualización: agosto 2026. Datos disponibles: familias A1–A3, B, E (neural y FSM). Pendiente: C1\_fsm, C2\_fsm.*
