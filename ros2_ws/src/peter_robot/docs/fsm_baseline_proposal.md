# Propuesta: FSM Gait Arbitration Baseline — Respuesta al Referee

**Comentario del referee atendido:** *"Add finite state machine (FSM) gait arbitration baseline comparisons under identical experimental conditions."*  
**Rama de implementación:** `fsm` (creada desde `sam`)  
**Estado del documento:** borrador de trabajo — zona de análisis pendiente de resultados  

---

## 1. Qué pide el referee y por qué importa

El comentario pide demostrar que la selección de marcha del sistema neuronal aporta valor medible frente a una alternativa clásica (FSM). El argumento que necesita el paper no es "el FSM falla y el neural funciona", sino:

> *El módulo de ganglios basales (BG) proporciona estabilidad temporal en la conmutación de marcha que un FSM de umbral fijo no puede replicar sin comprometer la velocidad de respuesta o la robustez ante ruido.*

Dicho de otro modo: el FSM puede reproducir la **selección correcta de modo** bajo condiciones ideales, pero lo hace a costa de dithering (oscilaciones de modo indeseadas) bajo condiciones de estímulo ruidoso o en transición. La diferencia cuantitativa se mide con las métricas descritas en la Sección 4.

---

## 2. Estrategia de rama

```
main
└── sam          ← red neuronal original (no se toca)
    └── fsm      ← esta rama, solo añade archivos nuevos
```

**Archivos a crear en la rama `fsm`:**

| Archivo | Propósito |
|---|---|
| `src/peter_robot/src/fsm_gait_arbitration.py` | Nodo ROS 2 — árbitro FSM de marcha |
| `src/peter_robot/config/experiments_config_fsm.yaml` | Suite de experimentos para el FSM |

**Archivos a modificar:**

| Archivo | Cambio |
|---|---|
| `src/peter_robot/setup.py` | Registrar `fsm_arbitration = src.fsm_gait_arbitration:main` |
| `src/peter_robot/launch/gazebo.launch.py` | Añadir arg `controller:=neural\|fsm` para elegir nodo |
| `src/peter_robot/launch/single_stimulus.launch.py` | Ídem |
| `src/peter_robot/launch/multiple_stimuli.launch.py` | Ídem |

**`red_neuronal.py` no se modifica.** La comparación es entre dos nodos independientes que publican en los mismos topics bajo la misma infraestructura de simulación y medición.

---

## 3. Diseño del FSM (`fsm_gait_arbitration.py`)

### 3.1 Principio de equivalencia

El FSM hereda todo el procesamiento sensorial y de navegación de `red_neuronal.py`:

- Callbacks de cámara (rojo, azul), LiDAR, IMU → **idénticos**
- Dinámica de neuronas LiDAR (`self.lidar`) → **idéntica**  
- Dinámica de neuronas de movimiento (`self.z[5]`–`self.z[13]`) → **idéntica**
- Ruido gaussiano inyectado (`_sigma_noise`) → **idéntico**
- Publicación de métricas (`/Metrics`, `/neuron_activity`) → **idéntica**

**Lo único que cambia:** las neuronas `z[14]`, `z[15]`, `z[16]` y la lógica de selección de modo con histéresis se reemplazan por un FSM determinista de tres estados.

### 3.2 Diagrama de estados FSM

```
                    ┌─────────────────────────────────────────┐
                    │           MODO ACTUAL                   │
         ┌──────────┤                                         ├────────────┐
         │          │  C (Cuadrúpedo)                         │            │
         │          │  H (Híbrido Móvil)                      │            │
         │          │  X (Omnidireccional)                    │            │
         │          └─────────────────────────────────────────┘            │
         │                                                                  │
         ▼                                                                  ▼
   EVALUACIÓN DE PRIORIDADES (cada ciclo de control, 0.15 s)

   1. Predador activo?    Gpe[0] > θ_H = 1.5  →  H  (máxima prioridad)
   2. Terreno activo?     pitch  > θ_P = 1.0
                       OR z[0]  > θ_Z = 0.3   →  H
   3. Obstáculo activo?   Gpe[1] > θ_X = 0.5  →  X
   4. Presa activa?       Gpe[2] > θ_C = 1.5  →  C
   5. Por defecto                               →  C
```

**Histéresis temporal** (equivalente a `min_dwell_time` neural): el modo no cambia a menos que el nuevo modo sea diferente al actual **y** hayan transcurrido ≥ 3 s desde el último cambio, o la condición H (huida/terreno) esté activa — H siempre se ejecuta sin espera de dwell.

### 3.3 Diferencia conceptual respecto al sistema neural

| Aspecto | Red Neuronal (BG) | FSM |
|---|---|---|
| Mecanismo | Inhibición lateral competitiva + integración temporal | Jerarquía de prioridades estática |
| Respuesta a estímulo nuevo | Integración gradual (τ) antes de cambiar modo | Inmediata al superar umbral |
| Estabilidad bajo ruido | Filtrado por dinámica (amortiguación natural) | Oscilación si señal ≈ umbral |
| Resolución de conflicto (R+B) | WTA con inhibición cruzada → ganador claro | Prioridad fija → siempre H sobre C |
| Cambios de modo por ciclo | ~N_lidar_events / 2.3 (factor de suavizado medido) | ~N_lidar_events (sin suavizado) |
| Parámetros a ajustar | Pesos sinápticos (w, τ, A, σ) | Umbrales θ |

---

## 4. Métricas de comparación

Las métricas están organizadas por lo que revelan sobre el arbitraje, no por su origen técnico. `firing_variance` y `temporal_consistency` del `neural_recorder` **no se usan como métricas comparativas**: son propiedades internas de la dinámica neuronal que el FSM no posee — incluirlas generaría una comparación vacía (siempre 0.0 para el FSM).

### 4.1 Métrica central: N_dither (oscilación de modo)

**Definición:** número de transiciones de modo cuya duración es inferior a 1 s. Cada dither es un cambio de modo que el robot ejecuta mecánicamente pero que el árbitro revierte antes de que tenga efecto locomotor útil.

**Por qué es la métrica más importante:** el neural evita el dithering por diseño — la integración temporal (τ) impide que fluctuaciones breves de estímulo crucen el umbral efectivo. El FSM reacciona al instante a cada cruce de umbral, lo que bajo señal ruidosa o en transición genera oscilaciones que el neural suprime. Esta diferencia es el *argumento central* del paper frente al referee.

**Fuente:** timestamps del topic `/peter_mode`, conteo de transiciones con duración < 1 s entre cambios consecutivos.

| Sistema | Comportamiento esperado |
|---|---|
| Neural | N_dither ≈ 0 (integración temporal absorbe fluctuaciones) |
| FSM | N_dither > 0, aumenta con nivel de ruido nl |

### 4.2 T_switch — velocidad de conmutación (trade-off honesto)

**Definición:** tiempo desde el onset del estímulo (primer frame de bounding box detectable, o primer cruce IMU) hasta la primera publicación del modo correcto en `/peter_mode`.

**Por qué es un trade-off:** el FSM probablemente tendrá T_switch **menor** que el neural en familias A y C — responde en el primer ciclo que supera el umbral, mientras el neural necesita integración. Esto debe reportarse honestamente: el neural sacrifica velocidad inicial para ganar estabilidad. La ganancia en N_dither justifica esa pérdida en T_switch.

**Fuente:** timestamps `/peter_mode` vs. onset de estímulo en `/bounding_box/*` o pico IMU en `/imu/data`.

### 4.3 Métricas por familia

Cada familia prueba un aspecto diferente del arbitraje. Las métricas se seleccionan según el comportamiento que se observa:

#### Familia A — selección básica de modo

| Métrica | Símbolo | Fuente | Qué revela |
|---|---|---|---|
| Modo correcto al estabilizarse | Acc_mode | `/peter_mode` (modo final) vs. estímulo | ¿Selecciona el modo adecuado? Bajo nl=0 ambos deberían coincidir; bajo nl alto el FSM puede errar |
| Oscilación de modo | N_dither | `/peter_mode` timestamps | Inestabilidad del árbitro durante la respuesta |
| Velocidad de conmutación | T_switch | `/peter_mode` vs. onset bounding box | Trade-off velocidad/estabilidad |
| Tasa de éxito | SR | `trial_summary.json` | Resultado global de la tarea |

#### Familia B — resolución de conflicto (R + A simultáneos)

| Métrica | Símbolo | Fuente | Qué revela |
|---|---|---|---|
| Eficiencia de conflicto | λ | `neural_recorder` → `lambda_efficiency` | **La métrica más importante de esta familia.** Calidad de la decisión bajo estímulos en pugna. FSM con prioridad fija: λ bajo. Neural con WTA: λ alto |
| Oscilaciones durante conflicto | N_osc | Cambios de modo mientras ambos estímulos activos | El FSM oscila entre H y C si las señales fluctúan cerca del umbral |
| Modo ganador | — | `/peter_mode` (moda durante conflicto) | El FSM siempre elige H; el neural puede elegir H o C según intensidad relativa |
| Tasa de éxito | SR | `trial_summary.json` | ¿El FSM completa la tarea pese al conflicto? |

#### Familia C — conmutación por terreno (IMU)

| Métrica | Símbolo | Fuente | Qué revela |
|---|---|---|---|
| Tiempo de respuesta a terreno | T_response | `/Metrics`[0] | FSM esperado más rápido; neural más estable. Reportar ambos |
| Estabilidad post-switch | N_dither | `/peter_mode` tras primer switch | Terreno irregular da señal IMU ruidosa → FSM re-conmuta, neural no |
| Estabilidad locomotora | roll_rms, pitch_rms | `/Metrics`[2,3] | ¿La conmutación tardía o inestable afecta el balance? |
| Tasa de éxito | SR | `trial_summary.json` | ¿Llega a completar el recorrido? |

#### Familia E — inspección reactiva

| Métrica | Símbolo | Fuente | Qué revela |
|---|---|---|---|
| Tasa de éxito | SR | `trial_summary.json` | Resultado global |
| Precisión de parada | d_final | `inspection_summary.json` | ¿El FSM para en el mismo punto que el neural? |
| Ratio de suavizado | ρ = N_lidar / N_mode_X | `inspection_summary.json` | Cuántas detecciones sensoriales producen un cambio de modo. Neural: ρ≈2.3 |
| Oscilación de modo | N_dither | `/peter_mode` timestamps | Consistencia con las otras familias |
| Tiempo de tarea | T_task | `trial_summary.json` | Eficiencia global |

### 4.4 Métrica de robustez bajo ruido (transversal a todas las familias)

El orquestador rota `noise_level_idx` en {0,1,2,3,4} (0%,5%,10%,20%,30% de ruido). Para cada familia se grafica:

**SR vs. nl** y **N_dither vs. nl** para Neural y FSM.

Hipótesis: con nl=0 las tasas de éxito serán comparables. A partir de nl=2 (10%) el FSM mostrará degradación más rápida en SR y aumento de N_dither, mientras el neural mantiene ambos estables. Esta gráfica es la demostración de robustez más directa posible.

---

## 5. Plan de pruebas

### 5.1 Justificación del número de repeticiones

El paper neural usa ≈15–20 trials por familia. Para el FSM baseline, **10 trials por sub-escenario** son estadísticamente suficientes para reportar media ± σ (error estándar < 20 % de la media para las métricas principales, que presentan baja varianza en el neural). El objetivo es una comparación científicamente honesta, no una replicación completa.

La selección de familias es **todas**, porque en todas hay cambio de modo — que es exactamente lo que se compara.

### 5.2 Tabla de pruebas FSM

| Familia | Sub-escenario | World | Trials FSM | Config YAML | Tiempo estimado |
|---|---|---|---|---|---|
| **A — estímulo único** | A1: Rojo (huida H) | `single_stimulus` | 10 | `stimulus_type:=red` | ~40 min |
| | A2: Azul (caza C) | `single_stimulus` | 10 | `stimulus_type:=blue` | ~40 min |
| | A3: Verde (evasión X) | `single_stimulus` | 10 | `stimulus_type:=green` | ~40 min |
| **B — conflicto** | B1: Rojo + Azul | `multiple_stimuli` | 10 | `spawn_red:=true spawn_blue:=true` | ~50 min |
| | B4: R + A + Verde | `multiple_stimuli` | 10 | `spawn_red:=true spawn_blue:=true spawn_green:=true` | ~50 min |
| **C — terreno** | C1: Irregular | `terrain` | 10 | `launch: terrain_navigation.launch.py` | ~50 min |
| | C2: Pendiente | `terrain` | 10 | `launch: slope_navigation.launch.py` | ~50 min |
| **E — inspección** | E: Pista obstáculos | `obstaculos` | 10 | `launch: gazebo.launch.py` | ~20 min |

**Total FSM: 80 trials** — estimado ~6 horas de simulación en total, distribuibles en sesiones.

### 5.3 Condiciones idénticas garantizadas

- Mismos mundos Gazebo
- Mismo orquestador (`test_manager.py`)
- Mismo nodo de medición (`neural_recorder`, `inspection_recorder`, `peter_stability_monitor`)
- Mismo nivel de ruido: `noise_level_idx` rotando 0–4 igual que en neural
- Mismos criterios de éxito/fallo (`Verdict.SUCCESS`, tipover, timeout)
- Mismo warmup (15 s)

La única diferencia: el arg `controller:=fsm` en el launch file arranca `fsm_gait_arbitration` en lugar de `red_neuronal`.

### 5.4 Nomenclatura de resultados

```
docs/resultados/
├── familia_a_apetitivo/          ← Neural (ya existe en paper)
├── familia_a_apetitivo_fsm/      ← FSM (rama fsm)
├── familia_b_compleja/
├── familia_b_compleja_fsm/
├── familia_c1_terreno_rugoso/
├── familia_c1_terreno_rugoso_fsm/
├── familia_c2_pendiente/
├── familia_c2_pendiente_fsm/
├── familia_e_inspeccion/         ← Neural (20 trials, ya existe)
└── familia_e_inspeccion_fsm/     ← FSM (10 trials)
```

Los resultados neurales originales se importan como `.zip` cuando se necesiten para el análisis comparativo.

---

## 6. Orden de implementación

```
1. git checkout sam && git checkout -b fsm

2. Crear fsm_gait_arbitration.py
   — Copiar red_neuronal.py como base
   — Eliminar z[14], z[15], z[16] y su dinámica
   — Implementar FSM (Sección 3.2) usando Gpe[] directamente
   — Mantener todo lo demás idéntico

3. Registrar en setup.py:
   'fsm_arbitration = src.fsm_gait_arbitration:main'

4. Añadir arg controller:= a los launch files

5. Crear experiments_config_fsm.yaml
   — Mismas suites que neural, suite_name con sufijo _fsm
   — repetitions: 10

6. make build (dentro del contenedor)

7. Correr pruebas por familia con make run-experiments
   pasando el YAML del FSM

8. Exportar resultados → importar ZIP neurales → análisis comparativo
```

---

## 7. Comparaciones que debe mostrar el paper

Las tablas y figuras siguientes son la respuesta directa al referee. Se construyen una vez que existan los resultados FSM. El orden refleja la jerarquía argumentativa: dithering primero, conflicto segundo, trade-off de velocidad tercero, robustez cuarta.

### Tabla C1 — Dithering y tasa de éxito (todas las familias)

La tabla central del paper. Demuestra el trade-off fundamental: el FSM puede ser más rápido (T_switch), pero genera oscilaciones que el neural evita.

| Familia | SR_Neural (%) | SR_FSM (%) | N_dither_Neural (μ±σ) | N_dither_FSM (μ±σ) | T_switch_Neural (s) | T_switch_FSM (s) |
|---|---|---|---|---|---|---|
| A1 (rojo → H) | — | — | — | — | — | — |
| A2 (azul → C) | — | — | — | — | — | — |
| A3 (verde → X) | — | — | — | — | — | — |
| B1 (R+A) | — | — | — | — | — | — |
| B4 (R+A+V) | — | — | — | — | — | — |
| C1 (terreno rugoso) | — | — | — | — | — | — |
| C2 (pendiente) | — | — | — | — | — | — |
| E (inspección) | 85 % (17/20) | — | — | — | — | — |

*Hipótesis: N_dither_Neural ≈ 0 en todas las familias. N_dither_FSM > 0 y creciente con la complejidad del escenario. T_switch_FSM < T_switch_Neural en A y C (FSM más veloz); relación inversa en B (conflicto ralentiza la prioridad fija).*

### Tabla C2 — Selección correcta de modo bajo ruido (Familia A)

Muestra si ambos sistemas eligen el modo adecuado para cada estímulo, y cómo se degrada esa precisión con el nivel de ruido.

| Estímulo | Acc_mode_Neural nl=0 | Acc_mode_FSM nl=0 | Acc_mode_Neural nl=2 | Acc_mode_FSM nl=2 | Acc_mode_Neural nl=4 | Acc_mode_FSM nl=4 |
|---|---|---|---|---|---|---|
| Rojo → H | — | — | — | — | — | — |
| Azul → C | — | — | — | — | — | — |
| Verde → X | — | — | — | — | — | — |

*Acc_mode = fracción de trials donde el modo final estabilizado coincide con el modo esperado para el estímulo. Hipótesis: ambos sistemas similares a nl=0; FSM se degrada antes con nl creciente.*

### Tabla C3 — Resolución de conflicto bajo estímulos simultáneos (Familia B)

| Condición | λ_Neural (μ±σ) | λ_FSM (μ±σ) | N_osc_Neural (μ±σ) | N_osc_FSM (μ±σ) |
|---|---|---|---|---|
| B1: Rojo + Azul | — | — | — | — |
| B4: R + A + Verde | — | — | — | — |

*λ = eficiencia de resolución de conflicto (neural_recorder). N_osc = cambios de modo mientras ambos estímulos están activos simultáneamente. Hipótesis: λ_Neural > λ_FSM; N_osc_FSM > N_osc_Neural.*

### Tabla C4 — Respuesta a terreno y estabilidad locomotora (Familia C)

| Condición | T_response_Neural (s) | T_response_FSM (s) | N_dither_Neural | N_dither_FSM | roll_rms_Neural | roll_rms_FSM |
|---|---|---|---|---|---|---|
| C1: terreno rugoso | — | — | — | — | — | — |
| C2: pendiente | — | — | — | — | — | — |

*T_response_FSM probablemente menor (reacción inmediata al umbral IMU). N_dither_FSM probablemente mayor (señal IMU rugosa genera cruces espurios). roll_rms captura si el dithering de modo se traduce en inestabilidad locomotora real.*

### Figura F1 — Dithering temporal (trial representativo por familia)

Una subfigura por familia (A, B, C, E). Cada subfigura: eje X = tiempo simulado (s), eje Y = modo activo (C / H / X) como función escalonada. Neural en azul, FSM en naranja.

Se espera que el FSM muestre rectángulos estrechos (modos de corta duración) que el neural no tiene. Esta figura es el argumento visual más directo para el referee: la diferencia entre los dos sistemas se ve a primera vista.

### Figura F2 — Robustez bajo ruido (SR y N_dither vs. nl)

Dos gráficas de línea, una para SR y otra para N_dither. Eje X = nivel de ruido nl ∈ {0,1,2,3,4}. Una línea por sistema (Neural, FSM). Aplica a la familia más representativa del paper (A o C).

Demuestra que la degradación del FSM bajo ruido es más rápida que la del neural, y que el N_dither del FSM crece con nl mientras el del neural permanece estable.

---

## 8. Argumentación para el paper

> *"To validate the contribution of the basal ganglia (BG) temporal integration mechanism, we implemented an equivalent FSM controller sharing the same sensory pipeline, movement generation, and dwell timer (3 s) as the neural system. The only substitution is the competitive inhibition dynamics (z[14]–z[16]) with a static priority hierarchy driven by the same Gpe[] signals.*
>
> *Under ideal conditions (nl=0, single stimulus, Family A), both systems select the correct locomotion mode at comparable rates [Table C2], confirming that the FSM's priority encoding is behaviorally correct. However, the FSM exhibits systematically higher mode-oscillation counts (N_dither) across all families [Table C1], a consequence of reacting instantaneously to every threshold crossing without temporal integration. The neural BG module suppresses these oscillations through its intrinsic dynamics, yielding N_dither ≈ 0 regardless of stimulus noise level.*
>
> *Under conflicting stimuli (Family B), the FSM's fixed priority order produces lower conflict-resolution efficiency (λ) than the neural WTA competition [Table C3], which dynamically resolves ambiguity based on relative stimulus intensity rather than a fixed hierarchy.*
>
> *Under sensory noise (nl > 0), the FSM degrades faster in both success rate and mode stability [Figure F2], while the neural system's temporal integration acts as a low-pass filter on stimulus fluctuations. This robustness, not raw switching speed, is the primary functional advantage of bio-inspired gait arbitration."*

---

## 9. Zona de análisis y resultados

> **Esta sección se completa cuando los resultados FSM estén disponibles (ZIP neural + ZIP FSM exportados del contenedor). Los datos neurales de Familia A, B, C se importan desde el ZIP del paper original. Los de Familia E están en `docs/resultados/familia_e_inspeccion/`.**

---

### 9.1 Tabla C1 — Dithering y tasa de éxito (todas las familias)

| Familia | SR_Neural (%) | SR_FSM (%) | N_dither_Neural (μ±σ) | N_dither_FSM (μ±σ) | T_switch_Neural (s) | T_switch_FSM (s) |
|---|---|---|---|---|---|---|
| A1 (rojo → H) | — | — | — | — | — | — |
| A2 (azul → C) | — | — | — | — | — | — |
| A3 (verde → X) | — | — | — | — | — | — |
| B1 (R+A) | — | — | — | — | — | — |
| B4 (R+A+V) | — | — | — | — | — | — |
| C1 (terreno rugoso) | — | — | — | — | — | — |
| C2 (pendiente) | — | — | — | — | — | — |
| E (inspección) | 85 % (17/20) | — | — | — | — | — |

---

### 9.2 Tabla C2 — Selección correcta de modo bajo ruido (Familia A)

| Estímulo | Acc_Neural nl=0 | Acc_FSM nl=0 | Acc_Neural nl=2 | Acc_FSM nl=2 | Acc_Neural nl=4 | Acc_FSM nl=4 |
|---|---|---|---|---|---|---|
| Rojo → H | — | — | — | — | — | — |
| Azul → C | — | — | — | — | — | — |
| Verde → X | — | — | — | — | — | — |

---

### 9.3 Tabla C3 — Resolución de conflicto (Familia B)

| Condición | λ_Neural (μ±σ) | λ_FSM (μ±σ) | N_osc_Neural (μ±σ) | N_osc_FSM (μ±σ) |
|---|---|---|---|---|
| B1: Rojo + Azul | — | — | — | — |
| B4: R + A + Verde | — | — | — | — |

---

### 9.4 Tabla C4 — Respuesta a terreno y estabilidad locomotora (Familia C)

| Condición | T_response_Neural (s) | T_response_FSM (s) | N_dither_Neural | N_dither_FSM | roll_rms_Neural | roll_rms_FSM |
|---|---|---|---|---|---|---|
| C1: terreno rugoso | — | — | — | — | — | — |
| C2: pendiente | — | — | — | — | — | — |

---

### 9.5 Resultados Familia E (comparación FSM vs. Neural)

**Neural — disponible:**

| Métrica | μ | σ | N |
|---|---|---|---|
| SR | 85 % | — | 20 |
| d_final (m) | 1.076 | 0.038 | 17 |
| N_lidar_events | 7.71 | 6.42 | 17 |
| N_mode_X_events | 3.29 | 2.80 | 17 |
| ρ (N_lidar / N_mode_X) | 2.35 | — | 17 |
| T_task (s) | 51.6 | 12.3 | 17 |
| N_dither | — | — | 17 |

**FSM — pendiente:**

| Métrica | μ | σ | N |
|---|---|---|---|
| SR | — | — | 10 |
| d_final (m) | — | — | — |
| N_lidar_events | — | — | — |
| N_mode_X_events | — | — | — |
| ρ (N_lidar / N_mode_X) | — | — | — |
| T_task (s) | — | — | — |
| N_dither | — | — | — |

---

### 9.6 Figura F1 — Dithering temporal (trial representativo por familia)

*(pendiente — generar con datos de ambos sistemas)*

---

### 9.7 Figura F2 — SR y N_dither vs. nivel de ruido nl

*(pendiente — generar con datos de ambos sistemas)*

---

### 9.8 Interpretación y conclusiones

*(pendiente)*

---

## 10. Notas para el equipo de redacción

- Los resultados neurales de todas las familias (A, B, C) están en el paper original pero no en este repo. Se importan como ZIP cuando sea necesario para comparación (ver `make export-results`).
- Los resultados de Familia E neural sí están en `docs/resultados/familia_e_inspeccion/` (20 trials, 17 exitosos).
- Los resultados FSM se generan en la rama `fsm` y se guardan en `docs/resultados/familia_*_fsm/`.
- El `.zip` del neural y del FSM deben exportarse juntos para el análisis final en la Sección 9.
- La rama `sam` (con `red_neuronal.py`) no se modifica en ningún momento de este proceso.
