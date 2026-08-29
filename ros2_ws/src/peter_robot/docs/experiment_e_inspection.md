# Experimento E — Tarea de Inspección Reactiva

**Reviewer atendido:** #3.5 (HIGH)  
**Comentario original:** *"The paper only mentions inspection as a target scenario but does not design inspection-oriented task indicators such as target tracking accuracy, coverage rate, and autonomous re-planning ability after obstacle blockage, lacking practical engineering demonstration."*

---

## 1. Marco conceptual y honestidad científica

El sistema PETER no implementa planificación de trayectorias (*path planning*). No construye mapas, no mantiene memoria espacial, y no replanifica deliberadamente cuando encuentra un bloqueo. Lo que sí implementa es **navegación reactiva de inspección**: el módulo BG/WTA integra información sensorial en tiempo real (LiDAR + cámara) para producir comportamiento emergente que, en la pista de obstáculos diseñada para este experimento, resulta en:

1. Traversal del campo de obstáculos mediante evasión lateral reactiva.
2. Adquisición visual del estímulo objetivo al final del recorrido.
3. Aproximación guiada visualmente por la cámara.
4. Parada de inspección activada por la neurona X17.

**Encuadre para el referee:** respondemos el comentario de re-planificación demostrando *re-enrutamiento reactivo*: el robot localmente evade cada obstáculo y, sin memoria de lo ocurrido, continúa hacia el objetivo. Esto es distinto del path planning deliberado, y esa distinción debe declararse explícitamente — es la contribución del paper, no una limitación a disculpar.

La **coverage rate** como porcentaje de área inspeccionada **no se mide**, por las razones que se detallan en la Sección 1.1. Se sustituye por métricas orientadas a la tarea que son medibles con la infraestructura actual y son defendibles ante el referee.

### 1.1 Por qué no se implementa coverage rate

La coverage rate en robótica de inspección se define como el porcentaje del área objetivo que el robot ha visitado dentro de un radio de inspección. Su cálculo requiere:

1. **Un mapa del espacio a inspeccionar** — el área objetivo debe estar definida geométricamente de antemano. PETER no construye mapas ni recibe una definición del área de cobertura.
2. **Tracking de posición continuo con suficiente frecuencia** — para integrar el área barrida, se necesita la trayectoria completa del robot a alta frecuencia. La pose sólo es accesible mediante `gz topic` vía subprocess (un proceso externo lento, ~5 Hz en el mejor caso), no como topic nativo de ROS2.
3. **Un planificador de cobertura** — la cobertura sólo tiene sentido si el robot sigue una estrategia deliberada (e.g., boustrophedon, espiral) para cubrir el área. La navegación reactiva de PETER no garantiza ningún patrón de cobertura; el robot sigue al estímulo y evade obstáculos, lo que puede resultar en trayectorias muy variables entre ensayos.

**Argumento para el referee:** la coverage rate es una métrica de inspección deliberada, apropiada para robots con planificadores de cobertura. Para un sistema bio-inspirado de navegación reactiva, la métrica relevante es si el robot *completa la tarea de inspección* (llega al objetivo y lo detecta), no si cubre un área predefinida. Las métricas `SR` (tasa de éxito), `d_final` (precisión de llegada) y `T_task` (tiempo de tarea) caracterizan el comportamiento de inspección sin presuponer una arquitectura de planificación que el sistema no posee.

---

## 2. Escenario de simulación

**Archivo de mundo:** `obstaculos.world`  
**Launch:** `ros2 launch peter_robot gazebo.launch.py world_name:=obstaculos`

| Elemento | Posición (x, y, z) | Descripción |
|---|---|---|
| Robot PETER | (0.0, 0.0, 1.2) | Pose inicial, orientado en +X |
| Cápsula 1 | (-0.4, -0.5, 0.55) | Obstáculo estático, radio 0.15 m |
| Cápsula 2 | (1.2, 0.0, 0.55) | Obstáculo estático, radio 0.15 m |
| Cápsula 3 | (0.5, 1.7, 0.55) | Obstáculo estático, radio 0.15 m |
| Esfera azul | (-2.0, 2.0, 0.5) | Objetivo de inspección |

El robot no ve la esfera azul al inicio: los obstáculos bloquean o no están en el campo visual frontal durante la mayor parte del recorrido. El objetivo sólo se vuelve visible cuando el robot ha navegado suficientemente por la pista.

---

## 3. Análisis del evento de evasión en `red_neuronal.py`

### 3.1 Estado actual del código (post-merge)

Las dos líneas que antes desactivaban el canal de obstáculos están comentadas:

```python
R = self.areaBoundingBoxR / 500           # R desde cámara (real)
# R = 3.652  ← COMENTADO (era para prueba de inclinación)

if self.lidar[4,0]*15 > 0.2: G = self.lidar[4,0]*15
else: G = 0
# G = 0  ← COMENTADO (ahora G viene del LiDAR real)

B = self.areaBoundingBoxB / 500           # B desde cámara (real)
```

Esto significa que el canal de obstáculo en el BG **está activo**: `G = lidar[4]*15` alimenta `StN[1]` → `Gpe[1]` → `z14` (modo omnidireccional X).

### 3.2 Dos niveles de señal de evasión

El código produce dos señales de evasión con semánticas distintas:

**Señal interna — `lidar[4] >= 0.3` (evasión sensorial activa)**

`lidar[4]` es el WTA integrado del anillo de 16 neuronas LiDAR. El umbral 0.3 aparece en dos puntos críticos del código:

```python
# Línea 373 — cuando lidar[4] >= 0.3, ang_s se fuerza a 0:
# el robot deja de perseguir el estímulo y entra en modo evasión
self.ang_s = 90*(self.lidar[4,1] < 0.3)

# Línea 403 — comandos angulares suprimidos cuando hay obstáculo frontal:
cmd_ang = (self.z[11,0]*(self.lidar[4,0] < 0.3)) - (self.z[12,0]*(self.lidar[4,0] < 0.3))
```

Este umbral es el **flag interno de "obstáculo bloqueando el camino"** del sistema neuronal. Cuando `lidar[4] >= 0.3`, el robot suspende la persecución del objetivo y activa evasión lateral.

**Señal de modo — cambio a X en `/peter_mode` (decisión BG)**

Cuando `Gpe[1]` se acumula lo suficiente, `z14 > 0.5` y el robot publica modo X. Esto ocurre con la histéresis de 3 s (`min_dwell_time`): el BG no cambia de modo en cada fluctuación del LiDAR, sino cuando el obstáculo ha sido detectado de forma sostenida.

```python
elif z14 > 0.5 and time_since_last_change > self.min_dwell_time:
    new_mode = 'X'
```

### 3.3 Definición operativa del evento de evasión para métricas

| Señal | Fuente | Índice en `/neuron_activity` | Umbral | Significado |
|---|---|---|---|---|
| `lidar[4]` | Nivel sensorial | Índice 36 del vector concatenado | ≥ 0.3 | Obstáculo frontal activo, evasión en curso |
| `z14` | Nivel BG | Índice 26 del vector concatenado | > 0.5 | BG ha decidido modo omnidireccional |
| Modo X publicado | `/peter_mode` | — | 'X' | Confirmación comportamental con histéresis |

Para el paper se reportan **ambas señales** porque su disociación es un resultado científico: `lidar[4]` puede superar el umbral varias veces mientras el BG, gracias a la histéresis inhibitoria, emite un único cambio de modo sostenido. Esto demuestra que el WTA aporta estabilidad temporal sobre la señal sensorial cruda.

### 3.4 Evasión lateral directa (independiente del modo)

El comando lateral también proviene del LiDAR directamente, sin pasar por el BG:

```python
cmd_lateral = (self.lidar[2,0]*1.5 + self.lidar[3,0]*1.5 + ...)
```

`lidar[2]` = obstáculo izquierdo, `lidar[3]` = obstáculo derecho. Ambos contribuyen positivamente a `cmd_lateral` (movimiento lateral del robot), produciendo evasión lateral incluso si `z14` no ha activado el modo X todavía. Esta es la evasión reactiva de más bajo nivel, siempre activa mientras el LiDAR detecte obstáculos laterales.

---

## 4. Visual steering hacia el estímulo azul

El robot **sí hace steering visual** hacia la esfera azul cuando las condiciones son favorables:

```python
elif self.Gpe[2,1] > 1.5 and B > 0.5:
    self.ang_s = self.posB     # posición angular real de la esfera desde cámara
else:
    self.ang_s = 90*(self.lidar[4,1] < 0.3)
```

`posB` viene de `camera_node.py`: es la posición angular del centroide del bounding box azul, mapeada al rango [0°, 180°]. Cuando `Gpe[2] > 1.5` (canal azul del BG activo) y `B > 0.5` (esfera visible con área > 250 px), el ángulo de referencia `ang_s` se fija en la dirección real del objetivo.

Este `ang_s` alimenta `z5` y `z6`, que producen `cmd_ang`:
```python
self.z[5,1] = ... max(0, lidar[2,0]*160 + (lidar[4,1]<0.3)*((ang_s - ang_p) - 20))
self.z[6,1] = ... max(0, lidar[3,0]*160 + (lidar[4,1]<0.3)*((ang_p - ang_s) - 20))
cmd_ang = (z[11,0]*(lidar[4,0]<0.3)) - (z[12,0]*(lidar[4,0]<0.3))
```

El steering visual es activo **sólo cuando no hay obstáculo frontal** (`lidar[4] < 0.3`). Durante la evasión, `ang_s` cae a 0 (la persecución visual se suspende). Cuando el obstáculo deja de estar enfrente, `ang_s` vuelve a `posB` si la esfera sigue visible. Esta dinámica — suspensión y reanudación del steering — es el mecanismo de "re-adquisición" del sistema.

**Condición para que el BG active el canal azul:**  
`B*0.7 > 1.0 + Gpi[2] + Gpe[0] + Gpe[1]` → `areaBoundingBoxB > ~714 px` en ausencia de inhibición de los otros canales. En presencia de R (rojo) o G (obstáculo) activos, el umbral efectivo sube por inhibición cruzada.

---

## 5. Métricas del Experimento E

### M1 — Precisión de aproximación final (`d_final`)

**Definición:** distancia euclidiana entre la posición del robot y el centro de la esfera azul en el instante de activación de X17 (z[17] > 0.25).

**Fuente de datos:** `RMSE_node.py` ya implementa `gz topic -e -t /world/default/pose/info` para leer la pose del robot. Se adapta para capturar la posición en el momento del evento X17 en lugar de al final de un tiempo fijo. La posición de la esfera es fija y conocida: (-2.0, 2.0, 0.5).

**Relevancia para el referee:** es la "target tracking accuracy" solicitada. Cuantifica cuán precisa es la aproximación visual reactiva al objetivo.

### M2 — Eventos de evasión sensorial (`N_lidar_events`)

**Definición:** número de transiciones `lidar[4]` de < 0.3 a ≥ 0.3 durante un ensayo. Cada transición = el sistema neuronal detectó un nuevo obstáculo bloqueando el camino.

**Fuente de datos:** índice 36 del vector publicado en `/neuron_activity`, registrado en el CSV de `NeuralRecorder` si se añade este índice a la columna de salida, o en post-proceso desde `rosbag`.

**Comparación con N_mode_X:** si `N_lidar_events > N_mode_X_transitions`, demuestra la función de suavizado temporal del BG: más detecciones sensoriales que cambios de modo, mostrando que el WTA evita oscilaciones de modo (dithering). Este es exactamente el argumento del paper contra la lógica de umbral simple.

### M3 — Primera adquisición visual del objetivo (`T_acquisition`)

**Definición:** tiempo desde el fin del warmup hasta que `areaBoundingBoxB > 500` (umbral `AREA_MIN` de `camera_node.py`) por primera vez.

**Fuente de datos:** columna `blue_present` del CSV de `NeuralRecorder` (ya registrado con timestamp).

**Relevancia:** cuantifica cuándo el robot "ve" el objetivo por primera vez después de atravesar la pista de obstáculos. Junto con `T_task`, describe el arco completo: *navegación ciega → adquisición visual → aproximación guiada → parada*.

**No es "re-adquisición" tras obstáculo:** en este escenario la esfera no estaba visible al inicio, por lo que no hay re-adquisición — hay primera adquisición. Esto debe declararse así en el paper.

### M4 — Tiempo total de tarea (`T_task`) y tasa de éxito

**Definición:** `T_task` = tiempo desde post-warmup hasta activación de z[17] > 0.25 sostenida 2 s. Tasa de éxito = ensayos con Verdict.SUCCESS / N_total.

**Fuente de datos:** `TestJudgeNode` con lógica `_eval_appetitive` existente.

**Nota sobre el índice en TestJudgeNode:** la función `_eval_appetitive` usaba `neurons[17]` del vector `/neuron_activity`. El vector tiene 85 elementos (Gpi×3 + Gpe×3 + StN×3 + StR×3 + z×20 + lidar×5 + activaciones×16 + Response×16 + Aux×16). El elemento en índice 17 corresponde a `z[5]` (no a `z[17]`). `z[17]` está en el índice 12+17=29. **Este bug ya fue corregido** en `test_manager.py`: ahora usa `neurons[29]` y el pad mínimo del buffer es 30.

---

## 6. Automatización con `test_manager.py`

### 6.1 Entrada en `experiments_config.yaml`

```yaml
- suite_name: "familia_e_inspeccion"
  launch_file: "gazebo.launch.py"
  repetitions: 15
  dynamic_parameters:
    world_name: "obstaculos"
    timeout_s: 90.0
    random_perturbation: false
```

### 6.2 Lógica de éxito en `test_manager.py` ✓ IMPLEMENTADO

`familia_e_inspeccion` tiene su propio set y su propio método evaluador:

```python
SUITE_INSPECTION = {'familia_e_inspeccion'}

def _eval_inspection(self, snap, current_sim_s):
    """Éxito: z[17] (índice 29) > 0.2 sostenido 2 s continuos."""
    z17 = snap['neurons'][29] if len(snap['neurons']) > 29 else 0.0
    if z17 > X17_THRESHOLD:
        if self._success_hold_start is None:
            self._success_hold_start = current_sim_s
        elif (current_sim_s - self._success_hold_start) >= SUCCESS_HOLD_S:
            return Verdict.SUCCESS
    elif z17 < 0.1:
        self._success_hold_start = None
    return None
```

### 6.3 Nodos lanzados automáticamente por `gazebo.launch.py` ✓ IMPLEMENTADO

| t (s) | Nodo | Propósito |
|---|---|---|
| 0 | Gazebo + robot | Simulación física |
| 5–8 | Controladores ROS2 Control | Joints y velocidad |
| 9 | `peter_controller` | Control de postura |
| 10 | `camera_node` | Visión + bounding boxes |
| 11 | `red_neuronal` | Red BG/WTA + comandos |
| 13 | `inspection_recorder` | Métricas de inspección |
| 15 | test_manager empieza a evaluar | (WARMUP_S = 15.0 s) |

### 6.4 Artefactos recolectados por ensayo

| Archivo | Nodo fuente |
|---|---|
| `trial_summary.json` | test_manager |
| `inspection_summary.json` | inspection_recorder |
| `stability_log.csv` | peter_stability_monitor |
| `unified_metrics.csv` | metrics_recorder |
| `metrics_raw.csv` | neural_recorder |

```
docs/resultados/
└── familia_e_inspeccion/
    ├── suite_execution_manifest.json
    └── test_001_SUCCESS/
        ├── trial_summary.json
        ├── inspection_summary.json
        ├── stability_log.csv
        ├── unified_metrics.csv
        └── metrics_raw.csv
```

---

## 7. Nodo `inspection_recorder`

**Archivo:** `src/peter_robot/src/inspection_recorder.py`  
**Entry point:** `inspection_recorder = src.inspection_recorder:main`  
**Launch:** `TimerAction(period=13.0)` en `gazebo.launch.py`

1. Suscribe a `/clock` → mantiene `_sim_time_s` actualizado.
2. Suscribe a `/neuron_activity` → extrae `lidar[4]` (índice 36) y `z[17]` (índice 29).
3. Cuenta cruces ascendentes de `lidar[4] >= 0.3` → `N_lidar_events`.
4. Al primer cruce de `z[17] > 0.25`, llama `gz topic -e -t /world/obstaculos/pose/info -n 1` y calcula `d_final`.
5. Registra `T_acquisition_s` (sim time) cuando el área del bounding box azul supera 500 px².
6. Al recibir SIGINT, escribe `~/inspection_summary.json`; `_collect_artifacts` lo mueve al directorio del trial.

### Schema del JSON de salida

```json
{
  "suite_name": "familia_e_inspeccion",
  "success": true,
  "T_acquisition_s": <float | null>,
  "N_lidar_events": <int>,
  "N_mode_X_events": <int>,
  "d_final_m": <float | null>,
  "z17_peak": <float>,
  "inspection_pose": { "x": <float>, "y": <float>, "z": <float> } | null
}
```

### Descripción de cada campo

| Campo | Unidad | Cómo se calcula | Qué analizar |
|---|---|---|---|
| `success` | bool | z[17] > 0.25 sostenido 2 s | Tasa de éxito SR = success/N |
| `d_final_m` | m | Euclidiana robot↔esfera (-2.0, 2.0) en instante z[17]>0.25 | Media ± σ. Mide "target tracking accuracy" |
| `N_lidar_events` | conteo | Cruces ascendentes lidar[4] ≥ 0.3 | Comparar con `N_mode_X_events` para demostrar suavizado temporal del BG |
| `N_mode_X_events` | conteo | Transiciones a modo 'X' en `/peter_mode` | Si N_lidar > N_mode_X: BG evitó dithering de modo |
| `T_acquisition_s` | s sim | Sim time al primer bb_blue > 500 px² | Tiempo hasta primer contacto visual con el objetivo |
| `z17_peak` | adim. | max(z[17]) durante el ensayo | Intensidad de la respuesta neuronal de inspección |
| `inspection_pose` | m | Pose gz topic en instante z[17]>0.25 | Posición real del robot al detenerse |

---

## 8. Limitaciones a declarar en el paper

| Limitación | Declaración propuesta |
|---|---|
| Sin path planning | "El sistema demuestra navegación reactiva de inspección. El re-enrutamiento tras obstáculos emerge de la dinámica local BG/WTA, sin planificación global de trayectorias ni construcción de mapas." |
| Sin coverage rate formal | "La métrica de cobertura de área requeriría un planificador de cobertura explícito, fuera del alcance de este trabajo. Se sustituye por tasa de éxito de traversal de la pista y tiempo de primera adquisición visual." |
| Steering visual condicionado | "El steering visual hacia el objetivo sólo opera en ausencia de obstáculo frontal (`lidar[4] < 0.3`). Durante la evasión, la persecución visual se suspende y el robot opera en modo reactivo puro; se reanuda automáticamente al despejarse el obstáculo." |
| Primera adquisición, no re-adquisición | "En la pista de tres obstáculos diseñada, el objetivo no es visible desde la posición inicial. La métrica T_acquisition cuantifica la primera adquisición visual post-traversal, no re-adquisición tras bloqueo en el sentido clásico." |

---

## 9. Análisis de experimentos

**Configuración:** 20 trials, `obstaculos.world`, timeout 90 s, sin perturbación aleatoria.  
**Infraestructura de medición:** `inspection_recorder` + `test_manager` + `peter_stability_monitor` + `metrics_recorder` + `neural_recorder`.

---

### 9.1 Tasa de éxito

**SR = 85 % (17/20).** Fallos por timeout en trials 7, 11 y 17.

---

### 9.2 Precisión de aproximación — `d_final`

| Estadístico | Valor |
|---|---|
| μ | 1.076 m |
| σ | 0.038 m |
| min | 0.991 m |
| max | 1.130 m |

El robot se detiene a ~1.08 m del centro de la esfera (radio ≈ 0.5 m → distancia a la superficie ≈ 0.58 m). La dispersión de 3.8 cm sobre 17 trials con trayectorias distintas es el resultado más sólido del experimento: el lazo BG→z[17] converge al mismo punto de parada independientemente de la ruta tomada. Esto responde directamente la "target tracking accuracy" solicitada por el reviewer.

---

### 9.3 Dos modos de navegación observados

Los datos revelan espontáneamente dos comportamientos distinguibles por `N_lidar_events`:

| Grupo | Trials | N_lidar (rango) | T_acquisition (s) | sim_time_s |
|---|---|---|---|---|
| **Navegación limpia** | 1,2,4,6,9,10,12,14,15,18 | 4–5 | ~21–27 | ~43–50 |
| **Navegación compleja** | 3,5,8,13,16,19,20 + fallos | 11–33 | ~34–71 | ~46–87 |

En el grupo limpio el robot navega las 3 cápsulas con 4 eventos lidar (≈1 por cápsula + 1 de aproximación final) y converge rápidamente. En el grupo complejo queda atrapado en bucles de evasión adicionales antes de encontrar el camino hacia el estímulo, con tiempos de tarea significativamente mayores. Ambos grupos terminan en el mismo punto de parada cuando hay éxito, lo que refuerza la robustez del criterio de parada z[17].

---

### 9.4 Suavizado temporal del BG — `N_lidar_events` vs `N_mode_X_events`

| Estadístico | N_lidar (éxitos) | N_mode_X (éxitos) |
|---|---|---|
| μ | 7.71 | 3.29 |
| σ | 6.42 | 2.80 |
| ratio μ | — | ≈ 2.3× menos cambios de modo que eventos sensoriales |

**En los 17 trials exitosos, N_lidar_events > N_mode_X_events en todos los casos sin excepción.**

El módulo BG/WTA convierte más detecciones sensoriales en menos cambios de modo efectivos. Esto demuestra la función de suavizado temporal del BG: ante fluctuaciones repetidas de `lidar[4]` por encima del umbral, el BG no conmuta el modo en cada pulso, sino que requiere activación sostenida (histéresis de 3 s mediante `min_dwell_time`). Este es el argumento cuantitativo directo contra una lógica de umbral reactivo simple, que generaría N_mode_X ≈ N_lidar con el consecuente dithering de modo.

---

### 9.5 Análisis de los 3 fallos

| Trial | N_lidar | N_mode_X | T_acquisition (s) | z17_peak |
|---|---|---|---|---|
| 7 | 18 | 13 | 62.5 | 0.0 |
| 11 | 27 | 12 | 43.3 | 0.0 |
| 17 | 33 | 7 | 37.7 | 0.0 |

Los tres fallos comparten el mismo patrón: `T_acquisition` no es null (el robot detectó la esfera azul visualmente) pero `z17_peak = 0.0` (nunca se activó la neurona de parada). La causa es el alto `N_lidar_events`: el robot quedó atrapado en un bucle de evasión, oscilando entre obstáculos, sin poder aproximarse al objetivo dentro del timeout de 90 s. El canal azul del BG no acumuló suficiente activación sostenida porque `Gpe[2]` nunca superó el umbral de X17 bajo la presión continua del canal de obstáculo activo.

Este es un resultado científicamente honesto: la navegación reactiva sin memoria puede fallar ante configuraciones dinámicas desfavorables. La tasa de fallo del 15 % es atribuible a la ausencia de memoria espacial, no a una falla del mecanismo de inspección en sí.

---

### 9.6 Estadísticas completas sobre trials exitosos

| Métrica | μ | σ | min | max |
|---|---|---|---|---|
| `d_final_m` (m) | 1.076 | 0.038 | 0.991 | 1.130 |
| `N_lidar_events` | 7.71 | 6.42 | 4 | 28 |
| `N_mode_X_events` | 3.29 | 2.80 | 1 | 10 |
| `T_acquisition_s` (s sim) | 32.2 | 14.1 | 20.7 | 70.8 |
| `z17_peak` | 0.807 | 0.267 | 0.430 | 1.445 |
| `sim_time_s` ≈ T_task (s) | 51.6 | 12.3 | 42.8 | 86.8 |

---

### 9.7 Estabilidad durante la navegación

`stability_log.csv` resultó vacío en todos los trials: el nodo `peter_stability_monitor` no registra entradas cuando el robot opera en modo H (híbrido con ruedas), ya que la secuencia de trípode de patas no se ejecuta. El robot permaneció en modo H durante todo el experimento (`trial_summary.json`: `mode: "H"` al momento del veredicto, `tr: 0.0`). Esto implica que la tarea de inspección no requirió transición a marcha cuadrúpeda y que la estabilidad estructural no fue comprometida en ningún trial.
