# FSM Gait Arbitration — Documentación técnica

Nodo: `fsm_gait_arbitration` · Archivo: `src/fsm_gait_arbitration.py`

Este nodo implementa un árbitro de marcha basado en una Máquina de Estados Finitos (FSM) diseñado como **baseline de comparación** frente al árbitro neuronal (`red_neuronal.py`). Comparte íntegramente la dinámica de locomoción de la red neuronal; la única diferencia arquitectónica es el mecanismo de selección de modo de marcha.

---

## 1. Modos de marcha

| Modo | Símbolo | Descripción |
|------|---------|-------------|
| Móvil H | `'H'` | Locomoción hexápoda estándar. Modo por defecto. |
| Cuadrúpedo | `'C'` | Marcha cuadrúpeda. Mayor estabilidad en terreno rugoso o ante amenaza. |
| Omnidireccional X | `'X'` | Desplazamiento lateral activo. Evasión de obstáculos cercanos. |

El modo activo se publica en `/peter_mode` cada vez que cambia.

---

## 2. Entradas sensoriales

### 2.1 Cámara — estímulos visuales

El nodo de cámara detecta objetos por color y publica bounding boxes en:

| Tópico | Señal interna | Descripción |
|--------|--------------|-------------|
| `/bounding_box/red` | `R = areaBoundingBoxR / 500` | Predador (estímulo hostil) |
| `/bounding_box/blue` | `B = areaBoundingBoxB / 500` | Presa (estímulo apetente) |

El estímulo verde (obstáculo) **no proviene de cámara** sino del LiDAR (ver §2.2).

La posición angular del estímulo en la imagen se reporta con la convención:
- `0–69°` → estímulo a la **derecha**
- `70–110°` → estímulo al **frente** (`ang_p = 90`)
- `111–180°` → estímulo a la **izquierda**

### 2.2 LiDAR — obstáculos y estímulo verde G

El LiDAR (`/scan`) alimenta una red de 16 neuronas con preferencias direccionales distribuidas en cuadrantes de 90°. Cada lectura válida (0.2 m – 1.0 m) activa las neuronas mediante una función gaussiana ponderada por distancia:

```
activacion[j] = gauss(vector_rayo, Om[:,j]) × 130 / (1000 × r)
```

Las activaciones se integran en cuatro neuronas direccionales (frente, atrás, izquierda, derecha) y una neurona de obstáculo global `lidar[4]`. La señal de obstáculo verde se define como:

```
G = lidar[4] × 15   si lidar[4] × 15 > 0.2
G = 0               en caso contrario
```

### 2.3 IMU — terreno

El IMU (`/imu/data`) provee dos señales de terreno:

- **`self.pitch`** (°): ángulo de inclinación longitudinal, calculado de cuaterniones. Se actualiza **siempre**, independientemente de cualquier flag.
- **`self.accel_std`**: desviación estándar de la magnitud de aceleración en una ventana de 100 muestras. Detecta vibración por terreno rugoso.

**Flag `ignore_imu`:** Activo durante 2 segundos tras cada cambio de modo C↔H. Congela `accel_std` para que el traqueteo del cambio de marcha no se interprete como terreno rugoso. **No afecta a `pitch` ni a `roll`**, que se actualizan siempre.

---

## 3. Circuito de ganglios basales (compartido con la red neuronal)

La FSM conserva íntegramente el circuito de ganglios basales que convierte las señales sensoriales en representaciones internas de cada estímulo. Este circuito **no participa en la selección de modo** (eso lo hace la FSM), pero sí en la generación de comandos de movimiento.

```
R, G, B
  ↓
StN[0,1,2]  (Núcleo Subtalámico — con inhibición lateral cruzada)
  ↓
Gpi[0,1,2]  (Globo Pálido Interno)
Gpe[0,1,2]  (Globo Pálido Externo)
StR[0,1,2]  (Estriado)
```

Mapeo de estímulos a canales de ganglios:

| Canal | Estímulo | Señal |
|-------|----------|-------|
| `[0]` | Predador (rojo) | `Gpe[0]` — señal hostil |
| `[1]` | Obstáculo (verde/LiDAR) | `Gpe[1]` — señal de obstáculo |
| `[2]` | Presa (azul) | `Gpe[2]` — señal apetente |

---

## 4. Neuronas reemplazadas por la FSM

En la red neuronal original, seis neuronas de la red z[] tienen roles específicos en la selección de modo. En la FSM, todas se fijan a cero porque su función queda absorbida por las reglas explícitas:

| Neurona | Rol en red neuronal | En FSM |
|---------|--------------------|----|
| `z[0]` | Relay de vibración IMU → z[15] (C) | `= 0` |
| `z[1]` | Relay de pitch IMU → z[16] (H) | `= 0` |
| `z[2]` | Relay de roll IMU → z[16] (H) | `= 0` |
| `z[14]` | Activación modo X (competencia Naka-Rushton) | `= 0` |
| `z[15]` | Activación modo C (competencia Naka-Rushton) | `= 0` |
| `z[16]` | Activación modo H (competencia Naka-Rushton) | `= 0` |

Las neuronas `z[3]`–`z[13]` y `z[17]` se integran normalmente y generan los comandos de velocidad (ver §5).

---

## 5. Generación de comandos de movimiento

Este bloque es idéntico al de `red_neuronal.py`.

### 5.1 Orientación hacia el estímulo

```
ang_p = 90  (frente del robot)
ang_s = posición angular del estímulo activo
```

Condiciones de actualización de `ang_s`:

```
si Gpe[0] > 1.5 y R > 0.5  →  ang_s = posR   (orienta hacia rojo)
si Gpe[2] > 1.5 y B > 0.5  →  ang_s = posB   (orienta hacia azul)
si no                        →  ang_s = 90 × (lidar[4] < 0.3)
```

El umbral `Gpe[0] > 1.5` filtra detecciones débiles de rojo; solo cuando el estímulo es prominente se actualiza la orientación objetivo.

### 5.2 Neuronas de giro

```
z[5] = señal de giro izquierda: lidar_izq × 160 + (sin_obstáculo) × ((ang_s − ang_p) − 20)
z[6] = señal de giro derecha:  lidar_der × 160 + (sin_obstáculo) × ((ang_p − ang_s) − 20)
```

La condición `(lidar[4] < 0.3)` activa el giro solo cuando no hay obstáculo frontal cercano.

### 5.3 Neuronas de integración de giro

```
z[7]  = z[5] + z[3] − w·z[4]   (giro izq con presa, sin amenaza)
z[8]  = z[5] + z[4] − w·z[3]   (giro izq con amenaza, sin presa)
z[9]  = z[4] + z[6] − w·z[3]   (giro der con amenaza, sin presa)
z[10] = z[3] + z[6] − w·z[4]   (giro der con presa, sin amenaza)

z[11] = z[7] + z[9]   →  comando neto de giro izquierda
z[12] = z[10] + z[8]  →  comando neto de giro derecha
```

El rol de `z[4]` (señal aversiva = `Gpe[1] + 2·Gpe[0]`) es amplificar el giro opuesto al estímulo hostil: cuando hay rojo a la derecha, `z[6]` y `z[4]` suman en `z[9]`, produciendo giro izquierda (huida).

### 5.4 Velocidad lineal

```
z[13] = cte + Gpe[2] − w·|cmd_ang|·z[11] − w·|cmd_ang|·z[12] − 2w·z[17]
```

- `cte = 3`: sesgo de avance constante (el robot avanza por defecto).
- `Gpe[2]`: la presa azul aumenta la velocidad de avance.
- `−w·|cmd_ang|·z[11/12]`: al girar, el avance se reduce proporcionalmente.
- `z[17]`: frena el avance cuando el área del estímulo azul supera el umbral `Area = 28` (robot demasiado cerca de la presa).

```
cmd_lineal = lidar_front × 1.5 − lidar_back × 1.5 + z[13] − 2·z[4] × (sin_giro)
```

El término `−2·z[4]` produce retroceso activo solo cuando no hay giro (permite huida directa hacia atrás).

### 5.5 Prioridad de comandos de velocidad

```
1. Stop      si z[17] > 0.25  (obstáculo azul demasiado cerca)
2. Giro      si |cmd_ang| > ε
3. Lateral   si |cmd_lateral| > ε  y  modo ≠ H
4. Lineal    si |cmd_lineal| > ε
```

Solo un comando se envía por ciclo. El desplazamiento lateral queda bloqueado en modo H porque la marcha hexápoda no lo admite.

---

## 6. FSM — Selección de modo

### 6.1 Flag de control `IGNORE_IMU_TERRAIN`

| Valor | Efecto |
|-------|--------|
| `True` (defecto) | Reglas IMU desactivadas. Solo se usan señales RGB. Configuración para experimentos de estímulos. |
| `False` | Reglas IMU activas. Configuración para experimentos de terreno. |

### 6.2 Tabla de prioridades

| Prioridad | Condición | Modo resultante | Fuente |
|-----------|-----------|-----------------|--------|
| 1 | `IGNORE_IMU_TERRAIN=False` y `pitch > 1°` | **H** | IMU — terreno inclinado |
| 2 | `IGNORE_IMU_TERRAIN=False` y `terrainchanger=True` | **C** | IMU — terreno rugoso (40 s) |
| 3 | `G > 0` | **X** | LiDAR — obstáculo cercano |
| 4 | `R > 0.5` o persistencia C activa | **C** | Cámara — predador |
| 5 | `B > 0.5` | **H** | Cámara — presa |
| 6 | (ninguna condición) | **H** | Default |

### 6.3 Detalle de cada regla

#### Regla 1 — Terreno inclinado → H

```
if pitch > Upitch (1°):  new_mode = 'H'
```

Derivada del comportamiento de `z[1]` en la red neuronal: cualquier pitch superior al umbral `Upitch = 1°` activa la neurona de inclinación, que en la red empuja z[16] (H) con coeficiente +5. En la FSM se reemplaza por una comparación directa. El pitch se actualiza en cada mensaje IMU, antes del bloque `ignore_imu`, por lo que esta regla es siempre reactiva.

#### Regla 2 — Terreno rugoso → C (40 s)

```
if accel_std > Usigma_az:
    terrainchanger = True  (activo por 40 s)

if terrainchanger:  new_mode = 'C'
```

Esta es la única regla que no es una traducción de dinámica neuronal: el mecanismo `terrainchanger` ya existía en la red neuronal como lógica FSM explícita (umbral + timer de 40 s) que inyectaba un valor fijo en `std_dev_accel_z`. En la FSM se usa directamente la variable `terrainchanger` como condición de modo, eliminando la neurona relay `z[0]`.

`Usigma_az` tiene dos configuraciones:
- `100` para experimentos de terreno inclinado (umbral prácticamente inalcanzable; solo pitch activo).
- `3.3` para experimentos de terreno rugoso-plano.

#### Regla 3 — Obstáculo → X

```
if G > 0:  new_mode = 'X'
```

`G` es la señal integrada del LiDAR. El valor `G = 0` cuando no hay nada a menos de 1 m en ninguna dirección; cualquier `G > 0` indica presencia de obstáculo.

#### Regla 4 — Predador → C con persistencia de 6 s

```
if R > 0.5:
    last_c_trigger_time = now

c_persist_active = (modo == 'C') y (now − last_c_trigger_time < 6 s)

if R > 0.5 or c_persist_active:  new_mode = 'C'
```

El umbral `R > 0.5` equivale a un área de bounding box mayor a 250 px² (señal de presencia robusta). Los 6 segundos de persistencia permiten que el robot complete la maniobra de huida aunque el predador salga brevemente del campo visual.

#### Regla 5 — Presa → H

```
if B > 0.5:  new_mode = 'H'
```

La presa activa el modo H (hexápodo). En la red neuronal, el azul excita z[16] (H) a través de z[3] = Gpe[2], empujando la competencia winner-take-all hacia H.

#### Regla 6 — Default → H

Sin ningún estímulo activo, el sesgo constante `cte = 3` en z[13] mantiene al robot avanzando en modo H, replicando el estado de reposo de la red (donde cte sostiene z[16] alto por defecto).

---

## 7. Mecanismo `ignore_imu` tras cambio de modo

Cada vez que se ejecuta un cambio de modo entre C y H (en cualquier dirección), `publish_mode` activa `ignore_imu = True` por 2 segundos:

```python
if current_mode in ('C', 'H') and new_mode != current_mode:
    ignore_imu = True   # congela accel_std durante 2 s
    ignore_timer = now
```

Durante esos 2 segundos, `accel_std` queda congelado en su último valor, evitando que las vibraciones mecánicas del cambio de marcha activen falsamente la detección de terreno rugoso. `pitch` y `roll` no se ven afectados.

---

## 8. Ruido gaussiano (experimentos de robustez)

Todas las señales sensoriales admiten inyección de ruido gaussiano proporcional al valor medido, controlado por el parámetro ROS 2 `nl` (índice en `[0%, 5%, 10%, 20%, 30%]`):

```
ros2 run peter_robot fsm_arbitration --ros-args -p nl:=2  # 10% de ruido
```

El ruido se aplica a: bounding box rojo (posición y área), bounding box azul (posición y área), rangos LiDAR, y aceleraciones IMU (ax, ay, az). La semilla es fija (`NoiseSeed = 42`) para reproducibilidad.

---

## 9. Diferencias arquitectónicas respecto a `red_neuronal.py`

| Aspecto | Red neuronal | FSM |
|---------|-------------|-----|
| Selección de modo | Competencia Naka-Rushton entre z[14], z[15], z[16] | Reglas de prioridad explícitas |
| Señal IMU pitch | z[1] integra Naka-Rushton(pitch − 1°) → z[16] | Comparación directa `pitch > 1°` |
| Señal IMU vibración | z[0] integra Naka-Rushton(std_dev_accel_z − Usigma_az) → z[15] | Flag `terrainchanger` directo |
| Neuronas IMU relay | z[0], z[1], z[2] activas | z[0] = z[1] = z[2] = 0 |
| Neuronas de modo | z[14], z[15], z[16] dinámicas | z[14] = z[15] = z[16] = 0 |
| Persistencia en C | Inercia dinámica de z[15] | Timer explícito de 6 s |
| Locomoción | Idéntica | Idéntica |
| Ganglios basales | Idénticos | Idénticos |
