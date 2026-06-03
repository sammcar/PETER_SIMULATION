# README — Pruebas Experimentales PETER

> **ROS distro:** ROS 2 Humble  
> **Simulador:** Gazebo Fortress (gz-sim 6.x)  
> **Entorno principal:** Docker (contenedor `peter_simulation`) + WSL2 Ubuntu 22.04  
> **Workspace host:** `~/PETER_SIMULATION/ros2_ws` → montado en contenedor como `/ros2_ws`

---

## Tabla de contenidos

1. [Estructura de archivos añadidos](#1-estructura-de-archivos-añadidos)
2. [Configuración Docker](#2-configuración-docker)
3. [Compilación](#3-compilación)
4. [Familia A — Pruebas con único estímulo](#4-familia-a--pruebas-con-único-estímulo)
5. [Familia B — Pruebas con múltiples estímulos](#5-familia-b--pruebas-con-múltiples-estímulos)
6. [Nodo de instrumentación: `neural_recorder`](#6-nodo-de-instrumentación-neural_recorder)
7. [Métricas calculadas](#7-métricas-calculadas)
8. [Teleoperación](#8-teleoperación)
9. [Makefile — targets disponibles](#9-makefile--targets-disponibles)
10. [Exportación de resultados](#10-exportación-de-resultados)
11. [Permisos y notas de build](#11-permisos-y-notas-de-build)

---

## 1. Estructura de archivos añadidos

```
ros2_ws/src/peter_robot/
├── worlds/
│   ├── single_stimulus.world      # Terreno plano, un estímulo parametrizable
│   └── multiple_stimuli.world     # Terreno plano, hasta 3 estímulos simultáneos
├── launch/
│   ├── single_stimulus.launch.py  # Launch familia A
│   └── multiple_stimuli.launch.py # Launch familia B
├── src/
│   ├── neural_recorder.py         # Nodo de instrumentación y métricas
│   └── peter_teleop_stdin.py      # Teleop por stdin (sin pynput)
└── docs/
    ├── README_experiments.md      # Este archivo
    └── Makefile                   # Targets Docker para experimentos
```

---

## 2. Configuración Docker

El entorno de ejecución estándar es el contenedor **`peter_simulation`** con las
siguientes opciones relevantes:

| Configuración            | Valor                                    |
|--------------------------|------------------------------------------|
| `--network`              | `host`                                   |
| `ROS_DOMAIN_ID`          | `0` (por defecto)                        |
| X11 / visualización      | `/tmp/.X11-unix` y `$XDG_RUNTIME_DIR` montados |
| Bind-mount workspace     | Host `~/PETER_SIMULATION/ros2_ws` → `/ros2_ws` |

> **Nombre canónico del contenedor:** `peter_simulation`  
> Este nombre está declarado en `docker-compose.yml` y es el valor por defecto
> de la variable `CONTAINER` en `docs/Makefile`. Para usar un nombre distinto
> pasa `CONTAINER=otro_nombre` a cualquier target de make.
>
> **Pipeline rápido desde cero:**
> ```bash
> make docker-build    # construir imagen
> make docker-create   # crear y arrancar contenedor
> make build           # compilar workspace
> ```

Los cambios editados en WSL / VS Code (Remote-WSL) se reflejan inmediatamente
en el contenedor gracias al bind-mount.

---

## 3. Compilación

**Siempre compilar dentro del contenedor** para evitar problemas de permisos y
asegurar que Gazebo y las librerías nativas se resuelvan correctamente:

```bash
# Desde el host WSL — compila en el contenedor
make -f docs/Makefile build

# Equivalente manual
docker exec -it peter_simulation bash -c \
  "source /opt/ros/humble/setup.bash && \
   cd /ros2_ws && colcon build --symlink-install"
```

> **Nota de permisos:** si `colcon build` dentro del contenedor genera archivos
> `root`-owned en el host, ejecuta:
> ```bash
> sudo chown -R $(id -u):$(id -g) ~/PETER_SIMULATION/ros2_ws
> ```
> O usa `make -f docs/Makefile chown-ws` para ver el comando exacto.

---

## 4. Familia A — Pruebas con único estímulo

### World: `worlds/single_stimulus.world`

- Terreno plano 100×100 m, sin ruido, fricción μ=1.0.
- Sin modelos embebidos: el estímulo se inyecta dinámicamente vía launch.
- Robot siempre en pose inicial fija (x=0, y=0, yaw=0).

### Launch: `launch/single_stimulus.launch.py`

| Argumento       | Por defecto  | Descripción                            |
|-----------------|-------------|----------------------------------------|
| `stimulus_type` | `red`        | Modelo: `red` \| `blue` \| `green`    |
| `stimulus_x`    | `4.0`        | Coordenada X del estímulo              |
| `stimulus_y`    | `0.0`        | Coordenada Y del estímulo              |
| `stimulus_z`    | `0.5`        | Coordenada Z del estímulo              |
| `robot_x/y/z`   | `0/0/1.2`    | Pose inicial del robot                 |
| `record_metrics`| `true`       | Activar `neural_recorder`              |
| `use_sim_time`  | `true`       | Usar reloj de simulación               |

```bash
# Estímulo rojo (hostil) en posición por defecto
make -f docs/Makefile sim-single

# Estímulo azul (apetente) personalizado
make -f docs/Makefile sim-single STIM=blue STIM_X=3.0 STIM_Y=1.0

# Equivalente directo con ros2 launch (dentro del contenedor)
docker exec -it peter_simulation bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 launch peter_robot single_stimulus.launch.py \
     stimulus_type:=blue stimulus_x:=3.0 stimulus_y:=1.0"
```

### Escenarios de prueba — Familia A

| ID  | Estímulo | X    | Y    | Objetivo                               |
|-----|----------|------|------|----------------------------------------|
| A1  | `red`    | 4.0  | 0.0  | Respuesta huida (modo H)               |
| A2  | `blue`   | 4.0  | 0.0  | Respuesta caza (modo cuadrúpedo)       |
| A3  | `green`  | 3.0  | 0.0  | Evitación de obstáculo (modo omni)     |
| A4  | `red`    | 4.0  | 2.0  | Estímulo lateral derecho               |
| A5  | `blue`   | 4.0  | -2.0 | Estímulo lateral izquierdo             |

---

## 5. Familia B — Pruebas con múltiples estímulos

### World: `worlds/multiple_stimuli.world`

- Misma geometría que `single_stimulus.world`.
- Soporte para hasta 3 estímulos simultáneos (rojo, azul, verde).

### Launch: `launch/multiple_stimuli.launch.py`

| Argumento       | Por defecto | Descripción                              |
|-----------------|-------------|------------------------------------------|
| `spawn_red`     | `true`      | Activar estímulo hostil rojo             |
| `spawn_blue`    | `true`      | Activar estímulo apetente azul           |
| `spawn_green`   | `false`     | Activar obstáculo verde                  |
| `red_x/y`       | `4.0, 2.0`  | Posición estímulo rojo                   |
| `blue_x/y`      | `4.0, -2.0` | Posición estímulo azul                   |
| `green_x/y`     | `3.0, 0.0`  | Posición obstáculo verde                 |
| `record_metrics`| `true`      | Activar `neural_recorder`                |

```bash
# Conflicto rojo + azul (caso estándar)
make -f docs/Makefile sim-multi-conflict

# Tres estímulos simultáneos
make -f docs/Makefile sim-multi-full

# Equivalente manual
docker exec -it peter_simulation bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 launch peter_robot multiple_stimuli.launch.py \
     spawn_red:=true spawn_blue:=true spawn_green:=true"
```

### Escenarios de prueba — Familia B

| ID  | Estímulos activos      | Objetivo                                         |
|-----|------------------------|--------------------------------------------------|
| B1  | red + blue             | Resolución de conflicto hostil vs. apetente      |
| B2  | red + green            | Huida con obstáculo en camino                    |
| B3  | blue + green           | Caza con obstáculo interpuesto                   |
| B4  | red + blue + green     | Conflicto triple, máxima presión de decisión     |
| B5  | red (x=4,y=2) + blue (x=4,y=-2) | Estímulos simétricos — simetría de decisión |

---

## 6. Nodo de instrumentación: `neural_recorder`

Archivo: `src/neural_recorder.py`  
Ejecutable ROS 2: `neural_recorder`

### Suscripciones

| Topic                | Tipo                   | Uso                                      |
|----------------------|------------------------|------------------------------------------|
| `neuron_activity`    | `Float32MultiArray`    | Vector de activación de todas las neuronas |
| `/cmd_vel`           | `Twist`                | Comando motor (para latencia)            |
| `/bounding_box/red`  | `Float32MultiArray`    | Presencia/intensidad estímulo rojo       |
| `/bounding_box/blue` | `Float32MultiArray`    | Presencia/intensidad estímulo azul       |
| `/peter_mode`        | `String`               | Modo actual del robot                    |

### Publicaciones

| Topic                 | Tipo                 | Contenido                                       |
|-----------------------|----------------------|-------------------------------------------------|
| `/experiment/metrics` | `Float32MultiArray`  | `[latency, firing_var, temp_consistency, λ]`   |
| `/experiment/status`  | `String`             | JSON con todas las métricas y metadatos         |

### Parámetros

| Parámetro        | Tipo    | Por defecto              | Descripción                     |
|------------------|---------|--------------------------|---------------------------------|
| `experiment_type`| string  | `unknown`                | Etiqueta del experimento        |
| `window_size`    | int     | `50`                     | Tamaño ventana deslizante       |
| `output_dir`     | string  | `~/peter_experiments`    | Directorio de salida CSV        |
| `save_csv`       | bool    | `true`                   | Guardar métricas en CSV         |

### Salida CSV

Los archivos se guardan en `~/peter_experiments/<exp_type>_<timestamp>/metrics.csv`:

```
sim_time_s, latency_s, firing_variance, temporal_consistency, lambda_efficiency, mode, red_present, blue_present
```

---

## 7. Métricas calculadas

### Latencia de decisión

**Definición:** Tiempo transcurrido (segundos) desde que un estímulo es detectado
por primera vez (área del bounding box > umbral) hasta que el nodo `red_neuronal`
emite el primer comando motor significativo (`|cmd_vel| > 0.01`).

```
latency = t_first_cmd - t_stimulus_appeared
```

**Interpretación:** Latencias bajas indican mayor capacidad de respuesta del circuito
de ganglios basales. Se calcula por episodio y se acumula en historial.

---

### Varianza de disparo

**Definición:** Promedio de la varianza temporal de cada neurona, calculada en una
ventana deslizante de `window_size` ciclos:

```
firing_variance = mean( var(neuron_i, over time) for all i )
```

**Interpretación:** Valores altos indican actividad oscilatoria o inestabilidad;
valores bajos indican convergencia o silencio. En presencia de estímulo
se espera varianza moderada-alta reflejando el procesamiento activo.

---

### Consistencia temporal

**Definición:** Correlación coseno media entre vectores de actividad en ciclos
consecutivos dentro de la ventana:

```
consistency = mean( dot(a_t, a_{t+1}) / (|a_t| · |a_{t+1}|) )
```

Rango: [-1, 1]. Valor 1.0 = perfectamente consistente (estado estable).

**Interpretación:** Mide la estabilidad de la representación neuronal a lo largo
del tiempo. Un sistema robusto debería mostrar alta consistencia una vez
tomada la decisión.

---

### Eficiencia de resolución de conflictos λ

**Definición:**

- Sin conflicto (un solo estímulo activo): λ = 1.0 (por definición).
- Con conflicto (dos o más estímulos simultáneos):

```
λ = cmd_magnitude / (||activity_vector|| + ε)
```

clamped to [0, 1].

**Interpretación:** λ → 1 indica que el sistema genera un comando motor claro y
energético a pesar del conflicto de estímulos; λ → 0 indica parálisis o
indecisión. Es la métrica central para evaluar la familia B.

---

## 8. Teleoperación

### Opción 1 — Stdin (recomendada desde host WSL)

Captura de teclado vía `termios`/`tty`, sin dependencias de pynput ni display.

```bash
# Desde el host WSL (con Python y ROS sourced)
cd ~/PETER_SIMULATION/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/peter_robot/src/peter_teleop_stdin.py

# O con make (desde docs/)
make -f docs/Makefile teleop-host
```

| Tecla       | Acción                  |
|-------------|-------------------------|
| `w` / `↑`   | Avanzar                 |
| `s` / `↓`   | Retroceder              |
| `a` / `←`   | Girar izquierda         |
| `d` / `→`   | Girar derecha           |
| `q`         | Lateral izquierda       |
| `e`         | Lateral derecha         |
| `espacio`   | Detener                 |
| `x`         | Salir                   |

### Opción 2 — pynput (dentro del contenedor)

Requiere que el contenedor tenga acceso a un display X11.

```bash
make -f docs/Makefile teleop-container
# O directamente:
docker exec -it peter_simulation bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 run peter_robot peter_teleop_keyboard"
```

> **Nota:** La captura de teclado por pynput puede fallar dentro del contenedor
> si no hay display X11 configurado. Se recomienda la opción stdin.

---

## 9. Makefile — targets disponibles

El `docs/Makefile` puede ejecutarse desde `ros2_ws/src/peter_robot/` o desde la raíz
del repositorio (`~/PETER_SIMULATION`) usando el `Makefile` raíz:

```bash
# Desde la raíz del repositorio (más cómodo)
cd ~/PETER_SIMULATION
make help

# O desde el paquete
cd ~/PETER_SIMULATION/ros2_ws/src/peter_robot
make -f docs/Makefile help
```

### Docker — ciclo de vida del contenedor

| Target             | Descripción                                               |
|--------------------|-----------------------------------------------------------|
| `docker-build`     | Construye la imagen `peter_sim:local` desde el Dockerfile |
| `docker-create`    | Crea el contenedor `peter_simulation` (workspace montado) |
| `docker-start`     | Inicia el contenedor existente                            |
| `docker-stop`      | Detiene el contenedor sin eliminarlo                      |
| `docker-rm`        | Elimina el contenedor                                     |
| `docker-status`    | Muestra estado del contenedor                             |
| `docker-logs`      | Muestra últimas 50 líneas de log del contenedor           |

### Flujo inicial desde cero

```bash
cd ~/PETER_SIMULATION
make docker-build       # Construir imagen
make docker-create      # Crear y arrancar el contenedor
make build              # Compilar workspace dentro del contenedor
make sim-single         # Lanzar primera simulación
```

### Workspace y simulaciones

| Target                    | Descripción                                           |
|---------------------------|-------------------------------------------------------|
| `build`                   | Compila el workspace en el contenedor                 |
| `rebuild`                 | Limpia y recompila el workspace                       |
| `clean`                   | Elimina build/install/log                             |
| `shell`                   | Shell interactiva en el contenedor                    |
| `chown-ws`                | Muestra comando para corregir permisos root-owned     |
| `sim-single`              | Simulación familia A (STIM=red\|blue\|green)          |
| `sim-single-red`          | Simulación A con estímulo rojo                        |
| `sim-single-blue`         | Simulación A con estímulo azul                        |
| `sim-single-green`        | Simulación A con obstáculo verde                      |
| `sim-multi`               | Simulación familia B (configurable)                   |
| `sim-multi-conflict`      | Simulación B: rojo + azul (conflicto)                 |
| `sim-multi-full`          | Simulación B: los tres estímulos                      |
| `teleop-host`             | Teleop stdin desde host WSL                           |
| `teleop-container`        | Teleop pynput dentro del contenedor                   |
| `record`                  | Lanza solo `neural_recorder`                          |
| `neural`                  | Lanza solo `red_neuronal`                             |
| `echo-metrics`            | Muestra `/experiment/metrics` en tiempo real          |
| `list-topics`             | Lista topics ROS 2 activos                            |
| `export-results`          | **Exporta resultados del contenedor al host (menú interactivo)** |
| `export-last-result`      | Exporta el resultado más reciente del contenedor      |
| `list-results`            | Lista los resultados disponibles en el contenedor     |
| `clean-container-results` | Elimina todos los resultados del contenedor           |

Variables configurables: `CONTAINER` (por defecto `peter_simulation`), `IMAGE`,
`WS`, `WS_HOST`, `REPO_ROOT`, `DOMAIN_ID`, `STIM`, `STIM_X`, `STIM_Y`,
`SPAWN_RED`, `SPAWN_BLUE`, `SPAWN_GREEN`.

---

## 10. Exportación de resultados

Los archivos CSV generados por `neural_recorder` se guardan dentro del contenedor
en `/root/peter_experiments/<exp_type>_<timestamp>/`. Para extraerlos al host:

### Exportación interactiva (recomendada)

```bash
cd ~/PETER_SIMULATION/ros2_ws/src/peter_robot
make -f docs/Makefile export-results
```

El script `scripts/export_results.sh` guía al usuario paso a paso:
1. Detecta si el contenedor está corriendo.
2. Lista las sesiones disponibles.
3. Permite seleccionar una o varias (con `fzf` si está instalado, o menú numerado).
4. Ofrece renombrar el directorio exportado.
5. Copia a `docs/resultados/` dentro del paquete (host path:
   `~/PETER_SIMULATION/ros2_ws/src/peter_robot/docs/resultados/`).
6. Ofrece eliminar los archivos exportados del contenedor para liberar espacio.

### Otros targets de exportación

```bash
# Solo listar resultados sin exportar
make -f docs/Makefile list-results

# Exportar automáticamente el más reciente (confirma nombre y borrado)
make -f docs/Makefile export-last-result

# Borrar TODOS los resultados del contenedor (pide confirmación)
make -f docs/Makefile clean-container-results
```

### Dónde quedan los resultados exportados

```
~/PETER_SIMULATION/ros2_ws/src/peter_robot/docs/resultados/
└── <exp_type>_<timestamp>/
    └── metrics.csv
```

> **Nota:** La carpeta `docs/resultados/` se crea automáticamente si no existe.
> Los archivos CSV están en `.gitignore` del paquete para no subir datos brutos al repo.
> Si quieres versionar un resultado, muévelo a una carpeta con nombre significativo
> fuera de `docs/resultados/` y haz commit manualmente.

---

## 11. Permisos y notas de build

- **Siempre usar `colcon build --symlink-install`** para desarrollo iterativo.
- Builds dentro del contenedor pueden generar archivos `root`-owned en el host.
  Para corregir: `sudo chown -R $(id -u):$(id -g) ~/PETER_SIMULATION/ros2_ws`.
- `ROS_DOMAIN_ID=0` por defecto; cambiar con `DOMAIN_ID=<n>` en make targets.
- Para añadir `peter_simulation` al PATH de `GAZEBO_MODEL_PATH`, los launch files
  lo configuran automáticamente mediante `SetEnvironmentVariable`.
  
  
  
 
 # Métricas de locomocion y prueba de robustez

## Inyección de ruido artificial

La red neuronal ahora tiene la facultad de añadir ruido artificial a los sensores para hacer la prueba de robustez (esta prueba deberá hacerse independiente a las otras). Con el siguiente comando puedes poner la cantidad de ruido que se añadirá a la prueba:

```bash
ros2 run peter_robot red_neuronal --ros-args -p nl:=0
```

Donde `nl` corresponde a **Noise Level** con las siguientes opciones:

| nl | Ruido |
|----|--------|
| 0 | 0% de ruido (este es el `nl` por defecto si no se anexa como argumento del run) |
| 1 | 5% de ruido |
| 2 | 10% de ruido |

---

## Índice de Robustez (RI)

Hay que tener en cuenta que durante todas las pruebas hay que tener el nodo `RobustnessMetric` activo. Una vez reciba el RMSE de todas ellas (mediante el nodo `RMSENode`) soltará el valor de RI (**Índice de Robustez**).

Esta es la forma de interpretar el resultado:

| Valor de RI | Interpretación |
|------------|---------------|
| RI ≈ 1 | Muy robusto |
| RI ≈ 0 | Degradación severa |
| RI < 0 | El sistema colapsa con ruido |

---

## Configuración del RMSENode

Hay que tener algo en cuenta y es que el `RMSENode` (en su función `__init__`) tiene 3 variables llamadas:

```python
self.start
self.goal
self.duration
```

Estas indican:

- **self.start**: posición inicial del robot.
- **self.goal**: posición que tiene que alcanzar (podría ser el estímulo apetente en este caso).
- **self.duration**: duración en segundos que le toma al robot llegar ahí cuando `nl = 0`.

La duración se deberá definir de forma manual y la posición final puede ser el estímulo apetente o el punto donde el robot llegue después de un debido tiempo.

Por ejemplo, se podrían registrar las coordenadas del robot y la duración en segundos cuando el robot cruce el terreno rocoso y definir ese punto como `self.goal`.

> **Importante:** La prueba de robustez es la única métrica que se debe medir de forma independiente y con pruebas que no tengan obstáculos o trayectorias muy variantes, ya que el RMSE traza una línea recta entre la posición inicial y final y la define como trayectoria óptima. Esto se hace así por efectos de simplicidad y tiempo.

---

## Tópico `/Metrics`

Las demás métricas se pueden medir en cualquier prueba para medir el desempeño del robot durante la misma o simplemente para tener una idea de los cambios de posición, rotación u otras cosas que pasaron durante la prueba.

El siguiente array corresponde al tópico `/Metrics`:

```python
self.metricsArr = [
    self.Tresponse,
    self.Tswitch,
    self.roll_rms,
    self.pitch_rms,
    self.ACTIVE_NOISE_LEVEL_IDX
]
```

---

# Explicación de las métricas

## Tresponse (Tiempo de respuesta)
Corresponde al tiempo que tarda la red neuronal en generar una respuesta ante la aparición de un estímulo relevante. Esta se mide la diferencia entre el momento en el que el robot llega al tereeno rocoso (SIN HABERLO DETECTADO) hasta que stable_condition es True, esta condición se define en la linea 554 de la red neuronal, si se ve que esta métrica no converge, puedes mover los valores donde se interpreta convergencia, los cuales dependen del roll pitch y vibración

## Tswitch (Tiempo de conmutación)
Corresponde al tiempo que tarda el robot en cambiar de modo desde que se envía la instrucción de cambio de modo, empieza desde que se envía la instrucción de cambio de modo hasta cuando es estable en teoría es el tiempo que se tarda el robot en cambiar de un modo a otro

## roll_rms
Corresponde al valor RMS del ángulo de roll del robot durante toda la prueba.

## pitch_rms
Corresponde al valor RMS del ángulo de pitch del robot durante toda la prueba.

## RMSE de posición
Este corresponde al tópico /rmse_ct e indica qué tanto el robot se desvía de la ruta óptima hacia un punto fijo


  
  
