# PLAN DE TRABAJO — P.E.T.E.R Simulation

> **Repositorio:** `sammcar/PETER_SIMULATION` (ID: 929631572)
> **Rama de trabajo:** `Deiv` (NUNCA modificar `main` directamente)
> **Objetivo:** Ejecutar la simulación de Gazebo dentro de Docker usando WSLg en Windows, con ROS 2 Humble + Gazebo Fortress.

---

## Convenciones del plan

- ⬜ Tarea pendiente
- ✅ Tarea completada
- ⏱ Tiempo estimado por tarea
- Los comandos son copy-paste ready (ejecutar en WSL/Ubuntu 22.04)

---

## Fase 0 — Preparación del entorno Windows / WSL

### Tarea 0.1 — Verificar y configurar WSL2 + WSLg

**⏱ Tiempo estimado: 10–20 min**

**Criterio de aceptación:** `wsl -l -v` muestra una distribución Ubuntu 22.04 en VERSION 2; `echo $DISPLAY` devuelve `:0` o similar dentro de WSL.

```bash
# En PowerShell (Windows):
wsl --version
wsl -l -v

# Si la distro es VERSION 1, actualizar:
wsl --set-version Ubuntu-22.04 2

# Verificar WSLg (desde dentro de WSL):
echo $DISPLAY
ls /tmp/.X11-unix/
```

- ⬜ WSL2 instalado y activo
- ⬜ Ubuntu 22.04 configurada como distribución por defecto
- ⬜ WSLg funcionando (`DISPLAY` disponible)

---

### Tarea 0.2 — Instalar / verificar Docker

**⏱ Tiempo estimado: 10–15 min**

**Criterio de aceptación:** `docker --version` muestra ≥ 24.0; `docker info` no reporta errores; `docker run hello-world` ejecuta sin problemas.

```bash
docker --version
docker info
docker run hello-world
```

- ⬜ Docker instalado (Docker Desktop o Docker Engine en WSL)
- ⬜ Usuario en el grupo `docker` (o `sudo` disponible)
- ⬜ `docker run hello-world` exitoso

---

### Tarea 0.3 — Instalar drivers NVIDIA y NVIDIA Container Toolkit (opcional)

**⏱ Tiempo estimado: 15–30 min**

**Criterio de aceptación:** `nvidia-smi` dentro de WSL muestra la GPU del host; `docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi` exitoso.

```bash
# Verificar driver en WSL
nvidia-smi

# Instalar NVIDIA Container Toolkit
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L "https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list" \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test final
docker run --rm --gpus all nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

- ⬜ Driver NVIDIA ≥ 525 instalado en Windows
- ⬜ `nvidia-smi` disponible en WSL
- ⬜ NVIDIA Container Toolkit instalado
- ⬜ Docker configurado con runtime NVIDIA
- ⬜ Test de GPU en Docker exitoso

---

## Fase 1 — Clonar el repositorio y configurar rama Deiv

### Tarea 1.1 — Clonar el repositorio

**⏱ Tiempo estimado: 2–5 min**

**Criterio de aceptación:** Carpeta `PETER_SIMULATION/ros2_ws/src/` existe con `peter_robot` y `peter_robot_c`.

```bash
cd ~
git clone https://github.com/sammcar/PETER_SIMULATION.git
cd PETER_SIMULATION
ls ros2_ws/src/
```

- ⬜ Repositorio clonado exitosamente
- ⬜ Estructura `ros2_ws/src/peter_robot` y `ros2_ws/src/peter_robot_c` verificada

---

### Tarea 1.2 — Crear / cambiar a la rama Deiv

**⏱ Tiempo estimado: 2 min**

**Criterio de aceptación:** `git branch` muestra `* Deiv`.

```bash
# Si la rama Deiv ya existe en remoto:
git fetch origin
git checkout -b Deiv origin/Deiv

# Si NO existe aún:
git checkout main && git pull origin main
git checkout -b Deiv

# Verificar
git branch
```

- ⬜ Rama `Deiv` creada o chequeada
- ⬜ `git branch` muestra `* Deiv`
- ⬜ NINGÚN cambio realizado en `main`

---

## Fase 2 — Instalación de dependencias y build nativo (sin Docker)

### Tarea 2.1 — Instalar ROS 2 Humble + Gazebo Fortress

**⏱ Tiempo estimado: 20–40 min**

**Criterio de aceptación:** `ros2 --version` devuelve `humble`; `gz sim --version` devuelve `6.x` (Fortress).

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-image \
  ros-humble-ros-gz-sim ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-plotjuggler-ros python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
ros2 --version
gz sim --version
```

- ⬜ ROS 2 Humble instalado (`ros2 --version` = humble)
- ⬜ Gazebo Fortress instalado (`gz sim --version` = 6.x)
- ⬜ Paquetes `ros_gz_*` disponibles

---

### Tarea 2.2 — Instalar dependencias con rosdep

**⏱ Tiempo estimado: 5–10 min**

**Criterio de aceptación:** `rosdep install` termina sin errores (o solo warnings de paquetes Python).

```bash
sudo rosdep init || true
rosdep update
cd ~/PETER_SIMULATION/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install numpy matplotlib scipy
```

- ⬜ `rosdep` inicializado y actualizado
- ⬜ Dependencias del workspace instaladas
- ⬜ Dependencias Python instaladas

---

### Tarea 2.3 — Compilar workspace con colcon

**⏱ Tiempo estimado: 5–15 min**

**Criterio de aceptación:** `colcon build` termina con `0 failed`; `install/peter_robot/` existe.

```bash
source /opt/ros/humble/setup.bash
cd ~/PETER_SIMULATION/ros2_ws
colcon build --symlink-install
echo "Exit code: $?"
ls install/peter_robot/share/peter_robot/worlds/
```

- ⬜ `colcon build` sin errores (`0 failed`)
- ⬜ Mundos instalados en `install/peter_robot/share/peter_robot/worlds/`
- ⬜ `source install/setup.bash` ejecutado correctamente
- ⬜ `ros2 pkg list | grep peter` muestra `peter_robot` y `peter_robot_c`

---

### Tarea 2.4 — Ejecutar simulación nativa (sin Docker)

**⏱ Tiempo estimado: 5–10 min**

**Criterio de aceptación:** Ventana de Gazebo se abre y el robot P.E.T.E.R aparece en el mundo `empty`.

```bash
source /opt/ros/humble/setup.bash
source ~/PETER_SIMULATION/ros2_ws/install/setup.bash
ros2 launch peter_robot gazebo.launch.py world_name:=empty
```

- ⬜ Gazebo GUI se abre (WSLg)
- ⬜ Robot P.E.T.E.R spawn en el mundo
- ⬜ Controladores cargados (`ros2 control list_controllers`)
- ⬜ Topics activos (`ros2 topic list | grep joint_states`)

---

## Fase 3 — Crear el Dockerfile y construir la imagen

### Tarea 3.1 — Crear el Dockerfile

**⏱ Tiempo estimado: 5 min**

**Criterio de aceptación:** Archivo `Dockerfile` existe en la raíz del repo con contenido válido.

```bash
# Crear el directorio para el entrypoint
mkdir -p ~/PETER_SIMULATION/docker

# Crear el entrypoint
cat > ~/PETER_SIMULATION/docker/entrypoint.sh << 'EOF'
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
EOF
chmod +x ~/PETER_SIMULATION/docker/entrypoint.sh

# Verificar existencia
ls ~/PETER_SIMULATION/Dockerfile
ls ~/PETER_SIMULATION/docker/entrypoint.sh
```

- ⬜ `Dockerfile` creado en la raíz del repo
- ⬜ `docker/entrypoint.sh` creado y con permisos de ejecución
- ⬜ `docker-compose.yml` creado en la raíz del repo

---

### Tarea 3.2 — Construir la imagen Docker

**⏱ Tiempo estimado: 15–30 min (primera vez, descarga imágenes base)**

**Criterio de aceptación:** `docker images | grep peter_sim` muestra `peter_sim:local`.

```bash
cd ~/PETER_SIMULATION
docker build -t peter_sim:local .

# Verificar imagen
docker images | grep peter_sim
```

- ⬜ `docker build` sin errores
- ⬜ Imagen `peter_sim:local` visible en `docker images`
- ⬜ Tamaño de imagen razonable (< 10 GB aprox.)

---

### Tarea 3.3 — Verificar imagen con bash interactivo

**⏱ Tiempo estimado: 3–5 min**

**Criterio de aceptación:** Dentro del contenedor `ros2 pkg list | grep peter` muestra ambos paquetes.

```bash
docker run -it --rm peter_sim:local bash

# Dentro del contenedor:
ros2 pkg list | grep peter
ls /ros2_ws/install/peter_robot/share/peter_robot/worlds/
exit
```

- ⬜ Contenedor arranca sin errores
- ⬜ `peter_robot` y `peter_robot_c` listados
- ⬜ Mundos disponibles en la ruta de instalación

---

## Fase 4 — Ejecutar la simulación en Docker

### Tarea 4.1 — Ejecutar con GUI usando WSLg

**⏱ Tiempo estimado: 5–10 min**

**Criterio de aceptación:** Ventana de Gazebo se abre en Windows desde el contenedor Docker.

```bash
docker run -it --rm \
  --name peter_sim_gui \
  --network host \
  --env DISPLAY="$DISPLAY" \
  --env WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  --env XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID=0 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}" \
  peter_sim:local \
  ros2 launch peter_robot gazebo.launch.py world_name:=empty
```

- ⬜ Gazebo GUI abre desde Docker con WSLg
- ⬜ Robot spawna correctamente
- ⬜ Controladores activos

---

### Tarea 4.2 — Ejecutar en modo headless (sin GUI)

**⏱ Tiempo estimado: 3–5 min**

**Criterio de aceptación:** `gz sim -s` corre sin errores y los topics ROS están activos.

```bash
docker run -it --rm \
  --name peter_sim_headless \
  --network host \
  --env ROS_DOMAIN_ID=0 \
  peter_sim:local \
  bash -c "source /opt/ros/humble/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           gz sim -s -r /ros2_ws/install/peter_robot/share/peter_robot/worlds/empty.world"
```

- ⬜ Modo headless sin errores críticos
- ⬜ Proceso `gz_sim` corriendo (`ps aux | grep gz_sim`)

---

### Tarea 4.3 — Ejecutar con GPU (si aplica)

**⏱ Tiempo estimado: 5 min**

**Criterio de aceptación:** `docker run --gpus all` no arroja error de runtime; `nvidia-smi` dentro del contenedor muestra la GPU.

```bash
docker run -it --rm --gpus all peter_sim:local nvidia-smi
```

- ⬜ GPU accesible desde el contenedor
- ⬜ Gazebo arranca con aceleración GPU (si disponible)

---

### Tarea 4.4 — Desarrollo iterativo con workspace montado

**⏱ Tiempo estimado: 5–10 min**

**Criterio de aceptación:** Edición de un script Python en el host se refleja en el contenedor sin reconstruir la imagen.

```bash
docker run -it --rm \
  --name peter_dev \
  --network host \
  --env DISPLAY="$DISPLAY" \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ~/PETER_SIMULATION/ros2_ws/src:/ros2_ws/src \
  peter_sim:local \
  bash

# Dentro:
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run peter_robot red_neuronal
```

- ⬜ Workspace montado correctamente
- ⬜ Build dentro del contenedor exitoso
- ⬜ Cambio en el host reflejado en el contenedor

---

## Fase 5 — Verificación final y pruebas de integración

### Tarea 5.1 — Verificar simulación completa

**⏱ Tiempo estimado: 10–15 min**

**Criterio de aceptación:** Topics de joint_states, scan, imu, clock y camera activos simultáneamente.

```bash
# En una terminal (simulación corriendo):
ros2 topic list | grep -E "joint_states|scan|imu|cmd_vel|clock|camera"

# Verificar frecuencia de topics
ros2 topic hz /joint_states
ros2 topic hz /scan
```

- ⬜ `/joint_states` publicando a ≥ 10 Hz
- ⬜ `/scan` (LiDAR) publicando
- ⬜ `/imu/data` publicando
- ⬜ `/camera/image_raw` publicando
- ⬜ Controladores activos: `ros2 control list_controllers`

---

### Tarea 5.2 — Probar teleoperación y red neuronal

**⏱ Tiempo estimado: 5–10 min**

**Criterio de aceptación:** Robot se mueve con teclado; red neuronal genera comandos de velocidad.

```bash
# Terminal 2 (con simulación activa):
source ~/PETER_SIMULATION/ros2_ws/install/setup.bash
ros2 run peter_robot peter_teleop_keyboard

# Terminal 3:
ros2 run peter_robot red_neuronal
```

- ⬜ Teleoperación por teclado funciona
- ⬜ Red neuronal (`red_neuronal`) se conecta y publica en `/cmd_vel`

---

### Tarea 5.3 — Probar mundos adicionales

**⏱ Tiempo estimado: 5 min**

**Criterio de aceptación:** Al menos 2 mundos adicionales a `empty` cargan sin errores.

```bash
ros2 launch peter_robot gazebo.launch.py world_name:=depot
ros2 launch peter_robot gazebo.launch.py world_name:=contextos
```

- ⬜ Mundo `depot` carga correctamente
- ⬜ Mundo `contextos` carga con estímulos (esferas)
- ⬜ Mundo `terrain` carga correctamente

---

### Tarea 5.4 — Probar gráficas y visualización

**⏱ Tiempo estimado: 3–5 min**

**Criterio de aceptación:** RViz2 y PlotJuggler abren y reciben datos de la simulación.

```bash
ros2 launch peter_robot graficas.launch.py
```

- ⬜ RViz2 abre con configuración cargada
- ⬜ PlotJuggler abre y recibe topics

---

## Fase 6 — Documentación y cierre

### Tarea 6.1 — Crear Dockerfile y docker-compose.yml en el repo

**⏱ Tiempo estimado: 5 min**

```bash
# Verificar archivos creados
ls ~/PETER_SIMULATION/Dockerfile
ls ~/PETER_SIMULATION/docker-compose.yml
ls ~/PETER_SIMULATION/docker/entrypoint.sh

# Agregar a git (rama Deiv)
cd ~/PETER_SIMULATION
git checkout Deiv
git add Dockerfile docker-compose.yml docker/entrypoint.sh README.md PLAN.md
git commit -m "docs: agregar Dockerfile, docker-compose y documentación completa WSLg"
git push origin Deiv
```

- ⬜ `Dockerfile` en la raíz del repo
- ⬜ `docker-compose.yml` en la raíz del repo
- ⬜ `docker/entrypoint.sh` creado
- ⬜ `README.md` actualizado
- ⬜ `PLAN.md` creado
- ⬜ Commit y push a rama `Deiv` (NO a `main`)

---

### Tarea 6.2 — Revisión final de seguridad y buenas prácticas

**⏱ Tiempo estimado: 5 min**

- ⬜ Verificar que `main` NO fue modificado: `git log --oneline origin/main`
- ⬜ `.gitignore` no excluye los nuevos archivos de documentación
- ⬜ Ningún secreto ni credencial en los archivos generados
- ⬜ Flags peligrosos (`--privileged`, `--net=host`) documentados con advertencias claras

---

## Resumen del plan

| Fase | Descripción | Tiempo total estimado |
|---|---|---|
| 0 | Preparación del entorno Windows/WSL | 35–65 min |
| 1 | Clonar repo y configurar rama Deiv | 5–7 min |
| 2 | Dependencias y build nativo | 35–75 min |
| 3 | Dockerfile y build de imagen Docker | 25–40 min |
| 4 | Simulación en Docker (GUI + headless + GPU) | 20–30 min |
| 5 | Verificación y pruebas de integración | 25–40 min |
| 6 | Documentación y cierre | 10 min |
| **Total** | | **~2.5–4.5 horas** |

---

## Checklist de aceptación final

- ⬜ `wsl -l -v` muestra Ubuntu 22.04 en VERSION 2
- ⬜ `echo $DISPLAY` dentro de WSL devuelve `:0` (WSLg activo)
- ⬜ `docker build -t peter_sim:local .` sin errores
- ⬜ `docker run ... ros2 launch peter_robot gazebo.launch.py` abre Gazebo GUI en Windows
- ⬜ Robot P.E.T.E.R aparece en Gazebo y responde a comandos
- ⬜ Modo headless funciona para CI
- ⬜ GPU accesible desde Docker (si NVIDIA disponible)
- ⬜ Todos los commits en rama `Deiv`, NUNCA en `main`
- ⬜ `README.md` y `PLAN.md` actualizados en el repo

---

> **Rama de trabajo:** `Deiv` | **Repositorio:** `sammcar/PETER_SIMULATION`
> ⚠️ Prohibido modificar `main` directamente.
