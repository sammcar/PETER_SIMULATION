# Makefile — PETER Simulation Experiments
# Entorno: Docker (peter_sim_dev) + ROS 2 Humble + Gazebo Fortress
# Host: WSL2 Ubuntu 22.04  |  Contenedor: peter_sim_dev
#
# Uso rápido:
#   make help                   — muestra todos los targets
#   make build                  — compila el workspace en el contenedor
#   make sim-single             — lanza simulación de estímulo único (rojo)
#   make sim-multi              — lanza simulación de múltiples estímulos
#   make teleop-host            — teleoperación desde host WSL (stdin, recomendado)
#   make teleop-container       — teleoperación dentro del contenedor (pynput)
#   make record                 — lanza solo el nodo de métricas
#
# Variables configurables:
#   CONTAINER   nombre del contenedor Docker      (default: peter_sim_dev)
#   WS          ruta del workspace en contenedor  (default: /ros2_ws)
#   DOMAIN_ID   ROS_DOMAIN_ID                     (default: 0)
#   STIM        tipo de estímulo único            (default: red)
#   STIM_X      coordenada X del estímulo         (default: 4.0)
#   STIM_Y      coordenada Y del estímulo         (default: 0.0)
#   SPAWN_RED   lanzar estímulo rojo en multi     (default: true)
#   SPAWN_BLUE  lanzar estímulo azul en multi     (default: true)
#   SPAWN_GREEN lanzar obstáculo verde en multi   (default: false)

CONTAINER   ?= peter_sim_dev
WS          ?= /ros2_ws
DOMAIN_ID   ?= 0
STIM        ?= red
STIM_X      ?= 4.0
STIM_Y      ?= 0.0
SPAWN_RED   ?= true
SPAWN_BLUE  ?= true
SPAWN_GREEN ?= false

# Prefijo para ejecutar comandos dentro del contenedor
DC = docker exec -it $(CONTAINER) bash -c

# Source del workspace en el contenedor
SOURCE = source /opt/ros/humble/setup.bash && source $(WS)/install/setup.bash

.PHONY: help build clean sim-single sim-multi teleop-host teleop-container \
        record shell chown-ws

# ─────────────────────────────────────────────────────────────────────────────
help: ## Muestra esta ayuda
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}; {printf "  \033[36m%-22s\033[0m %s\n", $$1, $$2}'

# ─────────────────────────────────────────────────────────────────────────────
build: ## Compila el workspace con symlink-install (dentro del contenedor)
	$(DC) "cd $(WS) && $(SOURCE) && \
	       colcon build --symlink-install 2>&1"

clean: ## Limpia build/, install/ y log/ del workspace
	$(DC) "rm -rf $(WS)/build $(WS)/install $(WS)/log"

chown-ws: ## Corrige permisos si colcon build generó archivos root-owned en el host
	@echo "Ejecuta en WSL host (requiere sudo):"
	@echo "  sudo chown -R \$$(id -u):\$$(id -g) ~/PETER_SIMULATION/ros2_ws"

# ─────────────────────────────────────────────────────────────────────────────
sim-single: ## Lanza simulación de único estímulo (STIM=red|blue|green)
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 launch peter_robot single_stimulus.launch.py \
	         stimulus_type:=$(STIM) stimulus_x:=$(STIM_X) stimulus_y:=$(STIM_Y)"

sim-single-blue: ## Lanza simulación de único estímulo azul (apetente)
	$(MAKE) sim-single STIM=blue STIM_X=4.0 STIM_Y=0.0

sim-single-red: ## Lanza simulación de único estímulo rojo (hostil)
	$(MAKE) sim-single STIM=red STIM_X=4.0 STIM_Y=0.0

sim-single-green: ## Lanza simulación de único estímulo verde (obstáculo)
	$(MAKE) sim-single STIM=green STIM_X=3.0 STIM_Y=0.0

sim-multi: ## Lanza simulación con múltiples estímulos simultáneos
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 launch peter_robot multiple_stimuli.launch.py \
	         spawn_red:=$(SPAWN_RED) spawn_blue:=$(SPAWN_BLUE) spawn_green:=$(SPAWN_GREEN)"

sim-multi-conflict: ## Lanza simulación con estímulos rojo+azul (conflicto máximo)
	$(MAKE) sim-multi SPAWN_RED=true SPAWN_BLUE=true SPAWN_GREEN=false

sim-multi-full: ## Lanza simulación con los tres estímulos activos
	$(MAKE) sim-multi SPAWN_RED=true SPAWN_BLUE=true SPAWN_GREEN=true

# ─────────────────────────────────────────────────────────────────────────────
teleop-host: ## Teleoperación por stdin desde el HOST WSL (recomendado)
	@echo "[Teleop] Ejecutando en host WSL — captura de teclado via stdin"
	@cd ../../.. && python3 src/peter_robot/src/peter_teleop_stdin.py

teleop-container: ## Teleoperación por pynput dentro del contenedor
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 run peter_robot peter_teleop_keyboard"

teleop-stdin-container: ## Teleoperación por stdin dentro del contenedor
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 run peter_robot peter_teleop_stdin"

# ─────────────────────────────────────────────────────────────────────────────
record: ## Lanza solo el nodo de métricas neuronales (para adjuntar a sim en curso)
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 run peter_robot neural_recorder \
	         --ros-args -p experiment_type:=manual"

neural: ## Lanza solo la red neuronal
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 run peter_robot red_neuronal"

# ─────────────────────────────────────────────────────────────────────────────
shell: ## Abre una shell interactiva dentro del contenedor
	docker exec -it $(CONTAINER) bash

echo-metrics: ## Muestra las métricas en tiempo real (host o contenedor)
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 topic echo /experiment/status"

echo-status: ## Muestra el status JSON de la simulación
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && \
	       ros2 topic echo /experiment/status std_msgs/msg/String"

list-topics: ## Lista los topics activos
	$(DC) "ROS_DOMAIN_ID=$(DOMAIN_ID) $(SOURCE) && ros2 topic list"
