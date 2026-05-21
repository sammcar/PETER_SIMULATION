# Makefile — PETER Simulation Experiments
# Entorno: Docker (peter_sim_dev) + ROS 2 Humble + Gazebo Fortress

CONTAINER   ?= peter_sim_dev
WS          ?= /ros2_ws
DOMAIN_ID   ?= 0
STIM        ?= red
STIM_X      ?= 4.0
STIM_Y      ?= 0.0
SPAWN_RED   ?= true
SPAWN_BLUE  ?= true
SPAWN_GREEN ?= false

DC = docker exec -it $(CONTAINER) bash -c

# Solo ROS base para compilar
ROS_SOURCE = source /opt/ros/humble/setup.bash

# ROS + overlay del workspace ya compilado
WS_SOURCE = source /opt/ros/humble/setup.bash && source $(WS)/install/setup.bash

.PHONY: help build rebuild clean clean-all sim-single sim-multi teleop-host teleop-container \
        teleop-stdin-container record neural shell chown-ws echo-metrics echo-status \
        list-topics check-install

help: ## Muestra esta ayuda
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}; {printf "  \033[36m%-24s\033[0m %s\n", $$1, $$2}'

build: ## Compila el workspace dentro del contenedor
	$(DC) "cd $(WS) && $(ROS_SOURCE) && colcon build --symlink-install"

rebuild: ## Limpia y recompila el workspace dentro del contenedor
	$(DC) "cd $(WS) && rm -rf build install log && $(ROS_SOURCE) && colcon build --symlink-install"

clean: ## Limpia build/, install/ y log/ del workspace dentro del contenedor
	$(DC) "rm -rf $(WS)/build $(WS)/install $(WS)/log"

clean-all: ## Limpia build/install/log también en el host montado
	rm -rf ~/PETER_SIMULATION/ros2_ws/build ~/PETER_SIMULATION/ros2_ws/install ~/PETER_SIMULATION/ros2_ws/log

check-install: ## Verifica que el paquete peter_robot exista en el install space
	$(DC) "test -f $(WS)/install/peter_robot/share/peter_robot/package.xml && echo '[OK] peter_robot instalado' || (echo '[ERROR] peter_robot no está instalado'; exit 1)"

chown-ws: ## Corrige permisos root-owned en el host
	@echo "Ejecuta en WSL host:"
	@echo "  sudo chown -R \$$(id -u):\$$(id -g) ~/PETER_SIMULATION/ros2_ws"

sim-single: check-install ## Lanza simulación de único estímulo
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 launch peter_robot single_stimulus.launch.py \
	         stimulus_type:=$(STIM) stimulus_x:=$(STIM_X) stimulus_y:=$(STIM_Y)"

sim-single-blue: ## Simulación single estímulo azul
	$(MAKE) sim-single STIM=blue STIM_X=4.0 STIM_Y=0.0

sim-single-red: ## Simulación single estímulo rojo
	$(MAKE) sim-single STIM=red STIM_X=4.0 STIM_Y=0.0

sim-single-green: ## Simulación single estímulo verde
	$(MAKE) sim-single STIM=green STIM_X=3.0 STIM_Y=0.0

sim-multi: check-install ## Lanza simulación con múltiples estímulos
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 launch peter_robot multiple_stimuli.launch.py \
	         spawn_red:=$(SPAWN_RED) spawn_blue:=$(SPAWN_BLUE) spawn_green:=$(SPAWN_GREEN)"

sim-multi-conflict: ## Rojo + azul
	$(MAKE) sim-multi SPAWN_RED=true SPAWN_BLUE=true SPAWN_GREEN=false

sim-multi-full: ## Rojo + azul + verde
	$(MAKE) sim-multi SPAWN_RED=true SPAWN_BLUE=true SPAWN_GREEN=true

teleop-host: ## Teleop stdin desde host WSL
	cd ~/PETER_SIMULATION/ros2_ws && \
	source /opt/ros/humble/setup.bash && \
	source install/setup.bash && \
	python3 src/peter_robot/src/peter_teleop_stdin.py

teleop-container: check-install ## Teleop pynput dentro del contenedor
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 run peter_robot peter_teleop_keyboard"

teleop-stdin-container: check-install ## Teleop stdin dentro del contenedor
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 run peter_robot peter_teleop_stdin"

record: check-install ## Lanza solo el nodo de métricas
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 run peter_robot neural_recorder --ros-args -p experiment_type:=manual"

neural: check-install ## Lanza solo la red neuronal
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 run peter_robot red_neuronal"

shell: ## Abre shell en el contenedor
	docker exec -it $(CONTAINER) bash

echo-metrics: check-install ## Muestra métricas en tiempo real
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 topic echo /experiment/metrics"

echo-status: check-install ## Muestra status JSON
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 topic echo /experiment/status"

list-topics: check-install ## Lista topics activos
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && ros2 topic list"