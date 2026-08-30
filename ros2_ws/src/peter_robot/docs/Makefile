# Makefile — PETER Simulation Experiments
# Entorno: Docker (peter_simulation) + ROS 2 Humble + Gazebo Fortress
#
# Nombre canónico del contenedor: peter_simulation
# Debe coincidir con el container_name definido en docker-compose.yml.
# Para usar otro contenedor: make <target> CONTAINER=mi_contenedor

# ── Variables Docker ──────────────────────────────────────────────────────────
# Raíz del repositorio en el host (ajustar si el repo está en otra ruta)
REPO_ROOT   ?= $(HOME)/PETER_SIMULATION
# Nombre de la imagen Docker del proyecto
IMAGE       ?= peter_sim:local
# Nombre canónico del contenedor — NO cambiar salvo sobreescritura explícita
CONTAINER   ?= peter_simulation
# Ruta del workspace en el host (se monta en $(WS) dentro del contenedor)
WS_HOST     ?= $(REPO_ROOT)/ros2_ws
WS          ?= /ros2_ws
DOMAIN_ID   ?= 0
STIM        ?= red
STIM_X      ?= 4.0
STIM_Y      ?= 0.0
SPAWN_RED   ?= true
SPAWN_BLUE  ?= true
SPAWN_GREEN ?= true
WORLD_NAME  ?= multiple_stimuli
RED_X       ?= 4.0
RED_Y       ?= 2.0
BLUE_X      ?= 4.0
BLUE_Y      ?= -2.0
GREEN_X     ?= 3.0
GREEN_Y     ?= 0.0

DC = docker exec -it $(CONTAINER) bash -c

# Solo ROS base para compilar
ROS_SOURCE = source /opt/ros/jazzy/setup.bash

# ROS + overlay del workspace ya compilado
WS_SOURCE = source /opt/ros/jazzy/setup.bash && source $(WS)/install/setup.bash

# Ruta al script de exportación (relativa al Makefile, que vive en docs/)
EXPORT_SCRIPT ?= $(REPO_ROOT)/ros2_ws/src/peter_robot/scripts/export_results.sh

.PHONY: help build rebuild clean clean-all sim-single sim-multi teleop-host teleop-container \
        teleop-stdin-container record neural shell chown-ws echo-metrics echo-status \
        list-topics check-install check-container \
        docker-build docker-create docker-start docker-stop docker-rm docker-status docker-logs \
        export-results export-last-result list-results clean-container-results \
        run-experiments run-experiments-fsm

help: ## Muestra esta ayuda
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}; {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}'

# ── Comprobaciones de estado ──────────────────────────────────────────────────

check-container: ## Verifica que el contenedor esté corriendo (muestra error accionable si no)
	@if ! docker inspect --format '{{.State.Running}}' $(CONTAINER) 2>/dev/null | grep -q true; then \
	  echo ""; \
	  echo "  [ERROR] El contenedor '$(CONTAINER)' no está corriendo."; \
	  if docker inspect $(CONTAINER) &>/dev/null; then \
	    echo "  → Inícialo con:  make -f docs/Makefile docker-start"; \
	  else \
	    echo "  → Créalo con:    make -f docs/Makefile docker-create"; \
	    echo "  → Si la imagen no existe, primero:  make -f docs/Makefile docker-build"; \
	  fi; \
	  echo ""; \
	  exit 1; \
	fi

# ── Docker — ciclo de vida del contenedor ─────────────────────────────────────

docker-build: ## Construye la imagen Docker del proyecto ($(IMAGE))
	@echo "[docker-build] Construyendo imagen $(IMAGE) desde $(REPO_ROOT) ..."
	docker build -t $(IMAGE) $(REPO_ROOT)
	@echo "[docker-build] ✓ Imagen $(IMAGE) lista."

docker-create: ## Crea el contenedor (sin iniciarlo si ya existe); requiere imagen
	@if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -Fxq '$(IMAGE)'; then \
	  echo ""; \
	  echo "  [ERROR] La imagen '$(IMAGE)' no existe localmente."; \
	  echo "  → Constrúyela con:  make -f docs/Makefile docker-build"; \
	  echo ""; \
	  exit 1; \
	fi
	@if docker inspect $(CONTAINER) &>/dev/null; then \
	  echo "[docker-create] El contenedor '$(CONTAINER)' ya existe."; \
	  echo "  → Inícialo con:   make -f docs/Makefile docker-start"; \
	  echo "  → Elimínalo con:  make -f docs/Makefile docker-rm"; \
	  exit 0; \
	fi
	@echo "[docker-create] Creando contenedor $(CONTAINER) ..."
	docker run -d --name $(CONTAINER) \
	  --network host \
	  -e DISPLAY=$${DISPLAY:-:0} \
	  -e WAYLAND_DISPLAY=$${WAYLAND_DISPLAY:-} \
	  -e XDG_RUNTIME_DIR=$${XDG_RUNTIME_DIR:-/tmp} \
	  -e QT_X11_NO_MITSHM=1 \
	  -e ROS_DOMAIN_ID=$(DOMAIN_ID) \
	  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	  -v $(WS_HOST):$(WS):rw \
	  --restart unless-stopped \
	  $(IMAGE) tail -f /dev/null
	@echo "[docker-create] ✓ Contenedor '$(CONTAINER)' creado y corriendo."
	@echo "  → Abre una shell:  make -f docs/Makefile shell"
	@echo "  → Compila:         make -f docs/Makefile build"

docker-start: ## Inicia un contenedor existente que estaba detenido
	@if ! docker inspect $(CONTAINER) &>/dev/null; then \
	  echo ""; \
	  echo "  [ERROR] El contenedor '$(CONTAINER)' no existe."; \
	  echo "  → Créalo con:  make -f docs/Makefile docker-create"; \
	  echo ""; \
	  exit 1; \
	fi
	docker start $(CONTAINER)
	@echo "[docker-start] ✓ Contenedor '$(CONTAINER)' iniciado."

docker-stop: ## Detiene el contenedor sin eliminarlo
	@docker stop $(CONTAINER) 2>/dev/null \
	  && echo "[docker-stop] ✓ Contenedor '$(CONTAINER)' detenido." \
	  || echo "[docker-stop] El contenedor '$(CONTAINER)' no estaba corriendo."

docker-rm: ## Elimina el contenedor (lo detiene primero si está corriendo)
	@echo "[docker-rm] Eliminando contenedor '$(CONTAINER)' ..."
	@docker rm -f $(CONTAINER) 2>/dev/null && \
	  echo "[docker-rm] ✓ Contenedor '$(CONTAINER)' eliminado." || \
	  echo "[docker-rm] El contenedor '$(CONTAINER)' no existía."

docker-status: ## Muestra el estado actual del contenedor
	@docker inspect --format \
	  'Contenedor: {{.Name}}  Estado: {{.State.Status}}  PID: {{.State.Pid}}' \
	  $(CONTAINER) 2>/dev/null \
	  || echo "[INFO] El contenedor '$(CONTAINER)' no existe. Usa 'make docker-create'."

docker-logs: ## Muestra las últimas 50 líneas de log del contenedor
	@docker logs --tail=50 $(CONTAINER) 2>/dev/null \
	  || echo "[ERROR] No se pueden obtener logs: contenedor '$(CONTAINER)' no existe."


build: check-container ## Compila el workspace dentro del contenedor
	$(DC) "cd $(WS) && $(ROS_SOURCE) && colcon build"

rebuild: check-container ## Limpia y recompila el workspace dentro del contenedor
	$(DC) "cd $(WS) && $(ROS_SOURCE) && colcon build"

clean: ## Limpia build/, install/ y log/ del workspace dentro del contenedor
	$(DC) "rm -rf $(WS)/build $(WS)/install $(WS)/log"

clean-all: ## Limpia build/install/log también en el host montado
	rm -rf ~/PETER_SIMULATION/ros2_ws/build ~/PETER_SIMULATION/ros2_ws/install ~/PETER_SIMULATION/ros2_ws/log

check-install: check-container ## Verifica que el paquete peter_robot exista en el install space
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

sim-multi: check-install ## Lanza simulación con múltiples estímulos parametrizados
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 launch peter_robot multiple_stimuli.launch.py \
	         world_name:=$(WORLD_NAME) \
	         spawn_red:=$(SPAWN_RED) red_x:=$(RED_X) red_y:=$(RED_Y) \
	         spawn_blue:=$(SPAWN_BLUE) blue_x:=$(BLUE_X) blue_y:=$(BLUE_Y) \
	         spawn_green:=$(SPAWN_GREEN) green_x:=$(GREEN_X) green_y:=$(GREEN_Y)"

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

shell: check-container ## Abre shell en el contenedor
	docker exec -it $(CONTAINER) bash

echo-metrics: check-install ## Muestra métricas en tiempo real
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 topic echo /experiment/metrics"

echo-status: check-install ## Muestra status JSON
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 topic echo /experiment/status"

echo-neurons: check-install ## Muestra estado de las neuronas
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 topic echo /neuron_activity"

list-topics: check-install ## Lista topics activos
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && ros2 topic list"

# ── Exportación de resultados ─────────────────────────────────────────────────

export-results: ## Exporta resultados del contenedor al host (menú interactivo)
	@chmod +x $(EXPORT_SCRIPT)
	CONTAINER=$(CONTAINER) bash $(EXPORT_SCRIPT)

export-last-result: ## Exporta el resultado más reciente del contenedor
	@chmod +x $(EXPORT_SCRIPT)
	CONTAINER=$(CONTAINER) bash $(EXPORT_SCRIPT) --last

list-results: ## Lista los resultados disponibles en el contenedor
	@chmod +x $(EXPORT_SCRIPT)
	CONTAINER=$(CONTAINER) bash $(EXPORT_SCRIPT) --list

clean-container-results: ## Elimina TODOS los resultados del contenedor (pide confirmación)
	@chmod +x $(EXPORT_SCRIPT)
	CONTAINER=$(CONTAINER) bash $(EXPORT_SCRIPT) --clean

run-experiments: check-install ## Ejecuta el orquestador automático de experimentos dentro del contenedor
	$(DC) "$(WS_SOURCE) && ROS_DOMAIN_ID=$(DOMAIN_ID) python3 $(WS)/src/peter_robot/scripts/test_manager.py $(WS)/src/peter_robot/config/experiments_config.yaml"

run-experiments-fsm: check-install ## Ejecuta el orquestador con lógica FSM (baseline de comparación)
	$(DC) "$(WS_SOURCE) && ROS_DOMAIN_ID=$(DOMAIN_ID) python3 $(WS)/src/peter_robot/scripts/test_manager.py $(WS)/src/peter_robot/config/experiments_config_fsm.yaml"

sim-terrain: check-install ## Lanza simulación base de Gazebo en el entorno de terreno irregular (Familia C1)
	$(DC) "cd $(WS) && ROS_DOMAIN_ID=$(DOMAIN_ID) $(WS_SOURCE) && \
	       ros2 launch peter_robot gazebo.launch.py \
	       world_name:=terrain"