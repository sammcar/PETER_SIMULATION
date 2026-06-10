# Makefile — PETER_SIMULATION (raíz del repositorio)
# Interfaz principal para construir, gestionar Docker y lanzar experimentos.
#
# Uso desde ~/PETER_SIMULATION:
#   make help              → Ver todos los comandos disponibles
#   make docker-build      → Construir imagen Docker
#   make docker-create     → Crear y arrancar el contenedor
#   make build             → Compilar workspace dentro del contenedor
#   make sim-single        → Lanzar simulación familia A
#   make sim-multi         → Lanzar simulación familia B
#
# Sobreescritura de variables:
#   make <target> CONTAINER=otro_nombre
#   make <target> IMAGE=otra_imagen:tag
#   make <target> DOMAIN_ID=1

# ── Variables (deben coincidir con docs/Makefile) ─────────────────────────────
IMAGE       ?= peter_sim:local
CONTAINER   ?= peter_simulation
WS          ?= /ros2_ws
REPO_ROOT   ?= $(HOME)/PETER_SIMULATION
WS_HOST     ?= $(REPO_ROOT)/ros2_ws
DOMAIN_ID   ?= 0
STIM        ?= blue
STIM_X      ?= 4.0
STIM_Y      ?= -2.0
SPAWN_RED   ?= true
SPAWN_BLUE  ?= true
SPAWN_GREEN ?= false

# Ruta al Makefile con todos los targets
MK := ros2_ws/src/peter_robot/docs/Makefile

# Pasar variables al sub-make para que los overrides funcionen
PASSTHROUGH := IMAGE=$(IMAGE) CONTAINER=$(CONTAINER) WS=$(WS) \
               REPO_ROOT=$(REPO_ROOT) WS_HOST=$(WS_HOST) DOMAIN_ID=$(DOMAIN_ID) \
               STIM=$(STIM) STIM_X=$(STIM_X) STIM_Y=$(STIM_Y) \
               SPAWN_RED=$(SPAWN_RED) SPAWN_BLUE=$(SPAWN_BLUE) SPAWN_GREEN=$(SPAWN_GREEN)

.PHONY: help docker-build docker-create docker-start docker-stop docker-rm \
        docker-status docker-logs build rebuild clean shell \
        sim-single sim-single-red sim-single-blue sim-single-green \
        sim-multi sim-multi-conflict sim-multi-full \
        teleop-host teleop-container record neural \
        echo-metrics echo-status list-topics \
        export-results export-last-result list-results clean-container-results \
        run-experiments

help: ## Muestra todos los comandos disponibles
	@make -f $(MK) help IMAGE=$(IMAGE) CONTAINER=$(CONTAINER) 2>/dev/null || \
	  echo "[ERROR] No se encontró $(MK). Asegúrate de estar en ~/PETER_SIMULATION."

# ── Docker ────────────────────────────────────────────────────────────────────
docker-build: ## Construye la imagen Docker del proyecto ($(IMAGE))
	@make -f $(MK) docker-build $(PASSTHROUGH)

docker-create: ## Crea el contenedor peter_simulation (requiere imagen)
	@make -f $(MK) docker-create $(PASSTHROUGH)

docker-start: ## Inicia el contenedor existente
	@make -f $(MK) docker-start $(PASSTHROUGH)

docker-stop: ## Detiene el contenedor
	@make -f $(MK) docker-stop $(PASSTHROUGH)

docker-rm: ## Elimina el contenedor
	@make -f $(MK) docker-rm $(PASSTHROUGH)

docker-status: ## Muestra el estado actual del contenedor
	@make -f $(MK) docker-status $(PASSTHROUGH)

docker-logs: ## Muestra los últimos logs del contenedor
	@make -f $(MK) docker-logs $(PASSTHROUGH)

# ── Workspace ─────────────────────────────────────────────────────────────────
build: ## Compila el workspace dentro del contenedor
	@make -f $(MK) build $(PASSTHROUGH)

rebuild: ## Limpia y recompila el workspace dentro del contenedor
	@make -f $(MK) rebuild $(PASSTHROUGH)

clean: ## Limpia build/install/log dentro del contenedor
	@make -f $(MK) clean $(PASSTHROUGH)

shell: ## Abre shell interactiva en el contenedor
	@make -f $(MK) shell $(PASSTHROUGH)

# ── Simulaciones ──────────────────────────────────────────────────────────────
sim-single: ## Simulación familia A — STIM=red|blue|green STIM_X=4.0 STIM_Y=0.0
	@make -f $(MK) sim-single $(PASSTHROUGH)

sim-single-red: ## Simulación A — estímulo rojo
	@make -f $(MK) sim-single-red $(PASSTHROUGH)

sim-single-blue: ## Simulación A — estímulo azul
	@make -f $(MK) sim-single-blue $(PASSTHROUGH)

sim-single-green: ## Simulación A — obstáculo verde
	@make -f $(MK) sim-single-green $(PASSTHROUGH)

sim-multi: ## Simulación familia B — múltiples estímulos
	@make -f $(MK) sim-multi $(PASSTHROUGH)

sim-multi-conflict: ## Simulación B — conflicto rojo + azul
	@make -f $(MK) sim-multi-conflict $(PASSTHROUGH)

sim-multi-full: ## Simulación B — los tres estímulos simultáneos
	@make -f $(MK) sim-multi-full $(PASSTHROUGH)

# ── Teleop y monitoreo ────────────────────────────────────────────────────────
teleop-host: ## Teleop stdin desde host WSL
	@make -f $(MK) teleop-host $(PASSTHROUGH)

teleop-container: ## Teleop pynput dentro del contenedor
	@make -f $(MK) teleop-container $(PASSTHROUGH)

record: ## Lanza solo el nodo neural_recorder
	@make -f $(MK) record $(PASSTHROUGH)

neural: ## Lanza solo la red neuronal
	@make -f $(MK) neural $(PASSTHROUGH)

echo-metrics: ## Muestra /experiment/metrics en tiempo real
	@make -f $(MK) echo-metrics $(PASSTHROUGH)

echo-status: ## Muestra /experiment/status en tiempo real
	@make -f $(MK) echo-status $(PASSTHROUGH)

list-topics: ## Lista topics ROS 2 activos
	@make -f $(MK) list-topics $(PASSTHROUGH)

# ── Resultados ────────────────────────────────────────────────────────────────
export-results: ## Exporta resultados del contenedor al host (menú interactivo)
	@make -f $(MK) export-results $(PASSTHROUGH)

export-last-result: ## Exporta el resultado más reciente
	@make -f $(MK) export-last-result $(PASSTHROUGH)

list-results: ## Lista los resultados disponibles en el contenedor
	@make -f $(MK) list-results $(PASSTHROUGH)

clean-container-results: ## Elimina todos los resultados del contenedor
	@make -f $(MK) clean-container-results $(PASSTHROUGH)

run-experiments: ## Lanza el orquestador automático de experimentos dentro del contenedor
	@make -f $(MK) run-experiments $(PASSTHROUGH)

sim-terrain: ## Lanza la simulación base de Gazebo en el entorno de terreno irregular (Familia C1)
	@make -f $(MK) sim-terrain $(PASSTHROUGH)

run-transitions: ## Ejecuta el evaluador de estabilidad de transformaciones y resguarda la telemetría final
	@make -f $(MK) run-transitions $(PASSTHROUGH)

kill-sim: ## Mata todos los procesos zombis de Gazebo y ROS 2 dentro del contenedor
	@echo "[kill-sim] Limpiando procesos huérfanos de simulación..."
	@docker exec -it $(CONTAINER) pkill -9 -f ign || true
	@docker exec -it $(CONTAINER) pkill -9 -f ruby || true
	@docker exec -it $(CONTAINER) pkill -9 -f ros2 || true
	@echo "[kill-sim] ✓ Contenedor Peter esterilizado y listo."

tail-sim: ## Muestra los logs de la simulación en tiempo real (red neuronal, prints, etc.)
	@echo "[tail-sim] Conectando al flujo de logs en vivo... (Ctrl+C para salir)"
	@docker exec -it $(CONTAINER) tail -f /root/sim_output.log