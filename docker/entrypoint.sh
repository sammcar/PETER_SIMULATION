#!/bin/bash
# Entrypoint del contenedor Docker para PETER_SIMULATION
# Hace source automático del entorno ROS 2 Humble y el workspace compilado
set -e
source /opt/ros/humble/setup.bash

# El workspace se monta desde el host (bind-mount) y puede no estar compilado
# todavia en un contenedor recien creado -- install/setup.bash solo existe
# despues de 'colcon build'. Si el entrypoint exige el source con set -e y el
# archivo no existe, el proceso principal muere al instante y Docker (con
# --restart unless-stopped) reinicia el contenedor en bucle infinito, sin
# forma de entrar a compilarlo. Se degrada a un aviso en vez de fallar.
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
else
    echo "[entrypoint] Aviso: /ros2_ws/install/setup.bash no existe (workspace sin compilar)." >&2
    echo "[entrypoint] Corre 'make build' (o 'make -f docs/Makefile build') y reinicia el contenedor." >&2
fi

exec "$@"
