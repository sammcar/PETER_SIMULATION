#!/bin/bash
# Entrypoint del contenedor Docker para PETER_SIMULATION
# Hace source automático del entorno ROS 2 Humble y el workspace compilado
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
