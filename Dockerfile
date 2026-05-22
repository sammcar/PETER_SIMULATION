# ============================================================
# Dockerfile sugerido para PETER_SIMULATION
# Base: ROS 2 Humble + Gazebo Fortress + soporte NVIDIA
# Plataforma objetivo: Ubuntu 22.04 (Jammy)
# Rama: Deiv
# ============================================================

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

LABEL maintainer="samm.car23@gmail.com"
LABEL description="Simulación P.E.T.E.R — ROS 2 Humble + Gazebo Fortress"

# Variables de entorno
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Soporte NVIDIA (el driver reside en el host Windows/WSL; aquí solo runtime)
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# --- Dependencias del sistema ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Herramientas generales
    git curl wget gnupg lsb-release ca-certificates \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    # ROS 2 + Gazebo Fortress (ros_gz stack)
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    # Soporte GUI: librerías OpenGL para Gazebo/RViz (WSLg/X11)
    libgl1-mesa-glx libgl1-mesa-dri \
    x11-apps mesa-utils \
    # Python científico
    python3-numpy python3-matplotlib python3-scipy \
  && rm -rf /var/lib/apt/lists/*

# --- Inicializar rosdep ---
RUN rosdep init || true && rosdep update

# --- Crear y configurar workspace ---
WORKDIR /ros2_ws

# Copiar fuentes del workspace al contenedor
COPY ros2_ws/src ./src

# --- Instalar dependencias con rosdep ---
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# --- Compilar workspace ---
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Source automático del workspace en sesiones interactivas ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# --- Entrypoint ---
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
