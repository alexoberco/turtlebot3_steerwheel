# ────────────────────────────────────────────────────────────
# 1. Base de ROS 2 Humble y variables
# ────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop
ARG ROS_DISTRO=humble
ENV \
  ROS_DOMAIN_ID=30 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  COLCON_WS=/home/ros/turtlebot_ws

# Usa bash para que 'source' esté disponible
SHELL ["/bin/bash", "-c"]

# ────────────────────────────────────────────────────────────
# 2. Instalación de herramientas, utilidades de joystick y ROS
# ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    joystick \
    jstest-gtk \
    evtest \
    python3-evdev \
    udev \
    ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-joy \
  && rm -rf /var/lib/apt/lists/*

# ────────────────────────────────────────────────────────────
# 3. Configurar udev para joysticks
# ────────────────────────────────────────────────────────────
RUN mkdir -p /etc/udev/rules.d \
  && echo 'KERNEL=="js[0-9]*", MODE="0666"' > /etc/udev/rules.d/99-joystick.rules

# ────────────────────────────────────────────────────────────
# 4. Crear workspace, copiar tu código y clonar TurtleBot3
# ────────────────────────────────────────────────────────────
WORKDIR ${COLCON_WS}

# Copia TODO el contenido de tu workspace al contenedor
COPY . .                                     

# ────────────────────────────────────────────────────────────
# 5. Compilar el workspace
# ────────────────────────────────────────────────────────────
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install \
      --from-paths src --ignore-src -r -y \
      --skip-keys hls_lfcd_lds_driver \
      --skip-keys tf_transformations && \
    colcon build --symlink-install

# ────────────────────────────────────────────────────────────
# 6. Persistir configuraciones en bashrc de /home/ros
# ────────────────────────────────────────────────────────────
RUN echo 'export ROS_DOMAIN_ID=30       # TURTLEBOT3'            >> /home/ros/.bashrc \
  && echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp'             >> /home/ros/.bashrc \
  && echo 'source /usr/share/gazebo/setup.sh'                      >> /home/ros/.bashrc \
  && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash'               >> /home/ros/.bashrc \
  && echo 'source /home/ros/turtlebot_ws/install/setup.bash'       >> /home/ros/.bashrc  # NO usar '~', usar rutas absolutas :contentReference[oaicite:3]{index=3}

# ────────────────────────────────────────────────────────────
# 7. Entrypoint: carga entornos en cada ejecución
# ────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
