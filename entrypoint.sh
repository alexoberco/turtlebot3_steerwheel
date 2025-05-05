#!/bin/bash
set -e

# Carga entorno de ROS 2 Humble
source /opt/ros/${ROS_DISTRO}/setup.bash

# Carga entorno de Gazebo
source /usr/share/gazebo/setup.sh

# Carga tu workspace compilado
if [ -f "${COLCON_WS}/install/setup.bash" ]; then
  source "${COLCON_WS}/install/setup.bash"
fi

exec "$@"
