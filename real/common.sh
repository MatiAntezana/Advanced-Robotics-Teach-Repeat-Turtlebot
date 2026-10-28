#!/usr/bin/env bash
# Configuración común del robot real. Se usa con `source` desde los demás scripts.
# Robot por defecto: 1. Cambiar con ROBOT_ID=2 o pasando el robot como último argumento.
#   Robot 1: 172.23.16.70 /tb4_1 (domain 1)    Robot 2: 172.23.16.88 /tb4_2 (domain 2)
ROBOT_ID="${ROBOT_ID:-1}"
REAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${REAL_DIR}/.." && pwd)"

if [ "$ROBOT_ID" == "2" ]; then
  TARGET_IP="172.23.16.88"; NS="/tb4_2"; DOMAIN_ID=2
else
  TARGET_IP="172.23.16.70"; NS="/tb4_1"; DOMAIN_ID=1
fi

export ROS_DISCOVERY_SERVER="${TARGET_IP}:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_${ROBOT_ID}.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TB4_NS="$NS"

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "${REPO_DIR}/ros_ws/install/setup.bash" ]; then
  source "${REPO_DIR}/ros_ws/install/setup.bash"
fi
