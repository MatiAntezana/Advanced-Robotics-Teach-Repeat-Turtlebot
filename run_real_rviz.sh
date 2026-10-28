#!/usr/bin/env bash
set -e

ROBOT_ID="${ROBOT_ID:-${1:-2}}"

if [ "$ROBOT_ID" == "2" ]; then
  TARGET_IP="172.23.16.88"
  DOMAIN_ID=2
else
  TARGET_IP="172.23.16.70"
  DOMAIN_ID=1
fi

export ROS_DISCOVERY_SERVER="${TARGET_IP}:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_${ROBOT_ID}.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "${SCRIPT_DIR}/ros_ws/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/ros_ws/install/setup.bash"
fi

# Si se ejecuta desde sesión remota/VS Code, usar display :0 por defecto
export DISPLAY="${DISPLAY:-:0}"

echo "🖥️ ==================================================="
echo "🖥️ ABRIENDO RVIZ PARA ROBOT ${ROBOT_ID} (DISPLAY=${DISPLAY})"
echo "🖥️ Visualizando mapa, keyframes, escaneos y setpoints"
echo "🖥️ ==================================================="

rviz2 -d "${SCRIPT_DIR}/ros_ws/src/Grupo4/tp1_teach_repeat/config/tp1_view.rviz" --ros-args -p use_sim_time:=false
