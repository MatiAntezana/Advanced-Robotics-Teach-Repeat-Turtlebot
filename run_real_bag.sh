#!/usr/bin/env bash
set -e

ROBOT_ID="${ROBOT_ID:-2}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}/evidence"

NAME="${1:-bag_$(date +%Y%m%d_%H%M%S)}"
# Si no incluye ruta, guardar dentro de evidence/
if [[ "$NAME" == *"/"* ]]; then
  OUT_PATH="${NAME}"
else
  OUT_PATH="${SCRIPT_DIR}/evidence/${NAME}"
fi

if [ "$ROBOT_ID" == "2" ]; then
  TARGET_IP="172.23.16.88"
  NS="/tb4_2"
  DOMAIN_ID=2
else
  TARGET_IP="172.23.16.70"
  NS="/tb4_1"
  DOMAIN_ID=1
fi

export ROS_DISCOVERY_SERVER="${TARGET_IP}:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_${ROBOT_ID}.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo "🔴 ==================================================="
echo "🔴 GRABANDO ROSBAG EN: ${OUT_PATH}"
echo "🔴 Grabando tópicos de Robot ${ROBOT_ID} y Teach & Repeat..."
echo "🔴 Para detener la grabación: presionar Ctrl+C"
echo "🔴 ==================================================="

ros2 bag record -o "${OUT_PATH}" \
  "${NS}/scan" \
  "${NS}/odom" \
  "${NS}/cmd_vel" \
  "${NS}/tf" \
  "${NS}/tf_static" \
  /tnr/odom \
  /rel_odom \
  /tnr/setpoint \
  /tnr/done \
  /tnr/status \
  /tnr/alignment
