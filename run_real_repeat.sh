#!/usr/bin/env bash
set -e

MODE="${1:-mixed}"
ROBOT_ID="${ROBOT_ID:-1}"

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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros_ws"

source /opt/ros/humble/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

echo "🤖 ==================================================="
echo "🤖 INICIANDO REPEAT EN ROBOT ${ROBOT_ID} (Namespace: ${NS})"
echo "🤖 MODO SELECCIONADO: ${MODE}"
echo "🤖 Leyendo ruta de: routes/lab_route.json"
echo "🤖 ==================================================="

ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false \
  mode:="${MODE}" \
  route_file:="${SCRIPT_DIR}/routes/lab_route.json" \
  publish_stamped:=false \
  params_file:="${SCRIPT_DIR}/params_tb4.yaml" \
  scan_topic:="${NS}/scan" \
  joint_states_topic:="${NS}/joint_states" \
  cmd_vel_topic:="${NS}/cmd_vel" \
  tf_topic:="${NS}/tf" \
  tf_static_topic:="${NS}/tf_static" \
  rviz:="${RVIZ:-false}"
