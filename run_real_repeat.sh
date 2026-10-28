#!/usr/bin/env bash
set -e

MODE="${1:-mixed}"

export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}/ros_ws"

source /opt/ros/humble/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

echo "🤖 ==================================================="
echo "🤖 INICIANDO REPEAT EN ROBOT 2 (/tb4_2)"
echo "🤖 MODO SELECCIONADO: ${MODE}"
echo "🤖 Leyendo ruta de: routes/lab_route.json"
echo "🤖 ==================================================="

ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false \
  mode:="${MODE}" \
  route_file:="${SCRIPT_DIR}/routes/lab_route.json" \
  publish_stamped:=false \
  params_file:="${SCRIPT_DIR}/params_tb4.yaml" \
  scan_topic:=/tb4_2/scan \
  joint_states_topic:=/tnr/joint_states \
  cmd_vel_topic:=/tb4_2/cmd_vel
