#!/usr/bin/env bash
set -e

MODE="${1:-lidar}"
ROBOT_ID="${2:-${ROBOT_ID:-2}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}/evidence"

if [ -f "${SCRIPT_DIR}/routes/lab_route_${MODE}.json" ]; then
  ROUTE_FILE="${SCRIPT_DIR}/routes/lab_route_${MODE}.json"
else
  ROUTE_FILE="${SCRIPT_DIR}/routes/lab_route.json"
fi

BAG_DIR="${SCRIPT_DIR}/evidence/repeat_bag_${MODE}"

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
if [ -f "${SCRIPT_DIR}/ros_ws/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/ros_ws/install/setup.bash"
fi

echo "🔵 ==================================================="
echo "🔵 INICIANDO REPEAT + ROSBAG (Robot ${ROBOT_ID})"
echo "🔵 MODO:   ${MODE}"
echo "🔵 RUTA:   ${ROUTE_FILE}"
echo "🔵 ROSBAG: ${BAG_DIR}"
echo "🔵 ==================================================="

# 1. Iniciar rosbag en segundo plano
rm -rf "${BAG_DIR}" 2>/dev/null || true
ros2 bag record -o "${BAG_DIR}" \
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
  /tnr/alignment > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  echo ""
  echo "🛑 Cerrando rosbag..."
  kill -2 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  echo "✅ Rosbag guardado en: ${BAG_DIR}"
}
trap cleanup EXIT

# 2. Iniciar Repeat
ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false \
  mode:="${MODE}" \
  route_file:="${ROUTE_FILE}" \
  publish_stamped:=false \
  params_file:="${SCRIPT_DIR}/params_tb4.yaml" \
  scan_topic:="${NS}/scan" \
  joint_states_topic:="${NS}/joint_states" \
  odom_topic:="${NS}/odom" \
  cmd_vel_topic:="${NS}/cmd_vel" \
  tf_topic:="${NS}/tf" \
  tf_static_topic:="${NS}/tf_static" \
  rviz:="${RVIZ:-false}"
