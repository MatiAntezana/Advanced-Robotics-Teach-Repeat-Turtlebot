#!/usr/bin/env bash
# Repeat + rosbag. Uso: real/repeat.sh [encoders|lidar|mixed] [robot]
# Ruta <- routes/lab_route_<modo>.json   Rosbag -> evidence/repeat_bag_<modo>
# Poner RVIZ=true para abrir RViz desde el launch. Ctrl+C cierra el bag.
set -e
MODE="${1:-lidar}"
ROBOT_ID="${2:-${ROBOT_ID:-1}}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
mkdir -p "${REPO_DIR}/evidence"

ROUTE_FILE="${REPO_DIR}/routes/lab_route_${MODE}.json"
BAG_DIR="${REPO_DIR}/evidence/repeat_bag_${MODE}"
[ -f "${ROUTE_FILE}" ] || { echo "❌ No existe ${ROUTE_FILE} (hacé el Teach primero)"; exit 1; }

echo "🔵 REPEAT + ROSBAG | Robot ${ROBOT_ID} | modo ${MODE}"
echo "🔵 Ruta: ${ROUTE_FILE}"
echo "🔵 Rosbag: ${BAG_DIR}"

rm -rf "${BAG_DIR}" 2>/dev/null || true
ros2 bag record -o "${BAG_DIR}" \
  "${NS}/scan" "${NS}/odom" "${NS}/cmd_vel" "${NS}/tf" "${NS}/tf_static" \
  /tnr/odom /rel_odom /tnr/setpoint /tnr/done /tnr/status /tnr/alignment \
  > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  echo ""
  echo "🛑 Cerrando rosbag..."
  kill -2 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  echo "✅ Rosbag guardado en: ${BAG_DIR}"
}
trap cleanup EXIT

ros2 launch tp1_teach_repeat repeat.launch.py \
  use_sim_time:=false mode:="${MODE}" route_file:="${ROUTE_FILE}" \
  publish_stamped:=false params_file:="${REPO_DIR}/params_tb4.yaml" \
  scan_topic:="${NS}/scan" joint_states_topic:="${NS}/joint_states" \
  odom_topic:="${NS}/odom" cmd_vel_topic:="${NS}/cmd_vel" \
  tf_topic:="${NS}/tf" tf_static_topic:="${NS}/tf_static" \
  rviz:="${RVIZ:-false}"
