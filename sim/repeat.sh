#!/usr/bin/env bash
# Repeat en simulación + rosbag. Uso: sim/repeat.sh [encoders|lidar|mixed]
# Requiere Gazebo abierto (`pixi run sim`) y la ruta del Teach del mismo modo.
# Ruta <- routes/sim_route_<modo>.json   Rosbag -> evidence/simulacion/repeat_bag_<modo>
# Ctrl+C cierra el bag (hacerlo cuando aparezca REPEAT COMPLETADO).
set -e
MODE="${1:-lidar}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
mkdir -p evidence/simulacion

ROUTE_FILE="${REPO_DIR}/routes/sim_route_${MODE}.json"
BAG_DIR="${REPO_DIR}/evidence/simulacion/repeat_bag_${MODE}"
[ -f "${ROUTE_FILE}" ] || { echo "No existe ${ROUTE_FILE} (hacé el Teach primero con sim/teach.sh ${MODE})"; exit 1; }

echo "REPEAT SIMULACION + ROSBAG | modo ${MODE}"
echo "Ruta: ${ROUTE_FILE}"
echo "Rosbag: ${BAG_DIR}"

reset_pose
rm -rf "${BAG_DIR}"
ros2 bag record -o "${BAG_DIR}" "${SIM_TOPICS[@]}" /tnr/setpoint /tnr/done /tnr/alignment > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "Cerrando rosbag..."
  kill -INT $BAG_PID 2>/dev/null || true
  sleep 0.5
  kill -15 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  echo "Rosbag guardado en: ${BAG_DIR}"
}
trap cleanup EXIT

ros2 launch tp1_teach_repeat repeat.launch.py mode:="${MODE}" route_file:="${ROUTE_FILE}" rviz:="${RVIZ:-false}"
