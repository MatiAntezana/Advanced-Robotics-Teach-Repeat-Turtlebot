#!/usr/bin/env bash
# Teach en simulación + rosbag. Uso: sim/teach.sh [encoders|lidar|mixed]
# Requiere Gazebo abierto (`pixi run sim`). Mover el robot en otra terminal con `pixi run teleop`.
# Ruta -> routes/sim_route_<modo>.json   Rosbag -> evidence/simulacion/teach_bag_<modo>
# Ctrl+C aquí guarda la ruta y cierra el bag.
set -e
MODE="${1:-lidar}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
mkdir -p evidence/simulacion routes

ROUTE_FILE="${REPO_DIR}/routes/sim_route_${MODE}.json"
BAG_DIR="${REPO_DIR}/evidence/simulacion/teach_bag_${MODE}"

echo "TEACH SIMULACION + ROSBAG | modo ${MODE}"
echo "Ruta: ${ROUTE_FILE}"
echo "Rosbag: ${BAG_DIR}"
echo "Mové el robot con 'pixi run teleop'. Ctrl+C aquí para guardar todo."

reset_pose
rm -rf "${BAG_DIR}"
ros2 bag record -o "${BAG_DIR}" "${SIM_TOPICS[@]}" > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "Guardando ruta en ${ROUTE_FILE}..."
  ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}' > /dev/null 2>&1 || true
  sleep 1
  echo "Cerrando rosbag..."
  kill -2 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  echo "LISTO: ${ROUTE_FILE} | ${BAG_DIR}"
}
trap cleanup EXIT

ros2 launch tp1_teach_repeat teach.launch.py mode:="${MODE}" route_file:="${ROUTE_FILE}" overwrite:=true
