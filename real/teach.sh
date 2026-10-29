#!/usr/bin/env bash
# Teach + rosbag. Uso: real/teach.sh [encoders|lidar|mixed] [robot] [circuito]
# Ruta -> routes/<circuito>/lab_route_<modo>.json
# Rosbag -> evidence/laboratorio/<circuito>/teach_bag_<modo>
# Mover el robot con real/teleop.sh; Ctrl+C aquí guarda la ruta y cierra el bag.
set -e
MODE="${1:-lidar}"
ROBOT_ID="${2:-${ROBOT_ID:-2}}"
CIRCUIT="${3:-${CIRCUIT:-circuito-2}}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
mkdir -p "${REPO_DIR}/routes/${CIRCUIT}" "${REPO_DIR}/evidence/laboratorio/${CIRCUIT}"

ROUTE_FILE="${REPO_DIR}/routes/${CIRCUIT}/lab_route_${MODE}.json"
BAG_DIR="${REPO_DIR}/evidence/laboratorio/${CIRCUIT}/teach_bag_${MODE}"

echo "TEACH + ROSBAG | Robot ${ROBOT_ID} | modo ${MODE} | circuito ${CIRCUIT}"
echo "Ruta: ${ROUTE_FILE}"
echo "Rosbag: ${BAG_DIR}"
echo "Mové el robot con real/teleop.sh. Ctrl+C aquí para guardar todo."

rm -rf "${BAG_DIR}" 2>/dev/null || true
ros2 bag record -o "${BAG_DIR}" \
  "${NS}/scan" "${NS}/odom" "${NS}/cmd_vel" "${NS}/tf" "${NS}/tf_static" \
  /tnr/odom /rel_odom /tnr/status > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  echo ""
  echo "Guardando ruta en ${ROUTE_FILE}..."
  ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}' > /dev/null 2>&1 || true
  sleep 1
  echo "Cerrando rosbag..."
  kill -2 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  cp -f "${ROUTE_FILE}" "${REPO_DIR}/routes/lab_route_${MODE}.json" 2>/dev/null || true
  echo "LISTO: ${ROUTE_FILE} | ${BAG_DIR}"
}
trap cleanup EXIT

ros2 run tp1_teach_repeat localization --ros-args \
  -p stage:=teach -p mode:="${MODE}" -p use_sim_time:=false \
  -p route_file:="${ROUTE_FILE}" -p overwrite:=true \
  --params-file "${REPO_DIR}/params_tb4.yaml" \
  -r /scan:="${NS}/scan" -r /joint_states:="${NS}/joint_states" \
  -r /odom:="${NS}/odom" -r /cmd_vel:="${NS}/cmd_vel" \
  -r /tf:="${NS}/tf" -r /tf_static:="${NS}/tf_static"
