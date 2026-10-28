#!/usr/bin/env bash
set -e

MODE="${1:-lidar}"
ROBOT_ID="${ROBOT_ID:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${SCRIPT_DIR}/routes" "${SCRIPT_DIR}/evidence"

ROUTE_FILE="${SCRIPT_DIR}/routes/lab_route_${MODE}.json"
BAG_DIR="${SCRIPT_DIR}/evidence/teach_bag_${MODE}"

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

echo "🟢 ==================================================="
echo "🟢 INICIANDO TEACH + ROSBAG (Robot ${ROBOT_ID})"
echo "🟢 MODO:   ${MODE}"
echo "🟢 RUTA:   ${ROUTE_FILE}"
echo "🟢 ROSBAG: ${BAG_DIR}"
echo "🟢 ==================================================="
echo "ℹ️  Mové el robot con ./run_real_teleop.sh en otra terminal."
echo "ℹ️  Al llegar a la meta, presioná Ctrl+C aquí para guardar todo."
echo "🟢 ==================================================="

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
  /tnr/status > /dev/null 2>&1 &
BAG_PID=$!

cleanup() {
  echo ""
  echo "💾 Guardando ruta en ${ROUTE_FILE}..."
  ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}' > /dev/null 2>&1 || true
  sleep 1
  echo "🛑 Cerrando rosbag..."
  kill -2 $BAG_PID 2>/dev/null || true
  wait $BAG_PID 2>/dev/null || true
  echo "✅ LISTO:"
  echo "   📁 Ruta:   ${ROUTE_FILE}"
  echo "   📁 Rosbag: ${BAG_DIR}"
}
trap cleanup EXIT

# 2. Iniciar nodo de Teach
ros2 run tp1_teach_repeat localization --ros-args \
  -p stage:=teach \
  -p mode:="${MODE}" \
  -p use_sim_time:=false \
  -p route_file:="${ROUTE_FILE}" \
  -p overwrite:=true \
  --params-file "${SCRIPT_DIR}/params_tb4.yaml" \
  -r /scan:="${NS}/scan" \
  -r /joint_states:="${NS}/joint_states" \
  -r /odom:="${NS}/odom" \
  -r /cmd_vel:="${NS}/cmd_vel" \
  -r /tf:="${NS}/tf" \
  -r /tf_static:="${NS}/tf_static"
