#!/usr/bin/env bash
set -e

# Configuración de red para TurtleBot 4 (Robot 2)
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

mkdir -p "${SCRIPT_DIR}/routes"

echo "🤖 ==================================================="
echo "🤖 INICIANDO TEACH EN ROBOT 2 (Namespace: /tb4_2)"
echo "🤖 Guardando ruta en: routes/lab_route.json"
echo "🤖 ==================================================="

ros2 run tp1_teach_repeat localization --ros-args \
  -p stage:=teach \
  -p mode:=mixed \
  -p use_sim_time:=false \
  -p route_file:="${SCRIPT_DIR}/routes/lab_route.json" \
  -p overwrite:=true \
  --params-file "${SCRIPT_DIR}/params_tb4.yaml" \
  -r /scan:=/tb4_2/scan \
  -r /joint_states:=/tnr/joint_states \
  -r /cmd_vel:=/tb4_2/cmd_vel
