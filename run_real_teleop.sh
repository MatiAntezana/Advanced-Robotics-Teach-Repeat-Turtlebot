#!/usr/bin/env bash
set -e

ROBOT_ID="${ROBOT_ID:-${1:-2}}"

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

echo "🎮 ==================================================="
echo "🎮 TELEOPERACION ROBOT ${ROBOT_ID} (${NS}/cmd_vel)"
echo "🎮 Controles: i (avanzar), j (girar izq), l (girar der), k (frenar)"
echo "🎮 ==================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5 -r cmd_vel:=${NS}/cmd_vel
