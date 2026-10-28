#!/usr/bin/env bash
set -e

ROBOT_ID="${ROBOT_ID:-${1:-2}}"

if [ "$ROBOT_ID" == "2" ]; then
  TARGET_IP="172.23.16.88"
  DOMAIN_ID=2
else
  TARGET_IP="172.23.16.70"
  DOMAIN_ID=1
fi

export ROS_DISCOVERY_SERVER="${TARGET_IP}:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_${ROBOT_ID}.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=$DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash 2>/dev/null || true

echo "💾 Guardando ruta en routes/lab_route.json (Robot ${ROBOT_ID})..."
ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
