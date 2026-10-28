#!/usr/bin/env bash
set -e

export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

source /opt/ros/humble/setup.bash

echo "💾 Guardando ruta en routes/lab_route.json..."
ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
