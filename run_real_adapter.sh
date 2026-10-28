#!/usr/bin/env bash
# Terminal propia: debe quedar corriendo durante Teach y Repeat.
export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
python3 "${SCRIPT_DIR}/tools/tb4_adapter.py" /tb4_2
