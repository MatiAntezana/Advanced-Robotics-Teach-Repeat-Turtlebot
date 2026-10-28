#!/usr/bin/env bash
# Uso: source real_env.sh [1|2]   (por defecto Robot 2)
ROBOT_ID="${1:-${ROBOT_ID:-2}}"

if [ "$ROBOT_ID" == "2" ]; then
  export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
  export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
  export ROS_DOMAIN_ID=2
  export TB4_NS="/tb4_2"
  echo "Entorno Robot 2 cargado (domain 2, discovery 172.23.16.88:11811, ns: /tb4_2)"
else
  export ROS_DISCOVERY_SERVER="172.23.16.70:11811"
  export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_1.xml"
  export ROS_DOMAIN_ID=1
  export TB4_NS="/tb4_1"
  echo "Entorno Robot 1 cargado (domain 1, discovery 172.23.16.70:11811, ns: /tb4_1)"
fi

export ROS_SUPER_CLIENT=True
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash 2>/dev/null || true
