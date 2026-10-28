#!/usr/bin/env bash
# Uso: source real_env.sh   (carga el entorno ROS 2 del Robot 2 en esta terminal)
export ROS_DISCOVERY_SERVER="172.23.16.88:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4_discovery/super_client_configuration_2.xml"
export ROS_SUPER_CLIENT=True
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
echo "Entorno Robot 2 cargado (domain 2, discovery 172.23.16.88:11811)"
