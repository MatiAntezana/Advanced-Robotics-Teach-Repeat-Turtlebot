#!/usr/bin/env bash
# Abre RViz con la configuración del TP. Uso: real/rviz.sh [robot]
set -e
ROBOT_ID="${1:-${ROBOT_ID:-2}}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
export DISPLAY="${DISPLAY:-:0}"

echo "ABRIENDO RVIZ PARA ROBOT ${ROBOT_ID} (DISPLAY=${DISPLAY})"
rviz2 -d "${REPO_DIR}/ros_ws/src/Grupo4/tp1_teach_repeat/config/tp1_view.rviz" --ros-args -p use_sim_time:=false
