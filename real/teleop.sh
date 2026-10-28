#!/usr/bin/env bash
# Teleoperación por teclado. Uso: real/teleop.sh [robot]
set -e
ROBOT_ID="${1:-${ROBOT_ID:-1}}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

echo "🎮 TELEOPERACION ROBOT ${ROBOT_ID} (${NS}/cmd_vel)"
echo "🎮 Controles: i (avanzar), j (girar izq), l (girar der), , (retroceder), k (frenar)"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.5 -r cmd_vel:=${NS}/cmd_vel
