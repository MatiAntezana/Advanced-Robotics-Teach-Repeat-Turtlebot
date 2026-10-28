#!/usr/bin/env bash
# Carga el entorno del robot en la terminal actual (diagnóstico: ros2 topic hz, etc.).
# Uso: source real/env.sh [1|2]
ROBOT_ID="${1:-${ROBOT_ID:-1}}"
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
echo "Entorno Robot ${ROBOT_ID} cargado (domain ${DOMAIN_ID}, discovery ${TARGET_IP}:11811, ns: ${NS})"
