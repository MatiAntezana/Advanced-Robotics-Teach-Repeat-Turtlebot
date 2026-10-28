#!/usr/bin/env bash
# Entorno común de la simulación (Gazebo + pixi). Se usa con `source` desde los demás scripts.
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SIM_DIR}/.." && pwd)"
cd "${REPO_DIR}"
eval "$(pixi shell-hook --frozen)"

# Pose inicial del robot en Gazebo (la misma para Teach y Repeat).
START_X="${START_X:--2.0}"
START_Y="${START_Y:--0.5}"

reset_pose() {
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 5000 --req "name: \"burger\" position: {x: ${START_X} y: ${START_Y} z: 0.01} orientation: {w: 1.0}" \
    > /dev/null || echo "No se pudo reposicionar el robot (¿está corriendo 'pixi run sim'?)"
  sleep 1
}

# Tópicos de simulación que se graban. /odom es la odometría de Gazebo (referencia
# independiente de los métodos). No se graba /joint_states por su frecuencia (~1 kHz).
SIM_TOPICS=(/clock /scan /odom /cmd_vel /tf /tf_static /rel_odom /tnr/odom /tnr/status)
