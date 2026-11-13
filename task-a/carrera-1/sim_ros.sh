#!/usr/bin/env bash
# Prueba del camino completo SIN robot: el simulador de carrera-1 hace de robot ROS 2
# (robot_ros_sim.py) y delante corre el puente UDP real (robot/tb4_bridge.py), todo en esta PC y en
# un dominio ROS aislado (no molesta al robot del laboratorio). Luego, en otra terminal:
#
#   python task-a/carrera-1/carrera.py 127.0.0.1 --go          # el programa de competencia, sin cambios
#
#   task-a/carrera-1/sim_ros.sh                  # mundo "demo"
#   task-a/carrera-1/sim_ros.sh aleatorio:12 --persona "Sofia Salazar"
#
# Necesita ROS 2 Jazzy. En la PC de Diego: distrobox enter ros-jazzy -- task-a/carrera-1/sim_ros.sh
# Variables: SIM_DOMAIN (77), PYTHON (python3 con numpy y opencv; p. ej. ~/.venvs/tb4/bin/python).
set -eo pipefail
cd "$(dirname "$0")/../.."
MUNDO="${1:-demo}"
[ $# -gt 0 ] && shift
export ROS_DOMAIN_ID="${SIM_DOMAIN:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
PY="${PYTHON:-python3}"

"$PY" task-a/carrera-1/robot_ros_sim.py "$MUNDO" "$@" &
SIM=$!
sleep 3
"$PY" robot/tb4_bridge.py &
PUENTE=$!
trap 'kill $PUENTE $SIM 2>/dev/null; wait 2>/dev/null' EXIT INT TERM
echo "[ok] robot simulado (dominio $ROS_DOMAIN_ID) + puente en 127.0.0.1:5600. Ctrl+C para terminar."
wait
