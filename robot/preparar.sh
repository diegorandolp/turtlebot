#!/usr/bin/env bash
# Deja el robot listo para correr la carrera EN la Raspberry, en un solo paso (desde Git Bash en la PC, raiz del
# repo). Es la secuencia que funciono el 19/11, con los tropiezos de ese dia resueltos solos:
#
#   bash robot/preparar.sh              # inicio + LiDAR + puente + camara + copiar codigo + verificar
#   bash robot/preparar.sh --force      # igual, pero corrigiendo la hora y reiniciando la Create 3 (~5 min mas)
#   bash robot/preparar.sh --rapido     # ademas: safety_override full + puente con --max-linear 0.46 (para --vel 0.45)
#   bash robot/preparar.sh --undock     # ademas: lo saca del dock (en el dock se apagan camara y LiDAR)
#
# Despues:  bash robot/desplegar_carrera.sh --correr "Nombre Apellido"  y  python robot/control_carrera.py $TB4_IP
#
# Pasos:
#   1. robot/tb4_inicio.sh (ping, hora, servicio de ROS, espera bateria y odometria)
#   2. si no hay nodo del LiDAR (pasa tras reiniciar ROS): reinicia turtlebot4.service y espera ~40 s
#   3. robot/deploy_bridge.sh --bg (el puente con los arreglos del 19/11: sin deadlock, numpy del sistema)
#   4. task-a/camara/camara_rgb.sh --hd (hay que relanzarla tras cada reinicio de ROS)
#   5. robot/desplegar_carrera.sh (copia el codigo y los modelos)
#   6. robot/desplegar_carrera.sh --verificar (LiDAR, odometria, camara preview y HD)
#
# Variables: TB4_IP (192.168.0.104), SSHPASS (contrasena de ubuntu: con sshpass no la pide; tambien se usa para
# el sudo del paso 2, si no se pide en la terminal).
set -uo pipefail

IP="${TB4_IP:-192.168.0.104}"
export TB4_IP="$IP"
TARGET="ubuntu@$IP"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
cd "$ROOT"

FORCE=""; RAPIDO=0; UNDOCK=0
for a in "$@"; do
  case "$a" in
    --force) FORCE="--force" ;;
    --rapido) RAPIDO=1 ;;
    --undock) UNDOCK=1 ;;
    -h|--help) sed -n 2,22p "$0"; exit 0 ;;
    *) echo "opcion desconocida: $a"; sed -n 2,22p "$0"; exit 1 ;;
  esac
done

SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then
  SSH=(sshpass -e "${SSH[@]}")
fi
PY="${PYTHON:-}"
if [ -z "$PY" ]; then     # el mismo criterio que tb4_inicio.sh: el python que tenga numpy (en Git Bash "python3"
  for c in python python3 py; do   # suele ser el atajo de la Microsoft Store, sin numpy)
    if command -v "$c" >/dev/null && "$c" -c "import numpy" 2>/dev/null; then PY="$c"; break; fi
  done
  PY="${PY:-python3}"
fi
ROS="source /etc/turtlebot4/setup.bash"

paso() { echo; echo "== $*"; }
falla() { echo "[FALLA] $*"; exit 1; }

paso "1/6 inicio (ping, hora, ROS, bateria y odometria)"
bash robot/tb4_inicio.sh $FORCE || falla "tb4_inicio.sh no termino bien (prueba: bash robot/preparar.sh --force)"

paso "2/6 LiDAR"
if "${SSH[@]}" "$TARGET" "$ROS; ros2 node list 2>/dev/null | grep -qi lidar"; then
  echo "[ok]   nodo del LiDAR corriendo"
else
  echo "[..]   no hay nodo del LiDAR: reinicio turtlebot4.service (pide la contrasena de ubuntu para sudo)"
  if [ -n "${SSHPASS:-}" ]; then
    echo "$SSHPASS" | "${SSH[@]}" "$TARGET" "sudo -S -p '' systemctl restart turtlebot4.service" \
      || "${SSH[@]}" -t "$TARGET" "sudo systemctl restart turtlebot4.service"
  else
    "${SSH[@]}" -t "$TARGET" "sudo systemctl restart turtlebot4.service"
  fi
  for i in $(seq 1 12); do
    sleep 5
    if "${SSH[@]}" "$TARGET" "$ROS; ros2 node list 2>/dev/null | grep -qi lidar"; then
      echo "[ok]   nodo del LiDAR corriendo ($((i * 5)) s)"; break
    fi
    [ "$i" -eq 12 ] && falla "el LiDAR no arranca: revisar el cable USB de la torrecita y correr de nuevo"
  done
fi

paso "3/6 puente UDP"
if [ "$RAPIDO" -eq 1 ]; then
  bash robot/deploy_bridge.sh --bg -- --max-linear 0.46 || falla "deploy_bridge.sh"
  # (en el TB4 con Jazzy los nodos de la Create 3 estan bajo /_do_not_use/: 21/11)
  "${SSH[@]}" "$TARGET" "$ROS; ros2 param set /_do_not_use/motion_control safety_override full 2>/dev/null \
      || ros2 param set /motion_control safety_override full" \
    && echo "[ok]   safety_override full (vuelve a none al reiniciar la Create 3): CARRERA_ARGS=\"--turbo\" ya sirve" \
    || echo "[aviso] no se pudo poner safety_override full: la Create 3 seguira en 0.31 m/s"
else
  bash robot/deploy_bridge.sh --bg || falla "deploy_bridge.sh"
fi

paso "4/6 camara"
bash task-a/camara/camara_rgb.sh --hd || falla "camara_rgb.sh --hd"

paso "5/6 copiar el codigo"
bash robot/desplegar_carrera.sh || falla "desplegar_carrera.sh"

if [ "$UNDOCK" -eq 1 ]; then
  echo "[..]   undock (retrocede: deja ~0.5 m libres detras)"
  "$PY" -c "
from tb4.client import TB4Client
with TB4Client('$IP') as c:
    st = c.wait_ready(8)
    print('[ok]   ya estaba fuera del dock' if st.docked is False else '[ok]   ' + str(c.undock()))
" || echo "[aviso] no se pudo sacar del dock: hacerlo con python -m herramientas.monitor --drive (tecla u)"
  sleep 8                                          # que arranquen el LiDAR y la camara (power_saver)
fi

paso "6/6 verificar"
bash robot/desplegar_carrera.sh --verificar
r=$?
echo
if [ $r -eq 0 ]; then
  echo "LISTO. Pon el robot en la salida y:"
  echo "  bash robot/desplegar_carrera.sh --correr \"Nombre Apellido\""
  echo "  python robot/control_carrera.py $IP"
else
  echo "Hay FALLAS arriba. Si dice 'en el dock': sacarlo del dock (o bash robot/preparar.sh --undock)."
fi
exit $r
