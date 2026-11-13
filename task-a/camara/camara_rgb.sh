#!/usr/bin/env bash
# Tarea A: lanza la camara OAK-D del robot SOLO con imagen en color (sin profundidad) y en USB 2.
# Es un apano para cuando el enlace USB 3 de la camara se cae al transmitir (X_LINK_ERROR):
# manda mucho menos datos. Basta para detectar caras (usamos la imagen en color de 250x250).
# La navegacion no cambia: usa el LiDAR, que es otro aparato.
#
#   task-a/camara/camara_rgb.sh                 # camara en color + USB 2 (por defecto)
#   task-a/camara/camara_rgb.sh --usb SUPER     # color en USB 3
#   task-a/camara/camara_rgb.sh --hd            # ademas publica la imagen grande (1280x720) para fotos HD
#   task-a/camara/camara_rgb.sh --normal        # vuelve a la camara de siempre (RGBD, config original)
#   task-a/camara/camara_rgb.sh --estado        # solo mira: proceso, imagenes por segundo, log
#
# No modifica nada del robot: el archivo de parametros va a ~/tb4_udp/task_a/ y la camara se lanza
# aparte (como robot/reiniciar_camara.sh). Al reiniciar el robot, todo vuelve a ser como siempre.
# Variables: TB4_IP (192.168.0.104), SSHPASS (si tienes sshpass).
set -euo pipefail

IP="${TB4_IP:-192.168.0.104}"
TARGET="ubuntu@$IP"
MODE="rgb"
USB="HIGH"                       # HIGH = USB 2 (480 Mb/s); SUPER = USB 3
HD="false"                       # true = publicar /oakd/rgb/image_raw (1280x720) ademas de la preview
while [ $# -gt 0 ]; do
  case "$1" in
    --usb) USB="$2"; shift 2 ;;
    --hd) HD="true"; shift ;;
    --normal) MODE="normal"; shift ;;
    --estado) MODE="estado"; shift ;;
    *) echo "Opcion desconocida: $1"; exit 1 ;;
  esac
done

SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then SSH=(sshpass -e "${SSH[@]}"); fi

"${SSH[@]}" "$TARGET" "MODE=$MODE USB=$USB HD=$HD bash -s" <<'EOF'
set -e
source /etc/turtlebot4/setup.bash
DIR=~/tb4_udp/task_a
LOG=$DIR/oakd.log
mkdir -p "$DIR"

estado() {
  echo "--- proceso camara:"; pgrep -af "__node:=oakd_container" | grep -v pgrep | cut -c1-80 || echo "ninguno"
  echo "--- imagenes por segundo (8 s):"
  timeout 8 ros2 topic hz /oakd/rgb/preview/image_raw 2>&1 | grep -m1 "average rate" || echo "NO llegan imagenes"
  [ -f "$LOG" ] && { echo "--- log ($LOG):"; grep -E "USB SPEED|Pipeline type|Finished|X_LINK|closed|died" "$LOG" | tail -n 4 | cut -c1-160; } || true
}
if [ "$MODE" = "estado" ]; then estado; exit 0; fi

# detener la camara actual (la del servicio o una lanzada aparte)
if pkill -f "__node:=oakd_container"; then
  echo "[..]   camara anterior detenida"
  for i in $(seq 10); do pgrep -f "__node:=oakd_container" >/dev/null || break; sleep 1; done
  pkill -9 -f "__node:=oakd_container" 2>/dev/null || true
fi
pkill -f "oakd.launch.py" 2>/dev/null || true
sleep 2

BRINGUP=$(ros2 pkg prefix turtlebot4_bringup)/share/turtlebot4_bringup
ORIG=$BRINGUP/config/oakd_lite.yaml
if [ "$MODE" = "normal" ]; then
  PARAMS=$ORIG
  echo "[..]   lanzando la camara con la configuracion original (RGBD)"
else
  PARAMS=$DIR/oakd_rgb.yaml
  # copia de la original cambiando solo: tipo de pipeline -> RGB, velocidad USB -> $USB,
  # y con --hd que publique tambien la imagen grande (i_publish_topic)
  sed -e "s/i_pipeline_type:.*/i_pipeline_type: RGB/" \
      -e "s/i_usb_speed:.*/i_usb_speed: $USB/" \
      -e "s/i_publish_topic:.*/i_publish_topic: $HD/" "$ORIG" > "$PARAMS"
  echo "[..]   lanzando la camara: solo color, USB=$USB, HD=$HD (parametros en $PARAMS)"
fi

nohup bash -c "source /etc/turtlebot4/setup.bash && exec ros2 launch turtlebot4_bringup oakd.launch.py camera:=oakd_lite params_file:=$PARAMS" \
  > "$LOG" 2>&1 < /dev/null &

for i in $(seq 60); do
  if grep -q "Finished setting up pipeline\|Camera ready" "$LOG"; then break; fi
  if grep -qE "X_LINK|No available devices|Cannot find any device|process has died" "$LOG"; then break; fi
  sleep 1
done
sleep 5                                   # que empiece a transmitir antes de medir
estado
EOF
