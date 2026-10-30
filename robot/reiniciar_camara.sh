#!/usr/bin/env bash
# Reinicia SOLO la camara OAK-D del robot (grupo 3), sin tocar turtlebot4.service ni la Create 3.
# Usalo cuando la camara deje de publicar (X_LINK_ERROR, start_camera sin respuesta).
#
#   robot/reiniciar_camara.sh
#
# Detiene el proceso oakd_container del servicio y lanza la camara aparte
# (turtlebot4_bringup oakd.launch.py camera:=oakd_lite), con log en ~/tb4_udp/oakd.log del robot.
# Tras reiniciar el robot, la camara vuelve a arrancar desde el servicio como siempre.
# Variables: TB4_IP (192.168.0.103), SSHPASS (si tienes sshpass).
set -euo pipefail

IP="${TB4_IP:-192.168.0.103}"
TARGET="ubuntu@$IP"
SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then SSH=(sshpass -e "${SSH[@]}"); fi

"${SSH[@]}" "$TARGET" 'bash -s' <<'EOF'
set -e
mkdir -p ~/tb4_udp
if pkill -f "__node:=oakd_container"; then
  echo "[..]   proceso de la camara detenido"
  for i in $(seq 10); do pgrep -f "__node:=oakd_container" >/dev/null || break; sleep 1; done
  pkill -9 -f "__node:=oakd_container" 2>/dev/null || true
  sleep 2
fi
nohup bash -c "source /etc/turtlebot4/setup.bash && exec ros2 launch turtlebot4_bringup oakd.launch.py camera:=oakd_lite" \
  > ~/tb4_udp/oakd.log 2>&1 < /dev/null &
echo "[..]   camara relanzada, esperando 'Camera ready' (max 60 s)"
for i in $(seq 60); do
  if grep -q "Camera ready" ~/tb4_udp/oakd.log; then echo "[ok]   camara lista en ${i} s"; exit 0; fi
  if grep -qE "X_LINK|No available devices" ~/tb4_udp/oakd.log; then break; fi
  sleep 1
done
echo "[FALLA] la camara no arranco. Revisa el cable USB de la OAK-D (puerto azul) y la bateria:"
grep -E "ERROR|X_LINK|devices" ~/tb4_udp/oakd.log | tail -n 5
exit 1
EOF
