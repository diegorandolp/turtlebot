#!/usr/bin/env bash
# Copia el puente UDP al robot y lo arranca (grupo 3). Se ejecuta desde la PC.
#
# Uso:
#   robot/deploy_bridge.sh                 # copia y arranca en segundo plano (log en el robot)
#   robot/deploy_bridge.sh --fg            # copia y arranca en esta terminal (Ctrl+C para parar)
#   robot/deploy_bridge.sh --stop          # detiene el puente
#   robot/deploy_bridge.sh --status        # dice si esta corriendo y muestra el final del log
#   robot/deploy_bridge.sh --bg -- --max-linear 0.2    # argumentos extra para tb4_bridge.py tras "--"
#
# En el robot queda: ~/tb4_udp/tb4_bridge.py + ~/tb4_udp/tb4/ (protocolo compartido).
#
# Variables: TB4_IP (por defecto 192.168.0.104). Si exportas SSHPASS y tienes sshpass,
# no te pedira la contrasena; si no, ssh la pedira (o usa ssh-copy-id una vez).
set -euo pipefail

IP="${TB4_IP:-192.168.0.104}"
TARGET="ubuntu@$IP"
DIR="tb4_udp"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
MODE="--bg"
if [ $# -gt 0 ] && [ "$1" != "--" ]; then MODE="$1"; shift; fi
[ "${1:-}" = "--" ] && shift
EXTRA="$*"

SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
SCP=(scp -q -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then
  SSH=(sshpass -e "${SSH[@]}"); SCP=(sshpass -e "${SCP[@]}")
fi

RUN="source /etc/turtlebot4/setup.bash && cd ~/$DIR && exec python3 -u tb4_bridge.py $EXTRA"
STOP="pkill -f '[t]b4_bridge.py' && echo 'puente detenido' || echo 'el puente no estaba corriendo'"

case "$MODE" in
  --stop)
    "${SSH[@]}" "$TARGET" "$STOP"; exit 0 ;;
  --status)
    "${SSH[@]}" "$TARGET" "pgrep -af '[t]b4_bridge.py' || echo 'puente NO esta corriendo'; tail -n 15 ~/$DIR/bridge.log 2>/dev/null"
    exit 0 ;;
  --fg|--bg) ;;
  *) echo "Modo desconocido: $MODE (usa --bg, --fg, --stop o --status)"; exit 1 ;;
esac

echo "== Copiando puente a $TARGET:~/$DIR"
"${SSH[@]}" "$TARGET" "mkdir -p ~/$DIR"
"${SSH[@]}" "$TARGET" "rm -rf ~/$DIR/tb4 ~/$DIR/tb4_protocol.py && mkdir -p ~/$DIR/tb4"
"${SCP[@]}" "$HERE/tb4_bridge.py" "$TARGET:$DIR/"
"${SCP[@]}" "$ROOT/tb4/__init__.py" "$ROOT/tb4/protocol.py" "$ROOT/tb4/client.py" "$TARGET:$DIR/tb4/"
"${SSH[@]}" "$TARGET" "pkill -f '[t]b4_bridge.py' && sleep 1 || true"

if [ "$MODE" = "--fg" ]; then
  echo "== Arrancando en primer plano (Ctrl+C para parar)"
  exec "${SSH[@]}" -t "$TARGET" "$RUN"
fi

echo "== Arrancando en segundo plano"
"${SSH[@]}" "$TARGET" "nohup bash -c \"$RUN\" > ~/$DIR/bridge.log 2>&1 < /dev/null &"
sleep 3
"${SSH[@]}" "$TARGET" "pgrep -f '[t]b4_bridge.py' >/dev/null && echo '[ok] puente corriendo' || { echo '[FALLA] el puente no arranco:'; tail -n 20 ~/$DIR/bridge.log; exit 1; }; tail -n 5 ~/$DIR/bridge.log"
