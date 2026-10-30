#!/usr/bin/env bash
# Inicio de sesion con el robot del grupo 3. Se ejecuta desde la PC, una vez por sesion.
#
#   robot/tb4_inicio.sh            # corrige la hora si hace falta, arranca el puente y espera datos
#   robot/tb4_inicio.sh --force    # fuerza la secuencia completa (hora + servicio + Create 3)
#
# Por que existe: la Raspberry no tiene reloj ni internet y arranca con hora de 2024.
# Si se adelanta la hora con ROS ya corriendo, Fast DDS deja de ver a los nodos que
# arrancaron antes (y la Create 3, que toma la hora de la Raspberry, tambien salta).
# Secuencia verificada el 2026-10-30: hora -> reiniciar turtlebot4.service -> reiniciar
# la Create 3 (~5 min hasta que vuelve a publicar) -> puente UDP.
#
# Variables: TB4_IP (192.168.0.103), SSHPASS (si tienes sshpass), PYTHON (python con numpy).
set -euo pipefail

IP="${TB4_IP:-192.168.0.103}"
TARGET="ubuntu@$IP"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PYTHON:-python3}"
FORCE=0; [ "${1:-}" = "--force" ] && FORCE=1

SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then SSH=(sshpass -e "${SSH[@]}"); fi
# sudo en el robot: usa SSHPASS como contrasena si existe; si no, ssh -t la pedira.
if [ -n "${SSHPASS:-}" ]; then SUDO="echo '$SSHPASS' | sudo -S -p ''"; TTY=(); else SUDO="sudo"; TTY=(-t); fi

ping -c 2 -W 2 "$IP" >/dev/null || { echo "[FALLA] no hay ping a $IP (misma red? ver docs/robot_grupo3.md)"; exit 1; }
echo "[ok]   ping a $IP"

offset=$(( $("${SSH[@]}" "$TARGET" date -u +%s) - $(date -u +%s) ))
echo "[..]   desfase de reloj del robot: ${offset} s"

RESET=$FORCE
if [ "${offset#-}" -gt 30 ]; then RESET=1; fi

if [ "$RESET" = 1 ]; then
  echo "[..]   corrigiendo hora y reiniciando ROS en el robot (la Create 3 tarda ~5 min)"
  "${SSH[@]}" "${TTY[@]}" "$TARGET" "$SUDO date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)' >/dev/null && \
    $SUDO systemctl restart turtlebot4.service && sleep 20 && \
    curl -sf -m 10 -X POST http://192.168.186.2/api/reboot -o /dev/null && echo '[ok]   hora corregida, servicio reiniciado, Create 3 reiniciando'"
  WAIT=420
else
  WAIT=60
fi

"$HERE/deploy_bridge.sh" --bg | tail -n 2

echo "[..]   esperando bateria y odometria de la Create 3 (max ${WAIT} s)"
cd "$(dirname "$HERE")"     # raiz del repositorio, para importar tb4
"$PY" - "$IP" "$WAIT" <<'EOF'
import sys, time
from tb4 import TB4Client
ip, wait = sys.argv[1], float(sys.argv[2])
with TB4Client(ip) as r:
    r.wait_ready(15)
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait:
        s, p = r.status(), r.pose()
        if s and s.battery >= 0 and p is not None:
            lidar = "sin LiDAR (normal en el dock)" if s.scan_stale else "LiDAR ok"
            print(f"[ok]   robot listo en {time.monotonic() - t0:.0f} s: bateria {s.battery * 100:.0f}%, "
                  f"dock {s.docked}, {lidar}")
            sys.exit(0)
        time.sleep(3)
    print("[FALLA] la Create 3 no publica bateria/odometria. Prueba: robot/tb4_inicio.sh --force")
    sys.exit(1)
EOF
