#!/usr/bin/env bash
# Correr la Tarea A (carrera-1) EN el TurtleBot 4 Lite (Raspberry Pi 4B, 4 GB), sin la laptop. Desde la PC:
#
#   robot/desplegar_carrera.sh                 # 1) copia el codigo y los modelos a ~/turtlebot en el robot
#   robot/desplegar_carrera.sh --instalar      # 2) instala numpy + opencv-contrib-python-headless en el robot (una vez, con internet)
#   robot/desplegar_carrera.sh --verificar     # 3) corre verificar_pi.py en el robot (modelos, puente, CPU, memoria)
#   robot/desplegar_carrera.sh --medir         #    medir_cpu.py en el robot (ms por pieza y % de un nucleo)
#   robot/desplegar_carrera.sh --correr "Nombre Apellido"   # 4) arranca la carrera en el robot (en pausa: ver --go)
#   robot/desplegar_carrera.sh --go            #    da la salida (lo que en la ventana es la tecla g)
#   robot/desplegar_carrera.sh --escuchar      #    sigue el log desde la PC y DICE EN VOZ ALTA el anuncio (el robot no tiene parlante)
#   robot/desplegar_carrera.sh --estado        #    corre? y las ultimas lineas del log
#   robot/desplegar_carrera.sh --parar         #    detiene la carrera (el puente frena el robot)
#   robot/desplegar_carrera.sh --video         # 5) trae el ultimo video y el registro de la corrida a corridas_robot/
#
# Requiere el puente corriendo en el robot (robot/tb4_inicio.sh o robot/deploy_bridge.sh): la carrera le habla
# en 127.0.0.1. Variables: TB4_IP (192.168.0.103), SSHPASS (si tienes sshpass), CARRERA_ARGS (extra para carrera.py,
# p. ej. CARRERA_ARGS="--hd --cam-periodo 0.3").
set -euo pipefail

IP="${TB4_IP:-192.168.0.103}"
TARGET="ubuntu@$IP"
DIR="turtlebot"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
MODE="${1:---copiar}"; [ $# -gt 0 ] && shift || true

SSH=(ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
SCP=(scp -q -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)
if [ -n "${SSHPASS:-}" ] && command -v sshpass >/dev/null; then
  SSH=(sshpass -e "${SSH[@]}"); SCP=(sshpass -e "${SCP[@]}")
fi
LOG="~/$DIR/carrera.log"
# En la Raspberry el puente ya tiene ROS; la carrera NO necesita ROS, solo Python, numpy y OpenCV.
PY_ROBOT="cd ~/$DIR && python3"

case "$MODE" in
  --copiar)
    cd "$ROOT"
    for f in reconocimiento/modelos/yunet_2023mar.onnx reconocimiento/modelos/arcface_mbf.onnx reconocimiento/modelos/galeria_mbf.npz; do
      [ -f "$f" ] || { echo "[FALLA] falta $f (ver reconocimiento/README.md)"; exit 1; }
    done
    echo "[..] copiando a $TARGET:~/$DIR (codigo + modelos, sin fotos ni corridas; ~20 MB)"
    tar czf - --exclude='__pycache__' --exclude='corridas' --exclude='fotos' --exclude='mapas' --exclude='*.mp4' \
        --exclude='reconocimiento/modelos/*r50*' \
        tb4 reconocimiento task-a/slam-1 task-a/auto-1 task-a/carrera-1 requirements.txt robot/oakd_caras.json robot/oakd_caras.md \
      | "${SSH[@]}" "$TARGET" "mkdir -p ~/$DIR && tar xzf - -C ~/$DIR && echo '[ok] copiado en ~/$DIR'"
    echo "Siguiente: $0 --instalar (solo la primera vez) y luego $0 --verificar"
    ;;
  --instalar)
    # Ubuntu 24.04 (Jazzy) exige --break-system-packages para pip fuera de un venv; headless = sin ventanas (menos
    # dependencias). contrib = lector de QR de WeChat. Hace falta internet en el robot (o un tethering).
    "${SSH[@]}" "$TARGET" "python3 -m pip install --user --break-system-packages 'numpy' 'opencv-contrib-python-headless>=4.8' \
      && python3 -c 'import cv2, numpy; print(\"[ok] OpenCV\", cv2.__version__, \"numpy\", numpy.__version__)'"
    ;;
  --verificar)
    "${SSH[@]}" "$TARGET" "$PY_ROBOT task-a/carrera-1/verificar_pi.py"
    ;;
  --medir)
    "${SSH[@]}" "$TARGET" "$PY_ROBOT task-a/carrera-1/medir_cpu.py ${CARRERA_ARGS:-}"
    ;;
  --correr)
    NOMBRE="${1:-}"
    [ -n "$NOMBRE" ] || { echo "uso: $0 --correr \"Nombre Apellido\"   (como en datos/rostros/)"; exit 1; }
    # nohup + setsid: sigue corriendo aunque se cierre la SSH. Empieza en PAUSA: --go da la salida.
    "${SSH[@]}" "$TARGET" "cd ~/$DIR && pkill -f '[c]arrera-1/carrera.py' 2>/dev/null; \
      setsid nohup python3 -u task-a/carrera-1/carrera.py 127.0.0.1 --no-gui --undock --mudo --video \
        --buscar '$NOMBRE' --go-archivo GO ${CARRERA_ARGS:-} > $LOG 2>&1 < /dev/null & \
      sleep 2; pgrep -af '[c]arrera-1/carrera.py' >/dev/null && echo '[ok] carrera arrancada (en pausa)' || { tail -n 20 $LOG; exit 1; }"
    echo "Ahora: $0 --escuchar (en una terminal) y $0 --go (la salida)"
    ;;
  --go)
    "${SSH[@]}" "$TARGET" "touch ~/$DIR/GO && echo '[ok] salida dada'"
    ;;
  --escuchar)
    # La Raspberry no tiene parlante: la PC sigue el log y dice en voz alta la linea del anuncio ("==== La palabra es X ====").
    "${SSH[@]}" "$TARGET" "tail -n 5 -F $LOG" | python3 -u "$ROOT/robot/escuchar_anuncio.py"
    ;;
  --estado)
    "${SSH[@]}" "$TARGET" "pgrep -af '[c]arrera-1/carrera.py' || echo 'la carrera NO esta corriendo'; tail -n 25 $LOG 2>/dev/null"
    ;;
  --parar)
    "${SSH[@]}" "$TARGET" "pkill -f '[c]arrera-1/carrera.py' && echo 'carrera detenida' || echo 'no estaba corriendo'"
    ;;
  --video)
    mkdir -p "$ROOT/corridas_robot"
    "${SSH[@]}" "$TARGET" "cd ~/$DIR/task-a/carrera-1/corridas && ls -t videos/*.mp4 2>/dev/null | head -1; ls -t *.json 2>/dev/null | head -1" \
      | while read -r f; do [ -n "$f" ] && "${SCP[@]}" "$TARGET:~/$DIR/task-a/carrera-1/corridas/$f" "$ROOT/corridas_robot/" && echo "[ok] $f -> corridas_robot/"; done
    ;;
  *)
    sed -n 2,20p "$0"; exit 1 ;;
esac
