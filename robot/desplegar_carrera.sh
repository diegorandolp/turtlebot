#!/usr/bin/env bash
# Correr la Tarea A (carrera-1) EN el TurtleBot 4 Lite (Raspberry Pi 4B, 4 GB), sin la laptop. Desde la PC:
#
#   robot/desplegar_carrera.sh                 # 1) copia el codigo y los modelos a ~/turtlebot en el robot
#   robot/desplegar_carrera.sh --instalar      # 2) instala numpy + opencv-contrib-python-headless en el robot (una vez, con internet)
#   robot/desplegar_carrera.sh --instalar-offline   # 2b) lo mismo SIN internet en el robot: copia las ruedas de robot/ruedas_pi/
#        (bajarlas antes en la PC con internet:  robot/desplegar_carrera.sh --bajar-ruedas )
#   robot/desplegar_carrera.sh --verificar     # 3) corre verificar_pi.py en el robot (modelos, puente, CPU, memoria)
#   robot/desplegar_carrera.sh --medir         #    medir_cpu.py en el robot (ms por pieza y % de un nucleo)
#   robot/desplegar_carrera.sh --correr "Nombre Apellido"   # 4) arranca la carrera en el robot (en pausa: ver --go)
#   robot/desplegar_carrera.sh --go            #    da la salida (o el BOTON 1 de la Create 3; en la ventana es la tecla g)
#   robot/desplegar_carrera.sh --start         #    START por UDP (router) y espera el anuncio: lo imprime y lo dice (robot/control_carrera.py)
#   robot/desplegar_carrera.sh --escuchar      #    (por SSH) sigue el log y dice el anuncio; sirve aunque el puente sea viejo
#   robot/desplegar_carrera.sh --estado        #    corre? y las ultimas lineas del log
#   robot/desplegar_carrera.sh --parar         #    detiene la carrera (el puente frena el robot)
#   robot/desplegar_carrera.sh --video         # 5) trae el ultimo video y el registro de la corrida a corridas_robot/
#
# SIN PC el dia de la competencia (ver robot/carrera.service):
#   robot/desplegar_carrera.sh --servicio      # instala el servicio: la carrera arranca sola al encender y espera el BOTON 1
#   robot/desplegar_carrera.sh --buscar "Nombre Apellido"   # deja escrito a quien buscar (~/turtlebot/BUSCAR.txt)
#   robot/desplegar_carrera.sh --resultado     # lee ~/turtlebot/RESULTADO.txt (el anuncio de la ultima carrera)
#   robot/desplegar_carrera.sh --sin-servicio  # lo desinstala (para volver a --correr a mano)
#
# Requiere el puente corriendo en el robot (robot/tb4_inicio.sh o robot/deploy_bridge.sh): la carrera le habla
# en 127.0.0.1. Variables: TB4_IP (192.168.0.104), SSHPASS (si tienes sshpass), CARRERA_ARGS (extra para carrera.py,
# p. ej. CARRERA_ARGS="--hd --cam-periodo 0.3").
set -euo pipefail

IP="${TB4_IP:-192.168.0.104}"
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
      && python3 -c 'import cv2, numpy; print(\"[ok] OpenCV\", cv2.__version__, \"numpy\", numpy.__version__)'; \
      which espeak-ng >/dev/null && echo '[ok] espeak-ng (voz por un parlante USB en la Raspberry)' \
      || echo '[aviso] sin espeak-ng: sudo apt install espeak-ng  (solo hace falta si se conecta un parlante USB)'"
    ;;
  --bajar-ruedas)
    # En la PC con internet: ruedas para la Raspberry (aarch64, Python 3.12 de Ubuntu 24.04). ~60 MB en robot/ruedas_pi/
    mkdir -p "$ROOT/robot/ruedas_pi"
    python3 -m pip download --only-binary=:all: --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
      --python-version 3.12 --implementation cp --abi cp312 -d "$ROOT/robot/ruedas_pi" \
      "numpy<2.3" "opencv-contrib-python-headless>=4.8,<5" && ls -la "$ROOT/robot/ruedas_pi"
    ;;
  --instalar-offline)
    RUEDAS="${RUEDAS:-$ROOT/robot/ruedas_pi}"
    ls "$RUEDAS"/*.whl >/dev/null 2>&1 || { echo "[FALLA] no hay ruedas en $RUEDAS: antes, con internet, $0 --bajar-ruedas"; exit 1; }
    "${SSH[@]}" "$TARGET" "mkdir -p ~/ruedas_pi"
    "${SCP[@]}" "$RUEDAS"/*.whl "$TARGET:~/ruedas_pi/"
    # Con pip: instala desde las ruedas sin tocar la red. Sin pip (Ubuntu minimo): descomprime las ruedas en el
    # site-packages del usuario (una rueda es un zip; numpy y opencv no necesitan scripts de instalacion).
    "${SSH[@]}" "$TARGET" "cd ~/ruedas_pi && PV=\$(python3 -c 'import sys; print(f\"{sys.version_info[0]}.{sys.version_info[1]}\")') \
      && echo \"Python \$PV\" && ls *.whl | grep -q \"cp\${PV/./}\|abi3\" || echo '[aviso] las ruedas son para Python 3.12; el robot tiene otro' ; \
      if python3 -m pip --version >/dev/null 2>&1; then \
        python3 -m pip install --user --break-system-packages --no-index --find-links . numpy opencv-contrib-python-headless; \
      else \
        echo '[aviso] sin pip: descomprimo las ruedas en ~/.local/lib/python'\$PV'/site-packages'; \
        D=~/.local/lib/python\$PV/site-packages; mkdir -p \$D; for w in *.whl; do python3 -c \"import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])\" \$w \$D; done; \
      fi; \
      python3 -c 'import cv2, numpy; print(\"[ok] OpenCV\", cv2.__version__, \"numpy\", numpy.__version__, \"wechat\", hasattr(cv2, \"wechat_qrcode_WeChatQRCode\"))'"
    ;;
  --servicio)
    "${SCP[@]}" "$ROOT/robot/carrera.service" "$TARGET:/tmp/carrera.service"
    "${SSH[@]}" -t "$TARGET" "sudo install -m 644 /tmp/carrera.service /etc/systemd/system/carrera.service \
      && sudo systemctl daemon-reload && sudo systemctl enable --now carrera.service \
      && sleep 3 && systemctl --no-pager status carrera.service | head -5"
    echo "Listo: al encender el robot la carrera espera el BOTON 1. A quien buscar: $0 --buscar \"Nombre\""
    ;;
  --sin-servicio)
    "${SSH[@]}" -t "$TARGET" "sudo systemctl disable --now carrera.service; echo '[ok] servicio quitado'"
    ;;
  --buscar)
    NOMBRE="${1:-}"
    "${SSH[@]}" "$TARGET" "printf '%s\n' '$NOMBRE' > ~/$DIR/BUSCAR.txt && echo \"[ok] el robot buscara a: $NOMBRE\""
    ;;
  --resultado)
    "${SSH[@]}" "$TARGET" "cat ~/$DIR/RESULTADO.txt 2>/dev/null || echo '(todavia no hay resultado)'"
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
      setsid nohup python3 -u task-a/carrera-1/carrera.py 127.0.0.1 --no-gui --undock --video --video-fps 4 --video-sin-h264 \
        --buscar '$NOMBRE' --go-archivo GO --go-boton --resultado-archivo RESULTADO.txt ${CARRERA_ARGS:-} > $LOG 2>&1 < /dev/null & \
      sleep 2; pgrep -af '[c]arrera-1/carrera.py' >/dev/null && echo '[ok] carrera arrancada (en pausa)' || { tail -n 20 $LOG; exit 1; }"
    echo "Ahora: $0 --go (o el BOTON 1 del robot) da la salida; $0 --escuchar para oir el anuncio en la PC"
    ;;
  --go)
    "${SSH[@]}" "$TARGET" "touch ~/$DIR/GO && echo '[ok] salida dada'"
    ;;
  --start)
    TB4_IP="$IP" python3 "$ROOT/robot/control_carrera.py" "$IP" "$@"
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
