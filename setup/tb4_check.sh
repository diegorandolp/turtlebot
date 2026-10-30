#!/usr/bin/env bash
# Checklist automatico de conexion con un TurtleBot 4 (ROS 2 Jazzy).
# Version ejecutable de TROUBLESHOOTING.md seccion 3. Solo lee: no mueve ni configura el robot.
#
# Uso:
#   ./tb4_check.sh [IP_ROBOT]            # checks desde la PC (usa $TB4_IP si no das IP)
#   ./tb4_check.sh [IP_ROBOT] --robot    # ademas entra por SSH y revisa el robot por dentro
#
# Codigo de salida = numero de fallos (0 = todo bien).

IP=""; ROBOT=0
for a in "$@"; do
  case "$a" in
    --robot) ROBOT=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) IP="$a" ;;
  esac
done
IP="${IP:-$TB4_IP}"
if [ -z "$IP" ]; then
  echo "Falta la IP del robot: ./tb4_check.sh <IP_ROBOT>  (o usa antes source tb4_connect.sh)"
  exit 1
fi

FAILS=0
ok()   { echo "  [ok]    $1"; }
warn() { echo "  [AVISO] $1"; }
fail() { echo "  [FALLA] $1"; echo "          -> $2"; FAILS=$((FAILS + 1)); }
port_open() { timeout 3 bash -c ">/dev/tcp/$1/$2" 2>/dev/null; }

echo "== 1. Entorno de esta PC"
[ "$ROS_DISTRO" = "jazzy" ] || source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ "$ROS_DISTRO" = "jazzy" ]; then ok "ROS_DISTRO=jazzy"; else fail "ROS 2 Jazzy no encontrado" "README secciones 2-3"; fi
if [ -n "$ROS_DOMAIN_ID" ]; then ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"; else warn "ROS_DOMAIN_ID sin definir (vale 0). Debe ser el del robot"; fi
ok "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<por defecto: rmw_fastrtps_cpp>}"
[ -n "$ROS_STATIC_PEERS" ] && ok "ROS_STATIC_PEERS=$ROS_STATIC_PEERS"

echo "== 2. Red hacia $IP"
if ping -c 3 -W 2 "$IP" >/dev/null 2>&1; then ok "ping"; else fail "ping sin respuesta" "misma red? IP correcta? (TROUBLESHOOTING 1.4)"; fi
if port_open "$IP" 22; then ok "SSH (22) abierto"; else fail "SSH (22) cerrado" "robot arrancando o IP equivocada"; fi
if port_open "$IP" 8080; then ok "portal Create 3 (8080) abierto"; else warn "portal 8080 no responde (solo afecta a configurar el Create 3)"; fi

echo "== 3. Descubrimiento DDS"
ros2 daemon stop >/dev/null 2>&1
TOPICS=$(ros2 topic list -t --no-daemon --spin-time 8 2>/dev/null)
N=$(printf '%s\n' "$TOPICS" | grep -c '^/')
if [ "$N" -gt 2 ]; then
  ok "$N topicos visibles"
else
  fail "solo $N topicos (solo los de esta PC)" "ROS_DOMAIN_ID distinto o multicast bloqueado: source tb4_connect.sh $IP <DOMAIN_ID> (TROUBLESHOOTING 1.1)"
fi

has() { printf '%s\n' "$TOPICS" | grep -q "^$1 "; }
for t in /scan /odom /cmd_vel /oakd/rgb/preview/image_raw /battery_state /tf; do
  if has "$t"; then ok "existe $t"
  else
    case "$t" in
      /odom|/battery_state|/cmd_vel) fail "falta $t (lo publica/escucha el Create 3)" "TROUBLESHOOTING 1.2 y 1.3" ;;
      /scan) fail "falta /scan" "TROUBLESHOOTING 2.2" ;;
      /oakd/*) fail "falta $t" "TROUBLESHOOTING 2.1" ;;
      *) fail "falta $t" "bringup activo? (README seccion 10)" ;;
    esac
  fi
done

CMD_TYPE=$(printf '%s\n' "$TOPICS" | grep '^/cmd_vel ' | sed 's/.*\[\(.*\)\]/\1/')
case "$CMD_TYPE" in
  *TwistStamped) ok "/cmd_vel es TwistStamped (usa stamped:=true)" ;;
  "") ;;
  *) warn "/cmd_vel es $CMD_TYPE (se esperaba TwistStamped en Jazzy)" ;;
esac

echo "== 4. Llegan datos"
for t in /scan /odom /oakd/rgb/preview/image_raw; do
  has "$t" || continue
  MSG=$(timeout 10 ros2 topic echo --once --no-arr "$t" 2>/dev/null)
  if [ -n "$MSG" ]; then
    ok "dato recibido en $t"
    # Sin internet la Pi arranca con la hora mal en cada encendido (no tiene RTC)
    SEC=$(printf '%s\n' "$MSG" | grep -m1 -E '^ +sec:' | awk '{print $2}')
    if [ -n "$SEC" ]; then
      D=$((SEC - $(date +%s)))
      if [ "${D#-}" -gt 5 ]; then
        fail "reloj desfasado ${D}s en $t (stamp vs hora de esta PC)" "copia la hora de la PC al robot (TROUBLESHOOTING 2.4)"
      fi
    fi
  else
    case "$t" in
      /oakd/*) fail "sin datos en $t" "camara caida, o Wi-Fi sin ancho de banda: prueba .../compressed (TROUBLESHOOTING 2.1)" ;;
      /odom) fail "sin datos en /odom" "Create 3 sin publicar: espera 2-3 min o reinicialo (TROUBLESHOOTING 1.3)" ;;
      *) fail "sin datos en $t" "TROUBLESHOOTING 2.2" ;;
    esac
  fi
done

if [ "$ROBOT" -eq 1 ]; then
  echo "== 5. Dentro del robot (SSH ubuntu@$IP, puede pedir contrasena)"
  ssh -o ConnectTimeout=5 "ubuntu@$IP" 'bash -s' <<'EOF'
source /opt/ros/jazzy/setup.bash 2>/dev/null
[ -f /etc/turtlebot4/setup.bash ] && source /etc/turtlebot4/setup.bash
echo "  ROS_DOMAIN_ID robot = ${ROS_DOMAIN_ID:-<sin definir>}   RMW = ${RMW_IMPLEMENTATION:-<defecto>}"
echo "  turtlebot4.service  = $(systemctl is-active turtlebot4.service)"
if ping -c 2 -W 2 192.168.186.2 >/dev/null 2>&1; then echo "  [ok]    Pi -> Create 3 (192.168.186.2)"; else echo "  [FALLA] Pi no alcanza al Create 3 (usb0): TROUBLESHOOTING 1.3"; fi
if lsusb | grep -q 03e7; then echo "  [ok]    OAK-D en USB"; else echo "  [FALLA] OAK-D no aparece en lsusb: TROUBLESHOOTING 2.1"; fi
if [ -e /dev/RPLIDAR ]; then echo "  [ok]    /dev/RPLIDAR presente"; else echo "  [FALLA] /dev/RPLIDAR no existe: TROUBLESHOOTING 2.2"; fi
echo "  Ultimos errores del bringup:"
journalctl -u turtlebot4.service -n 200 --no-pager 2>/dev/null | grep -iE 'error|fail' | tail -5 | sed 's/^/    /'
EOF
  [ $? -eq 255 ] && fail "SSH fallo" "usuario ubuntu, contrasena por defecto turtlebot4"
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "Resultado: todo OK"; else echo "Resultado: $FAILS fallo(s). Detalle en TROUBLESHOOTING.md"; fi
exit "$FAILS"
