#!/usr/bin/env bash
# Configura la terminal actual para hablar con un TurtleBot 4 (ROS 2 Jazzy).
# Prueba el descubrimiento DDS y, si la red bloquea multicast, pasa solo a
# ROS_STATIC_PEERS. Guarda el resultado en ~/.tb4_robot para reutilizarlo.
#
# Uso (siempre con "source", para que las variables queden en tu terminal):
#   source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID> [auto|multicast|peers]
#   source tb4_connect.sh                # reconecta con el ultimo robot guardado
#   source tb4_connect.sh --off          # limpia la configuracion de esta terminal
#
# Para que cada terminal nueva arranque ya configurada (sin repetir pruebas):
#   echo '[ -f ~/.tb4_robot ] && source ~/.tb4_robot' >> ~/.bashrc

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "Ejecutalo con source:  source $0 <IP_ROBOT> <DOMAIN_ID>"
  exit 1
fi

_tb4_key_topics() {
  # Cuenta cuantos topicos clave del robot se ven (0-4).
  ros2 topic list --no-daemon --spin-time "${TB4_SPIN:-8}" 2>/dev/null \
    | grep -cE '^/(scan|odom|cmd_vel|oakd/rgb/preview/image_raw)$'
}

_tb4_connect() {
  local cfg="$HOME/.tb4_robot"

  if [ "$1" = "--off" ]; then
    unset ROS_STATIC_PEERS TB4_IP TB4_MODE
    export ROS_DOMAIN_ID=0
    ros2 daemon stop >/dev/null 2>&1
    echo "Configuracion TB4 limpiada (ROS_DOMAIN_ID=0, sin peers)."
    return 0
  fi

  local ip="$1" domain="$2" mode="${3:-auto}"
  if [ -z "$ip" ]; then
    if [ ! -f "$cfg" ]; then
      echo "No hay robot guardado. Uso: source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>"
      return 1
    fi
    # shellcheck disable=SC1090
    source "$cfg"
    ip="$TB4_IP"; domain="$ROS_DOMAIN_ID"; mode="${TB4_MODE:-auto}"
  fi

  if ! [[ "$domain" =~ ^[0-9]+$ ]] || [ "$domain" -gt 232 ]; then
    echo "DOMAIN_ID invalido: '$domain' (usa 0-101)."
    return 1
  fi
  [ "$domain" -gt 101 ] && echo "Aviso: ROS_DOMAIN_ID=$domain > 101 puede chocar con puertos de Linux."
  case "$mode" in auto|multicast|peers) ;; *) echo "Modo invalido: $mode"; return 1 ;; esac

  [ "$ROS_DISTRO" = "jazzy" ] || source /opt/ros/jazzy/setup.bash || return 1
  export ROS_DOMAIN_ID="$domain"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export TB4_IP="$ip"
  unset ROS_STATIC_PEERS

  echo "== TurtleBot 4 en $ip | ROS_DOMAIN_ID=$domain | modo $mode"

  if ping -c 2 -W 2 "$ip" >/dev/null 2>&1; then
    echo "[ok]   ping a $ip"
  else
    echo "[FALLA] ping a $ip: no hay ruta IP. Misma red? IP correcta? (TROUBLESHOOTING 1.1 y 1.4)"
    return 1
  fi

  local found=0 used=""
  ros2 daemon stop >/dev/null 2>&1
  if [ "$mode" != "peers" ]; then
    found=$(_tb4_key_topics); used="multicast"
    echo "[..]   descubrimiento multicast: $found/4 topicos clave"
  fi
  if [ "$found" -lt 4 ] && [ "$mode" != "multicast" ]; then
    # Multicast ausente o parcial (habitual en WSL y redes con aislamiento): probar unicast.
    export ROS_STATIC_PEERS="$ip"
    local found_peers
    found_peers=$(_tb4_key_topics)
    echo "[..]   descubrimiento con ROS_STATIC_PEERS=$ip: $found_peers/4 topicos clave"
    if [ "$found_peers" -ge "$found" ]; then found=$found_peers; used="peers"; fi
  fi
  [ "$used" = "multicast" ] && unset ROS_STATIC_PEERS
  export TB4_MODE="$used"

  if [ "$found" -eq 0 ]; then
    echo "[FALLA] ping OK pero DDS no ve el robot. Revisa ROS_DOMAIN_ID en el robot,"
    echo "        que el bringup este activo, y TROUBLESHOOTING 1.1. Diagnostico: ./tb4_check.sh --robot"
    return 1
  fi

  {
    echo "# Generado por tb4_connect.sh el $(date '+%F %T')"
    echo "export TB4_IP=$ip"
    echo "export TB4_MODE=$used"
    echo "export ROS_DOMAIN_ID=$domain"
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    [ "$used" = "peers" ] && echo "export ROS_STATIC_PEERS=$ip"
  } > "$cfg"

  if [ "$found" -eq 4 ]; then
    echo "[ok]   conectado ($used). Guardado en $cfg"
  else
    echo "[AVISO] conectado ($used) pero faltan topicos clave. Guardado en $cfg. Ejecuta: ./tb4_check.sh"
  fi
}

_tb4_connect "$@"
