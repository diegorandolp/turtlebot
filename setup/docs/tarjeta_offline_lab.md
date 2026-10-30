# Tarjeta offline del laboratorio · TurtleBot 4 (ROS 2 Jazzy)

← [Volver al README (trabajo diario)](../README.md) · [Guía de conexión WSL 2](guia_conexion_turtlebot4.md) · [TROUBLESHOOTING](../TROUBLESHOOTING.md)

Ficha de bolsillo para conectar una laptop (Windows + WSL 2) al robot en la red del laboratorio,
que **no tiene internet**. Las redes del laboratorio siguen el patrón `Lab_Computech_<X>_5G`;
los valores de abajo son los del robot del laboratorio 3.

**Usa esta guía si:** estás en el laboratorio sin internet y necesitas los pasos mínimos para
conectar, o un recordatorio rápido de qué hacer si falla.

Convenciones: **[PC]** = tu laptop (Windows + WSL 2) · **[ROBOT]** = terminal SSH en la Raspberry Pi.

| Dato | Valor |
|---|---|
| Robot | **TurtleBot 4 Lite** (sin pantalla OLED), usuario `ubuntu` |
| Wi-Fi | `Lab_Computech_3_5G` (patrón general `Lab_Computech_<X>_5G`) |
| Contraseña del router | `<WIFI_PASSWORD>` (dada por el docente) |
| IP habitual | `192.168.0.104` (DHCP, puede cambiar) |
| ROS | ROS 2 Jazzy, `ROS_DOMAIN_ID=67` |
| MAC Wi-Fi | `d8:3a:dd:7e:5c:9c` |
| Ethernet / Create 3 | `192.168.185.3` · `192.168.186.2` (solo visible desde el robot) |

---

## Índice

1. [Estado de la preparación de la laptop](#1-estado-de-la-preparación-de-la-laptop-2026-09-30)
2. [Repetir la preparación (con internet)](#2-repetir-la-preparación-con-internet)
3. [En el laboratorio (sin internet)](#3-en-el-laboratorio-sin-internet)
4. [Si falla](#4-si-falla)
5. [Reglas y notas](#5-reglas-y-notas)

---

## 1. Estado de la preparación de la laptop (2026-09-30)

| Paso | Estado |
|---|---|
| WSL2 instalado (v2.6.1) | ✅ |
| `C:\Users\mando\.wslconfig` con `networkingMode=mirrored` | ✅ |
| Ubuntu-24.04 en WSL, usuario **root** (sin clave) | ✅ |
| ROS 2 Jazzy (`ros-jazzy-ros-base`), pygame 2.5.2, pytest | ✅ |
| Variables `ROS_DOMAIN_ID=67`, RMW, `source` en `/root/.bashrc` | ✅ (las añade el script de instalación; verificar con §3 paso 2) |
| Firewall Hyper-V (`Set-NetFirewallHyperVVMSetting`, requiere PowerShell **administrador**) | ⚠️ Lo hace el usuario; confirmar que no dio "Acceso denegado" |
| Robot: `powersave` del Wi-Fi en 2 | ✅ guardado (se aplica al reconectar el Wi-Fi) |

---

## 2. Repetir la preparación (con internet)

1. PowerShell admin: `wsl --install -d Ubuntu-24.04 --no-launch`
2. `C:\Users\mando\.wslconfig`:
   ```
   [wsl2]
   networkingMode=mirrored
   ```
   luego `wsl --shutdown`
3. PowerShell **admin**:
   `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`
4. ROS en WSL (como root, sin `sudo`): script `install_ros.sh` del scratchpad de la sesión, o los comandos:
   ```
   apt update && apt install -y software-properties-common curl
   add-apt-repository -y universe
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
   apt update && apt install -y ros-jazzy-ros-base python3-pygame python3-pytest
   ```
5. Al final de `~/.bashrc`:
   ```
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=67
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

---

## 3. En el laboratorio (sin internet)

1. Conecta la laptop a **Lab_Computech_3_5G** (en general `Lab_Computech_<X>_5G`; o la 2.4 GHz del mismo router) con la contraseña del router `<WIFI_PASSWORD>`. Windows dirá "sin internet": ignóralo.
   Configuración > Red > Wi-Fi > la red > perfil **Privada**.
2. Abre WSL: en PowerShell `wsl -d Ubuntu-24.04 -u root`
   Comprueba: `echo $ROS_DOMAIN_ID` → `67` y `ip -br a` → una IP `192.168.0.x`.
3. `ping 192.168.0.104`. Si no responde, busca la IP real:
   - PowerShell: `arp -a` (MAC del robot empieza con `d8-3a-dd`)
   - o el panel del router, o `ssh ubuntu@turtlebot4.local`
4. `ssh ubuntu@192.168.0.104` (clave del robot). Sal con `exit` antes del paso 5.
5. **Clave en esta red: el multicast NO pasa.** En WSL usa siempre (ya está en `~/.bashrc` si lo añadiste):
   `export ROS_STATIC_PEERS=192.168.0.104` (si la IP del robot cambia, ajusta aquí) y `ros2 daemon stop`.
   Sin esto `ros2 node list` sale vacío. Verificado funcionando el 2026-09-30.
   Desde WSL (no dentro del SSH): `ros2 node list` → debe listar `/oakd`, `/turtlebot4_node`, `/rplidar_composition`…
   `ros2 topic echo /odom --once` → debe mostrar datos.
   Topics útiles: `/scan` (lidar), `/odom`, `/imu`, `/cmd_vel`.

Opcional, `C:\Users\mando\.ssh\config`:
```
Host tb4
  HostName 192.168.0.104
  User ubuntu
  ServerAliveInterval 15
  ServerAliveCountMax 4
```
Con eso: `ssh tb4`.

---

## 4. Si falla

| Síntoma | Haz |
|---|---|
| Ping falla | Laptop y robot no están en la misma red; revisa SSID y busca la IP con `arp -a` |
| Ping OK, SSH no | IP equivocada o robot arrancando (espera 1 min) |
| SSH OK, `ros2 node list` vacío | **Primero:** `export ROS_STATIC_PEERS=<IP del robot>; ros2 daemon stop` (era la causa real). Luego `echo $ROS_DOMAIN_ID` debe ser 67; confirma `.wslconfig` mirrored + `wsl --shutdown`; red en perfil Privada; firewall Hyper-V (§2 paso 3); prueba `ros2 daemon stop` y repetir |
| Nodos y `/scan` OK pero `/odom` y `/battery_state` se cuelgan (datos del Create3) | Verificado 2026-09-30: tras reiniciar solo la Pi, el Create3 tarda en volver a publicar. Espera 2-3 min; si no, reinicia la app del Create3 (túnel `ssh -L 8080:192.168.186.2:80 tb4` y `http://localhost:8080` > Application > Restart) o apagado completo (botón central del Create3 ~10 s). Luego repetir la hora de la Pi |
| Topics sin datos / lento | No uses la cámara por Wi-Fi; usa `/scan` y `/odom`; acércate al router |
| Nada funciona | Cable Ethernet laptop↔robot: laptop IP `192.168.185.10/24` sin gateway, `ssh ubuntu@192.168.185.3` |
| SSH se congela | En el robot: `nmcli -f 802-11-wireless.powersave connection show netplan-wlan0-Lab_Computech_3_5G` debe dar 2 |
| Último recurso | `sudo reboot` al robot, espera 1 min, repite §3 |
| Perdiste acceso al robot | Cable Ethernet, o monitor micro-HDMI + teclado en la Pi; `turtlebot4-setup` reconfigura Wi-Fi/ROS |

---

## 5. Reglas y notas

- **Reloj del robot desfasado**: `/odom` marca julio 2025 (sin NTP). Pendiente sincronizarlo cuando el robot tenga internet (`sudo timedatectl set-ntp true` o fijarlo a mano con `sudo date -s`).
- Hora: la Pi se corrige con `ssh -t tb4 "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"`, pero el **Create3 conserva su propio reloj en 2025** (`/odom` y `/battery_state` marcan 1753...). Pendiente, solo importa para tf/fusión.
- `/scan` ~7.5 Hz estable; batería ~100 % (16.4 V) el 2026-09-30.
- La laptop estaba en `UTEC-Alumnos 3` (perfil Público) y aun así funcionó con `ROS_STATIC_PEERS`.
- No subir velocidades máximas (0.1 m/s lineal, 1.0 rad/s angular) sin decidirlo tú.
- En el robot corre `teleop_twist_joy_node`: evita que dos cosas publiquen a `/cmd_vel` a la vez.
- No borres `Lab_Computech_<X>_5G` (aquí `Lab_Computech_3_5G`) del robot; solo añade redes nuevas.
- El Lite no tiene pantalla: la IP se busca por red (§3 paso 3).
- WSL entra como root (sin clave); no hace falta `sudo`.
- Plan de evasión pendiente: `_legacy/superpowers/plans/2026-07-09-turtlebot4-obstacle-avoidance.md`.

---

← [Volver al README](../README.md)
