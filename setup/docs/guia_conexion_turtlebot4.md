# Guía de conexión WSL 2 · TurtleBot 4 (ROS 2 Jazzy)

← [Volver al README (trabajo diario)](../README.md) · [Tarjeta offline](tarjeta_offline_lab.md) · [TROUBLESHOOTING](../TROUBLESHOOTING.md)

Caso documentado de un robot concreto (hostname `turtlebot4`, laboratorio 3): dejarlo accesible
por varias vías y preparar una PC **Windows + WSL 2** para conectarse a él. Los valores (IP,
`ROS_DOMAIN_ID=67`, red `Lab_Computech_3_5G`) son los de ese robot; en otro laboratorio la red
sigue el patrón `Lab_Computech_<X>_5G`.

**Usa esta guía si:** trabajas desde Windows con WSL 2, o quieres dejar un robot con red de
respaldo y acceso por cable antes de sacarlo del laboratorio.

Convenciones: **[PC]** = tu PC de desarrollo (Windows + WSL 2) · **[ROBOT]** = terminal SSH en la Raspberry Pi.

| Dato (verificado el 2026-09-30) | Valor |
|---|---|
| Sistema y ROS | Ubuntu 24.04 + ROS 2 **Jazzy**, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, **`ROS_DOMAIN_ID=67`** |
| Wi-Fi guardado en el robot | Solo `Lab_Computech_3_5G` (robot: `192.168.0.104`, DHCP) |
| Contraseña del router | `<WIFI_PASSWORD>` (dada por el docente) |
| Ethernet / Create 3 | `eth0`: `192.168.185.3/24` · Create 3 (por `usb0`): `192.168.186.2` |
| PC de partida | Otra red (`L67`, `10.247.190.242`) y **sin ninguna distro WSL instalada** |
| Bringup | `turtlebot4.service` lo arranca solo |

> Regla de oro: **nunca borres `Lab_Computech_<X>_5G`** del robot (aquí `Lab_Computech_3_5G`). Solo agrega redes nuevas.

---

## Índice

1. [Antes de dejar el laboratorio](#1-antes-de-dejar-el-laboratorio-con-el-ssh-que-ya-tienes)
2. [Preparar tu PC (Windows + WSL 2)](#2-preparar-tu-pc-windows--wsl-2)
3. [Conectar (elige UNA vía)](#3-conectar-elige-una-vía)
4. [Si algo sale mal](#4-si-algo-sale-mal)
5. [Prueba final](#5-prueba-final)

---

## 1. Antes de dejar el laboratorio (con el SSH que ya tienes)

### 1.1 Aplicar el cambio de ahorro de energía (cortará el SSH unos segundos)
```bash
# [ROBOT]
sudo nmcli connection up netplan-wlan0-Lab_Computech_3_5G
nmcli -f 802-11-wireless.powersave connection show netplan-wlan0-Lab_Computech_3_5G
```
Debe decir `2 (disable)`. Vuelve a entrar por SSH si se corta.

### 1.2 Agregar una red de respaldo SIN cambiar de red ahora
Reemplaza `NOMBRE` y `CLAVE` (la clave la escribes tú, no la compartas en el chat):
```bash
sudo nmcli connection add type wifi ifname wlan0 con-name respaldo \
  ssid "NOMBRE" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "CLAVE" \
  connection.autoconnect yes connection.autoconnect-priority 5 \
  802-11-wireless.powersave 2
nmcli connection show
```
- `add` solo guarda la red; no te desconecta.
- Prioridad más baja que la del laboratorio: en el lab usa esa, fuera usa `respaldo`.
- Sirve el hotspot del celular (2.4 o 5 GHz). Evita redes con "aislamiento de clientes" (invitados, universidad, algunos hotspots): ahí ROS no verá al robot.

### 1.3 Confirmar la vía por cable (plan B sin Wi-Fi)
```bash
nmcli -f ipv4.method,ipv4.addresses connection show netplan-eth0
```
Si muestra `manual` y `192.168.185.3/24`, puedes conectar PC ↔ robot con un cable Ethernet directo, sin router ni Wi-Fi (ver §3).

### 1.4 Anotar el nombre del robot
```bash
hostname; hostname -I
```

---

## 2. Preparar tu PC (Windows + WSL 2)

> Versión resumida para este robot. La guía completa paso a paso (requisitos, firewall,
> perfil de red, dónde clonar el repo) está en [README § 3.1](../README.md#31-wsl-2-en-windows-11).

### 2.1 Instalar Ubuntu 24.04 en WSL (PowerShell normal; puede pedir reinicio)
```powershell
wsl --install -d Ubuntu-24.04
```

### 2.2 Red espejo para que DDS/ROS vea al robot
Crea/edita `C:\Users\mando\.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
Luego `wsl --shutdown` y vuelve a abrir Ubuntu. (WSL en modo NAT normal no recibe el multicast de ROS.)

### 2.3 Instalar ROS 2 Jazzy dentro de WSL
```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-ros-base
```

### 2.4 Variables (mismas que el robot) — añadir al final de `~/.bashrc`
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
```bash
source ~/.bashrc
```

---

## 3. Conectar (elige UNA vía)

| Vía | Cuándo | Cómo |
|---|---|---|
| **A. Mismo Wi-Fi** | Estás en el lab o el robot ya tiene `respaldo` | Conecta la PC a la MISMA red. `ssh ubuntu@192.168.0.104` (o la IP de `hostname -I`) |
| **B. Cable directo** | No hay Wi-Fi común | Cable Ethernet PC ↔ robot. En Windows pon a tu adaptador Ethernet IP fija `192.168.185.10`, máscara `255.255.255.0`, sin puerta de enlace. `ssh ubuntu@192.168.185.3` |
| **C. Hotspot del celular** | Sin router | Conecta PC y robot (`respaldo`) al hotspot. Busca la IP del robot en la lista de dispositivos del celular |

### 3.1 Verificación (en este orden)
```bash
ping <IP del robot>
ssh ubuntu@<IP del robot>
ros2 node list          # desde WSL: debe listar /oakd, /turtlebot4_node, etc.
ros2 topic echo /odom --once
```

### 3.2 Conexión SSH cómoda (opcional): `C:\Users\mando\.ssh\config`
```
Host tb4
  HostName 192.168.0.104
  User ubuntu
  ServerAliveInterval 15
  ServerAliveCountMax 4
```

---

## 4. Si algo sale mal

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `ping` al robot falla | Redes distintas, IP cambió | Misma red; en el robot `hostname -I`; usa la vía B (cable) |
| Ping OK pero SSH rechaza/timeout | SSH caído o IP equivocada | `ssh -v ubuntu@IP`; revisa que sea la IP correcta |
| SSH OK pero `ros2 node list` vacío | `ROS_DOMAIN_ID` distinto o multicast bloqueado | `echo $ROS_DOMAIN_ID` debe ser **67**; en WSL usa `mirrored`; evita Wi-Fi con aislamiento; prueba la vía B |
| Topics aparecen pero sin datos / muy lento | Wi-Fi saturado o imágenes por red | Usa cable; no suscribas `/oakd/rgb/preview/image_raw` por Wi-Fi flojo |
| SSH se congela o se cae a ratos | Ahorro de energía del Wi-Fi | Verifica que `powersave` sea `2`; confirma señal: `nmcli -f SSID,SIGNAL dev wifi` (>50) |
| El robot no aparece en ninguna red | Wi-Fi perdido | Conecta por cable (B) y repara; o teclado + monitor micro-HDMI a la Pi |
| Perdiste el acceso por completo | Borraste/cambiaste la red | Cable Ethernet 192.168.185.3 o monitor+teclado; el comando `turtlebot4-setup` (menú en el robot) reconfigura Wi-Fi/ROS |
| Create3 no responde | usb0 caído | En el robot: `ping 192.168.186.2`; si falla, revisa el cable USB del Create3 y reinicia el bringup: `sudo systemctl restart turtlebot4.service` |
| El robot se mueve solo o ignora tu script | Teleop por defecto en el robot (`teleop_twist_joy_node`, `joy_linux_node`) | Si usas el mando en la PC, no dejes otro mando conectado a la Pi; coordina quién publica en `/cmd_vel` |
| Nada de lo anterior | — | Reinicia el robot: `sudo reboot`, espera ~1 min y repite la verificación |

### 4.1 Comandos de diagnóstico útiles (solo lectura)
```bash
nmcli dev status; nmcli connection show
cat /proc/net/wireless
systemctl status turtlebot4.service --no-pager | head
journalctl -u turtlebot4.service -n 40 --no-pager
ros2 node list; ros2 topic list
```

---

## 5. Prueba final

1. Reinicia el robot (`sudo reboot`).
2. Sin tocar nada, espera ~1 min.
3. Desde la PC: `ping` → `ssh` → `ros2 node list` → `ros2 topic echo /odom --once`.
4. Repite en la otra red (respaldo) y por cable.
Si las 3 vías funcionan, el robot queda "pulido" en conectividad.

Después de esto se retoma el plan de evasión de obstáculos
(`_legacy/superpowers/plans/2026-07-09-turtlebot4-obstacle-avoidance.md`).

---

← [Volver al README](../README.md)
