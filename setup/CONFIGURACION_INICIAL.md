# Configuración inicial · TurtleBot 4 (ROS 2 Jazzy)

← [Volver al README (trabajo diario)](README.md) · [TROUBLESHOOTING](TROUBLESHOOTING.md)

Todo lo que **no viene configurado** o hay que volver a hacer si el robot se reseteó de
fábrica, se reflasheó la tarjeta SD o está desconfigurado. Válido para TurtleBot 4 Lite y
Standard con **ROS 2 Jazzy / Ubuntu 24.04**.

**Usa esta guía si:** el robot levanta la Wi-Fi `turtlebot4` en vez de conectarse al
laboratorio, faltan paquetes o sensores, el `ROS_DOMAIN_ID` del robot no es el
asignado, o el bringup no arranca solo.

Convenciones: **[PC]** = tu PC de desarrollo (Ubuntu nativo, WSL o VM) · **[ROBOT]** = terminal SSH en la Raspberry Pi.

| Marcador | Significado |
|---|---|
| `<IP_ROBOT>` | IP que el robot obtiene en la red del laboratorio |
| `<DOMAIN_ID>` | `ROS_DOMAIN_ID` asignado a este robot (0–101, único en el laboratorio) |
| `<WIFI_SSID>` | Red del laboratorio: `Lab_Computech_<X>_5G`, donde `<X>` es el número del laboratorio (p. ej. `Lab_Computech_3_5G`) |
| `<WIFI_PASSWORD>` | Contraseña del router del laboratorio (dada por el docente) |

---

## Índice

1. [Red desde cero](#1-red-desde-cero)
2. [Create 3: configuración por web (puerto 8080)](#2-create-3-configuración-por-web-puerto-8080)
3. [`ROS_DOMAIN_ID` del robot](#3-ros_domain_id-del-robot)
4. [Instalación base en el robot](#4-instalación-base-en-el-robot)
5. [Reglas udev (OAK-D y RPLIDAR)](#5-reglas-udev-oak-d-y-rplidar)
6. [Servicio persistente `turtlebot4.service`](#6-servicio-persistente-turtlebot4service)
7. [Extras para un robot nuevo](#7-extras-para-un-robot-nuevo)
8. [Verificación final](#8-verificación-final)

---

## 1. Red desde cero

### 1.1 Conexión al punto de acceso `turtlebot4`

Un robot sin red configurada levanta su propia Wi-Fi.

1. Enciende el robot y espera ~1 minuto.
2. Conecta la PC a la red **`turtlebot4`** (contraseña: `turtlebot4`).
3. Entra por SSH:
   ```bash
   ssh ubuntu@10.42.0.1
   # contraseña por defecto: turtlebot4
   ```

### 1.2 Wi-Fi del laboratorio con `turtlebot4-setup`

```bash
# [ROBOT]
turtlebot4-setup
```

En **Wi-Fi Setup** introduce los parámetros de la red:

| Campo | Valor |
|---|---|
| Wi-Fi Mode | `Client` |
| SSID | `<WIFI_SSID>` |
| Password | `<WIFI_PASSWORD>` |
| REG_DOMAIN | Código de tu país (p. ej. `PE`) |
| Band | `5GHz` (o `2.4GHz` según la red) |
| DHCP | `True` |

Guarda y ejecuta **Apply Settings**. El robot sale del modo AP y la sesión SSH se corta.
Conecta la PC a `<WIFI_SSID>` y busca la IP nueva:

- **Standard:** aparece en la pantalla.
- **Lite:** panel del router, `ssh ubuntu@turtlebot4.local`, o `arp -a` en la PC
  buscando una MAC de Raspberry Pi (`d8:3a:dd`, `dc:a6:32`, `e4:5f:01`).

> ⚠️ **No borres la red del laboratorio** de un robot ya configurado. Si te equivocas
> de contraseña, el robot no vuelve a levantar el AP; tendrás que entrar por cable
> (§1.4) o con monitor y teclado.

### 1.3 Redes adicionales (respaldo, hotspot)

Para usar el robot fuera del laboratorio sin perder la red principal, **agrega** una
conexión con menor prioridad:

```bash
# [ROBOT]
sudo nmcli connection add type wifi ifname wlan0 con-name respaldo \
  ssid "<SSID>" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<CLAVE>" \
  connection.autoconnect yes connection.autoconnect-priority 5 \
  802-11-wireless.powersave 2
nmcli connection show
```

`add` solo guarda la red: no te desconecta. Evita redes con aislamiento de clientes
(invitados, Wi-Fi universitaria abierta): el ping puede funcionar y ROS 2 no.

### 1.4 Acceso por cable (plan B)

La imagen del TurtleBot 4 trae `eth0` con IP fija **`192.168.185.3/24`**:

1. Cable Ethernet directo PC ↔ Raspberry Pi.
2. En la PC: IP `192.168.185.10`, máscara `255.255.255.0`, sin puerta de enlace.
3. `ssh ubuntu@192.168.185.3`

Comprueba que sigue así:

```bash
# [ROBOT]
nmcli -f ipv4.method,ipv4.addresses connection show netplan-eth0
```

### 1.5 Desactivar el ahorro de energía de la Wi-Fi

Sin este ajuste, las sesiones SSH se congelan a ratos y se pierden mensajes DDS.

```bash
# [ROBOT]
nmcli connection show                         # nombre de la conexión Wi-Fi
sudo nmcli connection modify "<CONEXION>" 802-11-wireless.powersave 2
sudo nmcli connection up "<CONEXION>"         # corta el SSH unos segundos
nmcli -f 802-11-wireless.powersave connection show "<CONEXION>"   # debe decir 2 (disable)
```

---

## 2. Create 3: configuración por web (puerto 8080)

La Raspberry Pi reenvía el portal web del Create 3, que está en `192.168.186.2` por `usb0`:

```
http://<IP_ROBOT>:8080
```

Si el puerto 8080 no responde, abre un túnel: `ssh -L 8080:192.168.186.2:80 ubuntu@<IP_ROBOT>`
y entra a `http://localhost:8080`.

### 2.1 Parámetros de red y ROS

En **Application → Configuration**:

| Parámetro | Valor |
|---|---|
| `ROS_DOMAIN_ID` | `<DOMAIN_ID>` (el mismo que la Pi y la PC) |
| RMW | `rmw_fastrtps_cpp` |
| Namespace | vacío (salvo despliegue multi-robot con namespaces) |

Guarda y ejecuta **Application → Restart Application**, o reinicia el robot:

```bash
# [ROBOT]
sudo reboot
```

### 2.2 Firmware

En **Update**, comprueba que el firmware del Create 3 corresponde a **Jazzy**. Con
firmware de otra distribución, los tópicos del Create 3 (`/odom`, `/battery_state`)
pueden aparecer pero no funcionar, o no aparecer. Actualiza desde la misma página con
el robot cargando.

### 2.3 Comprobar la conexión Pi ↔ Create 3

```bash
# [ROBOT]
ping -c 3 192.168.186.2
```

Si falla, revisa el cable USB-C entre la Pi y el Create 3.

---

## 3. `ROS_DOMAIN_ID` del robot

El bringup corre como servicio y lee la configuración del robot, **no** un `export` de tu
sesión SSH. Cámbialo con el asistente:

```bash
# [ROBOT]
turtlebot4-setup
```

**ROS Setup → Bash Setup**:

| Variable | Valor |
|---|---|
| `ROS_DOMAIN_ID` | `<DOMAIN_ID>` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| `ROBOT_NAMESPACE` | vacío (salvo multi-robot con namespaces) |

Guarda y vuelve al menú principal → **Apply Settings**. Reinicia el Create 3 desde el
mismo asistente (o desde el portal, §2.1) y espera la melodía de arranque.

Verifícalo en una **sesión SSH nueva**:

```bash
# [ROBOT]
echo $ROS_DOMAIN_ID
cat /etc/turtlebot4/setup.bash     # configuración persistente que usa el servicio
```

Comprueba también el valor en el portal del Create 3 (§2.1). Los tres valores (PC, Pi y
Create 3) deben coincidir.

> El rango técnico permite más valores, pero usa **0–101**: los IDs altos pueden chocar
> con puertos efímeros de Linux.

---

## 4. Instalación base en el robot

La imagen oficial trae casi todo. Esta sección repara o completa un robot "limpio".

### 4.1 Repositorio de ROS 2 (clave GPG y `sources.list.d`)

```bash
# [ROBOT]
sudo apt update
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Si `apt update` da error de firma (`EXPKEYSIG`/`NO_PUBKEY`), usa el paquete oficial
`ros2-apt-source`: [TROUBLESHOOTING § 4.1](TROUBLESHOOTING.md#41-error-de-firma-gpg-en-apt-update).

### 4.2 Nodos de prueba de comunicación

```bash
# [ROBOT]
sudo apt install -y ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py
```

> `ros-$ROS_DISTRO-...` solo funciona si ya hiciste `source /opt/ros/jazzy/setup.bash`;
> escribir `ros-jazzy-...` evita el problema.

### 4.3 Sensores y bringup

```bash
# [ROBOT]
sudo apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-depthai-ros \
  ros-jazzy-irobot-create-nodes \
  ros-jazzy-irobot-create-msgs \
  ros-jazzy-turtlebot4-msgs \
  ros-jazzy-turtlebot4-description \
  ros-jazzy-turtlebot4-bringup
```

Apaga y vuelve a encender el robot. Al reconectarte, verifica con
[README § 6](README.md#6-bringup-y-verificación-de-tópicos).

---

## 5. Reglas udev (OAK-D y RPLIDAR)

Normalmente ya vienen instaladas. Aplícalas si un sensor no aparece tras el bringup.

### 5.1 OAK-D no detectada

Comprueba primero el hardware:

```bash
# [ROBOT]
lsusb | grep 03e7        # debe aparecer el dispositivo Intel Movidius / Luxonis
```

Si aparece en `lsusb` pero el driver no puede abrirla (`X_LINK_ERROR`, `insufficient
permissions`), instala la regla de Luxonis:

```bash
# [ROBOT]
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Desconecta y reconecta la cámara (en un puerto **USB 3**, azul) y reinicia el servicio (§6).

### 5.2 RPLIDAR sin `/dev/RPLIDAR`

El bringup espera el enlace `/dev/RPLIDAR`. El RPLIDAR A1 usa un adaptador CP2102
(`10c4:ea60`):

```bash
# [ROBOT]
ls -l /dev/RPLIDAR /dev/ttyUSB*
```

Si falta el enlace:

```bash
# [ROBOT]
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="RPLIDAR"' \
  | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Si aparece `Permission denied` en `/dev/ttyUSB0`:

```bash
sudo usermod -aG dialout ubuntu    # cierra la sesión y vuelve a entrar
```

---

## 6. Servicio persistente `turtlebot4.service`

El servicio lanza el bringup (robot, cámara y LiDAR) en cada arranque.

```bash
# [ROBOT]
systemctl status turtlebot4.service --no-pager | head -5
journalctl -u turtlebot4.service -n 50 --no-pager      # logs recientes
```

| Acción | Comando |
|---|---|
| Reiniciar (tras cambiar configuración o reconectar un sensor) | `sudo systemctl restart turtlebot4.service` |
| Activar el arranque automático | `sudo systemctl enable --now turtlebot4.service` |
| Desactivarlo (para lanzar el bringup a mano) | `sudo systemctl disable --now turtlebot4.service` |

Si el servicio **no existe**, instálalo desde el asistente:
`turtlebot4-setup` → **ROS Setup → Robot Upstart → Install**.

> No combines servicio activo y `ros2 launch turtlebot4_bringup ...` a mano: duplica
> nodos y provoca comportamientos erráticos.

---

## 7. Extras para un robot nuevo

Ajustes que la imagen de fábrica no trae y que conviene hacer una vez:

| Ajuste | Por qué | Cómo **[ROBOT]** |
|---|---|---|
| **Nombre único** | Varios robots llamados `turtlebot4` hacen ambiguo `turtlebot4.local` | `sudo hostnamectl set-hostname tb4-<grupo>`, luego reiniciar |
| **Cambiar la contraseña por defecto** | `turtlebot4` es pública; cualquiera en la red puede entrar | `passwd` y comunicar la nueva al equipo |
| **Clave SSH de la PC** | Entrar sin contraseña; la necesitan `tb4_check.sh --robot` y los scripts | **[PC]** `ssh-keygen -t ed25519 && ssh-copy-id ubuntu@<IP_ROBOT>` |
| **Hora (NTP)** | La Pi no tiene RTC; con la hora mal, tf y la sincronización de sensores fallan | Router del laboratorio sin internet: copiar la hora de la PC **en cada sesión**, ver [TROUBLESHOOTING § 2.4](TROUBLESHOOTING.md#24-reloj-desfasado-tf--fusión-de-sensores). Con internet: `sudo timedatectl set-ntp true` |
| **Wi-Fi sin ahorro de energía** | Evita cortes de SSH y de DDS | §1.5 |
| **Firmware del Create 3** | Debe ser el de Jazzy | §2.2 |
| **Mando Bluetooth** | Un mando emparejado publica en `/cmd_vel` y compite con tu teleop | `turtlebot4-setup` → Bluetooth Setup; desempareja si no se usa |
| **Actualizar paquetes** | Correcciones del bringup y los drivers | `sudo apt update && sudo apt upgrade -y` |
| **Registro del laboratorio** | Saber qué IP e ID tiene cada robot | Anota hostname, MAC, IP, `<DOMAIN_ID>` y grupo |

---

## 8. Verificación final

Con todo aplicado, reinicia el robot (`sudo reboot`), espera ~1 minuto y desde la **PC**:

```bash
source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>
./tb4_check.sh <IP_ROBOT> --robot
```

O a mano:
1. `ping <IP_ROBOT>` y `ssh ubuntu@<IP_ROBOT>`.
2. Talker en la PC y listener en el robot ([README § 6.1](README.md#61-checklist-rápido-de-comunicación)).
3. `ros2 topic list`: deben aparecer `/scan`, `/odom`, `/cmd_vel`, `/oakd/rgb/preview/image_raw`.
4. `ros2 topic info /cmd_vel` debe indicar `TwistStamped`.

Si todo pasa, el robot está listo: vuelve al [README](README.md) para el trabajo diario.

---

← [Volver al README](README.md)
