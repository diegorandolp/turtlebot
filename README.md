# Guía de trabajo diario · TurtleBot 4 Lite (ROS 2 Jazzy)

Guía práctica para trabajar con el **TurtleBot 4 Lite** en el laboratorio de
Computer Vision, con **ROS 2 Jazzy sobre Ubuntu 24.04 Noble**. Supone el escenario normal: el robot
**ya está configurado y conectado a la red del laboratorio** (`Lab_Computech_<X>_5G`).

**Usa esta guía si:** vas a preparar tu PC por primera vez o a trabajar con un robot que ya
funciona en el laboratorio.

<table>
<tr>
<td>🛠️ <b><a href="CONFIGURACION_INICIAL.md">CONFIGURACION_INICIAL.md</a></b><br>
Robot de fábrica, reseteado o desconfigurado (red, Create 3, paquetes, udev, servicio).</td>
<td>🩺 <b><a href="TROUBLESHOOTING.md">TROUBLESHOOTING.md</a></b><br>
Diagnóstico por síntomas, errores frecuentes y checklist paso a paso.</td>
<td>📶 <b><a href="docs/tarjeta_offline_lab.md">tarjeta_offline_lab.md</a></b><br>
Ficha de referencia rápida y checklist para trabajar en la red sin internet.</td>
</tr>
</table>

> 🤖 **Grupo 3:** los datos y pasos de conexión de nuestro robot (IP `192.168.0.103`, `ROS_DOMAIN_ID=3`) están en [docs/robot_grupo3.md](docs/robot_grupo3.md).

> **Base:** Adaptado sobre la guía de Luis Cortijo ([requeerimientos_turtlebot4](https://github.com/LuisEnriqueCortijoGonzales/requeerimientos_turtlebot4)) con optimizaciones probadas en el laboratorio (WSL 2, FastDDS Unicast, sincronización horaria sin internet y scripts propios). Manual oficial: [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/).

Convenciones: **[PC]** = tu PC de desarrollo (Ubuntu nativo, WSL o VM) · **[ROBOT]** = terminal SSH en la Raspberry Pi.

| Marcador | Significado |
|---|---|
| `<IP_ROBOT>` | IP del robot en la red del laboratorio |
| `<DOMAIN_ID>` | `ROS_DOMAIN_ID` de **tu** robot (uno distinto por equipo, entre 0 y 101) |
| `<WIFI_SSID>` | Red del laboratorio: `Lab_Computech_<X>_5G`, donde `<X>` es el número del laboratorio (p. ej. `Lab_Computech_3_5G`) |
| `<WIFI_PASSWORD>` | Contraseña del router del laboratorio: `Computech2025!` |

---

## Índice

**Parte I: Preparar tu entorno (una sola vez, con internet)**
- [¿Qué entorno usas?](#parte-i-preparar-tu-entorno-una-sola-vez-con-internet) (Ubuntu nativo como estándar; WSL 2, VM o Mac)
- [1. Instalar ROS 2 Jazzy](#1-instalar-ros-2-jazzy)
- [2. Instalar paquetes del TurtleBot 4 y visión](#2-instalar-paquetes-del-turtlebot-4-y-visión)
- [3. Configuración para entornos virtuales o Windows](#3-configuración-para-entornos-virtuales-o-windows) ([WSL 2 paso a paso](#31-wsl-2-en-windows-11) / VirtualBox)
- [4. Variables de entorno](#4-variables-de-entorno)

**Parte II: Flujo en el laboratorio (cada sesión)**
- [5. Conectar con el robot (Wi-Fi, reloj y terminal)](#5-conectar-con-el-robot)
- [6. Bringup y verificación de tópicos](#6-bringup-y-verificación-de-tópicos)
- [7. Pruebas de movimiento (TwistStamped)](#7-pruebas-de-movimiento-twiststamped)
- [8. Visión, cámara y scripts del repositorio](#8-visión-cámara-y-scripts-del-repositorio)

**Anexos:** [A. Entornos soportados](#anexo-a-entornos-soportados) · [B. DDS y multi-robot](#anexo-b-descubrimiento-dds-y-varios-robots) · [C. Estructura del repositorio](#anexo-c-estructura-del-repositorio) · [D. Guía Mac detallada](#anexo-d-trabajar-desde-mac)

---

## Parte I: Preparar tu entorno (una sola vez, con internet)

> ⚠️ **Importante:** La red Wi-Fi del laboratorio **no tiene salida a internet**. Realiza toda la instalación de paquetes (`apt install`) antes de ir al laboratorio.

El entorno de referencia y más directo es **Ubuntu 24.04 nativo**. Si trabajas desde otro sistema:
- **Ubuntu 24.04 nativo:** Pasa directamente al [Paso 1](#1-instalar-ros-2-jazzy).
- **Windows 11 (WSL 2):** Primero instala y configura WSL 2 con el [Paso 3.1](#31-wsl-2-en-windows-11) (guía paso a paso) y después vuelve al [Paso 1](#1-instalar-ros-2-jazzy).
- **Windows 10 / VirtualBox:** Configura la VM con **Adaptador puente** ([Paso 3.2](#32-virtualbox-windows-10-o-alternativa)).
- **macOS:** Instala Ubuntu 24.04 ARM64/Intel en VM (UTM o Parallels) ([Anexo D](#anexo-d-trabajar-desde-mac)).

---

## 1. Instalar ROS 2 Jazzy

Dentro de tu Ubuntu 24.04:

```bash
# 1. Asegurar locales UTF-8
sudo apt install locales -y && sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Añadir repositorio de ROS 2
sudo apt install software-properties-common curl gnupg2 lsb-release -y
sudo add-apt-repository universe -y
sudo mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Instalar ROS 2 Desktop
sudo apt update && sudo apt install ros-jazzy-desktop -y
```

> 💡 Si `apt update` falla con `EXPKEYSIG` o `NO_PUBKEY`: consulta [TROUBLESHOOTING § 4.1](TROUBLESHOOTING.md#41-error-de-firma-gpg-en-apt-update).

---

## 2. Instalar paquetes del TurtleBot 4 y visión

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-turtlebot4-desktop \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rqt-image-view \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-cv-bridge \
  python3-opencv python3-numpy
```

---

## 3. Configuración para entornos virtuales o Windows

### 3.1 WSL 2 en Windows 11

WSL 2 (*Windows Subsystem for Linux*) ejecuta un Ubuntu real dentro de Windows, sin máquina
virtual aparte. Hazlo **antes del [Paso 1](#1-instalar-ros-2-jazzy)** y **con internet**.

**Requisitos:** Windows 11 22H2 o superior (el modo de red `mirrored` no existe en Windows 10) y
la virtualización activada en la BIOS/UEFI (*Intel VT-x* / *AMD-V*; en el Administrador de
tareas → Rendimiento → CPU debe decir "Virtualización: Habilitado").

**Paso 1. Instalar WSL y Ubuntu 24.04** (PowerShell **como Administrador**):
```powershell
wsl --install -d Ubuntu-24.04
```
Reinicia Windows si te lo pide. Al abrir "Ubuntu 24.04" desde el menú Inicio por primera vez,
te pedirá crear un **usuario y contraseña de Linux** (los usará `sudo`; no tienen que coincidir
con los de Windows).

**Paso 2. Comprobar que es WSL 2 y está actualizado** (PowerShell):
```powershell
wsl --update
wsl -l -v          # Ubuntu-24.04 debe mostrar VERSION 2
```
Si muestra `1`: `wsl --set-version Ubuntu-24.04 2`.

**Paso 3. Red espejo (`mirrored`)** para que ROS 2 vea al robot como si fuera tu propio Windows.
Crea o edita el archivo `%USERPROFILE%\.wslconfig` (p. ej. `notepad $env:USERPROFILE\.wslconfig`):
```ini
[wsl2]
networkingMode=mirrored
```
Aplica con `wsl --shutdown` y vuelve a abrir Ubuntu. Comprueba dentro de Ubuntu con `ip -br a`:
debe aparecer la misma IP que tiene Windows en el Wi-Fi.

**Paso 4. Permitir tráfico entrante en el firewall de Hyper-V** (PowerShell **como Administrador**):
```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```
Si responde "Acceso denegado", la ventana no era de Administrador.

**Paso 5. Perfil de red Privado** (en el laboratorio, ya conectado a `Lab_Computech_<X>_5G`):
Configuración → Red e Internet → Wi-Fi → la red → **Red privada**.

**Paso 6. Clonar el repositorio dentro de Linux** (terminal de Ubuntu):
```bash
cd ~ && git clone https://github.com/Tilinpro19/turtlebot67.git turtleclaude4
```
Trabaja siempre en `~/` y no en `/mnt/c/...`: es mucho más rápido y evita que Windows convierta
los finales de línea a CRLF, lo que rompe los scripts `.sh` (`bad interpreter: /bin/bash^M`).

**Paso 7. Continuar con ROS:** sigue el [Paso 1](#1-instalar-ros-2-jazzy), el [Paso 2](#2-instalar-paquetes-del-turtlebot-4-y-visión)
y el [Paso 4](#4-variables-de-entorno) dentro de Ubuntu. Las ventanas gráficas (RViz, `rqt`, OpenCV)
funcionan directamente gracias a WSLg; no hace falta instalar un servidor X.

> **Descubrimiento DDS:** Aun con `mirrored`, el multicast puede perderse en WSL. `source tb4_connect.sh`
> lo detecta y cambia solo a unicast (`ROS_STATIC_PEERS`). Más casos en [TROUBLESHOOTING § 1.1](TROUBLESHOOTING.md#11-la-pc-no-ve-los-tópicos-del-robot-ros2-topic-list-solo-muestra-parameter_events-y-rosout).

| Comando útil (PowerShell) | Para qué |
|---|---|
| `wsl` | Abrir Ubuntu en la terminal actual |
| `wsl --shutdown` | Reiniciar WSL (necesario tras editar `.wslconfig`) |
| `wsl -l -v` | Ver distros instaladas y su versión |
| `explorer.exe .` (dentro de Ubuntu) | Abrir la carpeta actual de Linux en el Explorador de Windows |

### 3.2 VirtualBox (Windows 10 o alternativa)
- En Configuración de la VM → **Red**: Cambia NAT a **Adaptador puente (Bridged)** seleccionando la interfaz Wi-Fi real. Con NAT el robot no podrá ser descubierto por ROS 2.

---

## 4. Variables de entorno

Agrega lo siguiente al final de tu `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=<DOMAIN_ID>          # Cambia por el ID asignado a tu robot (0-101)
```

> **Nota:** Al usar `source tb4_connect.sh`, las variables y el dominio se configuran dinámicamente según el robot conectado.

---

## Parte II: Flujo en el laboratorio (cada sesión)

## 5. Conectar con el robot

1. Conecta tu laptop a la red **Wi-Fi del laboratorio**, `Lab_Computech_<X>_5G` (p. ej. `Lab_Computech_3_5G` o la de 2.4 GHz del mismo router) con la contraseña `Computech2025!`. Windows dirá "sin internet": es totalmente normal.
2. **Obtener la `<IP_ROBOT>` (TurtleBot 4 Lite no tiene pantalla):**
   - **Forma principal:** Consultar la tabla de clientes DHCP en el panel del router (busca la MAC de la Raspberry Pi).
   - **Por comando en PC:** `arp -a` buscando la MAC del robot (prefijo `d8:3a:dd`, `dc:a6:32` o `e4:5f:01`; en el Lite del lab 3 es `d8:3a:dd:7e:5c:9c` y su IP habitual `192.168.0.104`).
   - *(Si tu red tiene mDNS activo: `ssh ubuntu@turtlebot4.local`).*

### 5.1 Sincronización horaria (Red sin internet)
Dado que el router del laboratorio **no tiene internet**, la Raspberry Pi no puede usar NTP y arranca con la hora desfasada en cada encendido. Esto corrompe los timestamps de `tf` y la odometría:

Copia la hora de tu PC al robot por SSH al inicio de la sesión:
```bash
ssh -t ubuntu@<IP_ROBOT> "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```
*(Más detalles y persistencia en [TROUBLESHOOTING § 2.4](TROUBLESHOOTING.md#24-reloj-desfasado-tf--fusión-de-sensores) y [tarjeta_offline_lab.md](docs/tarjeta_offline_lab.md)).*

### 5.2 Conexión con `tb4_connect.sh` (Recomendado)
Configura tu terminal para interactuar con el robot:
```bash
source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>
```
*Prueba multicast y, si no hay respuesta de tópicos clave, conmuta automáticamente a `ROS_STATIC_PEERS`. Guarda la sesión en `~/.tb4_robot`.*

- Reconectar al último robot: `source tb4_connect.sh`
- Limpiar configuración o desconectarse: `source tb4_connect.sh --off`

### 5.3 Conexión SSH directa al robot
```bash
ssh ubuntu@<IP_ROBOT>     # Contraseña de fábrica: turtlebot4 (cámbiala si el robot sale del lab)
```

---

## 6. Bringup y verificación de tópicos

El servicio `turtlebot4.service` normalmente arranca el bringup de forma automática al encender el robot. Verifica por SSH:
```bash
systemctl is-active turtlebot4.service    # Debe responder "active"
```
*(Si no estuviera activo: `ros2 launch turtlebot4_bringup lite.launch.py`).*

### 6.1 Checklist rápido de comunicación
En tu PC ejecuta:
```bash
ros2 topic list
```

Debes ver los tópicos base: `/scan`, `/odom`, `/cmd_vel` y `/tf` *(la cámara `/oakd/...` aparecerá si el nodo de visión correspondiente está activo)*.

Prueba de humo en ambos sentidos (`talker` en la PC, `listener` en el robot):
```bash
ros2 run demo_nodes_cpp talker        # [PC]
ros2 run demo_nodes_py listener       # [ROBOT] debe imprimir "I heard: ..."
```

### 6.2 Diagnóstico automático con `tb4_check.sh`
Valida frecuencias de sensores, Create 3 y desfase de reloj con un solo comando:
```bash
./tb4_check.sh <IP_ROBOT>            # Diagnóstico desde la PC
./tb4_check.sh <IP_ROBOT> --robot    # Diagnóstico completo PC + SSH al Robot
```

Si algún componente falla, consulta [TROUBLESHOOTING.md](TROUBLESHOOTING.md), y si nada resulta: **llamar a Cortijo 😄 o a Thiago 6️⃣7️⃣**.

---

## 7. Pruebas de movimiento (`TwistStamped`)

En ROS 2 Jazzy, el TurtleBot 4 requiere **`geometry_msgs/msg/TwistStamped`** en `/cmd_vel` (incluye marca de tiempo). Un mensaje `Twist` tradicional no moverá el robot.

### 7.1 Control por teclado estándar
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

### 7.2 Script WASD del repositorio
```bash
python3 teleop_wasd.py
```
*(Detecta si el tópico requiere `Twist` o `TwistStamped`, usa teclas `W/A/S/D` y frena automáticamente al soltar).*

---

## 8. Visión, cámara y scripts del repositorio

### 8.1 Visualizar la cámara OAK-D
En la Wi-Fi compartida del laboratorio, **utiliza siempre el tópico comprimido** para evitar saturar el ancho de banda y perder datos de LiDAR u odometría:

- **Visor OpenCV integrado (recomendado):**
  ```bash
  python3 ver_camara_tb4.py          # Consume /compressed por defecto
  ```
- **Con rqt:**
  ```bash
  ros2 run rqt_image_view rqt_image_view
  # Seleccionar: /oakd/rgb/preview/image_raw/compressed
  ```

### 8.2 Scripts disponibles en este repositorio
| Script | Función | Comando rápido |
|---|---|---|
| `tb4_connect.sh` | Configura variables ROS, comprueba ping y activa DDS | `source tb4_connect.sh <IP> <ID>` |
| `tb4_check.sh` | Valida tópicos clave, Hz y desfase horario de la Pi | `./tb4_check.sh <IP>` |
| `ver_camara_tb4.py` | Visor OpenCV con conteo de FPS | `python3 ver_camara_tb4.py` |
| `ver_lidar_tb4.py` | Vista 2D cenital del LiDAR con aviso de obstáculos | `python3 ver_lidar_tb4.py` |
| `teleop_wasd.py` | Teleoperación por teclado | `python3 teleop_wasd.py` |

### 8.3 Recomendaciones para tus nodos de Visión en Python
1. **Perfil QoS:** La cámara publica con QoS *Sensor Data* (Best Effort). Tu suscriptor debe usar:
   ```python
   from rclpy.qos import qos_profile_sensor_data
   from sensor_msgs.msg import CompressedImage
   # ...
   self.create_subscription(CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.callback, qos_profile_sensor_data)
   ```
2. **Decodificación OpenCV en el callback:**
   ```python
   cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
   ```

---

## Anexo A. Entornos soportados

- **Ubuntu 24.04 nativo:** Entorno principal recomendado. Comunicación DDS directa sin configuración extra y soporte completo de GPU NVIDIA para visión artificial.
- **WSL 2 (Windows 11):** Alternativa para laptops con Windows. Requiere `mirrored` y soporte DDS vía `ROS_STATIC_PEERS` gestionado por `tb4_connect.sh`. Soporta GPU NVIDIA mediante drivers de Windows.
- **VirtualBox:** Requiere adaptador puente sobre Wi-Fi. Sin aceleración por GPU.
- **macOS (UTM / Parallels):** VM con Ubuntu 24.04 Desktop (ARM64 en Apple Silicon o x86 en Intel). Ver detalles en el [Anexo D](#anexo-d-trabajar-desde-mac).

---

## Anexo B. Descubrimiento DDS y varios robots

- **`ROS_DOMAIN_ID`:** Cada robot del laboratorio **debe** tener un ID único (0–101) para que los comandos de un equipo no interfieran con otro robot.
- **Multicast vs Unicast:** En routers con aislamiento de clientes o bajo WSL 2, el multicast puede presentar pérdidas. `ROS_STATIC_PEERS=<IP_ROBOT>` establece el descubrimiento punto a punto directo.
- Si cambias de robot o terminas la sesión, usa `source tb4_connect.sh --off` para restablecer el entorno.

---

## Anexo C. Estructura del repositorio

```text
turtleclaude4/
├── README.md                  ← Guía principal de trabajo diario
├── CONFIGURACION_INICIAL.md   ← Setup inicial del robot desde cero
├── TROUBLESHOOTING.md         ← Diagnóstico y solución de problemas
├── tb4_connect.sh             ← Configuración de conexión y DDS (soporta --off)
├── tb4_check.sh               ← Checklist de sensores, tópicos y reloj
├── ver_camara_tb4.py          ← Visor OpenCV de cámara OAK-D (/compressed)
├── ver_lidar_tb4.py           ← Visor cenital de LiDAR
├── teleop_wasd.py             ← Teleoperación por teclado
├── docs/                      ← Documentación auxiliar y guías de campo
│   ├── guia_conexion_turtlebot4.md ← Puesta a punto Windows + WSL 2 y vías de conexión
│   ├── tarjeta_offline_lab.md ← Checklist de bolsillo para el laboratorio sin internet
│   └── robot_grupo3.md        ← Conexión al robot del grupo 3 (IP, dominio 3, uso compartido)
└── _legacy/                   ← Scripts antiguos y referencias previas
```

---

## Anexo D. Trabajar desde Mac

ROS 2 Jazzy no tiene soporte nativo en macOS. Se debe trabajar dentro de una VM con **Ubuntu 24.04 Desktop**.

### D.1 Mac con Apple Silicon (M1/M2/M3/M4)
1. Instala **[UTM](https://mac.getutm.app)**.
2. Descarga **Ubuntu 24.04 Desktop ARM64** ([enlace ISO ARM64](https://cdimage.ubuntu.com/releases/24.04/release/)). *(Nota: la ISO x86_64 habitual no sirve).*
3. En UTM: Crear VM → **Virtualizar** (Linux) con al menos 4 núcleos y 8 GB RAM.
4. **Configuración de red:** En Ajustes de la VM → *Red* → Selecciona **Puente (Bridged)** en la interfaz Wi-Fi (`en0`).
5. Sigue los pasos del [Paso 1](#1-instalar-ros-2-jazzy) en adelante. Jazzy cuenta con paquetes ARM64 completos.

### D.2 Mac con procesador Intel
Utiliza VirtualBox o UTM con la ISO estándar x86_64 y modo de red en Adaptador puente.

### D.3 Conexión rápida (Solo SSH)
Si solo requieres teleoperación por teclado o ejecutar scripts sin ventanas desde macOS, conéctate directamente desde la terminal con `ssh ubuntu@<IP_ROBOT>`.
