# Troubleshooting · TurtleBot 4 (ROS 2 Jazzy)

← [Volver al README (trabajo diario)](README.md) · [CONFIGURACION_INICIAL](CONFIGURACION_INICIAL.md)

Guía de fallos frecuentes en el laboratorio, válida para TurtleBot 4 Lite y Standard con
**ROS 2 Jazzy / Ubuntu 24.04**. El primer recurso es siempre el flujo del [README](README.md)
(basado en la guía de Cortijo); lo de aquí recoge lo aprendido cuando ese flujo no bastó.

**Usa esta guía si:** el robot está configurado pero algo falla (no ves tópicos, un sensor no
publica, el robot no se mueve, `apt` da errores). Si el robot está desconfigurado o viene de
fábrica, ve directamente a [CONFIGURACION_INICIAL.md](CONFIGURACION_INICIAL.md). Ejecuta primero
el [checklist rápido](#3-checklist-rápido-de-diagnóstico) y luego ve a la sección del síntoma.

Convenciones: **[PC]** = tu PC de desarrollo (Ubuntu nativo, WSL o VM) · **[ROBOT]** = terminal SSH en la Raspberry Pi.

| Marcador | Significado |
|---|---|
| `<IP_ROBOT>` | IP del robot en la red del laboratorio |
| `<DOMAIN_ID>` | `ROS_DOMAIN_ID` de **tu** robot (0–101, único en el laboratorio) |
| `<WIFI_SSID>` | Red del laboratorio: `Lab_Computech_<X>_5G`, donde `<X>` es el número del laboratorio (p. ej. `Lab_Computech_3_5G`) |
| `<WIFI_PASSWORD>` | Contraseña del router del laboratorio (dada por el docente) |

---

## Índice

1. [Red y DDS](#1-red-y-dds)
2. [Cámara y sensores](#2-cámara-y-sensores)
3. [Checklist rápido de diagnóstico](#3-checklist-rápido-de-diagnóstico)
4. [Instalación](#4-instalación)
5. [Escalar](#5-escalar)

---

## 1. Red y DDS

### 1.1 La PC no ve los tópicos del robot (`ros2 topic list` solo muestra `/parameter_events` y `/rosout`)

Revisa en orden. Cada paso descarta una capa.

| # | Comprobación | Comando | Esperado |
|---|---|---|---|
| 1 | Conectividad IP | **[PC]** `ping -c 3 <IP_ROBOT>` | Respuestas, 0 % pérdida |
| 2 | Mismo dominio | **[PC]** y **[ROBOT]** `echo $ROS_DOMAIN_ID` | Mismo número en ambos |
| 3 | Mismo RMW | `echo $RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` en ambos (o ambos vacíos) |
| 4 | Entorno cargado | `printenv ROS_DISTRO` | `jazzy` |
| 5 | Daemon con caché vieja | `ros2 daemon stop && ros2 topic list` | Lista completa |
| 6 | Multicast | **[ROBOT]** `ros2 multicast receive` · **[PC]** `ros2 multicast send` | El robot recibe `Hello World!` |

Si falla el paso **6**, la red bloquea multicast (WSL, Wi-Fi universitaria o de invitados,
"aislamiento de clientes"). `source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>` lo detecta y
aplica la solución solo. A mano, usa peers estáticos (Jazzy):

```bash
# [PC]
export ROS_STATIC_PEERS=<IP_ROBOT>
ros2 daemon stop
ros2 topic list
```

Fue la causa real en el caso WSL documentado en `docs/`. Si funciona, añádelo a `~/.bashrc`.

Causas específicas del entorno:

| Entorno | Causa | Solución |
|---|---|---|
| VirtualBox | Adaptador en NAT | Configuración → Red → **Adaptador puente**, sobre la interfaz Wi-Fi real |
| WSL2 | Red NAT por defecto | `.wslconfig` → `networkingMode=mirrored`, `wsl --shutdown` |
| WSL2 | Firewall Hyper-V | PowerShell **admin**: `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow` |
| Windows | Red en perfil "Público" | Configuración → Red → perfil **Privada** |
| Cualquiera | Ping OK pero sin tópicos y ningún arreglo funciona | Cable Ethernet directo: PC `192.168.185.10/24` sin gateway, robot `192.168.185.3` |

### 1.2 Discrepancias de `ROS_DOMAIN_ID`

Hay **tres** lugares que deben coincidir:

1. **PC/VM**: `~/.bashrc` (`export ROS_DOMAIN_ID=<N>`).
2. **Raspberry Pi**: `turtlebot4-setup` → *ROS Setup* → *Bash Setup*. Un `export` en tu sesión
   SSH **no** cambia el servicio `turtlebot4.service`.
3. **Create 3**: portal `http://<IP_ROBOT>:8080` → *Application → Configuration*.

Para corregir (2) y (3): [CONFIGURACION_INICIAL § 3](CONFIGURACION_INICIAL.md#3-ros_domain_id-del-robot)
y [§ 2.1](CONFIGURACION_INICIAL.md#21-parámetros-de-red-y-ros).

Síntoma típico de (3) desalineado: aparecen `/scan` y `/oakd/...` (que publica la Pi),
pero **faltan** `/odom`, `/battery_state`, `/imu`, `/hazard_detection` y `/cmd_vel` no mueve
el robot (los publica o suscribe el Create 3).

```bash
# [ROBOT] ¿la Pi ve al Create 3?
ping -c 3 192.168.186.2
ros2 node list | grep -iE 'motion_control|robot_state|create'
```

### 1.3 Reiniciar el Create 3

De menor a mayor intrusividad:

1. **`turtlebot4-setup`** → reiniciar Create 3 / *Apply Settings*. Espera la melodía de arranque.
2. Portal `http://<IP_ROBOT>:8080` → *Application → Restart Application*.
   Sin acceso directo al puerto: `ssh -L 8080:192.168.186.2:80 ubuntu@<IP_ROBOT>` y abre `http://localhost:8080`.
3. Apagado completo: robot fuera del dock, mantén el **botón central** del Create 3 ~10 s, y vuelve a encender.

Tras reiniciar solo la Pi, el Create 3 puede tardar **2–3 min** en volver a publicar `/odom`.

### 1.4 El robot no aparece en ninguna red

- **Standard**: la IP aparece en la pantalla del robot.
- **Lite** (sin pantalla): busca la IP por red (`arp -a` con MAC de Raspberry Pi `d8:3a:dd`/`dc:a6:32`/`e4:5f:01`, panel del router, `turtlebot4.local`).
- Si perdiste la Wi-Fi: cable Ethernet (`192.168.185.3`), o monitor micro-HDMI + teclado en la Pi, y luego `turtlebot4-setup`.
- SSH que se congela a ratos: ahorro de energía de la Wi-Fi.
  ```bash
  # [ROBOT] debe devolver 2 (disable)
  nmcli -f 802-11-wireless.powersave connection show <nombre_conexion>
  ```

---

## 2. Cámara y sensores

Primero confirma que el bringup está vivo:

```bash
# [ROBOT]
systemctl status turtlebot4.service --no-pager | head -5
journalctl -u turtlebot4.service -n 50 --no-pager
ros2 node list
```

Si lanzaste `lite.launch.py` / `standard.launch.py` a mano **y** el servicio está activo, hay nodos duplicados:
detén uno de los dos (`sudo systemctl restart turtlebot4.service` y no relances a mano).

### 2.1 OAK-D no publica imagen

| # | Comprobación | Comando | Esperado / acción |
|---|---|---|---|
| 1 | USB detecta la cámara | **[ROBOT]** `lsusb \| grep 03e7` | Línea `Intel Movidius` / `Luxonis`. Si no: revisa el cable, usa un puerto **USB 3 (azul)** |
| 2 | Nodo vivo | `ros2 node list \| grep oakd` | `/oakd` |
| 3 | Tópico existe | `ros2 topic list \| grep oakd/rgb` | `/oakd/rgb/preview/image_raw` |
| 4 | Llegan frames al robot | **[ROBOT]** `ros2 topic hz /oakd/rgb/preview/image_raw` | Frecuencia > 0 |
| 5 | Llegan frames a la PC | **[PC]** `ros2 topic hz /oakd/rgb/preview/image_raw/compressed` | Frecuencia > 0 |
| 6 | Errores del driver | `journalctl -u turtlebot4.service \| grep -iE 'oakd\|depthai\|X_LINK'` | Sin `X_LINK_ERROR` |

Arreglos:
- `X_LINK_ERROR` / `Device not found`: desconecta y reconecta la OAK-D, o reinicia el servicio. Con batería baja, la cámara se reinicia sola: carga el robot.
- Aparece en `lsusb` pero el driver no la abre (`insufficient permissions`): falta la regla udev → [CONFIGURACION_INICIAL § 5.1](CONFIGURACION_INICIAL.md#51-oak-d-no-detectada).
- Relanzar la cámara aislada (con el bringup detenido): `ros2 launch turtlebot4_bringup oakd.launch.py`.
- **Paso 4 OK, paso 5 falla**: es ancho de banda de Wi-Fi, no la cámara. Usa `/compressed` en `rqt_image_view`, acércate al router o usa cable.

### 2.2 RPLIDAR no publica `/scan`

| # | Comprobación | Comando | Esperado / acción |
|---|---|---|---|
| 1 | Dispositivo serie | **[ROBOT]** `ls -l /dev/RPLIDAR /dev/ttyUSB*` | `/dev/RPLIDAR → ttyUSB0` |
| 2 | Motor girando | Observación física | El cabezal gira. Si no: `ros2 service call /start_motor std_srvs/srv/Empty` |
| 3 | Nodo vivo | `ros2 node list \| grep -i rplidar` | `/rplidar_composition` |
| 4 | Frecuencia | `ros2 topic hz /scan` | ~7.5 Hz |
| 5 | Datos válidos | `ros2 topic echo /scan --once \| head -30` | `ranges` con valores finitos |

Arreglos:
- Sin `/dev/RPLIDAR`: falta la regla udev → [CONFIGURACION_INICIAL § 5.2](CONFIGURACION_INICIAL.md#52-rplidar-sin-devrplidar).
- `Permission denied` en `/dev/ttyUSB0`: `sudo usermod -aG dialout ubuntu`, cierra la sesión y vuelve a entrar.
- Relanzar aislado: `ros2 launch turtlebot4_bringup rplidar.launch.py`.

### 2.3 `/cmd_vel` no mueve el robot

| Causa | Diagnóstico | Solución |
|---|---|---|
| Publicas `Twist` en un tópico `TwistStamped` | `ros2 topic info /cmd_vel` → `TwistStamped` | `teleop_twist_keyboard ... -p stamped:=true`, o publica en `/cmd_vel_unstamped` |
| Create 3 desalineado | Faltan `/odom` y `/battery_state` | §1.2 y §1.3 |
| Robot en el dock o con un hazard activo | `ros2 topic echo /hazard_detection --once` · `/dock_status` | Desacopla; despeja el *cliff* o *bump* |
| Otro nodo publica ceros | `ros2 topic info /cmd_vel -v` (varios publishers) | Detén el otro teleop (p. ej. un mando conectado a la Pi) |

### 2.4 Reloj desfasado (tf / fusión de sensores)

La Pi no tiene RTC y el Create 3 conserva su propio reloj: los *stamps* pueden marcar otra
fecha (visto en el laboratorio: `/odom` con fecha de un año antes). Afecta a tf, `message_filters` y a la fusión de sensores.

En el laboratorio el router **no tiene internet**, así que NTP no funciona y la Pi arranca
con la hora mal **en cada encendido**. `tb4_check.sh` lo detecta (`reloj desfasado`).
Copia la hora de la PC al robot al empezar la sesión:

```bash
# [PC] (asegúrate antes de que la hora de tu PC es correcta)
ssh -t ubuntu@<IP_ROBOT> "sudo date -u -s '$(date -u +%Y-%m-%dT%H:%M:%S)'"
```

Si el robot tiene internet (otra red), basta con `sudo timedatectl set-ntp true` **[ROBOT]**.

---

## 3. Checklist rápido de diagnóstico

Ejecútalo completo **antes de escalar un incidente** y adjunta la salida.

**Versión automática:** `./tb4_check.sh <IP_ROBOT> --robot` ejecuta estos mismos pasos,
marca cada uno como `[ok]` / `[FALLA]` e indica la sección de esta guía a consultar
([README § 6.2](README.md#62-diagnóstico-automático-con-tb4_checksh)). A continuación, la versión manual.

```bash
# ---------- [PC] ----------
printenv | grep -E 'ROS_|RMW_'                 # 1. Entorno: DISTRO=jazzy, DOMAIN_ID, RMW
ping -c 3 <IP_ROBOT>                           # 2. IP
ros2 daemon stop                               # 3. Limpiar caché de descubrimiento
ros2 topic list                                # 4. ¿Se ven los tópicos?
ros2 topic hz /scan                            # 5. LiDAR (~7.5 Hz)        Ctrl+C
ros2 topic hz /oakd/rgb/preview/image_raw      # 6. Cámara                  Ctrl+C
ros2 topic echo /odom --once                   # 7. Create 3 vivo
ros2 topic info /cmd_vel                       # 8. Tipo = TwistStamped

# ---------- [ROBOT] ----------
echo $ROS_DOMAIN_ID                            # 9. Coincide con la PC
systemctl is-active turtlebot4.service         # 10. active
ping -c 3 192.168.186.2                        # 11. Pi ↔ Create 3 (usb0)
ros2 node list                                 # 12. /oakd, /rplidar_composition, /turtlebot4_node...
lsusb | grep 03e7; ls -l /dev/RPLIDAR          # 13. Hardware USB presente
journalctl -u turtlebot4.service -n 40 --no-pager   # 14. Errores del bringup
```

| Falla en | Ir a |
|---|---|
| 2 | §1.1 (red) / §1.4 |
| 4 (con 2 OK) | §1.1 pasos 2–6 |
| 5, 6, 12, 13 | §2.1 / §2.2 |
| 7, 11 | §1.2 / §1.3 |
| 8 o el robot no se mueve | §2.3 |

Prueba de humo final: `talker` en la PC y `listener` en el robot ([README § 6.1](README.md#61-checklist-rápido-de-comunicación)).

---

## 4. Instalación

### 4.1 Error de firma GPG en `apt update`

Ocurre con `EXPKEYSIG` / `NO_PUBKEY` en `packages.ros.org`. Sustituye la llave manual por
el paquete oficial `ros2-apt-source`:

```bash
sudo rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

### 4.2 `ros-$ROS_DISTRO-...` no se encuentra

`$ROS_DISTRO` está vacío hasta que haces `source /opt/ros/jazzy/setup.bash`. Escribe
`ros-jazzy-...` explícitamente.

---

## 5. Escalar

Si el checklist completo no resuelve el problema, contacta al responsable del
laboratorio (Cortijo) o a Thiago 6️⃣7️⃣ con: salida del §3, `ROS_DOMAIN_ID` usado, red (SSID) y tipo de
máquina (VirtualBox / WSL / nativo).

---

← [Volver al README](README.md)
