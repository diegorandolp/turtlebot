# Robot del grupo 3 · Conexión paso a paso

← [README del proyecto](../README.md) · [Guía ROS (setup)](../setup/README.md) · [TROUBLESHOOTING](../setup/TROUBLESHOOTING.md) · [CONFIGURACION_INICIAL](../setup/CONFIGURACION_INICIAL.md)

Guía específica para el **TurtleBot 4 Lite del grupo 3** (proyecto final *Autonomous and
Gesture-Controlled Navigation Challenge*). Todo lo de aquí fue verificado el **2026-10-30**.
Para instalar ROS 2 en tu PC sigue antes la [Parte I de la guía ROS](../setup/README.md#parte-i-preparar-tu-entorno-una-sola-vez-con-internet).

> ⚠️ Este robot **no es** el del laboratorio 3 que aparece en
> [tarjeta_offline_lab.md](../setup/docs/tarjeta_offline_lab.md) (`192.168.0.104`, dominio 67). No mezcles
> esos datos con los de aquí.

---

## Datos del robot

| Dato | Valor |
|---|---|
| Modelo | TurtleBot 4 **Lite** (sin pantalla), ROS 2 **Jazzy** |
| Router | TP-Link AC1200 `Lab_Computech_3` |
| Wi-Fi | `Lab_Computech_3_5G` (o `Lab_Computech_3`, 2.4 GHz) · contraseña `Computech2025!` |
| IP del robot | **`192.168.0.103`** (DHCP: puede cambiar, ver §2) |
| MAC Wi-Fi | `d8:3a:dd:7e:5e:48` |
| `ROS_DOMAIN_ID` | **`3`** (Raspberry Pi y Create 3) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| SSH | `ssh ubuntu@192.168.0.103` · contraseña `turtlebot4` (por defecto, se mantiene así) |
| Cable (plan B) | Raspberry Pi `192.168.185.3`; tu PC `192.168.185.10/24` sin gateway |

---

## Para el asistente (Claude) de cada compañero

Si eres un asistente ayudando a un miembro del grupo 3, sigue estas reglas:

1. Usa siempre `ROS_DOMAIN_ID=3` y la IP de la tabla. Si el ping falla, busca la IP por la MAC (§2)
   antes de suponer que el robot está mal.
2. **No cambies la configuración del robot** (Wi-Fi, dominio, contraseña, servicios, firmware)
   sin que el usuario lo pida explícitamente: otros compañeros pueden estar usándolo a la vez.
   En especial, `sudo systemctl restart turtlebot4.service` corta el robot para **todos**.
3. **No publiques en `/cmd_vel` sin confirmación del usuario** y comprueba antes que nadie más
   lo esté controlando (§4). El robot está en un aula con mesas y personas.
4. Antes de mover el robot de verdad, prueba con `python3 setup/teleop_wasd.py --dry`, o con
   `python3 -m herramientas.monitor` sin `--drive`.
5. Si `ping` funciona pero `ros2 topic list` no muestra `/scan`, `/odom`, `/cmd_vel`, sigue §6
   antes de tocar la red o el firewall a ciegas.
6. La red del laboratorio **no tiene internet**: instala todo antes (o usa el anclaje USB
   del celular para tener internet por cable y el Wi-Fi en el router del laboratorio).
7. **No cambies la hora del robot a mano ni reinicies solo `turtlebot4.service`**: rompe la
   comunicación con la Create 3 (§3). Usa `robot/tb4_inicio.sh`.

---

## 1. Preparar tu PC (una sola vez)

- **Ubuntu 24.04 / WSL 2 / VM:** [guía ROS, Parte I](../setup/README.md#parte-i-preparar-tu-entorno-una-sola-vez-con-internet)
  (ROS 2 Jazzy + `ros-jazzy-turtlebot4-desktop`).
- Añade además los mensajes del Create 3 (para leer el dock y usar `/undock`):
  ```bash
  sudo apt install -y ros-jazzy-irobot-create-msgs ros-jazzy-turtlebot4-msgs
  ```
- **Firewall:** si tu Linux tiene `ufw` activo, permite la red del laboratorio
  (sin esto el ping funciona pero ROS no ve al robot):
  ```bash
  sudo ufw allow from 192.168.0.0/24
  ```
  En WSL 2 aplica el firewall de Hyper-V del [guía ROS § 3.1](../setup/README.md#31-wsl-2-en-windows-11).
- Clona este repositorio e instala lo de Python: `pip install -r requirements.txt` (ver [README](../README.md)).

---

## 2. Conectarte a la red y encontrar el robot

1. Conecta tu PC a **`Lab_Computech_3_5G`** (contraseña `Computech2025!`). Dirá "sin internet": es normal.
2. Comprueba el robot:
   ```bash
   ping -c 3 192.168.0.103
   ```
3. Si no responde, búscalo por su MAC (puede haber cambiado de IP):
   ```bash
   for i in $(seq 1 254); do ping -c1 -W1 192.168.0.$i >/dev/null & done; wait
   ip neigh | grep -i d8:3a:dd:7e:5e:48        # Windows (PowerShell): arp -a | findstr d8-3a-dd-7e-5e-48
   ```
   Si tampoco aparece: el robot sigue arrancando (espera ~2 min tras la melodía) o está apagado.

---

## 3. Inicio de sesión (una persona, tras cada encendido)

> ⚠️ **La hora del robot es la causa de casi todos los problemas.** La Raspberry no tiene reloj ni
> internet y arranca con hora de 2024. Si se adelanta la hora **con ROS ya corriendo**, Fast DDS deja
> de ver a los nodos que arrancaron antes (ping OK pero sin tópicos, sin `/odom`, sin batería), y la
> Create 3, que toma la hora de la Raspberry, también deja de publicar. **No cambies la hora a mano:**
> usa el script, que la corrige y reinicia ROS en el orden correcto.

Desde la PC (Linux, WSL o Git Bash, con Python + numpy):
```bash
cd turtlebot67
robot/tb4_inicio.sh            # corrige la hora si hace falta, arranca el puente UDP y espera datos
robot/tb4_inicio.sh --force    # si algo sigue sin datos: secuencia completa aunque la hora esté bien
SSHPASS=turtlebot4 TB4_IP=192.168.0.104 robot/tb4_inicio.sh    # sin teclear la contraseña SSH (requiere sshpass)
```
Si la hora estaba mal, tarda **~5–6 min** (la Create 3 se reinicia). Resultado esperado:
`[ok] robot listo ...: bateria NN%, dock True, sin LiDAR (normal en el dock)`.
Los compañeros que lleguen después **no** lo repiten: solo se conectan (§5.3 o §3.1).

### 3.1 Conexión por ROS 2 directo (opcional)

Solo si necesitas herramientas ROS (`rqt`, RViz, `ros2 topic`). El código del proyecto usa UDP (§5.3).
```bash
cd turtlebot67/setup
source tb4_connect.sh 192.168.0.103 3    # configura dominio, RMW y modo de descubrimiento
./tb4_check.sh 192.168.0.103             # diagnóstico de tópicos, Hz y reloj
```

Resultado esperado de `tb4_connect.sh`: `descubrimiento multicast: 4/4 topicos clave` y `[ok] conectado`.
En este router el multicast **sí** funciona; en WSL el script puede pasar solo a `ROS_STATIC_PEERS`.

Comprobación manual:
```bash
ros2 topic list | grep -E '^/(scan|odom|cmd_vel|battery_state)$'
ros2 topic echo --once /battery_state --field percentage     # 0.0–1.0
ros2 topic info /cmd_vel                                       # Type: geometry_msgs/msg/TwistStamped
```

---

## 4. ¿Varios compañeros a la vez?

**Sí.** ROS 2 permite que varias PCs en la misma red y con `ROS_DOMAIN_ID=3` vean el robot a la vez.
Leer sensores (`/scan`, `/odom`, cámara) desde varias PCs funciona sin problemas. Reglas:

| Regla | Por qué |
|---|---|
| **Solo una persona controla el movimiento.** Antes de mover, mira quién publica: `ros2 topic info -v /cmd_vel` | Si dos PCs publican en `/cmd_vel`, el robot obedece a ambas a saltos. Lo normal es ver solo `teleop_twist_joy_node` (del propio robot, inactivo sin mando) como publicador |
| **Cámara: máximo 1–2 personas** y siempre por `/compressed` | Cada suscriptor duplica el tráfico por Wi-Fi; con varios se pierden datos de `/scan` y `/odom` |
| **Nadie reinicia el robot ni su servicio sin avisar** | Se corta para todos |
| `tb4_inicio.sh` lo ejecuta una sola persona tras cada encendido | Repetirlo reinicia ROS para todos si la hora estaba mal |
| Todos con dominio 3 y los demás grupos con otro | Si otro grupo usa el 3, controlaría vuestro robot |

---

## 5. Mover el robot

### 5.1 Sacarlo del dock

Si el robot está cargando en el dock, **la cámara está apagada** (`power_saver` del TurtleBot 4
la detiene en el dock). Para sacarlo de forma segura (retrocede y se separa del dock solo):
```bash
ros2 topic echo --once /dock_status --field is_docked       # true = en el dock
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
# Para volver a cargar:  ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"   (el dock debe estar a la vista)
```

### 5.2 `setup/teleop_wasd.py` (por ROS)

```bash
cd turtlebot67/setup
python3 teleop_wasd.py --dry     # primero: muestra qué enviaría, sin mover nada
python3 teleop_wasd.py           # control real
```

| Tecla | Acción |
|---|---|
| `w` / `s` | adelante / atrás |
| `a` / `d` | girar en el sitio izquierda / derecha |
| `q` / `e` | adelante girando |
| `espacio` / `x` | parar |
| `u` / `j` · `i` / `k` | subir/bajar velocidad lineal · angular |
| `Ctrl+C` | salir (envía velocidad cero) |

**Cuánto se mueve:** solo mientras **mantienes** la tecla; al soltarla se detiene en ~0.6 s.
Arranca a **0.10 m/s** (≈10 cm por segundo de tecla) y **1.0 rad/s** al girar (≈57° por segundo).
Cada `u` suma 0.05 m/s; el Create 3 limita a ~0.31 m/s por su seguridad de fábrica.

**Con mesas alrededor:**
- No usa el LiDAR para frenar: si vas contra algo, choca. Solo el parachoques frontal detiene la base al tocar.
- Las patas de mesas y sillas son lo más fácil de golpear; los tableros quedan por encima del LiDAR.
- **Hacia atrás no tiene sensores.** Retrocede poco y mirando.
- Da toques cortos a `w` al principio y deja la velocidad inicial hasta conocer el robot.
- Para ver obstáculos mientras conduces: `python3 setup/ver_lidar_tb4.py` en otra terminal
  (este visor no aplica el giro del LiDAR: usa `--rot 90` para que el frente salga arriba).
- La Create 3 tiene `safety_override: none`: **no retrocede más de ~2–3 cm** seguidos (sin sensores
  de desnivel atrás). No es una avería.

### 5.3 Puente UDP (lo que usa el código del proyecto)

El robot corre `robot/tb4_bridge.py` (lo arranca `robot/tb4_inicio.sh`). La PC solo necesita Python con
`numpy` y `opencv-python`; **no necesita ROS**. Detalles y API en [comunicacion_udp.md](comunicacion_udp.md).

```bash
cd turtlebot67                           # siempre desde la raíz del repositorio
python3 -m tb4.client                     # prueba: imprime batería, dock, distancias y odometría
python3 -m herramientas.monitor           # LiDAR desde arriba + estado (solo mirar)
python3 -m herramientas.monitor --drive   # además conducir con WASD (u = sacar del dock, p = foto)
```

Seguridad del puente (en el robot, aunque el Wi-Fi se retrase):
- Para solo si no recibe comandos durante 0.3 s.
- No avanza si hay algo a menos de **0.28 m** en el pasillo del robot (vuelve a permitirlo sobre 0.38 m);
  frena progresivamente desde 0.60 m. Lo mismo hacia atrás. Girar en el sitio siempre se permite.
- Sin LiDAR (p. ej. en el dock) no avanza ni retrocede: usa `undock` (tecla `u`) para salir.
- Solo una PC conduce; las demás solo reciben datos.

---

## 6. Problemas conocidos de este robot

| Síntoma | Causa | Solución |
|---|---|---|
| Ping OK, pero sin tópicos, sin `/odom` o sin batería (en ROS o en el puente UDP) | Se cambió la hora del robot con ROS corriendo, o se reinició solo el servicio de la Raspberry | **Avisando al grupo:** `robot/tb4_inicio.sh --force` y esperar ~5 min |
| Ping OK, nada de ROS, en tu PC Linux | Firewall local | `sudo ufw allow from 192.168.0.0/24` |
| Cámara sin imágenes | Robot en el dock (`power_saver`) | Desacoplar (§5.1) |
| Foto falla fuera del dock / `X_LINK_ERROR` en `journalctl -u turtlebot4.service` | La OAK-D perdió la conexión USB (pasó 2 veces el 2026-10-30) | `robot/reiniciar_camara.sh` (reinicia solo la cámara, ~15 s). Si se repite: cargar y reconectar la OAK-D en el USB azul ([TROUBLESHOOTING § 2.1](../setup/TROUBLESHOOTING.md)) |
| `/odom` o `/battery_state` sin datos tras reiniciar la Create 3 | Tarda **~5 min** en volver a publicar | Esperar; si no, `robot/tb4_inicio.sh --force` |
| `herramientas.monitor` dice "SIN LIDAR" | Robot en el dock (`power_saver` apaga LiDAR y cámara) | Desacoplar (§5.1 o tecla `u`) |
| tf/odometría con tiempos raros | Reloj de la Raspberry desfasado | `robot/tb4_inicio.sh` (§3) |

---

## 7. Cambios aplicados al robot (registro)

| Fecha | Cambio |
|---|---|
| 2026-10-30 | `ROS_DOMAIN_ID` de `0` → `3` en `/etc/turtlebot4/setup.bash` y en el Create 3 (mismo procedimiento que `turtlebot4-setup`, con NTP del Create 3 apuntando a la Raspberry) |
| 2026-10-30 | Drop-in `/etc/systemd/system/turtlebot4.service.d/wait-wifi.conf`: el bringup espera hasta 60 s a que `wlan0` tenga IPv4. Para deshacerlo: borrar el archivo y `sudo systemctl daemon-reload` |
| 2026-10-30 | Puente UDP en `~/tb4_udp/` (no es un servicio: lo arranca `robot/tb4_inicio.sh` o `robot/deploy_bridge.sh`; se detiene con `robot/deploy_bridge.sh --stop`) |

Datos del hardware: Create 3 con firmware `I.0.0.FastDDS` (ROS 2 **Iron**, la Raspberry usa Jazzy;
funciona) y `safety_override: none`. LiDAR montado girado **+90°** respecto a `base_link` (TF medido).

---

← [README del proyecto](../README.md)
