# Puente UDP del TurtleBot 4 (grupo 3)

Comunicación PC ↔ robot por UDP, sin ROS en la PC. Datos del robot y pasos de conexión en
[robot_grupo3.md](robot_grupo3.md). Todos los comandos se ejecutan desde la raíz del repositorio.

```text
PC (Python + numpy, sin ROS)                     Robot (Raspberry Pi, ROS 2 Jazzy)
 tb4/client.py ── CMD v,w / PHOTO / (UN)DOCK ──►  tb4_bridge.py ──► /cmd_vel (con freno por LiDAR)
               ◄── SCAN / ODOM / STATUS / IMG ──                ◄── /scan /odom /battery_state
                         UDP 5600                                    /dock_status /oakd/...
```

| Archivo | Dónde corre | Para qué |
|---|---|---|
| `tb4/protocol.py` | ambos | Formato binario de los paquetes, geometría del LiDAR y freno de seguridad |
| `tb4/client.py` | PC | Librería para el código del proyecto (`from tb4 import TB4Client`; prueba: `python3 -m tb4.client`) |
| `robot/tb4_bridge.py` | robot | Traduce UDP ↔ ROS 2; aplica la seguridad (en el robot queda en `~/tb4_udp/`) |
| `robot/tb4_inicio.sh` | PC | Inicio de sesión: hora, ROS y puente en el orden correcto |
| `robot/deploy_bridge.sh` | PC | Copiar/arrancar/parar el puente (`--fg`, `--bg`, `--stop`, `--status`) |
| `robot/reiniciar_camara.sh` | PC | Reinicia solo la cámara OAK-D si deja de publicar |
| `herramientas/monitor.py` | PC | Visor del LiDAR y del estado (`python3 -m herramientas.monitor`); conducir con `--drive` |
| `ejemplos/esquivar.py` | PC | Avanzar hasta un obstáculo, girar y desplazarse (`python3 -m ejemplos.esquivar`) |
| `tests/test_protocol.py` | PC | Pruebas sin robot: `python3 -m unittest discover tests` |

## Uso desde Python

```python
import time
from tb4 import TB4Client

with TB4Client("192.168.0.103") as robot:      # al salir del with, el robot se detiene
    robot.wait_ready()
    robot.undock()                              # sale del dock (enciende LiDAR y cámara)

    while True:
        scan = robot.scan()                     # None hasta que llegue el primer scan
        if scan is None or scan.age > 0.5:
            robot.stop(); time.sleep(0.05); continue
        front = scan.sector_min(-15, 15)        # m al obstáculo más cercano al frente
        robot.move(0.15 if front > 0.6 else 0.0, 0.0)   # llamar en cada vuelta del bucle
        time.sleep(0.05)
```

| Método | Devuelve |
|---|---|
| `scan()` | `Scan`: `angles` (rad; 0 = frente, + = izquierda), `ranges` (m; `nan` inválido, `inf` libre), `age`, `points()` → (x, y), `sector_min(deg1, deg2)` |
| `pose()` | `Pose`: `x`, `y`, `yaw` de la odometría (origen = donde arrancó la Create 3), `v`, `w`, `age` |
| `status()` | `Status`: `battery`, `docked`, `front_blocked`, `rear_blocked`, `scan_stale`, `is_driver`, `front_clear`, `rear_clear`, `n_clients` |
| `move(v, w)` | Envía velocidad (m/s, rad/s). Si dejas de llamarlo 0.5 s, el robot se detiene |
| `stop()` | Parada inmediata |
| `photo()` | Imagen BGR (numpy) de la OAK-D, 250×250 (*preview*). Falla si el robot está en el dock |
| `photo(hd=True)` | La imagen grande (1280×720) de `/oakd/rgb/image_raw`; hace falta lanzar la cámara con `task-a/camara/camara_rgb.sh --hd`. Un puente viejo devuelve la preview |
| `undock()` / `dock()` | Acciones del Create 3; devuelven el mensaje del robot o lanzan `RuntimeError` |

Ejes: x adelante, y izquierda (`base_link`). El puente ya corrige que el LiDAR del TurtleBot 4 está
montado girado +90°.

## Seguridad (en el robot)

El puente limita **siempre** lo que se envía, aunque el código de la PC se equivoque o el Wi-Fi se retrase:

- Topes: 0.31 m/s y 1.9 rad/s (`--max-linear` para bajarlo).
- Pasillo de ±0.20 m alrededor del eje del robot: no avanza si hay algo a menos de 0.28 m
  (vuelve a dejarlo sobre 0.38 m), y entre 0.60 m y 0.28 m reduce la velocidad máxima. Lo mismo hacia atrás.
- Sin `/scan` reciente (0.5 s) no hay avance ni retroceso; girar en el sitio sí.
- Sin comandos durante 0.3 s → parada. Solo una PC conduce a la vez (la primera que manda `CMD`;
  se libera tras 1 s sin comandos).
- La Create 3 añade lo suyo: parachoques, sensores de desnivel y no retrocede más de ~2–3 cm seguidos.

Esto **no** reemplaza la navegación de la Tarea A: es la última línea de defensa, no un planificador.

## Fotos

El puente se suscribe a la cámara **al primer pedido** y deja la suscripción abierta mientras sigan
llegando pedidos: cada foto espera solo el próximo cuadro (con la preview, ~12 fotos/s medidas en la
simulación ROS). A los 3 s sin pedidos (`--foto-mantener`) la cierra y la cámara deja de mandar datos al
puente. Con `--hd-topic` se elige el tópico de las fotos HD (`Image`, o `CompressedImage` si termina en
`/compressed`: un JPEG ya comprimido se reenvía sin decodificar) y con `--hd-width` el ancho máximo.

Para probar el puente sin robot: `task-a/carrera-1/sim_ros.sh` (simulador como robot ROS 2 + este puente).

## Protocolo

Little-endian. Cabecera `G3TB` + versión (1) + tipo (1 byte) + token (8 bytes = sha256 de la clave
del grupo, `grupo3` por defecto; solo evita mezclarse con otro grupo, no es seguridad). Los formatos
exactos están en `tb4/protocol.py`. `PHOTO` lleva `id, calidad` y, opcionalmente, un byte de cámara
(`0` = preview, `1` = HD): un puente viejo solo lee los 5 primeros bytes y responde con la preview. El LiDAR viaja en milímetros `uint16` (~2 bytes por punto), la
foto en trozos de 1200 bytes que el cliente vuelve a unir. Las PCs mandan `HELLO` cada segundo; el
robot deja de enviarles datos tras 3 s sin noticias.

Diseñado para evitar los problemas típicos de un puente UDP simple: publicar ceros continuamente en
`/cmd_vel`, aceptar comandos de cualquiera, emparejamiento que no sobrevive a reiniciar la PC, LiDAR
en texto, imágenes continuas y la falta de odometría.
