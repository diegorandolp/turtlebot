#!/usr/bin/env python3
"""
Puente UDP <-> ROS 2 que corre EN EL ROBOT (Raspberry Pi del TurtleBot 4, grupo 3).

La PC habla solo UDP (ver tb4/protocol.py y tb4/client.py); este nodo traduce:
  - CMD (v, w) de la PC        -> /cmd_vel (TwistStamped), con freno de emergencia por LiDAR
  - /scan, /odom, bateria, dock -> SCAN, ODOM, STATUS a todas las PCs conectadas
  - PHOTO                       -> un JPEG de la camara OAK-D, solo cuando se pide (preview 250x250 o HD)
  - DOCK / UNDOCK               -> acciones /dock y /undock del Create 3

Seguridad (se aplica aqui, en el robot, aunque el Wi-Fi vaya lento):
  - Solo una PC conduce a la vez ("driver"); las demas solo reciben datos.
  - Si no llegan comandos durante --cmd-timeout s, el robot se detiene.
  - Avance/retroceso bloqueado si hay algo a menos de --stop-dist en el pasillo del robot,
    o si el LiDAR no publica (p. ej. en el dock). Los giros en el sitio siempre se permiten.
  - Solo publica en /cmd_vel mientras conduce (y unos ceros al parar): no pisa a otros teleops.

Uso (en el robot; mas facil con robot/tb4_inicio.sh o robot/deploy_bridge.sh desde la PC):
  source /etc/turtlebot4/setup.bash
  python3 tb4_bridge.py                     # puerto 5600, clave "grupo3"
  python3 tb4_bridge.py --max-linear 0.2    # limitar mas la velocidad

Fotos: mientras una PC siga pidiendo fotos, la suscripcion a la camara queda abierta (cada pedido espera
solo el proximo cuadro); a los --foto-mantener s sin pedidos se cierra y la camara deja de mandar datos.
"""
import argparse
import math
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, CompressedImage, Image, LaserScan

try:
    from tb4 import protocol as P
except ImportError:  # ejecutado desde el repositorio: robot/../tb4
    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from tb4 import protocol as P

CONTROL_HZ = 20
TELEMETRY_STATUS_HZ = 4
CLIENT_TIMEOUT = 3.0       # s sin HELLO -> la PC deja de recibir datos
DRIVER_RELEASE = 1.0       # s sin CMD -> otra PC puede tomar el control
STOP_REPEAT = 3            # ceros publicados al detenerse
PHOTO_TIMEOUT = 3.0


def imgmsg_a_bgr(msg):
    """sensor_msgs/Image -> BGR (numpy). Con cv_bridge si esta; si no, a mano para las codificaciones comunes."""
    try:
        from cv_bridge import CvBridge
        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except ImportError:
        pass
    canales = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}.get(msg.encoding)
    if canales is None:
        raise RuntimeError(f"codificacion {msg.encoding} no soportada sin cv_bridge")
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * canales]
    a = a.reshape(msg.height, msg.width, canales)
    if msg.encoding in ("rgb8", "rgba8"):
        a = a[:, :, 2::-1] if canales == 3 else a[:, :, [2, 1, 0]]
    elif canales == 4:
        a = a[:, :, :3]
    elif canales == 1:
        a = np.repeat(a, 3, axis=2)
    return np.ascontiguousarray(a)


class Bridge(Node):
    def __init__(self, args):
        super().__init__("tb4_udp_bridge")
        self.args = args
        self.token = P.make_token(args.key)
        self.safety = P.SafetyFilter(P.SafetyConfig(
            max_linear=args.max_linear, max_angular=args.max_angular,
            stop_dist=args.stop_dist, clear_dist=args.stop_dist + 0.10,
            slow_dist=args.slow_dist))
        self.lock = threading.Lock()

        # Estado compartido entre el hilo UDP y los callbacks de ROS
        self.pcs = {}            # addr -> ultima vez que se supo de ella
        self.driver = None           # addr de la PC que conduce
        self.cmd = (0.0, 0.0)
        self.cmd_time = 0.0
        self.moving = False
        self.stop_left = 0
        self.flags = P.FLAG_SCAN_STALE
        self.front = self.rear = math.inf
        self.scan_time = None
        self.battery = -1.0
        self.docked = P.DOCK_UNKNOWN
        self.pending = []            # peticiones (tipo, addr, id, extra) para el hilo de ROS
        self.fotos = {P.PREVIEW: dict(topic=args.image_topic, max_w=0),       # una suscripcion por camara
                      P.HD: dict(topic=args.hd_topic, max_w=args.hd_width)}
        for f in self.fotos.values():
            f.update(sub=None, esperan=[], t_pedido=0.0)
        self.last_log = {}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", args.port))
        self.sock.settimeout(0.2)

        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/battery_state", self.on_battery, qos_profile_sensor_data)
        self.dock_client = self.undock_client = None
        try:
            from irobot_create_msgs.action import Dock, Undock
            from irobot_create_msgs.msg import DockStatus
            self.Dock, self.Undock = Dock, Undock
            self.create_subscription(DockStatus, "/dock_status", self.on_dock, qos_profile_sensor_data)
            self.dock_client = ActionClient(self, Dock, "/dock")
            self.undock_client = ActionClient(self, Undock, "/undock")
        except ImportError:
            self.get_logger().warn("irobot_create_msgs no disponible: sin dock/undock ni estado del dock")

        self.create_timer(1.0 / CONTROL_HZ, self.control_step)
        self.create_timer(1.0 / TELEMETRY_STATUS_HZ, self.send_status)
        self.running = True
        threading.Thread(target=self.udp_loop, daemon=True).start()
        self.get_logger().info(
            f"Puente UDP en 0.0.0.0:{args.port} (clave '{args.key}'), max {args.max_linear:.2f} m/s, "
            f"freno a {args.stop_dist:.2f} m, offset LiDAR {args.lidar_offset_deg:+.0f} grados")

    # ---------- utilidades ----------

    def log_every(self, key, period, text, level="info"):
        now = time.monotonic()
        if now - self.last_log.get(key, 0.0) >= period:
            self.last_log[key] = now
            getattr(self.get_logger(), level)(text)

    def send(self, data, addr):
        try:
            self.sock.sendto(data, addr)
        except OSError as e:
            self.log_every(("send", addr), 5.0, f"No se pudo enviar a {addr}: {e}", "warn")

    def broadcast(self, data):
        with self.lock:
            addrs = list(self.pcs)
        for addr in addrs:
            self.send(data, addr)

    def reply(self, addr, req_id, ok, text):
        self.send(P.pack_event(self.token, req_id, ok, text), addr)

    # ---------- hilo UDP ----------

    def udp_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                kind, payload = P.unpack(data, self.token)
                self.on_packet(kind, payload, addr)
            except Exception as e:  # un paquete malo nunca tumba el hilo
                self.log_every(("bad", addr[0]), 10.0, f"Paquete ignorado de {addr}: {e}", "warn")

    def on_packet(self, kind, payload, addr):
        now = time.monotonic()
        with self.lock:
            if addr not in self.pcs and kind != P.BYE:
                self.get_logger().info(f"PC conectada: {addr[0]}:{addr[1]}")
            self.pcs[addr] = now

            if kind == P.BYE:
                self.pcs.pop(addr, None)
                if self.driver == addr:
                    self.driver = None
                self.get_logger().info(f"PC desconectada: {addr[0]}:{addr[1]}")
            elif kind == P.CMD:
                _, v, w = P.unpack_cmd(payload)
                if self.take_control(addr, now):
                    self.cmd, self.cmd_time = (v, w), now
            elif kind == P.PHOTO:
                req_id, quality, camara = P.unpack_photo(payload)
                self.pending.append((P.PHOTO, addr, req_id, (quality, camara)))
            elif kind in (P.DOCK, P.UNDOCK):
                req_id = P.unpack_req(payload)
                if self.take_control(addr, now):
                    self.pending.append((kind, addr, req_id, None))
                else:
                    self.pending.append((None, addr, req_id, "otra PC esta conduciendo"))

    def take_control(self, addr, now):
        """Llamar con self.lock tomado. True si addr puede conducir."""
        if self.driver not in (None, addr) and now - self.cmd_time < DRIVER_RELEASE:
            self.log_every(("busy", addr), 5.0, f"{addr[0]} quiere conducir pero conduce {self.driver[0]}", "warn")
            return False
        if self.driver != addr:
            self.get_logger().info(f"Conduce ahora: {addr[0]}:{addr[1]}")
            self.driver = addr
        return True

    # ---------- callbacks de ROS ----------

    def on_scan(self, msg):
        angle_min = P.scan_angles_to_base(msg.angle_min, self.args.lidar_offset_deg)
        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = angle_min + msg.angle_increment * np.arange(n, dtype=np.float32)
        front, rear = P.corridor_clearance(angles, ranges, self.args.half_width,
                                           max(msg.range_min, 0.15))
        with self.lock:
            self.front, self.rear, self.scan_time = front, rear, time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.broadcast(P.pack_scan(self.token, stamp, angle_min, msg.angle_increment,
                                   msg.range_min, msg.range_max, ranges))

    def on_odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.broadcast(P.pack_odom(self.token, stamp, p.x, p.y, yaw, t.linear.x, t.angular.z))

    def on_battery(self, msg):
        self.battery = float(msg.percentage)

    def on_dock(self, msg):
        self.docked = int(bool(msg.is_docked))

    # ---------- control a 20 Hz ----------

    def control_step(self):
        now = time.monotonic()
        with self.lock:
            for addr in [a for a, t in self.pcs.items() if now - t > CLIENT_TIMEOUT]:
                del self.pcs[addr]
                self.get_logger().info(f"PC sin respuesta, se olvida: {addr[0]}:{addr[1]}")
                if self.driver == addr:
                    self.driver = None
            active = self.driver is not None and now - self.cmd_time < self.args.cmd_timeout
            v, w = self.cmd if active else (0.0, 0.0)
            scan_age = None if self.scan_time is None else now - self.scan_time
            front, rear = self.front, self.rear
            pending, self.pending = self.pending, []

        v_out, w_out, flags = self.safety.apply(v, w, front, rear, scan_age)
        if flags != self.flags and (flags & ~self.flags):
            names = [n for b, n in ((P.FLAG_FRONT_BLOCKED, "frente bloqueado"),
                                    (P.FLAG_REAR_BLOCKED, "atras bloqueado"),
                                    (P.FLAG_SCAN_STALE, "sin LiDAR")) if flags & b]
            self.get_logger().info("Freno de seguridad: " + ", ".join(names))
        self.flags = flags

        if active and (v != 0.0 or w != 0.0):
            self.publish(v_out, w_out)
            self.moving, self.stop_left = True, STOP_REPEAT
        elif self.moving or self.stop_left > 0:
            self.publish(0.0, 0.0)
            self.moving = False
            self.stop_left -= 1

        for kind, addr, req_id, extra in pending:
            if kind == P.PHOTO:
                self.start_photo(addr, req_id, *extra)
            elif kind in (P.DOCK, P.UNDOCK):
                self.start_dock(kind, addr, req_id)
            else:
                self.reply(addr, req_id, False, extra)
        self.check_photo_timeout()

    def publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)

    def send_status(self):
        now = time.monotonic()
        stamp = time.time()
        with self.lock:
            clients, driver = list(self.pcs), self.driver
            front, rear = self.front, self.rear
        for addr in clients:
            self.send(P.pack_status(self.token, stamp, self.battery, self.docked, self.flags,
                                    addr == driver, front, rear, len(clients)), addr)

    # ---------- foto bajo demanda ----------

    def start_photo(self, addr, req_id, quality, camara=P.PREVIEW):
        f = self.fotos.get(camara)
        if f is None or not f["topic"]:
            self.reply(addr, req_id, False, f"camara {camara} no configurada en el puente")
            return
        now = time.monotonic()
        f["esperan"].append((addr, req_id, quality, now))
        f["t_pedido"] = now
        if f["sub"] is None:
            comprimida = f["topic"].endswith("/compressed")
            tipo = CompressedImage if comprimida else Image
            f["sub"] = self.create_subscription(tipo, f["topic"], lambda m, c=camara: self.on_image(c, m),
                                                qos_profile_sensor_data)
            self.get_logger().info(f"Camara {f['topic']}: suscripcion abierta")

    def on_image(self, camara, msg):
        f = self.fotos[camara]
        if not f["esperan"]:
            return                                             # suscripcion abierta esperando el proximo pedido
        esperan, f["esperan"] = f["esperan"], []
        quality = max(q for _, _, q, _ in esperan)
        try:
            data, ancho, alto = self.encode(msg, quality, f["max_w"])
        except Exception as e:
            for addr, req_id, _, _ in esperan:
                self.reply(addr, req_id, False, f"error al codificar la imagen: {e}")
            return
        for addr, req_id, _, _ in esperan:
            for chunk in P.pack_img_chunks(self.token, req_id, data):
                self.send(chunk, addr)
        self.log_every(("foto", camara), 10.0, f"Foto {ancho}x{alto} ({len(data) // 1024} KB) enviada")

    @staticmethod
    def encode(msg, quality, max_w=0):
        """Mensaje de imagen de ROS -> (JPEG, ancho, alto). Un JPEG ya comprimido que no hay que achicar
        se reenvia tal cual (sin decodificar en la Raspberry)."""
        import cv2
        if isinstance(msg, CompressedImage):
            data = bytes(msg.data)
            if "jpeg" in msg.format.lower() and not max_w:
                return data, 0, 0
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"no se pudo decodificar ({msg.format})")
        else:
            img = imgmsg_a_bgr(msg)
        if max_w and img.shape[1] > max_w:
            img = cv2.resize(img, (max_w, int(img.shape[0] * max_w / img.shape[1])), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if not ok:
            raise RuntimeError("cv2.imencode fallo")
        return jpeg.tobytes(), img.shape[1], img.shape[0]

    def check_photo_timeout(self):
        now = time.monotonic()
        for camara, f in self.fotos.items():
            vencidas = [e for e in f["esperan"] if now - e[3] > PHOTO_TIMEOUT]
            if vencidas:
                f["esperan"] = [e for e in f["esperan"] if now - e[3] <= PHOTO_TIMEOUT]
                msg = (f"sin imagen de {f['topic']} (en el dock la camara esta apagada por power_saver"
                       + ("; para HD lanza la camara con task-a/camara/camara_rgb.sh --hd)" if camara == P.HD else ")"))
                for addr, req_id, _, _ in vencidas:
                    self.reply(addr, req_id, False, msg)
            if f["sub"] is not None and not f["esperan"] and now - f["t_pedido"] > self.args.foto_mantener:
                self.destroy_subscription(f["sub"])
                f["sub"] = None
                self.get_logger().info(f"Camara {f['topic']}: suscripcion cerrada (sin pedidos)")

    # ---------- dock / undock ----------

    def start_dock(self, kind, addr, req_id):
        client = self.dock_client if kind == P.DOCK else self.undock_client
        name = "dock" if kind == P.DOCK else "undock"
        if client is None or not client.server_is_ready():
            self.reply(addr, req_id, False, f"accion /{name} no disponible")
            return
        goal = self.Dock.Goal() if kind == P.DOCK else self.Undock.Goal()
        self.get_logger().info(f"{name} pedido por {addr[0]}")
        future = client.send_goal_async(goal)

        def on_goal(f):
            handle = f.result()
            if not handle.accepted:
                self.reply(addr, req_id, False, f"{name} rechazado")
                return
            handle.get_result_async().add_done_callback(
                lambda r: self.reply(addr, req_id, r.result().status == 4,  # 4 = SUCCEEDED
                                     f"{name} terminado (estado {r.result().status})"))
        future.add_done_callback(on_goal)

    def shutdown(self):
        self.running = False
        for _ in range(STOP_REPEAT):
            self.publish(0.0, 0.0)
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description="Puente UDP del TurtleBot 4 (grupo 3)")
    ap.add_argument("--port", type=int, default=P.DEFAULT_PORT)
    ap.add_argument("--key", default=P.DEFAULT_KEY, help="clave compartida con las PCs del grupo")
    ap.add_argument("--max-linear", type=float, default=0.31, help="m/s (tope de fabrica 0.31)")
    ap.add_argument("--max-angular", type=float, default=1.9, help="rad/s")
    ap.add_argument("--stop-dist", type=float, default=0.28, help="m: freno por obstaculo en el pasillo")
    ap.add_argument("--slow-dist", type=float, default=0.60, help="m: empieza a frenar progresivamente")
    ap.add_argument("--half-width", type=float, default=0.20, help="m: medio ancho del pasillo vigilado")
    ap.add_argument("--cmd-timeout", type=float, default=0.3, help="s sin comandos -> parar")
    ap.add_argument("--lidar-offset-deg", type=float, default=90.0,
                    help="giro de rplidar_link respecto a base_link (TF medido: +90)")
    ap.add_argument("--image-topic", default="/oakd/rgb/preview/image_raw", help="camara de las fotos normales")
    ap.add_argument("--hd-topic", default="/oakd/rgb/image_raw",
                    help="camara de las fotos HD (Image, o CompressedImage si termina en /compressed)")
    ap.add_argument("--hd-width", type=int, default=1280, help="ancho maximo de las fotos HD (0 = sin achicar)")
    ap.add_argument("--foto-mantener", type=float, default=3.0,
                    help="s que la suscripcion a la camara sigue abierta despues del ultimo pedido")
    args = ap.parse_args()

    rclpy.init()
    node = Bridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
