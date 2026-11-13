"""
Cliente UDP del TurtleBot 4 (grupo 3) para la PC. Python puro: no necesita ROS.

Requisitos: numpy, y opencv-python solo para photo(). En el robot debe estar corriendo
robot/tb4_bridge.py (lo arranca robot/tb4_inicio.sh).

Ejemplo:
    from tb4 import TB4Client

    with TB4Client("192.168.0.103") as robot:
        robot.wait_ready()
        print(robot.status())             # bateria, dock, frenos activos...
        scan = robot.scan()               # angulos (0 = frente, + = izquierda) y distancias (m)
        print(scan.sector_min(-15, 15))   # lo mas cercano al frente
        robot.move(0.1, 0.0)              # hay que llamarlo en bucle: si dejas de llamarlo,
                                          # el robot se para solo (~0.5 s)
        img = robot.photo()               # imagen BGR de la camara (numpy), o TimeoutError
        img = robot.photo(hd=True)        # la imagen grande (1280x720), si el puente y la camara la dan

Prueba rapida de conexion (desde la raiz del repositorio):
    python3 -m tb4.client 192.168.0.103
"""
import math
import socket
import threading
import time
from dataclasses import dataclass

import numpy as np

from . import protocol as P

HELLO_PERIOD = 1.0
SEND_HZ = 20
MOVE_HOLD = 0.5      # s: si el codigo deja de llamar move(), se deja de enviar el comando


@dataclass
class Scan:
    angles: np.ndarray    # rad en base_link: 0 = frente, + = izquierda
    ranges: np.ndarray    # m; nan = lectura invalida, inf = nada dentro del alcance
    range_max: float
    stamp: float          # hora del robot (s)
    received: float       # time.monotonic() de la PC al recibirlo

    @property
    def age(self):
        return time.monotonic() - self.received

    def points(self):
        """(x, y) en metros de las lecturas validas y finitas. x adelante, y izquierda."""
        ok = np.isfinite(self.ranges)
        return self.ranges[ok] * np.cos(self.angles[ok]), self.ranges[ok] * np.sin(self.angles[ok])

    def sector_min(self, deg_from, deg_to):
        """Distancia minima (m) entre dos angulos en grados (p. ej. -15, 15 = frente)."""
        a = np.degrees(self.angles)
        a = (a + 180.0) % 360.0 - 180.0
        sel = (a >= deg_from) & (a <= deg_to) & ~np.isnan(self.ranges)
        return float(np.min(self.ranges[sel])) if sel.any() else math.inf


@dataclass
class Pose:
    x: float              # m, odometria del Create 3 (origen = donde arranco el robot)
    y: float
    yaw: float            # rad
    v: float              # m/s medida
    w: float              # rad/s medida
    stamp: float
    received: float

    @property
    def age(self):
        return time.monotonic() - self.received


@dataclass
class Status:
    battery: float        # 0..1, -1 si no se sabe
    docked: bool | None
    front_blocked: bool   # el puente no deja avanzar (obstaculo al frente)
    rear_blocked: bool
    scan_stale: bool      # sin LiDAR: no deja avanzar ni retroceder (normal en el dock)
    is_driver: bool       # esta PC es la que conduce
    front_clear: float    # m libres en el pasillo del robot, hacia adelante
    rear_clear: float
    n_clients: int
    received: float

    @property
    def age(self):
        return time.monotonic() - self.received


class TB4Client:
    def __init__(self, ip, port=P.DEFAULT_PORT, key=P.DEFAULT_KEY):
        self.addr = (ip, port)
        self.token = P.make_token(key)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self._scan = self._pose = self._status = None
        self._cmd = (0.0, 0.0)
        self._cmd_time = 0.0
        self._seq = 0
        self._req = int(time.time()) & 0xFFFF
        self._waiting = {}       # req_id -> dict(event, result, chunks)
        self._running = True
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx.start()
        self._tx.start()

    # ---------- API ----------

    def wait_ready(self, timeout=5.0):
        """Espera el primer STATUS del robot. Lanza TimeoutError si no responde."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self._status is not None:
                return self._status
            time.sleep(0.05)
        raise TimeoutError(f"El puente no responde en {self.addr[0]}:{self.addr[1]}. "
                           "Esta corriendo tb4_bridge.py en el robot? Misma red? Misma clave?")

    def status(self):
        return self._status

    def scan(self):
        return self._scan

    def pose(self):
        return self._pose

    def move(self, v, w):
        """Velocidad lineal v (m/s, + adelante) y angular w (rad/s, + izquierda).
        Llamalo en cada iteracion de tu bucle; si dejas de hacerlo el robot se detiene."""
        with self.lock:
            self._cmd, self._cmd_time = (float(v), float(w)), time.monotonic()

    def stop(self):
        with self.lock:
            self._cmd, self._cmd_time = (0.0, 0.0), 0.0
        for _ in range(3):
            self._send(P.pack_cmd(self.token, self._next_seq(), 0.0, 0.0))

    def photo(self, timeout=4.0, quality=85, hd=False):
        """Pide una imagen a la camara del robot. Devuelve BGR (numpy) o lanza TimeoutError/RuntimeError.
        hd=True pide la imagen grande (ver robot/tb4_bridge.py --hd-topic); un puente viejo devuelve la
        preview de 250x250."""
        import cv2
        cam = P.HD if hd else P.PREVIEW
        req = self._request(lambda r: P.pack_photo(self.token, r, quality, cam), timeout)
        data = b"".join(req["chunks"][i] for i in range(req["total"]))
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("JPEG recibido no se pudo decodificar")
        return img

    def undock(self, timeout=30.0):
        """Saca el robot del dock (retrocede solo). Devuelve el mensaje del robot."""
        return self._request(lambda r: P.pack_req(P.UNDOCK, self.token, r), timeout)["text"]

    def dock(self, timeout=90.0):
        """Vuelve al dock (debe estar a la vista del robot)."""
        return self._request(lambda r: P.pack_req(P.DOCK, self.token, r), timeout)["text"]

    def close(self):
        if not self._running:
            return
        self.stop()
        self._send(P.pack(P.BYE, self.token))
        self._running = False
        self._rx.join(timeout=1.0)
        self._tx.join(timeout=1.0)
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---------- interno ----------

    def _next_seq(self):
        self._seq += 1
        return self._seq

    def _send(self, data):
        try:
            self.sock.sendto(data, self.addr)
        except OSError:
            pass

    def _request(self, build, timeout):
        with self.lock:
            self._req = (self._req + 1) & 0xFFFFFFFF
            req_id = self._req
            entry = {"event": threading.Event(), "ok": None, "text": "", "chunks": {}, "total": None}
            self._waiting[req_id] = entry
        try:
            self._send(build(req_id))
            if not entry["event"].wait(timeout):
                raise TimeoutError("el robot no respondio a tiempo")
            if not entry["ok"]:
                raise RuntimeError(entry["text"])
            return entry
        finally:
            with self.lock:
                self._waiting.pop(req_id, None)

    def _tx_loop(self):
        last_hello = 0.0
        period = 1.0 / SEND_HZ
        while self._running:
            now = time.monotonic()
            if now - last_hello >= HELLO_PERIOD:
                self._send(P.pack(P.HELLO, self.token))
                last_hello = now
            with self.lock:
                cmd, fresh = self._cmd, now - self._cmd_time < MOVE_HOLD
            if fresh:
                self._send(P.pack_cmd(self.token, self._next_seq(), *cmd))
            time.sleep(period)

    def _rx_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                kind, payload = P.unpack(data, self.token)
                self._handle(kind, payload)
            except Exception:
                continue

    def _handle(self, kind, payload):
        now = time.monotonic()
        if kind == P.SCAN:
            stamp, angles, ranges, range_max = P.unpack_scan(payload)
            self._scan = Scan(angles, ranges, range_max, stamp, now)
        elif kind == P.ODOM:
            stamp, x, y, yaw, v, w = P.unpack_odom(payload)
            self._pose = Pose(x, y, yaw, v, w, stamp, now)
        elif kind == P.STATUS:
            _, bat, docked, flags, driver, front, rear, n = P.unpack_status(payload)
            self._status = Status(
                battery=bat, docked=None if docked == P.DOCK_UNKNOWN else bool(docked),
                front_blocked=bool(flags & P.FLAG_FRONT_BLOCKED),
                rear_blocked=bool(flags & P.FLAG_REAR_BLOCKED),
                scan_stale=bool(flags & P.FLAG_SCAN_STALE), is_driver=bool(driver),
                front_clear=front, rear_clear=rear, n_clients=n, received=now)
        elif kind == P.IMG:
            req_id, idx, total, chunk = P.unpack_img(payload)
            with self.lock:
                entry = self._waiting.get(req_id)
                if entry is None:
                    return
                entry["total"] = total
                entry["chunks"][idx] = chunk
                if len(entry["chunks"]) == total:
                    entry["ok"] = True
                    entry["event"].set()
        elif kind == P.EVENT:
            req_id, ok, text = P.unpack_event(payload)
            with self.lock:
                entry = self._waiting.get(req_id)
                if entry is not None:
                    entry["ok"], entry["text"] = ok, text
                    entry["event"].set()


def _fmt(d):
    return "libre" if math.isinf(d) else f"{d:.2f} m"


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Prueba de conexion con el puente UDP del TB4")
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--port", type=int, default=P.DEFAULT_PORT)
    ap.add_argument("--key", default=P.DEFAULT_KEY)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    with TB4Client(args.ip, args.port, args.key) as robot:
        robot.wait_ready()
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            s, sc, po = robot.status(), robot.scan(), robot.pose()
            line = (f"bateria {s.battery * 100:.0f}% | dock {s.docked} | frente {_fmt(s.front_clear)}"
                    f" | atras {_fmt(s.rear_clear)} | PCs {s.n_clients}")
            if s.scan_stale:
                line += " | SIN LIDAR"
            if sc is not None:
                line += f" | scan {len(sc.ranges)} pts, edad {sc.age * 1000:.0f} ms"
            if po is not None:
                line += f" | pose ({po.x:.2f}, {po.y:.2f}, {math.degrees(po.yaw):.0f} grados)"
            print(line)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
