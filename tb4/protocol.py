"""
Protocolo UDP entre la PC y el puente del TurtleBot 4 (grupo 3).

Lo usan los dos lados: robot/tb4_bridge.py (en el robot) y tb4/client.py (en la PC).
Solo depende de numpy, para poder copiarlo tal cual a la Raspberry.

Formato de cada datagrama (little-endian):
  cabecera  = MAGIC(4) | VERSION(u8) | tipo(u8) | token(8)
  token     = primeros 8 bytes de sha256(clave del grupo); evita que el robot obedezca
              paquetes de otro grupo que use el mismo puerto (no es seguridad real).

PC -> robot:  HELLO, BYE, CMD(seq, v, w), PHOTO(req, calidad), DOCK(req), UNDOCK(req)
Robot -> PC:  SCAN, ODOM, STATUS, IMG (trozos de un JPEG), EVENT (respuestas)

Convencion de ejes (base_link del robot): x adelante, y izquierda, angulos en radianes,
0 = frente, positivo = izquierda (antihorario). El puente ya aplica el giro del LiDAR.
"""
import hashlib
import math
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"G3TB"
VERSION = 1
DEFAULT_PORT = 5600
DEFAULT_KEY = "grupo3"

# Tipos de paquete
HELLO, BYE, CMD, PHOTO, DOCK, UNDOCK = b"H", b"B", b"C", b"P", b"D", b"U"
SCAN, ODOM, STATUS, IMG, EVENT = b"s", b"o", b"t", b"i", b"e"

_HEADER = struct.Struct("<4sBc8s")
_CMD = struct.Struct("<Iff")              # seq, v (m/s), w (rad/s)
_REQ = struct.Struct("<I")                # id de peticion
_PHOTO = struct.Struct("<IB")             # id, calidad JPEG
_SCAN = struct.Struct("<dfffH")           # stamp, angle_min, angle_inc, range_max, n
_ODOM = struct.Struct("<dfffff")          # stamp, x, y, yaw, v, w
_STATUS = struct.Struct("<dfBBBffB")      # stamp, bateria, docked, flags, driver, front, rear, n_clientes
_IMG = struct.Struct("<IHH")              # id, indice del trozo, total de trozos
_EVENT = struct.Struct("<IB")             # id, ok + texto utf-8

HEADER_SIZE = _HEADER.size
IMG_CHUNK = 1200          # bytes de JPEG por datagrama (cabe en un paquete Wi-Fi sin fragmentar)
RANGE_INVALID = 0         # lectura invalida (nan o menor que range_min)
RANGE_FAR = 65535         # sin obstaculo dentro del alcance (inf)

# Bits de STATUS.flags
FLAG_FRONT_BLOCKED = 1
FLAG_REAR_BLOCKED = 2
FLAG_SCAN_STALE = 4
DOCK_UNKNOWN = 255


class ProtocolError(ValueError):
    pass


def make_token(key):
    return hashlib.sha256(key.encode("utf-8")).digest()[:8]


def pack(kind, token, payload=b""):
    return _HEADER.pack(MAGIC, VERSION, kind, token) + payload


def unpack(data, token):
    """Devuelve (tipo, payload) o lanza ProtocolError si el paquete no es nuestro."""
    if len(data) < HEADER_SIZE:
        raise ProtocolError("paquete demasiado corto")
    magic, version, kind, tok = _HEADER.unpack_from(data)
    if magic != MAGIC or version != VERSION:
        raise ProtocolError("magic/version distintos")
    if tok != token:
        raise ProtocolError("token de otro grupo")
    return kind, data[HEADER_SIZE:]


# ---------- PC -> robot ----------

def pack_cmd(token, seq, v, w):
    return pack(CMD, token, _CMD.pack(seq & 0xFFFFFFFF, v, w))


def unpack_cmd(payload):
    seq, v, w = _CMD.unpack_from(payload)
    if not (math.isfinite(v) and math.isfinite(w)):
        raise ProtocolError("velocidad no finita")
    return seq, v, w


def pack_photo(token, req_id, quality=80):
    return pack(PHOTO, token, _PHOTO.pack(req_id, max(1, min(100, int(quality)))))


def unpack_photo(payload):
    return _PHOTO.unpack_from(payload)


def pack_req(kind, token, req_id):
    return pack(kind, token, _REQ.pack(req_id))


def unpack_req(payload):
    return _REQ.unpack_from(payload)[0]


# ---------- Robot -> PC ----------

def encode_ranges(ranges, range_min, range_max):
    """float (m) -> uint16 (mm); 0 = invalida, 65535 = sin obstaculo."""
    r = np.asarray(ranges, dtype=np.float64)
    mm = np.clip(np.round(r * 1000.0), 1, RANGE_FAR - 1)
    mm = np.where(np.isfinite(r), mm, RANGE_FAR)
    mm = np.where(np.isnan(r) | (r < range_min), RANGE_INVALID, mm)
    mm = np.where(np.isfinite(r) & (r > range_max), RANGE_FAR, mm)
    return mm.astype("<u2")


def decode_ranges(mm):
    r = mm.astype(np.float32) / 1000.0
    r[mm == RANGE_INVALID] = np.nan
    r[mm == RANGE_FAR] = np.inf
    return r


def pack_scan(token, stamp, angle_min, angle_inc, range_min, range_max, ranges):
    mm = encode_ranges(ranges, range_min, range_max)
    return pack(SCAN, token, _SCAN.pack(stamp, angle_min, angle_inc, range_max, len(mm)) + mm.tobytes())


def unpack_scan(payload):
    stamp, angle_min, angle_inc, range_max, n = _SCAN.unpack_from(payload)
    body = payload[_SCAN.size:]
    if len(body) != 2 * n:
        raise ProtocolError("scan truncado")
    ranges = decode_ranges(np.frombuffer(body, dtype="<u2"))
    angles = angle_min + angle_inc * np.arange(n, dtype=np.float32)
    return stamp, angles, ranges, range_max


def pack_odom(token, stamp, x, y, yaw, v, w):
    return pack(ODOM, token, _ODOM.pack(stamp, x, y, yaw, v, w))


def unpack_odom(payload):
    return _ODOM.unpack_from(payload)


def pack_status(token, stamp, battery, docked, flags, is_driver, front, rear, n_clients):
    return pack(STATUS, token, _STATUS.pack(stamp, battery, docked, flags, int(is_driver),
                                            front, rear, min(n_clients, 255)))


def unpack_status(payload):
    return _STATUS.unpack_from(payload)


def pack_img_chunks(token, req_id, jpeg):
    """Parte un JPEG en datagramas pequenos; el cliente los vuelve a unir."""
    chunks = [jpeg[i:i + IMG_CHUNK] for i in range(0, len(jpeg), IMG_CHUNK)] or [b""]
    if len(chunks) > 0xFFFF:
        raise ProtocolError("imagen demasiado grande")
    return [pack(IMG, token, _IMG.pack(req_id, i, len(chunks)) + c) for i, c in enumerate(chunks)]


def unpack_img(payload):
    req_id, idx, total = _IMG.unpack_from(payload)
    return req_id, idx, total, payload[_IMG.size:]


def pack_event(token, req_id, ok, text):
    return pack(EVENT, token, _EVENT.pack(req_id, int(ok)) + text.encode("utf-8")[:1000])


def unpack_event(payload):
    req_id, ok = _EVENT.unpack_from(payload)
    return req_id, bool(ok), payload[_EVENT.size:].decode("utf-8", errors="replace")


# ---------- Geometria y seguridad (comun a robot y PC) ----------

def scan_angles_to_base(angle_min, offset_deg):
    """Angulo del LiDAR -> angulo en base_link (el LiDAR del TB4 esta girado +90 grados)."""
    return angle_min + math.radians(offset_deg)


def corridor_clearance(angles, ranges, half_width, min_valid=0.15):
    """
    Distancia libre (m) hacia adelante y hacia atras dentro del pasillo que ocupa el robot.

    Mide sobre el eje x los puntos con |y| < half_width, es decir, lo que el robot
    golpearia si avanza o retrocede en linea recta. Devuelve (frente, atras); inf = libre.
    """
    a = np.asarray(angles, dtype=np.float64)
    r = np.asarray(ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r >= min_valid)
    x = r[ok] * np.cos(a[ok])
    y = r[ok] * np.sin(a[ok])
    inside = np.abs(y) < half_width
    ahead = x[inside & (x > 0)]
    behind = -x[inside & (x < 0)]
    front = float(ahead.min()) if ahead.size else math.inf
    rear = float(behind.min()) if behind.size else math.inf
    return front, rear


@dataclass
class SafetyConfig:
    max_linear: float = 0.31      # m/s, tope de fabrica del Create 3
    max_angular: float = 1.9      # rad/s, tope del TurtleBot 4
    stop_dist: float = 0.28       # m desde el centro del LiDAR (~11 cm desde el parachoques)
    clear_dist: float = 0.38      # histeresis: vuelve a permitir avanzar por encima de esto
    slow_dist: float = 0.60       # por debajo, la velocidad maxima baja en proporcion
    min_slow_speed: float = 0.05  # velocidad minima permitida dentro de la zona lenta
    scan_timeout: float = 0.5     # s; sin LiDAR reciente no se permite avance ni retroceso


class SafetyFilter:
    """Limita (v, w) segun el LiDAR. Los giros en el sitio siempre se permiten."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.front_blocked = False
        self.rear_blocked = False

    def _update_block(self, blocked, dist):
        if blocked:
            return dist < self.cfg.clear_dist
        return dist < self.cfg.stop_dist

    def _limit(self, speed, dist, blocked):
        if blocked:
            return 0.0
        c = self.cfg
        if dist < c.slow_dist:
            frac = (dist - c.stop_dist) / max(1e-6, c.slow_dist - c.stop_dist)
            return min(speed, max(c.min_slow_speed, c.max_linear * frac))
        return speed

    def apply(self, v, w, front, rear, scan_age):
        """Devuelve (v, w, flags) con las restricciones aplicadas."""
        c = self.cfg
        v = max(-c.max_linear, min(c.max_linear, v))
        w = max(-c.max_angular, min(c.max_angular, w))
        flags = 0
        if scan_age is None or scan_age > c.scan_timeout:
            return 0.0, w, FLAG_SCAN_STALE
        self.front_blocked = self._update_block(self.front_blocked, front)
        self.rear_blocked = self._update_block(self.rear_blocked, rear)
        if self.front_blocked:
            flags |= FLAG_FRONT_BLOCKED
        if self.rear_blocked:
            flags |= FLAG_REAR_BLOCKED
        if v > 0:
            v = self._limit(v, front, self.front_blocked)
        elif v < 0:
            v = -self._limit(-v, rear, self.rear_blocked)
        return v, w, flags
