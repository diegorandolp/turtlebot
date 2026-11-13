"""
Simulador del TurtleBot 4 con LiDAR y CAMARA renderizada (Tarea A, carrera-1). Sirve para probar
todo el recorrido A -> B -> A sin robot: navegacion, flechas, rostro y reconocimiento.

Expone la misma API que tb4.client.TB4Client (scan, pose, status, move, stop, photo, undock,
wait_ready) con tiempo simulado: quien lo usa llama a advance(dt).

El mundo (mundos.py) son paredes y cajas verticales, cilindros y carteles (texturas: senales de
flecha, la foto de la persona en B). La camara es un raycaster por columnas (como los juegos 3D de
los 90): para cada columna de la imagen se lanza un rayo, se ordenan las superficies que corta y se
pintan de la mas lejana a la mas cercana con su altura real. Asi una caja baja tapa solo la parte de
abajo de la pared de atras y un cartel a ras del suelo se ve como lo veria la OAK-D (camara a ~13 cm
del suelo, mirando horizontal). Luego se agrega desenfoque, ruido y JPEG.

Del robot real se copia: el freno del puente (tb4.protocol.SafetyFilter con el pasillo de +-0.20 m),
aceleraciones limitadas, ruido de odometria, choque con paredes (se cuentan los choques).
"""
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))           # raiz del repo: tb4
from tb4 import protocol as P                                           # noqa: E402
from tb4.client import Pose, Scan, Status                               # noqa: E402

ROBOT_R = 0.17          # m, radio del Create 3
LIDAR_Z = 0.17          # m, altura del plano del LiDAR
CAM_Z = 0.13            # m, altura de la camara


# ---------------------------------------------------------------- mundo

@dataclass
class Pared:
    a: tuple                    # (x, y) extremos de la base
    b: tuple
    alto: float = 0.5
    color: tuple = (200, 200, 205)          # BGR


@dataclass
class Cilindro:
    c: tuple
    r: float
    alto: float = 0.5
    color: tuple = (90, 90, 160)


@dataclass
class Cartel:
    """Rectangulo vertical con textura, visible solo por delante (su normal apunta a la izquierda de a->b)."""
    a: tuple
    b: tuple
    z0: float
    z1: float
    textura: np.ndarray         # BGR
    tipo: str = ""              # "rostro" | "flecha:der" | ...


@dataclass
class Mundo:
    paredes: list
    inicio: tuple                           # (x, y, yaw) del punto A
    carteles: list = field(default_factory=list)
    cilindros: list = field(default_factory=list)
    b: tuple | None = None                  # (x, y) del cartel del rostro (punto B)
    persona: str = ""                       # nombre de la persona del cartel
    nombre: str = ""
    luz: float = 1.0                        # ganancia global de la iluminacion
    semilla: int = 0
    foto_b: str = ""                        # 'Carpeta/foto.jpg' de la galeria usada en el cartel


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _hits_segmentos(ox, oy, dx, dy, A, B):
    """Rayos (R,) contra segmentos (N,): distancia t (R, N) (inf si no cortan) y parametro s en el segmento."""
    ex, ey = B[:, 0] - A[:, 0], B[:, 1] - A[:, 1]
    den = _cross(dx[:, None], dy[:, None], ex[None, :], ey[None, :])
    qx, qy = A[:, 0] - ox, A[:, 1] - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        t = _cross(qx[None, :], qy[None, :], ex[None, :], ey[None, :]) / den
        s = _cross(qx[None, :], qy[None, :], dx[:, None], dy[:, None]) / den
    ok = (np.abs(den) > 1e-12) & (t > 1e-4) & (s >= 0) & (s <= 1)
    return np.where(ok, t, np.inf), s


def _hits_cilindros(ox, oy, dx, dy, C, r):
    fx, fy = C[:, 0] - ox, C[:, 1] - oy
    b = dx[:, None] * fx[None, :] + dy[:, None] * fy[None, :]
    c = fx ** 2 + fy ** 2 - r ** 2
    disc = b ** 2 - c[None, :]
    t = b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc >= 0) & (t > 1e-4), t, np.inf)


class Geometria:
    """Superficies del mundo en arrays, para lanzar rayos rapido."""

    def __init__(self, mundo):
        self.m = mundo
        ps = mundo.paredes
        self.A = np.array([p.a for p in ps], float).reshape(-1, 2)
        self.B = np.array([p.b for p in ps], float).reshape(-1, 2)
        self.alto = np.array([p.alto for p in ps], float)
        self.color = np.array([p.color for p in ps], float).reshape(-1, 3)
        cs = mundo.cilindros
        self.C = np.array([c.c for c in cs], float).reshape(-1, 2)
        self.Cr = np.array([c.r for c in cs], float)
        self.Calto = np.array([c.alto for c in cs], float)
        self.Ccolor = np.array([c.color for c in cs], float).reshape(-1, 3)
        ks = mundo.carteles
        self.KA = np.array([k.a for k in ks], float).reshape(-1, 2)
        self.KB = np.array([k.b for k in ks], float).reshape(-1, 2)
        e = self.KB - self.KA
        self.Kn = np.stack([-e[:, 1], e[:, 0]], 1) / np.maximum(np.linalg.norm(e, axis=1, keepdims=True), 1e-9)
        # para la colision y el LiDAR solo cuentan las superficies mas altas que el plano del LiDAR
        self.lidar_seg = self.alto >= LIDAR_Z
        self.lidar_cil = self.Calto >= LIDAR_Z

    def lidar(self, x, y, angulos, rmax=12.0):
        dx, dy = np.cos(angulos), np.sin(angulos)
        t = np.full(len(angulos), np.inf)
        if self.lidar_seg.any():
            ts, _ = _hits_segmentos(x, y, dx, dy, self.A[self.lidar_seg], self.B[self.lidar_seg])
            t = np.minimum(t, ts.min(1))
        if self.lidar_cil.any():
            t = np.minimum(t, _hits_cilindros(x, y, dx, dy, self.C[self.lidar_cil], self.Cr[self.lidar_cil]).min(1))
        return np.where(t <= rmax, t, np.inf)

    def distancia_min(self, x, y):
        """Distancia del punto (x, y) a la superficie solida mas cercana (para choques)."""
        d = np.inf
        if len(self.A):
            e = self.B - self.A
            L2 = np.maximum((e ** 2).sum(1), 1e-12)
            s = np.clip(((x - self.A[:, 0]) * e[:, 0] + (y - self.A[:, 1]) * e[:, 1]) / L2, 0, 1)
            px, py = self.A[:, 0] + s * e[:, 0], self.A[:, 1] + s * e[:, 1]
            dd = np.hypot(px - x, py - y)
            d = min(d, float(dd[self.alto >= 0.05].min())) if (self.alto >= 0.05).any() else d
        if len(self.C):
            d = min(d, float((np.hypot(self.C[:, 0] - x, self.C[:, 1] - y) - self.Cr).min()))
        return d


# ---------------------------------------------------------------- camara

class Camara:
    """Camara pinhole horizontal a CAM_Z del suelo. hfov en grados (preview de la OAK-D ~ 40)."""

    def __init__(self, geo, ancho=250, alto=250, hfov=40.0, capas=6, semilla=0):
        self.geo, self.W, self.H, self.K = geo, ancho, alto, capas
        self.f = (ancho / 2) / math.tan(math.radians(hfov) / 2)
        self.cx, self.cy = (ancho - 1) / 2, (alto - 1) / 2
        self.off = np.arctan((self.cx - np.arange(ancho)) / self.f)          # angulo de cada columna (+ = izquierda)
        rng = np.random.default_rng(semilla)
        self.rng = rng
        self._fondo = self._panorama(rng)
        self._suelo = rng.normal(0, 6, (64, 64)).astype(np.float32)
        self.v = np.arange(alto, dtype=np.float32)[:, None]

    @staticmethod
    def _panorama(rng, ancho=1440, alto=200):
        """Lo que hay mas alla del circuito (el aula): bandas y rectangulos al azar, sin flechas ni caras."""
        img = np.full((alto, ancho, 3), 215, np.uint8)
        img[:] = (rng.integers(190, 230), rng.integers(190, 230), rng.integers(190, 230))
        for _ in range(70):
            x, w = int(rng.integers(0, ancho)), int(rng.integers(8, 120))
            y0 = int(rng.integers(alto // 3, alto - 20))
            col = tuple(int(c) for c in rng.integers(40, 230, 3))
            cv2.rectangle(img, (x, y0), (x + w, alto), col, -1)
        for _ in range(25):                                                     # patas de mesas y sillas
            x = int(rng.integers(0, ancho))
            cv2.line(img, (x, int(rng.integers(alto // 2, alto - 10))), (x, alto), (40, 40, 40), int(rng.integers(2, 6)))
        return img

    def render(self, x, y, yaw):
        g, W, H, f, cy = self.geo, self.W, self.H, self.f, self.cy
        ang = yaw + self.off
        dx, dy = np.cos(ang), np.sin(ang)
        cosoff = np.cos(self.off)
        img = np.empty((H, W, 3), np.float32)
        # fondo: panorama del aula arriba, suelo abajo
        az = ((ang % (2 * math.pi)) / (2 * math.pi) * self._fondo.shape[1]).astype(int) % self._fondo.shape[1]
        horizonte = int(round(cy))
        filas = np.clip(((np.arange(H) / max(horizonte, 1)) * (self._fondo.shape[0] - 1)).astype(int), 0, self._fondo.shape[0] - 1)
        img[:] = self._fondo[filas[:, None], az[None, :]]
        vv = self.v[horizonte + 1:]
        dep = f * CAM_Z / np.maximum(vv - cy, 0.5)                              # distancia al suelo en cada fila
        gx = (x + dep / cosoff[None, :] * dx[None, :])
        gy = (y + dep / cosoff[None, :] * dy[None, :])
        tex = self._suelo[(gx * 4).astype(int) % 64, (gy * 4).astype(int) % 64]
        base = 150 + 25 * np.exp(-dep / 3.0) + tex
        img[horizonte + 1:] = np.stack([base * 0.98, base, base * 1.02], -1)
        # superficies: todas las que corta cada rayo, de la mas lejana a la mas cercana
        capas = []                                                              # (t, tipo, indice, s)
        if len(g.A):
            t, s = _hits_segmentos(x, y, dx, dy, g.A, g.B)
            for i in range(t.shape[1]):
                capas.append((t[:, i], 0, i, s[:, i]))
        if len(g.C):
            t = _hits_cilindros(x, y, dx, dy, g.C, g.Cr)
            for i in range(t.shape[1]):
                capas.append((t[:, i], 1, i, None))
        if len(g.KA):
            t, s = _hits_segmentos(x, y, dx, dy, g.KA, g.KB)
            frente = (dx[:, None] * g.Kn[None, :, 0] + dy[:, None] * g.Kn[None, :, 1]) < 0
            t = np.where(frente, t - 0.003, np.inf)                             # el cartel va pegado delante de la pared
            for i in range(t.shape[1]):
                capas.append((t[:, i], 2, i, s[:, i]))
        if capas:
            T = np.stack([c[0] for c in capas], 1)                              # (W, n)
            orden = np.argsort(T, axis=1)[:, :self.K]
            for k in range(min(self.K, T.shape[1]) - 1, -1, -1):
                idx = orden[:, k]
                tk = T[np.arange(W), idx]
                for j in np.unique(idx[np.isfinite(tk)]):
                    cols = (idx == j) & np.isfinite(tk)
                    self._pintar(img, capas[j], cols, tk, cosoff)
        img *= self.geo.m.luz
        return img

    def _pintar(self, img, capa, cols, t, cosoff):
        _, tipo, i, s = capa
        g, f, cy = self.geo, self.f, self.cy
        dep = (t * cosoff)[cols]                                                # profundidad sobre el eje optico
        if tipo == 2:
            k = g.m.carteles[i]
            z0, z1 = k.z0, k.z1
        else:
            z0, z1 = 0.0, (g.alto[i] if tipo == 0 else g.Calto[i])
        v_top = cy - f * (z1 - CAM_Z) / dep
        v_bot = cy - f * (z0 - CAM_Z) / dep
        cidx = np.nonzero(cols)[0]
        m = (self.v >= v_top[None, :]) & (self.v <= v_bot[None, :])             # (H, ncols)
        if not m.any():
            return
        if tipo == 2:
            tex = g.m.carteles[i].textura
            th, tw = tex.shape[:2]
            z = CAM_Z + (cy - self.v) * dep[None, :] / f
            tv = np.clip(((z1 - z) / (z1 - z0) * (th - 1)).astype(int), 0, th - 1)
            tu = np.clip(((1.0 - s[cols]) * (tw - 1)).astype(int), 0, tw - 1)   # de frente, a->b va de derecha a izquierda
            tu = np.broadcast_to(tu[None, :], tv.shape)
            luz = 0.92 + 0.08 * np.exp(-dep / 2.0)
            sub = img[:, cidx]
            sub[m] = tex[tv[m], tu[m]].astype(np.float32) * np.broadcast_to(luz[None, :], m.shape)[m][:, None]
            img[:, cidx] = sub
            return
        if tipo == 0:
            e = g.B[i] - g.A[i]
            n = np.array([-e[1], e[0]]) / max(np.linalg.norm(e), 1e-9)
            ang = (self.geo.m.inicio[2] + 0.7)                                  # luz fija del mundo
            sombra = 0.75 + 0.25 * abs(math.cos(math.atan2(n[1], n[0]) - ang))
            col = g.color[i] * sombra
        else:
            col = g.Ccolor[i] * 0.9
        niebla = np.exp(-dep / 12.0)
        c = col[None, :] * (0.85 + 0.15 * niebla[:, None])                     # (ncols, 3)
        # borde inferior mas oscuro (zocalo / sombra de contacto con el suelo)
        z = CAM_Z + (cy - self.v) * dep[None, :] / f
        osc = np.where(z < 0.03 * (z1 - z0) + 0.01, 0.6, 1.0)
        sub = img[:, cidx]
        val = c[None, :, :] * osc[:, :, None]
        sub[m] = val[m]
        img[:, cidx] = sub

    def foto(self, x, y, yaw, calidad=85, ruido=3.0, desenfoque=0.6):
        img = self.render(x, y, yaw)
        img = cv2.GaussianBlur(img, (0, 0), desenfoque)
        img += self.rng.normal(0, ruido, img.shape).astype(np.float32)
        img = np.clip(img, 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, calidad])
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)


# ---------------------------------------------------------------- robot

class SimRobot:
    """Mismo API que TB4Client, con tiempo simulado (advance)."""

    def __init__(self, mundo, hfov=40.0, odom_ruido=0.01, odom_sesgo=0.0, semilla=1, camara_ok=True,
                 acel=1.0, acel_w=4.0, foto_hd=None, freno=True):
        """foto_hd = (ancho, alto, hfov) de la camara grande (photo(hd=True)); freno=False si delante va el
        puente real (robot_ros_sim.py), que ya aplica el freno."""
        self.freno = freno
        self.mundo = mundo
        self.geo = Geometria(mundo)
        self.cam = Camara(self.geo, hfov=hfov, semilla=mundo.semilla)
        self.cam_hd = Camara(self.geo, *foto_hd, semilla=mundo.semilla) if foto_hd else None
        self.camara_ok = camara_ok
        self.wx, self.wy, self.wth = mundo.inicio
        self.ox = self.oy = self.oth = 0.0
        self.v = self.w = 0.0
        self.rng = np.random.default_rng(semilla)
        self.ruido, self.sesgo = odom_ruido, odom_sesgo
        self.acel, self.acel_w = acel, acel_w
        self.time = 0.0
        self.cmd, self.cmd_t = (0.0, 0.0), -1e9
        self.angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
        self.safety = P.SafetyFilter(P.SafetyConfig())
        self.flags = 0
        self.path = [(self.wx, self.wy)]
        self.collisions = 0
        self._en_choque = False
        self.fotos = 0
        self._scan_cache = None

    # ---- API del cliente real ----
    def wait_ready(self, timeout=5.0):
        return self.status()

    def undock(self, timeout=30.0):
        return "sim"

    def dock(self, timeout=90.0):
        return "sim"

    def move(self, v, w):
        self.cmd, self.cmd_t = (float(v), float(w)), self.time

    def stop(self):
        self.cmd, self.cmd_t = (0.0, 0.0), self.time

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()

    def _ranges(self):
        if self._scan_cache is None or self._scan_cache[0] != self.time:
            r = self.geo.lidar(self.wx, self.wy, self.angles + self.wth)
            self._scan_cache = (self.time, r)
        return self._scan_cache[1]

    def scan(self):
        r = self._ranges().copy()
        r = r + self.rng.normal(0, 0.008, r.shape)
        r[self.rng.random(r.shape) < 0.03] = np.nan                            # lecturas perdidas
        return Scan(self.angles.copy(), r.astype(np.float32), 12.0, self.time, time.monotonic())

    def pose(self):
        return Pose(self.ox, self.oy, self.oth, self.v, self.w, self.time, time.monotonic())

    def status(self):
        fr, rr = P.corridor_clearance(self.angles, self._ranges(), 0.20)
        return Status(0.9, False, bool(self.flags & P.FLAG_FRONT_BLOCKED), bool(self.flags & P.FLAG_REAR_BLOCKED),
                      False, True, fr, rr, 1, time.monotonic())

    def photo(self, timeout=4.0, quality=85, hd=False):
        if not self.camara_ok:
            raise RuntimeError("sin imagen de la camara (simulada caida)")
        self.fotos += 1
        cam = self.cam_hd if (hd and self.cam_hd is not None) else self.cam
        return cam.foto(self.wx, self.wy, self.wth, quality)

    # ---- simulacion ----
    def verdad(self):
        """Pose real (marco del mundo)."""
        return self.wx, self.wy, self.wth

    def advance(self, dt):
        v_cmd, w_cmd = self.cmd if self.time - self.cmd_t < 0.3 else (0.0, 0.0)
        fr, rr = P.corridor_clearance(self.angles, self._ranges(), 0.20)
        if self.freno:
            v_cmd, w_cmd, self.flags = self.safety.apply(v_cmd, w_cmd, fr, rr, 0.0)   # el freno del puente
        dv = np.clip(v_cmd - self.v, -self.acel * dt, self.acel * dt)
        dw = np.clip(w_cmd - self.w, -self.acel_w * dt, self.acel_w * dt)
        self.v, self.w = self.v + dv, self.w + dw
        nth = self.wth + self.w * dt
        nx = self.wx + self.v * math.cos(nth) * dt
        ny = self.wy + self.v * math.sin(nth) * dt
        if self.geo.distancia_min(nx, ny) < ROBOT_R:
            if not self._en_choque:
                self.collisions += 1
            self._en_choque = True
            nx, ny, self.v = self.wx, self.wy, 0.0
        else:
            self._en_choque = self.geo.distancia_min(nx, ny) < ROBOT_R + 0.005
        self.wx, self.wy, self.wth = nx, ny, nth
        # odometria (marco con origen en A): misma cinematica con error proporcional
        k = 1.0 + self.sesgo / 2 + (self.rng.normal(0, self.ruido) if self.ruido else 0.0)
        self.oth += self.w * dt * (1.0 + self.sesgo + (self.rng.normal(0, self.ruido * 0.5) if self.ruido else 0.0))
        self.ox += self.v * math.cos(self.oth) * dt * k
        self.oy += self.v * math.sin(self.oth) * dt * k
        self.time += dt
        if math.hypot(self.wx - self.path[-1][0], self.wy - self.path[-1][1]) > 0.03:
            self.path.append((self.wx, self.wy))

    # ---- vista desde arriba (para la ventana y para guardar el recorrido) ----
    def vista(self, px_m=60, margen=0.4, extra=None):
        g = self.geo
        pts = np.vstack([g.A, g.B, g.C.reshape(-1, 2) if len(g.C) else np.zeros((0, 2)), [self.mundo.inicio[:2]]])
        x0, y0 = pts.min(0) - margen
        x1, y1 = pts.max(0) + margen
        W, H = int((x1 - x0) * px_m), int((y1 - y0) * px_m)
        img = np.full((H, W, 3), 245, np.uint8)

        def px(x, y):
            return int((x - x0) * px_m), int(H - 1 - (y - y0) * px_m)

        for p in self.mundo.paredes:
            col = (60, 60, 60) if p.alto >= LIDAR_Z else (150, 150, 150)
            cv2.line(img, px(*p.a), px(*p.b), col, 2)
        for c in self.mundo.cilindros:
            cv2.circle(img, px(*c.c), int(c.r * px_m), (60, 60, 60), 2)
        for k in self.mundo.carteles:
            col = (255, 0, 255) if k.tipo == "rostro" else (0, 140, 255)
            cv2.line(img, px(*k.a), px(*k.b), col, 4)
        for a, b in zip(self.path, self.path[1:]):
            cv2.line(img, px(*a), px(*b), (200, 120, 0), 2)
        sx, sy, _ = self.mundo.inicio
        cv2.drawMarker(img, px(sx, sy), (0, 160, 0), cv2.MARKER_DIAMOND, 14, 2)
        p = px(self.wx, self.wy)
        cv2.circle(img, p, int(ROBOT_R * px_m), (0, 0, 255), 2)
        cv2.line(img, p, (int(p[0] + 20 * math.cos(self.wth)), int(p[1] - 20 * math.sin(self.wth))), (0, 0, 255), 2)
        if extra is not None:
            extra(img, px)
        return img
