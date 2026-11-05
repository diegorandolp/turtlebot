"""
Nucleo del SLAM 2D (Tarea A, slam-1). Sin dependencias del robot: recibe objetos con los mismos
campos que tb4.client.Scan y tb4.client.Pose, asi se puede probar sin robot.

Dos motores con la misma interfaz:
  BreezyEngine  BreezySLAM (RMHC_SLAM): corrige la odometria con el LiDAR. Es el motor del plan.
  OdomEngine    Mapa por ocupacion usando SOLO la odometria. Sin dependencias raras; sirve para
                comparar (cuanto corrige BreezySLAM) y como respaldo si BreezySLAM no se instala.

Marco del mapa: origen = donde estaba el robot al empezar, x adelante, y izquierda, en metros.
map_image() devuelve uint8 (n, n): fila = y, columna = x; 0 = ocupado, 127 = desconocido, ~255 = libre.
"""
import math

import numpy as np

BEAMS = 360          # BreezySLAM necesita rayos equiespaciados: se re-muestrea a 1 por grado
MIN_RANGE = 0.12     # m: por debajo es el propio robot / ruido
OCC_STEP, FREE_STEP = 20, 6   # log-odds por scan: una pared se marca rapido; lo libre, algo mas despacio


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def scan_to_mm(scan, beams=BEAMS, min_range=MIN_RANGE):
    """Scan del cliente (angulos arbitrarios, rad, 0 = frente, + = izquierda) -> lista de `beams`
    distancias en mm, indice 0 = -180 grados, creciendo hacia la izquierda. 0 = sin lectura."""
    deg = np.degrees(np.asarray(scan.angles, dtype=float))
    r = np.asarray(scan.ranges, dtype=float)
    ok = np.isfinite(r) & (r >= min_range) & (r <= scan.range_max)
    idx = (np.floor((deg[ok] + 180.0) * beams / 360.0 + 0.5).astype(int)) % beams
    out = np.full(beams, np.inf)
    np.minimum.at(out, idx, r[ok])           # varios puntos en un mismo grado: el mas cercano
    out[~np.isfinite(out)] = 0.0
    return (out * 1000.0).astype(int).tolist()


class OdomTracker:
    """Convierte poses de odometria sucesivas en (avance_mm, giro_grados, dt_s) para BreezySLAM."""

    def __init__(self):
        self.prev = None

    def delta(self, pose):
        prev, self.prev = self.prev, pose
        if prev is None:
            return 0.0, 0.0, 0.1
        dx, dy = pose.x - prev.x, pose.y - prev.y
        dyaw = wrap(pose.yaw - prev.yaw)
        mid = prev.yaw + dyaw / 2.0
        forward = dx * math.cos(mid) + dy * math.sin(mid)     # componente en el rumbo medio
        dt = pose.stamp - prev.stamp
        return forward * 1000.0, math.degrees(dyaw), dt if dt > 1e-3 else 0.1


class OdomEngine:
    name = "odometria pura"

    def __init__(self, size_m=20.0, pixels=500, range_max=12.0):
        self.n, self.res, self.range_max = pixels, size_m / pixels, range_max
        self.lo = np.zeros((pixels, pixels), np.int8)         # log-odds: >0 ocupado, <0 libre
        self.origin = None
        self.pos = (0.0, 0.0, 0.0)
        t = np.arange(0.0, range_max, self.res)               # muestras a lo largo de cada rayo
        self._t = t

    def update(self, scan, pose):
        if self.origin is None:
            self.origin = (pose.x, pose.y, pose.yaw)
        x0, y0, t0 = self.origin
        dx, dy = pose.x - x0, pose.y - y0
        c, s = math.cos(-t0), math.sin(-t0)
        x, y, th = c * dx - s * dy, s * dx + c * dy, wrap(pose.yaw - t0)
        self.pos = (x, y, th)

        r = np.asarray(scan.ranges, dtype=float)
        ang = np.asarray(scan.angles, dtype=float) + th
        valid = ~np.isnan(r) & (r >= MIN_RANGE)
        r, ang = r[valid], ang[valid]
        hit = np.isfinite(r) & (r <= self.range_max)
        reach = np.where(hit, r, self.range_max)

        cs, sn = np.cos(ang)[:, None], np.sin(ang)[:, None]
        free = self._t[None, :] < (reach[:, None] - 3 * self.res)           # celdas antes del obstaculo
        self._mark(x + self._t[None, :] * cs, y + self._t[None, :] * sn, free, -FREE_STEP)
        ex, ey = x + r[hit] * np.cos(ang[hit]), y + r[hit] * np.sin(ang[hit])
        self._mark(ex[:, None], ey[:, None], np.ones((ex.size, 1), bool), OCC_STEP)

    def _mark(self, px, py, mask, delta):
        col = (self.n / 2 + px / self.res).astype(int)
        row = (self.n / 2 + py / self.res).astype(int)
        m = mask & (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        col, row = col[m], row[m]
        self.lo[row, col] = np.clip(self.lo[row, col].astype(np.int16) + delta, -127, 127)

    def position(self):
        return self.pos

    def map_image(self):
        return np.clip(127 - self.lo.astype(np.int16), 0, 255).astype(np.uint8)


class BreezyEngine:
    name = "BreezySLAM (RMHC)"

    def __init__(self, size_m=20.0, pixels=500, range_max=12.0, scan_hz=7.0, beams=BEAMS, seed=9999,
                 sigma_xy_mm=100.0, sigma_theta_deg=20.0, search_iter=1000):
        from breezyslam.algorithms import RMHC_SLAM      # ImportError si no esta instalado
        from breezyslam.sensors import Laser
        self.n, self.size_m, self.beams = pixels, size_m, beams
        self.res = size_m / pixels
        laser = Laser(beams, scan_hz, 360, range_max * 1000.0)
        self.slam = RMHC_SLAM(laser, pixels, size_m, random_seed=seed, sigma_xy_mm=sigma_xy_mm,
                              sigma_theta_degrees=sigma_theta_deg, max_search_iter=search_iter)
        self.odo = OdomTracker()
        self._buf = bytearray(pixels * pixels)

    def update(self, scan, pose):
        self.slam.update(scan_to_mm(scan, self.beams), pose_change=self.odo.delta(pose))

    def position(self):
        x, y, th = self.slam.getpos()                      # mm desde la esquina, grados
        return x / 1000.0 - self.size_m / 2, y / 1000.0 - self.size_m / 2, math.radians(th)

    def map_image(self):
        self.slam.getmap(self._buf)
        return np.frombuffer(self._buf, np.uint8).reshape(self.n, self.n).copy()


def make_engine(kind="auto", **kw):
    """kind: 'breezy', 'odom' o 'auto' (BreezySLAM si esta instalado; si no, odometria pura)."""
    if kind in ("breezy", "auto"):
        try:
            return BreezyEngine(**kw)
        except ImportError:
            if kind == "breezy":
                raise
            print("[aviso] BreezySLAM no esta instalado: uso odometria pura (ver README).")
    kw.pop("scan_hz", None)
    kw.pop("beams", None)
    for k in ("seed", "sigma_xy_mm", "sigma_theta_deg", "search_iter"):
        kw.pop(k, None)
    return OdomEngine(**kw)
