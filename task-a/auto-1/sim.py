"""
Simulador 2D del TurtleBot 4 en un laberinto (Tarea A, auto-1). Sirve para probar la exploracion
SIN mover el robot real. Expone la misma API que tb4.client.TB4Client (scan, pose, status, move,
stop, photo, undock, wait_ready), con tiempo simulado: el bucle llama a advance(dt).

Incluye lo que el puente real hace por seguridad (parar si hay algo a menos de 0.28 m al frente) y
colision con las paredes. La "cara" en B es un punto: photo() devuelve una imagen blanca si el robot
la tiene delante, cerca y sin pared de por medio, y negra si no (el detector del simulador mira eso).
"""
import math
import time

import numpy as np

from tb4.client import Pose, Scan, Status

RES = 0.02          # m por celda del mundo
ROBOT_R = 0.17      # m, radio del Create 3


def default_world():
    """Laberinto 9 x 7 m con paredes interiores. Devuelve (rectangulos de pared, inicio, cara)."""
    walls = [(-0.1, -0.1, 9.1, 0.0), (-0.1, 7.0, 9.1, 7.1), (-0.1, -0.1, 0.0, 7.1), (9.0, -0.1, 9.1, 7.1),   # borde
             (3.0, 0.0, 3.1, 4.5),        # pared que sale de abajo (hueco arriba)
             (6.0, 2.5, 6.1, 7.0),        # pared que sale de arriba (hueco abajo)
             (1.0, 4.5, 3.0, 4.6),        # repisa
             (4.2, 1.2, 5.0, 2.0),        # caja
             (7.2, 3.8, 8.2, 4.6)]        # caja
    return walls, (1.0, 1.0, 0.0), (8.4, 6.2)


class SimRobot:
    def __init__(self, world=None, odom_noise=0.0, seed=1, odom_bias=0.0):
        walls, start, face = world or default_world()
        self.start, self.face = start, face
        self.nx, self.ny = int(9.1 / RES) + 1, int(7.1 / RES) + 1
        self.grid = np.zeros((self.ny, self.nx), bool)
        for x0, y0, x1, y1 in walls:
            self.grid[max(int(y0 / RES), 0):int(y1 / RES) + 1, max(int(x0 / RES), 0):int(x1 / RES) + 1] = True
        self.wx, self.wy, self.wth = start                   # pose real
        self.ox = self.oy = self.oth = 0.0                   # odometria (origen = inicio)
        self.rng = np.random.default_rng(seed)
        self.noise = odom_noise
        self.bias = odom_bias            # error sistematico de la odometria (p. ej. 0.05 = gira un 5 % de mas, avanza 2.5 % de mas)
        self.time = 0.0
        self.cmd, self.cmd_t = (0.0, 0.0), -1e9
        self.angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        self.path = [(self.wx, self.wy)]
        self.collisions = 0
        self.docked = False

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

    def scan(self):
        r = self._ranges(self.wx, self.wy, self.wth)
        r = r + self.rng.normal(0, 0.01, r.shape)
        return Scan(self.angles.copy(), r, 12.0, self.time, time.monotonic())

    def pose(self):
        return Pose(self.ox, self.oy, self.oth, self.cmd[0], self.cmd[1], self.time, time.monotonic())

    def status(self):
        s = self.scan()
        fc = float(np.min(np.where((np.abs(s.points()[1]) < 0.2) & (s.points()[0] > 0), s.points()[0], 12.0)))
        return Status(0.9, False, fc < 0.28, False, False, True, fc, 5.0, 1, time.monotonic())

    def photo(self, timeout=4.0, quality=85):
        return np.full((250, 250, 3), 255 if self.face_visible() else 0, np.uint8)

    # ---- simulacion ----
    def blocked(self, x, y):
        r = int(ROBOT_R / RES)
        cx, cy = int(x / RES), int(y / RES)
        if cx - r < 0 or cy - r < 0 or cx + r >= self.nx or cy + r >= self.ny:
            return True
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        return bool(self.grid[cy - r:cy + r + 1, cx - r:cx + r + 1][(xx ** 2 + yy ** 2) <= r * r].any())

    def _ranges(self, x, y, th, rmax=12.0):
        t = np.arange(0.05, rmax, RES)
        a = self.angles + th
        px = x + np.cos(a)[:, None] * t[None, :]
        py = y + np.sin(a)[:, None] * t[None, :]
        ix, iy = (px / RES).astype(int), (py / RES).astype(int)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        hit = np.zeros(px.shape, bool)
        hit[inside] = self.grid[iy[inside], ix[inside]]
        first = hit.argmax(axis=1)
        out = np.where(hit.any(axis=1), t[first], np.inf)
        return out

    def face_visible(self, max_dist=2.5, fov=math.radians(35)):
        fx, fy = self.face
        d = math.hypot(fx - self.wx, fy - self.wy)
        if d > max_dist:
            return False
        ang = math.atan2(fy - self.wy, fx - self.wx) - self.wth
        ang = math.atan2(math.sin(ang), math.cos(ang))
        if abs(ang) > fov:
            return False
        for s in np.arange(0.1, d, RES):                      # linea de vision
            if self.grid[int((self.wy + (fy - self.wy) * s / d) / RES), int((self.wx + (fx - self.wx) * s / d) / RES)]:
                return False
        return True

    def advance(self, dt):
        v, w = self.cmd if self.time - self.cmd_t < 0.5 else (0.0, 0.0)
        if v > 0:                                              # freno de emergencia del puente
            pts = self._ranges(self.wx, self.wy, self.wth)
            xs, ys = pts * np.cos(self.angles), pts * np.sin(self.angles)
            ok = np.isfinite(pts) & (np.abs(ys) < 0.2) & (xs > 0)
            if ok.any() and xs[ok].min() < 0.28:
                v = 0.0
        nth = self.wth + w * dt
        nx, ny = self.wx + v * math.cos(nth) * dt, self.wy + v * math.sin(nth) * dt
        if self.blocked(nx, ny):
            self.collisions += 1
            nx, ny, v = self.wx, self.wy, 0.0
        self.wx, self.wy, self.wth = nx, ny, nth
        # odometria: misma cinematica con deriva opcional (noise = fraccion de error por metro/radian)
        k = 1.0 + self.bias / 2 + (self.rng.normal(0, self.noise) if self.noise else 0.0)
        self.oth += w * dt * (1.0 + self.bias + (self.rng.normal(0, self.noise) if self.noise else 0.0))
        self.ox += v * math.cos(self.oth) * dt * k
        self.oy += v * math.sin(self.oth) * dt * k
        self.time += dt
        self.path.append((self.wx, self.wy))
