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
    """Laberinto 9 x 7 m con paredes interiores. Devuelve (paredes, inicio, cara, orientacion de la cara)."""
    walls = [(-0.1, -0.1, 9.1, 0.0), (-0.1, 7.0, 9.1, 7.1), (-0.1, -0.1, 0.0, 7.1), (9.0, -0.1, 9.1, 7.1),   # borde
             (3.0, 0.0, 3.1, 4.5),        # pared que sale de abajo (hueco arriba)
             (6.0, 2.5, 6.1, 7.0),        # pared que sale de arriba (hueco abajo)
             (1.0, 4.5, 3.0, 4.6),        # repisa
             (4.2, 1.2, 5.0, 2.0),        # caja
             (7.2, 3.8, 8.2, 4.6)]        # caja
    return walls, (1.0, 1.0, 0.0), (8.4, 6.2), math.atan2(-1.0, -1.0)


def lateral_world():
    """Pasillo en L: la foto esta en una pared LATERAL del primer tramo, fuera del cono de la camara al ir de frente."""
    walls = [(-0.1, -0.1, 8.1, 0.0), (-0.1, 4.0, 5.0, 4.1), (-0.1, -0.1, 0.0, 4.1),          # borde sur, norte (parte), oeste
             (5.0, 2.0, 8.1, 2.1), (8.0, -0.1, 8.1, 2.1), (5.0, 2.0, 5.1, 6.1), (7.0, 6.0, 7.1, 6.1),   # tramo este
             (3.0, 1.2, 3.8, 1.9)]        # obstaculo
    # foto en la pared norte del pasillo (y = 4.0), mirando al sur; el robot parte mirando al este
    return walls, (0.6, 0.6, 0.0), (1.6, 3.95), -math.pi / 2


def pista_world():
    """Circuito de la pizarra (version 2, 2026-11-13), medidas a escala del dibujo (5 m de ancho):
      - abajo, un pasillo horizontal de 1.1 m; A en su extremo derecho, con la camara mirando a la IZQUIERDA
      - a la izquierda, un pasillo vertical de 1.27 m que sube
      - arriba, una sala de 2.8 m de alto; F1 y F2 en la pared de arriba (miran hacia abajo)
      - en medio, un muro saliente que nace de la pared derecha (0.65 m de grueso); F3 y F4 en su cara de
        arriba (miran hacia la sala)
    Ajusta las medidas a las reales cuando las tengas."""
    t = 0.1
    x0, x1 = 0.5, 5.5                                   # interior: 5.0 m de ancho
    y0, y1 = 0.5, 5.05                                  # interior: 4.55 m de alto
    walls = [(x0 - t, y0 - t, x1 + t, y0), (x0 - t, y1, x1 + t, y1 + t),        # sur, norte
             (x0 - t, y0 - t, x0, y1 + t), (x1, y0 - t, x1 + t, y1 + t),        # oeste, este
             (1.77, 1.6, x1, 2.25)]                                             # muro saliente desde la derecha
    fotos = [(3.24, 5.03, -math.pi / 2), (4.62, 5.03, -math.pi / 2),           # F1, F2 (pared de arriba)
             (3.24, 2.28, math.pi / 2), (4.52, 2.28, math.pi / 2)]             # F3, F4 (cara de arriba del muro)
    return walls, (5.0, 1.05, math.pi), fotos, None


# ---- pista vertical (pizarra v3, 2026-11-13): parte de ARRIBA fija + 3 variantes de la zona de ABAJO ----
# Rectangulo interior x 0.5..5.5 (5 m), y 0.5..8.95 (8.45 m). A abajo, mirando hacia ARRIBA.
#   arriba (igual que `pista`): sala de 2.8 m (y 6.15..8.95) con F1, F2 en la pared de arriba; muro saliente desde
#   la pared derecha (x 1.77..5.5, y 5.5..6.15) con F3, F4 en su cara de arriba; hueco a la izquierda (x 0.5..1.77)
#   abajo (y 0.5..5.5, 5 x 5 m): la zona que cambia en cada variante
PV_X0, PV_X1, PV_Y0, PV_YM, PV_Y1 = 0.5, 5.5, 0.5, 5.5, 8.95


def _pista_vertical(zona_abajo, inicio=(3.0, 1.0, math.pi / 2)):
    t = 0.1
    walls = [(PV_X0 - t, PV_Y0 - t, PV_X1 + t, PV_Y0), (PV_X0 - t, PV_Y1, PV_X1 + t, PV_Y1 + t),     # sur, norte
             (PV_X0 - t, PV_Y0 - t, PV_X0, PV_Y1 + t), (PV_X1, PV_Y0 - t, PV_X1 + t, PV_Y1 + t),     # oeste, este
             (1.77, PV_YM, PV_X1, PV_YM + 0.65)]                                                    # muro saliente
    fotos = [(3.24, PV_Y1 - 0.02, -math.pi / 2), (4.62, PV_Y1 - 0.02, -math.pi / 2),               # F1, F2
             (3.24, PV_YM + 0.68, math.pi / 2), (4.52, PV_YM + 0.68, math.pi / 2)]                 # F3, F4
    return walls + zona_abajo, inicio, fotos, None


def pista_obstaculos_world():
    """Zona de abajo como en el dibujo: un muro en escuadra arriba a la izquierda y una caja a la derecha."""
    return _pista_vertical([(1.0, 4.2, 2.7, 4.35), (1.0, 3.3, 1.15, 4.35),     # escuadra
                            (3.7, 2.4, 4.6, 3.9),                              # caja
                            (2.2, 2.0, 2.6, 2.4)])                             # columna


def pista_zigzag_world():
    """Zona de abajo en serpiente: muros alternos desde la izquierda y la derecha obligan a cruzar de lado a lado."""
    return _pista_vertical([(PV_X0, 2.0, 4.2, 2.12),          # desde la izquierda (hueco a la derecha)
                            (1.8, 3.6, PV_X1, 3.72),          # desde la derecha (hueco a la izquierda)
                            (3.0, 4.5, 3.4, 4.9),             # columna en el ultimo tramo
                            (2.6, 2.12, 2.9, 2.42)])          # saliente pegado a la pared (deja 1.2 m de paso)


def _laberinto_perfecto(cols, filas, x0, y0, celda, salida_col, seed, t=0.1):
    """Laberinto 'perfecto' (arbol de expansion por DFS): entre dos celdas cualesquiera hay UN solo camino, y el
    resto son callejones sin salida (distractores). Devuelve paredes internas + borde de arriba (con la salida
    abierta en `salida_col`). Los bordes sur, este y oeste los pone la sala."""
    import random
    rnd = random.Random(seed)
    # pared_e[c][f]: pared entre (c, f) y (c+1, f); pared_n[c][f]: entre (c, f) y (c, f+1)
    pared_e = [[True] * filas for _ in range(cols - 1)]
    pared_n = [[True] * (filas - 1) for _ in range(cols)]
    visto, pila = {(0, 0)}, [(0, 0)]
    while pila:
        c, f = pila[-1]
        vecinos = [(c + dc, f + df) for dc, df in ((1, 0), (-1, 0), (0, 1), (0, -1))
                   if 0 <= c + dc < cols and 0 <= f + df < filas and (c + dc, f + df) not in visto]
        if not vecinos:
            pila.pop()
            continue
        nc, nf = rnd.choice(vecinos)
        if nc != c:
            pared_e[min(c, nc)][f] = False
        else:
            pared_n[c][min(f, nf)] = False
        visto.add((nc, nf))
        pila.append((nc, nf))
    walls = []
    for c in range(cols - 1):
        for f in range(filas):
            if pared_e[c][f]:
                x = x0 + (c + 1) * celda
                walls.append((x - t / 2, y0 + f * celda - t / 2, x + t / 2, y0 + (f + 1) * celda + t / 2))
    for c in range(cols):
        for f in range(filas - 1):
            if pared_n[c][f]:
                y = y0 + (f + 1) * celda
                walls.append((x0 + c * celda - t / 2, y - t / 2, x0 + (c + 1) * celda + t / 2, y + t / 2))
    ytop = y0 + filas * celda
    for c in range(cols):                                     # borde de arriba, salvo la celda de salida
        if c != salida_col:
            walls.append((x0 + c * celda, ytop - t, x0 + (c + 1) * celda, ytop))
    return walls


def _laberinto_unicursal(cols, filas, x0, y0, celda, inicio, salida_col, seed, t=0.1):
    """Laberinto UNICURSAL: un unico pasillo sin bifurcaciones que pasa por TODAS las celdas, desde la celda
    `inicio` (col, fila) hasta la celda de salida (salida_col, fila de arriba). Es un camino hamiltoniano en la
    rejilla, buscado con retroceso + heuristica de Warnsdorff (primero las celdas con menos salidas)."""
    import random
    rnd = random.Random(seed)
    fin = (salida_col, filas - 1)
    total = cols * filas

    def vecinos(c, f, visto):
        return [(c + dc, f + df) for dc, df in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= c + dc < cols and 0 <= f + df < filas and (c + dc, f + df) not in visto]

    for _ in range(200):                                       # varios intentos por si el retroceso se alarga
        camino, visto, pasos = [inicio], {inicio}, [0]

        def dfs():
            pasos[0] += 1
            if pasos[0] > 200000:
                return False
            if len(camino) == total:
                return camino[-1] == fin
            c, f = camino[-1]
            opciones = [v for v in vecinos(c, f, visto) if v != fin or len(camino) == total - 1]
            rnd.shuffle(opciones)
            opciones.sort(key=lambda v: len(vecinos(v[0], v[1], visto)))
            for v in opciones:
                camino.append(v)
                visto.add(v)
                if dfs():
                    return True
                camino.pop()
                visto.discard(v)
            return False
        if dfs():
            break
    else:
        raise RuntimeError("no se encontro un laberinto unicursal con esa semilla")
    abiertos = {frozenset(par) for par in zip(camino, camino[1:])}
    walls = []
    for c in range(cols):
        for f in range(filas):
            if c + 1 < cols and frozenset(((c, f), (c + 1, f))) not in abiertos:
                x = x0 + (c + 1) * celda
                walls.append((x - t / 2, y0 + f * celda - t / 2, x + t / 2, y0 + (f + 1) * celda + t / 2))
            if f + 1 < filas and frozenset(((c, f), (c, f + 1))) not in abiertos:
                y = y0 + (f + 1) * celda
                walls.append((x0 + c * celda - t / 2, y - t / 2, x0 + (c + 1) * celda + t / 2, y + t / 2))
    ytop = y0 + filas * celda
    for c in range(cols):
        if c != salida_col:
            walls.append((x0 + c * celda, ytop - t, x0 + (c + 1) * celda, ytop))
    return walls, camino


def pista_unicursal_world(seed=1):
    """Zona de abajo = laberinto UNICURSAL de 6 x 6 celdas (pasillos de ~0.73 m): un solo pasillo, sin
    bifurcaciones, que recorre todas las celdas desde A hasta la salida de arriba a la izquierda."""
    celda = (PV_X1 - PV_X0) / 6
    zona, _ = _laberinto_unicursal(6, 6, PV_X0, PV_Y0, celda, inicio=(2, 0), salida_col=0, seed=seed)
    return _pista_vertical(zona, inicio=(PV_X0 + 2.5 * celda, PV_Y0 + 0.5 * celda, math.pi / 2))


def pista_laberinto_world(seed=7):
    """Zona de abajo = laberinto perfecto de 6 x 6 celdas de 0.83 m (pasillos de ~0.73 m; el robot mide 0.34 m):
    una sola solucion desde A hasta la salida (arriba a la izquierda, hacia el hueco del muro saliente) y varios
    callejones sin salida como distractores."""
    celda = (PV_X1 - PV_X0) / 6
    zona = _laberinto_perfecto(cols=6, filas=6, x0=PV_X0, y0=PV_Y0, celda=celda, salida_col=0, seed=seed)
    return _pista_vertical(zona, inicio=(PV_X0 + 2.5 * celda, PV_Y0 + 0.5 * celda, math.pi / 2))   # A en el centro de su celda


def pista_camino_world():
    """Pizarra v4 (2026-11-13), a escala del dibujo: A abajo a la derecha mirando arriba. Camino: subir por la
    derecha hasta un muro horizontal, ir a la izquierda por el pasillo del medio, subir por el centro y salir por el
    hueco del muro saliente. Desvios: un bolsillo cerrado abajo a la izquierda y un rincon cerrado arriba a la derecha."""
    t2 = 0.05
    return _pista_vertical([(4.58 - t2, 2.94, 4.58 + t2, PV_YM),     # vertical que baja del muro saliente
                            (2.74, 2.94 - t2, PV_X1, 2.94 + t2),     # horizontal que tapa la subida por la derecha
                            (2.74 - t2, 2.94, 2.74 + t2, 3.83),      # gancho: vertical
                            (2.74, 3.83 - t2, 3.79, 3.83 + t2),      # gancho: horizontal
                            (PV_X0, 3.06 - t2, 1.55, 3.06 + t2),     # tapa del bolsillo de la izquierda
                            (1.42 - t2, 1.71, 1.42 + t2, 3.06),      # pared del bolsillo
                            (1.42, 1.71 - t2, 4.05, 1.71 + t2)],     # horizontal de abajo
                           inicio=(4.75, 0.9, math.pi / 2))


def pista_serpiente_world():
    """Camino unico en S: tres muros horizontales alternos (izquierda, derecha, izquierda) obligan a cruzar el ancho
    tres veces. Pasillos de ~1.2 m y huecos de 1.1 m."""
    t2 = 0.05
    return _pista_vertical([(PV_X0, 1.6 - t2, 4.4, 1.6 + t2),       # desde la izquierda (hueco a la derecha)
                            (1.6, 2.9 - t2, PV_X1, 2.9 + t2),       # desde la derecha (hueco a la izquierda)
                            (PV_X0, 4.2 - t2, 4.4, 4.2 + t2)],      # desde la izquierda (hueco a la derecha)
                           inicio=(3.0, 1.0, math.pi / 2))


def pista_desvios_world():
    """Camino en U con dos callejones sin salida: A sube por la columna derecha (que acaba cerrada bajo el muro
    saliente, con un bolsillo lateral), el camino bueno cruza por abajo a la columna del medio, sube y pasa a la
    izquierda por encima de un muro; la parte baja de la columna izquierda es otro callejon."""
    t2 = 0.05
    return _pista_vertical([(2.0 - t2, PV_Y0, 2.0 + t2, 3.8),       # separa izquierda / medio (sale del suelo)
                            (3.6 - t2, 1.6, 3.6 + t2, PV_YM),       # separa medio / derecha (baja del muro saliente)
                            (4.4, 3.0 - t2, PV_X1, 3.0 + t2),       # bolsillo en la columna derecha
                            (PV_X0, 2.4 - t2, 1.2, 2.4 + t2)],      # saliente en la columna izquierda
                           inicio=(4.55, 0.9, math.pi / 2))


WORLDS = {"laberinto": default_world, "lateral": lateral_world, "pista": pista_world,
          "pista_camino": pista_camino_world, "pista_serpiente": pista_serpiente_world,
          "pista_desvios": pista_desvios_world,
          "pista_obstaculos": pista_obstaculos_world, "pista_zigzag": pista_zigzag_world,
          "pista_laberinto": pista_laberinto_world}
# mas laberintos: 3 perfectos (una solucion + callejones) y 3 unicursales (un solo pasillo), con semillas distintas
for _i, _seed in enumerate((1, 2, 3), start=2):
    WORLDS[f"pista_laberinto_{_i}"] = (lambda sd: (lambda: pista_laberinto_world(seed=sd)))(_seed)
for _i, _seed in enumerate((1, 2, 3), start=1):
    WORLDS[f"pista_unicursal_{_i}"] = (lambda sd: (lambda: pista_unicursal_world(seed=sd)))(_seed)


class SimDetector:
    """'Detector' del simulador: encuentra los recuadros blancos que dibuja SimRobot.photo(). Misma interfaz que
    face.FaceDetector: faces(img) -> [(x, y, w, h, score)] y __call__(img) -> bool."""

    def __init__(self, min_width=4):
        self.min_width = min_width

    def faces(self, img):
        import cv2
        m = (img[..., 0] > 200).astype(np.uint8)
        n, _, st, _ = cv2.connectedComponentsWithStats(m)
        return [(float(st[i, 0]), float(st[i, 1]), float(st[i, 2]), float(st[i, 3]), 0.95)
                for i in range(1, n) if st[i, 2] >= self.min_width]

    def __call__(self, img):
        return len(self.faces(img)) > 0


class SimRobot:
    def __init__(self, world=None, odom_noise=0.0, seed=1, odom_bias=0.0, cam_fov_deg=60.0, cam_range=1.5,
                 camera_ok=True):
        if isinstance(world, str):
            world = WORLDS[world]()
        walls, start, face, face_normal = world or default_world()
        if face_normal is None:                                # varias fotos: [(x, y, normal), ...]
            self.faces = [tuple(f) for f in face]
        else:
            self.faces = [(face[0], face[1], face_normal)]
        self.start = start
        self.face, self.face_normal = self.faces[0][:2], self.faces[0][2]      # la primera (compatibilidad)
        self.face_seen = None                                  # indice de la ultima foto vista
        self.cam_fov, self.cam_range, self.camera_ok = math.radians(cam_fov_deg), cam_range, camera_ok
        self.photos_taken = 0
        ancho = max(9.1, max(w[2] for w in walls) + 0.1)    # el tamano sale de las paredes (minimo 9.1 x 7.1 m)
        alto = max(7.1, max(w[3] for w in walls) + 0.1)
        self.nx, self.ny = int(ancho / RES) + 1, int(alto / RES) + 1
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
        if not self.camera_ok:
            raise RuntimeError("sin imagen de la camara (simulada caida)")
        self.photos_taken += 1
        # cada foto visible se dibuja como un recuadro blanco donde caeria en la imagen (camara pinhole), con el
        # tamano de una cara de ~12 cm a esa distancia. SimDetector los lee como si fuera YuNet.
        self.face_visible()                                    # actualiza face_seen (la mas centrada)
        img = np.zeros((250, 250, 3), np.uint8)
        f = 125.0 / math.tan(self.cam_fov / 2)
        for _, rel, d in self.visible_faces():
            cx = 125.0 - f * math.tan(rel)
            w = max(4.0, f * 0.12 / d)
            x0, x1 = int(round(cx - w / 2)), int(round(cx + w / 2))
            y0, y1 = int(round(125 - 0.6 * w)), int(round(125 + 0.6 * w))
            img[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)] = 255
        return img

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

    def face_visible(self):
        """Alguna foto se ve si esta dentro del cono de la camara (rango y angulo), sin pared de por medio y
        de frente (la foto esta pegada a una superficie: no se ve desde mas de ~70 grados de la normal)."""
        vis = self.visible_faces()
        if not vis:
            return False
        self.face_seen = min(vis, key=lambda t: abs(t[1]))[0]   # la mas centrada en la imagen
        return True

    def visible_faces(self):
        """[(indice, angulo relativo (rad, + = izquierda), distancia)] de las fotos que ve la camara ahora."""
        out = []
        for i, (fx, fy, normal) in enumerate(self.faces):
            if self._face_visible(fx, fy, normal):
                ang = math.atan2(fy - self.wy, fx - self.wx)
                out.append((i, math.atan2(math.sin(ang - self.wth), math.cos(ang - self.wth)),
                            math.hypot(fx - self.wx, fy - self.wy)))
        return out

    def _face_visible(self, fx, fy, normal):
        d = math.hypot(fx - self.wx, fy - self.wy)
        if d > self.cam_range or d < 0.15:
            return False
        ang = math.atan2(fy - self.wy, fx - self.wx)
        rel = math.atan2(math.sin(ang - self.wth), math.cos(ang - self.wth))
        if abs(rel) > self.cam_fov / 2:
            return False
        view = math.atan2(math.sin(ang + math.pi - normal), math.cos(ang + math.pi - normal))
        if abs(view) > math.radians(70):
            return False
        for s in np.arange(0.1, d - 0.05, RES):                # linea de vision
            ix, iy = int((self.wx + (fx - self.wx) * s / d) / RES), int((self.wy + (fy - self.wy) * s / d) / RES)
            if self.grid[iy, ix]:
                return False
        return True

    def render_world(self, px_per_m=80):
        """Imagen BGR del mundo REAL simulado (lo que el robot no conoce): paredes, foto con cara (magenta,
        con flecha hacia donde mira), punto A, recorrido real, robot y cono de la camara (amarillo; verde
        cuando ve la cara). Sirve para comparar con el mapa que construye el SLAM."""
        import cv2
        k = px_per_m * RES                                     # px por celda del mundo
        h, w = int(self.ny * k), int(self.nx * k)
        img = np.full((h, w, 3), 245, np.uint8)
        walls = cv2.resize(self.grid.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
        img[walls] = (60, 60, 60)
        img = np.ascontiguousarray(img[::-1])                  # y hacia arriba

        def p(x, y):
            return int(x * px_per_m), int(h - 1 - y * px_per_m)
        for a, b in zip(self.path[::3], self.path[3::3]):
            cv2.line(img, p(*a), p(*b), (200, 120, 0), 1)
        ax, ay, _ = self.start
        cv2.drawMarker(img, p(ax, ay), (0, 0, 0), cv2.MARKER_DIAMOND, 14, 2)
        cv2.putText(img, "A", (p(ax, ay)[0] + 8, p(ax, ay)[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        for i, (fx, fy, normal) in enumerate(self.faces):
            cv2.circle(img, p(fx, fy), 7, (255, 0, 255), -1)
            cv2.arrowedLine(img, p(fx, fy), p(fx + 0.4 * math.cos(normal), fy + 0.4 * math.sin(normal)),
                            (255, 0, 255), 2, tipLength=0.35)
            nombre = f"F{i + 1}" if len(self.faces) > 1 else "cara (B)"
            lx, ly = p(fx + 0.62 * math.cos(normal), fy + 0.62 * math.sin(normal))    # etiqueta pasada la punta de la flecha
            cv2.putText(img, nombre, (lx - 10, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA)
        ve = self.face_visible()
        cone = [p(self.wx, self.wy)] + [p(self.wx + self.cam_range * math.cos(self.wth + a),
                                          self.wy + self.cam_range * math.sin(self.wth + a))
                                        for a in np.linspace(-self.cam_fov / 2, self.cam_fov / 2, 12)]
        over = img.copy()
        cv2.fillPoly(over, [np.array(cone, np.int32)], (0, 200, 0) if ve else (0, 220, 255))
        img = cv2.addWeighted(over, 0.35, img, 0.65, 0)
        c = p(self.wx, self.wy)
        cv2.circle(img, c, int(ROBOT_R * px_per_m), (0, 0, 255), 2)
        cv2.line(img, c, p(self.wx + ROBOT_R * math.cos(self.wth), self.wy + ROBOT_R * math.sin(self.wth)), (0, 0, 255), 2)
        img = np.vstack([np.full((26, img.shape[1], 3), 255, np.uint8), img])        # franja para el texto
        cv2.putText(img, f"MUNDO REAL (simulado)  t={self.time:.0f}s  choques={self.collisions}  camara ve: {(('F' + str(self.face_seen + 1)) if len(self.faces) > 1 else 'la cara') if ve else 'nada'}",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return img

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
