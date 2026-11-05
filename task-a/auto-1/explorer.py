"""
Controlador de exploracion autonoma (Tarea A, auto-1). Sin E/S: recibe mapa, pose y scan, devuelve (v, w).

Estados:
  PAUSA    no se mueve (arranque seguro, o la tecla de espacio)
  EXPLORAR ir a la frontera alcanzable mas cercana, una y otra vez
  CONFIRMAR parado, esperando que el detector confirme el rostro
  VOLVER   (opcional) regresar al punto de partida A tras encontrar B
  HECHO    B encontrado (o volvio a A); parado
  SIN_B    no quedan fronteras y no se vio ningun rostro; parado

El puente del robot sigue frenando por LiDAR (0.28 m al frente) pase lo que pase: esta capa es la
navegacion, no la seguridad.
"""
import math

import numpy as np

from planner import Planner

PAUSA, EXPLORAR, CONFIRMAR, VOLVER, HECHO, SIN_B = "PAUSA", "EXPLORAR", "CONFIRMAR", "VOLVER", "HECHO", "SIN_B"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Explorer:
    def __init__(self, planner: Planner, vmax=0.12, wmax=0.8, lookahead=0.35, replan_s=1.0,
                 stop_front=0.38, volver=False):
        self.pl = planner
        self.vmax, self.wmax, self.look = vmax, wmax, lookahead
        self.replan_s, self.stop_front = replan_s, stop_front
        self.volver = volver
        self.state = PAUSA
        self.path, self.goal, self.plan = [], None, None
        self.bad_goals = []
        self.last_plan = -1e9
        self._prog_t, self._prog_xy = None, None
        self._block_t = None
        self._nf_t = None                 # desde cuando no hay fronteras (hay que esperar: el mapa puede estar vacio al arrancar)
        self._spin_until, self._spin_w = 0.0, 0.0
        self.b_xy = None                  # donde se vio el rostro
        self.home = (0.0, 0.0)            # punto A: donde arranco el robot (origen del mapa)
        self.log = []

    # ---------- control externo ----------
    def start(self):
        if self.state == PAUSA:
            self.state = EXPLORAR

    def pause(self):
        self.state = PAUSA

    def face_candidate(self):
        if self.state == EXPLORAR:
            self.state = CONFIRMAR

    def face_rejected(self):
        if self.state == CONFIRMAR:
            self.state = EXPLORAR

    def face_confirmed(self, pos):
        self.b_xy = (pos[0], pos[1])
        self._note(f"B encontrado en x={pos[0]:.2f} y={pos[1]:.2f}")
        self.state = VOLVER if self.volver else HECHO
        self._invalidate()

    def _note(self, msg):
        self.log.append(msg)

    def _invalidate(self):
        self.path, self.last_plan = [], -1e9

    # ---------- bucle ----------
    def step(self, now, img, pos, scan):
        """-> (v, w). pos = (x, y, theta) del SLAM en el marco del mapa."""
        if self.state in (PAUSA, CONFIRMAR, HECHO, SIN_B):
            return 0.0, 0.0
        if now < self._spin_until:                                  # maniobra de recuperacion en curso
            return 0.0, self._spin_w
        x, y, th = pos

        if self.state == VOLVER and math.hypot(x - self.home[0], y - self.home[1]) < 0.30:
            self._note("de vuelta en A")
            self.state = HECHO
            return 0.0, 0.0

        if not self.path or now - self.last_plan > self.replan_s or self._path_blocked(img):
            self._replan(now, img, (x, y))
            if self.state in (HECHO, SIN_B):
                return 0.0, 0.0
        if not self.path:
            return 0.0, self.wmax * 0.5                             # sin ruta: gira buscando mapa nuevo

        if self._stuck(now, x, y):
            return 0.0, self._spin_w
        return self._follow(now, x, y, th, scan)

    # ---------- piezas ----------
    def _replan(self, now, img, xy):
        self.last_plan = now
        if self.state == VOLVER:
            res = self.pl.plan_to(img, xy, self.home)
        else:
            res = self.pl.plan_explore(img, xy, self.goal, self.bad_goals)
        self.plan = res
        if res.status == "ok":
            self.path, self.goal = res.path, res.goal
            self._nf_t = None
            return
        self.path = []
        if res.status == "no_frontier":
            if self._nf_t is None:
                self._nf_t = now
            if now - self._nf_t < 3.0:                              # puede ser solo que el mapa aun no se ha formado
                return
            if self.state == VOLVER:
                self.state = HECHO
            else:
                self._note("no quedan fronteras y no se vio ningun rostro")
                self.state = SIN_B
        elif res.status == "no_path" and self.goal is not None:
            self.bad_goals.append(self.goal)                         # inalcanzable: se descarta
            self.goal = None

    def _path_blocked(self, img):
        """La ruta actual cruza una celda que ahora es pared (mapa actualizado)."""
        if len(self.path) < 2:
            return False
        occ, _, _ = self.pl.grid(img)
        for px, py in self.path[:40]:
            r, c = self.pl.to_cell(px, py)
            if 0 <= r < self.pl.m and 0 <= c < self.pl.m and occ[r, c]:
                return True
        return False

    def _stuck(self, now, x, y):
        """Si en 6 s no avanza ni 12 cm, descarta la meta y gira para reorientarse."""
        if self._prog_t is None:
            self._prog_t, self._prog_xy = now, (x, y)
            return False
        if now - self._prog_t < 6.0:
            return False
        moved = math.hypot(x - self._prog_xy[0], y - self._prog_xy[1])
        self._prog_t = None                                          # reinicia la ventana de 6 s
        if moved >= 0.12:
            return False
        if self.goal is not None:
            self.bad_goals.append(self.goal)
        self.goal = None
        self._invalidate()
        self._spin_until, self._spin_w = now + 2.5, self.wmax
        self._note(f"atascado en ({x:.2f},{y:.2f}): descarto la meta y giro")
        return True

    def _follow(self, now, x, y, th, scan):
        # punto de la ruta mas cercano y, desde ahi, el punto a `lookahead` m
        d = [math.hypot(px - x, py - y) for px, py in self.path]
        i = int(np.argmin(d))
        j = i
        while j < len(self.path) - 1 and math.hypot(self.path[j][0] - x, self.path[j][1] - y) < self.look:
            j += 1
        tx, ty = self.path[j]
        remaining = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
        if remaining < 0.22:                                         # llego a la meta
            if self.state == EXPLORAR and self.goal is not None:
                self.bad_goals.append(self.goal)                     # ya no hay mas que ver desde ahi
                self.goal = None
            self._invalidate()
            return 0.0, 0.0
        err = wrap(math.atan2(ty - y, tx - x) - th)
        w = float(np.clip(1.6 * err, -self.wmax, self.wmax))
        v = self.vmax * max(0.0, math.cos(err)) if abs(err) < 0.9 else 0.0

        front = scan.sector_min(-25, 25) if scan is not None else math.inf
        if front < self.stop_front:                                  # obstaculo cerca: no avanzar, girar al lado libre
            left, right = scan.sector_min(25, 100), scan.sector_min(-100, -25)
            w = self.wmax * 0.8 * (1.0 if left >= right else -1.0)
            v = 0.0
            if self._block_t is None:
                self._block_t = now
            elif now - self._block_t > 4.0:                          # lleva 4 s bloqueado: otra meta
                if self.goal is not None:
                    self.bad_goals.append(self.goal)
                self.goal = None
                self._invalidate()
                self._block_t = None
                self._note("bloqueado por un obstaculo: cambio de meta")
        else:
            self._block_t = None
            v *= float(np.clip((front - 0.30) / 0.60, 0.25, 1.0))     # frena al acercarse
        return v, w
