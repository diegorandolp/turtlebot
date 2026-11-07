"""
Controlador de exploracion autonoma (Tarea A, auto-1). Sin E/S: recibe mapa, pose y scan, devuelve (v, w).

El LiDAR ve 360 grados pero la camara solo un cono al frente, asi que hay DOS cosas que cubrir:
el mapa (fronteras) y lo que la camara ha mirado (cobertura, ver coverage.py).

Estados:
  PAUSA        no se mueve (arranque seguro, o la tecla de espacio)
  EXPLORAR     ir a la frontera alcanzable mas cercana (el LiDAR va revelando el mapa)
  BARRIDO      parado, girando en el sitio hacia los rumbos donde la camara aun no vio superficies
  INSPECCIONAR sin fronteras: ir a los mejores puntos de vista hasta cubrir las superficies
  CONFIRMAR    parado, esperando que el detector confirme el rostro
  VOLVER       (opcional) regresar al punto de partida A tras encontrar B
  HECHO        B encontrado (o volvio a A); parado
  SIN_B        mapa explorado y superficies cubiertas sin ver ningun rostro; parado

Si la camara no da fotos, no se queda barriendo: sigue explorando con el LiDAR y al final se rinde.
El puente del robot sigue frenando por LiDAR (0.28 m al frente) pase lo que pase: esta capa es la
navegacion, no la seguridad.
"""
import math

import numpy as np

from planner import Planner

PAUSA, EXPLORAR, CONFIRMAR, VOLVER, HECHO, SIN_B = "PAUSA", "EXPLORAR", "CONFIRMAR", "VOLVER", "HECHO", "SIN_B"
INSPECCIONAR, BARRIDO = "INSPECCIONAR", "BARRIDO"
BUSCANDO = (EXPLORAR, INSPECCIONAR, BARRIDO)        # estados en los que la camara busca el rostro


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def front_clear(scan, half=0.22):
    """Distancia (m) al obstaculo mas cercano DENTRO del pasillo por el que va a pasar el robot
    (ancho 2*half). Un sector angular daria falsas alarmas al rozar una esquina."""
    if scan is None:
        return math.inf
    if hasattr(scan, "points"):
        x, y = scan.points()
        sel = (np.abs(y) < half) & (x > 0.0)
        return float(x[sel].min()) if sel.any() else math.inf
    return scan.sector_min(-25, 25)


class Explorer:
    def __init__(self, planner: Planner, cov=None, vmax=0.12, wmax=0.8, lookahead=0.35, replan_s=1.0,
                 stop_front=0.34, volver=False, sweep_timeout=30.0):
        self.pl, self.cov = planner, cov
        self.vmax, self.wmax, self.look = vmax, wmax, lookahead
        self.replan_s, self.stop_front = replan_s, stop_front
        self.volver = volver
        self.sweep_timeout = sweep_timeout
        self.state = PAUSA
        self.path, self.goal, self.plan = [], None, None
        self.view = None                  # (x, y, rumbo) del punto de vista actual (INSPECCIONAR)
        self.bad_goals = []
        self.last_plan = -1e9
        self._prog_t, self._prog_xy = None, None
        self._block_t = None
        self._nf_t = None                 # desde cuando no hay fronteras (el mapa puede estar vacio al arrancar)
        self._spin_until, self._spin_w = 0.0, 0.0
        self._want_sweep = False
        self._t0 = None                   # primer paso tras empezar (para esperar el primer mapa)
        self._prev = EXPLORAR             # a donde volver si un rostro candidato se descarta
        self.after_sweep = EXPLORAR
        self.sweep = None
        self.cam_ok = True
        self.n_sweeps = 0
        self.b_xy = None                  # donde se vio el rostro
        self.home = (0.0, 0.0)            # punto A: donde arranco el robot (origen del mapa)
        self.log = []

    # ---------- control externo ----------
    def start(self):
        if self.state == PAUSA:
            self.state = EXPLORAR
            self._want_sweep = self.cov is not None          # mirada inicial de 360 grados desde A

    def pause(self):
        self.state = PAUSA

    def face_candidate(self):
        if self.state in BUSCANDO:
            self._prev = self.state
            self.state = CONFIRMAR

    def face_rejected(self):
        if self.state == CONFIRMAR:
            self.state = self._prev

    def face_confirmed(self, pos):
        self.b_xy = (pos[0], pos[1])
        self._note(f"B encontrado en x={pos[0]:.2f} y={pos[1]:.2f}")
        self.state = VOLVER if self.volver else HECHO
        self.sweep = None
        self._invalidate()

    def _note(self, msg):
        self.log.append(msg)

    def _invalidate(self):
        self.path, self.last_plan = [], -1e9

    # ---------- bucle ----------
    def step(self, now, img, pos, scan, photos=0, cam_ok=True):
        """-> (v, w). pos = (x, y, theta) del SLAM en el marco del mapa. photos = fotos recibidas hasta ahora."""
        self.cam_ok = cam_ok
        if self.state in (PAUSA, CONFIRMAR, HECHO, SIN_B):
            return 0.0, 0.0
        if self.state == BARRIDO:
            return self._sweep_step(now, img, pos, scan, photos)
        if now < self._spin_until:                                  # maniobra de recuperacion en curso
            return 0.0, self._spin_w
        x, y, th = pos

        if self.state == VOLVER and math.hypot(x - self.home[0], y - self.home[1]) < 0.30:
            self._note("de vuelta en A")
            self.state = HECHO
            return 0.0, 0.0

        if self._want_sweep and self.state == EXPLORAR:
            if self._t0 is None:
                self._t0 = now
            if not cam_ok:
                self._want_sweep = False
            elif self.cov.weights(img)[1].sum() >= self.cov.min_gain:   # ya hay superficies en el mapa
                self._want_sweep = False
                self._begin_sweep(now, "mirada inicial")
                return 0.0, 0.0
            elif now - self._t0 < 3.0:                                  # esperar el primer mapa (parado)
                return 0.0, 0.0
            else:
                self._want_sweep = False

        if not self.path or now - self.last_plan > self.replan_s or self._path_blocked(img):
            self._replan(now, img, (x, y))
            if self.state in (HECHO, SIN_B):
                return 0.0, 0.0
        if not self.path:
            return 0.0, self.wmax * 0.5                             # sin ruta: gira buscando mapa nuevo

        if self._stuck(now, x, y):
            return 0.0, self._spin_w
        return self._follow(now, img, pos, scan)

    # ---------- barrido de camara ----------
    def _begin_sweep(self, now, why):
        self.after_sweep = self.state
        self.state = BARRIDO
        self.sweep = dict(t0=now, stage="pick", done=set(), p=None, pt=now, target=None, k=None, t_settle=0.0, p_at=0)
        self.n_sweeps += 1
        self._note(f"barrido ({why})")

    def _end_sweep(self, why=""):
        if why:
            self._note(why)
        self.state = self.after_sweep
        self.sweep = None
        self._invalidate()

    def _sweep_step(self, now, img, pos, scan, photos):
        sw = self.sweep
        if sw is None:
            self.state = self.after_sweep
            return 0.0, 0.0
        if sw["p"] is None or photos != sw["p"]:
            sw["p"], sw["pt"] = photos, now
        if not self.cam_ok or now - sw["pt"] > 6.0:
            self._end_sweep("barrido cancelado: la camara no da fotos")
            return 0.0, 0.0
        if now - sw["t0"] > self.sweep_timeout:
            self._end_sweep("barrido: tiempo maximo")
            return 0.0, 0.0
        th = pos[2]
        if sw["stage"] == "pick":
            _, W, _ = self.cov.weights(img)
            k = self.cov.best_heading(pos, scan, W, exclude=sw["done"])
            if k is None:
                self._end_sweep()
                return 0.0, 0.0
            sw["k"], sw["target"], sw["stage"] = k, self.cov.heading(k), "turn"
        if sw["stage"] == "turn":
            err = wrap(sw["target"] - th)
            if abs(err) < 0.12:
                sw["stage"], sw["t_settle"], sw["p_at"] = "settle", now, photos
                return 0.0, 0.0
            w = float(np.clip(1.8 * err, -self.wmax, self.wmax))
            return 0.0, math.copysign(max(abs(w), 0.25), w)
        # settle: esperar una foto tomada ya parado en este rumbo
        if now - sw["t_settle"] > 0.3 and photos >= sw["p_at"] + 2:
            sw["done"].add(sw["k"])
            sw["stage"] = "pick"
        return 0.0, 0.0

    # ---------- piezas ----------
    def _replan(self, now, img, xy):
        self.last_plan = now
        if self.state == VOLVER:
            res = self.pl.plan_to(img, xy, self.home)
        elif self.state == INSPECCIONAR:
            res = self._plan_inspect(img, xy)
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
            self._nf_t = None
            if self.state == VOLVER:
                self.state = HECHO
            elif self.state == INSPECCIONAR:
                self._note("superficies cubiertas y ningun rostro")
                self.state = SIN_B
            elif self.cov is not None and self.cam_ok:
                self._note("mapa explorado: paso a inspeccionar con la camara")
                self.state = INSPECCIONAR
                self._invalidate()
            else:
                self._note("no quedan fronteras y no se vio ningun rostro")
                self.state = SIN_B
        elif res.status == "no_path" and self.goal is not None:
            self.bad_goals.append(self.goal)                         # inalcanzable: se descarta
            self.goal = None

    def _plan_inspect(self, img, xy):
        """Siguiente punto de vista (next-best-view); lo mantiene mientras siga valiendo la pena."""
        from planner import PlanResult
        if not self.cam_ok:
            return PlanResult("no_frontier")
        if self.view is not None:
            _, W, occ = self.cov.weights(img)
            vx, vy, vh = self.view
            if self.cov._map_gain(vx, vy, vh, occ, W) >= self.cov.min_gain:
                res = self.pl.plan_to(img, xy, (vx, vy))
                if res.status == "ok":
                    return res
        v = self.cov.plan_view(img, xy)
        if v is None:
            self.view = None
            return PlanResult("no_frontier")
        x, y, h, g, path = v
        self.view = (x, y, h)
        return PlanResult("ok", path=path, goal=(x, y))

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

    def _drop_goal(self):
        if self.goal is not None:
            if self.state == INSPECCIONAR and self.cov is not None:
                self.cov.bad_views.append(self.goal)
            else:
                self.bad_goals.append(self.goal)
        self.goal, self.view = None, None

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
        self._drop_goal()
        self._invalidate()
        self._spin_until, self._spin_w = now + 2.5, self.wmax
        self._note(f"atascado en ({x:.2f},{y:.2f}): descarto la meta y giro")
        return True

    def _follow(self, now, img, pos, scan):
        x, y, th = pos
        # punto de la ruta mas cercano y, desde ahi, el punto a `lookahead` m
        d = [math.hypot(px - x, py - y) for px, py in self.path]
        i = int(np.argmin(d))
        j = i
        while j < len(self.path) - 1 and math.hypot(self.path[j][0] - x, self.path[j][1] - y) < self.look:
            j += 1
        tx, ty = self.path[j]
        remaining = math.hypot(self.path[-1][0] - x, self.path[-1][1] - y)
        if remaining < 0.22:                                         # llego a la meta
            arrived = self.state
            self._drop_goal()                                        # ya se vio lo que habia que ver desde ahi
            self._invalidate()
            if self.cov is not None and self.cam_ok and arrived in (EXPLORAR, INSPECCIONAR):
                _, W, _ = self.cov.weights(img)
                if arrived == INSPECCIONAR or self.cov.local_gain(pos, scan, W) >= self.cov.min_gain:
                    self._begin_sweep(now, "llegada a " + ("punto de vista" if arrived == INSPECCIONAR else "frontera"))
            return 0.0, 0.0
        err = wrap(math.atan2(ty - y, tx - x) - th)
        w = float(np.clip(1.6 * err, -self.wmax, self.wmax))
        v = self.vmax * max(0.0, math.cos(err)) if abs(err) < 0.9 else 0.0

        front = front_clear(scan)
        if front < self.stop_front:                                  # obstaculo en el pasillo: no avanzar, girar al lado libre
            left, right = scan.sector_min(25, 100), scan.sector_min(-100, -25)
            w = self.wmax * 0.8 * (1.0 if left >= right else -1.0)
            v = 0.0
            if self._block_t is None:
                self._block_t = now
            elif now - self._block_t > 4.0:                          # lleva 4 s bloqueado: otra meta
                self._drop_goal()
                self._invalidate()
                self._block_t = None
                self._note("bloqueado por un obstaculo: cambio de meta")
        else:
            self._block_t = None
            v *= float(np.clip((front - 0.30) / 0.60, 0.25, 1.0))     # frena al acercarse
        return v, w
