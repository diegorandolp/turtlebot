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
SEGUIR_PARED = "SEGUIR_PARED"                         # estrategia "derecha": regla de la mano derecha
ACERCARSE, VERIFICAR = "ACERCARSE", "VERIFICAR"       # ir al punto de vista de una cara vista / reconocerla de frente
BUSCANDO = (EXPLORAR, INSPECCIONAR, BARRIDO, SEGUIR_PARED, ACERCARSE, VERIFICAR)   # la camara busca rostros


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
                 stop_front=0.34, volver=False, sweep_timeout=30.0, inspeccion_directa=False,
                 estrategia="fronteras", mirar_cada=0.7, d_pared=0.35):
        # estrategia "fronteras" (por defecto): exploracion por fronteras + cobertura de camara
        # estrategia "derecha": seguir la pared DERECHA con el LiDAR (regla de la mano derecha; en un laberinto
        #   perfecto siempre llega a la salida) mirando a la pared cada `mirar_cada` m (0 = no mirar). Al llegar a
        #   una zona abierta (la sala de arriba) o si da vueltas en circulo, pasa a "fronteras" + inspeccion.
        if estrategia not in ("fronteras", "derecha"):
            raise ValueError(f"estrategia desconocida: {estrategia}")
        self.estrategia, self.mirar_cada, self.d_pared = estrategia, mirar_cada, d_pared
        self.wf = None
        self.pl, self.cov = planner, cov
        # True cuando ya hay un mapa completo de un intento anterior (--mapa): se salta la exploracion
        # por fronteras y va directo a los puntos de vista para que la camara mire todas las superficies
        self.inspeccion_directa = inspeccion_directa and cov is not None
        self.vmax, self.wmax, self.look = vmax, wmax, lookahead
        self.replan_s, self.stop_front = replan_s, stop_front
        self.volver = volver
        self.sweep_timeout = sweep_timeout
        self.state = PAUSA
        self.path, self.goal, self.plan = [], None, None
        self.view = None                  # (x, y, rumbo) del punto de vista actual (INSPECCIONAR)
        self.bad_goals = []
        self._reintentos, self.max_reintentos = 0, 3   # veces que se vuelven a probar las metas descartadas
        self.last_goal = None             # ultima meta alcanzada/descartada (estrategia de fronteras "rama")
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
        self.nombre = None                # nombre de la persona reconocida en B (con --buscar)
        self.cara_vp, self.cara_fallida, self._antes_de_cara = None, False, None   # ACERCARSE / VERIFICAR
        self._llego_cara = False

    # ---------- control externo ----------
    def start(self):
        if self.state == PAUSA:
            if self.inspeccion_directa:
                self.state = INSPECCIONAR
                self._note("mapa previo cargado: inspeccion directa")
                return
            if self.estrategia == "derecha" and self.wf is None:
                self.state = SEGUIR_PARED
                self.wf = dict(last_xy=None, dist=0.0, glance=None, glance_xy=None, arc_t=None, recto=False,
                               open_t=None, muestras=[], prog=None)
                self._note("sigo la pared derecha")
                return
            if self.estrategia == "derecha" and self.wf is not None and self.wf.get("activo", True):
                self.state = SEGUIR_PARED                    # reanudar tras una pausa
                return
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

    def face_confirmed(self, pos, nombre=None):
        self.b_xy = (pos[0], pos[1])
        self.nombre = nombre
        self._note(f"B encontrado en x={pos[0]:.2f} y={pos[1]:.2f}" + (f": {nombre}" if nombre else ""))
        self.state = VOLVER if self.volver else HECHO
        self._llego_cara = False
        self.sweep = None
        self._invalidate()

    def ir_a_cara(self, vp):
        """Ir al punto de vista (x, y, rumbo) de una cara vista, para reconocerla de cerca y de frente."""
        if self.state not in (ACERCARSE, VERIFICAR):
            self._antes_de_cara = self.after_sweep if self.state == BARRIDO else self.state
        self.sweep = None
        self.cara_vp, self.cara_fallida, self._llego_cara = vp, False, False
        self.state = ACERCARSE
        self._invalidate()
        self._prog_t = None

    def terminar_cara(self):
        """La cara actual ya se reviso (o no se pudo llegar): volver a lo que se hacia antes."""
        self.cara_vp, self.cara_fallida = None, False
        self.state = self._antes_de_cara or EXPLORAR
        self._invalidate()
        self._prog_t = None

    def face_otra_persona(self, nombre, sim):
        """La cara confirmada NO es la persona buscada: seguir buscando (lo que se estuviera haciendo)."""
        self._note(f"cara de {nombre} ({sim:.2f}): no es la persona buscada, sigo buscando")
        if self.state == CONFIRMAR:
            self.state = self._prev

    def _note(self, msg):
        self.log.append(msg)

    def _invalidate(self):
        self.path, self.last_plan = [], -1e9

    # ---------- bucle ----------
    def step(self, now, img, pos, scan, photos=0, cam_ok=True):
        """-> (v, w). pos = (x, y, theta) del SLAM en el marco del mapa. photos = fotos recibidas hasta ahora."""
        self.cam_ok = cam_ok
        if self.state in (PAUSA, CONFIRMAR, HECHO, SIN_B, VERIFICAR):
            return 0.0, 0.0
        if self.state == BARRIDO:
            return self._sweep_step(now, img, pos, scan, photos)
        if self.state == SEGUIR_PARED:
            return self._wall_step(now, pos, scan, photos)
        if now < self._spin_until:                                  # maniobra de recuperacion en curso
            return 0.0, self._spin_w
        x, y, th = pos

        d_home = math.hypot(x - self.home[0], y - self.home[1])
        # llegada a A: cerca, o al final de la ruta de vuelta (misma razon que en ACERCARSE: la ruta termina "cerca"
        # de A y el seguidor para antes del final; exigir 0.30 m dejaba al robot parado a ~0.35 m para siempre)
        if self.state == VOLVER and (d_home < 0.30 or (self._llego_cara and d_home < 0.6)):
            self._note("de vuelta en A")
            self.state = HECHO
            return 0.0, 0.0

        if self.state == ACERCARSE:                                  # llegada al punto de vista: girar de frente a la foto
            vx, vy, vh = self.cara_vp
            # llegada: cerca del punto de vista, o al final de la ruta (el planificador la termina "cerca" del
            # punto y el seguidor para a ~0.22 m del final: exigir 0.25 m dejaba al robot yendo y viniendo)
            if math.hypot(x - vx, y - vy) < 0.25 or (self._llego_cara and math.hypot(x - vx, y - vy) < 0.6):
                ok, w = self._girar_a(vh, th)
                if not ok:
                    return 0.0, w
                self.state = VERIFICAR
                self._llego_cara = False
                self._invalidate()
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

    # ---------- seguir la pared derecha ----------
    def _salir_de_pared(self, why):
        self._note(why)
        self.wf["activo"] = False
        self.state = EXPLORAR
        self._want_sweep = False
        self._invalidate()
        self._prog_t = None

    def _girar_a(self, target, th):
        err = wrap(target - th)
        w = float(np.clip(1.8 * err, -self.wmax, self.wmax))
        return abs(err) < 0.12, math.copysign(max(abs(w), 0.25), w)

    def _wall_step(self, now, pos, scan, photos):
        st = self.wf
        x, y, th = pos
        if scan is None:
            return 0.0, 0.0
        # --- mirada a la pared: girar 90 grados a la derecha, esperar 2 fotos quieto, volver ---
        g = st["glance"]
        if g is not None:
            if g["stage"] == "ida":
                ok, w = self._girar_a(g["target"], th)
                if not ok:
                    return 0.0, w
                g.update(stage="quieto", t=now, p=photos)
            if g["stage"] == "quieto":
                if (now - g["t"] > 0.3 and photos >= g["p"] + 2) or now - g["t"] > 4.0 or not self.cam_ok:
                    g["stage"] = "vuelta"
                else:
                    return 0.0, 0.0
            ok, w = self._girar_a(g["h0"], th)
            if not ok:
                return 0.0, w
            st["glance"], st["glance_xy"] = None, (x, y)
            st["prog"] = None
            return 0.0, 0.0
        # --- odometro y muestras para detectar vueltas en circulo ---
        if st["last_xy"] is not None:
            st["dist"] += math.hypot(x - st["last_xy"][0], y - st["last_xy"][1])
        st["last_xy"] = (x, y)
        if st["glance_xy"] is None:
            st["glance_xy"] = (x, y)
        if not st["muestras"] or st["dist"] - st["muestras"][-1][3] > 0.3:
            for mx, my, mth, md in st["muestras"]:
                if st["dist"] - md > 4.0 and math.hypot(x - mx, y - my) < 0.3 and abs(wrap(th - mth)) < 0.6:
                    self._salir_de_pared("seguir la pared me trajo a un sitio ya visitado: paso a fronteras")
                    return 0.0, 0.0
            st["muestras"].append((x, y, th, st["dist"]))
        # --- zona abierta (la sala de arriba): pasar a fronteras + inspeccion ---
        r = np.asarray(scan.ranges, float)
        a = (np.degrees(np.asarray(scan.angles, float)) + 180.0) % 360.0 - 180.0
        delante = (np.abs(a) <= 90) & ~np.isnan(r)
        abierto = delante.any() and np.mean(np.where(np.isfinite(r[delante]), r[delante], 12.0) > 1.6) > 0.6
        if abierto:
            st["open_t"] = st["open_t"] or now
            if now - st["open_t"] > 1.5:
                self._salir_de_pared("zona abierta: paso a fronteras + inspeccion de camara")
                return 0.0, 0.0
        else:
            st["open_t"] = None
        # --- sin avance en 15 s (sin contar las miradas): pasar a fronteras. Ojo: en una celda con la unica salida
        # a la derecha, la regla gira 270 grados a la izquierda en el sitio (~5-9 s): no es un atasco ---
        if st["prog"] is None or now - st["prog"][0] > 15.0:
            if st["prog"] is not None and math.hypot(x - st["prog"][1], y - st["prog"][2]) < 0.12:
                self._salir_de_pared(f"siguiendo la pared no avanzo en ({x:.2f},{y:.2f}): paso a fronteras")
                return 0.0, 0.0
            st["prog"] = (now, x, y)
        d_r = scan.sector_min(-100, -80)                 # pared a la derecha
        d_rf = scan.sector_min(-60, -30)                 # derecha-delante (anticipar)
        front = front_clear(scan)
        # --- mirar a la pared cada `mirar_cada` m (si hay pared a la derecha) ---
        if (self.mirar_cada > 0 and self.cam_ok and d_r < 1.0
                and math.hypot(x - st["glance_xy"][0], y - st["glance_xy"][1]) >= self.mirar_cada):
            st["glance"] = dict(stage="ida", target=wrap(th - math.pi / 2), h0=th)
            return 0.0, 0.0
        # --- regla de la mano derecha (reactiva, con el LiDAR) ---
        if front < 0.42:                                 # pared delante: girar a la izquierda en el sitio
            st["arc_t"], st["recto"] = None, False
            return 0.0, self.wmax
        if st["recto"]:                                  # buscando pared: recto hasta encontrar algo delante
            if d_r < 0.6:
                st["recto"] = False
            else:
                return self.vmax, 0.0
        if d_r > 0.75 and d_rf > 0.75:                   # se acabo la pared derecha: doblar la esquina a la derecha
            st["arc_t"] = st["arc_t"] or now
            if now - st["arc_t"] > 5.0:                  # demasiado tiempo girando sin encontrar pared: ir recto
                st["recto"], st["arc_t"] = True, None
            return 0.07, -0.55
        st["arc_t"] = None
        w = -1.6 * (d_r - self.d_pared)                  # lejos de la pared -> a la derecha; cerca -> a la izquierda
        if d_rf < 0.30:
            w += 0.5
        w = float(np.clip(w, -self.wmax * 0.7, self.wmax * 0.7))
        v = self.vmax * float(np.clip(1.0 - abs(w) / self.wmax, 0.35, 1.0))
        v *= float(np.clip((front - 0.30) / 0.60, 0.3, 1.0))
        return v, w

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
        elif self.state == ACERCARSE:
            res = self.pl.plan_to(img, xy, self.cara_vp[:2])
            self.plan = res
            if res.status == "ok":
                self.path, self.goal = res.path, res.goal
            else:
                self.path, self.cara_fallida = [], True                # el punto de vista no es alcanzable
            return
        else:
            res = self.pl.plan_explore(img, xy, self.goal, self.bad_goals, anchor=self.last_goal)
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
            # antes de dar el mapa por explorado: si hay metas descartadas (por un atasco o un bloqueo puntual),
            # volver a intentarlas. En un pasillo unico la meta descartada puede ser la UNICA continuacion del camino.
            if self.state == EXPLORAR and self.bad_goals and self._reintentos < self.max_reintentos:
                self._reintentos += 1
                self._note(f"sin fronteras: reintento las {len(self.bad_goals)} metas descartadas "
                           f"({self._reintentos}/{self.max_reintentos})")
                self.bad_goals.clear()
                self._invalidate()
                return
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
        if self.state == ACERCARSE:                                  # no contaminar las metas de exploracion
            self.goal = None
            return
        if self.goal is not None:
            self.last_goal = self.goal                       # ancla de la estrategia "rama" (ver planner)
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
        if self.state == ACERCARSE:
            self.cara_fallida = True                                 # no llego al punto de vista de la cara
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
            if arrived in (ACERCARSE, VOLVER):
                self._llego_cara = True                              # fin de la ruta: cuenta como llegada (ver step)
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
