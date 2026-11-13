"""
Navegacion de la carrera (Tarea A, carrera-1): A -> B -> A lo mas rapido posible. Sin E/S: recibe
pose, scan, mapa y lo que vio la camara, y devuelve (v, w). Asi se prueba igual en el simulador.

Estados:
  PAUSA      quieto (arranque seguro, o la tecla espacio)
  IDA        avanzar rapido por el espacio libre. En cada paso se elige un "carril" (rumbo por el que
             el robot cabe, medido con el LiDAR) que tenga espacio, siga la direccion de avance o la
             que marco una flecha, y no lleve a donde el robot ya estuvo (asi sale solo de una rama
             sin salida y no repite caminos)
  ACERCAR    se vio una cara (2 detecciones): ir hacia ella hasta tenerla grande en la imagen,
             votando quien es con cada foto
  RECONOCER  quieto frente a la foto hasta que la votacion decide (o se acaba el tiempo: entonces
             gana el mas votado, nunca "desconocido"; sin ningun voto no decide: se acerca mas)
  VOLVER     ruta mas corta a A sobre el mapa armado en la ida (Dijkstra de auto-1); si no hay o no
             avanza, Dijkstra con margen chico o el camino de ida acortado con rectas. A toda velocidad
  HECHO      en A: parado; quien llama anuncia el nombre
  EXPLORAR   plan B si la IDA deja de descubrir sitios nuevos: exploracion por fronteras y cobertura de
             la camara (auto-1), mas lenta pero sistematica. Si ve la cara, pasa a ACERCAR

El movimiento sale de una ventana dinamica: entre arcos (v, w) de 1.2 s se elige el que mas avanza hacia
el rumbo deseado sin pasar a menos de 20 cm de nada. La seguridad final es del puente (freno a 0.28 m).
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

PAUSA, IDA, ACERCAR, RECONOCER, VOLVER, HECHO, EXPLORAR = (
    "PAUSA", "IDA", "ACERCAR", "RECONOCER", "VOLVER", "HECHO", "EXPLORAR")
BUSCANDO = (IDA, EXPLORAR)
ANGS = np.radians(np.arange(-180, 180, 5.0))           # rumbos candidatos, marco del robot


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Config:
    vmax: float = 0.30              # m/s en la ida (el Create 3 llega a ~0.31)
    vmax_vuelta: float = 0.31       # m/s en la vuelta (camino conocido)
    wmax: float = 1.8               # rad/s
    medio_ancho: float = 0.18       # m: medio ancho del carril para ELEGIR rumbo (radio 0.17 + 1 cm)
    libre_min: float = 0.42         # m: un rumbo cuenta si hay al menos esto libre
    radio_seguro: float = 0.20      # m: los arcos que se prueban no pasan a menos de esto de nada
    d_flecha_armar: float = 2.2     # m: a esta distancia de una flecha confirmada se arma el giro
    px_objetivo: float = 46.0       # px: ancho de cara con el que se deja de acercar
    d_parada: float = 0.55          # m: no acercarse mas que esto a la foto
    d_max_b: float = 1.2            # m: para reconocer "en B" hay que estar al menos asi de cerca (salvo --temprano)
    temprano: bool = False          # volver apenas se decide quien es, sin terminar de acercarse
    d_temprano: float = 1.3         # m: con `temprano`, volver si ya esta a esto de la foto
    estancado_s: float = 25.0       # s sin pisar sitios nuevos -> EXPLORAR
    reconocer_max_s: float = 6.0    # s quieto frente a la foto antes de decidir con lo que haya
    usar_flechas: bool = True
    celda: float = 0.30             # m: celdas del mapa de "ya estuve aqui"


# arcos que prueba la ventana dinamica: 6 velocidades de 0 a vmax y 19 giros, mantenidos 1.2 s
T_ARCO = np.arange(1, 13) * 0.1
RADIO_FISICO = 0.175            # m: radio del Create 3 (0.17) + 5 mm; ningun arco pasa mas cerca que esto
FRENO = 0.30                    # m: el puente frena con algo a < 0.28 m en el pasillo del robot (+-0.20 m)
FRENO_SOLTAR = 0.40             # m: ...y no suelta hasta tener 0.38 m libres (histeresis)


def arcos(vmax, wmax):
    """-> v, w (K,) y la trayectoria x, y, th (K, pasos) de cada arco, en el marco del robot."""
    V, W = np.meshgrid(np.linspace(0.0, vmax, 6), np.linspace(-wmax, wmax, 19), indexing="ij")
    v, w = V.ravel(), W.ravel()
    th = w[:, None] * T_ARCO[None, :]
    recto = np.abs(w[:, None]) < 1e-6
    wn = np.where(recto, 1.0, w[:, None])
    x = np.where(recto, v[:, None] * T_ARCO[None, :], v[:, None] / wn * np.sin(th))
    y = np.where(recto, 0.0, v[:, None] / wn * (1 - np.cos(th)))
    return v, w, x, y, th


def ventana_dinamica(px, py, h_rel, vmax, wmax, radio=0.20):
    """Elige (v, w) entre los arcos de 1.2 s que no pasan a menos de `radio` de ningun punto del LiDAR
    (px, py en el marco ACTUAL del robot): el que mas avanza hacia `h_rel` (rad, marco del robot), con
    buena orientacion al final y lejos de lo ocupado. None si ninguno es seguro.
    Si el robot ya esta a menos de `radio` de algo (paso angosto), valen los arcos que no lo acercan
    mas, sin bajar nunca de RADIO_FISICO: asi puede salir en vez de quedarse girando."""
    v, w, x, y, th = arcos(vmax, wmax)
    d_pts = np.hypot(px, py)
    d0 = float(d_pts.min()) if d_pts.size else 9.0
    if d0 < radio:
        radio = max(RADIO_FISICO, d0 - 0.005)
    cerca = d_pts < 1.0
    px, py = px[cerca], py[cerca]
    d_fin = np.full(len(v), 9.0)
    if px.size:
        dx, dy = px[None, None, :] - x[..., None], py[None, None, :] - y[..., None]      # (K, pasos, M)
        dd = np.sqrt(dx ** 2 + dy ** 2)
        dmin, d_fin = dd.min(axis=(1, 2)), dd[:, -1, :].min(axis=1)
        # la regla del puente en cada paso del arco: nada en su pasillo (+-0.20 m) a menos de FRENO m por
        # delante, o frena en seco (y para soltar pide 0.38 m: si ya esta cerca, al empezar se pide eso)
        c, s = np.cos(th)[..., None], np.sin(th)[..., None]
        delante, lado = c * dx + s * dy, -s * dx + c * dy
        en_pasillo = (delante > 0) & (np.abs(lado) < 0.20)
        frena = (en_pasillo & (delante < FRENO)).any(axis=(1, 2))
        if d0 < 0.25:
            frena |= (en_pasillo[:, :1] & (delante[:, :1] < FRENO_SOLTAR)).any(axis=(1, 2))
        dmin = np.where(frena & (v > 0), 0.0, dmin)
    else:
        dmin = np.full(len(v), 9.0)
    avance = x[:, -1] * math.cos(h_rel) + y[:, -1] * math.sin(h_rel)
    orient = np.cos(wrap_arr(th[:, th.shape[1] // 2 - 1] - h_rel))      # a mitad del arco: girar con ganas
    score = 2.0 * avance + 0.6 * orient + 0.5 * np.minimum(dmin, 0.5) + 0.3 * v - 0.05 * np.abs(w)
    if d0 < 0.25:
        # pegado a algo (el puente no deja avanzar con una pared a < 0.20 m de costado): premiar despegarse
        score += 2.0 * (np.minimum(d_fin, 0.35) - d0)
    score = np.where(dmin >= radio, score, -np.inf)
    if not np.isfinite(score).any():
        return None
    k = int(np.argmax(score))
    return float(v[k]), float(w[k])


def carriles(scan, medio=0.18, rmax=4.0, angs=ANGS):
    """Distancia libre (m) en cada rumbo de `angs` (marco del robot) para un robot de ancho 2*medio:
    lo mas cercano del LiDAR dentro de esa franja recta. rmax si no hay nada."""
    if scan is None:
        return np.zeros(len(angs))
    x, y = scan.points()
    ok = np.hypot(x, y) > 0.12
    x, y = x[ok], y[ok]
    ca, sa = np.cos(angs)[:, None], np.sin(angs)[:, None]
    along = x[None, :] * ca + y[None, :] * sa
    lat = -x[None, :] * sa + y[None, :] * ca
    block = (np.abs(lat) < medio) & (along > 0)
    return np.minimum(np.where(block, along, rmax).min(axis=1, initial=rmax), rmax)


class Visitado:
    """Celdas por las que el robot ya paso (con el instante de la primera vez). Se marca todo lo que
    queda a menos de `radio` del centro: un pasillo recorrido cuenta entero, no solo la linea del centro."""

    def __init__(self, celda, radio=0.45):
        self.c = celda
        self.t = {}
        k = int(math.ceil(radio / celda))
        self._vec = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                     if math.hypot(i, j) * celda <= radio + 0.5 * celda]

    def marcar(self, x, y, now):
        ci, cj = int(math.floor(x / self.c)), int(math.floor(y / self.c))
        for i, j in self._vec:
            self.t.setdefault((ci + i, cj + j), now)

    def viejo(self, x, y, now, edad=3.0):
        t = self.t.get((int(math.floor(x / self.c)), int(math.floor(y / self.c))))
        return t is not None and now - t > edad

    def n(self):
        return len(self.t)


class Carrera:
    def __init__(self, cfg=None, planner=None, votacion=None, objetivo=None, senales=None, explorador=None,
                 planner_fino=None):
        self.cfg = cfg or Config()
        self.pl = planner                     # auto-1 Planner (para volver por el mapa); None = desandar
        self.pl_fino = planner_fino           # el mismo con margen chico (pasos estrechos)
        self.modo_ruta = 0
        self._mejor = None                    # (m que faltaban, t) para vigilar el progreso de la vuelta
        self.vot, self.obj, self.sen = votacion, objetivo, senales
        self.exp = explorador                 # auto-1 Explorer (plan B); None = sin plan B
        self.estado = PAUSA
        self.home = (0.0, 0.0)
        self.h_pref = None                    # rumbo preferido (marco del mapa)
        self.giro_hasta = -1e9                # mientras se ejecuta el giro de una flecha, h_pref no sigue al robot
        self.h_elegido = None                 # ultimo rumbo elegido (marco del mapa), para no titubear
        self._scan_ref, self.libres_scan, self.pos_scan = None, np.zeros(len(ANGS)), (0.0, 0.0, 0.0)
        self._pts_scan = (np.zeros(0), np.zeros(0))        # puntos del LiDAR (marco del robot al tomar el scan)
        self.px = self.py = np.zeros(0)                     # los mismos en el marco actual del robot
        self.vis = Visitado(self.cfg.celda)
        self.t_nuevo = None                   # ultima vez que se piso una celda nueva
        self.n_vis = 0
        self.trail = []                       # (x, y) de la ida, para desandar
        self.ruta, self.t_ruta = [], -1e9
        self.nombre = None                    # persona reconocida
        self.prob = 0.0
        self.t_inicio = self.t_b = self.t_fin = None
        self.t_reconocer = None
        self._prog = None                     # (t, x, y) para detectar que no avanza
        self._maniobra = (-1e9, 0.0)          # (hasta, w) giro en el sitio (al cambiar de ruta)
        self._desvio = (-1e9, 0.0)            # (hasta, rumbo en el mapa) para salir de un atasco
        self.log = []
        self.libres = None                    # ultimo resultado de carriles (para dibujar)
        self.phi = 0.0
        self._antes_pausa = IDA
        self.t_flecha = -1e9
        self.armado = None                    # giro pendiente de una flecha: (rumbo, posicion de la senal, t, dir)
        self.px_extra = 1.0                   # >1 si hay que acercarse mas de lo normal (no salieron votos)
        self.traza = None                     # lista para depurar: (t, estado, x, y, yaw, h_pref, phi, v, w)
        self._pos = (0.0, 0.0, 0.0)           # ultima pose recibida
        self.vista_b = np.zeros(2)            # desde donde se vio la foto de B
        self._mejor_b, self.por_mapa = None, False   # progreso al acercarse; True = acercarse por ruta del mapa

    # ---------- control externo ----------
    def empezar(self, now, yaw):
        if self.estado == PAUSA:
            if self.t_inicio is None:
                self.t_inicio = now
                self.h_pref = yaw
                self.t_nuevo = now
            self.estado = self._antes_pausa

    def pausar(self):
        if self.estado != PAUSA:
            self._antes_pausa = self.estado
            self.estado = PAUSA

    def nota(self, now, msg):
        t = 0.0 if self.t_inicio is None else now - self.t_inicio
        self.log.append(f"[{t:6.1f} s] {msg}")

    # ---------- percepcion ----------
    def ver_cara(self, now, o, peso):
        """Llamar con cada cara detectada. Devuelve True si se uso."""
        if self.obj is None or self.estado in (PAUSA, VOLVER, HECHO):
            return False
        lejos = self.obj.xy is not None and np.linalg.norm(self.obj.xy - np.array(o.xy)) > 1.2
        if lejos and self.obj.n >= 3 and self.obj.visto_hace(now) < 3.0:
            return False                                        # otra cara lejos de la que se esta siguiendo
        if lejos:
            self.obj.olvidar_posicion()                         # la de antes era otra cosa o estaba mal ubicada
        self.obj.agregar(o, peso, self._pos)
        if self.estado in BUSCANDO and self.obj.n >= 2:
            self.nota(now, f"cara a {o.dist:.1f} m ({o.px:.0f} px): ACERCAR")
            self.estado = ACERCAR
            self.vista_b = np.array(self._pos[:2], float)       # desde aqui se ve la foto
            self._mejor_b, self.por_mapa = None, False
        return True

    def ver_flecha(self, now, o):
        if self.sen is None or not self.cfg.usar_flechas or self.estado not in BUSCANDO:
            return
        s = self.sen.agregar(o)
        if s is not None and s.n == 2 and s.confirmada():
            self.nota(now, f"flecha '{s.dir()}' confirmada en ({s.xy[0]:.1f}, {s.xy[1]:.1f})")

    # ---------- bucle ----------
    def paso(self, now, pos, scan, mapa=None, fotos=0, cam_ok=True, pos_scan=None):
        """-> (v, w). pos = (x, y, yaw) ACTUAL en el marco del mapa (origen = A); pos_scan = pose cuando
        se tomo `scan` (el LiDAR va a ~7 Hz y el robot se mueve entre scans)."""
        if scan is not self._scan_ref:
            self._scan_ref = scan
            self.libres_scan = carriles(scan, self.cfg.medio_ancho)
            self.pos_scan = pos if pos_scan is None else pos_scan
            if scan is not None:
                sx, sy = scan.points()
                ok = np.hypot(sx, sy) > 0.12
                self._pts_scan = (sx[ok], sy[ok])
        # carriles y puntos en el marco actual del robot (lo que se movio desde el scan)
        bx, by, bth = self.pos_scan
        self.libres = np.roll(self.libres_scan, int(round(wrap(bth - pos[2]) / math.radians(5))))
        sx, sy = self._pts_scan
        gx = bx + math.cos(bth) * sx - math.sin(bth) * sy - pos[0]
        gy = by + math.sin(bth) * sx + math.cos(bth) * sy - pos[1]
        c, s = math.cos(-pos[2]), math.sin(-pos[2])
        self.px, self.py = c * gx - s * gy, s * gx + c * gy
        self._pos = pos
        v, w = self._paso(now, pos, scan, mapa, fotos, cam_ok)
        if self.traza is not None:
            self.traza.append((now, self.estado, *pos, self.h_pref, self.phi, v, w))
        return v, w

    @property
    def quiere_mapa(self):
        """Si este paso necesita la imagen del mapa (cuesta ~1 ms: solo se pide cuando hace falta)."""
        return self.estado in (VOLVER, EXPLORAR) or (self.estado == ACERCAR and self.por_mapa)

    def _paso(self, now, pos, scan, mapa, fotos, cam_ok):
        x, y, th = pos
        if self.estado in (PAUSA, HECHO):
            return 0.0, 0.0
        if self.estado in BUSCANDO + (ACERCAR,):
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.1:
                self.trail.append((x, y))
            self.vis.marcar(x, y, now)
            if self.vis.n() != self.n_vis:
                self.n_vis, self.t_nuevo = self.vis.n(), now
        if now < self._maniobra[0]:
            return 0.0, self._maniobra[1]
        if self.estado == IDA:
            return self._ida(now, pos, scan)
        if self.estado == EXPLORAR:
            return self._explorar(now, pos, scan, mapa, fotos, cam_ok)
        if self.estado == ACERCAR:
            return self._acercar(now, pos, mapa)
        if self.estado == RECONOCER:
            return self._reconocer(now, pos)
        if self.estado == VOLVER:
            return self._volver(now, pos, mapa)
        return 0.0, 0.0

    # ---------- piezas ----------
    def _elegir(self, now, pos, h_obj, w_obj, w_vis, giro_coste=0.25):
        """Mejor rumbo (marco del robot) segun espacio libre, objetivo y sitios ya visitados."""
        x, y, th = pos
        L = self.libres
        rel = wrap_arr(h_obj - th - ANGS) if h_obj is not None else None
        score = np.clip((L - 0.4) / 1.6, 0.0, 1.0)            # 0 a 0.4 m libres, 1 desde 2 m
        score -= np.where(L < 0.65, 0.4, 0.0)                   # carril que no lleva a ningun lado
        if rel is not None:
            score += w_obj * np.cos(rel)
        if w_vis > 0:
            vis = np.zeros(len(ANGS))
            for k, a in enumerate(ANGS):
                if L[k] < self.cfg.libre_min:
                    continue
                ds = [d for d in (0.5, 1.0, 1.6) if d < L[k] + 0.3] or [L[k] * 0.8]
                vis[k] = np.mean([self.vis.viejo(x + d * math.cos(th + a), y + d * math.sin(th + a), now) for d in ds])
            score -= w_vis * vis
        score -= giro_coste * np.abs(ANGS) / math.pi
        if self.h_elegido is not None:
            score -= 0.3 * np.abs(wrap_arr(th + ANGS - self.h_elegido)) / math.pi
        score = np.where(L >= self.cfg.libre_min, score, -np.inf)
        if not np.isfinite(score).any():
            return None
        k = int(np.argmax(score))
        self.h_elegido = wrap(th + float(ANGS[k]))
        return float(ANGS[k])

    def _mover(self, now, pos, phi, vmax):
        """Del rumbo elegido a (v, w) con la ventana dinamica: el arco seguro que mas avanza hacia el."""
        x, y, th = pos
        if now < self._desvio[0]:                               # desvio para salir de un atasco
            phi = wrap(self._desvio[1] - th)
        self.phi = phi
        if phi is None:                                         # encerrado: girar hacia lo mas abierto
            k = int(np.argmax(self.libres))
            return 0.0, math.copysign(self.cfg.wmax * 0.6, ANGS[k] if ANGS[k] != 0 else 1.0)
        r = ventana_dinamica(self.px, self.py, phi, vmax, self.cfg.wmax, self.cfg.radio_seguro)
        if r is None:                                           # ningun arco seguro: girar en el sitio
            v, w = 0.0, math.copysign(self.cfg.wmax * 0.6, phi if phi != 0 else 1.0)
        else:
            v, w = r
        self._vigilar_atasco(now, pos)
        return v, w

    def _vigilar_atasco(self, now, pos):
        """Si en 3 s no se movio ni 6 cm (pared o caja delante, o el freno del puente activado), se desvia
        2.5 s hacia el carril mas abierto que no sea el de enfrente y luego retoma su objetivo."""
        x, y, th = pos
        if self._prog is None or math.hypot(x - self._prog[1], y - self._prog[2]) > 0.06:
            self._prog = (now, x, y)
            return
        if now - self._prog[0] < 3.0 or now < self._desvio[0]:
            return
        L = np.where(np.abs(ANGS) >= math.radians(45), self.libres, 0)
        k = int(np.argmax(L))
        self._desvio = (now + 2.5, wrap(th + float(ANGS[k])))
        self._prog = (now, x, y)
        self.nota(now, f"no avanza: me desvio {math.degrees(ANGS[k]):+.0f} grados ({L[k]:.1f} m libres)")

    def _ida(self, now, pos, scan):
        x, y, th = pos
        # flechas confirmadas cerca: "armar" el giro hacia donde indican (respecto a la direccion de
        # avance) y girar recien cuando aparezca una abertura de ese lado, cerca de la senal
        if self.sen is not None and self.cfg.usar_flechas and now - self.t_flecha > 3.0:
            for s in self.sen.pendientes():
                dx, dy = s.xy[0] - x, s.xy[1] - y
                d = math.hypot(dx, dy)
                delante = abs(wrap(math.atan2(dy, dx) - th)) < math.radians(75)
                if delante and d < self.cfg.d_flecha_armar:
                    giro = {"izq": math.pi / 2, "der": -math.pi / 2}.get(s.dir(), 0.0)
                    self.armado = (wrap(self.h_pref + giro), s.xy.copy(), now, s.dir())
                    self.t_flecha = now
                    self.sen.usar_cerca(s.xy)
                    self.nota(now, f"flecha '{s.dir()}' a {d:.1f} m: giro en la proxima abertura")
                    break
                elif not delante and d < 1.0:
                    s.usada = True                              # ya la paso de largo
        if self.armado is not None:
            obj, sxy, t_arm, dir_ = self.armado
            hacia = np.array(sxy) - np.array([x, y])
            falta = float(hacia @ np.array([math.cos(self.h_pref), math.sin(self.h_pref)]))   # m hasta la senal, a lo largo
            lado = np.abs(wrap_arr(th + ANGS - obj)) < math.radians(35)
            if (lado & (self.libres >= 1.0)).any() and falta < 1.4:
                self.h_pref = obj
                self.giro_hasta = now + 6.0
                self.armado = None
                self.nota(now, f"giro '{dir_}'")
            elif now - t_arm > 12.0:
                self.armado = None
                self.nota(now, f"la flecha '{dir_}' no lleva a ninguna abertura: la ignoro")
        # rumbo preferido: sigue la direccion de avance (inercia), salvo mientras se ejecuta un giro
        if now < self.giro_hasta:
            if abs(wrap(th - self.h_pref)) < math.radians(20):
                self.giro_hasta = -1e9
        else:
            self.h_pref = wrap(self.h_pref + 0.04 * wrap(th - self.h_pref))
        if self.exp is not None and now - self.t_nuevo > self.cfg.estancado_s:
            self.nota(now, f"{self.cfg.estancado_s:.0f} s sin sitios nuevos: EXPLORAR (plan B)")
            self.estado = EXPLORAR
            self.exp.start()
            return 0.0, 0.0
        phi = self._elegir(now, pos, self.h_pref, 0.9 if now < self.giro_hasta else 0.55, 0.9)
        return self._mover(now, pos, phi, self.cfg.vmax)

    def _explorar(self, now, pos, scan, mapa, fotos, cam_ok):
        if mapa is None:
            return 0.0, 0.0
        v, w = self.exp.step(now, mapa, pos, scan, photos=fotos, cam_ok=cam_ok)
        if self.exp.state in ("HECHO", "SIN_B"):
            self.nota(now, "exploracion terminada sin ver la foto: vuelvo a IDA")
            self.estado = IDA
            self.t_nuevo = now
        return v, w

    def _objetivo(self, pos):
        x, y, th = pos
        dx, dy = self.obj.xy[0] - x, self.obj.xy[1] - y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def _acercar(self, now, pos, mapa):
        d, hacia = self._objetivo(pos)
        o = self.obj.ultima
        sin_ver = self.obj.visto_hace(now)
        if sin_ver > 2.0:
            # perdida de vista: ir (por el mapa si hace falta) a donde estaba y, ya cerca, girar buscandola
            if sin_ver > 30.0:
                self.nota(now, "no encuentro la cara donde la vi: vuelvo a explorar")
                self.estado = IDA
                self.h_pref = hacia
                self.giro_hasta = now + 4.0
                return 0.0, 0.0
            if sin_ver > 8.0 and not self.por_mapa:
                self.por_mapa = True
                self._reset_ruta()
                self.nota(now, "perdi la cara: voy por el mapa a donde la vi")
            if d > 0.9:
                if self.por_mapa and mapa is not None:
                    return self._por_ruta(now, pos, mapa, self._frente_b(), 0.2, ("mapa", "mapa fino"), "el regreso a B")
                phi = self._elegir(now, pos, hacia, 2.0, 0.0, giro_coste=0.1)
                return self._mover(now, pos, phi, 0.18)
            err = wrap(hacia - pos[2])
            return 0.0, float(np.clip(1.5 * err, -1.0, 1.0)) if abs(err) > 0.15 else 0.9   # mirar alrededor
        if self.cfg.temprano and self.vot.decidido() and d < self.cfg.d_temprano:
            return self._decidir(now, "temprano")
        cerca = d <= self.cfg.d_parada + 0.05 and self.px_extra == 1.0
        grande = o.px >= self.cfg.px_objetivo * self.px_extra and o.t > now - 1.0 and d <= self.cfg.d_max_b
        if grande or cerca or o.borde_sup:
            self.nota(now, f"frente a la foto: {d:.2f} m, cara de {o.px:.0f} px: RECONOCER")
            self.estado = RECONOCER
            self.t_reconocer = now
            self.t_b = now
            return 0.0, 0.0
        vmax = min(self.cfg.vmax, max(0.06, 0.7 * (d - self.cfg.d_parada)))
        if now - o.t < 0.6 and self.obj.pose_ultima is not None:
            hacia = wrap(self.obj.pose_ultima[2] + o.rumbo)     # recien vista: apuntar a donde se ve (servo visual)
        # si en 6 s no se acerca 15 cm (obstaculos en el medio), ir por una ruta del mapa a un punto frente a la foto
        if self._mejor_b is None or d < self._mejor_b[0] - 0.15:
            self._mejor_b = (d, now)
        elif now - self._mejor_b[1] > 6.0 and not self.por_mapa:
            self.por_mapa = True
            self._reset_ruta()
            self.nota(now, "no logro acercarme directo: voy por una ruta del mapa")
        if self.por_mapa and mapa is not None:
            return self._por_ruta(now, pos, mapa, self._frente_b(), vmax, ("mapa", "mapa fino"), "el acercamiento")
        phi = self._elegir(now, pos, hacia, 2.0, 0.0, giro_coste=0.1)
        return self._mover(now, pos, phi, vmax)

    def _frente_b(self):
        """Punto a 0.7 m frente a la foto de B, del lado desde donde se la vio."""
        u = self.vista_b - self.obj.xy
        u = u / max(np.linalg.norm(u), 1e-6)
        return tuple(self.obj.xy + 0.7 * u)

    def _reconocer(self, now, pos):
        o = self.obj.ultima
        if self.vot.decidido():
            return self._decidir(now, "votacion")
        sin_ver = self.obj.visto_hace(now)
        if self.vot.n > 0 and (now - self.t_reconocer > self.cfg.reconocer_max_s or sin_ver > 2.5):
            return self._decidir(now, "tiempo")
        if self.vot.n == 0 and now - self.t_reconocer > self.cfg.reconocer_max_s:
            # nunca se decide sin votos: si no esta pegada al borde de arriba, acercarse mas
            if not o.borde_sup:
                self.nota(now, "sin votos todavia: me acerco mas")
                self.estado = ACERCAR
                self.px_extra = 1.5
            self.t_reconocer = now
        if sin_ver > 2.5:                                       # sin cara: girar hacia donde estaba
            d, hacia = self._objetivo(pos)
            return 0.0, float(np.clip(1.5 * wrap(hacia - pos[2]), -0.8, 0.8))
        w = 1.5 * o.rumbo if abs(o.rumbo) > math.radians(6) and now - o.t < 0.6 else 0.0   # centrar la cara
        return 0.0, float(np.clip(w, -0.6, 0.6))

    def _decidir(self, now, por):
        self.nombre, self.prob = self.vot.lider()
        self.t_b = self.t_b or now
        top = ", ".join(f"{n} {p:.2f}" for n, p in self.vot.top(3))
        self.nota(now, f"ES {self.nombre} (p={self.prob:.2f}, {self.vot.n} fotos, por {por}; {top}): VOLVER")
        self.estado = VOLVER
        self._reset_ruta()
        self.h_elegido = None
        return 0.0, 0.0

    def _volver(self, now, pos, mapa):
        x, y, th = pos
        d_home = math.hypot(x - self.home[0], y - self.home[1])
        if d_home < 0.22:
            self.estado = HECHO
            self.t_fin = now
            self.nota(now, f"de vuelta en A ({d_home:.2f} m)")
            return 0.0, 0.0
        vmax = min(self.cfg.vmax_vuelta, max(0.08, 0.9 * d_home))
        return self._por_ruta(now, pos, mapa, self.home, vmax, MODOS_RUTA, "la vuelta")

    def _reset_ruta(self):
        self.ruta, self.t_ruta, self._mejor, self.modo_ruta = [], -1e9, None, 0

    def _por_ruta(self, now, pos, mapa, destino, vmax, modos, que):
        """Ir a `destino` siguiendo una ruta del mapa (se recalcula cada 2 s) con pure pursuit. Si en 8 s
        no se acorta lo que falta, prueba la siguiente forma de calcular la ruta (`modos`)."""
        x, y, th = pos
        if now - self.t_ruta > 2.0 or not self.ruta:
            self.t_ruta = now
            self.ruta = self._ruta_hacia(mapa, (x, y), destino, modos)
        resta, meta = self._seguir(x, y)
        if math.hypot(x - destino[0], y - destino[1]) < 0.6:
            meta = destino                                      # el ultimo tramo, al punto exacto
        if self._mejor is None or resta < self._mejor[0] - 0.15:
            self._mejor = (resta, now)
        elif now - self._mejor[1] > 8.0:
            self.modo_ruta = (self.modo_ruta + 1) % len(modos)
            self._mejor, self.t_ruta = (resta, now), -1e9
            L = np.where(np.abs(ANGS) > math.radians(60), self.libres, 0)
            self._maniobra = (now + 1.0, math.copysign(self.cfg.wmax * 0.7, ANGS[int(np.argmax(L))]))
            self.nota(now, f"{que} no avanza: pruebo la ruta '{modos[self.modo_ruta]}'")
            return 0.0, 0.0
        hacia = math.atan2(meta[1] - y, meta[0] - x)
        phi = self._elegir(now, pos, hacia, 2.5, 0.0, giro_coste=0.1)
        return self._mover(now, pos, phi, vmax)

    def _seguir(self, x, y, adelante=0.6):
        """-> (m de ruta que faltan, punto a `adelante` m por la ruta) (pure pursuit)."""
        r = self.ruta or [self.home]
        d = [math.hypot(px - x, py - y) for px, py in r]
        i = int(np.argmin(d))
        resta = d[i] + sum(math.hypot(r[k + 1][0] - r[k][0], r[k + 1][1] - r[k][1]) for k in range(i, len(r) - 1))
        j = i
        while j < len(r) - 1 and math.hypot(r[j][0] - x, r[j][1] - y) < adelante:
            j += 1
        return resta, r[j]

    def _ruta_hacia(self, mapa, xy, destino, modos):
        """Ruta a `destino`. Se prueban, desde `modo_ruta`: Dijkstra en el mapa (margen normal), Dijkstra
        con margen chico (pasos estrechos por los que el robot si paso) y, solo para volver a A, el
        camino de ida al reves, acortado con rectas donde el mapa muestra paso libre."""
        for k in range(len(modos)):
            modo = modos[(self.modo_ruta + k) % len(modos)]
            if modo == "ida":
                if not self.trail:
                    return [destino]
                d = [math.hypot(px - xy[0], py - xy[1]) for px, py in self.trail]
                i = int(np.argmin(d))
                ida = [xy] + self.trail[i::-2] + [destino]       # un punto cada ~0.2 m
                return self._tirar(ida, mapa)
            pl = {"mapa": self.pl, "mapa fino": self.pl_fino}.get(modo)
            if pl is None or mapa is None:
                continue
            res = pl.plan_to(mapa, xy, destino)
            if res.status == "ok" and len(res.path) > 1:
                return res.path + [destino]
        return [destino]

    def _tirar(self, pts, mapa, holgura=0.19, alcance=60):
        """Acorta un camino: desde cada punto salta al mas lejano (de los `alcance` siguientes) al que se
        llega en linea recta con `holgura` m a lo ocupado y por celdas libres (string pulling)."""
        if mapa is None or self.pl is None or len(pts) < 3:
            return pts
        res, n = self.pl.res, self.pl.n
        libre = mapa >= 150
        dist = cv2.distanceTransform((mapa >= 90).astype(np.uint8), cv2.DIST_L2, 3) * res

        def recta(a, b):
            k = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.04))
            c = (n / 2 + np.linspace(a[0], b[0], k) / res).astype(int)
            r = (n / 2 + np.linspace(a[1], b[1], k) / res).astype(int)
            if c.min() < 0 or r.min() < 0 or c.max() >= n or r.max() >= n:
                return False
            return bool((dist[r, c] >= holgura).all() and libre[r, c].mean() > 0.9)

        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = i + 1
            for k in range(min(len(pts) - 1, i + alcance), i + 1, -1):
                if recta(pts[i], pts[k]):
                    j = k
                    break
            out.append(pts[j])
            i = j
        return out


MODOS_RUTA = ("mapa", "mapa fino", "ida")


def wrap_arr(a):
    return np.arctan2(np.sin(a), np.cos(a))
