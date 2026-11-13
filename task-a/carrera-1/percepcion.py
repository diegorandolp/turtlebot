"""
Percepcion por camara (Tarea A, carrera-1): de cada foto saca las caras y las flechas, con su rumbo
(angulo respecto al frente del robot) y su posicion en el mapa (rumbo + distancia que da el LiDAR en
ese rumbo: la foto y las senales estan pegadas a una superficie que el LiDAR ve).

  Percepcion.procesar(img, pose, scan) -> (caras, flechas)        listas de Obs
  Objetivo     junta las caras vistas: donde esta B y la votacion de quien es
  Senales      junta las flechas vistas: cada senal con su posicion y su sentido confirmado
  CamaraHilo   (robot real) pide fotos sin parar en otro hilo y las procesa; el bucle de control
               solo recoge resultados, asi nunca se frena esperando la camara
"""
import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from flechas import ARRIBA, DER, IZQ


@dataclass
class Obs:
    tipo: str                   # "cara" | "flecha"
    t: float                    # instante de la foto
    rumbo: float                # rad, + izquierda, respecto al frente del robot al tomar la foto
    dist: float                 # m hasta la superficie (LiDAR; si no hay lectura, por el tamano)
    xy: tuple                   # posicion estimada en el marco del mapa
    px: float                   # ancho de la cara / lado de la flecha, en px
    caja: tuple                 # (x, y, w, h) en la imagen
    dir: str = ""               # flechas: der | izq | arriba | abajo
    res: object = None          # caras: reconocimiento.Resultado (si la cara es bastante grande)
    borde_sup: bool = False     # la cara toca el borde de arriba (al acercarse mas se sale de la foto)
    girando: bool = False       # foto tomada girando rapido: el rumbo y la distancia son poco fiables


def distancia_lidar(scan, rumbo, ancho=math.radians(2.5)):
    """Mediana de las lecturas del LiDAR en +-ancho alrededor de `rumbo` (marco del robot). nan si no hay."""
    if scan is None:
        return math.nan
    a = np.arctan2(np.sin(scan.angles - rumbo), np.cos(scan.angles - rumbo))
    r = np.asarray(scan.ranges, float)
    sel = (np.abs(a) <= ancho) & np.isfinite(r) & (r > 0.12)
    return float(np.median(r[sel])) if sel.any() else math.nan


class Percepcion:
    def __init__(self, reconocedor, det_flechas, hfov=40.0, min_px_reconocer=10, cara_m=0.11, flecha_m=0.14):
        self.rec, self.det_fl = reconocedor, det_flechas
        self.hfov = math.radians(hfov)
        self.min_px = min_px_reconocer
        self.cara_m, self.flecha_m = cara_m, flecha_m     # tamanos supuestos, solo si el LiDAR no da distancia

    def _rumbo(self, u, W):
        f = (W / 2) / math.tan(self.hfov / 2)
        return math.atan(((W - 1) / 2 - u) / f), f

    def _ubicar(self, u, W, px, tam_m, pose, scan, dth, rango=(0.5, 2.2)):
        """Rumbo, distancia y posicion en el mapa de algo visto en la columna `u`. La distancia es la del
        LiDAR en ese rumbo, salvo que no haya lectura o no cuadre con el tamano aparente (d/d_tam fuera
        de `rango`: algo tapa la senal o el LiDAR pasa al lado); entonces se usa el tamano supuesto."""
        rumbo, f = self._rumbo(u, W)
        d = distancia_lidar(scan, rumbo + dth)
        d_tam = f * tam_m / max(px, 1.0)
        if not math.isfinite(d) or not (rango[0] * d_tam < d < rango[1] * d_tam):
            d = d_tam
        x, y, th = pose
        return rumbo, d, (x + d * math.cos(th + rumbo), y + d * math.sin(th + rumbo))

    def procesar(self, img, pose, scan, t, w=0.0, th_scan=None):
        """pose = (x, y, yaw) al tomar la foto; th_scan = rumbo cuando se tomo `scan`.
        `w`: velocidad de giro (rad/s) al tomar la foto. Girando rapido las posiciones son poco
        fiables (se marcan)."""
        H, W = img.shape[:2]
        dth = 0.0 if th_scan is None else math.atan2(math.sin(pose[2] - th_scan), math.cos(pose[2] - th_scan))
        girando = abs(w) > 0.6
        caras, flechas = [], []
        for c in self.rec.det(img)[:3]:
            if c.score < 0.7:
                continue
            u, _ = c.centro
            # la cara de una foto puede medir de 4 a 15 cm: el tamano casi no sirve, manda el LiDAR
            rumbo, d, xy = self._ubicar(u, W, c.ancho, self.cara_m, pose, scan, dth, rango=(0.3, 1.6))
            res = self.rec.resultado(img, c) if c.ancho >= self.min_px else None
            caras.append(Obs("cara", t, rumbo, d, xy, c.ancho, c.caja, res=res,
                             borde_sup=c.caja[1] < 0.04 * H, girando=girando))
        cajas = [o.caja for o in caras]
        for f in self.det_fl(img)[:4]:
            u, v = f.centro
            if any(x - 0.5 * w <= u <= x + 1.5 * w and y - 0.5 * h <= v <= y + 2.0 * h for x, y, w, h in cajas):
                continue                                  # ojos, boca o ropa de la foto de B: no son flechas
            rumbo, d, xy = self._ubicar(u, W, f.lado, self.flecha_m, pose, scan, dth)
            flechas.append(Obs("flecha", t, rumbo, d, xy, f.lado, f.caja, dir=f.dir, girando=girando))
        return caras, flechas


# ---------------------------------------------------------------- memoria de lo visto

class Objetivo:
    """La foto de B: posicion (mediana de las ultimas detecciones, asi una mala no la arrastra) y
    votacion de quien es."""

    def __init__(self, votacion, recientes=9):
        self.vot = votacion
        self.xy = None
        self.n = 0
        self.t_ultima = -1e9
        self.ultima = None          # ultima Obs
        self.pose_ultima = None     # pose del robot en la ultima deteccion (para apuntar a la cara vista)
        self.ancho_max = 0.0
        self._pos = []              # posiciones recientes (sin las fotos tomadas girando)
        self._k = recientes

    def agregar(self, o, peso, pose=None):
        if not o.girando or self.xy is None:
            self._pos = (self._pos + [o.xy])[-self._k:]
            self.xy = np.median(np.array(self._pos, float), axis=0)
        self.n += 1
        self.t_ultima = o.t
        self.ultima = o
        self.pose_ultima = pose
        self.ancho_max = max(self.ancho_max, o.px)
        if o.res is not None:
            self.vot.agregar(o.res.sims, peso)

    def visto_hace(self, now):
        return now - self.t_ultima

    def olvidar_posicion(self):
        self._pos, self.xy = [], None


@dataclass
class Senal:
    xy: np.ndarray
    votos: dict = field(default_factory=dict)
    n: int = 0
    t0: float = 0.0
    t1: float = 0.0
    usada: bool = False
    px_max: float = 0.0

    def dir(self):
        return max(self.votos, key=self.votos.get) if self.votos else ""

    def confirmada(self, min_n=2, mayoria=0.7):
        if self.n < min_n:
            return False
        return self.votos[self.dir()] / self.n >= mayoria


class Senales:
    """Flechas vistas, agrupadas por posicion: varias fotos de la misma senal votan su sentido."""

    def __init__(self, radio=1.0):
        self.radio = radio
        self.lista = []

    def agregar(self, o):
        """Suma una foto de una flecha. None si se descarta (tomada girando rapido)."""
        if o.girando:
            return None
        p = np.array(o.xy, float)
        cerca = [s for s in self.lista if np.linalg.norm(s.xy - p) < self.radio]
        if cerca:
            s = min(cerca, key=lambda s: np.linalg.norm(s.xy - p))
            k = 0.3 if o.dist < 1.5 else 0.15
            s.xy = (1 - k) * s.xy + k * p
        else:
            s = Senal(p, t0=o.t)
            self.lista.append(s)
        s.votos[o.dir] = s.votos.get(o.dir, 0) + 1
        s.n += 1
        s.t1 = o.t
        s.px_max = max(s.px_max, o.px)
        return s

    def usar_cerca(self, xy, radio=1.6):
        """Al tomar una flecha, las que esten cerca son la misma senal vista desde otro sitio."""
        for s in self.lista:
            if np.linalg.norm(s.xy - np.asarray(xy)) < radio:
                s.usada = True

    def pendientes(self):
        return [s for s in self.lista if not s.usada and s.confirmada() and s.dir() in (IZQ, DER, ARRIBA)]


# ---------------------------------------------------------------- robot real: camara en otro hilo

class CamaraHilo:
    """Pide fotos al robot sin parar, las procesa y deja los resultados en una cola.
    `sensores()` -> (pose actual en el marco del mapa, scan, rumbo al tomar el scan, w); se lee al
    pedir la foto."""

    def __init__(self, robot, percepcion, sensores, hd=False):
        self.robot, self.per, self.sensores, self.hd = robot, percepcion, sensores, hd
        self.lock = threading.Lock()
        self.resultados = []
        self.ultima_img = None
        self.fotos = 0
        self.errores = 0
        self.ultimo_error = ""
        self.t_ok = None
        self.activa = True
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
            if not self.activa:
                time.sleep(0.05)
                continue
            (pose, scan, th_scan, w), t = self.sensores(), time.monotonic()
            try:
                img = self.robot.photo(timeout=2.0, hd=True) if self.hd else self.robot.photo(timeout=2.0)
            except Exception as e:                        # camara caida, en el dock, timeout
                self.errores += 1
                self.ultimo_error = f"{type(e).__name__}: {e}"[:80]
                time.sleep(0.5)
                continue
            if pose is None:
                continue
            caras, flechas = self.per.procesar(img, pose, scan, t, w, th_scan)
            with self.lock:
                self.resultados.append((t, img, caras, flechas))
                self.ultima_img = (img, caras, flechas)
                self.fotos += 1
                self.t_ok = time.monotonic()

    def recoger(self):
        with self.lock:
            r, self.resultados = self.resultados, []
        return r

    def close(self):
        self._run = False
