"""
Percepcion por camara (Tarea A, carrera-1): de cada foto saca las caras y las flechas, con su rumbo
(angulo respecto al frente del robot) y su posicion en el mapa (rumbo + distancia que da el LiDAR en
ese rumbo: la foto y las senales estan pegadas a una superficie que el LiDAR ve).

  Percepcion.procesar(img, pose, scan, leer_qr=False) -> (caras, flechas, qrs)   listas de Obs
               (con det_bandera, la bandera a cuadros de la meta va en `flechas` como Obs tipo "bandera")
  Objetivo     junta las caras vistas de UNA foto: donde esta y la votacion de quien es (con --buscar
               hay una por cada foto de la meta; ver navegador.Carrera.ver_cara)
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
from qr import buscar_qrs


@dataclass
class Obs:
    tipo: str                   # "cara" | "flecha" | "qr" | "bandera" (la bandera a cuadros de la meta, 21/11)
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
    texto: str = ""             # QR: lo que dice
    cara: object = None         # QR: la Obs de la cara (de la misma foto) a la que pertenece, o None


def distancia_lidar(scan, rumbo, ancho=math.radians(2.5)):
    """Mediana de las lecturas del LiDAR en +-ancho alrededor de `rumbo` (marco del robot). nan si no hay."""
    if scan is None:
        return math.nan
    a = np.arctan2(np.sin(scan.angles - rumbo), np.cos(scan.angles - rumbo))
    r = np.asarray(scan.ranges, float)
    sel = (np.abs(a) <= ancho) & np.isfinite(r) & (r > 0.12)
    return float(np.median(r[sel])) if sel.any() else math.nan


class Percepcion:
    def __init__(self, reconocedor, det_flechas, hfov=42.0, min_px_reconocer=10, cara_m=0.11, flecha_m=0.14,
                 hfov_hd=None, qr_m=0.10, max_caras=3, det_bandera=None, bandera_m=0.297):
        self._t_completo = -1e9       # ultima vez que YuNet reviso la imagen entera (con cajas de la camara)
        self.rec, self.det_fl = reconocedor, det_flechas
        self.det_band, self.bandera_m = det_bandera, bandera_m   # bandera.DetectorBandera (hoja A4 apaisada)
        self.hfov = math.radians(hfov)
        self.hfov_hd = math.radians(hfov_hd) if hfov_hd else self.hfov   # fotos HD (16:9): otro campo de vision
        self.qr_m = qr_m
        self.max_caras = max_caras                           # 6 con --buscar: la meta tiene varias fotos en fila
        self.min_px = min_px_reconocer
        self.cara_m, self.flecha_m = cara_m, flecha_m     # tamanos supuestos, solo si el LiDAR no da distancia

    def _rumbo(self, u, W):
        hfov = self.hfov_hd if W >= 600 else self.hfov          # la preview es 250x250; la HD, 1280x720
        f = (W / 2) / math.tan(hfov / 2)
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

    def procesar(self, img, pose, scan, t, w=0.0, th_scan=None, leer_qr=False, cajas=None):
        """pose = (x, y, yaw) al tomar la foto; th_scan = rumbo cuando se tomo `scan`.
        `w`: velocidad de giro (rad/s) al tomar la foto. Girando rapido las posiciones son poco
        fiables (se marcan). `leer_qr`: buscar tambien los QR (cuesta: solo frente a la meta).
        `cajas`: caras que la propia camara ya detecto (fraccion 0..1 del cuadro; robot/oakd_caras.md). Con
        ellas YuNet corre solo en esos recortes (1-3 ms por cara) y, si la camara no vio ninguna, la imagen
        entera se revisa solo una vez por segundo por si a la camara se le escapa una cara chica. None = sin
        ayuda de la camara: YuNet en toda la imagen, como siempre.
        -> (caras, flechas, qrs)"""
        H, W = img.shape[:2]
        dth = 0.0 if th_scan is None else math.atan2(math.sin(pose[2] - th_scan), math.cos(pose[2] - th_scan))
        girando = abs(w) > 0.6
        caras, flechas = [], []
        if cajas is None or (not cajas and t - self._t_completo > 1.0):
            self._t_completo = t
            detectadas = self.rec.det(img)
        else:
            detectadas = self.rec.det.en_cajas(img, cajas) if cajas else []
        for c in detectadas[:self.max_caras]:
            if c.score < 0.7:
                continue
            u, _ = c.centro
            # la cara de una foto puede medir de 4 a 15 cm: el tamano casi no sirve, manda el LiDAR. Si el LiDAR
            # da menos, la foto no puede estar mas lejos (las cajas tapan igual a la camara, que va mas baja)
            rumbo, d, xy = self._ubicar(u, W, c.ancho, self.cara_m, pose, scan, dth, rango=(0.12, 1.6))
            # girando rapido la foto sale corrida (19/11): solo se ubica la cara, sin ArcFace (lo mas caro en la Pi)
            res = self.rec.resultado(img, c) if c.ancho >= self.min_px and abs(w) < 1.0 else None
            caras.append(Obs("cara", t, rumbo, d, xy, c.ancho, c.caja, res=res,
                             borde_sup=c.caja[1] < 0.04 * H, girando=girando))
        cajas = [o.caja for o in caras]
        # la bandera a cuadros de la meta (21/11): en la pared del fondo del tubo. Solo en la preview (en la HD,
        # frente a la foto, es puro costo) y no mientras se gira rapido (sale corrida)
        banderas = []
        if self.det_band is not None and W <= 400 and not leer_qr and abs(w) < 1.0:
            for b in self.det_band(img)[:1]:
                u, v = b.centro
                rumbo, d, xy = self._ubicar(u, W, b.ancho, self.bandera_m, pose, scan, dth, rango=(0.4, 2.5))
                banderas.append(Obs("bandera", t, rumbo, d, xy, b.ancho, b.caja, girando=girando))
        # frente a la meta (leer_qr) las flechas ya no se usan: en una foto HD buscarlas es puro costo en la Pi
        for f in ([] if (leer_qr and W > 400) else self.det_fl(img)[:4]):
            u, v = f.centro
            if any(x - 1.6 * w <= u <= x + 2.6 * w and y - 0.8 * h <= v <= y + 4.0 * h for x, y, w, h in cajas):
                continue                                  # dentro de la hoja de una foto (ojos, ropa, el QR): no es flecha
            if any(x - 0.2 * w <= u <= x + 1.2 * w and y - 0.2 * h <= v <= y + 1.2 * h for x, y, w, h in
                   (o.caja for o in banderas)):
                continue                                  # un trozo de la bandera no es una flecha
            rumbo, d, xy = self._ubicar(u, W, f.lado, self.flecha_m, pose, scan, dth)
            flechas.append(Obs("flecha", t, rumbo, d, xy, f.lado, f.caja, dir=f.dir, girando=girando))
        flechas += banderas
        qrs = []
        if leer_qr:                                       # sin cara al lado solo cuenta frente a la buscada (ver_qr)
            for q in buscar_qrs(img, cajas):
                u = q.caja[0] + q.caja[2] / 2
                rumbo, d, xy = self._ubicar(u, W, max(q.caja[2], 1), self.qr_m, pose, scan, dth, rango=(0.4, 1.8))
                qrs.append(Obs("qr", t, rumbo, d, xy, q.caja[2], q.caja, girando=girando, texto=q.texto,
                               cara=caras[q.cara] if q.cara is not None else None))
        return caras, flechas, qrs


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
        self.estado = "pendiente"   # con --buscar: pendiente | objetivo | otra
        self.id = 0
        self.t_ignorar = -1e9       # con --buscar: no elegirla hasta este instante (no se la encontro)
        self.por_ranking = False    # con --buscar: elegida como la buscada por ser la mas parecida (ver navegador)
        self.descartada = False     # con --buscar: se la miro de cerca y es otra persona
        self.intentos_frente = 0    # con --buscar: veces que se fue a mirarla de frente porque se la veia de costado
        self.revisiones = 0         # con --buscar: veces que se la miro de cerca sin poder decir quien es (dudosa)

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
    t_usada: float = -1e9       # cuando se uso (o se paso de largo)
    px_max: float = 0.0

    def dir(self):
        return max(self.votos, key=self.votos.get) if self.votos else ""

    def confirmada(self, min_n=2, mayoria=0.7):
        if self.n < min_n:
            return False
        return self.votos[self.dir()] / self.n >= mayoria


class Senales:
    """Flechas vistas, agrupadas por posicion: varias fotos de la misma senal votan su sentido."""

    def __init__(self, radio=0.7, min_n=2):
        self.radio = radio                  # m: fotos a menos de esto son la misma senal (en un laberinto de celdas
        #                                     de 0.8 m dos senales distintas pueden estar a 0.9 m: con 1.0 se unian)
        self.min_n = min_n                  # fotos para confirmar una senal (3 si se aceptan flechas de color)
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

    def usar_cerca(self, xy, radio=0.8, t=0.0):
        """Al tomar una flecha, las que esten cerca son la misma senal vista desde otro sitio."""
        for s in self.lista:
            if np.linalg.norm(s.xy - np.asarray(xy)) < radio:
                s.usada, s.t_usada = True, t

    def pendientes(self):
        return [s for s in self.lista if not s.usada and s.confirmada(self.min_n) and s.dir() in (IZQ, DER, ARRIBA)]


# ---------------------------------------------------------------- robot real: camara en otro hilo

class CamaraHilo:
    """Pide fotos al robot sin parar, las procesa y deja los resultados en una cola.
    `sensores()` -> (pose actual en el marco del mapa, scan, rumbo al tomar el scan, w); se lee al
    pedir la foto."""

    def __init__(self, robot, percepcion, sensores, hd=False):
        self.robot, self.per, self.sensores, self.hd = robot, percepcion, sensores, hd
        self.qr = False                     # buscar QR en cada foto (frente a la meta)
        self.hd_qr = False                  # ...y pedirlas en HD mientras tanto (el QR se lee de mas lejos)
        self.hd_falla = 0                   # pedidos HD seguidos que fallaron: desde 2, se usa la preview
        self.solo_ver = False               # True = fotos solo para la ventana/video (la vuelta): sin procesar
        self.lock = threading.Lock()
        self.resultados = []
        self.ultima_img = None
        self.fotos = 0
        self.errores = 0
        self.ultimo_error = ""
        self.t_ok = None
        self.activa = True
        # tiempos medidos (s): pedir la foto al puente y procesarla, por tipo -> [suma, n]. En la Raspberry el
        # 19/11 cada foto HD tardo ~4 s en total: esto dice si es el puente (conversion + UDP) o el procesado
        self.tiempos = {k: [0.0, 0] for k in ("foto", "foto_hd", "proc", "proc_hd", "proc_qr", "proc_qr_hd")}
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
                hd = (self.hd or (self.qr and self.hd_qr)) and self.hd_falla < 2
                t_pedir = time.monotonic()
                try:
                    img = self.robot.photo(timeout=2.0, hd=True) if hd else self.robot.photo(timeout=2.0)
                    self._medir("foto_hd" if hd else "foto", time.monotonic() - t_pedir)
                    if hd:
                        self.hd_falla = 0
                except Exception:
                    if not hd:
                        raise
                    # la HD no llega (la camara no se lanzo con camara_rgb.sh --hd): seguir con la preview, que
                    # es mejor que quedarse sin camara justo frente a la meta
                    self.hd_falla += 1
                    if self.hd_falla == 2:
                        self.ultimo_error = "la foto HD no llega: sigo con la preview (lanzar camara_rgb.sh --hd)"
                    img = self.robot.photo(timeout=2.0)
            except Exception as e:                        # camara caida, en el dock, timeout
                self.errores += 1
                self.ultimo_error = f"{type(e).__name__}: {e}"[:80]
                time.sleep(0.5)
                continue
            if pose is None:
                continue
            if self.solo_ver:
                caras, flechas, qrs = [], [], []
                time.sleep(0.25)                          # ~3 fotos/s alcanzan para ver
            else:
                cajas = self.robot.det() if hasattr(self.robot, "det") else None    # caras detectadas en la camara
                t_proc = time.monotonic()
                caras, flechas, qrs = self.per.procesar(img, pose, scan, t, w, th_scan, leer_qr=self.qr,
                                                        cajas=cajas)
                grande = img.shape[1] > 400
                self._medir(("proc_qr" if self.qr else "proc") + ("_hd" if grande else ""), time.monotonic() - t_proc)
            with self.lock:
                self.resultados.append((t, img, caras, flechas, qrs))
                self.ultima_img = (img, caras, flechas)
                self.fotos += 1
                self.t_ok = time.monotonic()

    def _medir(self, que, dt):
        with self.lock:
            self.tiempos[que][0] += dt
            self.tiempos[que][1] += 1

    def resumen_tiempos(self):
        """{que: (ms promedio, n)} de lo medido (solo lo que paso al menos una vez)."""
        with self.lock:
            return {k: (round(1000 * s / n), n) for k, (s, n) in self.tiempos.items() if n}

    def recoger(self):
        with self.lock:
            r, self.resultados = self.resultados, []
        return r

    def close(self):
        self._run = False
