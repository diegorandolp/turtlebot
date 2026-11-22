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
  LEER_QR    (con --buscar) frente a la foto de la persona buscada: quieto, girando un poco a cada lado,
             acercandose y alejandose, hasta leer el QR de su esquina de abajo (o rendirse a los qr_max_s)
  FILA       (con --buscar) ya vio fotos de la meta pero ninguna es la buscada: recorre la fila de fotos
             (la recta que pasa por las que vio, un poco mas alla de cada punta), parandose frente a la
             pared cada ~0.3 m para mirarla de frente. Una foto que pueda ser la buscada -> ACERCAR
  META       (con --buscar, 21/11) vio la BANDERA a cuadros del fondo del tubo: va al centro del tubo frente a
             ella (a d_meta del fondo) y ahi, quieto, gira a un costado y al otro (6 rumbos) para ver las 4
             fotos (2 y 2). Terminado el barrido, va a la MAS PARECIDA a la buscada (ranking, sin umbral), la
             confirma de cerca y, si no es, a la siguiente; si ya descarto a todas menos una, esa es
  HECHO      en A: parado; quien llama anuncia la palabra del QR (o el nombre, si no hay QR)
  EXPLORAR   plan B si la IDA deja de descubrir sitios nuevos: exploracion por fronteras y cobertura de
             la camara (auto-1), mas lenta pero sistematica. Si ve la cara, pasa a ACERCAR

El movimiento sale de una ventana dinamica: entre arcos (v, w) de 1.2 s se elige el que mas avanza hacia
el rumbo deseado sin pasar a menos de 20 cm de nada. La seguridad final es del puente (freno a 0.28 m).

Con `Config.buscar` (la meta tiene varias fotos, cada una con un QR): cada foto vista es una "candidata"
con su propia votacion. Se va primero a la que mas probablemente es la buscada; una candidata que resulta
ser otra persona se descarta y se pasa a la siguiente (o se sigue buscando). Los QR leidos se guardan con
la foto a la que pertenecen: si el de la buscada ya se leyo al acercarse, no hace falta LEER_QR.
"""
import math
from dataclasses import dataclass, replace as dc_replace

import cv2
import numpy as np

PAUSA, IDA, ACERCAR, RECONOCER, VOLVER, HECHO, EXPLORAR, LEER_QR, FILA, META = (
    "PAUSA", "IDA", "ACERCAR", "RECONOCER", "VOLVER", "HECHO", "EXPLORAR", "LEER_QR", "FILA", "META")
BUSCANDO = (IDA, EXPLORAR, FILA, META)
ANGS = np.radians(np.arange(-180, 180, 5.0))           # rumbos candidatos, marco del robot


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Config:
    vmax: float = 0.30              # m/s en la ida (el Create 3 llega a ~0.31)
    vmax_vuelta: float = 0.31       # m/s en la vuelta (camino conocido)
    wmax: float = 1.8               # rad/s
    medio_ancho: float = 0.21       # m: medio ancho del carril para ELEGIR rumbo. >= el pasillo del freno del
                                    # puente (+-0.20 m): con 0.18 elegia carriles con una caja a 0.19 m de lado,
                                    # el freno no lo dejaba avanzar y se quedaba quieto mirando ese rumbo
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
    px_min_decidir: float = 24.0    # px: sin haber visto la cara al menos asi de grande no se decide por tiempo
                                    # (con caras de ~14 px el reconocimiento acierta ~80 %): primero acercarse
    t_limite_decidir: float = 140.0 # s de carrera desde los que se decide con lo que haya (no perder el intento)
    usar_flechas: bool = True
    celda: float = 0.30             # m: celdas del mapa de "ya estuve aqui"
    # --- varias fotos en la meta (--buscar) ---
    buscar: str = ""                # persona buscada, como en la galeria ("" = una sola foto en B: la de siempre)
    leer_qr: bool = True            # con `buscar`: leer el QR de su foto y anunciar lo que dice
    px_descartar: float = 20.0      # px: una candidata solo se descarta ("es otra") si se vio al menos asi de grande
    d_candidata: float = 0.22       # m: dos caras a menos de esto en el mapa son la misma foto (menos que la
                                    # separacion entre fotos: a 0.35 m se juntaban dos fotos vecinas)
    d_qr: float = 0.40              # m: lo mas cerca que se acerca para leer el QR
    px_qr_max: float = 110.0        # px: con la cara mas grande que esto esta demasiado cerca (la camara va a 13 cm
    #                                 del suelo: de muy cerca la cara sale cortada arriba y se reconoce mal)
    margen_qr: float = 0.08         # el QR cuenta si su cara se reconoce como la buscada con esta ventaja (coseno)
    qr_max_s: float = 30.0          # s maximos frente a la foto buscando el QR (despues vuelve igual)
    d_frente_qr: float = 0.65       # m: frente a la foto para leer el QR (21/11: 0.55 era muy cerca). El QR va DEBAJO de la foto, a la altura de
                                    # la camara (13 cm): a 0.55 m mide ~57 px en la preview y se lee sin HD, y la
                                    # cara todavia entra (a 0.45 m la de un retrato de medio cuerpo queda cortada)
    qr_preview_s: float = 5.0       # s leyendo de la preview antes de pedir HD (en la Raspberry la HD tarda ~4 s)
    px_qr_1: float = 35.0           # px: con la cara asi de grande (y clara) basta leer el QR 1 vez
    # --- decidir mas rapido (en la Raspberry hay ~2 fotos/s: esperar p >= 0.97 con peso 1 costaba ~35 s) ---
    p_temprana: float = 0.90        # una foto es la buscada con p >= esto en >= 2 vistas con la cara >= px_temprana
    px_temprana: float = 24.0       # px (a 24 px el reconocimiento acierta ~96 % por foto, 97 % por voto)
    n_fotos: int = 4                # fotos en la meta si se sabe (21/11: 4, dos en cada costado; 0 = no se sabe):
                                    # vistas todas, se elige por ranking y no se busca una mas
    mirar_s: float = 1.1            # s quieto en cada punto de la fila (la Raspberry toma ~2 fotos por s)
    d_vista_fila: tuple = (0.5, 0.45, 0.55, 0.62)   # m de la pared para mirar cada foto de la fila (preferidas
                                    # primero). Fotos chicas (cara ~5 cm): a 0.5 m la cara mide ~33 px. Las fotos van
                                    # bajas (QR en el piso, foto hasta ~25 cm: medido el 19/11) y la camara (13 cm)
                                    # las ve enteras aun a 0.35 m. Banco: 0.5 m 19/25 y 112 s, 0.62 m 17/25 y 125 s
    v_fila: float = 0.25            # m/s entre puntos de la fila
    rank_p: float = 0.0             # ranking: la mas parecida necesita p >= esto... (21/11: sin umbral, se va a la
    rank_ventaja: float = 1.0       # ...y esta ventaja (veces) sobre la segunda   mas parecida y se confirma de cerca)
    rank_confirmar: float = 0.5     # de cerca (cara >= px_confirmar) la elegida por ranking se da por buena si la
    px_confirmar: float = 24.0      # buscada es la mas votada con p >= rank_confirmar (foto chica a 0.55 m: ~25 px)
    p_aceptar: float = 0.45         # (meta) la buscada gana de cerca pero con p menor: no basta, queda dudosa (21/11:
                                    # dijo "Luis" con p=0.10, acerto de suerte; un nombre equivocado es la peor nota)
    p_descartar: float = 0.6        # ...y se DESCARTA solo si otra persona gana con p >= esto; si de cerca no se sabe
                                    # quien es (p chicas), queda "dudosa" y se revisan primero las demas
    # --- la meta del 21/11: bandera a cuadros al fondo del tubo, 4 fotos en los costados (2 y 2) ---
    d_meta: float = 0.65            # m del fondo a los que se para a mirar los costados (21/11: con 0.9 quedaba lejos) (desde ahi una foto a 0.5 m
                                    # de la pared de al lado mide ~33 px; a 1 m del punto, ~17 px: se ve, se vota poco)
    barrido_meta: tuple = (45.0, 90.0, 135.0, -135.0, -90.0, -45.0)   # grados respecto a la bandera, en orden: un
                                    # costado y luego el otro, sin pararse mirando atras (22/11: ahi no hay fotos y se
                                    # perdian ~5 s); de 135 a -135 pasa por atras sin detenerse
    confirmar_hd: bool = False      # (carrera.py lo activa con la bandera) confirmar cada foto de cerca con fotos HD si en la preview la cara es chica
    px_hd_confirmar: float = 40.0   # px: con la cara asi de grande ya no hace falta HD
    t_confirmar_max: float = 11.0   # s maximos frente a una foto para confirmarla (2 fotos HD en la Pi)
    exigir_bandera: bool = False    # (carrera.py lo activa) las fotos solo cuentan si hay bandera confirmada y estan a < r_meta de ella
    r_meta: float = 2.2             # m: (21/11: tomaba caras de gente del fondo del lab y fotos sueltas por meta)
    t_sin_bandera: float = 100.0    # s de carrera sin ver la bandera: desde ahi cualquier cara cuenta (respaldo)
    r_salida: float = 1.0           # m: una bandera a menos de esto de A es la de la salida, no la meta (22/11:
                                    # 1 m y no mas, por si el circuito es una U con la meta cerca de la salida)
    z_max_cara: float = 0.5         # m: una cara mas alta que esto no es de una foto de la meta (es una persona)
    mirar_meta_s: float = 1.2       # s quieto en cada rumbo del barrido, minimo; sigue quieto hasta recibir 3 fotos
    mirar_meta_max: float = 2.5     # (2 tomadas ya parado: la Pi entrega cada foto ~0.5 s despues) o hasta este max
    w_meta: float = 0.4             # rad/s maximos al girar entre rumbos (21/11: lento, que las fotos salgan nitidas) (21/11: a 1.6 las fotos salian corridas y
                                    # girando a > 1 rad/s no se reconoce)


# arcos que prueba la ventana dinamica: 6 velocidades de 0 a vmax y 19 giros, mantenidos 1.2 s
T_ARCO = np.arange(1, 13) * 0.1
RADIO_FISICO = 0.175            # m: radio del Create 3 (0.17) + 5 mm; ningun arco pasa mas cerca que esto
FRENO = 0.30                    # m: el puente frena con algo a < 0.28 m en el pasillo del robot (+-0.20 m)
FRENO_SOLTAR = 0.40             # m: ...y no suelta hasta tener 0.38 m libres (histeresis)
PUENTE_PARA, PUENTE_SUELTA = 0.28, 0.38   # m: los umbrales del freno del puente (tb4.protocol.SafetyConfig)
REVERSA_V, REVERSA_S = -0.12, 1.2         # retroceso para salir de un atasco con el frente bloqueado


_ARCOS = {}


def arcos(vmax, wmax):
    """-> v, w (K,) y la trayectoria x, y, th (K, pasos) de cada arco, en el marco del robot (float32, y
    tambien cos/sin de th), en cache por (vmax, wmax): se llama 20 veces por segundo con los mismos valores."""
    k = (round(vmax, 4), round(wmax, 4))
    a = _ARCOS.get(k)
    if a is None:
        V, W = np.meshgrid(np.linspace(0.0, vmax, 6), np.linspace(-wmax, wmax, 19), indexing="ij")
        v, w = V.ravel(), W.ravel()
        th = w[:, None] * T_ARCO[None, :]
        recto = np.abs(w[:, None]) < 1e-6
        wn = np.where(recto, 1.0, w[:, None])
        x = np.where(recto, v[:, None] * T_ARCO[None, :], v[:, None] / wn * np.sin(th))
        y = np.where(recto, 0.0, v[:, None] / wn * (1 - np.cos(th)))
        a = tuple(np.ascontiguousarray(z, dtype=np.float32) for z in (v, w, x, y, th, np.cos(th), np.sin(th)))
        if len(_ARCOS) > 32:
            _ARCOS.clear()
        _ARCOS[k] = a
    return a


def _reducir_puntos(px, py, max_m=1.0, paso_deg=1.5):
    """Los puntos del LiDAR a menos de `max_m`, quedandose con el MAS CERCANO de cada sector de `paso_deg`:
    de 720 puntos quedan <= 240, y el que se descarta esta mas lejos que el que se conserva en casi la misma
    direccion (el margen de `radio` sobre el radio fisico cubre la diferencia). Es lo que hace que la
    ventana dinamica cueste 3 ms y no 12 en una PC (en la Raspberry, x4)."""
    d = np.hypot(px, py)
    cerca = d < max_m
    if not cerca.any():
        return px[:0].astype(np.float32), py[:0].astype(np.float32)
    px, py, d = px[cerca], py[cerca], d[cerca]
    sector = np.floor((np.arctan2(py, px) + np.pi) / math.radians(paso_deg)).astype(np.int32)
    orden = np.lexsort((d, sector))                                 # por sector y, dentro, por distancia
    primero = np.ones(len(orden), bool)
    primero[1:] = sector[orden][1:] != sector[orden][:-1]
    sel = orden[primero]
    return px[sel].astype(np.float32), py[sel].astype(np.float32)


def ventana_dinamica(px, py, h_rel, vmax, wmax, radio=0.20, bloqueado=False):
    """Elige (v, w) entre los arcos de 1.2 s que no pasan a menos de `radio` de ningun punto del LiDAR
    (px, py en el marco ACTUAL del robot): el que mas avanza hacia `h_rel` (rad, marco del robot), con
    buena orientacion al final y lejos de lo ocupado. None si ninguno es seguro.
    Si el robot ya esta a menos de `radio` de algo (paso angosto), valen los arcos que no lo acercan
    mas, sin bajar nunca de RADIO_FISICO: asi puede salir en vez de quedarse girando.
    `bloqueado` = el freno del puente esta puesto: no lo suelta hasta tener PUENTE_SUELTA libres, asi que
    un arco que avance antes de eso no sirve (el puente lo deja en v = 0 y el robot se queda quieto)."""
    v, w, x, y, th, c, s = arcos(vmax, wmax)
    d0 = float(np.hypot(px, py).min()) if px.size else 9.0
    if d0 < radio:
        radio = max(RADIO_FISICO, d0 - 0.005)
    px, py = _reducir_puntos(px, py)
    d_fin = np.full(len(v), 9.0)
    if px.size:
        dx, dy = px[None, None, :] - x[..., None], py[None, None, :] - y[..., None]      # (K, pasos, M) float32
        dd2 = dx * dx
        dd2 += dy * dy                                                                   # distancias al cuadrado
        dmin, d_fin = np.sqrt(dd2.min(axis=(1, 2))), np.sqrt(dd2[:, -1, :].min(axis=1))
        # la regla del puente en cada paso del arco: nada en su pasillo (+-0.20 m) a menos de FRENO m por
        # delante, o frena en seco (y para soltar pide 0.38 m: si ya esta cerca, al empezar se pide eso)
        delante = c[..., None] * dx + s[..., None] * dy
        lado = c[..., None] * dy
        lado -= s[..., None] * dx
        en_pasillo = (delante > 0) & (np.abs(lado) < 0.20)
        frena = (en_pasillo & (delante < FRENO)).any(axis=(1, 2))
        if d0 < 0.25 or bloqueado:
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
        self.bloq_frente = False              # copia del estado del freno del puente (misma histeresis)
        self.libre_frente = self.libre_atras = 9.0   # m libres en el pasillo del robot (+-0.20 m)
        self._reversa = -1e9                  # hasta cuando retroceder (salir de un atasco de frente)
        self.mirada = None                    # mirada.MiradaLateral (mirar paredes de costado en la IDA); None = no
        self._vot0 = votacion                 # plantilla de votacion (con --buscar cada candidata tiene la suya)
        self.candidatas = []                  # con --buscar: una Objetivo por cada foto vista
        self.palabra = None                   # lo que dice el QR de la persona buscada
        self.qr_de = {}                       # id de candidata -> texto del QR leido junto a esa foto
        self._qr_votos = {}                   # texto -> fotos que lo vieron junto a una cara reconocida como la buscada
        self._qr_otros = {}                   # texto -> persona junto a la que se vio (otra que la buscada)
        self._qr = None                       # fase de LEER_QR: dict(paso, t0, th0, t_ini)
        self._t_ahora = 0.0                   # el `now` del ultimo paso() (para quiere_hd)
        self._fila_completa = False           # se recorrio entera la fila (o el tubo): vale decidir por ranking
        self.qr_vista = None                  # recorte de la foto con el QR aceptado (lo pone carrera.py, para mostrar)
        self._mirar = []                      # rumbos (mapa) a los que girar en el sitio, en orden (barrido)
        self._mapa = None                     # el ultimo mapa recibido (LEER_QR lo usa para ponerse de frente)
        self.t_acercar = -1e9                 # cuando se eligio (por candidata) la foto a la que se va
        self._n_usadas, self._xy_giro = 0, None   # flechas ya usadas y donde se uso la ultima (barrido lateral)
        self._frentes = []                    # (x, y) donde ya se miro la pared de enfrente antes de doblar
        self._mirar_quieto = 0.0              # s quieto al llegar al primer rumbo de _mirar (vistazo a una pared)
        self._barridos = []                   # (x, y) donde ya se hizo un barrido (no repetir en el mismo lugar)
        self._fila = None                     # FILA: dict(puntos=[(x, y, rumbo, x_pared, y_pared, lado)], t_punto,
        #                                       mirando_desde); lado = +-1 si el punto es "a ver si la fila sigue"
        self._fila_vistos = []                # puntos de la PARED ya mirados de frente en FILA (no repetir)
        self._fila_vacios = []                # puntos de la pared donde la fila resulto no seguir (no extender)
        self._now, self._fotos = 0.0, 0
        self.bandera = None                   # META: dict(xy, n, t, pos=[...]) de la bandera a cuadros del fondo
        self._meta = None                     # META: dict(punto, rumbos, fase, t_punto, mirando_desde, t_ini)
        self._barrido_meta = False            # ya se hizo el barrido desde el centro: vale decidir por ranking
        self.descartes = 0                    # fotos miradas de cerca que resultaron otra persona
        self.marcas = {}                      # tiempos (s de carrera) de hitos, para el registro: bandera, barrido...

    # ---------- control externo ----------
    def empezar(self, now, yaw):
        if self.estado == PAUSA:
            if self.t_inicio is None:
                self.t_inicio = now
                self.h_pref = yaw
                self.t_nuevo = now
                if self.mirada is not None:
                    self.mirada.t_inicio = now
            self.estado = self._antes_pausa

    def pausar(self):
        if self.estado != PAUSA:
            self._antes_pausa = self.estado
            self.estado = PAUSA

    def nota(self, now, msg):
        t = 0.0 if self.t_inicio is None else now - self.t_inicio
        self.log.append(f"[{t:6.1f} s] {msg}")

    def marca(self, now, que):
        """Hito con su tiempo de carrera (una sola vez por nombre): queda en el registro de la corrida."""
        if que not in self.marcas:
            self.marcas[que] = round(0.0 if self.t_inicio is None else now - self.t_inicio, 1)

    def _resumen_candidatas(self):
        """Una linea por foto vista: id, donde, p de ser la buscada, a quien se parece mas, votos, tamano, estado."""
        out = []
        for c in self.candidatas:
            if c.xy is None:
                continue
            nombre, p = c.vot.lider() if c.vot.n else ("-", 0.0)
            out.append(f"#{c.id} ({c.xy[0]:.2f},{c.xy[1]:.2f}) p_buscada={self._p_buscada(c):.2f} "
                       f"lider={nombre} {p:.2f} votos={c.vot.n} max={c.ancho_max:.0f}px {c.estado}"
                       + (" descartada" if c.descartada else "") + (" ranking" if c.por_ranking else ""))
        return out

    # ---------- percepcion ----------
    def ver_cara(self, now, o, peso):
        """Llamar con cada cara detectada. Devuelve True si se uso."""
        if self.cfg.buscar:
            return self._ver_cara_buscar(now, o, peso)
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
            if self.mirada is not None:
                self.mirada.activa = None                       # deja de mirar al costado: ya la encontro
            self.vista_b = np.array(self._pos[:2], float)       # desde aqui se ve la foto
            self._mejor_b, self.por_mapa = None, False
        return True

    # ---------- varias fotos: candidatas, la buscada y su QR ----------
    @property
    def quiere_hd(self):
        """Pedir fotos HD para el QR: solo frente a la foto y si la preview no alcanzo en qr_preview_s (en la
        Raspberry cada foto HD tarda ~4 s: pedirla al acercarse, como antes, frenaba todo)."""
        if self.confirmando_hd:
            return True
        f = self._qr
        espera = 0.0 if self.cfg.confirmar_hd else self.cfg.qr_preview_s     # (meta: la preview no alcanza, HD ya)
        return bool(self.quiere_qr and self.estado == LEER_QR and f is not None and f["paso"] >= 0
                    and self._t_ahora - f.get("t_frente", f["t_ini"]) > espera)

    @property
    def confirmando_hd(self):
        """Frente a una foto que hay que confirmar y en la preview la cara salio chica (< px_hd_confirmar): pedir
        fotos HD (21/11: en la preview las fotos del lab miden 14-25 px y el reconocimiento no distingue a nadie;
        en la HD la misma cara mide ~3 veces mas). Cada HD tarda ~4 s en la Pi: una o dos por foto."""
        return bool(self.cfg.buscar and self.cfg.confirmar_hd and self.estado == RECONOCER and self.obj is not None
                    and self.obj.ancho_max < self.cfg.px_hd_confirmar)

    @property
    def quiere_qr(self):
        """La camara debe buscar QR (cuesta: solo cerca de la meta)."""
        return bool(self.cfg.buscar and self.cfg.leer_qr and self.palabra is None
                    and self.estado in (ACERCAR, RECONOCER, LEER_QR))

    def _p_buscada(self, c):
        """Probabilidad (votacion de la candidata) de que sea la persona buscada."""
        if c.vot.n == 0:
            return 1.0 / max(len(c.vot.nombres), 1)
        p = c.vot.probabilidades()
        return float(p[c.vot.nombres.index(self.cfg.buscar)])

    def _candidata(self, xy, radio=None):
        """La candidata mas cercana a xy (a menos de `radio`), o None."""
        radio = self.cfg.d_candidata if radio is None else radio
        mejor, d_mejor = None, radio
        for c in self.candidatas:
            if c.xy is None:
                continue
            d = float(np.hypot(c.xy[0] - xy[0], c.xy[1] - xy[1]))
            if d < d_mejor:
                mejor, d_mejor = c, d
        return mejor

    def _candidata_para(self, o):
        """La candidata (foto) a la que pertenece la cara `o`: la mas cercana a menos de d_candidata, salvo que
        esa foto ya sea claramente de OTRA persona que la que esta cara muestra con claridad (fotos vecinas
        casi pegadas, o el odometro corrido, las juntarian). None = es una foto nueva."""
        r = o.res
        clara = r is not None and o.px >= self.cfg.px_descartar and r.margen >= self.cfg.margen_qr
        cerca = []
        x, y = self._pos[:2]
        a_o = math.atan2(o.xy[1] - y, o.xy[0] - x)
        # el rumbo solo sirve si se ve la pared de frente (de refilon dos fotos vecinas quedan en casi el mismo rumbo)
        n = self._normal_pared(o.xy)
        hacia = np.array([x - o.xy[0], y - o.xy[1]], float)
        de_frente = float(n @ hacia) / max(float(np.linalg.norm(hacia)), 1e-6) > math.cos(math.radians(40))
        con_bandera = self.bandera is not None and self.bandera["n"] >= 2       # (solo en la meta del 21/11)
        for c in self.candidatas:
            if c.xy is not None:
                d = float(np.hypot(c.xy[0] - o.xy[0], c.xy[1] - o.xy[1]))
                # misma direccion vista desde el robot (< 5 grados) y no muy lejos: es la misma foto aunque la
                # distancia (LiDAR o tamano) haya salido distinta (21/11: una foto se partia en 2-3 candidatas y el
                # robot oscilaba entre ellas)
                da = abs(wrap(math.atan2(c.xy[1] - y, c.xy[0] - x) - a_o))
                if d < self.cfg.d_candidata or (con_bandera and de_frente and da < math.radians(5) and d < 0.5):
                    cerca.append((d, c))
        for d, c in sorted(cerca, key=lambda t: t[0]):
            if clara and c.vot.n >= 3:
                nombre, p = c.vot.lider()
                if p >= 0.9 and nombre != r.nombre:
                    continue                                    # esa foto es de otra persona: no mezclar
            return c
        return None

    def _clasificar(self, c):
        """pendiente | objetivo (es la buscada) | otra. Se recalcula con cada foto: puede cambiar."""
        v = c.vot
        if c.descartada and not (v.lider()[0] == self.cfg.buscar and v.lider()[1] >= v.prob_min):
            c.estado = "otra"
            return
        if c.por_ranking and (v.n < 4 or v.lider()[0] == self.cfg.buscar):
            return                                              # elegida por ranking: sigue siendolo mientras la
                                                                # buscada sea la mas parecida
        temprana = v.n >= v.min_fotos and c.ancho_max >= self.cfg.px_temprana
        if v.n < v.min_fotos or (v.peso < 1.0 and not temprana):
            c.estado = "pendiente"
            return
        nombre, p = v.lider()
        if nombre == self.cfg.buscar and p >= v.prob_min:
            c.estado = "objetivo"
        elif (nombre == self.cfg.buscar and p >= self.cfg.p_temprana and c.ancho_max >= self.cfg.px_temprana
              and not any(k.estado == "objetivo" for k in self.candidatas if k is not c)):
            c.estado = "objetivo"                               # clara y sin rival: no esperar mas fotos
        elif c.ancho_max >= self.cfg.px_descartar and (
                (nombre != self.cfg.buscar and p >= v.prob_min) or (v.n >= 4 and self._p_buscada(c) < 0.01)):
            c.estado = "otra"                                   # solo de cerca: de lejos podria ser la buscada
            if c.por_ranking:                                   # la elegida por ranking resulto otra: no volver
                c.descartada, c.por_ranking = True, False
                self.descartes += 1
        else:
            c.estado = "pendiente"

    def _en_meta(self, xy):
        """La foto en xy puede ser de la meta: hay bandera confirmada y esta a < r_meta de ella (o no se exige
        bandera, o paso t_sin_bandera sin verla)."""
        if not self.cfg.exigir_bandera:
            return True
        b = self.bandera
        if b is None or b["n"] < 2:
            t = 0.0 if self.t_inicio is None else self._t_ahora - self.t_inicio
            return t >= self.cfg.t_sin_bandera
        return math.hypot(xy[0] - b["xy"][0], xy[1] - b["xy"][1]) < self.cfg.r_meta

    def _ver_cara_buscar(self, now, o, peso):
        if self.estado in (PAUSA, VOLVER, HECHO):
            return False
        if not self._en_meta(o.xy):
            return False                                        # sin bandera (o lejos de ella) una cara no es la meta
        if self.cfg.exigir_bandera and getattr(o, "z", 0.0) > self.cfg.z_max_cara:
            return False                                        # muy alta: una persona, no una foto pegada abajo
        c = self._candidata_para(o)
        if c is None:
            v0 = self._vot0
            c = type(self.obj)(type(v0)(v0.nombres, temperatura=v0.T, min_fotos=v0.min_fotos,
                                        max_fotos=v0.max_fotos, prob=v0.prob_min))
            c.id = len(self.candidatas) + 1
            self.candidatas.append(c)
        # de costado la cara se reconoce mal (y una foto de la buscada vista de refilon se descartaba como
        # "otra"): el voto pesa cos^2 del angulo con que se ve la pared, y de mas de 55 grados no vota
        n = self._normal_pared(o.xy)
        hacia = np.array([self._pos[0] - o.xy[0], self._pos[1] - o.xy[1]], float)
        cos_inc = float(n @ hacia) / max(float(np.linalg.norm(hacia)), 1e-6)
        if cos_inc < math.cos(math.radians(65)):               # (65: desde el centro del tubo las fotos de los
            o = dc_replace(o, res=None)                         # costados se ven a 50-60 grados; con 55 no votaban)
        else:
            peso *= cos_inc ** 2
        antes = c.estado
        c.agregar(o, peso, self._pos)
        self._clasificar(c)
        if c.estado != antes and c.estado != "pendiente":
            nombre, p = c.vot.lider()
            self.nota(now, f"foto #{c.id}: {'ES ' + self.cfg.buscar if c.estado == 'objetivo' else 'es ' + nombre} "
                           f"(p={p:.2f}, {c.vot.n} fotos, {c.ancho_max:.0f} px)")
        self._elegir_candidata(now)
        return True

    def _elegir_candidata(self, now):
        """Elige a que foto ir: la buscada si ya se sabe cual es; si no, la pendiente que mas se le parece."""
        if self.estado in (PAUSA, VOLVER, HECHO):
            return
        actual = self.obj if self.obj in self.candidatas else None
        if self.estado == LEER_QR and actual is not None and actual.estado != "otra":
            return                                                 # leyendo el QR: no cambiar de foto ni de plan
        # una candidata que no se encontro donde se la vio se deja de lado un rato (salvo que se la vuelva a
        # ver): si no, ACERCAR la elige, no la ve y vuelve a IDA en cada foto (bucle)
        def vale(c):
            return now > c.t_ignorar or c.visto_hace(now) < 1.0
        objs = [c for c in self.candidatas if c.estado == "objetivo" and vale(c)]
        if self.estado == META and self._meta is not None and self._meta["fase"] == "girar":
            objs = [c for c in objs if c.vot.lider()[0] == self.cfg.buscar and c.vot.lider()[1] >= self.cfg.p_temprana]
            if not objs:
                return                                             # barriendo: nada lo interrumpe salvo la buscada segura
        if not objs:
            r = self._ranking(now)
            if r is not None:
                objs = [r]
        pend = [c for c in self.candidatas if c.estado == "pendiente" and c.n >= 2 and vale(c)]
        if objs:
            mejor = max(objs, key=self._p_buscada)
        elif pend and self.estado in (FILA, META):
            return                                                 # recorriendo la fila / barriendo la meta: solo la
                                                                   # interrumpe la buscada
        elif pend and self._empezar_fila(now):
            return                                                 # mirar cada foto DE FRENTE (de costado se reconoce mal)
        elif pend:
            x, y = self._pos[:2]
            mejor = max(pend, key=lambda c: self._p_buscada(c) - 0.05 * float(np.hypot(c.xy[0] - x, c.xy[1] - y)))
            if actual in pend and actual is not mejor:
                if self._p_buscada(mejor) < self._p_buscada(actual) + 0.3:
                    mejor = actual                                 # no saltar de una a otra por poco
        else:
            mejor = None
        if self.estado == LEER_QR and actual is not None and actual.estado != "otra":
            return                                                 # leyendo el QR: no cambiar de foto
        if mejor is None:
            if self.estado in (FILA, META):
                return                                             # sigue recorriendo la fila / el barrido
            if self._en_la_meta_sin_salir(now):
                return
            if self._empezar_fila(now):
                return
            hay_otra = any(c.estado == "otra" for c in self.candidatas)
            if hay_otra and (self.estado in BUSCANDO or (actual is not None and actual.estado == "otra")):
                venia = self.estado not in BUSCANDO
                if venia:
                    self.estado = IDA
                    self.t_nuevo = now
                    self.h_pref = self._pos[2]
                x, y, th = self._pos
                if all(math.hypot(x - bx, y - by) > 0.6 for bx, by in self._barridos):
                    # las fotos de la meta estan juntas (en fila): mirar 55 grados a cada lado antes de irse
                    self._barridos.append((x, y))
                    self._mirar = [wrap(th + math.radians(55)), wrap(th - math.radians(55)), th]
                    self.nota(now, "no veo otra foto por revisar: miro a los costados")
                elif venia:
                    self.nota(now, "no queda ninguna foto por revisar: sigo buscando")
            return
        if mejor is not actual or self.estado in BUSCANDO:
            if self.estado in BUSCANDO or mejor is not actual:
                nombre, p = mejor.vot.lider() if mejor.vot.n else ("-", 0.0)
                self.nota(now, f"voy a la foto #{mejor.id} (p de ser {self.cfg.buscar} = {self._p_buscada(mejor):.2f}; "
                               f"lider {nombre} {p:.2f}, {mejor.vot.n} votos, {mejor.ancho_max:.0f} px"
                               f"{', por ranking' if mejor.por_ranking else ''}): ACERCAR")
            self.obj, self.vot = mejor, mejor.vot
            self.estado = ACERCAR
            self.t_acercar = now                    # una candidata vista hace rato no esta "perdida" recien elegida
            self._mirar = []
            self._fila = None
            if self.mirada is not None:
                self.mirada.activa = None
            self.vista_b = np.array(self._pos[:2], float)
            self._mejor_b, self.por_mapa, self.px_extra = None, False, 1.0
            n = self._normal_pared(mejor.xy)                       # vista de refilon (pared de un costado del tubo):
            hacia = self.vista_b - np.asarray(mejor.xy, float)     # ir derecho a la foto choca con la pared; ir por
            dh = float(np.linalg.norm(hacia))                      # el mapa al punto de enfrente
            if dh > 1e-6 and float(n @ hacia) / dh < math.cos(math.radians(45)):
                self.por_mapa = True
                self._reset_ruta()

    def _ranking(self, now):
        """Si ya se vieron todas las fotos (las `n_fotos` si se sabe cuantas son, se hizo el barrido de la meta
        desde el centro del tubo, o se recorrio la fila / el tubo entero), la buscada es la que MAS se le parece
        (de las que no se descartaron de cerca): no hace falta estar seguro de cada una, se confirma de cerca y
        si no es, se va a la siguiente (el 19/11 Thiago tenia 0.96 a los 15 s y se confirmo recien a los 52 s).
        Con rank_p / rank_ventaja > 0 se exige ademas ese minimo y esa ventaja sobre la segunda."""
        if self.estado in (PAUSA, VOLVER, HECHO, LEER_QR, META):
            return None                                          # (en META: primero la vuelta entera)
        vistas = [c for c in self.candidatas if c.xy is not None and c.n >= 2 and c.vot.n >= 1 and self._en_meta(c.xy)
                  and not c.descartada and c.estado != "otra" and (now > c.t_ignorar or c.visto_hace(now) < 1.0)]
        if not vistas:
            return None
        n_vistas = sum(1 for c in self.candidatas if c.xy is not None and c.n >= 2 and c.vot.n >= 1
                       and self._en_meta(c.xy))
        completo = ((self.cfg.n_fotos > 0 and n_vistas >= self.cfg.n_fotos) or self._fila_completa
                    or self._barrido_meta)
        if not completo:
            return None
        # las ya miradas de cerca sin saber quien es (dudosas) van despues de las que faltan revisar
        orden = sorted(vistas, key=lambda c: (c.revisiones, -self._p_buscada(c)))
        todas = self.cfg.n_fotos and n_vistas >= self.cfg.n_fotos
        if orden[0].revisiones >= 1 and not todas and not self._fila_completa:
            return None                                          # solo quedan dudosas y faltan fotos por ver: buscarlas
        if orden[0].revisiones >= 2 and not (todas and len(vistas) == 1):
            return None                                          # ya revisada dos veces sin saber: no oscilar entre
                                                                 # dudosas (21/11); seguir buscando
        p1 = self._p_buscada(orden[0])
        mismas = [c for c in orden[1:] if c.revisiones == orden[0].revisiones]
        p2 = self._p_buscada(mismas[0]) if mismas else 0.0
        if p1 < self.cfg.rank_p or p1 < self.cfg.rank_ventaja * p2:
            return None
        c = orden[0]
        if not (c.estado == "objetivo" and c.por_ranking):
            por = ("por descarte: las otras no son" if (self.cfg.n_fotos and len(vistas) == 1
                                                        and n_vistas >= self.cfg.n_fotos)
                   else ("la menos descartable (ya revisadas todas)" if c.revisiones else "la mas parecida"))
            self.nota(now, f"ranking ({n_vistas} fotos vistas, {len(vistas)} por revisar): {por} a {self.cfg.buscar} "
                           f"es la #{c.id} (p={p1:.2f}, la segunda {p2:.2f})")
            for linea in self._resumen_candidatas():
                self.nota(now, "    " + linea)
            self.marca(now, "ranking")
        c.estado, c.por_ranking = "objetivo", True
        return c

    def _quizas_barrer(self, now, x, y, th, cam_ok):
        """Pasadas las senales y sin ninguna foto vista, cada ~0.9 m mira hacia las paredes cercanas: en un
        pasillo la camara (40 grados) apunta a lo largo y una foto en la pared de al lado no entra nunca."""
        if (not cam_ok or self.armado is not None or any(c.estado != "otra" for c in self.candidatas)
                or self.sen.pendientes()):
            return
        n_usadas = sum(1 for s in self.sen.lista if s.usada)
        if n_usadas != self._n_usadas:                          # acaba de usar una flecha: el giro va primero
            self._n_usadas, self._xy_giro = n_usadas, (x, y)
            return
        t = 0.0 if self.t_inicio is None else now - self.t_inicio
        if (n_usadas == 0 and t < 45.0) or math.hypot(x - self.home[0], y - self.home[1]) < 1.5:
            return                                              # al principio (sin senales aun) cuesta mas de lo que da
        if now < self.giro_hasta or (self._xy_giro is not None
                                     and math.hypot(x - self._xy_giro[0], y - self._xy_giro[1]) < 1.5):
            return                                              # recien doble: puede venir otra flecha
        if any(math.hypot(x - bx, y - by) < 0.9 for bx, by in self._barridos):
            return
        if len(self._barridos) >= 1 + t / 25.0:
            return                                              # a lo mas uno cada 25 s: en un laberinto hay paredes por todos lados
        lados = []
        for sgn in (1, -1):
            m = (np.abs(self.px) < 0.35) & (sgn * self.py > 0.1)
            if m.any() and float((sgn * self.py[m]).min()) < 1.4:
                lados.append(sgn)
        if lados:
            self._barridos.append((x, y))
            self._mirar = [wrap(th + sgn * math.radians(75)) for sgn in lados] + [th]
            self.nota(now, "sin fotos a la vista: miro a las paredes de los costados")

    def _vista_frente(self, pared, n, dists=(0.75, 0.65, 0.55, 0.45)):
        """Punto frente a `pared` (sobre la normal n) a la mayor de `dists` que deje >= 0.25 m libres segun el
        ultimo scan (en un pasillo angosto, 0.75 m quedaria pegado a la pared de enfrente)."""
        sx, sy = self._pts_scan
        if sx.size:
            bx, by, bth = self.pos_scan
            gx = bx + math.cos(bth) * sx - math.sin(bth) * sy
            gy = by + math.sin(bth) * sx + math.cos(bth) * sy
        for d in dists:
            q = np.asarray(pared, float) + d * n
            if not sx.size or float(np.min(np.hypot(gx - q[0], gy - q[1]))) >= 0.25:
                return q
        return None                                                 # ahi no cabe el robot (caja, rincon)

    def _rango_scan(self, q, ventana_deg=1.5):
        """(distancia a q, lo que mide el LiDAR en esa direccion) desde donde se tomo el ultimo scan: inf si en
        esa direccion no devolvio nada, None si no hay scan."""
        bx, by, bth = self.pos_scan
        dx, dy = q[0] - bx, q[1] - by
        rq = math.hypot(dx, dy)
        sx, sy = self._pts_scan
        if not sx.size:
            return rq, None                                      # sin scan: no se sabe
        bq = wrap(math.atan2(dy, dx) - bth)
        m = np.abs(wrap_arr(np.arctan2(sy, sx) - bq)) < math.radians(ventana_deg)
        return rq, (float(np.hypot(sx[m], sy[m]).min()) if m.any() else math.inf)

    def _pared_enfrente(self, centro, n, d_min=0.5, d_max=1.8):
        """Distancia (m) de la pared paralela de enfrente: puntos del LiDAR a d_min-d_max de `centro` sobre la
        normal n (y a < 0.4 m de esa recta). None si no hay (no es un pasillo/tubo)."""
        sx, sy = self._pts_scan
        if sx.size < 5:
            return None
        bx, by, bth = self.pos_scan
        gx = bx + math.cos(bth) * sx - math.sin(bth) * sy
        gy = by + math.sin(bth) * sx + math.cos(bth) * sy
        rel = np.stack([gx - centro[0], gy - centro[1]], 1)
        a = rel @ n
        lat = rel @ np.array([-n[1], n[0]])
        m = (a > d_min) & (a < d_max) & (np.abs(lat) < 0.4)
        if m.sum() < 5:
            return None
        return float(np.median(a[m]))

    def _empezar_fila(self, now):
        """Apenas se ve una foto se asume que se esta en la zona de la meta: las demas fotos estan cerca, en fila
        sobre la misma pared, y si es un pasillo sin salida ("el tubo", 19/11: las fotos van en los DOS costados,
        nunca al fondo) tambien en la pared de enfrente. Puntos de vista DE FRENTE a cada pared (a ~0.55-0.75 m):
        uno frente a cada foto ya vista y, hacia cada lado, uno cada 0.45 m (lo que separa dos fotos) hasta 2.7 m
        mas alla de la ultima o hasta donde no cabe el robot. En el tubo se recorre por estaciones: en cada una se
        mira una pared y despues la otra (solo hay que girar). De costado la cara se reconoce mal y el QR no se
        lee. False si no hay fotos o ya se visitaron todos los puntos."""
        fotos = [c for c in self.candidatas if c.xy is not None and c.n >= 2]
        if not fotos:
            return False
        x, y = self._pos[:2]
        normales = [self._normal_pared(c.xy) for c in fotos]
        ref = int(np.argmin([math.hypot(c.xy[0] - x, c.xy[1] - y) for c in fotos]))
        n_ref = normales[ref]
        filas = []                                                 # (centro, n, [t de fotos conocidas], codigo)
        for signo, cod in ((1, 1), (-1, 2)):
            idx = [i for i, n in enumerate(normales) if signo * float(n @ n_ref) > 0.7]
            if not idx:
                continue
            P = np.array([fotos[i].xy for i in idx], float)
            nn = np.mean([normales[i] for i in idx], axis=0)
            nn /= max(float(np.linalg.norm(nn)), 1e-6)
            filas.append([P.mean(0), nn, P, cod])
        replanear = False
        if len(filas) == 1:                                        # tubo: fotos solo vistas en una pared
            c0, n0, P0, _ = filas[0]
            a_robot = float((np.array([x, y]) - c0) @ n0)
            D = self._pared_enfrente(c0, n0)
            if D is not None and 0.2 < a_robot < D - 0.2:          # el robot esta entre las dos paredes:
                filas.append([c0 + D * n0, -n0, np.zeros((0, 2)), 2])   # la de enfrente tambien puede tener fotos
            elif D is not None or a_robot <= 0.2 or a_robot > 1.8:
                replanear = True                                   # desde afuera la pared de enfrente se mide mal:
                                                                   # replanear desde el primer punto
        u0 = np.array([-n_ref[1], n_ref[0]])                       # eje comun (a lo largo de las paredes)
        t_r = float(np.array([x, y]) @ u0)
        todos = []                                                 # (t sobre u0, punto)
        for centro, n, P, cod in filas:
            u = np.array([-n[1], n[0]])
            if float(u @ u0) < 0:
                u = -u                                             # misma orientacion en las dos paredes
            rumbo = math.atan2(-n[1], -n[0])
            t = (P - centro) @ u if len(P) else np.zeros(0)

            def punto(ti, lado, centro=centro, n=n, u=u, rumbo=rumbo):
                pared = centro + ti * u
                if any(math.hypot(pared[0] - a, pared[1] - b) < 0.25 for a, b in self._fila_vistos):
                    return "visto"                                 # ese tramo de pared ya se miro de frente
                rp, rs = self._rango_scan(pared)
                bx, by, _ = self.pos_scan
                de_frente = abs((bx - pared[0]) * n[0] + (by - pared[1]) * n[1]) > 0.5 * rp   # < 60 grados
                if rs is not None and rp < 4.0 and rs > rp + 0.35 and de_frente:
                    return None                                    # el LiDAR ve mas alla: ahi no hay pared (abertura)
                q = self._vista_frente(pared, n, dists=self.cfg.d_vista_fila)
                if q is None:
                    return None                                    # ahi no se puede estar (se acabo la pared)
                rq, rs = self._rango_scan(q)
                if rs is not None and rq < 4.0 and rs < rq - 0.15:
                    return None                                    # detras de una pared (pasado el fondo del tubo)
                return (float(q[0]), float(q[1]), rumbo, float(pared[0]), float(pared[1]), lado)

            lo = float(t.min()) if len(t) else 0.0
            hi = float(t.max()) if len(t) else 0.0
            for ti in sorted(set(round(float(v), 2) for v in t)):
                q = punto(ti, 0)
                if q not in (None, "visto"):
                    todos.append((float((centro + ti * u) @ u0), q))
            for signo in (1, -1):
                borde = hi if signo == 1 else lo
                for k in range(1, 7):
                    ti = borde + signo * 0.45 * k
                    w = centro + ti * u
                    if any(math.hypot(w[0] - a, w[1] - b) < 0.3 for a, b in self._fila_vacios):
                        break                                      # ya se sabe que por ahi la fila no sigue
                    q = punto(ti, signo * cod)
                    if q is None:
                        break                                      # se acabo la pared: la fila no sigue por ahi
                    if q != "visto":
                        todos.append((float(w @ u0), q))
            if not len(t):                                         # pared de enfrente sin fotos vistas: un punto
                q = punto(0.0, 0)                                  # frente al centro tambien
                if q not in (None, "visto"):
                    todos.append((float(centro @ u0), q))
        if not todos:
            return False
        # orden: estaciones a lo largo del eje empezando por el extremo del lado del robot; en cada estacion
        # (puntos a < 0.25 m sobre el eje) primero la pared mas cercana al robot
        ts = [t for t, _ in todos]
        cerca = -1 if abs(t_r - min(ts)) < abs(t_r - max(ts)) else 1
        # una sola pasada por estaciones desde el extremo del robot (ir y volver por el tubo cuesta mas que mirar
        # un punto de mas): las fotos ya vistas y los puntos "a ver si hay mas" en el orden en que se pasa por ellos
        todos.sort(key=lambda tq: (round(-cerca * tq[0] / 0.25), math.hypot(tq[1][0] - x, tq[1][1] - y)))
        # zigzag: en cada estacion empezar por la pared que se estaba mirando al final de la anterior (ahorra
        # medio giro por estacion: ~1.7 s)
        orden, i = [], 0
        while i < len(todos):
            j = i + 1
            while j < len(todos) and abs(todos[j][0] - todos[i][0]) < 0.25:
                j += 1
            grupo = todos[i:j]
            if orden and len(grupo) > 1:
                previo = orden[-1][1][2]
                grupo.sort(key=lambda tq: abs(wrap(tq[1][2] - previo)))
            orden += grupo
            i = j
        todos = orden
        puntos = [q for _, q in todos]
        if self.cfg.n_fotos and len(fotos) >= self.cfg.n_fotos:
            puntos = [q for q in puntos if q[5] == 0]              # ya se vieron todas: no buscar mas
        if not puntos:
            return False
        self._fila = dict(puntos=puntos, t_punto=now, mirando_desde=None, completa=True, vacios={}, replanear=replanear,
                          tubo=len(filas) > 1)
        self.estado = FILA
        self._mirar = []
        self._reset_ruta()
        donde = "en las dos paredes" if len(filas) > 1 else "en la pared"
        self.nota(now, f"ninguna foto vista es {self.cfg.buscar}: recorro la fila {donde} ({len(puntos)} puntos de vista)")
        return True

    def _recorrer_fila(self, now, pos, mapa):
        f = self._fila
        if not f or not f["puntos"]:
            if f and f.get("completa") and not f.get("incompleta"):
                self._fila_completa = True                         # se miraron (de verdad) todas las paredes
            self._fila = None
            self.estado = IDA
            self.t_nuevo = now
            self._elegir_candidata(now)                            # otra foto pendiente, mas fila, o seguir buscando
            if self.estado == IDA:
                self.nota(now, "recorri la fila sin encontrar a la buscada: sigo buscando")
            return 0.0, 0.0
        x, y, th = pos
        px, py, rumbo, wx, wy, lado = f["puntos"][0]
        if wx is not None and f["mirando_desde"] is None and any(
                c.estado == "otra" and c.xy is not None and math.hypot(c.xy[0] - wx, c.xy[1] - wy) < 0.2
                for c in self.candidatas):
            f["puntos"].pop(0)                                      # ahi hay una foto que ya se sabe que es otra
            self._fila_vistos.append((wx, wy))
            f["t_punto"] = now
            self._reset_ruta()
            return 0.0, 0.0
        d = math.hypot(px - x, py - y)
        if d < 0.22 or now - f["t_punto"] > (6.0 if lado else 10.0):   # llego (o no llega: pasar al siguiente)
            self._prog = None                                       # girar y mirar no es estar atascado
            err = wrap(rumbo - th)
            if f["mirando_desde"] is None:
                if "llego" not in f:
                    f["llego"] = d < 0.5                            # (por tiempo y lejos: ese tramo no se vio bien)
                if abs(err) > 0.12 and now - f["t_punto"] < 14.0:
                    return 0.0, float(np.clip(2.2 * err, -1.8, 1.8))
                f["mirando_desde"] = now
            if now - f["mirando_desde"] < self.cfg.mirar_s:        # quieto mirando la pared (2 fotos en la Pi)
                return 0.0, 0.0
            if f.pop("llego", True):
                self._fila_vistos.append((px, py) if wx is None else (wx, wy))
            else:
                f["incompleta"] = True                             # no se llego: no cuenta como mirado (otra fila lo
                                                                   # reintenta) y no vale decidir por ranking
            f["puntos"].pop(0)
            if f.get("replanear"):
                f["replanear"] = False
                if self._empezar_fila(now):                        # ya adentro: ahora se ve la pared de enfrente
                    return 0.0, 0.0
                self._fila = f
            if lado and not any(c.xy is not None and math.hypot(c.xy[0] - wx, c.xy[1] - wy) < 0.3
                                for c in self.candidatas):
                # se miro (o no se pudo llegar) y mas alla de las fotos conocidas no aparecio ninguna: con DOS
                # puntos vacios seguidos la fila termina por ese lado (los puntos de los lados son especulativos;
                # con uno solo se perdia una foto despues de un hueco)
                vac = f.setdefault("vacios", {})
                vac[lado] = vac.get(lado, 0) + 1
                if vac[lado] >= (3 if f.get("tubo") else 2):       # en el tubo puede haber huecos de 2 fotos
                    f["puntos"] = [q for q in f["puntos"] if q[5] != lado]
                    self._fila_vacios.append((wx, wy))
            elif lado:
                f.setdefault("vacios", {})[lado] = 0
            f.update(t_punto=now, mirando_desde=None)
            self._reset_ruta()
            return 0.0, 0.0
        if mapa is not None:
            return self._por_ruta(now, pos, mapa, (px, py), self.cfg.v_fila, ("mapa fino", "mapa"), "la fila")
        return self._mover(now, pos, wrap(math.atan2(py - y, px - x) - th), 0.15)

    def ver_qr(self, now, o):
        """Llamar con cada QR leido (Obs tipo 'qr').

        Un QR cuenta para la persona buscada solo si, EN LA MISMA FOTO, esta en la esquina de abajo de una
        cara (qr._de_que_cara) que esa misma foto reconoce como la buscada (nombre + margen sobre la segunda
        persona, cara de >= px_descartar). No se usa la posicion en el mapa: con el odometro corrido, fotos
        vecinas se mezclan. Se acepta un texto cuando lo leyeron >= 2 fotos, con 2 de ventaja sobre cualquier
        otro texto, y ninguna foto lo vio junto a OTRA persona. Si nunca se cumple, a los qr_max_s se vuelve
        con el nombre: mejor eso que una palabra equivocada."""
        if not self.cfg.buscar or self.palabra is not None or self.estado in (PAUSA, VOLVER, HECHO):
            return
        cara = o.cara
        if cara is None and self.estado == LEER_QR and self.obj is not None and self.obj.xy is not None \
                and self.obj.estado == "objetivo" and self.vot.lider()[0] == self.cfg.buscar \
                and self._qr_es_de_la_buscada(o):
            # de cerca la camara (a 13 cm del suelo) ve el QR pero ya no la cara de arriba: frente a la foto ya
            # reconocida, un QR justo debajo de ella es el suyo. Necesita 2 lecturas
            if o.texto not in self._qr_votos:
                self.nota(now, f"QR debajo de la foto #{self.obj.id} ({self.cfg.buscar}): '{o.texto}'")
            self._qr_votos[o.texto] = self._qr_votos.get(o.texto, 0) + 1
            orden = sorted(((t, n) for t, n in self._qr_votos.items() if t not in self._qr_otros), key=lambda kv: -kv[1])
            if orden and orden[0][1] >= 2 and orden[0][1] - (orden[1][1] if len(orden) > 1 else 0) >= 2:
                self._aceptar_qr(now, orden[0][0], f"leido {orden[0][1]} veces debajo de la foto")
            return
        if cara is None or cara.res is None or cara.px < self.cfg.px_descartar:
            return
        c = self._candidata(cara.xy)
        if c is not None and self.qr_de.get(c.id) != o.texto:
            self.qr_de[c.id] = o.texto                              # (para dibujar: el QR de cada foto)
        r = cara.res
        if r.margen < self.cfg.margen_qr:
            return                                                  # esa foto no deja claro de quien es la cara
        if r.nombre == self.cfg.buscar:
            if o.texto not in self._qr_votos:
                self.nota(now, f"QR junto a {self.cfg.buscar}: '{o.texto}'")
            self._qr_votos[o.texto] = self._qr_votos.get(o.texto, 0) + 1
        else:
            if o.texto not in self._qr_otros:
                self.nota(now, f"QR junto a {r.nombre}: '{o.texto}' (no es la buscada)")
            self._qr_otros[o.texto] = r.nombre
        # los textos vistos junto a otra persona no compiten ni se aceptan
        orden = sorted(((t, n) for t, n in self._qr_votos.items() if t not in self._qr_otros), key=lambda kv: -kv[1])
        if not orden:
            return
        texto, n = orden[0]
        n2 = orden[1][1] if len(orden) > 1 else 0
        clara = r.nombre == self.cfg.buscar and cara.px >= self.cfg.px_qr_1 and r.margen >= 2 * self.cfg.margen_qr
        if (n >= 2 and n - n2 >= 2) or (n == 1 and n2 == 0 and clara):
            for k in self.candidatas:                               # la foto activa: la buscada (para dibujar)
                if k.estado == "objetivo" and self.obj is not k and self.estado != LEER_QR:
                    self.obj, self.vot = k, k.vot
            self._aceptar_qr(now, texto, f"leido {n} veces" if n >= 2 else f"leido 1 vez, cara de {cara.px:.0f} px")

    def _qr_es_de_la_buscada(self, o):
        """Un QR visto sin cara al lado es de la buscada solo si no hay duda con una foto vecina (las fotos van a
        0.40-0.55 m): de frente (a menos de 10 grados del centro de la imagen: en LEER_QR el robot mira a la
        buscada), a menos de 0.25 m de ella en el mapa y al menos 0.15 m mas cerca de ella que de cualquier otra
        candidata."""
        if abs(o.rumbo) > math.radians(10):
            return False
        d_obj = math.hypot(o.xy[0] - self.obj.xy[0], o.xy[1] - self.obj.xy[1])
        if d_obj > 0.25:
            return False
        otras = [math.hypot(o.xy[0] - c.xy[0], o.xy[1] - c.xy[1]) for c in self.candidatas
                 if c is not self.obj and c.xy is not None]
        return not otras or min(otras) > d_obj + 0.15

    def _aceptar_qr(self, now, texto, como):
        self.palabra = texto
        self.nombre = self.cfg.buscar                       # el QR se acepto junto a caras reconocidas como ella
        self.prob = max([self._p_buscada(c) for c in self.candidatas if c.estado == "objetivo"] or [self._p_buscada(self.obj)])
        self.t_b = self.t_b or now
        self.nota(now, f"QR de {self.cfg.buscar}: '{texto}' ({como}): VOLVER")
        self.estado = VOLVER
        self._reset_ruta()
        self.h_elegido = None
        return True

    def _normal_pared(self, xy):
        """Hacia donde mira la pared de la foto (unitario, hacia el robot): recta ajustada a los puntos del
        LiDAR a menos de 0.4 m de la foto. Sin puntos suficientes, la direccion foto -> robot."""
        x, y = self._pos[:2]
        hacia = np.array([x - xy[0], y - xy[1]], float)
        hacia /= max(float(np.linalg.norm(hacia)), 1e-6)
        sx, sy = self._pts_scan
        if sx.size < 5:
            return hacia
        bx, by, bth = self.pos_scan
        gx = bx + math.cos(bth) * sx - math.sin(bth) * sy
        gy = by + math.sin(bth) * sx + math.cos(bth) * sy
        cerca = np.hypot(gx - xy[0], gy - xy[1]) < 0.4
        if cerca.sum() < 5:
            return hacia
        P = np.stack([gx[cerca], gy[cerca]], 1)
        P = P - P.mean(0)
        _, sv, vt = np.linalg.svd(P, full_matrices=False)
        if sv[0] < 3 * max(sv[1], 1e-6):                       # no es una recta (esquina, ruido): no confiar
            return hacia
        n = np.array([-vt[0, 1], vt[0, 0]])
        return n if n @ hacia > 0 else -n

    def _leer_qr(self, now, pos):
        """Frente a la foto de la buscada. Primero se pone DE FRENTE (a 0.6 m sobre la normal de la pared: de
        costado el QR no se lee aunque la cara si se reconozca); despues quieto -> RETROCEDE despacio (21/11: de
        cerca el QR de abajo queda fuera del cuadro) -> quieto -> mira 12 grados a cada lado -> se acerca hasta
        d_qr -> quieto -> otra vez. A los qr_max_s se rinde y vuelve igual
        (anunciara el nombre)."""
        f = self._qr
        if now - f["t_ini"] > self.cfg.qr_max_s:
            orden = sorted(((t, n) for t, n in self._qr_votos.items() if t not in self._qr_otros), key=lambda kv: -kv[1])
            if len(orden) == 1 or (len(orden) > 1 and orden[0][1] > orden[1][1]):
                return self._aceptar_qr(now, orden[0][0], f"el mas leido ({orden[0][1]}) al rendirme") and (0.0, 0.0)
            self.nota(now, f"no pude leer el QR en {self.cfg.qr_max_s:.0f} s: vuelvo y anuncio el nombre")
            self.estado = VOLVER
            self._reset_ruta()
            return 0.0, 0.0
        x, y, th = pos
        if f["paso"] >= 0:
            self._prog = None                                      # quieto mirando: no es estar atascado
        if f["paso"] == -1:                                        # ponerse de frente
            mp = self.obj.mejor_pose if self.cfg.exigir_bandera else None
            if mp is not None and "frente" not in f:
                # (22/11) volver a donde mejor se la vio (y ya cerca: quedarse) en vez de buscar el frente por el mapa
                if math.hypot(mp[0] - x, mp[1] - y) < 0.25:
                    f.update(paso=0, t0=now, th0=mp[2], t_frente=now)
                    self.nota(now, "leo el QR desde aqui (donde mejor la vi)")
                    return 0.0, 0.0
                f["frente"] = (mp[0], mp[1])
                f["t_n"] = now + 1e9                               # fijo: no recalcular por la pared
                self.nota(now, f"vuelvo a donde mejor la vi ({mp[0]:.2f}, {mp[1]:.2f}) para leer el QR")
            if now - f.get("t_n", -1e9) > 1.0:
                f["t_n"] = now
                n = self._normal_pared(self.obj.xy)
                f["frente"] = tuple(np.asarray(self.obj.xy) + self.cfg.d_frente_qr * n)
            fx, fy = f["frente"]
            d = math.hypot(fx - x, fy - y)
            if d < 0.12 or now - f["t0"] > 12.0:
                mirar = math.atan2(self.obj.xy[1] - y, self.obj.xy[0] - x)
                if abs(wrap(mirar - th)) < 0.1 or now - f["t0"] > 15.0:
                    f.update(paso=0, t0=now, th0=mirar, t_frente=now)
                return 0.0, float(np.clip(1.8 * wrap(mirar - th), -0.8, 0.8))
            if self._mapa is not None:
                return self._por_ruta(now, pos, self._mapa, (fx, fy), 0.15, ("mapa fino", "mapa"), "ponerme de frente")
            return self._mover(now, pos, math.atan2(fy - y, fx - x) - th, 0.12)
        o = self.obj.ultima
        dt = now - f["t0"]
        if o is not None and now - o.t < 0.6 and self.obj.pose_ultima is not None:
            f["th0"] = wrap(self.obj.pose_ultima[2] + o.rumbo)    # seguir centrando la foto mientras se la ve

        def girar(obj):
            return float(np.clip(1.8 * wrap(obj - th), -0.5, 0.5))

        def sig():
            f["paso"], f["t0"] = (f["paso"] + 1) % 7, now

        paso = f["paso"]
        if paso == 0:                                              # quieto (la camara toma 2-3 fotos)
            if dt > 1.5:
                sig()
            return 0.0, girar(f["th0"])
        if paso == 1:                                              # RETROCEDER despacio mirando la foto (21/11: de
            if dt > 5.0 or self.libre_atras < 0.45:                # cerca el QR de abajo queda fuera de la camara)
                sig()
                return 0.0, 0.0
            return -0.07, girar(f["th0"])
        if paso == 2:                                              # quieto de lejos
            if dt > 1.5:
                sig()
            return 0.0, girar(f["th0"])
        if paso in (3, 4):                                         # mirar a cada lado (por si el QR no esta debajo)
            obj = wrap(f["th0"] + math.radians(12) * (1 if paso == 3 else -1))
            if abs(wrap(obj - th)) < 0.05 and dt > 0.8 or dt > 3.0:
                sig()
            return 0.0, girar(obj)
        muy_cerca = o is not None and now - o.t < 1.0 and (o.px > self.cfg.px_qr_max or o.borde_sup)
        if paso == 5:                                              # acercarse (el QR sale mas grande)
            if self.libre_frente <= self.cfg.d_qr + 0.02 or dt > 6.0 or muy_cerca:
                sig()
                return 0.0, 0.0
            return 0.06, girar(f["th0"])
        if dt > 1.5:                                               # paso 6: quieto de cerca; luego otra vuelta
            sig()                                                  # (retrocede de nuevo)
        return 0.0, girar(f["th0"])

    def ver_flecha(self, now, o):
        if self.sen is None or not self.cfg.usar_flechas or self.estado not in BUSCANDO:
            return
        if self.cfg.buscar and (self.candidatas or self.bandera is not None):
            return                                                  # ya se llego a la meta: las flechas no aportan
        s = self.sen.agregar(o)
        if s is None:
            return
        if s.n == self.sen.min_n and s.confirmada(self.sen.min_n):
            self.nota(now, f"flecha '{s.dir()}' confirmada en ({s.xy[0]:.1f}, {s.xy[1]:.1f})")
        if (s.usada and self.armado is None and o.dist < 1.2 and abs(o.rumbo) < math.radians(25)
                and now - s.t_usada > 15.0):
            # una senal "usada" hace rato (vista de lejos, o desde otro pasillo) que ahora se tiene de frente y
            # cerca vale otra vez: en un laberinto la misma senal se ve desde varios lados antes de llegar a ella
            s.usada = False
            self.nota(now, f"flecha '{s.dir()}' de frente otra vez: la vuelvo a tener en cuenta")

    # ---------- la meta del 21/11: bandera al fondo, 4 fotos en los costados ----------
    def ver_bandera(self, now, o):
        """Llamar con cada bandera a cuadros vista (Obs tipo 'bandera'). Con 2 vistas se da por confirmada: si se
        esta buscando, se va al centro del tubo frente a ella (META)."""
        if not self.cfg.buscar or self.estado in (PAUSA, VOLVER, HECHO) or o.girando:
            return False
        if math.hypot(o.xy[0] - self.home[0], o.xy[1] - self.home[1]) < self.cfg.r_salida:
            return False                                            # (22/11: tambien habra una bandera en la salida)
        if any(math.hypot(o.xy[0] - fx, o.xy[1] - fy) < 1.0 for fx, fy in getattr(self, "banderas_falsas", [])):
            return False                                            # ya se vio que ahi no hay fotos
        b = self.bandera
        if b is None or math.hypot(b["xy"][0] - o.xy[0], b["xy"][1] - o.xy[1]) > 1.0:
            if b is not None and b["n"] >= 2:
                return False                                        # otra cosa lejos de la bandera ya confirmada
            b = self.bandera = dict(xy=np.array(o.xy, float), n=0, t=now, pos=[])
        b["pos"] = (b["pos"] + [tuple(o.xy)])[-7:]
        b["xy"] = np.median(np.array(b["pos"], float), axis=0)
        b["n"] += 1
        b["t"] = now
        if b["n"] == 2:
            self.nota(now, f"bandera de la meta confirmada en ({b['xy'][0]:.2f}, {b['xy'][1]:.2f}) a {o.dist:.1f} m")
            self.marca(now, "bandera")
        if (b["n"] >= 2 and self.estado in (IDA, EXPLORAR, FILA) and not self._barrido_meta
                and not any(c.estado == "objetivo" for c in self.candidatas)):
            self._empezar_meta(now)
        return True

    def _en_la_meta_sin_salir(self, now):
        """(22/11) Ya se encontro la meta (bandera + vuelta hecha) y no queda foto a la que ir: NO salir a explorar
        el resto del circuito (el robot se iba lejos sabiendo donde estaba la bandera). Otra vuelta desde el
        centro (hasta 2 en total); despues, la mas parecida a la buscada de las fotos de la meta, por descarte."""
        if not (self.cfg.exigir_bandera and self._barrido_meta and self.bandera is not None):
            return False
        self.vueltas_meta = getattr(self, "vueltas_meta", 1)
        if self.vueltas_meta < 2:
            self.vueltas_meta += 1
            self._barrido_meta = False
            for c in self.candidatas:                               # las dudosas se pueden volver a revisar
                if not c.descartada:
                    c.revisiones = 0
            self.nota(now, "no queda foto por revisar en la meta: otra vuelta desde el centro (no salgo a explorar)")
            self._empezar_meta(now)
            return True
        fotos = [c for c in self.candidatas if c.xy is not None and c.vot.n >= 1 and not c.descartada
                 and self._en_meta(c.xy)]
        if not fotos:
            return False
        c = max(fotos, key=self._p_buscada)
        self.nota(now, f"segunda vuelta sin decidir: la #{c.id} (p={self._p_buscada(c):.2f}) es la mas parecida a "
                       f"{self.cfg.buscar} de las de la meta: voy a leer su QR")
        c.estado, c.por_ranking = "objetivo", False
        self.obj, self.vot = c, c.vot
        self.nombre, self.prob = self.cfg.buscar, self._p_buscada(c)
        self.t_b = self.t_b or now
        self.estado = LEER_QR
        self._qr = dict(paso=-1, t0=now, th0=self._pos[2], t_ini=now)
        self._reset_ruta()
        return True

    def _centro_tubo(self, q, n):
        """Corre `q` (frente al fondo, sobre su normal n) al medio entre las paredes de los costados segun el
        ultimo scan: puntos del LiDAR a menos de 0.35 m de q a lo largo de n, a cada lado."""
        sx, sy = self._pts_scan
        if sx.size < 5:
            return q
        bx, by, bth = self.pos_scan
        gx = bx + math.cos(bth) * sx - math.sin(bth) * sy
        gy = by + math.sin(bth) * sx + math.cos(bth) * sy
        u = np.array([-n[1], n[0]])
        rel = np.stack([gx - q[0], gy - q[1]], 1)
        a, lat = rel @ n, rel @ u
        m = np.abs(a) < 0.35
        izq, der = lat[m & (lat > 0)], lat[m & (lat < 0)]
        if izq.size < 3 or der.size < 3:
            return q
        li, ld = float(np.percentile(izq, 10)), float(-np.percentile(der, 90))
        if li + ld < 0.55:
            return q                                                # no cabe (o no es el tubo)
        return q + u * (li - ld) / 2.0

    def _empezar_meta(self, now):
        b = self.bandera
        n = self._normal_pared(b["xy"])                             # de la pared del fondo hacia el tubo
        q = self._centro_tubo(np.asarray(b["xy"], float) + self.cfg.d_meta * n, n)
        rq, rs = self._rango_scan(q)
        if rs is not None and rq < 4.0 and rs < rq - 0.15:
            q = np.asarray(b["xy"], float) + max(0.5, self.cfg.d_meta - 0.3) * n   # detras de algo: mas cerca
        base = math.atan2(-n[1], -n[0])                             # mirando al fondo
        rumbos = [wrap(base + math.radians(g)) for g in self.cfg.barrido_meta]   # (wrap: el giro va por el lado corto,
                                                                                  # paso a paso de 45 en el mismo sentido)
        self._meta = dict(punto=(float(q[0]), float(q[1])), rumbos=rumbos, fase="ir", t_ini=now, t_punto=now,
                          mirando_desde=None, base=base)
        self.estado = META
        self._fila = None
        self._mirar = []
        self._reset_ruta()
        x, y = self._pos[:2]
        self.nota(now, f"META: voy al centro del tubo frente a la bandera ({q[0]:.2f}, {q[1]:.2f}), a "
                       f"{self.cfg.d_meta:.1f} m del fondo y {math.hypot(q[0] - x, q[1] - y):.1f} m de aqui; "
                       f"ahi doy una vuelta lenta ({len(rumbos)} paradas)")

    def _meta_paso(self, now, pos, mapa):
        m = self._meta
        x, y, th = pos
        px, py = m["punto"]
        if m["fase"] == "ir":
            d = math.hypot(px - x, py - y)
            if d < 0.2 or now - m["t_ini"] > 20.0 or (d < 0.35 and self.bloq_frente):
                m["fase"] = "girar"
                m["t_punto"] = now
                self.nota(now, f"en el centro del tubo (a {d:.2f} m del punto): barrido de los costados")
                self.marca(now, "centro")
                self._prog = None
                return 0.0, 0.0
            if mapa is not None:
                return self._por_ruta(now, pos, mapa, (px, py), self.cfg.v_fila, ("mapa fino", "mapa"), "la meta")
            return self._mover(now, pos, wrap(math.atan2(py - y, px - x) - th), 0.15)
        # girar: un rumbo tras otro, quieto mirar_meta_s en cada uno
        self._prog = None
        if not m["rumbos"]:
            self._barrido_meta = True
            self.marca(now, "barrido")
            vistas = [c for c in self.candidatas if c.xy is not None and c.n >= 2 and self._en_meta(c.xy)]
            self.nota(now, f"vuelta completa: {len(vistas)} fotos de la meta vistas"
                           + (f" (se esperaban {self.cfg.n_fotos})" if self.cfg.n_fotos else ""))
            for linea in self._resumen_candidatas():
                self.nota(now, "    " + linea)
            self.estado = IDA
            self.t_nuevo = now
            if not vistas and self.cfg.exigir_bandera:
                # meta = bandera + rostros: una bandera sin fotos alrededor no es la meta (bandera falsa o la de la
                # salida mal ubicada). Se descarta y se sigue buscando otra
                b = self.bandera
                self.banderas_falsas = getattr(self, "banderas_falsas", []) + [tuple(b["xy"])]
                self.nota(now, f"la bandera de ({b['xy'][0]:.2f}, {b['xy'][1]:.2f}) no tiene fotos alrededor: no es la "
                               f"meta, sigo buscando")
                self.bandera, self._barrido_meta, self._meta = None, False, None
                self.marcas.pop("bandera", None)
                return 0.0, 0.0
            self._elegir_candidata(now)                             # ranking -> ACERCAR; si no vio nada: fila
            if self.estado == IDA:
                if self._empezar_fila(now):
                    return 0.0, 0.0
                self.nota(now, "no vi ninguna foto desde el centro: sigo buscando")
                self.h_pref = m["base"]
            return 0.0, 0.0
        err = wrap(m["rumbos"][0] - th)
        if m["mirando_desde"] is None:
            if abs(err) > 0.1 and now - m["t_punto"] < 8.0:
                return 0.0, float(np.clip(2.0 * err, -self.cfg.w_meta, self.cfg.w_meta))   # (girando rapido la foto
            m["mirando_desde"] = now                                                        # sale corrida: 21/11)
            m["fotos0"] = self._fotos
            return 0.0, 0.0
        # quieto hasta tener 2 fotos tomadas ya parado (la Pi tarda ~0.5 s en entregar cada una), minimo
        # mirar_meta_s y maximo mirar_meta_max
        quieto = now - m["mirando_desde"]
        if quieto < self.cfg.mirar_meta_s or (quieto < self.cfg.mirar_meta_max and self._fotos - m["fotos0"] < 3):
            return 0.0, 0.0
        m["rumbos"].pop(0)
        vistas = [c for c in self.candidatas if c.xy is not None and c.vot.n >= 2 and self._en_meta(c.xy)]
        if self.cfg.n_fotos and len(vistas) >= self.cfg.n_fotos and m["rumbos"]:
            self.nota(now, f"ya vi las {len(vistas)} fotos de la meta: corto la vuelta")
            m["rumbos"] = []
        m.update(t_punto=now, mirando_desde=None)
        return 0.0, 0.0

    # ---------- bucle ----------
    def paso(self, now, pos, scan, mapa=None, fotos=0, cam_ok=True, pos_scan=None):
        """-> (v, w). pos = (x, y, yaw) ACTUAL en el marco del mapa (origen = A); pos_scan = pose cuando
        se tomo `scan` (el LiDAR va a ~7 Hz y el robot se mueve entre scans)."""
        self._t_ahora = now
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
        en_pasillo = np.abs(self.py) < 0.20
        fr, at = self.px[en_pasillo & (self.px > 0)], -self.px[en_pasillo & (self.px < 0)]
        self.libre_frente = float(fr.min()) if fr.size else 9.0
        self.libre_atras = float(at.min()) if at.size else 9.0
        self.bloq_frente = self.libre_frente < (PUENTE_SUELTA if self.bloq_frente else PUENTE_PARA)
        self._pos = pos
        self._fotos = fotos
        self._mapa = mapa
        v, w = self._paso(now, pos, scan, mapa, fotos, cam_ok)
        self._now = now
        if self.traza is not None:
            self.traza.append((now, self.estado, *pos, self.h_pref, self.phi, v, w))
        return v, w

    @property
    def quiere_mapa(self):
        """Si este paso necesita la imagen del mapa (cuesta ~1 ms: solo se pide cuando hace falta)."""
        return (self.estado in (VOLVER, EXPLORAR, LEER_QR, FILA, META) or (self.estado == ACERCAR and self.por_mapa)
                or (self.estado == IDA and self.mirada is not None and self.mirada.toca(self._now)))

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
        if now < self._reversa:
            if self.libre_atras > 0.40:
                return REVERSA_V, 0.0
            self._reversa = -1e9                                # atras tambien hay algo: deja de retroceder
        if now < self._maniobra[0]:
            return 0.0, self._maniobra[1]
        if self.estado == IDA and not self._mirar:
            self._quizas_barrer(now, x, y, th, cam_ok)
        if self._mirar and self.estado in BUSCANDO and self.estado != META:
            err = wrap(self._mirar[0] - th)
            if abs(err) < 0.08:
                self._mirar.pop(0)
                if self._mirar_quieto > 0:                        # quieto un momento mirando el primer rumbo
                    self._maniobra = (now + self._mirar_quieto, 0.0)
                    self._mirar_quieto = 0.0
            self._prog = None                                   # mirar no es estar atascado
            return 0.0, float(np.clip(2.0 * err, -1.0, 1.0))
        if self.estado == IDA:
            return self._ida(now, pos, scan, mapa, fotos, cam_ok)
        if self.estado == EXPLORAR:
            return self._explorar(now, pos, scan, mapa, fotos, cam_ok)
        if self.estado == ACERCAR:
            return self._acercar(now, pos, mapa)
        if self.estado == RECONOCER:
            return self._reconocer(now, pos)
        if self.estado == LEER_QR:
            return self._leer_qr(now, pos)
        if self.estado == FILA:
            return self._recorrer_fila(now, pos, mapa)
        if self.estado == META:
            return self._meta_paso(now, pos, mapa)
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
        r = ventana_dinamica(self.px, self.py, phi, vmax, self.cfg.wmax, self.cfg.radio_seguro, self.bloq_frente)
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
        if self.estado == VOLVER and MODOS_RUTA[self.modo_ruta] == "ida exacta":
            # desandando: desviarse lo saca del camino que si cabe. Solo retrocede si esta pegado de frente
            self._prog = (now, x, y)
            if self.libre_frente < PUENTE_SUELTA and self.libre_atras > 0.45:
                self._reversa = now + REVERSA_S
                self.nota(now, f"desandando: pegado de frente ({self.libre_frente:.2f} m), retrocedo un poco")
            return
        L = np.where(np.abs(ANGS) >= math.radians(45), self.libres, 0)
        k = int(np.argmax(L))
        self._prog = (now, x, y)
        if self.libre_frente < PUENTE_SUELTA and self.libre_atras > 0.45:
            # pegado de frente (el freno del puente no lo deja avanzar): primero retrocede un poco, por donde
            # vino, y despues se desvia; girando en el sitio pegado a algo vuelve a quedar trabado
            self._reversa = now + REVERSA_S
            self._desvio = (now + REVERSA_S + 2.5, wrap(th + float(ANGS[k])))
            self.nota(now, f"no avanza (frente a {self.libre_frente:.2f} m): retrocedo y me desvio "
                           f"{math.degrees(ANGS[k]):+.0f} grados")
            return
        self._desvio = (now + 2.5, wrap(th + float(ANGS[k])))
        self.nota(now, f"no avanza: me desvio {math.degrees(ANGS[k]):+.0f} grados ({L[k]:.1f} m libres)")

    def _ida(self, now, pos, scan, mapa=None, fotos=0, cam_ok=True):
        x, y, th = pos
        if self.mirada is not None and cam_ok and self.mirada.toca(now):
            r = self.mirada.paso(now, pos, scan, mapa, fotos, self.cfg.wmax)
            if r is not None:
                if self.mirada.activa is not None and self.mirada.activa.get("nueva", True):
                    self.mirada.activa["nueva"] = False
                    self.nota(now, f"miro al costado ({math.degrees(wrap(self.mirada.activa['obj'] - th)):+.0f} grados): "
                                   "pared cercana sin mirar de frente")
                self._prog = None                               # mirar no es estar atascado
                return r
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
                    self.sen.usar_cerca(s.xy, t=now)
                    self.nota(now, f"flecha '{s.dir()}' a {d:.1f} m: giro en la proxima abertura")
                    break
                elif not delante and d < 1.0:
                    s.usada, s.t_usada = True, now              # ya la paso de largo
        h_obj = None
        if self.armado is not None:
            obj, sxy, t_arm, dir_ = self.armado
            hacia = np.array(sxy) - np.array([x, y])
            falta = float(hacia @ np.array([math.cos(self.h_pref), math.sin(self.h_pref)]))   # m hasta la senal, a lo largo
            lado = np.abs(wrap_arr(th + ANGS - obj)) < math.radians(35)
            # acercarse a la senal lo mas que se pueda (la flecha esta en la pared de enfrente, justo donde hay
            # que doblar) y recien ahi doblar: asi no se toma una abertura anterior
            cerca = falta < 0.35 or self.libre_frente < 0.5
            if not cerca:
                h_obj = math.atan2(hacia[1], hacia[0])
            if (lado & (self.libres >= 1.0)).any() and cerca:
                self.h_pref = obj
                self.giro_hasta = now + 6.0
                self.armado = None
                self._xy_giro = (x, y)                              # (el barrido lateral espera a alejarse del giro)
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
            if self._en_la_meta_sin_salir(now):
                return 0.0, 0.0
            self.nota(now, f"{self.cfg.estancado_s:.0f} s sin sitios nuevos: EXPLORAR (plan B)")
            self.estado = EXPLORAR
            self.exp.start()
            return 0.0, 0.0
        if h_obj is not None:                                    # yendo hacia la senal
            phi = self._elegir(now, pos, h_obj, 0.9, 0.9)
        else:
            phi = self._elegir(now, pos, self.h_pref, 0.9 if now < self.giro_hasta else 0.55, 0.9)
        if (phi is not None and abs(phi) > math.radians(40) and 0.5 < self.libre_frente < 1.6 and cam_ok
                and self.cfg.usar_flechas and self.armado is None and self._n_usadas > 0
                and all(math.hypot(x - a, y - b) > 0.8 for a, b in self._frentes)):
            # antes de doblar por una abertura, un vistazo a la pared de enfrente: ahi puede estar la senal que
            # dice para donde ir (la camara ve 40 grados y al girar la pierde antes de confirmarla). Solo cuando
            # ya se uso alguna senal: en un circuito con senales hay una en cada giro
            self._frentes.append((x, y))
            ang = np.arctan2(self.py, self.px)                     # de frente a la pared: su punto mas cercano
            sel = np.abs(ang) < math.radians(25)                   # dentro del sector de adelante (la normal)
            d = np.hypot(self.px, self.py)
            k = int(np.argmin(np.where(sel, d, np.inf))) if sel.any() else None
            self._mirar = [wrap(th + float(ang[k])) if k is not None else th, th]   # ...y volver al rumbo que traia
            self._mirar_quieto = 0.6
            self._prog = None
            return 0.0, 0.0
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
        sin_ver = min(self.obj.visto_hace(now), now - self.t_acercar)
        if self.px_extra > 1.0 and self.vot.n > 0 and now - self.t_reconocer > 5.0:
            # el segundo acercamiento (por una cara chica) no llego: decidir con los votos que hay, no
            # perderse dando vueltas junto a la pared
            self.nota(now, "no logro acercarme mas: decido con lo que vi")
            return self._decidir(now, "tiempo")
        en_camino = False
        if self.por_mapa:                                       # yendo por el mapa al punto de enfrente: no se la ve
            fx, fy = self._frente_b()                           # mientras se viaja (mira hacia adelante)
            en_camino = math.hypot(fx - pos[0], fy - pos[1]) > 0.35 and now - self.t_acercar < 25.0
        if sin_ver > 2.0:
            # perdida de vista: ir (por el mapa si hace falta) a donde estaba y, ya cerca, girar buscandola
            # 30 s si se la vio desde que se eligio; 12 s si se eligio por el recuerdo y no aparece
            if not en_camino and sin_ver > (30.0 if self.obj.visto_hace(now) < now - self.t_acercar else 12.0):
                self.nota(now, "no encuentro la cara donde la vi: vuelvo a explorar")
                self.obj.t_ignorar = now + 25.0                 # con --buscar: no volver a elegirla enseguida
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
        if self.cfg.temprano and self._seguro() and d < self.cfg.d_temprano:
            return self._decidir(now, "temprano")
        cerca = d <= self.cfg.d_parada + 0.05 and self.px_extra == 1.0
        if (cerca and self.cfg.buscar and o.px < self.cfg.px_confirmar and now - o.t < 1.0
                and self.libre_frente > self.cfg.d_parada + 0.1):
            cerca = False                     # la cara sigue chica: la distancia (LiDAR) debe estar mal; seguir
        grande = o.px >= self.cfg.px_objetivo * self.px_extra and o.t > now - 1.0 and d <= self.cfg.d_max_b
        if grande or cerca or (o.borde_sup and d <= self.cfg.d_max_b):   # (cortada arriba de lejos: no es "cerca")
            self.nota(now, f"frente a la foto: {d:.2f} m, cara de {o.px:.0f} px: RECONOCER"
                           + (" (confirmo con fotos HD)" if (self.cfg.buscar and self.cfg.confirmar_hd
                                                              and self.obj.ancho_max < self.cfg.px_hd_confirmar) else ""))
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
        """Punto frente a la foto de B. Con --buscar: sobre la normal de su pared, a la distancia de mirar la fila
        que deje pasar al robot (en un tubo de 0.9 m, 0.7 m quedaba pegado a la pared de enfrente y el
        acercamiento no avanzaba nunca). Si no, a 0.7 m del lado desde donde se la vio."""
        if self.cfg.buscar:
            q = self._vista_frente(self.obj.xy, self._normal_pared(self.obj.xy), dists=self.cfg.d_vista_fila)
            if q is not None:
                return tuple(q)
        u = self.vista_b - self.obj.xy
        u = u / max(np.linalg.norm(u), 1e-6)
        return tuple(self.obj.xy + 0.7 * u)

    def _seguro(self):
        """La votacion esta segura: probabilidad >= prob_min (0.97) con evidencia suficiente. NO basta con
        haber juntado max_fotos: votos de caras lejanas (14-20 px) llegan a 8 enseguida y dejaban decidir con
        p = 0.7 (de ahi el nombre equivocado del croquis con odometria desviada)."""
        v = self.vot
        return v.n >= v.min_fotos and v.peso >= 1.0 and v.lider()[1] >= v.prob_min

    def _reconocer(self, now, pos):
        o = self.obj.ultima
        if self._seguro():
            return self._decidir(now, "votacion")
        if (self.cfg.buscar and self.obj.estado == "objetivo" and self.vot.n >= 2
                and self.vot.lider()[0] == self.cfg.buscar
                and (not self.obj.por_ranking or (self.vot.lider()[1] >= self.cfg.rank_confirmar
                                                  and self.obj.ancho_max >= self.cfg.px_confirmar))):
            return self._decidir(now, "ranking confirmado de cerca" if self.obj.por_ranking else "votacion temprana")
        sin_ver = self.obj.visto_hace(now)
        if self.cfg.buscar and self.obj.por_ranking and self.vot.n >= 3 and now - self.t_reconocer > 1.5 and (
                (self.cfg.confirmar_hd and (self.obj.ancho_max >= self.cfg.px_hd_confirmar
                                            or now - self.t_reconocer > self.cfg.t_confirmar_max))
                or (not self.cfg.confirmar_hd and (self.obj.ancho_max >= self.cfg.px_confirmar
                                                   or self.libre_frente < self.cfg.d_parada or self.px_extra > 1.0))):
            # elegida por ranking y ya vista de cerca con 3 votos: decidir ya (descartar, dudosa o es), sin esperar
            # los reconocer_max_s (con 4 fotos para revisar, 6 s cada una era mucho)
            return self._decidir(now, "ranking, de cerca")
        espera = (self.cfg.t_confirmar_max if (self.cfg.buscar and self.cfg.confirmar_hd and self.obj.por_ranking)
                  else self.cfg.reconocer_max_s)
        if self.vot.n > 0 and (now - self.t_reconocer > espera
                               or (sin_ver > (6.0 if self.confirmando_hd else 2.5)
                                   and (now - self.t_reconocer > 3.0 or not self.obj.por_ranking))):
            # (por ranking recien llegado y sin verla: primero girar a mirarla, abajo; no decidir con votos viejos)
            t_carrera = now - (self.t_inicio if self.t_inicio is not None else now)
            if (self.obj.ancho_max < self.cfg.px_min_decidir and self.px_extra == 1.0 and not o.borde_sup
                    and t_carrera < self.cfg.t_limite_decidir and self.libre_frente > 0.5):
                # (con la pared a menos de 0.5 m no hay mas que acercarse: la foto es chica, se decide asi)
                # solo vio la cara chica: acercarse mas antes de arriesgar el nombre (una vez)
                self.nota(now, f"cara de solo {self.obj.ancho_max:.0f} px: me acerco mas antes de decidir")
                self.estado = ACERCAR
                self.px_extra = 1.5
                self.t_reconocer = now
                return 0.0, 0.0
            return self._decidir(now, "tiempo")
        if self.cfg.buscar and self.vot.n == 0 and now - self.t_reconocer > 1.5:
            # con --buscar solo votan las fotos tomadas de frente: si no hay votos es que se la ve de costado
            # (pasillo angosto). Ir a un punto frente a la foto, mirarla y volver a acercarse desde ahi
            self.obj.intentos_frente += 1
            if self.obj.intentos_frente > 2:                     # ya se intento ponerse de frente: no insistir
                self.nota(now, f"no logro ver de frente la foto #{self.obj.id}: la dejo por ahora")
                self.obj.t_ignorar = now + 30.0
                self.estado = IDA
                self.t_nuevo = now
                self._elegir_candidata(now)
                return 0.0, 0.0
            n = self._normal_pared(self.obj.xy)
            q = self._vista_frente(self.obj.xy, n, dists=self.cfg.d_vista_fila)
            if q is None:
                q = np.asarray(self.obj.xy) + 0.5 * n
            self._fila = dict(puntos=[(float(q[0]), float(q[1]), math.atan2(-n[1], -n[0]), None, None, 0)],
                              t_punto=now, mirando_desde=None)
            self.estado = FILA
            self._reset_ruta()
            self.nota(now, f"veo la foto #{self.obj.id} de costado: me pongo de frente")
            return 0.0, 0.0
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
        if self.cfg.buscar:
            return self._decidir_buscar(now, por)
        self.nombre, self.prob = self.vot.lider()
        self.t_b = self.t_b or now
        top = ", ".join(f"{n} {p:.2f}" for n, p in self.vot.top(3))
        self.nota(now, f"ES {self.nombre} (p={self.prob:.2f}, {self.vot.n} fotos, por {por}; {top}): VOLVER")
        self.estado = VOLVER
        self._reset_ruta()
        self.h_elegido = None
        return 0.0, 0.0

    def _decidir_buscar(self, now, por):
        """Frente a una candidata: si es la buscada, leer su QR (o volver si ya se leyo); si no, la siguiente."""
        c = self.obj
        nombre, p = self.vot.lider()
        top = ", ".join(f"{n} {q:.2f}" for n, q in self.vot.top(3))
        if nombre == self.cfg.buscar and not (self.cfg.exigir_bandera and c.por_ranking and p < self.cfg.p_aceptar):
            c.estado = "objetivo"
            self.nombre, self.prob = nombre, p
            self.t_b = self.t_b or now
            if not self.cfg.leer_qr:
                return self._decidir_fin(now, por, top)
            self.nota(now, f"foto #{c.id} ES {nombre} (p={p:.2f}, {self.vot.n} fotos, por {por}; {top}): LEER_QR")
            self.estado = LEER_QR
            self._qr = dict(paso=-1, t0=now, th0=self._pos[2], t_ini=now)
            self._reset_ruta()
            return 0.0, 0.0
        quedan = [k for k in self.candidatas if k.xy is not None and k.n >= 2 and k is not c
                  and not k.descartada and k.estado != "otra"]
        todas_vistas = self.cfg.n_fotos and sum(1 for k in self.candidatas if k.xy is not None and k.n >= 2) >= self.cfg.n_fotos
        if p < self.cfg.p_descartar and c.por_ranking:
            # de cerca tampoco se sabe quien es (galeria pobre, foto rara): no se descarta. Si ya se vieron todas las
            # fotos y no queda otra por revisar, es esta (por descarte)
            c.revisiones += 1
            if todas_vistas and not [k for k in quedan if k.revisiones < c.revisiones]:
                c.estado = "objetivo"
                self.nombre, self.prob = self.cfg.buscar, self._p_buscada(c)
                self.t_b = self.t_b or now
                self.nota(now, f"foto #{c.id}: de cerca no se sabe quien es (lider {nombre} {p:.2f}, {self.vot.n} votos, "
                               f"{c.ancho_max:.0f} px) y no queda otra sin descartar: la doy por {self.cfg.buscar}"
                               f"{'' if self.cfg.leer_qr else ': VOLVER'}")
                if not self.cfg.leer_qr:
                    return self._decidir_fin(now, "descarte", top)
                self.estado = LEER_QR
                self._qr = dict(paso=-1, t0=now, th0=self._pos[2], t_ini=now)
                self._reset_ruta()
                return 0.0, 0.0
            c.estado, c.por_ranking = "pendiente", False
            self.nota(now, f"foto #{c.id}: de cerca no se sabe quien es (lider {nombre} {p:.2f}, {self.vot.n} votos, "
                           f"{c.ancho_max:.0f} px): dudosa, reviso primero las otras {len(quedan)}")
        else:
            c.estado = "otra"
            c.descartada = True                                     # se la miro de cerca: no volver a ella
            self.descartes += 1
            self.nota(now, f"foto #{c.id} es {nombre}, no {self.cfg.buscar} (p={p:.2f}, {self.vot.n} votos, "
                           f"{c.ancho_max:.0f} px, por {por}; {top}): descartada #{self.descartes}, quedan "
                           f"{len(quedan)} por revisar")
        self._elegir_candidata(now)
        if self.estado == RECONOCER:                               # no hay otra a la que ir todavia
            self.estado = IDA
            self.t_nuevo = now
        return 0.0, 0.0

    def _decidir_fin(self, now, por, top):
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
        exacta = modos[self.modo_ruta] == "ida exacta"
        # mirar mas adelante en la ruta cuanto mas rapido se va: la esquina se empieza a doblar antes y en
        # curva, sin frenar en seco (a 0.45 m/s mira ~1 m adelante). La ventana dinamica sigue revisando que
        # el arco no pase cerca de nada, asi que cortar la esquina nunca la acerca a la pared
        adelante = 0.35 if exacta else 0.6 + 1.6 * max(0.0, vmax - 0.2)
        resta, meta = self._seguir(x, y, adelante=adelante)
        if exacta:
            vmax = min(vmax, 0.18)                              # por donde paso, despacio y sin cortar esquinas
        if math.hypot(x - destino[0], y - destino[1]) < 0.6:
            meta = destino                                      # el ultimo tramo, al punto exacto
        if self._mejor is None or resta < self._mejor[0] - 0.15:
            self._mejor = (resta, now)
        elif now - self._mejor[1] > 8.0:
            self.modo_ruta = (self.modo_ruta + 1) % len(modos)
            self._mejor, self.t_ruta = (resta, now), -1e9
            self._prog, self._desvio = None, (-1e9, 0.0)        # el nuevo modo empieza sin atasco heredado
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
            if modo == "ida exacta":
                if not self.trail:
                    return [destino]
                d = [math.hypot(px - xy[0], py - xy[1]) for px, py in self.trail]
                i = int(np.argmin(d))
                return [xy] + self.trail[i::-1] + [destino]
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


MODOS_RUTA = ("mapa", "ida exacta", "mapa fino", "ida")
# "ida exacta": el camino de ida al reves, punto por punto y sin atajos. Es lento pero el robot YA paso por
# ahi, asi que cabe: es la salida segura de un rincon con pasos angostos (por eso va segunda).


def wrap_arr(a):
    return np.arctan2(np.sin(a), np.cos(a))
