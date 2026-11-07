"""
Clasificador de gestos de la mano a partir de los 21 puntos de MediaPipe Hands.

Gestos:
  Índice señalando (1 dedo) en 7 direcciones -> GIRAR hacia ahí (relativo al frente del robot):
    DER / IZQ           derecha / izquierda             (90°)
    ARR_DER / ARR_IZQ   diagonal arriba                 (45°)
    ABA_DER / ABA_IZQ   diagonal abajo                  (135°)
    ABA                 abajo                           (180°, dar la vuelta)
  (el índice hacia arriba no es un gesto: arriba se usa para V, 3 dedos y STOP)
  LENTO    ✌️ V: índice + corazón hacia arriba           -> avanzar lento
  RAPIDO   🤟 índice + corazón + anular hacia arriba     -> avanzar rápido
  STOP     ✋ mano abierta hacia arriba (4 dedos)         -> parar
  NINGUNO  cualquier otra cosa o sin mano

El pulgar no cuenta. No depende de mediapipe: recibe una lista de 21 puntos (x, y) normalizados [0, 1]
con y hacia abajo, así se prueba sin cámara.

Las direcciones son las de la IMAGEN que recibe: si el programa voltea la imagen como un espejo (vista
selfie, lo normal), "DER" coincide con la derecha de la persona.
"""
import math
from collections import deque

# Índices de los puntos de MediaPipe Hands
WRIST = 0
INDEX = (5, 6, 7, 8)       # MCP, PIP, DIP, TIP
MIDDLE = (9, 10, 11, 12)
RING = (13, 14, 15, 16)
PINKY = (17, 18, 19, 20)

NINGUNO, STOP, LENTO, RAPIDO = "NINGUNO", "STOP", "LENTO", "RAPIDO"
DER, ARR_DER, ARR_IZQ = "DER", "ARR_DER", "ARR_IZQ"
IZQ, ABA_IZQ, ABA, ABA_DER = "IZQ", "ABA_IZQ", "ABA", "ABA_DER"

# Dirección en la imagen (grados, 0 = derecha, 90 = arriba) de cada señal
CENTROS = {DER: 0.0, ARR_DER: 45.0, ARR_IZQ: 135.0,
           IZQ: 180.0, ABA_IZQ: -135.0, ABA: -90.0, ABA_DER: -45.0}
SENALES = tuple(CENTROS)
AVANCES = (LENTO, RAPIDO)

NOMBRE = {NINGUNO: "--", STOP: "STOP", LENTO: "AVANZAR lento", RAPIDO: "AVANZAR rapido",
          DER: "girar ->", IZQ: "<- girar", ABA: "dar la vuelta",
          ARR_DER: "girar 45 der", ARR_IZQ: "girar 45 izq", ABA_DER: "girar 135 der", ABA_IZQ: "girar 135 izq"}


def es_senal(g):
    return g in CENTROS


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dedo_estirado(pts, dedo, factor=1.15):
    """Estirado si la punta está claramente más lejos de la muñeca que la articulación PIP."""
    mcp, pip, _, tip = dedo
    w = pts[WRIST]
    return _dist(w, pts[tip]) > factor * _dist(w, pts[pip]) and _dist(pts[mcp], pts[tip]) > _dist(pts[mcp], pts[pip])


def dedo_doblado(pts, dedo):
    """Doblado si la punta queda más cerca de la muñeca que la articulación PIP."""
    _, pip, _, tip = dedo
    w = pts[WRIST]
    return _dist(w, pts[tip]) < _dist(w, pts[pip])


def _angulo(a, b):
    """Ángulo en grados del vector a->b en la imagen: 0 = derecha, 90 = arriba, ±180 = izquierda."""
    return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))


def _dif(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _media(pts, idx):
    return (sum(pts[i][0] for i in idx) / len(idx), sum(pts[i][1] for i in idx) / len(idx))


def clasificar(pts, margen=20.0, margen_arriba=40.0):
    """
    Devuelve (gesto, info).
    margen: grados que el índice puede apartarse de cada una de las 7 direcciones (máx. 22.5; entre
            dos direcciones queda un hueco que no cuenta, así no salta de una a otra).
    margen_arriba: grados que V, 3 dedos y STOP pueden apartarse de la vertical.
    """
    info = {"estirados": "", "angulo": None}
    if pts is None or len(pts) < 21:
        return NINGUNO, info

    dedos4 = (INDEX, MIDDLE, RING, PINKY)
    ext = [dedo_estirado(pts, d) for d in dedos4]
    dob = [dedo_doblado(pts, d) for d in dedos4]
    info["estirados"] = "".join("IMAP"[i] if e else "." for i, e in enumerate(ext))

    # Gestos "hacia arriba" según cuántos dedos (desde el índice) están estirados
    n = 4 if all(ext) else 3 if ext[:3] == [True] * 3 and not ext[3] else \
        2 if ext[:2] == [True] * 2 and not ext[2] and not ext[3] and (dob[2] or dob[3]) else 0
    if n:
        k = n if n < 4 else 2
        ang = _angulo(_media(pts, [d[0] for d in dedos4[:k]]), _media(pts, [d[3] for d in dedos4[:k]]))
        if n == 4:                                      # mano abierta: se mide la palma
            ang = _angulo(pts[WRIST], pts[MIDDLE[0]])
        info["angulo"] = ang
        if abs(_dif(ang, 90.0)) > margen_arriba:
            return NINGUNO, info
        return {4: STOP, 3: RAPIDO, 2: LENTO}[n], info

    # Señalar con el índice solo
    if not ext[0] or ext[1] or ext[2] or ext[3] or sum(dob[1:]) < 2:
        return NINGUNO, info
    ang = _angulo(pts[INDEX[0]], pts[INDEX[3]])
    info["angulo"] = ang
    for g, centro in CENTROS.items():
        if abs(_dif(ang, centro)) <= margen:
            return g, info
    return NINGUNO, info


class Estabilizador:
    """
    Confirma un gesto cuando aparece en `n` de los últimos `ventana` cuadros. STOP se confirma antes
    (`n_stop`), porque es el gesto de seguridad. Un gesto confirmado se mantiene hasta que casi
    desaparece de la ventana, así los cuadros de transición no lo sueltan.
    """

    def __init__(self, n=6, ventana=8, n_stop=3):
        self.n_por_defecto, self.n_stop = n, n_stop
        self.hist = deque(maxlen=ventana)
        self.actual = NINGUNO

    def _n(self, g):
        return self.n_stop if g == STOP else self.n_por_defecto

    def update(self, gesto):
        self.hist.append(gesto)
        if self.hist.count(STOP) >= self.n_stop:
            self.actual = STOP
            return self.actual
        for g in set(self.hist):
            if g not in (NINGUNO, self.actual) and self.hist.count(g) >= self._n(g):
                self.actual = g
                return self.actual
        if self.actual != NINGUNO and self.hist.count(self.actual) < self._n(self.actual) // 2:
            self.actual = NINGUNO
        return self.actual
