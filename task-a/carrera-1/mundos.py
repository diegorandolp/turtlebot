"""
Mundos para el simulador de carrera-1: circuitos con paredes, obstaculos, senales de flecha y la
foto de una persona en B. Hay mundos fijos (para la demo) y un generador aleatorio (para medir
cuantas veces sale bien la estrategia en circuitos que nunca vio).

  construir("demo")              circuito en S con flechas, una rama sin salida y obstaculos
  construir("demo-sin-flechas")  el mismo, sin flechas: el robot tiene que decidir solo
  construir("sala")              sala abierta con obstaculos sueltos y la foto en la pared del fondo
  construir("aleatorio:17")      circuito aleatorio (semilla 17), con flechas
  construir("aleatorio:17:sin")  el mismo sin flechas

El circuito aleatorio se arma "excavando" rectangulos en una rejilla (tramos rectos con giros de
90 grados, a veces una rama sin salida en un cruce) y las paredes son el borde de lo excavado.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from flechas import chevron, flecha_curva, flecha_giro, flecha_recta     # noqa: E402
from simulador import Cartel, Cilindro, Mundo, Pared                    # noqa: E402

ROSTROS = HERE.parents[1] / "datos" / "rostros"
PX_M = 900                                  # px de textura por metro de cartel


# ---------------------------------------------------------------- texturas

def textura_rostro(foto, ancho_m, rng):
    """Foto impresa: la imagen sobre papel blanco con un margen pequeno."""
    img = cv2.imread(str(foto))
    if img is None:
        raise FileNotFoundError(foto)
    h, w = img.shape[:2]
    alto_m = ancho_m * h / w
    W, H = int(ancho_m * PX_M), int(alto_m * PX_M)
    m = max(4, int(0.04 * W))
    tex = np.full((H, W, 3), 245, np.uint8)
    tex[m:H - m, m:W - m] = cv2.resize(img, (W - 2 * m, H - 2 * m), interpolation=cv2.INTER_AREA)
    return tex, alto_m


def textura_flecha(direccion, rng, estilo=None, lado_m=0.22, color=(25, 25, 25), aro=None):
    """Senal cuadrada de papel con una flecha (estilos variados). direccion: 'der' | 'izq' | 'arriba'."""
    estilo = estilo or rng.choice(["giro", "recta", "curva", "chevron"])
    if estilo == "giro":
        m = flecha_giro(rng.uniform(0.12, 0.2), 0.28, rng.uniform(0.24, 0.3), rng.choice([0.0, 0.18]))
    elif estilo == "curva":
        m = flecha_curva(rng.uniform(0.1, 0.16), 0.28, 0.25, rng.uniform(120, 170))
    elif estilo == "chevron":
        m = chevron(rng.uniform(0.12, 0.2))
    else:
        m = flecha_recta(rng.uniform(0.12, 0.25), rng.uniform(0.28, 0.4), rng.uniform(0.28, 0.38))
    if direccion == "izq":
        m = cv2.flip(m, 1)
    elif direccion == "arriba":
        m = cv2.rotate(m, cv2.ROTATE_90_COUNTERCLOCKWISE)
    aro = (estilo == "giro" and rng.random() < 0.7) if aro is None else aro
    N = int(lado_m * PX_M)
    tex = np.full((N, N, 3), 240, np.uint8)
    k = 0.62 if aro else 0.82
    mm = cv2.resize(m, (int(N * k), int(N * k)), interpolation=cv2.INTER_AREA)
    o = (N - mm.shape[0]) // 2
    sel = mm > 127
    tex[o:o + mm.shape[0], o:o + mm.shape[1]][sel] = color
    if aro:
        cv2.circle(tex, (N // 2, N // 2), int(N * 0.44), color, max(3, int(N * 0.06)), cv2.LINE_AA)
    return tex


def cartel_en_pared(a, b, centro_s, ancho, z0, alto, textura, tipo, lado=1):
    """Cartel pegado a la pared a->b, centrado en la fraccion `centro_s`, mirando hacia `lado`
    (+1 = a la izquierda de a->b)."""
    a, b = np.array(a, float), np.array(b, float)
    L = np.linalg.norm(b - a)
    u = (b - a) / L
    c = a + u * centro_s * L
    p, q = c - u * ancho / 2, c + u * ancho / 2
    if lado < 0:
        p, q = q, p
    n = np.array([-(q - p)[1], (q - p)[0]]) / ancho
    p, q = p + n * 0.004, q + n * 0.004                    # 4 mm delante de la pared
    return Cartel(tuple(p), tuple(q), z0, z0 + alto, textura, tipo)


_FRONTAL = {}


def _es_frontal(foto):
    """La persona mira a la camara (la foto de B sera un retrato, no un perfil)."""
    if foto not in _FRONTAL:
        sys.path.insert(0, str(HERE.parents[1]))
        from reconocimiento import Detector
        from reconocimiento.evaluar import es_frontal
        if "det" not in _FRONTAL:
            _FRONTAL["det"] = Detector()
        img = cv2.imread(str(foto))
        caras = _FRONTAL["det"](img) if img is not None else []
        _FRONTAL[foto] = bool(caras) and es_frontal(caras[0])
    return _FRONTAL[foto]


def elegir_persona(rng, persona=None, frontal=True):
    """-> (nombre, ruta de la foto) al azar de datos/rostros (solo retratos de frente si `frontal`)."""
    carpetas = sorted(d for d in ROSTROS.iterdir() if d.is_dir())
    if persona:
        carpetas = [d for d in carpetas if d.name.replace("_", " ") == persona or d.name == persona]
    orden = rng.permutation(len(carpetas))
    for i in orden:
        d = carpetas[int(i)]
        fotos = sorted(d.glob("*.jpg"))
        for j in rng.permutation(len(fotos)):
            if not frontal or _es_frontal(fotos[int(j)]):
                return d.name.replace("_", " "), fotos[int(j)]
    d = carpetas[int(orden[0])]
    return d.name.replace("_", " "), sorted(d.glob("*.jpg"))[0]


def ref_foto(foto):
    return f"{foto.parent.name}/{foto.name}"


# ---------------------------------------------------------------- rejilla -> paredes

RES = 0.05


class Excavadora:
    """Rejilla booleana de lo libre; las paredes son su borde."""

    def __init__(self, tam_m=14.0):
        self.n = int(tam_m / RES)
        self.libre = np.zeros((self.n, self.n), bool)
        self.o = tam_m / 2                                  # el origen del mundo en el centro

    def celda(self, x, y):
        return int(round((y + self.o) / RES)), int(round((x + self.o) / RES))

    def rect(self, x0, y0, x1, y1, valor=True):
        r0, c0 = self.celda(min(x0, x1), min(y0, y1))
        r1, c1 = self.celda(max(x0, x1), max(y0, y1))
        self.libre[max(r0, 0):r1, max(c0, 0):c1] = valor

    def ocupado(self, x0, y0, x1, y1):
        r0, c0 = self.celda(min(x0, x1), min(y0, y1))
        r1, c1 = self.celda(max(x0, x1), max(y0, y1))
        return self.libre[max(r0, 0):r1, max(c0, 0):c1].any()

    def paredes(self, alto, color):
        """Bordes libre/ocupado -> segmentos horizontales y verticales unidos."""
        L = self.libre
        segs = []
        # bordes horizontales: entre fila r-1 y r
        dif = L[1:, :] != L[:-1, :]
        for r in range(dif.shape[0]):
            fila = dif[r]
            c = 0
            while c < len(fila):
                if fila[c]:
                    c0 = c
                    while c < len(fila) and fila[c]:
                        c += 1
                    y = (r + 1) * RES - self.o
                    segs.append(((c0 * RES - self.o, y), (c * RES - self.o, y)))
                c += 1
        dif = L[:, 1:] != L[:, :-1]
        for c in range(dif.shape[1]):
            col = dif[:, c]
            r = 0
            while r < len(col):
                if col[r]:
                    r0 = r
                    while r < len(col) and col[r]:
                        r += 1
                    x = (c + 1) * RES - self.o
                    segs.append(((x, r0 * RES - self.o), (x, r * RES - self.o)))
                r += 1
        return [Pared(a, b, alto, color) for a, b in segs]


def _camino_libre(ex, a, b, radio=0.22):
    """Hay paso de a a b para un robot de `radio` (BFS en la rejilla con lo ocupado inflado)."""
    k = int(math.ceil(radio / RES))
    occ = (~ex.libre).astype(np.uint8)
    inflado = cv2.dilate(occ, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))).astype(bool)
    ok = ~inflado
    s, g = ex.celda(*a), ex.celda(*b)
    if not ok[s] or not ok[g]:
        return False
    n, lab = cv2.connectedComponents(ok.astype(np.uint8), connectivity=8)
    return lab[s] == lab[g]


DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]                   # este, norte, oeste, sur


def _rect(p, q, h):
    """Rectangulo (x0, y0, x1, y1) del pasillo de p a q, de medio ancho h, incluidos los cuadrados de los extremos."""
    return min(p[0], q[0]) - h, min(p[1], q[1]) - h, max(p[0], q[0]) + h, max(p[1], q[1]) + h


def _caja(paredes, c, w2, h2, alto, col):
    x0, y0, x1, y1 = c[0] - w2, c[1] - h2, c[0] + w2, c[1] + h2
    paredes += [Pared((x0, y0), (x1, y0), alto, col), Pared((x1, y0), (x1, y1), alto, col),
                Pared((x1, y1), (x0, y1), alto, col), Pared((x0, y1), (x0, y0), alto, col)]


def circuito_aleatorio(semilla, flechas=True, persona=None, tramos=None, ramas=None, obstaculos=None):
    """Pasillo de `tramos` tramos rectos con giros de 90 grados; ramas sin salida en algunos giros
    (del lado contrario al giro: ahi una flecha evita perder tiempo); obstaculos que dejan paso."""
    rng = np.random.default_rng(semilla)
    ancho = round(rng.uniform(0.95, 1.5) / (2 * RES)) * 2 * RES    # multiplos de la rejilla: las paredes caen justo
    h = ancho / 2
    sep = 0.25                                              # pared minima entre dos pasillos
    n_tramos = int(tramos or rng.integers(2, 5))
    ex = Excavadora()
    pos, d = np.array([0.0, 0.0]), 0
    centros, giros = [pos.copy()], []                       # giros: (punto, direccion de entrada, +1 izq / -1 der)
    for i in range(n_tramos):
        u = np.array(DIRS[d], float)
        largo = round(rng.uniform(2.0, 3.4) / RES) * RES
        while True:
            fin = pos + u * largo
            # lo nuevo (sin el cuadrado del empalme) no puede tocar lo ya excavado
            x0, y0, x1, y1 = _rect(pos + u * (2 * h + 2 * sep), fin, h + sep)
            if i == 0 or (not ex.ocupado(x0, y0, x1, y1) and np.abs(fin).max() < 6.0):
                break
            largo = round(largo * 0.8 / RES) * RES
            if largo < 1.2:
                break
        if largo < 1.2:                                     # no cabe otro tramo: termina en el giro anterior
            if giros:
                _, d, _ = giros.pop()
            break
        ex.rect(*_rect(pos, fin, h))
        pos = fin
        centros.append(pos.copy())
        if i < n_tramos - 1:
            giro = int(rng.choice([-1, 1]))
            giros.append((pos.copy(), d, giro))
            d = (d + giro) % 4
    n_ramas = int(rng.integers(0, 2) if ramas is None else ramas)
    for p, d_in, giro in giros[:n_ramas]:
        u = np.array(DIRS[(d_in - giro) % 4], float)
        fin = p + u * round(rng.uniform(1.0, 1.8) / RES) * RES
        x0, y0, x1, y1 = _rect(p + u * (2 * h + 2 * sep), fin, h + sep)
        if not ex.ocupado(x0, y0, x1, y1):
            ex.rect(*_rect(p, fin, h))
    alto_pared = rng.uniform(0.3, 0.6)
    gris = int(rng.integers(170, 225))
    color = (gris, int(np.clip(gris + rng.integers(-8, 8), 0, 255)), int(np.clip(gris + rng.integers(-10, 10), 0, 255)))
    paredes = ex.paredes(alto_pared, color)

    inicio =(centros[0][0] - h + 0.40, centros[0][1], 0.0)            # A: al principio, mirando a lo largo del pasillo
    ud = np.array(DIRS[d], float)
    perp = np.array([-ud[1], ud[0]])
    fondo = centros[-1] + ud * h                                         # B: pared del fondo del ultimo tramo
    nombre, foto = elegir_persona(rng, persona)
    ancho_foto = rng.uniform(0.18, 0.30)
    tex, alto_foto = textura_rostro(foto, ancho_foto, rng)
    s = 0.5 + rng.uniform(-0.25, 0.25)
    a, b = fondo - perp * h, fondo + perp * h                            # asi el cartel mira hacia el pasillo
    carteles = [cartel_en_pared(a, b, s, ancho_foto, rng.uniform(0.0, 0.12), alto_foto, tex, "rostro")]
    b_xy = tuple(a + (b - a) * s)
    if flechas:
        for p, d_in, giro in giros:
            u = np.array(DIRS[d_in], float)
            pp = np.array([-u[1], u[0]])
            frente = p + u * h
            a, b = frente - pp * h, frente + pp * h                    # mirando al robot que llega
            dir_ = "izq" if giro > 0 else "der"
            lado = rng.uniform(0.18, 0.28)
            tex = textura_flecha(dir_, rng, lado_m=lado)
            carteles.append(cartel_en_pared(a, b, 0.5 + rng.uniform(-0.2, 0.2), lado, rng.uniform(0.0, 0.15), lado,
                                            tex, "flecha:" + dir_))
    meta = tuple(np.array(b_xy) - ud * 0.6)
    n_obs = int(rng.integers(1, 5) if obstaculos is None else obstaculos)
    cil, puestos = [], 0
    for _ in range(80):
        if puestos >= n_obs:
            break
        i = int(rng.integers(len(centros) - 1))
        seg = centros[i + 1] - centros[i]
        L = np.linalg.norm(seg)
        t = rng.uniform(0.3, 0.8)
        if i == 0 and t * L < 1.0:
            continue                                         # no pegado a la salida
        u = seg / L
        pp = np.array([-u[1], u[0]])
        lado = rng.uniform(0.15, 0.40)
        c = centros[i] + seg * t + pp * rng.choice([-1, 1]) * rng.uniform(0.0, max(h - lado / 2, 0.01))
        prueba = ex.libre.copy()
        r = lado / 2
        ex.rect(c[0] - r, c[1] - r, c[0] + r, c[1] + r, valor=False)
        ok = _camino_libre(ex, inicio[:2], meta)
        if not ok:
            ex.libre = prueba
            continue
        puestos += 1
        if rng.random() < 0.3:
            cil.append(Cilindro(tuple(c), r, rng.uniform(0.3, 0.6), (int(rng.integers(40, 120)), 90, 160)))
        else:
            col = tuple(int(v) for v in rng.integers(60, 200, 3))
            _caja(paredes, c, r, r, rng.uniform(0.25, 0.5), col)
    return Mundo(paredes, inicio, carteles, cil, b_xy, nombre, f"aleatorio:{semilla}" + ("" if flechas else ":sin"),
                 luz=float(rng.uniform(0.8, 1.1)), semilla=semilla, foto_b=ref_foto(foto))

def sala(semilla=3, persona=None):
    """Sala abierta de 5 x 4 m con obstaculos sueltos; la foto en la pared del fondo."""
    rng = np.random.default_rng(semilla)
    W, H = 5.0, 4.0
    col = (205, 205, 210)
    paredes = [Pared((0, 0), (W, 0), 0.5, col), Pared((W, 0), (W, H), 0.5, col),
               Pared((W, H), (0, H), 0.5, col), Pared((0, H), (0, 0), 0.5, col)]
    for (cx, cy, w, h) in ((1.8, 1.4, 0.35, 0.35), (2.6, 2.7, 0.4, 0.25), (3.6, 1.2, 0.3, 0.5)):
        x0, y0, x1, y1 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
        c = (90, 120, 170)
        paredes += [Pared((x0, y0), (x1, y0), 0.35, c), Pared((x1, y0), (x1, y1), 0.35, c),
                    Pared((x1, y1), (x0, y1), 0.35, c), Pared((x0, y1), (x0, y0), 0.35, c)]
    nombre, foto = elegir_persona(rng, persona)
    tex, alto = textura_rostro(foto, 0.24, rng)
    y = rng.uniform(1.2, 2.8)
    carteles = [cartel_en_pared((W, 0), (W, H), y / H, 0.24, 0.03, alto, tex, "rostro")]
    return Mundo(paredes, (0.5, 2.0, 0.0), carteles, [Cilindro((1.2, 3.0), 0.12, 0.4)], (W, y), nombre, "sala",
                 semilla=semilla, foto_b=ref_foto(foto))


def construir(nombre, persona=None):
    partes = nombre.split(":")
    if partes[0] == "demo":
        return circuito_aleatorio(7, flechas=True, persona=persona, tramos=3, ramas=1, obstaculos=3)
    if partes[0] == "demo-sin-flechas":
        return circuito_aleatorio(7, flechas=False, persona=persona, tramos=3, ramas=1, obstaculos=3)
    if partes[0] == "sala":
        return sala(int(partes[1]) if len(partes) > 1 else 3, persona)
    if partes[0] == "aleatorio":
        semilla = int(partes[1]) if len(partes) > 1 else 0
        return circuito_aleatorio(semilla, flechas=not (len(partes) > 2 and partes[2] == "sin"), persona=persona)
    raise ValueError(f"mundo desconocido: {nombre} (demo | demo-sin-flechas | sala | aleatorio:N[:sin])")
