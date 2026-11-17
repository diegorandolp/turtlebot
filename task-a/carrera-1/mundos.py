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


# Palabras de los QR del simulador. El robot NO las conoce: lee lo que diga el QR (en la competencia
# las palabras pueden ser nuevas).
PALABRAS_QR = ["alfa", "beta", "gamma", "delta", "omega", "phi", "rho", "sigma", "kappa", "lambda", "tau",
               "zeta", "condor", "vicuna", "quinua", "huaca", "puma", "colibri", "lucuma", "chasqui"]


def texto_qr(rng):
    return f"milestone_{rng.choice(PALABRAS_QR)}_{int(rng.integers(10, 100))}"


def imagen_qr(texto, lado_px):
    """QR real (cv2.QRCodeEncoder) con su margen blanco, en BGR de lado_px x lado_px."""
    q = cv2.QRCodeEncoder.create().encode(texto)
    q = cv2.copyMakeBorder(q, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=255)
    q = cv2.resize(q, (lado_px, lado_px), interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(q, cv2.COLOR_GRAY2BGR)


def textura_rostro_qr(foto, ancho_m, rng, texto, qr_m=0.10, esquina=None):
    """Foto impresa con el QR (de `qr_m` de lado, como los del laboratorio) pegado en una esquina de abajo."""
    tex, alto_m = textura_rostro(foto, ancho_m, rng)
    H, W = tex.shape[:2]
    lado = min(int(qr_m * PX_M), W - 4, H - 4)
    q = imagen_qr(texto, lado)
    esquina = esquina or str(rng.choice(["izq", "der"]))
    x0 = 2 if esquina == "izq" else W - lado - 2
    tex[H - lado - 2:H - 2, x0:x0 + lado] = q
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



# ---------------------------------------------------------------- salas con muchas cajas (laboratorio)

PASO_MIN = 0.24          # radio con el que se exige que haya camino: deja pasos de >= ~0.48 m
CARTON = (95, 140, 178)  # BGR: las paredes y obstaculos del laboratorio son cajas de carton (marron medio)
SUELO_LAB = 115.0        # brillo del piso del laboratorio (gris medio)
COLORES_SENAL = [(25, 25, 25), (150, 60, 20), (40, 40, 190), (40, 140, 30)]   # BGR: negra, azul, roja, verde


def _posiciones_meta(t_b, k, rng, sep=None, rango=(0.4, 3.6)):
    """Coordenada (a lo largo de la pared) de k fotos en fila alrededor de t_b, separadas `sep` m (defecto:
    0.40-0.55 al azar; menos si no caben en `rango`)."""
    if k <= 1:
        return [t_b]
    lo, hi = rango
    sep = min(float(rng.uniform(0.40, 0.55)) if sep is None else sep, (hi - lo) / (k - 1))
    t0 = float(np.clip(t_b - sep * (k - 1) / 2, lo, hi - sep * (k - 1)))
    return [t0 + i * sep for i in range(k)]


def _en_pared(pared, t, W, H, d=0.0):
    """Punto de la pared `pared` (der | arriba | abajo) en la coordenada t, a d m hacia adentro de la sala."""
    if pared == "der":
        return (W - d, t)
    if pared == "arriba":
        return (t, H - d)
    return (t, d)


def _cartel_meta(pared, t, W, H, ancho, z0, alto, tex):
    """Cartel en la pared, mirando hacia adentro de la sala, centrado en t."""
    if pared == "der":
        return cartel_en_pared((W, 0), (W, H), t / H, ancho, z0, alto, tex, "rostro")
    if pared == "arriba":
        return cartel_en_pared((W, H), (0, H), (W - t) / W, ancho, z0, alto, tex, "rostro")
    return cartel_en_pared((0, 0), (W, 0), t / W, ancho, z0, alto, tex, "rostro")


def _sala_con_cajas(W, H, cajas, inicio, t_b, rng, persona, nombre_mundo, semilla, achicar=True, n_rostros=1,
                    sep=None, pared="der", rango=None, extra=()):
    """Sala W x H (m) con cajas (cx, cy, ancho, largo, alto) y la meta en la pared `pared` (der | arriba |
    abajo) alrededor de la coordenada t_b a lo largo de esa pared. Una caja que cerraria el camino de A a
    la meta se achica (o se quita): siempre hay un paso de al menos 2 * PASO_MIN. Con achicar=False las
    cajas van tal cual y, si alguna cierra el camino, ValueError (un mundo dibujado a mano debe tener paso).
    n_rostros > 1: la meta tiene esa cantidad de fotos en fila (personas distintas), cada una con un QR en
    una esquina de abajo; la persona buscada es una de ellas al azar (mundo.persona, su QR en mundo.qr).
    `extra`: carteles adicionales (flechas)."""
    ex = Excavadora()
    ex.rect(0.0, 0.0, W, H)
    largo = H if pared == "der" else W
    ts = _posiciones_meta(t_b, n_rostros, rng, sep, rango or (0.4, largo - 0.4))   # >= 0.4: el robot llega
    k_obj = int(rng.integers(len(ts))) if n_rostros > 1 else 0
    t_b = ts[k_obj]
    metas = [_en_pared(pared, t, W, H, 0.30) for t in ts]                # donde termina de acercarse a cada foto
    paredes = ex.paredes(0.5, CARTON)                                     # el laboratorio: cajas de carton
    for (cx, cy, w, h, alto) in cajas:
        if min(math.hypot(cx - mx, cy - my) for mx, my in metas) < 0.35:
            continue                                                        # nada pegado delante de una foto
        for _ in range(8 if achicar else 1):
            x0, y0, x1, y1 = max(cx - w / 2, 0.0), max(cy - h / 2, 0.0), min(cx + w / 2, W), min(cy + h / 2, H)
            prueba = ex.libre.copy()
            ex.rect(x0, y0, x1, y1, valor=False)
            if all(_camino_libre(ex, inicio[:2], m, radio=PASO_MIN) for m in metas):
                col = tuple(int(np.clip(c * rng.uniform(0.8, 1.15), 0, 255)) for c in CARTON)   # carton, mas o menos claro
                _caja(paredes, ((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0) / 2, (y1 - y0) / 2, alto, col)
                break
            ex.libre = prueba
            if not achicar:
                raise ValueError(f"{nombre_mundo}: la caja {(x0, y0, x1, y1)} cierra el camino de A a la meta")
            w, h = w * 0.8, h * 0.8                                         # no cabe: la caja un poco mas chica
    b = _en_pared(pared, t_b, W, H)
    if n_rostros <= 1:
        nombre, foto = elegir_persona(rng, persona)
        ancho_foto = rng.uniform(0.18, 0.28)
        tex, alto_foto = textura_rostro(foto, ancho_foto, rng)
        carteles = [_cartel_meta(pared, t_b, W, H, ancho_foto, rng.uniform(0.0, 0.12), alto_foto, tex)] + list(extra)
        return Mundo(paredes, inicio, carteles, [], b, nombre, nombre_mundo, luz=float(rng.uniform(0.8, 1.1)),
                     semilla=semilla, foto_b=ref_foto(foto), fotos_excluir=(ref_foto(foto),), suelo=SUELO_LAB)
    # varias fotos con QR: personas distintas, la buscada en la posicion k_obj
    elegidas, usadas = [], set()
    for i in range(len(ts)):
        for _ in range(50):
            nombre, foto = elegir_persona(rng, persona if i == k_obj else None)
            if nombre not in usadas:
                break
        usadas.add(nombre)
        elegidas.append((nombre, foto))
    if elegidas[k_obj][0] != (persona or elegidas[k_obj][0]):
        raise ValueError(f"no se pudo poner a {persona} en la meta")
    carteles, rostros, textos = [], [], set()
    z0 = float(rng.uniform(0.0, 0.10))                                    # misma altura para toda la fila
    sep_real = abs(ts[1] - ts[0]) if len(ts) > 1 else 1.0
    for (nombre, foto), t in zip(elegidas, ts):
        tq = texto_qr(rng)
        while tq in textos:
            tq = texto_qr(rng)
        textos.add(tq)
        ancho_foto = float(min(rng.uniform(0.20, 0.26), sep_real - 0.04))    # que no se monten
        tex, alto_foto = textura_rostro_qr(foto, ancho_foto, rng, tq, qr_m=min(0.10, ancho_foto - 0.02))
        carteles.append(_cartel_meta(pared, t, W, H, ancho_foto, z0, alto_foto, tex))
        rostros.append((nombre, tq, _en_pared(pared, t, W, H)))
    nombre_obj, foto_obj = elegidas[k_obj]
    return Mundo(paredes, inicio, carteles + list(extra), [], b, nombre_obj,
                 f"{nombre_mundo}+{len(ts)}" + (f":{sep:g}" if sep else ""),
                 luz=float(rng.uniform(0.8, 1.1)), semilla=semilla, foto_b=ref_foto(foto_obj),
                 fotos_excluir=tuple(ref_foto(f) for _, f in elegidas), qr=rostros[k_obj][1], rostros=rostros,
                 suelo=SUELO_LAB)


# El croquis del laboratorio (dibujo a mano: rojo = cajas, azul = senales). El dibujo no esta a escala (los
# pasos entre cajas son mas angostos que el robot), asi que se rehizo en metros con la MISMA FORMA: A abajo
# a la izquierda en un rincon cerrado por una repisa y un tabique pegado al suelo; para salir hay que subir,
# cruzar por arriba y bajar por el pasillo de la derecha, cerrado por un tabique largo, donde estan las fotos
# (supuesto: el dibujo no marca la meta). Ningun paso mide menos de ~0.5 m; no hay atajo derecho.
_CROQUIS_CAJAS = [            # x0, y0, x1, y1 (m) en una sala de 4 x 4
    (0.00, 1.75, 0.80, 2.00),   # repisa sobre A, pegada a la pared izquierda
    (1.40, 0.00, 1.65, 1.80),   # tabique pegado abajo: desde A no se puede ir derecho al este
    (1.95, 2.50, 2.20, 3.15),   # tabique al medio, arriba
    (2.60, 3.45, 2.85, 4.00),   # tabique pegado arriba
    (1.95, 0.85, 2.15, 1.85),   # tabique al medio
    (2.35, 2.50, 2.85, 2.72),   # caja sobre la entrada del pasillo de la meta
    (2.20, 1.05, 2.35, 1.95),   # tabique chico
    (1.90, 0.30, 2.40, 0.52),   # caja horizontal abajo
    (2.65, 0.00, 2.85, 2.30),   # tabique largo pegado abajo: el pasillo de la meta (~1.15 m) solo se entra por arriba
]
# senales del dibujo (azul): (x, y de la pared, rumbo del robot que llega, direccion de la flecha)
_CROQUIS_FLECHAS = [((1.40, 0.95), 0.0, "izq"),             # en el tabique, frente a A: subir
                    ((1.10, 4.00), math.pi / 2, "der"),     # en la pared de arriba: ir al este
                    ((4.00, 3.05), 0.0, "der")]             # en la pared derecha: bajar por el pasillo


def croquis(W=4.0, H=4.0, semilla=0, persona=None, n_rostros=1, sep=None, flechas=True, color_flechas=True):
    """El circuito del croquis (ver _CROQUIS_CAJAS), escalado a W x H m (defecto 4 x 4). Las fotos estan en
    la pared derecha, dentro del pasillo de la meta; con `flechas`, las senales del dibujo (de colores al
    azar con `color_flechas`: en el laboratorio pueden no ser negras)."""
    rng = np.random.default_rng(semilla)
    sx, sy = W / 4.0, H / 4.0
    cajas = [((x0 + x1) / 2 * sx, (y0 + y1) / 2 * sy, (x1 - x0) * sx, (y1 - y0) * sy, float(rng.uniform(0.3, 0.5)))
             for x0, y0, x1, y1 in _CROQUIS_CAJAS]
    extra = []
    if flechas:
        for (px, py), rumbo, dir_ in _CROQUIS_FLECHAS:
            u = np.array([math.cos(rumbo), math.sin(rumbo)])
            pp = np.array([-u[1], u[0]])
            c = np.array([px * sx, py * sy])
            lado = 0.22
            col = COLORES_SENAL[int(rng.integers(len(COLORES_SENAL)))] if color_flechas else COLORES_SENAL[0]
            tex = textura_flecha(dir_, rng, estilo="giro", lado_m=lado, color=col)
            extra.append(cartel_en_pared(c - pp * 0.4, c + pp * 0.4, 0.5, lado, 0.05, lado, tex, "flecha:" + dir_))
    inicio = (0.45 * sx, 0.9 * sy, 0.0)
    return _sala_con_cajas(W, H, cajas, inicio, 1.2 * sy, rng, persona, f"croquis:{W:g}:{H:g}", semilla,
                           achicar=False, n_rostros=n_rostros, sep=sep, rango=(0.4 * sy, 2.1 * sy), extra=extra)


def sala_densa(semilla, W=4.0, H=4.0, persona=None, n_rostros=1, sep=None, pared="der"):
    """Sala de W x H m con 7 a 11 cajas al azar (algunas largas como tabiques, algunas pegadas a la pared),
    A a la izquierda mirando al este y la meta en la pared `pared` (der | arriba | abajo; "azar" = una de
    las tres). Pasos angostos (>= ~0.48 m) y caminos que no se ven desde A."""
    rng = np.random.default_rng(semilla)
    inicio = (0.4, float(rng.uniform(0.6, H - 0.6)), 0.0)
    y_b = float(rng.uniform(0.5, H - 0.5))
    cajas, n_cajas = [], int(rng.integers(7, 12))
    metas_y = [y_b] if n_rostros <= 1 else [y_b + (i - (n_rostros - 1) / 2) * 0.48 for i in range(n_rostros)]
    for _ in range(200):
        if len(cajas) >= n_cajas:
            break
        if rng.random() < 0.45:                                             # tabique: largo y angosto
            w, h = float(rng.uniform(0.18, 0.3)), float(rng.uniform(0.8, 1.8))
        else:
            w = h = float(rng.uniform(0.25, 0.6))
        if rng.random() < 0.5:
            w, h = h, w
        cx, cy = float(rng.uniform(0.9, W - 0.3)), float(rng.uniform(0.2, H - 0.2))
        if rng.random() < 0.25:                                             # pegada a una pared
            cy = h / 2 if rng.random() < 0.5 else H - h / 2
        cerca_meta = min(math.hypot(cx - (W - 0.3), cy - y) for y in metas_y) if pared == "der" else 9.0
        if math.hypot(cx - inicio[0], cy - inicio[1]) < 0.75 or cerca_meta < 0.6:
            continue
        cajas.append((cx, cy, w, h, float(rng.uniform(0.25, 0.5))))
    t_b = y_b
    if pared == "azar":
        pared = str(rng.choice(["der", "arriba", "abajo"]))
    if pared != "der":                                                      # arriba / abajo: lejos de A
        t_b = float(rng.uniform(0.55, 0.85) * W)
    nombre = f"denso:{semilla}" if pared == "der" else f"lab:{semilla}"
    return _sala_con_cajas(W, H, cajas, inicio, t_b, rng, persona, nombre, semilla, n_rostros=n_rostros,
                           sep=sep, pared=pared)


def construir(nombre, persona=None):
    """Mundos: demo | demo-sin-flechas | sala[:N] | aleatorio:N[:sin] | croquis[:W:H] | denso:N[:W:H] | lab:N
    (lab = sala con cajas y la meta en la pared de la derecha, arriba o abajo).
    Con "+K" al final (solo croquis y denso), la meta tiene K fotos con QR: p. ej. croquis+6, denso:3+6.
    "+K:S" fija la separacion entre fotos en S m (defecto 0.40-0.55 al azar): denso:3+6:0.3 = mas juntas."""
    n_rostros, sep = 1, None
    if "+" in nombre:
        nombre, k = nombre.split("+", 1)
        if ":" in k:
            k, sep = k.split(":", 1)
            sep = float(sep)
        n_rostros = int(k)
        if not nombre.startswith(("croquis", "denso", "lab")):
            raise ValueError(f"'+{k}' (varias fotos con QR) solo existe para croquis y denso, no para {nombre}")
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
    if partes[0] == "croquis":                                   # croquis[:W:H]
        W = float(partes[1]) if len(partes) > 1 else 4.0
        H = float(partes[2]) if len(partes) > 2 else W
        return croquis(W, H, persona=persona, n_rostros=n_rostros, sep=sep)
    if partes[0] == "lab":                                       # lab:N = denso con la meta en cualquier pared
        semilla = int(partes[1]) if len(partes) > 1 else 0
        return sala_densa(semilla, 4.0, 4.0, persona, n_rostros=n_rostros, sep=sep, pared="azar")
    if partes[0] == "denso":                                     # denso:N[:W:H]
        semilla = int(partes[1]) if len(partes) > 1 else 0
        W = float(partes[2]) if len(partes) > 2 else 4.0
        H = float(partes[3]) if len(partes) > 3 else W
        return sala_densa(semilla, W, H, persona, n_rostros=n_rostros, sep=sep)
    raise ValueError(f"mundo desconocido: {nombre} "
                     "(demo | demo-sin-flechas | sala | aleatorio:N[:sin] | croquis[:W:H] | denso:N[:W:H], "
                     "lab:N, y +K al final de croquis/denso/lab para K fotos con QR)")
