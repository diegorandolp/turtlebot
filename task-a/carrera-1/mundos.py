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


# Cajas del laboratorio como se vieron con la camara del robot el 19/11: carton mas gris que naranja, con
# logos y letras impresas (Tambo, San Jose...), cinta, etiquetas amarillas y codigos de barra. Las letras y
# los iconos "este lado arriba" son los que dieron candidatas falsas de flecha en el robot.
ANCHO_FOTO = (0.12, 0.16)  # m: las fotos impresas del 19/11 median ~10-15 cm de ancho (la cara, ~5 cm)
PALABRAS_CAJA = ["TAMBO", "SAN JOSE", "Suavecito", "FRAGIL", "VT2", "GLORIA", "LUCKY", "ARROZ", "LECHE",
                 "ESTE LADO", "PERU", "MANI", "INKA", "500 g", "x 12", "KR", "PAMPA", "COSTENO"]
TINTAS_CAJA = [(40, 40, 150), (30, 30, 30), (120, 60, 30), (40, 110, 40), (60, 60, 160), (90, 50, 110)]


def textura_cajas(largo_m, alto_m, rng, color, ppm=250.0, letras=0.7):
    """Frente de una fila de cajas de carton (largo_m x alto_m) a `ppm` px/m: costuras entre cajas cada
    0.3-0.6 m (cada caja un poco mas clara u oscura), cinta, y en una parte de las cajas letras/logos impresos
    (`letras` = fraccion de cajas con impresion), etiquetas amarillas, codigos de barra y a veces el icono de
    dos flechitas "este lado arriba"."""
    W, H = max(int(largo_m * ppm), 8), max(int(alto_m * ppm), 8)
    base = np.empty((H, W, 3), np.float32)
    base[:] = color
    base += rng.normal(0, 3.0, (H, W, 1)).astype(np.float32)                # fibra del carton
    cajas, x = [], 0
    while x < W:
        w = int(rng.uniform(0.30, 0.60) * ppm)
        base[:, x:x + w] *= float(rng.uniform(0.9, 1.08))
        if x > 0:
            base[:, max(x - 1, 0):x + 1] *= 0.7                             # costura entre dos cajas
        if rng.random() < 0.35:                                             # cinta de embalaje
            y0, hc = int(rng.uniform(0.1, 0.8) * H), int(0.05 * ppm)
            base[y0:y0 + hc, x:x + w] = base[y0:y0 + hc, x:x + w] * 1.08 + 6
        cajas.append((x, w))
        x += w
    tex = np.clip(base, 0, 255).astype(np.uint8)
    for x, w in cajas:
        if rng.random() < letras:                                           # impresion: palabras / logo
            for _ in range(int(rng.integers(1, 4))):
                pal = str(rng.choice(PALABRAS_CAJA))
                esc = float(rng.uniform(0.025, 0.08)) * ppm / 22.0          # letras de 2.5 a 8 cm
                grosor = max(1, int(esc * rng.uniform(1.5, 3.0)))
                col = TINTAS_CAJA[int(rng.integers(len(TINTAS_CAJA)))]
                (tw, th), _ = cv2.getTextSize(pal, cv2.FONT_HERSHEY_DUPLEX, esc, grosor)
                px0 = x + int(rng.uniform(0, max(w - tw, 1)))
                py0 = int(rng.uniform(th + 2, max(H - 4, th + 3)))
                cv2.putText(tex, pal, (px0, py0), cv2.FONT_HERSHEY_DUPLEX, esc, col, grosor, cv2.LINE_AA)
            if rng.random() < 0.3:                                          # logo: ovalo con borde
                c = (x + int(rng.uniform(0.2, 0.8) * w), int(rng.uniform(0.3, 0.7) * H))
                ej = (int(rng.uniform(0.04, 0.09) * ppm), int(rng.uniform(0.025, 0.05) * ppm))
                cv2.ellipse(tex, c, ej, 0, 0, 360, TINTAS_CAJA[int(rng.integers(len(TINTAS_CAJA)))], 2, cv2.LINE_AA)
        if rng.random() < 0.25:                                             # etiqueta amarilla con renglones
            ew, eh = int(0.07 * ppm), int(0.045 * ppm)
            ex0, ey0 = x + int(rng.uniform(0, max(w - ew, 1))), int(rng.uniform(0, max(H - eh, 1)))
            tex[ey0:ey0 + eh, ex0:ex0 + ew] = (40, 200, 230)
            for r in range(3):
                yy = ey0 + int((r + 1) * eh / 4)
                cv2.line(tex, (ex0 + 2, yy), (ex0 + ew - 3, yy), (40, 40, 40), 1)
        if rng.random() < 0.2:                                              # codigo de barras
            bx0, by0 = x + int(rng.uniform(0, max(w - 0.06 * ppm, 1))), int(rng.uniform(0.1, 0.7) * H)
            for k2 in range(int(0.06 * ppm)):
                if rng.random() < 0.5 and bx0 + k2 < W:
                    tex[by0:by0 + int(0.03 * ppm), bx0 + k2] = 30
        if rng.random() < 0.12:                                             # "este lado arriba": dos flechitas
            fx, fy = x + int(rng.uniform(0.1, 0.8) * w), int(rng.uniform(0.15, 0.5) * H)
            L = int(0.045 * ppm)
            for d in (0, int(0.025 * ppm)):
                cv2.arrowedLine(tex, (fx + d, fy + L), (fx + d, fy), (30, 30, 30), 2, cv2.LINE_AA, tipLength=0.4)
    return tex


def texturizar(paredes, rng, letras=0.7, ppm=250.0):
    """Le pone textura de cajas a cada pared de carton (in place)."""
    for p in paredes:
        L = math.hypot(p.b[0] - p.a[0], p.b[1] - p.a[1])
        if L > 0.02:
            p.textura = textura_cajas(L, p.alto, rng, p.color, ppm, letras)
            p.ppm = ppm
    return paredes


def carteles_foto_qr(a, b, centro_s, foto, ancho_foto, rng, texto, z_qr=None, gap=None, lado_qr=0.10):
    """La foto impresa con la hoja del QR DEBAJO y centrada (asi las pegamos el 19/11 y el profe dijo que esta
    bien): dos carteles en la pared a->b, centrados en la fraccion `centro_s`. -> ([cartel_foto, cartel_qr],
    z de arriba de la foto)."""
    tex_f, alto_f = textura_rostro(foto, ancho_foto, rng)
    marco = 0.006                                                            # margen blanco de la hoja del QR
    N = int((lado_qr + 2 * marco) * PX_M)
    q = imagen_qr(texto, N - 2 * int(marco * PX_M))
    hoja = np.full((N, N, 3), 245, np.uint8)
    o = int(marco * PX_M)
    hoja[o:o + q.shape[0], o:o + q.shape[1]] = q
    z_qr = float(rng.uniform(0.02, 0.06)) if z_qr is None else z_qr
    gap = float(rng.uniform(0.0, 0.02)) if gap is None else gap
    alto_q = lado_qr + 2 * marco
    k_q = cartel_en_pared(a, b, centro_s, alto_q, z_qr, alto_q, hoja, "qr")
    z_f = z_qr + alto_q + gap
    k_f = cartel_en_pared(a, b, centro_s, ancho_foto, z_f, alto_f, tex_f, "rostro")
    return [k_f, k_q], z_f + alto_f


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
CARTON = (128, 138, 146)  # BGR: cajas de carton como las ve la camara del robot (medido el 19/11: gris-beige)
SUELO_LAB = 178.0         # brillo del piso del laboratorio: gris muy claro (~195-200 cerca de la camara, 19/11)
ALTO_CAJAS = (0.30, 0.45) # m: las cajas son bajas, la camara ve el aula por encima
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


def _pared_meta(pared, t, W, H):
    """(a, b, fraccion) de la pared de la meta en la coordenada t, con a->b orientada para mirar adentro."""
    if pared == "der":
        return (W, 0), (W, H), t / H
    if pared == "arriba":
        return (W, H), (0, H), (W - t) / W
    return (0, 0), (W, 0), t / W


def _cartel_meta(pared, t, W, H, ancho, z0, alto, tex):
    """Cartel en la pared, mirando hacia adentro de la sala, centrado en t."""
    if pared == "der":
        return cartel_en_pared((W, 0), (W, H), t / H, ancho, z0, alto, tex, "rostro")
    if pared == "arriba":
        return cartel_en_pared((W, H), (0, H), (W - t) / W, ancho, z0, alto, tex, "rostro")
    return cartel_en_pared((0, 0), (W, 0), t / W, ancho, z0, alto, tex, "rostro")


def _sala_con_cajas(W, H, cajas, inicio, t_b, rng, persona, nombre_mundo, semilla, achicar=True, n_rostros=1,
                    sep=None, pared="der", rango=None, extra=(), paso=PASO_MIN):
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
    paredes = ex.paredes(0.42, CARTON)                                    # el laboratorio: cajas de carton
    def d_rect(cx, cy, w, h, px, py):                                    # distancia del rectangulo al punto
        return math.hypot(max(abs(px - cx) - w / 2, 0.0), max(abs(py - cy) - h / 2, 0.0))

    for (cx, cy, w, h, alto) in cajas:
        if min(d_rect(cx, cy, w, h, mx, my) for mx, my in metas) < (0.15 if not achicar else 0.5):
            continue   # nada delante de una foto: hay que poder ponerse a 0.5-0.75 m de frente (laberinto: pasillo)
        for _ in range(8 if achicar else 1):
            x0, y0, x1, y1 = max(cx - w / 2, 0.0), max(cy - h / 2, 0.0), min(cx + w / 2, W), min(cy + h / 2, H)
            prueba = ex.libre.copy()
            ex.rect(x0, y0, x1, y1, valor=False)
            if all(_camino_libre(ex, inicio[:2], m, radio=paso) for m in metas):
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
        texturizar(paredes, rng)
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
    z_qr = float(rng.uniform(0.02, 0.06))                                 # misma altura para toda la fila
    sep_real = abs(ts[1] - ts[0]) if len(ts) > 1 else 1.0
    for (nombre, foto), t in zip(elegidas, ts):
        tq = texto_qr(rng)
        while tq in textos:
            tq = texto_qr(rng)
        textos.add(tq)
        ancho_foto = float(min(rng.uniform(*ANCHO_FOTO), sep_real - 0.04))    # impresiones chicas, que no se monten
        a, b, fr = _pared_meta(pared, t, W, H)
        ks, _ = carteles_foto_qr(a, b, fr, foto, ancho_foto, rng, tq, z_qr=z_qr)
        carteles += ks
        rostros.append((nombre, tq, _en_pared(pared, t, W, H)))
    nombre_obj, foto_obj = elegidas[k_obj]
    texturizar(paredes, rng)
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
    cajas = [((x0 + x1) / 2 * sx, (y0 + y1) / 2 * sy, (x1 - x0) * sx, (y1 - y0) * sy, float(rng.uniform(*ALTO_CAJAS)))
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
        cajas.append((cx, cy, w, h, float(rng.uniform(*ALTO_CAJAS))))
    t_b = y_b
    azar = pared == "azar"
    if azar:
        pared = str(rng.choice(["der", "arriba", "abajo"]))
    if pared != "der":                                                      # arriba / abajo: lejos de A
        t_b = float(rng.uniform(0.55, 0.85) * W)
    nombre = f"lab:{semilla}" if azar else f"denso:{semilla}"
    return _sala_con_cajas(W, H, cajas, inicio, t_b, rng, persona, nombre, semilla, n_rostros=n_rostros,
                           sep=sep, pared=pared)


def laberinto(semilla, C=7, F=6, celda=0.8, persona=None, n_rostros=1, sep=None, flechas=True):
    """Laberinto largo de C x F celdas de `celda` m (defecto 7 x 6 de 0.8 m = 5.6 x 4.8 m) hecho con tabiques
    de carton de 15 cm: pasillos de ~0.65 m (el robot mide 0.34), callejones sin salida y algunos ciclos (mas de un camino). La
    meta es una sala de >= 2.6 m (3-4 celdas) por 2 celdas de fondo en la pared derecha, con las fotos en fila; A es la celda de la columna
    izquierda mas lejana (por el laberinto) de la meta, mirando hacia donde sigue el camino. Con `flechas`,
    una senal (de color al azar) en cada giro del camino correcto donde hay pared enfrente."""
    rng = np.random.default_rng(semilla)
    W, H, t = C * celda, F * celda, 0.15
    ns = min(F, max(3, math.ceil(2.6 / celda)))            # celdas de la sala de la meta (6 fotos en >= 2.6 m)
    nc = 1 if celda >= 1.1 else 2                          # columnas: la sala tiene >= 1.3 m de fondo para mirar
    f0 = int(rng.integers(0, F - ns + 1))                  # filas f0..f0+ns-1 de las ultimas nc columnas
    sala = {(C - 1 - j, f0 + k) for k in range(ns) for j in range(nc)}
    # muros entre celdas: abiertos[(a, b)] con a < b
    abiertos = set()
    visto, pila = {(0, int(rng.integers(F)))}, []
    pila.append(next(iter(visto)))
    while pila:                                              # DFS: laberinto perfecto (un solo camino)
        c = pila[-1]
        vec = [(c[0] + dx, c[1] + dy) for dx, dy in DIRS
               if 0 <= c[0] + dx < C and 0 <= c[1] + dy < F and (c[0] + dx, c[1] + dy) not in visto]
        if not vec:
            pila.pop()
            continue
        v = vec[int(rng.integers(len(vec)))]
        abiertos.add(tuple(sorted((c, v))))
        visto.add(v)
        pila.append(v)
    for k in range(ns):                                      # la sala de la meta no tiene tabiques adentro
        for j in range(nc):
            if k + 1 < ns:
                abiertos.add(((C - 1 - j, f0 + k), (C - 1 - j, f0 + k + 1)))
            if j + 1 < nc:
                abiertos.add(((C - 2 - j, f0 + k), (C - 1 - j, f0 + k)))
    todos = [tuple(sorted(((i, j), (i + dx, j + dy)))) for i in range(C) for j in range(F) for dx, dy in ((1, 0), (0, 1))
             if i + dx < C and j + dy < F]
    cerrados = [m for m in todos if m not in abiertos]
    for k in rng.permutation(len(cerrados))[:max(1, len(cerrados) // 8)]:   # algunos ciclos
        abiertos.add(cerrados[k])

    def vecinos(c):
        return [(c[0] + dx, c[1] + dy) for dx, dy in DIRS
                if tuple(sorted((c, (c[0] + dx, c[1] + dy)))) in abiertos]

    # A: la celda de la columna 0 mas lejana de la sala (BFS)
    dist, cola = {c: 0 for c in sala}, list(sala)
    for c in cola:
        for v in vecinos(c):
            if v not in dist:
                dist[v] = dist[c] + 1
                cola.append(v)
    a_celda = max(((0, j) for j in range(F)), key=lambda c: dist.get(c, -1))
    camino = [a_celda]
    while camino[-1] not in sala:
        camino.append(min(vecinos(camino[-1]), key=lambda v: dist.get(v, 1e9)))
    if flechas:
        # en cada giro del camino correcto tiene que haber pared enfrente para poner la senal (como en el
        # laboratorio: la flecha se ve de frente al llegar). Si ahi el laberinto sigue derecho, se cierra:
        # el camino correcto no cambia y lo que queda detras sigue siendo callejon o ciclo
        for k in range(1, len(camino) - 1):
            u = (camino[k][0] - camino[k - 1][0], camino[k][1] - camino[k - 1][1])
            v = (camino[k + 1][0] - camino[k][0], camino[k + 1][1] - camino[k][1])
            if u != v and camino[k] not in sala:
                abiertos.discard(tuple(sorted((camino[k], (camino[k][0] + u[0], camino[k][1] + u[1])))))
    centro = lambda c: np.array([(c[0] + 0.5) * celda, (c[1] + 0.5) * celda])   # noqa: E731
    d0 = camino[1][0] - camino[0][0], camino[1][1] - camino[0][1]
    ca = centro(a_celda)
    inicio = (float(ca[0] + 0.1 * d0[0]), float(ca[1] + 0.1 * d0[1]), math.atan2(d0[1], d0[0]))
    # tabiques: uno por muro cerrado (cajas de t de ancho), con las esquinas tapadas
    cajas = []
    for m in todos:
        if m in abiertos:
            continue
        (i, j), (i2, j2) = m
        alto = float(rng.uniform(*ALTO_CAJAS))
        if i2 != i:                                          # muro vertical en x = (i+1) * celda
            cajas.append(((i + 1) * celda, (j + 0.5) * celda, t, celda + t, alto))
        else:                                                # muro horizontal en y = (j+1) * celda
            cajas.append(((i + 0.5) * celda, (j + 1) * celda, celda + t, t, alto))
    extra = []
    if flechas:
        for k in range(1, len(camino) - 1):
            u = np.array(camino[k], float) - np.array(camino[k - 1], float)
            v = np.array(camino[k + 1], float) - np.array(camino[k], float)
            giro = u[0] * v[1] - u[1] * v[0]
            adelante = (camino[k][0] + int(u[0]), camino[k][1] + int(u[1]))
            hay_pared = (not (0 <= adelante[0] < C and 0 <= adelante[1] < F)
                         or tuple(sorted((camino[k], adelante))) not in abiertos)
            if giro == 0 or not hay_pared:
                continue
            dir_ = "izq" if giro > 0 else "der"
            pp = np.array([-u[1], u[0]])
            pc = centro(camino[k]) + u * (celda / 2 - t / 2)
            lado = 0.20
            col = COLORES_SENAL[int(rng.integers(len(COLORES_SENAL)))]
            tex = textura_flecha(dir_, rng, estilo="giro", lado_m=lado, color=col)
            extra.append(cartel_en_pared(pc - pp * 0.35, pc + pp * 0.35, 0.5, lado, 0.05, lado, tex, "flecha:" + dir_))
    t_b = (f0 + ns / 2) * celda
    return _sala_con_cajas(W, H, cajas, inicio, t_b, rng, persona, f"laberinto:{semilla}:{C}:{F}" + ("" if celda == 0.8 else f":{celda:g}"), semilla,
                           achicar=False, n_rostros=n_rostros, sep=sep, pared="der",
                           rango=(f0 * celda + 0.35, (f0 + ns) * celda - 0.35), extra=extra,
                           paso=min(PASO_MIN, (celda - t) / 2 - 0.1))   # pasillos angostos: holgura en la rejilla


def tubo(semilla, persona=None, n_rostros=5, sep=None, giros=None, ramas=None, flechas=True, cajas=None):
    """Circuito como el del simulacro del 19/11 y lo que dijo el profe: pasillos de cajas bajas con 1-3 giros
    de 90 grados (una senal de color en la pared de enfrente de cada giro, a veces una rama sin salida del otro
    lado) y al final un callejon sin salida ("el tubo") con las fotos pegadas en sus DOS COSTADOS, nunca en la
    pared del fondo. Fotos chicas (12-16 cm) con su QR debajo. Algunas cajas sueltas en los pasillos (no en el
    tubo) dejan pasos de >= 0.5 m."""
    rng = np.random.default_rng(semilla)
    ancho = round(rng.uniform(0.85, 1.20) / (2 * RES)) * 2 * RES          # pasillos de ~1 m como en el lab
    h = ancho / 2
    sepm = 0.25                                                           # pared minima entre dos pasillos
    n_giros = int(rng.integers(1, 4) if giros is None else giros)
    n_rostros = max(int(n_rostros), 1)
    sep_f = float(rng.uniform(0.40, 0.55)) if sep is None else float(sep)
    largo_tubo = max(1.6, (math.ceil(n_rostros / 2) + 0.6) * sep_f + h + 0.5)   # caben media fila por lado + margen
    for _intento in range(30):                                            # se rehace si un tramo no cabe
        ex = Excavadora()
        pos, d = np.array([0.0, 0.0]), 0
        centros, giros_l, ok = [pos.copy()], [], True
        for i in range(n_giros + 1):
            u = np.array(DIRS[d], float)
            ultimo = i == n_giros
            minimo = largo_tubo if ultimo else 1.4
            largo = round((largo_tubo + float(rng.uniform(0.0, 0.6)) if ultimo else rng.uniform(1.6, 3.0)) / RES) * RES
            while True:
                fin = pos + u * largo
                x0, y0, x1, y1 = _rect(pos + u * (2 * h + 2 * sepm), fin, h + sepm)
                if i == 0 or (not ex.ocupado(x0, y0, x1, y1) and np.abs(fin).max() < 6.0):
                    break
                largo = round(largo * 0.85 / RES) * RES
                if largo < minimo:
                    ok = False
                    break
            if not ok:
                break
            ex.rect(*_rect(pos, fin, h))
            pos = fin
            centros.append(pos.copy())
            if not ultimo:
                giro = int(rng.choice([-1, 1]))
                giros_l.append((pos.copy(), d, giro))
                d = (d + giro) % 4
        if ok:
            break
    n_ramas = int(rng.integers(0, 2) if ramas is None else ramas)
    for p, d_in, giro in giros_l[:n_ramas]:
        u = np.array(DIRS[(d_in - giro) % 4], float)
        fin = p + u * round(rng.uniform(1.0, 1.6) / RES) * RES
        x0, y0, x1, y1 = _rect(p + u * (2 * h + 2 * sepm), fin, h + sepm)
        if not ex.ocupado(x0, y0, x1, y1):
            ex.rect(*_rect(p, fin, h))
    inicio = (float(centros[0][0] - h + 0.35), float(centros[0][1]), 0.0)
    # el tubo: ultimo tramo, de centros[-2] (el ultimo giro) a centros[-1] (el fondo)
    ud = np.array(DIRS[d], float)
    perp = np.array([-ud[1], ud[0]])                                      # a la izquierda de quien entra
    c0, c1 = centros[-2], centros[-1]
    largo_t = float(np.linalg.norm(c1 - c0))
    # puestos en los costados: desde la boca del tubo (pasado el empalme) hasta 0.25 m del fondo
    t_ini, t_fin = h + 0.25, largo_t + h - 0.25                          # medidos desde c0 a lo largo de ud
    puestos = []
    for lado in (1, -1):
        k = 0
        while t_ini + k * sep_f <= t_fin:
            puestos.append((t_ini + k * sep_f + float(rng.uniform(-0.04, 0.04)), lado))
            k += 1
    rng.shuffle(puestos)
    elegidos = sorted(puestos[:n_rostros])
    # cajas sueltas en los pasillos de antes del tubo (obstaculos), sin cerrar el camino
    meta = tuple(c1 - ud * 0.4)
    n_cajas = int(rng.integers(0, 3) if cajas is None else cajas)
    paredes_cajas = []
    for _ in range(40):
        if len(paredes_cajas) >= n_cajas or len(centros) < 3:
            break
        i = int(rng.integers(len(centros) - 2))                          # no en el tubo
        a_, b_ = centros[i], centros[i + 1]
        seg = b_ - a_
        L = float(np.linalg.norm(seg))
        t = float(rng.uniform(0.35, 0.75))
        if i == 0 and t * L < 1.0:
            continue
        u = seg / max(L, 1e-9)
        pp = np.array([-u[1], u[0]])
        lado_c = float(rng.uniform(0.25, 0.40))
        c = a_ + seg * t + pp * rng.choice([-1, 1]) * (h - lado_c / 2)    # pegada a una pared del pasillo
        r = lado_c / 2
        prueba = ex.libre.copy()
        ex.rect(c[0] - r, c[1] - r, c[0] + r, c[1] + r, valor=False)
        if not _camino_libre(ex, inicio[:2], meta, radio=PASO_MIN):
            ex.libre = prueba
            continue
        paredes_cajas.append((c, r, float(rng.uniform(*ALTO_CAJAS))))
    paredes = ex.paredes(0.42, CARTON)
    for c, r, alto in paredes_cajas:
        col = tuple(int(np.clip(v * rng.uniform(0.85, 1.12), 0, 255)) for v in CARTON)
        _caja(paredes, c, r, r, alto, col)
    texturizar(paredes, rng)
    # senales: en la pared de enfrente de cada giro, mirando al que llega
    carteles = []
    if flechas:
        for p, d_in, giro in giros_l:
            u = np.array(DIRS[d_in], float)
            pp = np.array([-u[1], u[0]])
            frente = p + u * h
            a, b = frente - pp * h, frente + pp * h
            dir_ = "izq" if giro > 0 else "der"
            lado = float(rng.uniform(0.18, 0.22))
            col = COLORES_SENAL[int(rng.integers(len(COLORES_SENAL)))]
            tex = textura_flecha(dir_, rng, estilo="giro", lado_m=lado, color=col)
            carteles.append(cartel_en_pared(a, b, 0.5 + float(rng.uniform(-0.15, 0.15)), lado,
                                            float(rng.uniform(0.04, 0.12)), lado, tex, "flecha:" + dir_))
    # fotos con QR: personas distintas, la buscada en uno de los puestos al azar
    k_obj = int(rng.integers(len(elegidos)))
    gente, usadas = [], set()
    for i in range(len(elegidos)):
        for _ in range(50):
            nombre, foto = elegir_persona(rng, persona if i == k_obj else None)
            if nombre not in usadas:
                break
        usadas.add(nombre)
        gente.append((nombre, foto))
    rostros, textos = [], set()
    z_qr = float(rng.uniform(0.02, 0.06))
    boca = c0 - ud * h                                                    # t = 0: donde empieza el tubo
    for (nombre, foto), (t, lado) in zip(gente, elegidos):
        tq = texto_qr(rng)
        while tq in textos:
            tq = texto_qr(rng)
        textos.add(tq)
        ancho_foto = float(rng.uniform(*ANCHO_FOTO))
        pared_c = boca + ud * t + perp * lado * h                         # punto de la pared del costado
        a, b = pared_c - ud * 0.3, pared_c + ud * 0.3
        if lado > 0:                                                      # que mire hacia adentro del tubo
            a, b = b, a
        ks, _ = carteles_foto_qr(a, b, 0.5, foto, ancho_foto, rng, tq, z_qr=z_qr)
        carteles += ks
        rostros.append((nombre, tq, (float(pared_c[0]), float(pared_c[1]))))
    nombre_obj, foto_obj = gente[k_obj]
    return Mundo(paredes, inicio, carteles, [], rostros[k_obj][2], nombre_obj,
                 f"tubo:{semilla}+{len(elegidos)}" + (f":{sep:g}" if sep else ""), luz=float(rng.uniform(0.85, 1.1)),
                 semilla=semilla, foto_b=ref_foto(foto_obj), fotos_excluir=tuple(ref_foto(f) for _, f in gente),
                 qr=rostros[k_obj][1], rostros=rostros, suelo=SUELO_LAB)


def construir(nombre, persona=None):
    """Mundos: demo | demo-sin-flechas | sala[:N] | aleatorio:N[:sin] | croquis[:W:H] | denso:N[:W:H] | lab:N
    (lab = sala con cajas y la meta en la pared de la derecha, arriba o abajo) | laberinto:N[:C:F] (laberinto
    largo de C x F celdas de 0.8 m, defecto 7 x 6; con :celda, otro ancho: laberinto:1:6:5:0.7).
    Con "+K" al final (solo croquis y denso), la meta tiene K fotos con QR: p. ej. croquis+6, denso:3+6.
    "+K:S" fija la separacion entre fotos en S m (defecto 0.40-0.55 al azar): denso:3+6:0.3 = mas juntas."""
    n_rostros, sep, nombre_orig = 1, None, nombre
    if "+" in nombre:
        nombre, k = nombre.split("+", 1)
        if ":" in k:
            k, sep = k.split(":", 1)
            sep = float(sep)
        n_rostros = int(k)
        if not nombre.startswith(("croquis", "denso", "lab", "laberinto", "tubo")):
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
    if partes[0] == "laberinto":                                 # laberinto:N[:C:F[:celda]] (celdas de 0.8 m)
        semilla = int(partes[1]) if len(partes) > 1 else 0
        C = int(partes[2]) if len(partes) > 2 else 7
        F = int(partes[3]) if len(partes) > 3 else 6
        celda = float(partes[4]) if len(partes) > 4 else 0.8
        return laberinto(semilla, C, F, celda, persona=persona, n_rostros=n_rostros, sep=sep)
    if partes[0] == "tubo":                                      # tubo:N[+K] (defecto 5 fotos en los costados)
        semilla = int(partes[1]) if len(partes) > 1 else 0
        return tubo(semilla, persona, n_rostros=n_rostros if "+" in nombre_orig else 5, sep=sep)
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
                     "lab:N, laberinto:N[:C:F], y +K al final para K fotos con QR)")
