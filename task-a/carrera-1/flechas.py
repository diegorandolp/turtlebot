"""
Deteccion de senales de flecha en la camara del robot (Tarea A, carrera-1).

No sabemos como seran las flechas del circuito, ni si habra. Por eso el detector es generico y no
se entreno con ninguna senal concreta:

  1. "Tinta": pixeles bastante mas oscuros que su entorno en el canal minimo min(B, G, R). Asi una
     flecha negra, roja, verde o azul sobre papel claro se ve igual.
  2. Cada mancha de tinta de tamano y forma plausibles, rodeada de fondo claro (el papel), es una
     candidata. El aro de una senal circular es otra mancha y se descarta solo (no parece flecha).
  3. La mancha se normaliza a 32x32 y se compara (correlacion normalizada) con un banco de flechas
     SINTETICAS dibujadas aqui mismo: rectas, de giro (como la senal de transito), curvas y
     chevrones, gruesas y finas, con y sin aro. Cada una apuntando a la derecha, su espejo a la
     izquierda y las rectas giradas hacia arriba/abajo.
  4. Gana la direccion con mayor parecido; la flecha cuenta si el parecido supera `umbral` y saca
     `margen` al espejo (izquierda y derecha son simetricas: la diferencia decide el sentido).

Uso:
    det = DetectorFlechas()
    for f in det(img_bgr):          # de mayor a menor confianza
        print(f.dir, f.caja, f.score)
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

N = 32                                  # lado del parche normalizado
DER, IZQ, ARRIBA, ABAJO = "der", "izq", "arriba", "abajo"
OPUESTA = {DER: IZQ, IZQ: DER, ARRIBA: ABAJO, ABAJO: ARRIBA}


@dataclass
class Flecha:
    dir: str                # "der" | "izq" | "arriba" | "abajo"
    caja: tuple             # (x, y, w, h) en px de la imagen
    score: float            # parecido con la mejor plantilla (correlacion, -1..1)
    margen: float           # ventaja sobre la direccion opuesta

    @property
    def centro(self):
        x, y, w, h = self.caja
        return x + w / 2.0, y + h / 2.0

    @property
    def lado(self):
        return float(max(self.caja[2], self.caja[3]))


# ---------------------------------------------------------------- flechas sinteticas

S = 128                                 # lienzo para dibujar las plantillas


def _triangulo(img, base_x, cy, largo, medio_ancho):
    pts = np.array([[base_x, cy - medio_ancho], [base_x + largo, cy], [base_x, cy + medio_ancho]], np.int32)
    cv2.fillPoly(img, [pts], 255)


def flecha_recta(grosor=0.18, cabeza=0.35, ancho_cabeza=0.30, largo=0.85):
    """-> mascara SxS de una flecha recta hacia la derecha (medidas en fraccion del lienzo)."""
    img = np.zeros((S, S), np.uint8)
    x0, x1 = S * (1 - largo) / 2, S * (1 + largo) / 2
    base = x1 - cabeza * S
    cy = S / 2
    cv2.rectangle(img, (int(x0), int(cy - grosor * S / 2)), (int(base + 2), int(cy + grosor * S / 2)), 255, -1)
    _triangulo(img, base, cy, cabeza * S, ancho_cabeza * S)
    return img


def flecha_giro(grosor=0.16, cabeza=0.28, ancho_cabeza=0.24, radio=0.18, alto=0.75, x_tallo=0.30, y_brazo=0.35):
    """Flecha de giro a la derecha (senal de transito): tallo vertical desde abajo, curva y punta a la derecha."""
    img = np.zeros((S, S), np.uint8)
    t = max(2, int(grosor * S))
    xs, yb, ya = int(x_tallo * S), int((0.5 + alto / 2) * S), int(y_brazo * S)
    r = int(radio * S)
    punta = int(0.92 * S)
    base = punta - int(cabeza * S)
    cv2.line(img, (xs, yb), (xs, ya + r), 255, t)                       # tallo
    if r > 0:
        cv2.ellipse(img, (xs + r, ya + r), (r, r), 180, 0, 90, 255, t)  # curva (cuarto de circulo)
    cv2.line(img, (xs + r, ya), (base + 2, ya), 255, t)                  # brazo horizontal
    _triangulo(img, base, ya, cabeza * S, ancho_cabeza * S)
    return img


def flecha_curva(grosor=0.14, cabeza=0.28, ancho_cabeza=0.25, abertura=150):
    """Flecha en arco (como 'girar a la derecha' dibujada a mano): arco que termina en punta a la derecha."""
    img = np.zeros((S, S), np.uint8)
    t = max(2, int(grosor * S))
    r = int(0.32 * S)
    c = (int(0.45 * S), int(0.62 * S))
    cv2.ellipse(img, c, (r, r), 0, 180, 180 + abertura, 255, t)
    a = math.radians(180 + abertura)
    ex, ey = c[0] + r * math.cos(a), c[1] + r * math.sin(a)
    tang = a + math.pi / 2                                              # sentido del recorrido del arco
    L, W = cabeza * S, ancho_cabeza * S
    tip = (ex + L * math.cos(tang), ey + L * math.sin(tang))
    n = (-math.sin(tang), math.cos(tang))
    pts = np.array([[ex + W * n[0], ey + W * n[1]], tip, [ex - W * n[0], ey - W * n[1]]], np.int32)
    cv2.fillPoly(img, [pts], 255)
    return img


def chevron(grosor=0.16, abertura=0.38):
    """'>' grueso."""
    img = np.zeros((S, S), np.uint8)
    t = max(2, int(grosor * S))
    pts = np.array([[0.3 * S, (0.5 - abertura) * S], [0.72 * S, 0.5 * S], [0.3 * S, (0.5 + abertura) * S]], np.int32)
    cv2.polylines(img, [pts], False, 255, t)
    return img


def banco_derecha():
    """Plantillas (mascaras SxS) de flechas hacia la DERECHA, de estilos variados.
    -> (lista de mascaras, lista de booleanos 'es recta' para generar las de arriba/abajo)."""
    out = []
    for g in (0.10, 0.18, 0.28):
        for cab, ac in ((0.30, 0.26), (0.40, 0.36), (0.25, 0.40)):
            out.append((flecha_recta(g, cab, max(ac, g * 0.9), 0.85), True))
    for g in (0.10, 0.16, 0.22):
        for r in (0.0, 0.18):
            for alto in (0.65, 0.8):
                out.append((flecha_giro(g, 0.28, max(0.22, g * 1.3), r, alto), False))
    for g in (0.10, 0.16):
        for ab in (120, 170):
            out.append((flecha_curva(g, 0.28, 0.24, ab), False))
    for g in (0.12, 0.20):
        out.append((chevron(g), True))
    return [m for m, _ in out], [r for _, r in out]


def banco_negativo():
    """Formas que NO son flechas pero salen como manchas de tinta: aros, la senal de prohibido
    (aro con barra), barras, cruces, una L o una T sin punta, cuadros de un tablero, letras gruesas."""
    out = []
    c, R = (S // 2, S // 2), int(S * 0.45)
    for g in (0.06, 0.12):
        m = np.zeros((S, S), np.uint8)
        cv2.circle(m, c, R, 255, int(g * S))
        out.append(m)
        for ang in (45, 135):
            mm = m.copy()
            d = R * 0.95
            a = math.radians(ang)
            cv2.line(mm, (int(c[0] - d * math.cos(a)), int(c[1] - d * math.sin(a))),
                     (int(c[0] + d * math.cos(a)), int(c[1] + d * math.sin(a))), 255, int(0.14 * S))
            out.append(mm)
    for ang in (45, 135):                                       # barra diagonal sola
        m = np.zeros((S, S), np.uint8)
        a = math.radians(ang)
        cv2.line(m, (int(c[0] - R * math.cos(a)), int(c[1] - R * math.sin(a))),
                 (int(c[0] + R * math.cos(a)), int(c[1] + R * math.sin(a))), 255, int(0.2 * S))
        out.append(m)
    m = np.zeros((S, S), np.uint8)
    cv2.circle(m, c, R, 255, -1)
    out.append(m)
    for g in (0.15, 0.25):                                      # cruz y X (una L o una T no: a pocos px son una flecha de giro)
        t = int(g * S)
        m = np.zeros((S, S), np.uint8)
        cv2.line(m, (c[0], 10), (c[0], S - 10), 255, t)
        cv2.line(m, (10, c[1]), (S - 10, c[1]), 255, t)
        out.append(m)
        m = np.zeros((S, S), np.uint8)
        cv2.line(m, (10, 10), (S - 10, S - 10), 255, t)
        cv2.line(m, (10, S - 10), (S - 10, 10), 255, t)
        out.append(m)
    m = np.zeros((S, S), np.uint8)                              # cuadros de tablero que se tocan en diagonal
    k = S // 4
    for i in range(4):
        for j in range(4):
            if (i + j) % 2 == 0:
                m[i * k:(i + 1) * k, j * k:(j + 1) * k] = 255
    out.append(m)
    return out


def quitar_aro(comp, n=64):
    """Si la mancha es un aro (senal circular) con algo pegado por dentro, devuelve solo lo de adentro
    (mascara n x n, puede quedar vacia). Si no es un aro, None. Trabaja con la caja estirada a un
    cuadrado: un aro visto de costado (elipse) vuelve a ser un circulo."""
    p = cv2.resize(comp, (n, n), interpolation=cv2.INTER_AREA) > 127
    yy, xx = np.mgrid[:n, :n]
    c = (n - 1) / 2.0
    r = np.hypot(xx - c, yy - c) / c
    ang = np.arctan2(yy - c, xx - c)
    bins = ((ang + math.pi) / (2 * math.pi) * 36).astype(int) % 36
    borde = p & (r > 0.78) & (r <= 1.08)
    if len(np.unique(bins[borde])) < 31:                       # el borde no esta cubierto todo alrededor
        return None
    # grosor del aro: en cada direccion, desde el borde hacia adentro hasta el primer hueco
    rs = np.linspace(1.0, 0.0, 40)
    r_in = []
    for a in np.linspace(-math.pi, math.pi, 36, endpoint=False):
        xs = np.clip((c + rs * c * math.cos(a)).round().astype(int), 0, n - 1)
        ys = np.clip((c + rs * c * math.sin(a)).round().astype(int), 0, n - 1)
        v = p[ys, xs]
        k = np.argmax(v)                                        # primer pixel de tinta (borde exterior)
        hueco = np.nonzero(~v[k:])[0]
        r_in.append(rs[k + hueco[0]] if hueco.size else 0.0)
    lim = float(np.median(r_in)) - 0.06
    if lim <= 0.2:                                              # disco lleno, no aro
        return None
    return ((p & (r < lim)).astype(np.uint8) * 255)


def normalizar(mask):
    """Mascara binaria (o gris 0..255) -> parche NxN float: recortado a su caja y estirado al cuadrado.
    Estirar quita el efecto de ver la senal de costado (perspectiva que la estrecha)."""
    ys, xs = np.nonzero(mask > 127)
    if len(xs) == 0:
        return np.zeros((N, N), np.float32)
    crop = mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
    p = cv2.resize(crop, (N - 4, N - 4), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0
    return cv2.copyMakeBorder(p, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=0.0)


def _vec(p):
    v = cv2.GaussianBlur(p, (3, 3), 0.8).ravel().astype(np.float32)
    v = v - v.mean()
    return v / (np.linalg.norm(v) + 1e-6)


class Banco:
    """Vectores de las plantillas por direccion, listos para correlacionar."""

    def __init__(self):
        der, rectas = banco_derecha()
        P = {DER: [], IZQ: [], ARRIBA: [], ABAJO: []}
        for m, recta in zip(der, rectas):
            P[DER].append(m)
            P[IZQ].append(cv2.flip(m, 1))
            if recta:
                P[ARRIBA].append(cv2.rotate(m, cv2.ROTATE_90_COUNTERCLOCKWISE))
                P[ABAJO].append(cv2.rotate(m, cv2.ROTATE_90_CLOCKWISE))
        self.dirs = list(P)
        self.M = {d: np.array([_vec(normalizar(m)) for m in ms]) for d, ms in P.items()}
        self.neg = np.array([_vec(normalizar(m)) for m in banco_negativo()])

    def puntajes(self, parche):
        """-> ({direccion: parecido}, parecido con la mejor forma que NO es flecha)."""
        v = _vec(parche)
        return {d: float((M @ v).max()) for d, M in self.M.items()}, float((self.neg @ v).max())


# ---------------------------------------------------------------- detector

class DetectorFlechas:
    def __init__(self, umbral=0.68, margen=0.10, margen_neg=0.04, min_lado=9, max_frac=0.7, contraste=0.72,
                 color=False):
        """color=True acepta tambien flechas de color sobre papel blanco (mas falsos positivos con
        objetos de colores del fondo; usar solo si las flechas del circuito son de color)."""
        self.banco = Banco()
        self.color = color
        self.umbral, self.margen, self.margen_neg = umbral, margen, margen_neg
        self.min_lado, self.max_frac = min_lado, max_frac
        self.contraste = contraste            # tinta / papel: la tinta debe tener < 72 % del brillo del papel

    def tinta(self, img):
        m = img.min(axis=2) if img.ndim == 3 else img
        block = max(15, int(0.14 * min(m.shape[:2])) | 1)
        ink = cv2.adaptiveThreshold(m, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, 18)
        return m, cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

    def candidatos(self, img):
        """-> lista de (caja, mascara del componente en la caja) con forma y contexto plausibles."""
        m, ink = self.tinta(img)
        H, W = m.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) if img.ndim == 3 else None
        V = hsv[:, :, 2] if hsv is not None else m
        S = hsv[:, :, 1] if hsv is not None else np.zeros_like(m)
        brillo = float(np.percentile(V, 95))
        tinta_ancha = cv2.dilate(ink, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(ink, connectivity=8)
        out = []
        for i in range(1, n):
            x, y, w, h, area = st[i]
            lado = max(w, h)
            if lado < self.min_lado or lado > self.max_frac * min(H, W) or area < 0.6 * self.min_lado ** 2 / 2:
                continue
            if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:   # cortada por el borde de la imagen
                continue
            if self._textura(i, st, cen):
                continue
            if not (0.25 <= w / h <= 4.0):
                continue
            llenado = area / float(w * h)
            if not (0.15 <= llenado <= 0.8):           # ni un cuadrado lleno ni un hilo
                continue
            comp = (lab[y:y + h, x:x + w] == i).astype(np.uint8) * 255
            if self._papel(x, y, w, h, comp, V, S, tinta_ancha, brillo):
                out.append(((int(x), int(y), int(w), int(h)), comp))
        return out

    def _papel(self, x, y, w, h, comp, V, S, tinta_ancha, brillo):
        """La senal es tinta sobre PAPEL: alrededor de la mancha (sin contar ninguna tinta) el fondo es
        claro para la escena, poco saturado y parejo; la tinta es oscura, o de un color fuerte sobre
        papel blanco. Descarta manchas del fondo del aula, cajas de colores y bordes de paredes."""
        H, W = V.shape
        mx, my = max(2, int(0.15 * w)), max(2, int(0.15 * h))     # franja angosta: el papel puede tener poco margen
        x0, y0, x1, y1 = max(0, x - mx), max(0, y - my), min(W, x + w + mx), min(H, y + h + my)
        libre = tinta_ancha[y0:y1, x0:x1] == 0
        if libre.sum() < 12:
            return False
        pv, ps = V[y0:y1, x0:x1][libre], S[y0:y1, x0:x1][libre]
        papel_v, papel_s = float(np.median(pv)), float(np.median(ps))
        if papel_v < max(60.0, 0.42 * brillo) or papel_s > 60:
            return False
        if (np.abs(pv - papel_v) < 35).mean() < 0.6:   # el "papel" no es parejo: es una escena
            return False
        # el papel rodea la mancha por los 4 lados (un trozo de zocalo o de borde sigue oscuro a los costados)
        zv, zl = V[y0:y1, x0:x1], libre
        ix0, iy0 = x - x0, y - y0
        for tira_v, tira_l in ((zv[:, :ix0], zl[:, :ix0]), (zv[:, ix0 + w:], zl[:, ix0 + w:]),
                               (zv[:iy0, :], zl[:iy0, :]), (zv[iy0 + h:, :], zl[iy0 + h:, :])):
            t = tira_v[tira_l]
            if t.size >= 4 and (t >= 0.8 * papel_v).mean() < 0.5:
                return False
        sel = comp > 0
        tinta_v = float(np.median(V[y:y + h, x:x + w][sel]))
        if tinta_v <= self.contraste * papel_v:
            return True                                # tinta oscura (lo normal: senal impresa en negro)
        tinta_s = float(np.median(S[y:y + h, x:x + w][sel]))
        return self.color and tinta_s >= 100 and papel_s <= 35 and papel_v >= 0.7 * brillo

    @staticmethod
    def _textura(i, st, cen):
        """True si la mancha i tiene al lado otras de tamano y forma parecidos (tablero, letras, rejas): una
        senal es una forma aislada sobre papel. No cuenta la que la encierra (el aro de una senal circular)
        ni las tiras finas que el umbral marca en el borde del papel."""
        x, y, w, h, area = st[i]
        lado = max(w, h)
        cx, cy = cen[i]
        d = np.hypot(cen[:, 0] - cx, cen[:, 1] - cy)
        enc = (st[:, 0] <= x) & (st[:, 1] <= y) & (st[:, 0] + st[:, 2] >= x + w) & (st[:, 1] + st[:, 3] >= y + h)
        parecida = (np.abs(np.log(np.maximum(st[:, 2], 1) / w)) < 0.5) & (np.abs(np.log(np.maximum(st[:, 3], 1) / h)) < 0.5)
        vec = (d < 1.6 * lado) & (st[:, 4] >= 0.3 * area) & (st[:, 4] <= 3.0 * area) & parecida & ~enc
        vec[0] = vec[i] = False
        return int(vec.sum()) >= 3

    def clasificar(self, comp):
        """Mascara del componente -> (direccion, score, margen sobre la opuesta, parecido a una no-flecha).
        None si es un aro vacio."""
        interior = quitar_aro(comp)
        if interior is not None:
            if (interior > 0).sum() < 20:                      # aro sin nada adentro
                return None
            comp = interior
        s, neg = self.banco.puntajes(normalizar(comp))
        d = max(s, key=s.get)
        return d, s[d], s[d] - s[OPUESTA[d]], neg

    def __call__(self, img):
        out = []
        for caja, comp in self.candidatos(img):
            c = self.clasificar(comp)
            if c is None:
                continue
            d, sc, mg, neg = c
            if sc >= self.umbral and mg >= self.margen and sc >= neg + self.margen_neg:
                out.append(Flecha(d, caja, sc, mg))
        out.sort(key=lambda f: -(f.score + f.margen))
        return out


def dibujar(img, flechas, color=(0, 140, 255)):
    for f in flechas:
        x, y, w, h = f.caja
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, f"{f.dir} {f.score:.2f}", (x, max(10, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return img
