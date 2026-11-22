"""
Deteccion de la BANDERA A CUADROS de la meta (Tarea A, 21/11): una hoja con un damero negro y blanco
(ondulado, como la bandera de llegada) pegada en la pared del FONDO del ultimo tramo ("el tubo"); los 4
rostros van en los dos costados (2 y 2).

No se busca el dibujo exacto: se buscan MUCHOS cuadritos negros parecidos entre si, pegados unos a otros
en diagonal, sobre papel claro. Eso no lo tiene nada mas del laboratorio (las letras de las cajas son
pocas y distintas; un QR tiene cuadros de tamanos muy variados y un marco que los une).

  1. Tinta: pixeles mas oscuros que su entorno (umbral adaptativo sobre min(B, G, R)).
  2. Cerrando la mascara (7x7, 11x11 y 15x15) los cuadros se unen en UNA mancha: la caja del dibujo entero. De
     tinta tiene entre el 20 y el 70 %.
  3. Dentro de esa caja la tinta y el papel ALTERNAN muchas veces a lo largo (>= 4 cambios por fila) Y a lo
     alto (>= 3 por columna), parecido en las dos direcciones (un texto alterna mucho a lo largo y poco a lo
     alto), y las corridas son parejas (rango intercuartil / mediana <= 0.7: los cuadros son del mismo tamano;
     un QR o unas letras tienen corridas de todos los largos). El papel entre los cuadros es claro y poco
     saturado. Probado con la foto del laboratorio (21/11) de 28 a 150 px de ancho, de frente y de costado, y
     sin falsos positivos con QR de 40-300 px ni textos de cajas.
  `n` es el numero de columnas de cuadros que se cuentan (~ (cambios por fila + 1) / 2).

Uso:
    det = DetectorBandera()
    for b in det(img_bgr):          # de mayor a menor numero de cuadros
        print(b.caja, b.n)
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Bandera:
    caja: tuple             # (x, y, w, h) en px de la imagen: la caja de todos los cuadros
    n: int                  # cuadritos encontrados
    score: float            # 0..1: cuan parejos son los cuadros (1 = todos del mismo tamano)
    carton: float = 1.0     # 0..1: cuanto del borde de la hoja es carton (la de verdad va sobre una caja)

    @property
    def centro(self):
        x, y, w, h = self.caja
        return x + w / 2.0, y + h / 2.0

    @property
    def ancho(self):
        return float(self.caja[2])


class DetectorBandera:
    def __init__(self, min_lado=3, sobre_carton=False, min_carton=0.35):
        self.min_lado = min_lado
        self.sobre_carton = sobre_carton    # (22/11) la de verdad es una hoja blanca pegada en una caja de carton
        self.min_carton = min_carton

    @staticmethod
    def carton_alrededor(img, caja):
        """Fraccion del borde alrededor de la HOJA (la zona clara que contiene la caja de la bandera) que es
        carton: tono naranja-marron (H 8-30 en OpenCV) con algo de color (S >= 18). Medido en los videos del
        21/11: el carton alrededor da H 17-21, S 28-62; el piso y las paredes grises, S < 15."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        Hh, Ww = hsv.shape[:2]
        X, Y, w, h = caja
        V, S = hsv[:, :, 2], hsv[:, :, 1]
        ref = float(np.percentile(V[Y:Y + h, X:X + w], 90))
        papel = ((V >= 0.8 * ref) & (S < 60)).astype(np.uint8) * 255
        papel = cv2.morphologyEx(papel, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        papel[Y:Y + h, X:X + w] = 255
        n, lab, st, _ = cv2.connectedComponentsWithStats(papel, connectivity=8)
        i = lab[Y + h // 2, X + w // 2]
        if i == 0 or st[i, 2] > 0.9 * Ww or st[i, 3] > 0.9 * Hh:
            return 0.0                                      # la "hoja" se derrama por toda la imagen: no es una hoja
        hoja = (lab == i).astype(np.uint8) * 255
        k = max(5, int(0.06 * max(st[i, 2], st[i, 3])) | 1)
        anillo = (cv2.dilate(hoja, np.ones((3 * k, 3 * k), np.uint8)) > 0) & (cv2.dilate(hoja, np.ones((k, k), np.uint8)) == 0)
        if anillo.sum() < 10:
            return 0.0
        hh, ss = hsv[:, :, 0][anillo], S[anillo]
        return float(((hh >= 8) & (hh <= 30) & (ss >= 18)).mean())

    @staticmethod
    def tinta(img):
        # en gris (no el canal minimo como las flechas: la bandera es negra sobre blanco, y con el canal minimo
        # una caja o un piso de color quedaban "oscuros" al lado de la hoja y se pegaban a los cuadros)
        m = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        block = max(15, int(0.14 * min(m.shape[:2])) | 1)
        ink = cv2.adaptiveThreshold(m, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, 18)
        # ...y ademas bastante mas oscuro que lo mas claro de alrededor (el papel): el fondo gris pegado a una
        # hoja blanca tambien queda bajo la media local, pero no a la mitad de su brillo
        claro = cv2.dilate(m, np.ones((block, block), np.uint8))
        ink[m >= 0.5 * claro] = 0
        # ...y sobre una HOJA: dentro de una zona clara (>= 82 % de lo mas brillante de la escena) con sus huecos
        # rellenos (los cuadros). Las letras de una caja pegada a la hoja quedan afuera y no se mezclan
        brillo = float(np.percentile(m, 99))
        papel = (m >= 0.82 * brillo).astype(np.uint8) * 255
        papel = cv2.morphologyEx(papel, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        H, W = m.shape
        # una hoja es una zona clara ACOTADA (el piso o el cielo claros no: ocupan media imagen)
        n, lab, st, _ = cv2.connectedComponentsWithStats(papel, connectivity=8)
        grandes = np.nonzero((st[:, 4] > 0.2 * H * W) | (st[:, 2] > 0.8 * W) | (st[:, 3] > 0.8 * H))[0]
        grandes = grandes[grandes > 0]
        if grandes.size:
            papel[np.isin(lab, grandes)] = 0
        fondo = np.zeros((H + 2, W + 2), np.uint8)
        inv = cv2.bitwise_not(papel)
        cv2.floodFill(inv, fondo, (0, 0), 0)                    # lo no-papel que toca el borde se va; quedan los
        papel = cv2.bitwise_or(papel, inv)                      # huecos encerrados: se rellenan
        ink[cv2.dilate(papel, np.ones((3, 3), np.uint8)) == 0] = 0
        return m, ink

    @staticmethod
    def _corridas(mask):
        """Longitudes de las corridas (tinta y papel) a lo largo de las filas de `mask` (bool), y transiciones
        por fila."""
        d = np.diff(mask.astype(np.int8), axis=1)
        trans = (d != 0).sum(axis=1)
        corridas = []
        for fila in mask:
            idx = np.flatnonzero(np.diff(fila.astype(np.int8)))
            if idx.size >= 2:
                corridas.extend(np.diff(idx).tolist())           # corridas interiores (las de los bordes no)
        return trans, np.array(corridas, float)

    def __call__(self, img):
        m, ink = self.tinta(img)
        H, W = m.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) if img.ndim == 3 else None
        V = hsv[:, :, 2] if hsv is not None else m
        S = hsv[:, :, 1] if hsv is not None else None
        brillo = float(np.percentile(V, 95))
        # fuera las manchas grandes y ralas (bordes del papel, de las cajas, zocalos): si no, al cerrar la
        # mascara la bandera se pega al marco de su hoja y de ahi al resto de la escena
        n0, lab0, st0, _ = cv2.connectedComponentsWithStats(ink, connectivity=8)
        if n0 > 1:
            lado0 = np.maximum(st0[1:, 2], st0[1:, 3])
            llen0 = st0[1:, 4] / np.maximum(st0[1:, 2] * st0[1:, 3], 1).astype(float)
            fuera = np.nonzero((llen0 < 0.12) | (lado0 > 0.5 * min(H, W)))[0] + 1
            if fuera.size:
                ink = np.where(np.isin(lab0, fuera), 0, ink).astype(np.uint8)
        out, vistas = [], []
        # los cuadros se unen en UNA mancha cerrando la mascara; dos tamanos de cierre: de lejos (cuadros de
        # 3-6 px) alcanza 7x7, de cerca (cuadros de 10-25 px) hace falta 15x15
        for k in (7, 11, 15):
            cerrada = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
            n, lab, st, _ = cv2.connectedComponentsWithStats(cerrada, connectivity=8)
            for i in range(1, n):
                x, y, w, h, area = (int(v) for v in st[i])
                if w < 20 or h < 10 or max(w, h) > 0.9 * min(H, W):
                    continue                                     # (la hoja A4 a 2.5 m mide ~40 x 28 px)
                if x <= 0 or y <= 0 or x + w >= W - 1 or y + h >= H - 1:
                    continue                                     # cortada por el borde: se vera mejor despues
                if any(abs(x - a) < 4 and abs(y - b) < 4 and abs(w - c) < 6 and abs(h - d) < 6 for a, b, c, d in vistas):
                    continue
                comp = lab[y:y + h, x:x + w] == i
                tinta = (ink[y:y + h, x:x + w] > 0) & comp
                frac = float(tinta.mean())
                if not (0.15 <= frac <= 0.42):                   # ondulado, de tinta tiene un 22-34 % de su caja
                    continue                                     # (un QR, 43-70 %)
                # transiciones tinta/papel a lo largo y a lo alto: muchas, y en las DOS direcciones (un texto
                # alterna mucho a lo largo y poco a lo alto; una raya, solo en una)
                tr_f, cor_f = self._corridas(tinta)
                tr_c, cor_c = self._corridas(tinta.T)
                filas = tr_f[tr_f >= 2]
                cols = tr_c[tr_c >= 2]
                if len(filas) < 0.5 * h or len(cols) < 0.5 * w:
                    continue
                n_f, n_c = float(np.median(filas)), float(np.median(cols))
                if n_f < 5 or n_c < 4 or n_c < 0.4 * n_f or n_f < 0.4 * n_c:
                    continue                                     # (la impresa: 6-8 por fila, 4-6 por columna)
                # corridas parejas (cuadros del mismo tamano): un QR o unas letras las tienen de todos los largos.
                # Medida robusta: (cuartil 3 - cuartil 1) / mediana (la bandera real, ondulada: 0.2-0.67; un QR
                # o un texto, >= 0.7). El `cv` recortado (sin corridas > 4 medianas) va en el score
                cor = np.concatenate([cor_f, cor_c])
                if cor.size < 20:
                    continue
                med = float(np.median(cor))
                q1, q3 = np.percentile(cor, [25, 75])
                if (q3 - q1) / max(med, 1e-6) > 0.7:
                    continue
                c2 = cor[cor <= 4 * med]
                cv = float(np.std(c2) / max(np.mean(c2), 1e-6))
                if cv > 0.7:
                    continue
                zona = ~tinta
                v = V[y:y + h, x:x + w][zona]
                if v.size < 10 or float(np.median(v)) < max(60.0, 0.72 * brillo):
                    continue                                     # el "papel" no es claro (carton con letras: no)
                if S is not None and float(np.median(S[y:y + h, x:x + w][zona])) > 70:
                    continue
                vistas.append((x, y, w, h))
                out.append(Bandera((x, y, w, h), int(round((n_f + 1) / 2)), float(np.clip(1.0 - cv, 0.0, 1.0))))
        if img.ndim == 3:                                   # confianza, no requisito (sobre_carton=True: requisito)
            for b in out:
                b.carton = self.carton_alrededor(img, b.caja)
            if self.sobre_carton:
                out = [b for b in out if b.carton >= self.min_carton]
        out.sort(key=lambda b: -(b.n * b.score))
        return out


def dibujar(img, banderas, color=(255, 200, 0)):
    for b in banderas:
        x, y, w, h = b.caja
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, f"meta {b.n}", (x, max(10, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
