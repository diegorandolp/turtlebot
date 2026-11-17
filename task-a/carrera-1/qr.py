"""
Lectura de los QR (Tarea A, carrera-1): en la meta hay varias fotos y cada una tiene un QR en una esquina
de abajo. Hay que leer el de la persona buscada y decir la palabra al volver a A.

    buscar_qrs(img, caras) -> [QR(texto, caja, cara)]

Lee el recorte de la imagen donde pueden estar los QR (debajo y a los costados de cada cara; sin caras,
la imagen entera). Con opencv-contrib-python usa el lector de WeChat (zxing), que lee QR de ~2 px por
modulo y lee mas lejos (ver la tabla del README). Sin el, el de OpenCV con algunas variantes.
`cara` es el indice de la cara (en `caras`) a la que pertenece el QR (ver _de_que_cara), o None.

Basado en el lector de Sofia (task-a/laberinto-1/qr.py, rama manzana-zanabu), que probaba variantes de la
imagen. Aca se agrega WeChat, leer solo donde puede haber QR y saber de que foto es cada QR.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class QR:
    texto: str
    caja: tuple            # (x, y, w, h) en la imagen completa
    cara: int | None       # indice de la cara de la que es el QR (None = no habia caras)


_det = None
_aruco = None
_wechat = None


def _detectores():
    global _det, _aruco
    if _det is None:
        _det = cv2.QRCodeDetector()
        _aruco = cv2.QRCodeDetectorAruco() if hasattr(cv2, "QRCodeDetectorAruco") else None
    return [d for d in (_det, _aruco) if d is not None]


def _lector_wechat():
    """Lector de QR de WeChat (zxing): lee QR de ~2 px por modulo, donde el de OpenCV falla. Viene en
    opencv-contrib-python (no en opencv-python). None si no esta."""
    global _wechat
    if _wechat is None:
        try:
            _wechat = cv2.wechat_qrcode_WeChatQRCode()
        except (AttributeError, cv2.error):
            _wechat = False
    return _wechat or None


def lector():
    """'wechat' (opencv-contrib-python, lee desde mas lejos) u 'opencv'."""
    return "wechat" if _lector_wechat() is not None else "opencv"


def _caja(p):
    p = np.asarray(p, float).reshape(-1, 2)
    bx, by = p[:, 0].min(), p[:, 1].min()
    return int(bx), int(by), int(p[:, 0].max() - bx), int(p[:, 1].max() - by)


def _opencv(gris, multi):
    """Lector de OpenCV (el normal y el Aruco): todos los QR (multi) o uno. -> [(texto, caja)]"""
    out = []
    for d in _detectores():
        try:
            if multi:
                ok, textos, puntos, _ = d.detectAndDecodeMulti(gris)
                if ok and puntos is not None:
                    out += [(t, _caja(p)) for t, p in zip(textos, puntos) if t]
            else:
                t, p, _ = d.detectAndDecode(gris)
                if t and p is not None:
                    out.append((t, _caja(p)))
        except cv2.error:
            continue
        if out and not multi:
            break
    return out


def _wechat_lee(gris):
    w = _lector_wechat()
    if w is None:
        return []
    try:
        textos, puntos = w.detectAndDecode(gris)
    except cv2.error:
        return []
    return [(t, _caja(p)) for t, p in zip(textos, puntos) if t]


def _zona(caja, W, H):
    """Region de la imagen donde puede estar el QR de la foto de esta cara: debajo (hasta el borde de abajo:
    en una foto de cuerpo entero el QR queda muy por debajo de la cara) y a los costados."""
    x, y, w, h = caja
    x0, x1 = int(x - 3.0 * w), int(x + 4.0 * w)
    y0 = int(y + 0.3 * h)
    return max(0, x0), max(0, y0), min(W, x1), H


def buscar_qrs(img, cajas_caras=()):
    """img BGR; cajas_caras = [(x, y, w, h)] de las caras detectadas. -> [QR] sin repetidos.

    Medido en el simulador (QR de 10 cm, foto HD, ~/pruebas): lo que mas lee es el lector de OpenCV de a UN
    QR por zona (debajo y a los costados de cada cara: 97 % a 0.6 m, 78 % a 1 m) junto con WeChat sobre la
    imagen entera (91 % / 56 %, y el unico que lee algo a 1.5 m). Leer todos de una vez (multi) o WeChat
    sobre un recorte lee bastante menos. Las zonas chicas (preview 250x250) se amplian x2."""
    if img is None or img.size == 0:
        return []
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    H, W = gris.shape[:2]
    vistos = {}

    def agregar(lista, dx=0, dy=0, k=1.0):
        for t, (bx, by, bw, bh) in lista:
            vistos.setdefault(t, (int(bx / k) + dx, int(by / k) + dy, int(bw / k), int(bh / k)))

    zonas = [z for z in (_zona(c, W, H) for c in cajas_caras) if z[2] - z[0] >= 20 and z[3] - z[1] >= 20]
    for x0, y0, x1, y1 in zonas:
        z = gris[y0:y1, x0:x1]
        if max(z.shape) < 450:                          # recorte chico (preview): ampliar ayuda
            agregar(_opencv(cv2.resize(z, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC), False), x0, y0, 2.0)
        else:
            agregar(_opencv(z, False), x0, y0)
    agregar(_wechat_lee(gris))
    if not zonas and not vistos:                         # sin caras: la imagen entera con OpenCV
        agregar(_opencv(gris, True))
    return [QR(t, c, _de_que_cara(c, cajas_caras)) for t, c in vistos.items()]


def _de_que_cara(caja_qr, cajas_caras):
    """La cara (de la misma imagen) de cuya foto es el QR, o None si no se puede asegurar.

    Las fotos de la meta estan en fila, a la misma altura: lo que separa el QR propio del de la vecina es la
    distancia HORIZONTAL al centro de la cara, medida en anchos de cara `w`. En un retrato la hoja mide ~3 w
    y el QR de 10 cm (~1.4 w) queda con su centro a ~0.8 w a un costado; en una foto de cuerpo entero la
    cara es mas chica respecto a la hoja (~5 w) y el QR queda a ~1.6 w. El QR de la foto vecina, aunque
    esten casi pegadas, queda a >= 2.5 w. Se asigna si el QR esta debajo del centro de la cara (>= 0.35 w: en un primer plano la esquina de la hoja se monta sobre el menton) y a <= 2.0 w
    de lado; si dos caras cumplen, la mas centrada sobre el QR."""
    if not cajas_caras:
        return None
    qx, qy = caja_qr[0] + caja_qr[2] / 2, caja_qr[1] + caja_qr[3] / 2
    mejor, dx_mejor = None, 1e9
    for i, (x, y, w, h) in enumerate(cajas_caras):
        w = max(float(w), 1.0)
        dx = abs(qx - (x + w / 2)) / w
        dy = (qy - (y + h / 2)) / w
        if dx <= 2.0 and dy >= 0.35 and dx < dx_mejor:    # 0.35: en un primer plano el QR se monta sobre la cara
            mejor, dx_mejor = i, dx
    return mejor
