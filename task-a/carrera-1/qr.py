"""
Lectura de los QR (Tarea A, carrera-1): en la meta hay varias fotos y cada una tiene un QR en una esquina
de abajo. Hay que leer el de la persona buscada y decir la palabra al volver a A.

    buscar_qrs(img, caras) -> [QR(texto, caja, cara)]

Busca primero cerca de cada cara (debajo y a los costados: ahi esta el QR de esa foto), con varias
variantes de la imagen (tal cual, gris con contraste, ampliada x2/x3 si es chica), y si no hay caras en la
foto, en la imagen entera. `cara` es el indice de la cara (en `caras`) a la que pertenece el QR: la de
cuya esquina de abajo esta mas cerca, o None si no hay caras.

Basado en el lector de Sofia (task-a/laberinto-1/qr.py, rama manzana-zanabu), que probaba las mismas
variantes. Aca se agrega la busqueda por zonas y saber de que foto es cada QR (la meta tiene varias).
"""
from dataclasses import dataclass

import cv2
import numpy as np

_det = None
_aruco = None


@dataclass
class QR:
    texto: str
    caja: tuple            # (x, y, w, h) en la imagen completa
    cara: int | None       # indice de la cara de la que es el QR (None = no habia caras)


def _detectores():
    global _det, _aruco
    if _det is None:
        _det = cv2.QRCodeDetector()
        _aruco = cv2.QRCodeDetectorAruco() if hasattr(cv2, "QRCodeDetectorAruco") else None
    return [d for d in (_det, _aruco) if d is not None]


def _decodificar(img):
    """-> [(texto, puntos 4x2 en coordenadas de `img`)] de todos los QR que se puedan leer."""
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    variantes = [(gris, 1.0), (cv2.equalizeHist(gris), 1.0)]
    if max(gris.shape[:2]) < 700:
        variantes += [(cv2.resize(gris, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC), float(k)) for k in (2, 3)]
    for v, k in variantes:
        for d in _detectores():
            try:
                ok, textos, puntos, _ = d.detectAndDecodeMulti(v)
            except cv2.error:
                continue
            if ok and puntos is not None:
                out = [(t, np.asarray(p, float) / k) for t, p in zip(textos, puntos) if t]
                if out:
                    return out
    return []


def _zona(caja, W, H):
    """Region de la imagen donde puede estar el QR de la foto de esta cara: debajo (hasta el borde de abajo:
    en una foto de cuerpo entero el QR queda muy por debajo de la cara) y a los costados."""
    x, y, w, h = caja
    x0, x1 = int(x - 3.0 * w), int(x + 4.0 * w)
    y0 = int(y + 0.3 * h)
    return max(0, x0), max(0, y0), min(W, x1), H


def buscar_qrs(img, cajas_caras=()):
    """img BGR; cajas_caras = [(x, y, w, h)] de las caras detectadas. -> [QR] sin repetidos."""
    if img is None or img.size == 0:
        return []
    H, W = img.shape[:2]
    zonas = [_zona(c, W, H) for c in cajas_caras] or [(0, 0, W, H)]
    vistos = {}
    for x0, y0, x1, y1 in zonas:
        if x1 - x0 < 20 or y1 - y0 < 20:
            continue
        for texto, p in _decodificar(img[y0:y1, x0:x1]):
            if texto in vistos:
                continue
            p = p + np.array([x0, y0])
            bx, by = p[:, 0].min(), p[:, 1].min()
            vistos[texto] = (int(bx), int(by), int(p[:, 0].max() - bx), int(p[:, 1].max() - by))
    if not vistos and cajas_caras:                         # el QR puede estar fuera de las zonas (cara cortada)
        for texto, p in _decodificar(img):
            bx, by = p[:, 0].min(), p[:, 1].min()
            vistos.setdefault(texto, (int(bx), int(by), int(p[:, 0].max() - bx), int(p[:, 1].max() - by)))
    return [QR(t, c, _de_que_cara(c, cajas_caras)) for t, c in vistos.items()]


def _de_que_cara(caja_qr, cajas_caras):
    """La cara (de la misma imagen) de cuya foto es el QR, o None si no se puede asegurar.

    Las fotos de la meta estan en fila, a la misma altura: lo que separa el QR propio del de la vecina es la
    distancia HORIZONTAL al centro de la cara, medida en anchos de cara `w`. En un retrato la hoja mide ~3 w
    y el QR de 10 cm (~1.4 w) queda con su centro a ~0.8 w a un costado; en una foto de cuerpo entero la
    cara es mas chica respecto a la hoja (~5 w) y el QR queda a ~1.6 w. El QR de la foto vecina, aunque
    esten casi pegadas, queda a >= 2.5 w. Se asigna si el QR esta debajo de la cara (>= 0.6 w) y a <= 2.0 w
    de lado; si dos caras cumplen, la mas centrada sobre el QR."""
    if not cajas_caras:
        return None
    qx, qy = caja_qr[0] + caja_qr[2] / 2, caja_qr[1] + caja_qr[3] / 2
    mejor, dx_mejor = None, 1e9
    for i, (x, y, w, h) in enumerate(cajas_caras):
        w = max(float(w), 1.0)
        dx = abs(qx - (x + w / 2)) / w
        dy = (qy - (y + h / 2)) / w
        if dx <= 2.0 and dy >= 0.6 and dx < dx_mejor:
            mejor, dx_mejor = i, dx
    return mejor
