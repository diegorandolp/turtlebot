"""
Senales de color del laberinto (Tarea A, laberinto-1).

Solo cuenta el COLOR (no la forma de la flecha):
  azul = camino de IDA (hay que seguir por ahi)      rojo = camino de VUELTA (se anota, no hace falta para ir)

detectar_senales(img) -> [dict(color, caja=(x, y, w, h), area)] con las manchas azules/rojas de la foto.
RegistroSenales las ubica en el mapa (direccion por la imagen, distancia por el LiDAR) y exige varias
detecciones para confirmarlas, asi un reflejo suelto no cambia el rumbo del robot.
"""
import math

import cv2
import numpy as np

# HSV de OpenCV: H en 0..179. Azul ~ 100-130, rojo cruza el 0 (0-10 y 170-179).
RANGOS = {
    "azul": [((98, 110, 50), (132, 255, 255))],
    "rojo": [((0, 120, 70), (9, 255, 255)), ((171, 120, 70), (179, 255, 255))],
}


def detectar_senales(img, min_frac=0.0006, aspecto=(0.4, 2.5)):
    """Manchas de color grandes y compactas. `min_frac` = area minima como fraccion de la imagen
    (0.0006 = 37 px en 250x250); el aspecto descarta tiras (bordes, cables)."""
    if img is None or img.size == 0:
        return []
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    h, w = img.shape[:2]
    out = []
    for color, rangos in RANGOS.items():
        m = np.zeros((h, w), np.uint8)
        for lo, hi in rangos:
            m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))     # une la flecha negra interior
        n, _, st, _ = cv2.connectedComponentsWithStats(m)
        for i in range(1, n):
            x, y, bw, bh, area = (int(v) for v in st[i])
            if area < min_frac * w * h or not aspecto[0] <= bw / max(bh, 1) <= aspecto[1]:
                continue
            if area < 0.4 * bw * bh:                                          # mancha poco "llena"
                continue
            out.append(dict(color=color, caja=(float(x), float(y), float(bw), float(bh)), area=area))
    return out


class RegistroSenales:
    """Senales apuntadas en el mapa. Una senal se CONFIRMA a la `need`-esima deteccion cerca del mismo sitio."""

    def __init__(self, fusion=0.6, need=2):
        self.items = []                          # dicts: color, x, y, normal, vistas, orden
        self.fusion, self.need = fusion, need
        self._orden = 0

    def agregar(self, color, fx, fy, normal):
        """-> (senal, recien_confirmada)."""
        for s in self.items:
            if s["color"] == color and math.hypot(s["x"] - fx, s["y"] - fy) < self.fusion:
                k = s["vistas"]
                s["x"], s["y"] = (s["x"] * k + fx) / (k + 1), (s["y"] * k + fy) / (k + 1)
                s["vistas"] = k + 1
                if k + 1 == self.need:
                    self._orden += 1
                    s["orden"] = self._orden
                    return s, True
                return s, False
        s = dict(color=color, x=fx, y=fy, normal=normal, vistas=1, orden=None)
        self.items.append(s)
        return s, False

    def confirmadas(self, color=None):
        return [s for s in self.items if s["vistas"] >= self.need and (color is None or s["color"] == color)]

    def ultima_azul(self):
        """La senal azul confirmada mas reciente (la que marca por donde seguir) o None."""
        az = self.confirmadas("azul")
        return max(az, key=lambda s: s["orden"]) if az else None
