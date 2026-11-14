"""
Simulador del laberinto con senales de color y QR (Tarea A, laberinto-1). Extiende el de auto-1:

  - senales azules (ida) y rojas (vuelta) pegadas a las paredes: la camara las ve como manchas de color
  - cada foto (cara) lleva debajo un codigo QR REAL con su mensaje (se genera con cv2.QRCodeEncoder)
  - photo(hd=True) devuelve 1280x720 (como el robot con el puente HD); sin hd, 250x250
  - la "cara" se dibuja como un recuadro de gris 250 (el detector del simulador solo mira ese gris)

Mundos: lab_desvios, lab_camino, lab_laberinto (perfecto, 6x6 con callejones), lab_serpiente.
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np

AUTO1 = Path(__file__).resolve().parents[1] / "auto-1"
sys.path.insert(0, str(AUTO1))
sys.path.insert(0, str(AUTO1.parents[1]))
import sim as base                                      # noqa: E402

GRIS_CARA = 250


def _mensajes(n):
    return [f"MENSAJE F{i + 1}: la clave de la persona {i + 1} es {1000 + 37 * i}" for i in range(n)]


def _qr(texto, lado_px):
    """QR como imagen BGR cuadrada con borde blanco, de `lado_px` de lado."""
    enc = cv2.QRCodeEncoder.create()
    q = enc.encode(texto)
    q = cv2.copyMakeBorder(q, 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=255)
    q = cv2.resize(q, (max(int(lado_px), 8),) * 2, interpolation=cv2.INTER_NEAREST)
    return cv2.cvtColor(q, cv2.COLOR_GRAY2BGR) if q.ndim == 2 else q


# senales: (x, y, normal, color). normal = hacia donde mira la senal (hacia el pasillo)
SENALES = {
    "lab_desvios": [(2.8, 0.52, math.pi / 2, "azul"), (2.07, 3.0, 0.0, "azul"), (0.52, 4.6, 0.0, "azul"),
                    (5.48, 2.0, math.pi, "rojo"), (5.48, 4.5, math.pi, "rojo")],
    "lab_camino": [(5.48, 2.0, math.pi, "azul"), (3.5, 2.97, -math.pi / 2, "azul"), (2.0, 5.0, 0.0, "azul"),
                   (0.52, 2.0, 0.0, "rojo")],
    "lab_serpiente": [(2.0, 1.55, -math.pi / 2, "azul"), (4.0, 2.95, -math.pi / 2, "azul"),
                      (2.0, 4.15, -math.pi / 2, "azul")],
    "lab_laberinto": [],
}
BASE = {"lab_desvios": base.pista_desvios_world, "lab_camino": base.pista_camino_world,
        "lab_serpiente": base.pista_serpiente_world, "lab_laberinto": base.pista_laberinto_world}
MUNDOS = tuple(BASE)


class SimDetectorLab:
    """Detector de caras del simulador: busca el recuadro de gris GRIS_CARA (el QR y las senales no cuentan)."""

    def __init__(self, min_width=4):
        self.min_width = min_width

    def faces(self, img):
        m = np.all(np.abs(img.astype(int) - GRIS_CARA) <= 1, axis=2).astype(np.uint8)
        n, _, st, _ = cv2.connectedComponentsWithStats(m)
        return [(float(st[i, 0]), float(st[i, 1]), float(st[i, 2]), float(st[i, 3]), 0.95)
                for i in range(1, n) if st[i, 2] >= self.min_width]

    def __call__(self, img):
        return len(self.faces(img)) > 0


class SimLab(base.SimRobot):
    def __init__(self, world="lab_desvios", **kw):
        senales = None
        if isinstance(world, str):
            senales = SENALES.get(world, [])
            world = BASE[world]()
        super().__init__(world=world, **kw)
        self.senales = list(senales or [])
        self.mensajes = _mensajes(len(self.faces))
        self.qr_visto = 0                                        # fotos HD en las que habia un QR dentro del cono

    def photo(self, timeout=4.0, quality=85, hd=False):
        if not self.camera_ok:
            raise RuntimeError("sin imagen de la camara (simulada caida)")
        self.photos_taken += 1
        W, H = (1280, 720) if hd else (250, 250)
        self.face_visible()
        img = np.full((H, W, 3), 90, np.uint8)                   # pared gris oscuro
        f = (W / 2) / math.tan(self.cam_fov / 2)
        cy = H / 2
        for fx, fy, normal, color in self.senales:               # senales de color
            if not self._face_visible(fx, fy, normal):
                continue
            d = math.hypot(fx - self.wx, fy - self.wy)
            rel = self._rel(fx, fy)
            cx, lado = W / 2 - f * math.tan(rel), f * 0.18 / d
            x0, y0 = int(round(cx - lado / 2)), int(round(cy - lado / 2))
            img[max(y0, 0):max(y0 + int(lado), 0), max(x0, 0):max(x0 + int(lado), 0)] = (255, 0, 0) if color == "azul" else (0, 0, 255)
        for i, rel, d in self.visible_faces():                   # cara (recuadro gris) + QR debajo
            cx = W / 2 - f * math.tan(rel)
            w = max(4.0, f * 0.12 / d)
            x0, x1 = int(round(cx - w / 2)), int(round(cx + w / 2))
            y0, y1 = int(round(cy - 0.6 * w)), int(round(cy + 0.6 * w))
            img[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)] = GRIS_CARA
            lado = f * 0.07 / d                                  # QR de 7 cm, 13 cm por debajo del centro de la cara
            qy = int(round(cy + f * 0.13 / d - lado / 2))
            qx = int(round(cx - lado / 2))
            if lado >= 8 and 0 <= qx and qx + int(lado) <= W and 0 <= qy and qy + int(lado) <= H:
                q = _qr(self.mensajes[i], lado)
                img[qy:qy + q.shape[0], qx:qx + q.shape[1]] = q
                self.qr_visto += 1
        return img

    def _rel(self, x, y):
        ang = math.atan2(y - self.wy, x - self.wx)
        return math.atan2(math.sin(ang - self.wth), math.cos(ang - self.wth))

    def render_world(self, px_per_m=80):
        img = super().render_world(px_per_m)
        h = img.shape[0] - 26
        for x, y, normal, color in self.senales:
            p = (int(x * px_per_m), 26 + h - 1 - int(y * px_per_m))
            cv2.circle(img, p, 6, (255, 0, 0) if color == "azul" else (0, 0, 255), -1)
        return img
