import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
from qr import FaseQR, leer_qr                      # noqa: E402
from senales import RegistroSenales, detectar_senales  # noqa: E402


def mancha(color):
    img = np.full((250, 250, 3), 90, np.uint8)
    img[100:150, 100:150] = {"azul": (255, 0, 0), "rojo": (0, 0, 255)}[color]
    return img


class Senales(unittest.TestCase):
    def test_colores(self):
        self.assertEqual([s["color"] for s in detectar_senales(mancha("azul"))], ["azul"])
        self.assertEqual([s["color"] for s in detectar_senales(mancha("rojo"))], ["rojo"])

    def test_pared_lisa_no_es_senal(self):
        self.assertEqual(detectar_senales(np.full((250, 250, 3), 90, np.uint8)), [])

    def test_confirmacion_y_ultima_azul(self):
        r = RegistroSenales(need=2)
        self.assertFalse(r.agregar("azul", 1, 1, 0)[1])
        self.assertTrue(r.agregar("azul", 1.1, 1, 0)[1])
        self.assertEqual(r.ultima_azul()["orden"], 1)
        r.agregar("rojo", 5, 5, 0)
        self.assertEqual(r.confirmadas("rojo"), [])


class QR(unittest.TestCase):
    def test_lee_y_fase_termina(self):
        import cv2
        q = cv2.QRCodeEncoder.create().encode("hola")
        img = cv2.cvtColor(cv2.copyMakeBorder(cv2.resize(q, (200, 200), interpolation=cv2.INTER_NEAREST), 20, 20, 20, 20,
                                              cv2.BORDER_CONSTANT, value=255), cv2.COLOR_GRAY2BGR)
        self.assertEqual(leer_qr(img), "hola")
        f = FaseQR()
        f.step(0.0, img, (0, 0, 0), None, 2.0)
        self.assertTrue(f.ok and f.terminada)

    def test_sin_qr_se_rinde(self):
        f, t = FaseQR(t_quieto=0.1, t_giro=0.1, t_avance=0.2, t_final=0.1), 0.0
        while not f.terminada and t < 60:
            f.step(t, None, (0, 0, 0), None, 2.0)
            t += 0.1
        self.assertTrue(f.terminada and not f.ok)


if __name__ == "__main__":
    unittest.main()
