"""Pruebas del grabador de video (sin robot): duracion en tiempo de la carrera, saltos del reloj, cambio de
tamano de la ventana y que un error al grabar no tumbe nada."""
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from video import Grabador, _encajar  # noqa: E402


def n_cuadros(ruta):
    cap = cv2.VideoCapture(str(ruta))
    n = 0
    while cap.read()[0]:
        n += 1
    cap.release()
    return n


class TestGrabador(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_duracion_en_tiempo_de_carrera(self):
        """10 s de carrera a 10 fps = 100 cuadros, aunque se ofrezcan 400 (el bucle va a 40 Hz)."""
        g = Grabador(self.dir / "a.mp4", fps=10, bloquear=True, h264=False)
        img = np.zeros((120, 160, 3), np.uint8)
        for k in range(400):
            t = k * 0.025
            if g.toca(t):
                g.agregar(img, t)
        ruta = g.cerrar()
        self.assertIsNotNone(ruta)
        self.assertEqual(n_cuadros(ruta), 100)

    def test_salto_del_reloj_repite_cuadros(self):
        """Si el bucle se traba 2 s, el video repite el cuadro: sigue en tiempo real."""
        g = Grabador(self.dir / "b.mp4", fps=10, bloquear=True, h264=False)
        img = np.zeros((120, 160, 3), np.uint8)
        g.agregar(img, 0.0)
        g.agregar(img, 2.0)
        self.assertEqual(n_cuadros(g.cerrar()), 21)

    def test_sostener_final(self):
        g = Grabador(self.dir / "c.mp4", fps=10, bloquear=True, h264=False)
        g.agregar(np.zeros((100, 100, 3), np.uint8), 0.0)
        g.sostener(2.0)
        self.assertEqual(n_cuadros(g.cerrar()), 21)

    def test_cambio_de_tamano_y_gris(self):
        g = Grabador(self.dir / "d.mp4", fps=10, bloquear=True, h264=False)
        g.agregar(np.zeros((101, 161, 3), np.uint8), 0.0)               # lados impares -> pares
        g.agregar(np.zeros((300, 200, 3), np.uint8), 0.1)               # otro tamano -> se encaja
        g.agregar(np.zeros((101, 161), np.uint8), 0.2)                  # gris -> BGR
        ruta = g.cerrar()
        cap = cv2.VideoCapture(str(ruta))
        ok, f = cap.read()
        cap.release()
        self.assertTrue(ok)
        self.assertEqual(f.shape[:2], (100, 160))
        self.assertEqual(n_cuadros(ruta), 3)

    def test_error_no_tumba_la_carrera(self):
        """Una ruta imposible apaga la grabacion sin lanzar excepciones."""
        bloqueo = self.dir / "archivo"
        bloqueo.write_text("x")
        g = Grabador(bloqueo / "sub" / "e.mp4", fps=10, bloquear=True, h264=False)
        for k in range(20):
            g.agregar(np.zeros((50, 50, 3), np.uint8), k * 0.1)
        self.assertIsNone(g.cerrar())
        self.assertFalse(g.activo)

    def test_sin_cuadros(self):
        self.assertIsNone(Grabador(self.dir / "f.mp4").cerrar())

    def test_encajar(self):
        out = _encajar(np.full((50, 200, 3), 255, np.uint8), (100, 100))
        self.assertEqual(out.shape, (100, 100, 3))


if __name__ == "__main__":
    unittest.main()
