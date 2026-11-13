"""Pruebas sin robot de carrera-1. Desde la raiz del repo:
   python -m unittest discover -s task-a/carrera-1 -p "test_*.py"
(la de punta a punta corre una carrera completa en el simulador: ~10 s)
"""
import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from flechas import (DER, IZQ, DetectorFlechas, flecha_curva, flecha_giro, flecha_recta,   # noqa: E402
                     quitar_aro)
from mundos import _camino_libre, Excavadora, construir, textura_flecha               # noqa: E402
from navegador import (ACERCAR, HECHO, IDA, RECONOCER, VOLVER, Carrera, Config,     # noqa: E402
                       carriles, ventana_dinamica)
from percepcion import Obs, Objetivo, Senales                                         # noqa: E402
from reconocimiento import Votacion                                                   # noqa: E402
from simulador import Mundo, Pared, SimRobot                                          # noqa: E402
from tb4.client import Scan                                                           # noqa: E402


def escena(mascara, lado=60, x=95, y=90, fondo=150, flip=False):
    """Senal de papel blanco con la mascara (negra) pegada en una 'pared' gris de 250x250."""
    img = np.full((250, 250, 3), fondo, np.uint8)
    img[160:] = 110                                                  # suelo
    papel = np.full((lado, lado, 3), 235, np.uint8)
    m = cv2.resize(cv2.flip(mascara, 1) if flip else mascara, (int(lado * 0.8),) * 2, interpolation=cv2.INTER_AREA)
    o = (lado - m.shape[0]) // 2
    papel[o:o + m.shape[0], o:o + m.shape[1]][m > 127] = 20
    img[y:y + lado, x:x + lado] = papel
    return cv2.GaussianBlur(img, (3, 3), 0.7)


def scan_pasillo(ancho=1.2, largo=6.0, frente=None, n=720):
    """Scan de un pasillo recto (x adelante) de `ancho` m; opcionalmente una pared al frente a `frente` m."""
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    with np.errstate(divide="ignore"):
        r_lat = np.where(np.abs(np.sin(a)) > 1e-6, (ancho / 2) / np.abs(np.sin(a)), np.inf)
        r_fr = np.where(np.cos(a) > 1e-6, (frente if frente else largo) / np.cos(a), np.inf)
        r_tr = np.where(np.cos(a) < -1e-6, 1.0 / -np.cos(a), np.inf)
    r = np.minimum(np.minimum(r_lat, r_fr), r_tr)
    return Scan(a.astype(np.float32), r.astype(np.float32), 12.0, 0.0, 0.0)


class FlechasTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.det = DetectorFlechas()

    def test_estilos_y_sentidos(self):
        for nombre, m in (("recta", flecha_recta()), ("giro", flecha_giro()), ("curva", flecha_curva())):
            for flip, esperado in ((False, DER), (True, IZQ)):
                for lado in (40, 80):
                    with self.subTest(estilo=nombre, sentido=esperado, lado=lado):
                        fl = self.det(escena(m, lado, flip=flip))
                        self.assertTrue(fl, "no detecto la flecha")
                        self.assertEqual(fl[0].dir, esperado)

    def test_senal_circular_de_giro(self):
        rng = np.random.default_rng(3)
        tex = textura_flecha("izq", rng, estilo="giro", aro=True)
        img = np.full((250, 250, 3), 160, np.uint8)
        img[80:150, 90:160] = cv2.resize(tex, (70, 70), interpolation=cv2.INTER_AREA)
        fl = self.det(img)
        self.assertTrue(fl)
        self.assertEqual(fl[0].dir, IZQ)

    def test_no_flechas(self):
        aro = np.zeros((128, 128), np.uint8)
        cv2.circle(aro, (64, 64), 56, 255, 9)
        prohibido = aro.copy()
        cv2.line(prohibido, (24, 104), (104, 24), 255, 16)
        cuadrado = np.zeros((128, 128), np.uint8)
        cuadrado[20:108, 20:108] = 255
        for nombre, m in (("aro", aro), ("prohibido", prohibido), ("cuadrado", cuadrado)):
            with self.subTest(nombre):
                self.assertEqual(self.det(escena(m, 70)), [])
        self.assertEqual(self.det(np.full((250, 250, 3), 128, np.uint8)), [])

    def test_mancha_de_color_sin_papel(self):
        img = np.full((250, 250, 3), 200, np.uint8)
        pts = np.array([[100, 80], [160, 110], [100, 140]], np.int32)
        cv2.fillPoly(img, [pts], (60, 60, 220))                     # triangulo rojo, sin papel: no es flecha
        self.assertEqual(self.det(img), [])

    def test_quitar_aro(self):
        m = np.zeros((128, 128), np.uint8)
        cv2.circle(m, (64, 64), 58, 255, 8)
        cv2.rectangle(m, (40, 56), (90, 72), 255, -1)
        dentro = quitar_aro(m)
        self.assertIsNotNone(dentro)
        self.assertGreater((dentro > 0).sum(), 30)
        self.assertIsNone(quitar_aro(flecha_recta()))


class MovimientoTests(unittest.TestCase):
    def test_carriles_en_un_pasillo(self):
        L = carriles(scan_pasillo(1.2, frente=2.0))
        k0 = len(L) // 2                                             # rumbo 0
        self.assertAlmostEqual(L[k0], 2.0, delta=0.05)
        self.assertLess(L[k0 + 18], 0.7)                             # 90 grados: la pared a 0.6 m

    def test_ventana_dinamica_no_choca(self):
        s = scan_pasillo(1.2, frente=0.45)
        x, y = s.points()
        v, w = ventana_dinamica(x, y, 0.0, 0.3, 1.8)
        self.assertLessEqual(v * 1.2, 0.45 - 0.30 + 1e-6)             # pared a 0.45 m: no se mete en la zona de freno del puente
        s = scan_pasillo(1.2)
        x, y = s.points()
        v, w = ventana_dinamica(x, y, 0.0, 0.3, 1.8)
        self.assertGreater(v, 0.25)                                  # pasillo libre: a fondo
        self.assertLess(abs(w), 0.3)

    def test_ventana_dinamica_gira_hacia_el_objetivo(self):
        x, y = scan_pasillo(3.0).points()
        v, w = ventana_dinamica(x, y, math.radians(90), 0.3, 1.8)
        self.assertGreater(w, 0.5)


class EstadosTests(unittest.TestCase):
    def nueva(self):
        nombres = ["Ana", "Beto"]
        vot = Votacion(nombres, min_fotos=2)
        nav = Carrera(Config(), None, vot, Objetivo(vot), Senales())
        nav.empezar(0.0, 0.0)
        return nav

    def cara(self, t, px, d=1.0, sims=(0.6, 0.1)):
        res = type("R", (), {"sims": np.array(sims)})() if px >= 10 else None
        return Obs("cara", t, 0.0, d, (d, 0.0), px, (100, 100, px, px), res=res)

    def test_flujo_completo(self):
        nav = self.nueva()
        s = scan_pasillo(1.2, frente=1.5)
        self.assertEqual(nav.estado, IDA)
        nav.ver_cara(1.0, self.cara(1.0, 20, 1.5), 0.3)
        self.assertEqual(nav.estado, IDA)                            # una deteccion sola no alcanza
        nav.ver_cara(1.2, self.cara(1.2, 22, 1.5), 0.3)
        self.assertEqual(nav.estado, ACERCAR)
        nav.ver_cara(2.0, self.cara(2.0, 50, 0.7), 1.0)
        nav.paso(2.0, (0.8, 0.0, 0.0), s)
        self.assertEqual(nav.estado, RECONOCER)
        nav.ver_cara(2.1, self.cara(2.1, 52, 0.7), 1.0)
        nav.paso(2.1, (0.8, 0.0, 0.0), s)
        self.assertEqual(nav.estado, VOLVER)
        self.assertEqual(nav.nombre, "Ana")
        nav.paso(10.0, (0.1, 0.05, math.pi), s)                       # ya en A
        self.assertEqual(nav.estado, HECHO)

    def test_nunca_decide_sin_votos(self):
        nav = self.nueva()
        s = scan_pasillo(1.2, frente=1.5)
        for t in (1.0, 1.1):
            nav.ver_cara(t, self.cara(t, 8, 0.6), 0.1)               # caras chiquitas: sin reconocimiento
        o = self.cara(1.2, 8, 0.6)
        o.borde_sup = True
        nav.ver_cara(1.2, o, 0.1)
        nav.paso(1.2, (0.0, 0.0, 0.0), s)
        self.assertEqual(nav.estado, RECONOCER)
        for t in np.arange(1.3, 12.0, 0.1):
            nav.paso(float(t), (0.0, 0.0, 0.0), s)
        self.assertIsNone(nav.nombre)
        self.assertNotEqual(nav.estado, VOLVER)


class SimuladorTests(unittest.TestCase):
    def caja(self):
        paredes = [Pared((-1, -1), (3, -1)), Pared((3, -1), (3, 1)), Pared((3, 1), (-1, 1)), Pared((-1, 1), (-1, -1))]
        return Mundo(paredes, (0.0, 0.0, 0.0))

    def test_lidar_y_freno(self):
        r = SimRobot(self.caja(), odom_ruido=0.0)
        s = r.scan()
        self.assertAlmostEqual(float(np.nanmedian(s.ranges[355:365])), 3.0, delta=0.05)
        for _ in range(400):                                         # 20 s contra la pared
            r.move(0.3, 0.0)
            r.advance(0.05)
        self.assertEqual(r.collisions, 0)                            # el freno del puente lo para antes
        self.assertGreater(r.wx, 2.5)

    def test_foto(self):
        img = SimRobot(construir("demo")).photo()
        self.assertEqual(img.shape, (250, 250, 3))

    def test_mundos_aleatorios_tienen_paso(self):
        for k in range(1, 6):
            with self.subTest(semilla=k):
                m = construir(f"aleatorio:{k}")
                self.assertTrue(m.persona)
                caras = [c for c in m.carteles if c.tipo == "rostro"]
                self.assertEqual(len(caras), 1)

    def test_excavadora(self):
        ex = Excavadora()
        ex.rect(0, -0.5, 3, 0.5)
        self.assertTrue(_camino_libre(ex, (0.3, 0), (2.7, 0)))
        ex.rect(1.4, -0.5, 1.6, 0.5, valor=False)
        self.assertFalse(_camino_libre(ex, (0.3, 0), (2.7, 0)))


class PuntaAPuntaTests(unittest.TestCase):
    def test_demo_completa(self):
        import carrera
        args = carrera.parse(["--sim", "--go", "--no-gui", "--mudo", "--no-guardar", "--modelo", "mbf",
                              "--cam-periodo", "0.15"])
        m = construir("demo")
        res = carrera.correr(SimRobot(m), args, sim=True, gui=False, mundo=m, max_t=150)
        self.assertTrue(res["exito"], res["log"][-3:])
        self.assertEqual(res["choques"], 0)
        self.assertLess(res["tiempo"], 90)


if __name__ == "__main__":
    unittest.main()
