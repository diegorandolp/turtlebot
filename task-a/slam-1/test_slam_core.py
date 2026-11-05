"""Pruebas sin robot del nucleo SLAM. Desde la raiz del repo:
   python3 -m unittest discover -s task-a/slam-1 -p "test_*.py"
"""
import math
import time
import unittest
from dataclasses import dataclass

import numpy as np

from slam_core import BEAMS, OdomEngine, OdomTracker, make_engine, scan_to_mm


@dataclass
class FakeScan:
    angles: np.ndarray
    ranges: np.ndarray
    range_max: float = 12.0


@dataclass
class FakePose:
    x: float
    y: float
    yaw: float
    stamp: float = 0.0


def room_scan(x, y, yaw, half=3.0, n=360):
    """Scan de un cuarto cuadrado [-half, half]^2 visto desde (x, y, yaw)."""
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    wa = a + yaw
    c, s = np.cos(wa), np.sin(wa)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(c > 0, (half - x) / c, (-half - x) / c)
        ty = np.where(s > 0, (half - y) / s, (-half - y) / s)
    return FakeScan(a, np.minimum(tx, ty))


class TestScanToMm(unittest.TestCase):
    def test_frente_va_al_indice_central(self):
        sc = FakeScan(np.array([0.0, math.pi / 2, -math.pi / 2]), np.array([1.0, 2.0, 3.0]))
        mm = scan_to_mm(sc)
        self.assertEqual(len(mm), BEAMS)
        self.assertEqual(mm[180], 1000)      # frente
        self.assertEqual(mm[270], 2000)      # izquierda (+90): indice crece hacia la izquierda
        self.assertEqual(mm[90], 3000)       # derecha

    def test_invalidos_a_cero_y_minimo_por_grado(self):
        sc = FakeScan(np.array([0.0, 0.001, 1.0, 1.1]), np.array([2.0, 1.5, np.nan, np.inf]))
        mm = scan_to_mm(sc)
        self.assertEqual(mm[180], 1500)
        self.assertEqual(sum(1 for m in mm if m > 0), 1)


class TestOdomTracker(unittest.TestCase):
    def test_avance_y_giro(self):
        t = OdomTracker()
        self.assertEqual(t.delta(FakePose(0, 0, 0, 0.0))[:2], (0.0, 0.0))
        d, dth, dt = t.delta(FakePose(1.0, 0, 0, 0.5))
        self.assertAlmostEqual(d, 1000.0)
        self.assertAlmostEqual(dth, 0.0)
        self.assertAlmostEqual(dt, 0.5)
        d, dth, _ = t.delta(FakePose(1.0, 0, math.pi / 2, 1.0))
        self.assertAlmostEqual(d, 0.0)
        self.assertAlmostEqual(dth, 90.0)

    def test_marcha_atras_es_negativa(self):
        t = OdomTracker()
        t.delta(FakePose(0, 0, 0, 0.0))
        self.assertLess(t.delta(FakePose(-0.2, 0, 0, 0.1))[0], 0)


class TestOdomEngine(unittest.TestCase):
    def test_pared_en_el_mismo_sitio_aunque_el_robot_se_mueva(self):
        eng = OdomEngine(size_m=20.0, pixels=500)
        for i, x in enumerate([0.0, 0.0, 0.5, 1.0, 1.0]):
            eng.update(room_scan(x, 0, 0), FakePose(x, 0, 0, i * 0.2))
        img = eng.map_image()
        n, res = 500, 20.0 / 500
        row = n // 2
        col_wall = int(n / 2 + 3.0 / res)
        self.assertLess(img[row - 3:row + 4, col_wall - 1:col_wall + 2].min(), 60)   # pared a x = 3 m (tolera 1 celda)
        self.assertGreater(img[row, int(n / 2 + 1.0 / res)], 140)           # libre a x = 1 m
        # la pared no se duplica 1 m antes (seria un error de registro)
        self.assertGreater(img[row, int(n / 2 + 2.0 / res)], 140)

    def test_origen_es_la_pose_inicial(self):
        eng = OdomEngine()
        eng.update(room_scan(5.0, 2.0, 0.0, half=9.0), FakePose(5.0, 2.0, 0.0))
        self.assertEqual(eng.position(), (0.0, 0.0, 0.0))
        eng.update(room_scan(6.0, 2.0, 0.0, half=9.0), FakePose(6.0, 2.0, 0.0, 0.2))
        self.assertAlmostEqual(eng.position()[0], 1.0)

    def test_coste_por_actualizacion(self):
        eng = OdomEngine()
        sc, po = room_scan(0, 0, 0), FakePose(0, 0, 0)
        t0 = time.perf_counter()
        for _ in range(20):
            eng.update(sc, po)
        ms = (time.perf_counter() - t0) / 20 * 1000
        print(f"\n[info] OdomEngine: {ms:.1f} ms por scan")
        self.assertLess(ms, 100)


class TestBreezy(unittest.TestCase):
    def test_breezy_si_esta_instalado(self):
        try:
            eng = make_engine("breezy", size_m=20.0, pixels=500)
        except ImportError:
            self.skipTest("BreezySLAM no instalado")
        for i in range(10):
            eng.update(room_scan(0, 0, 0), FakePose(0, 0, 0, i * 0.2))
        x, y, _ = eng.position()
        self.assertLess(math.hypot(x, y), 0.3)                  # quieto: sigue en el origen
        self.assertLess(eng.map_image().min(), 60)              # hay paredes


if __name__ == "__main__":
    unittest.main()
