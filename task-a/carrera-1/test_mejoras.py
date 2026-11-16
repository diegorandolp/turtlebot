"""Pruebas sin robot de las mejoras de carrera-1: relocalizar.py (volver exacto a A), la decision del nombre
solo con votacion segura, y mirada.py (cobertura que solo cuenta paredes vistas de frente).

    python -m unittest discover -s task-a/carrera-1 -p "test_mejoras.py"
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent / "auto-1", HERE.parents[1]):
    sys.path.insert(0, str(p))

from navegador import ACERCAR, RECONOCER, VOLVER, Carrera, Config   # noqa: E402
from relocalizar import Relocalizador, icp                         # noqa: E402
from reconocimiento.rostros import Votacion                         # noqa: E402


class FakeScan:
    """Scan de una sala rectangular con una caja, visto desde (x, y, th) en el marco de la sala."""

    def __init__(self, x, y, th, n=720, ruido=0.0, seed=0):
        rng = np.random.default_rng(seed)
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        segs = [((0, 0), (4, 0)), ((4, 0), (4, 3)), ((4, 3), (0, 3)), ((0, 3), (0, 0)),
                ((1.5, 1.0), (2.2, 1.0)), ((2.2, 1.0), (2.2, 1.6)), ((2.2, 1.6), (1.5, 1.6)), ((1.5, 1.6), (1.5, 1.0))]
        r = np.full(n, np.inf)
        for k, a in enumerate(self.angles + th):
            dx, dy = math.cos(a), math.sin(a)
            for (ax, ay), (bx, by) in segs:
                ex, ey = bx - ax, by - ay
                den = dx * ey - dy * ex
                if abs(den) < 1e-9:
                    continue
                t = ((ax - x) * ey - (ay - y) * ex) / den
                u = ((ax - x) * dy - (ay - y) * dx) / den
                if t > 0 and 0 <= u <= 1:
                    r[k] = min(r[k], t)
        self.ranges = r + rng.normal(0, ruido, n)

    def points(self):
        return self.ranges * np.cos(self.angles), self.ranges * np.sin(self.angles)


class TestRelocalizar(unittest.TestCase):
    def test_icp_recupera_transformacion(self):
        ref = np.stack(FakeScan(0.6, 0.8, 0.0).points(), 1)
        ref = ref[np.isfinite(ref).all(1)]
        th, t = 0.12, np.array([0.15, -0.10])
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        src = (ref - t) @ R                      # src tal que R @ src + t = ref
        tx, ty, th2, err, frac = icp(src, ref)
        self.assertAlmostEqual(th2, th, places=2)
        self.assertAlmostEqual(tx, t[0], places=2)
        self.assertAlmostEqual(ty, t[1], places=2)
        self.assertLess(err, 0.01)

    def test_corrige_a_con_odometria_desviada(self):
        # A real en (0.6, 0.8) de la sala, mirando a +x. Marco del mapa = A.
        A = (0.6, 0.8)
        rl = Relocalizador()
        self.assertTrue(rl.referencia(FakeScan(*A, 0.0, ruido=0.005, seed=1)))
        # el robot esta de verdad 0.5 m delante de A (en el marco de A: (0.5, 0.1), mirando 170 grados)...
        real = (0.5, 0.1, math.radians(170))
        scan = FakeScan(A[0] + real[0], A[1] + real[1], real[2], ruido=0.005, seed=2)
        # ...pero la odometria se desvio: cree estar en (0.25, -0.05) con rumbo 160 grados
        est = (0.25, -0.05, math.radians(160))
        h = rl.corregir(scan, est)
        self.assertIsNotNone(h)
        # yendo a h (marco del mapa) el robot llega a la A real: comprobar que h, visto desde la pose
        # estimada, cae donde esta A visto desde la pose real
        def en_robot(p, pose):
            dx, dy = p[0] - pose[0], p[1] - pose[1]
            c, s = math.cos(-pose[2]), math.sin(-pose[2])
            return c * dx - s * dy, s * dx + c * dy
        a_desde_real = en_robot((0.0, 0.0), real)
        h_desde_est = en_robot(h, est)
        self.assertLess(math.hypot(a_desde_real[0] - h_desde_est[0], a_desde_real[1] - h_desde_est[1]), 0.05)

    def test_lejos_de_a_no_corrige(self):
        rl = Relocalizador()
        rl.referencia(FakeScan(0.6, 0.8, 0.0))
        self.assertIsNone(rl.corregir(FakeScan(3.5, 2.5, 0.0), (2.9, 1.7, 0.0)))

    def test_mala_alineacion_se_rechaza(self):
        rl = Relocalizador()
        rl.referencia(FakeScan(0.6, 0.8, 0.0))
        basura = FakeScan(0.6, 0.8, 0.0)
        basura.ranges = np.random.default_rng(3).uniform(0.3, 3.0, len(basura.ranges))
        self.assertIsNone(rl.corregir(basura, (0.1, 0.0, 0.0)))


class O:
    def __init__(self, px=20.0, borde_sup=False, t=0.0):
        self.px, self.borde_sup, self.t, self.rumbo = px, borde_sup, t, 0.0


class Obj:
    def __init__(self, ancho_max):
        self.ancho_max, self.ultima, self.xy = ancho_max, O(ancho_max), np.zeros(2)

    def visto_hace(self, now):
        return 0.1


class TestDecision(unittest.TestCase):
    def _nav(self, sims_list, pesos, ancho_max):
        vot = Votacion(["Ana", "Beto", "Cami"])
        for s, w in zip(sims_list, pesos):
            vot.agregar(np.array(s), w)
        nav = Carrera(Config(), votacion=vot)
        nav.obj = Obj(ancho_max)
        nav.estado, nav.t_inicio, nav.t_reconocer = RECONOCER, 0.0, 0.0
        return nav

    def test_ocho_fotos_dudosas_no_bastan(self):
        # 8 fotos chicas (peso 0.15) con el lider a p ~0.7: antes decidia por max_fotos
        nav = self._nav([[0.30, 0.28, 0.10]] * 8, [0.15] * 8, ancho_max=40)
        self.assertTrue(nav.vot.decidido())                # la Votacion sola si decidiria
        self.assertFalse(nav._seguro())
        nav._reconocer(1.0, (0, 0, 0))
        self.assertEqual(nav.estado, RECONOCER)            # sigue mirando

    def test_votacion_segura_decide(self):
        nav = self._nav([[0.6, 0.2, 0.1]] * 3, [1.0] * 3, ancho_max=50)
        nav._reconocer(1.0, (0, 0, 0))
        self.assertEqual((nav.estado, nav.nombre), (VOLVER, "Ana"))

    def test_cara_chica_se_acerca_antes_de_decidir_por_tiempo(self):
        nav = self._nav([[0.30, 0.28, 0.10]] * 3, [0.15] * 3, ancho_max=16)
        nav._reconocer(10.0, (0, 0, 0))                    # pasado reconocer_max_s
        self.assertEqual(nav.estado, ACERCAR)
        self.assertEqual(nav.px_extra, 1.5)

    def test_al_final_decide_igual(self):
        nav = self._nav([[0.30, 0.28, 0.10]] * 3, [0.15] * 3, ancho_max=16)
        nav.t_reconocer = 150.0
        nav._reconocer(160.0, (0, 0, 0))                   # 160 s de carrera: no perder el intento
        self.assertEqual((nav.estado, nav.nombre), (VOLVER, "Ana"))


if __name__ == "__main__":
    unittest.main()


class ParedScan:
    """Scan de una pared recta en x = 1.0 m (marco del mapa), visto desde (x, y, th)."""

    def __init__(self, x, y, th, n=720):
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        a = self.angles + th
        c = np.cos(a)
        with np.errstate(divide="ignore"):
            t = np.where(c > 1e-6, (1.0 - x) / c, np.inf)
        self.ranges = np.where(np.abs(y + t * np.sin(a)) < 5.0, t, np.inf)


class TestCoberturaFrontal(unittest.TestCase):
    def setUp(self):
        from mirada import CoberturaFrontal
        from planner import Planner
        self.pl = Planner(20.0, 500, cell=2, robot_radius=0.17, margin=0.05)
        self.img = np.full((500, 500), 255, np.uint8)                 # todo libre...
        col = int(500 / 2 + 1.0 / self.pl.res)
        self.img[:, col:col + 2] = 0                                  # ...salvo una pared en x = 1.0 m
        self.cov = CoberturaFrontal(self.pl, fov_deg=40.0, rng=1.5)

    def _pendiente_cerca(self, y):
        surface, W, _ = self.cov.weights(self.img)
        r, c = self.pl.to_cell(1.0, y)
        return float(W[r - 2:r + 3, c - 1:c + 3][surface[r - 2:r + 3, c - 1:c + 3]].sum())

    def test_de_frente_y_cerca_cuenta(self):
        self.cov.mark((0.0, 0.0, 0.0), ParedScan(0.0, 0.0, 0.0), img=self.img)
        self.assertEqual(self._pendiente_cerca(0.0), 0.0)

    def test_de_frente_pero_lejos_no_cuenta(self):
        # a 2.5 m una cara de 11 cm mide ~15 px: no se detectaria
        self.cov.rng = 3.0
        self.cov._t = np.arange(0.0, 3.0 + self.pl.cres, self.pl.cres * 0.75)
        self.cov.mark((-1.5, 0.0, 0.0), ParedScan(-1.5, 0.0, 0.0), img=self.img)
        self.assertGreater(self._pendiente_cerca(0.0), 0.0)

    def test_de_refilon_no_cuenta(self):
        # pasando junto a la pared (a 0.3 m), mirando a lo largo de ella: la ve a ~70 grados
        for y in np.arange(-1.5, 1.6, 0.1):
            self.cov.mark((0.7, y, math.pi / 2), ParedScan(0.7, y, math.pi / 2), img=self.img)
        self.assertGreater(self._pendiente_cerca(0.0), 0.0)

    def test_libre_delante_no_tapa_la_pared(self):
        # el espacio libre justo delante de la pared se ve, pero eso no marca la pared (antes si, al dilatar)
        for y in np.arange(-1.5, 1.6, 0.1):
            self.cov.mark((0.7, y, math.pi / 2), ParedScan(0.7, y, math.pi / 2), img=self.img)
        r, c = self.pl.to_cell(0.85, 0.0)
        self.assertTrue(self.cov.seen[r - 1:r + 2, c - 1:c + 2].any())
        self.assertGreater(self._pendiente_cerca(0.0), 0.0)
