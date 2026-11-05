"""Pruebas sin robot de auto-1. Desde la raiz del repo:
   python -m unittest discover -s task-a/auto-1 -p "test_*.py"
(la exploracion completa en el simulador se prueba a mano: python task-a/auto-1/explorar.py --sim --no-gui --go)
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from explorer import CONFIRMAR, EXPLORAR, HECHO, PAUSA, SIN_B, VOLVER, Explorer   # noqa: E402
from face import FaceWatcher                                                      # noqa: E402
from planner import Planner                                                       # noqa: E402
from sim import SimRobot                                                          # noqa: E402

N, SIZE = 500, 20.0
RES = SIZE / N


def px(x):
    return int(N / 2 + x / RES)


def corridor_map():
    """Mapa conocido: sala libre de x in [-1, 3], y in [-1, 1] con paredes alrededor; lo de mas alla del
    borde derecho (x > 3) es desconocido (frontera ahi)."""
    img = np.full((N, N), 127, np.uint8)
    img[px(-1):px(1), px(-1):px(3)] = 255                       # libre (fila = y, columna = x)
    img[px(-1) - 3:px(-1), px(-1) - 3:px(3)] = 0                # pared sur
    img[px(1):px(1) + 3, px(-1) - 3:px(3)] = 0                  # pared norte
    img[px(-1):px(1), px(-1) - 3:px(-1)] = 0                    # pared oeste
    return img


class FakeScan:
    def __init__(self, front=5.0, left=5.0, right=5.0):
        self.f, self.l, self.r = front, left, right

    def sector_min(self, a, b):
        if a >= -25 and b <= 25:
            return self.f
        return self.l if a >= 0 else self.r


class TestPlanner(unittest.TestCase):
    def setUp(self):
        self.pl = Planner(SIZE, N)

    def test_va_a_la_frontera_y_no_cruza_paredes(self):
        res = self.pl.plan_explore(corridor_map(), (0.0, 0.0))
        self.assertEqual(res.status, "ok")
        self.assertGreater(res.goal[0], 2.0)                    # la frontera esta al este
        img = corridor_map()
        for x, y in res.path:
            self.assertGreater(img[px(y), px(x)], 200)         # toda la ruta sobre celdas libres

    def test_sin_fronteras_si_todo_esta_cerrado(self):
        img = corridor_map()
        img[px(-1) - 3:px(1) + 3, px(3):px(3) + 3] = 0          # tapa el borde este
        self.assertEqual(self.pl.plan_explore(img, (0.0, 0.0)).status, "no_frontier")

    def test_meta_descartada_no_se_repite(self):
        res = self.pl.plan_explore(corridor_map(), (0.0, 0.0))
        res2 = self.pl.plan_explore(corridor_map(), (0.0, 0.0), bad_goals=[res.goal])
        self.assertTrue(res2.status != "ok" or math.hypot(res2.goal[0] - res.goal[0], res2.goal[1] - res.goal[1]) > 0.3)

    def test_plan_to_vuelve_al_origen(self):
        res = self.pl.plan_to(corridor_map(), (2.0, 0.0), (0.0, 0.0))
        self.assertEqual(res.status, "ok")
        self.assertLess(math.hypot(res.goal[0], res.goal[1]), 0.4)


class TestExplorer(unittest.TestCase):
    def ex(self, **kw):
        return Explorer(Planner(SIZE, N), **kw)

    def test_en_pausa_no_se_mueve(self):
        e = self.ex()
        self.assertEqual(e.step(0.0, corridor_map(), (0, 0, 0), FakeScan()), (0.0, 0.0))

    def test_explora_hacia_la_frontera(self):
        e = self.ex()
        e.start()
        v, w = e.step(0.0, corridor_map(), (0.0, 0.0, 0.0), FakeScan())
        self.assertGreater(v, 0.0)                              # apunta al este y avanza
        self.assertLessEqual(v, e.vmax + 1e-9)

    def test_gira_en_sitio_si_la_meta_esta_detras(self):
        e = self.ex()
        e.start()
        v, w = e.step(0.0, corridor_map(), (0.0, 0.0, math.pi), FakeScan())
        self.assertEqual(v, 0.0)
        self.assertNotEqual(w, 0.0)

    def test_obstaculo_cerca_para_y_gira_al_lado_libre(self):
        e = self.ex()
        e.start()
        v, w = e.step(0.0, corridor_map(), (0.0, 0.0, 0.0), FakeScan(front=0.30, left=2.0, right=0.5))
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)                              # izquierda = mas libre

    def test_cara_confirmada_para_y_marca_B(self):
        e = self.ex()
        e.start()
        e.face_candidate()
        self.assertEqual(e.state, CONFIRMAR)
        self.assertEqual(e.step(0.0, corridor_map(), (0, 0, 0), FakeScan()), (0.0, 0.0))
        e.face_confirmed((1.5, 0.2, 0.0))
        self.assertEqual(e.state, HECHO)
        self.assertEqual(e.b_xy, (1.5, 0.2))

    def test_con_volver_regresa_a_A(self):
        e = self.ex(volver=True)
        e.start()
        e.face_confirmed((2.0, 0.0, 0.0))
        self.assertEqual(e.state, VOLVER)
        v, w = e.step(0.0, corridor_map(), (2.0, 0.0, math.pi), FakeScan())
        self.assertGreater(v, 0.0)                              # de cara a A (al oeste)
        e.step(1.0, corridor_map(), (0.1, 0.0, math.pi), FakeScan())
        self.assertEqual(e.state, HECHO)

    def test_sin_fronteras_termina_sin_B(self):
        e = self.ex()
        e.start()
        img = corridor_map()
        img[px(-1) - 3:px(1) + 3, px(3):px(3) + 3] = 0
        e.step(0.0, img, (0.0, 0.0, 0.0), FakeScan())             # al principio espera: el mapa puede estar vacio
        self.assertEqual(e.state, EXPLORAR)
        e.step(4.0, img, (0.0, 0.0, 0.0), FakeScan())             # 4 s sin fronteras: se rinde
        self.assertEqual(e.state, SIN_B)

    def test_atasco_descarta_meta(self):
        e = self.ex()
        e.start()
        img = corridor_map()
        e.step(0.0, img, (0.0, 0.0, 0.0), FakeScan())            # fija la meta
        goal = e.goal
        e.step(3.0, img, (0.0, 0.0, 0.0), FakeScan())
        v, w = e.step(7.0, img, (0.02, 0.0, 0.0), FakeScan())    # 7 s sin avanzar
        self.assertEqual(v, 0.0)
        self.assertIn(goal, e.bad_goals)


class TestFaceWatcher(unittest.TestCase):
    def test_exige_dos_detecciones_seguidas(self):
        fw = FaceWatcher(period=1.0, need=2, retry_s=0.4)
        seen = iter([True, True])
        photo = lambda: None                       # noqa: E731
        detect = lambda img: next(seen)            # noqa: E731
        self.assertEqual(fw.update(0.0, photo, detect), "candidate")
        self.assertEqual(fw.update(0.1, photo, detect), "candidate")   # aun no toca nueva foto
        self.assertEqual(fw.update(0.5, photo, detect), "confirmed")

    def test_un_falso_positivo_se_descarta(self):
        fw = FaceWatcher()
        flags = iter([True, False])
        photo = lambda: None                       # noqa: E731
        detect = lambda img: next(flags)           # noqa: E731
        self.assertEqual(fw.update(0.0, photo, detect), "candidate")
        self.assertEqual(fw.update(0.5, photo, detect), "none")

    def test_error_de_camara_no_rompe(self):
        fw = FaceWatcher()

        def boom():
            raise RuntimeError("camara apagada")
        self.assertEqual(fw.update(0.0, boom, lambda i: True), "none")
        self.assertEqual(fw.update(1.0, lambda: None, lambda i: True), "none")   # en espera tras el error


class TestSim(unittest.TestCase):
    def test_no_atraviesa_paredes(self):
        r = SimRobot()
        for _ in range(400):                       # 60 s empujando hacia delante
            r.move(0.2, 0.0)
            r.advance(0.15)
        self.assertEqual(r.collisions, 0)          # el freno del puente para antes de chocar
        self.assertLess(r.wx, 3.0)                 # la pared interior esta en x = 3

    def test_la_cara_se_ve_solo_de_frente_y_cerca(self):
        r = SimRobot()
        self.assertFalse(r.face_visible())
        r.wx, r.wy, r.wth = r.face[0] - 1.5, r.face[1], 0.0
        self.assertTrue(r.face_visible())
        r.wth = math.pi
        self.assertFalse(r.face_visible())


if __name__ == "__main__":
    unittest.main()
