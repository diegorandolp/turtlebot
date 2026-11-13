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

from coverage import Coverage                                                     # noqa: E402
from explorer import (BARRIDO, CONFIRMAR, EXPLORAR, HECHO, INSPECCIONAR, PAUSA, SIN_B, VOLVER,   # noqa: E402
                      Explorer)
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

    def points(self):                       # un unico punto delante, a la distancia `front`
        return np.array([self.f]), np.array([0.0])

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


class RoomScan:
    """Scan de un cuarto cuadrado [-half, half]^2 (para cobertura): misma interfaz que tb4.client.Scan."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, half=2.0, n=360):
        self.angles = np.linspace(-math.pi, math.pi, n, endpoint=False) + 1e-6      # evita sin(0) = 0 exacto
        wa = self.angles + yaw
        c, s = np.cos(wa), np.sin(wa)
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(c > 0, (half - x) / c, (-half - x) / c)
            ty = np.where(s > 0, (half - y) / s, (-half - y) / s)
        self.ranges = np.minimum(tx, ty)
        self.range_max = 12.0

    def points(self):
        ok = np.isfinite(self.ranges)
        return self.ranges[ok] * np.cos(self.angles[ok]), self.ranges[ok] * np.sin(self.angles[ok])

    def sector_min(self, a, b):
        d = np.degrees(self.angles)
        sel = (d >= a) & (d <= b) & np.isfinite(self.ranges)
        return float(self.ranges[sel].min()) if sel.any() else math.inf


def room_map(half=2.0):
    """Mapa de un cuarto cuadrado ya conocido: libre dentro, pared en el borde."""
    img = np.full((N, N), 127, np.uint8)
    img[px(-half):px(half), px(-half):px(half)] = 255
    img[px(-half) - 3:px(-half), px(-half) - 3:px(half) + 3] = 0
    img[px(half):px(half) + 3, px(-half) - 3:px(half) + 3] = 0
    img[px(-half):px(half), px(-half) - 3:px(-half)] = 0
    img[px(-half):px(half), px(half):px(half) + 3] = 0
    return img


class TestCoverage(unittest.TestCase):
    def setUp(self):
        self.pl = Planner(SIZE, N)
        self.cov = Coverage(self.pl, fov_deg=60, rng=1.5)
        self.img = room_map()

    def test_al_principio_todo_pendiente(self):
        vistas, total = self.cov.stats(self.img)
        self.assertEqual(vistas, 0)
        self.assertGreater(total, 50)

    def test_una_foto_cubre_solo_el_cono_y_solo_hasta_el_rango(self):
        # robot en el centro mirando al este: la pared este esta a 2 m (fuera de rango 1.5): nada que ver
        self.cov.mark((0.0, 0.0, 0.0), RoomScan(0, 0, 0))
        self.assertEqual(self.cov.stats(self.img)[0], 0)
        # a 0.8 m de la pared este: sí la ve, pero no las otras
        self.cov.mark((1.2, 0.0, 0.0), RoomScan(1.2, 0, 0))
        vistas, total = self.cov.stats(self.img)
        self.assertGreater(vistas, 3)
        self.assertLess(vistas, total / 3)

    def test_no_ve_a_traves_de_una_pared(self):
        # el scan dice que hay pared a 0.5 m al frente: el cono no marca nada mas alla
        sc = RoomScan(1.5, 0.0, 0.0)                     # pared este a 0.5 m
        self.cov.mark((1.5, 0.0, 0.0), sc)
        r, c = self.pl.to_cell(1.9, 0.0)
        self.assertFalse(self.cov.seen[r, c + 6:c + 12].any())     # nada tras la pared (x > 2.4)

    def test_barrido_completo_cubre_todas_las_paredes_cercanas(self):
        for k in range(8):
            yaw = k * math.pi / 4
            self.cov.mark((0.0, 0.0, yaw), RoomScan(0, 0, yaw, half=1.2))
        img = room_map(1.2)
        vistas, total = self.cov.stats(img)
        self.assertGreater(vistas / total, 0.8)

    def test_best_heading_apunta_a_lo_pendiente_y_excluye_lo_hecho(self):
        img = room_map(1.2)
        _, W, _ = self.cov.weights(img)
        sc = RoomScan(0, 0, 0, half=1.2)
        k = self.cov.best_heading((0.0, 0.0, 0.0), sc, W)
        self.assertEqual(k, 0)                           # ya mira al este: el que exige girar menos
        k2 = self.cov.best_heading((0.0, 0.0, 0.0), sc, W, exclude={0})
        self.assertNotIn(k2, (None, 0))

    def test_plan_view_devuelve_un_punto_de_vista_alcanzable_y_termina(self):
        img = room_map(2.5)
        res = self.cov.plan_view(img, (0.0, 0.0))
        self.assertIsNotNone(res)
        x, y, h, g, path = res
        self.assertGreaterEqual(g, self.cov.min_gain)
        self.assertGreater(len(path), 1)
        # con todo visto, ya no hay nada que inspeccionar
        self.cov.seen[:, :] = True
        self.assertIsNone(self.cov.plan_view(img, (0.0, 0.0)))


class TestBarrido(unittest.TestCase):
    def mk(self):
        pl = Planner(SIZE, N)
        cov = Coverage(pl, fov_deg=60, rng=1.5)
        return Explorer(pl, cov=cov), cov

    def test_empieza_con_una_mirada_de_360_y_gira_en_sitio(self):
        e, cov = self.mk()
        img, sc = room_map(1.2), RoomScan(0, 0, 0, half=1.2)
        e.start()
        e.step(0.0, img, (0, 0, 0), sc, photos=0)
        self.assertEqual(e.state, BARRIDO)
        v, w = e.step(0.1, img, (0, 0, 0), sc, photos=1)         # mira al este: ya es el rumbo, pasa a esperar foto
        self.assertEqual(v, 0.0)

    def test_la_mirada_inicial_espera_a_que_haya_mapa(self):
        e, cov = self.mk()
        vacio = np.full((N, N), 127, np.uint8)
        e.start()
        self.assertEqual(e.step(0.0, vacio, (0, 0, 0), RoomScan(), photos=0), (0.0, 0.0))
        self.assertEqual(e.state, EXPLORAR)                       # espera quieto, no se rinde ni barre
        e.step(1.0, room_map(1.2), (0, 0, 0), RoomScan(half=1.2), photos=0)
        self.assertEqual(e.state, BARRIDO)                        # ya hay mapa: barre

    def test_espera_foto_y_pasa_al_siguiente_rumbo(self):
        e, cov = self.mk()
        img, sc = room_map(1.2), RoomScan(0, 0, 0, half=1.2)
        e.start()
        e.step(0.0, img, (0, 0, 0), sc, photos=0)
        e.step(0.1, img, (0, 0, 0), sc, photos=0)                 # elige rumbo 0 y se alinea -> settle
        self.assertEqual(e.sweep["stage"], "settle")
        for t, ph in ((0.2, 0), (0.5, 1), (1.0, 2)):              # hacen falta 2 fotos nuevas, ya parado
            v, w = e.step(t, img, (0, 0, 0), sc, photos=ph)
            self.assertEqual((v, w), (0.0, 0.0))
        self.assertIn(0, e.sweep["done"])

    def test_si_la_camara_no_da_fotos_el_barrido_se_cancela(self):
        e, cov = self.mk()
        img, sc = room_map(1.2), RoomScan(0, 0, 0, half=1.2)
        e.start()
        e.step(0.0, img, (0, 0, 0), sc, photos=0)
        e.step(0.1, img, (0, 0, 0), sc, photos=0)
        e.step(7.0, img, (0, 0, 0), sc, photos=0)                 # 7 s sin una foto nueva
        self.assertEqual(e.state, EXPLORAR)
        self.assertTrue(any("camara no da fotos" in m for m in e.log))

    def test_con_camara_caida_no_hace_barrido_ni_inspeccion(self):
        e, cov = self.mk()
        e.start()
        e.step(0.0, corridor_map(), (0.0, 0.0, 0.0), FakeScan(), photos=0, cam_ok=False)
        self.assertEqual(e.state, EXPLORAR)                       # sigue con el LiDAR, sin barrer
        img = corridor_map()
        img[px(-1) - 3:px(1) + 3, px(3):px(3) + 3] = 0            # sin fronteras
        for t in (2.0, 3.5, 7.0):                                 # replanifica; tras 3 s sin fronteras se rinde
            e.step(t, img, (0.0, 0.0, 0.0), FakeScan(), photos=0, cam_ok=False)
        self.assertEqual(e.state, SIN_B)                          # se rinde: no puede inspeccionar

    def test_sin_fronteras_con_camara_pasa_a_inspeccionar(self):
        e, cov = self.mk()
        e.start()
        e._want_sweep = False
        img = corridor_map()
        img[px(-1) - 3:px(1) + 3, px(3):px(3) + 3] = 0
        e.step(0.0, img, (0.0, 0.0, 0.0), FakeScan(), photos=5, cam_ok=True)
        e.step(4.0, img, (0.0, 0.0, 0.0), FakeScan(), photos=5, cam_ok=True)
        self.assertEqual(e.state, INSPECCIONAR)

    def test_rostro_durante_un_barrido_para_y_si_se_descarta_sigue_el_barrido(self):
        e, cov = self.mk()
        img, sc = room_map(1.2), RoomScan(0, 0, 0, half=1.2)
        e.start()
        e.step(0.0, img, (0, 0, 0), sc, photos=0)
        self.assertEqual(e.state, BARRIDO)
        e.face_candidate()
        self.assertEqual(e.state, CONFIRMAR)
        self.assertEqual(e.step(0.1, img, (0, 0, 0), sc, photos=1), (0.0, 0.0))
        e.face_rejected()
        self.assertEqual(e.state, BARRIDO)


class TestFrenoReactivo(unittest.TestCase):
    def test_pasar_rozando_una_esquina_no_frena(self):
        """Un obstaculo a 0.30 m pero FUERA del pasillo (|y| = 0.5) no debe parar al robot."""
        from explorer import front_clear

        class S:
            def points(self):
                return np.array([0.30, 1.0]), np.array([0.50, 0.0])
        self.assertAlmostEqual(front_clear(S()), 1.0)
        class S2:
            def points(self):
                return np.array([0.30]), np.array([0.10])
        self.assertAlmostEqual(front_clear(S2()), 0.30)


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
        r.wx, r.wy, r.wth = r.face[0] - 1.2, r.face[1], 0.0
        self.assertTrue(r.face_visible())
        r.wth = math.pi
        self.assertFalse(r.face_visible())


class TestIntentos(unittest.TestCase):
    """Estrategia de 3 intentos: el 1 mapea y guarda; el 2 y el 3 cargan el mapa e inspeccionan."""

    def test_combinar_mapa_previo_y_vivo(self):
        import explorar
        prior = corridor_map()
        live = np.full((N, N), 127, np.uint8)
        live[px(0.5), px(1.5)] = 0                               # obstaculo nuevo (no estaba en el intento 1)
        live[px(5.0), px(5.0)] = 255                             # zona que el mapa previo no conocia
        out = explorar.combinar(prior, live)
        self.assertEqual(out[px(0.5), px(1.5)], 0)
        self.assertEqual(out[px(5.0), px(5.0)], 255)
        self.assertEqual(out[px(0.0), px(0.0)], 255)             # lo libre del mapa previo se conserva
        self.assertEqual(out[px(-1) - 1, px(1)], 0)              # y sus paredes tambien

    def test_guardar_y_cargar_ultimo(self):
        import tempfile

        import explorar

        class Eng:
            def map_image(self):
                return corridor_map()
        with tempfile.TemporaryDirectory() as d:
            explorar.guardar_mapa(Eng(), d, SIZE)
            m = explorar.cargar_mapa("ultimo", d, N)
            self.assertTrue(np.array_equal(m, corridor_map()))
            antes = (Path(d) / "ultimo" / "mapa.npy").stat().st_mtime_ns

            class Vacio:
                def map_image(self):
                    return np.full((N, N), 127, np.uint8)
            explorar.guardar_mapa(Vacio(), d, SIZE, ultimo=False)   # una inspeccion no pisa el "ultimo"
            self.assertEqual((Path(d) / "ultimo" / "mapa.npy").stat().st_mtime_ns, antes)
            self.assertTrue(np.array_equal(explorar.cargar_mapa("ultimo", d, N), corridor_map()))

    def test_inspeccion_directa_salta_la_exploracion(self):
        pl = Planner(SIZE, N)
        e = Explorer(pl, cov=Coverage(pl), inspeccion_directa=True)
        e.start()
        self.assertEqual(e.state, INSPECCIONAR)
        e.pause()
        e.start()                                                # al reanudar sigue inspeccionando
        self.assertEqual(e.state, INSPECCIONAR)

    def test_sin_mapa_previo_explora_como_siempre(self):
        pl = Planner(SIZE, N)
        e = Explorer(pl, cov=Coverage(pl))
        e.start()
        self.assertEqual(e.state, EXPLORAR)


if __name__ == "__main__":
    unittest.main()
