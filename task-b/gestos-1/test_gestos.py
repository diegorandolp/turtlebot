"""Pruebas del clasificador y del control con manos sintéticas (sin cámara, mediapipe ni robot)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from control import AVANZANDO, GIRANDO, PARADO, Control                           # noqa: E402
from gestos import (ABA, ABA_DER, ABA_IZQ, ARR_DER, ARR_IZQ, DER, IZQ,             # noqa: E402
                    LENTO, NINGUNO, RAPIDO, STOP, Estabilizador, clasificar)


def mano(angulo=0.0, estirados=(True, False, False, False), giro=None):
    """
    Mano sintética de 21 puntos. `estirados` = (índice, corazón, anular, meñique). Los dedos estirados
    apuntan hacia `angulo` (grados, 0 = derecha en la imagen, 90 = arriba); la palma hacia `giro`
    (por defecto igual). Coordenadas normalizadas con y hacia abajo.
    """
    giro = angulo if giro is None else giro
    cx, cy = 0.5, 0.5
    pts = [(0.0, 0.0)] * 21

    def p(r, a):
        a = math.radians(a)
        return (cx + r * math.cos(a), cy - r * math.sin(a))

    pts[0] = (cx, cy)
    pts[1], pts[2], pts[3], pts[4] = p(0.05, giro + 40), p(0.08, giro + 45), p(0.10, giro + 50), p(0.12, giro + 55)
    for k, a, est in zip((5, 9, 13, 17), (giro + 15, giro + 5, giro - 5, giro - 15), estirados):
        mcp = p(0.10, a)
        pts[k] = mcp
        if est:
            ar = math.radians(angulo)
            for j, r in enumerate((0.04, 0.07, 0.10), start=1):
                pts[k + j] = (mcp[0] + r * math.cos(ar), mcp[1] - r * math.sin(ar))
        else:   # doblado: PIP sale un poco, DIP y punta vuelven hacia la muñeca
            pts[k + 1], pts[k + 2], pts[k + 3] = p(0.13, a), p(0.10, a), p(0.08, a)
    return pts


UNO = (True, False, False, False)
DOS = (True, True, False, False)
TRES = (True, True, True, False)
TODOS = (True, True, True, True)


class TestClasificar(unittest.TestCase):
    def test_siete_direcciones(self):
        casos = {0: DER, 45: ARR_DER, 135: ARR_IZQ, 180: IZQ, -135: ABA_IZQ, -90: ABA, -45: ABA_DER}
        for ang, esperado in casos.items():
            self.assertEqual(clasificar(mano(ang, UNO))[0], esperado, ang)
            self.assertEqual(clasificar(mano(ang + 15, UNO, giro=ang))[0], esperado, ang)   # con margen

    def test_indice_arriba_no_cuenta(self):
        self.assertEqual(clasificar(mano(90, UNO))[0], NINGUNO)
        self.assertEqual(clasificar(mano(100, UNO))[0], NINGUNO)

    def test_entre_dos_direcciones_no_cuenta(self):
        self.assertEqual(clasificar(mano(67, UNO))[0], NINGUNO)
        self.assertEqual(clasificar(mano(-158, UNO))[0], NINGUNO)

    def test_avanzar(self):
        self.assertEqual(clasificar(mano(90, DOS))[0], LENTO)
        self.assertEqual(clasificar(mano(110, DOS))[0], LENTO)
        self.assertEqual(clasificar(mano(90, TRES))[0], RAPIDO)
        self.assertEqual(clasificar(mano(75, TRES))[0], RAPIDO)

    def test_stop(self):
        self.assertEqual(clasificar(mano(90, TODOS))[0], STOP)
        self.assertEqual(clasificar(mano(110, TODOS))[0], STOP)

    def test_avanzar_y_stop_solo_hacia_arriba(self):
        for dedos in (DOS, TRES, TODOS):
            self.assertEqual(clasificar(mano(0, dedos))[0], NINGUNO)
            self.assertEqual(clasificar(mano(-90, dedos))[0], NINGUNO)

    def test_otros_no_cuentan(self):
        self.assertEqual(clasificar(mano(90, (False,) * 4))[0], NINGUNO)               # puño
        self.assertEqual(clasificar(mano(90, (True, False, False, True)))[0], NINGUNO)  # cuernos
        self.assertEqual(clasificar(mano(90, (False, True, False, False)))[0], NINGUNO)  # solo corazón
        self.assertEqual(clasificar(None)[0], NINGUNO)


class TestEstabilizador(unittest.TestCase):
    def test_confirma_y_suelta(self):
        e = Estabilizador(n=3, ventana=4, n_stop=2)
        self.assertEqual(e.update(DER), NINGUNO)
        e.update(DER)
        self.assertEqual(e.update(DER), DER)
        self.assertEqual(e.update(NINGUNO), DER)         # un cuadro malo no lo suelta
        for _ in range(4):
            e.update(NINGUNO)
        self.assertEqual(e.actual, NINGUNO)

    def test_cambia_de_lento_a_rapido(self):
        e = Estabilizador(n=3, ventana=4)
        for _ in range(4):
            e.update(LENTO)
        for _ in range(3):
            e.update(RAPIDO)
        self.assertEqual(e.actual, RAPIDO)

    def test_stop_gana_rapido(self):
        e = Estabilizador(n=3, ventana=4, n_stop=2)
        for _ in range(4):
            e.update(RAPIDO)
        e.update(STOP)
        self.assertEqual(e.update(STOP), STOP)

    def test_ruido_no_confirma(self):
        e = Estabilizador(n=3, ventana=4)
        for g in (DER, IZQ, LENTO, ARR_DER, NINGUNO):
            self.assertEqual(e.update(g), NINGUNO)


class RobotFalso:
    """Integra (v, w) para probar el control en bucle cerrado."""

    def __init__(self, yaw=0.0):
        self.x = self.y = 0.0
        self.yaw = yaw

    def correr(self, ctrl, gesto, segundos, dt=0.05):
        v = w = 0.0
        for _ in range(int(segundos / dt)):
            v, w = ctrl.paso(gesto, self.yaw)
            self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        return v, w


def grados(a):
    return math.degrees(a)


class TestControl(unittest.TestCase):
    def assertYaw(self, r, deg):
        dif = (grados(r.yaw) - deg + 180) % 360 - 180
        self.assertLess(abs(dif), 4, f"yaw {grados(r.yaw):.1f}, esperaba {deg}")

    def test_parado_al_inicio(self):
        self.assertEqual(Control().paso(NINGUNO, 0.0), (0.0, 0.0))

    def test_senalar_solo_gira_y_se_queda_quieto(self):
        c, r = Control(), RobotFalso()
        v, w = c.paso(DER, r.yaw)
        self.assertEqual((c.estado, v), (GIRANDO, 0.0))
        self.assertLess(w, 0)                            # derecha = w negativo
        self.assertEqual(r.correr(c, DER, 6), (0.0, 0.0))
        self.assertEqual(c.estado, PARADO)
        self.assertYaw(r, -90)
        self.assertAlmostEqual(r.x, 0.0)

    def test_giros_de_cada_direccion(self):
        casos = {IZQ: 90, DER: -90, ARR_DER: -45, ARR_IZQ: 45, ABA_DER: -135, ABA_IZQ: 135, ABA: 180}
        for g, deg in casos.items():
            c, r = Control(), RobotFalso()
            r.correr(c, g, 10)
            self.assertEqual(c.estado, PARADO, g)
            self.assertYaw(r, deg)

    def test_v_avanza_lento_y_3_dedos_rapido(self):
        c, r = Control(v_lento=0.07, v_rapido=0.15), RobotFalso(yaw=0.3)
        v, _ = r.correr(c, LENTO, 2)
        self.assertEqual(c.estado, AVANZANDO)
        self.assertAlmostEqual(v, 0.07)
        v, _ = r.correr(c, RAPIDO, 2)
        self.assertAlmostEqual(v, 0.15)
        self.assertAlmostEqual(r.yaw, 0.3, delta=math.radians(2))
        self.assertGreater(r.x, 0.3)

    def test_girar_y_luego_avanzar(self):
        c, r = Control(), RobotFalso()
        r.correr(c, ARR_IZQ, 5)
        r.correr(c, NINGUNO, 1)
        r.correr(c, LENTO, 4)
        self.assertYaw(r, 45)
        self.assertGreater(r.y, 0.1)                     # avanzó en diagonal
        self.assertGreater(r.x, 0.1)

    def test_avanzar_mientras_gira_termina_el_giro(self):
        c, r = Control(), RobotFalso()
        r.correr(c, IZQ, 0.5)
        self.assertEqual(c.estado, GIRANDO)
        v, _ = r.correr(c, RAPIDO, 6)
        self.assertEqual(c.estado, AVANZANDO)
        self.assertGreater(v, 0)
        self.assertYaw(r, 90)

    def test_senalar_mientras_avanza_para_y_gira(self):
        c, r = Control(), RobotFalso()
        r.correr(c, LENTO, 2)
        v, _ = r.correr(c, DER, 0.2)
        self.assertEqual((c.estado, v), (GIRANDO, 0.0))
        r.correr(c, DER, 6)
        self.assertEqual(c.estado, PARADO)
        self.assertYaw(r, -90)

    def test_sin_mano_sigue_avanzando(self):
        c, r = Control(), RobotFalso()
        r.correr(c, LENTO, 1)
        v, _ = r.correr(c, NINGUNO, 2)
        self.assertGreater(v, 0)

    def test_mantener_senal_no_repite_giro(self):
        c, r = Control(), RobotFalso()
        r.correr(c, DER, 15)
        self.assertYaw(r, -90)

    def test_volver_a_senalar_gira_otra_vez(self):
        c, r = Control(), RobotFalso()
        r.correr(c, DER, 6)
        r.correr(c, NINGUNO, 0.5)
        r.correr(c, DER, 6)
        self.assertYaw(r, 180)

    def test_stop_para_aunque_este_girando_o_avanzando(self):
        c, r = Control(), RobotFalso()
        r.correr(c, ABA, 0.5)
        self.assertEqual(c.paso(STOP, r.yaw), (0.0, 0.0))
        self.assertEqual(c.estado, PARADO)
        r.correr(c, LENTO, 1)
        r.correr(c, STOP, 0.2)
        self.assertEqual(r.correr(c, NINGUNO, 2), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
