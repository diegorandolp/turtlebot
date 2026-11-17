"""Pruebas de las reglas del laboratorio (sin robot): varias fotos en la meta, cada una con un QR en una
esquina de abajo; buscar a una persona, leer SU QR y anunciar la palabra."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent / "auto-1", HERE.parent / "slam-1", HERE.parents[1]):
    sys.path.insert(0, str(p))

from mundos import construir, imagen_qr                                   # noqa: E402
from navegador import ACERCAR, FILA, IDA, LEER_QR, VOLVER, Carrera, Config        # noqa: E402
from percepcion import Obs, Objetivo                                        # noqa: E402
from qr import _de_que_cara, buscar_qrs                                     # noqa: E402
from reconocimiento import Votacion                                         # noqa: E402

NOMBRES = ["Ana Uno", "Beto Dos", "Caro Tres", "Dani Cuatro"]


def pared_con_fotos(textos, cara_px=60, sep_px=None):
    """Imagen 1280x720 con fotos en fila: un 'retrato' gris con una cara (rectangulo) y un QR real en la
    esquina de abajo de cada una. -> (img, cajas de las caras)."""
    img = np.full((720, 1280, 3), 190, np.uint8)
    hoja_w, hoja_h = int(3 * cara_px), int(3.8 * cara_px)
    sep_px = sep_px or hoja_w + 40
    x0 = (1280 - sep_px * (len(textos) - 1) - hoja_w) // 2
    cajas = []
    for i, t in enumerate(textos):
        x = x0 + i * sep_px
        y = 150
        cv2.rectangle(img, (x, y), (x + hoja_w, y + hoja_h), (240, 240, 240), -1)
        cx = x + (hoja_w - cara_px) // 2
        cajas.append((cx, y + 20, cara_px, int(1.2 * cara_px)))
        cv2.rectangle(img, (cx, y + 20), (cx + cara_px, y + 20 + int(1.2 * cara_px)), (120, 140, 170), -1)
        lado = int(1.4 * cara_px)
        q = imagen_qr(t, lado)
        qx = x + 4 if i % 2 == 0 else x + hoja_w - lado - 4                  # esquina izquierda o derecha
        img[y + hoja_h - lado - 4:y + hoja_h - 4, qx:qx + lado] = q
    return img, cajas


class LecturaQR(unittest.TestCase):
    def test_lee_y_asigna_cada_qr_a_su_foto(self):
        textos = ["milestone_alfa", "milestone_beta", "milestone_gamma"]
        img, cajas = pared_con_fotos(textos)
        qrs = buscar_qrs(img, cajas)
        self.assertEqual(sorted(q.texto for q in qrs), sorted(textos))
        for q in qrs:
            self.assertEqual(q.cara, textos.index(q.texto), q)

    def test_fotos_casi_pegadas_no_se_mezclan(self):
        """Con las hojas a 10 px una de otra, el QR de la vecina no se asigna a la cara equivocada."""
        textos = ["milestone_rho", "milestone_phi", "milestone_tau"]
        img, cajas = pared_con_fotos(textos, sep_px=int(3 * 60) + 10)
        for q in buscar_qrs(img, cajas):
            self.assertIn(q.cara, (None, textos.index(q.texto)), q)

    def test_sin_cara_vecina_el_qr_de_al_lado_no_es_de_la_buscada(self):
        """Si la cara vecina no se detecto, su QR (a >= 2.5 caras de lado) no se le asigna a la que si se ve."""
        self.assertIsNone(_de_que_cara((300, 300, 80, 80), [(100, 100, 60, 70)]))
        self.assertEqual(_de_que_cara((80, 250, 80, 80), [(100, 100, 60, 70)]), 0)

    def test_sin_caras_busca_en_toda_la_imagen(self):
        img, _ = pared_con_fotos(["milestone_kappa"])
        qrs = buscar_qrs(img, [])
        self.assertEqual([q.texto for q in qrs], ["milestone_kappa"])
        self.assertIsNone(qrs[0].cara)


def res(nombre, margen=0.3, px=50):
    sims = np.array([0.6 if n == nombre else 0.6 - margen for n in NOMBRES])
    return SimpleNamespace(nombre=nombre, sim=0.6, margen=margen, sims=sims)


def cara(t, xy, nombre, px=50, margen=0.3):
    return Obs("cara", t, 0.0, 1.0, xy, px, (0, 0, px, px), res=res(nombre, margen))


def qr(t, texto, de_cara):
    return Obs("qr", t, 0.0, 1.0, de_cara.xy, 60, (0, 0, 60, 60), texto=texto, cara=de_cara)


def navegador(buscar="Caro Tres"):
    v = Votacion(NOMBRES)
    nav = Carrera(Config(buscar=buscar), None, v, Objetivo(v), None, None)
    nav.empezar(0.0, 0.0)
    nav._pos = (0.0, 0.0, 0.0)
    return nav


class Candidatas(unittest.TestCase):
    def test_descarta_otra_y_va_a_la_buscada(self):
        nav = navegador()
        for k in range(4):                                           # una foto de Ana: "otra"
            nav.ver_cara(0.1 * k, cara(0.1 * k, (2.0, 0.0), "Ana Uno"), 1.0)
        self.assertEqual(nav.candidatas[0].estado, "otra")
        for k in range(4):                                           # la de Caro, 0.45 m al lado: la buscada
            nav.ver_cara(1 + 0.1 * k, cara(1 + 0.1 * k, (2.0, 0.45), "Caro Tres"), 1.0)
        self.assertEqual(len(nav.candidatas), 2)
        self.assertEqual(nav.candidatas[1].estado, "objetivo")
        self.assertIs(nav.obj, nav.candidatas[1])
        self.assertEqual(nav.estado, ACERCAR)

    def test_fotos_pegadas_de_personas_distintas_no_se_juntan(self):
        """Dos fotos a 0.25 m (menos que d_candidata): si cada una es claramente de otra persona, no se mezclan."""
        nav = navegador()
        for k in range(4):
            nav.ver_cara(0.1 * k, cara(0.1 * k, (2.0, 0.0), "Ana Uno"), 1.0)
        for k in range(4):
            nav.ver_cara(1 + 0.1 * k, cara(1 + 0.1 * k, (2.0, 0.25), "Caro Tres"), 1.0)
        self.assertEqual([c.estado for c in nav.candidatas], ["otra", "objetivo"])

    def test_cara_chica_no_se_descarta(self):
        """De lejos (cara chica) una foto no se da por 'otra': podria ser la buscada mal reconocida."""
        nav = navegador()
        for k in range(5):
            nav.ver_cara(0.1 * k, cara(0.1 * k, (3.0, 0.0), "Ana Uno", px=14), 1.0)
        self.assertEqual(nav.candidatas[0].estado, "pendiente")

    def test_sin_otras_fotos_mira_a_los_costados_y_sigue_buscando(self):
        nav = navegador()
        for k in range(4):
            nav.ver_cara(0.1 * k, cara(0.1 * k, (2.0, 0.0), "Ana Uno"), 1.0)
        # ya se sabe que es otra: recorrer la fila de fotos (las de la meta estan juntas) en vez de irse
        self.assertEqual(nav.estado, FILA)
        self.assertGreaterEqual(len(nav._fila["puntos"]), 2)


class Robustez(unittest.TestCase):
    def test_buscada_perdida_no_hace_bucle(self):
        """Si la buscada no se encuentra donde se la vio, se deja de lado un rato: una cara cualquiera no
        la vuelve a elegir enseguida (antes: ACERCAR -> IDA en cada foto)."""
        nav = navegador()
        for k in range(4):
            nav.ver_cara(0.1 * k, cara(0.1 * k, (2.0, 0.45), "Caro Tres"), 1.0)
        self.assertEqual(nav.estado, ACERCAR)
        nav.obj.t_ignorar = 100.0                                    # lo que hace _acercar tras 30 s sin verla
        nav.estado = IDA
        nav.ver_cara(50.0, cara(50.0, (0.0, 3.0), "Ana Uno", px=12), 1.0)   # otra cara lejos
        self.assertIn(nav.estado, (IDA, FILA))                       # puede revisar la fila, pero no ACERCAR
        nav.ver_cara(51.0, cara(51.0, (2.0, 0.45), "Caro Tres"), 1.0)      # la vuelve a ver: vale otra vez
        self.assertEqual(nav.estado, ACERCAR)

    def test_camara_sigue_con_preview_si_la_hd_no_llega(self):
        import time
        from percepcion import CamaraHilo

        class Robot:
            def __init__(self):
                self.pedidos = []

            def photo(self, timeout=2.0, hd=False):
                self.pedidos.append(hd)
                if hd:
                    raise TimeoutError("sin HD")
                return np.zeros((250, 250, 3), np.uint8)

        class Per:
            def procesar(self, *a, **k):
                return [], [], []

        r = Robot()
        cam = CamaraHilo(r, Per(), lambda: ((0.0, 0.0, 0.0), None, 0.0, 0.0))
        cam.qr, cam.hd_qr = True, True
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and len(cam.recoger()) < 1:
            time.sleep(0.02)
        time.sleep(0.2)
        cam.close()
        self.assertGreaterEqual(cam.fotos, 3)                        # nunca se queda sin fotos
        self.assertEqual(r.pedidos.count(True), 2)                   # dos intentos HD y despues solo preview
        self.assertIn("HD", cam.ultimo_error)


class AceptarQR(unittest.TestCase):
    def preparado(self):
        nav = navegador()
        for k in range(4):
            nav.ver_cara(0.1 * k, cara(0.1 * k, (2.0, 0.45), "Caro Tres"), 1.0)
        nav.estado = LEER_QR
        return nav

    def test_dos_lecturas_junto_a_la_buscada(self):
        nav = self.preparado()
        c = cara(1.0, (2.0, 0.45), "Caro Tres")
        nav.ver_qr(1.0, qr(1.0, "milestone_ok", c))
        self.assertIsNone(nav.palabra)                              # una sola lectura no alcanza
        nav.ver_qr(1.2, qr(1.2, "milestone_ok", c))
        self.assertEqual(nav.palabra, "milestone_ok")
        self.assertEqual(nav.nombre, "Caro Tres")
        self.assertEqual(nav.estado, VOLVER)

    def test_texto_visto_junto_a_otra_persona_nunca_se_acepta(self):
        nav = self.preparado()
        otra = cara(1.0, (2.0, 0.0), "Ana Uno")
        nav.ver_qr(1.0, qr(1.0, "milestone_vecina", otra))
        buscada = cara(1.1, (2.0, 0.45), "Caro Tres")
        for k in range(4):                                          # mal asociado a la buscada varias veces
            nav.ver_qr(1.1 + k, qr(1.1 + k, "milestone_vecina", buscada))
        self.assertIsNone(nav.palabra)

    def test_dos_textos_en_conflicto_necesitan_ventaja(self):
        nav = self.preparado()
        b = cara(1.0, (2.0, 0.45), "Caro Tres")
        nav.ver_qr(1.0, qr(1.0, "milestone_a", b))
        nav.ver_qr(1.1, qr(1.1, "milestone_b", b))
        nav.ver_qr(1.2, qr(1.2, "milestone_a", b))
        self.assertIsNone(nav.palabra)                              # 2 a 1: todavia no
        nav.ver_qr(1.3, qr(1.3, "milestone_a", b))
        self.assertEqual(nav.palabra, "milestone_a")                # 3 a 1

    def test_reconocimiento_dudoso_no_cuenta(self):
        nav = self.preparado()
        b = cara(1.0, (2.0, 0.45), "Caro Tres", margen=0.02)
        for k in range(4):
            nav.ver_qr(1.0 + k, qr(1.0 + k, "milestone_x", b))
        self.assertIsNone(nav.palabra)

    def test_si_no_lo_lee_vuelve_igual(self):
        nav = self.preparado()
        nav._qr = dict(paso=0, t0=0.0, th0=0.0, t_ini=0.0)
        nav._paso(nav.cfg.qr_max_s + 1.0, (0.0, 0.0, 0.0), None, None, 0, True)
        self.assertEqual(nav.estado, VOLVER)
        self.assertIsNone(nav.palabra)


class MundosConQR(unittest.TestCase):
    def test_meta_con_seis_fotos(self):
        m = construir("croquis+6")
        self.assertEqual(len(m.rostros), 6)
        self.assertEqual(len({r[0] for r in m.rostros}), 6)        # personas distintas
        self.assertEqual(len({r[1] for r in m.rostros}), 6)        # palabras distintas
        self.assertIn(m.persona, [r[0] for r in m.rostros])
        self.assertEqual(m.qr, dict((r[0], r[1]) for r in m.rostros)[m.persona])
        self.assertEqual(len(m.fotos_excluir), 6)

    def test_separacion(self):
        m = construir("denso:3+6:0.3")
        ys = sorted(r[2][1] for r in m.rostros)
        self.assertTrue(all(abs((b - a) - 0.3) < 1e-6 for a, b in zip(ys, ys[1:])))


class PuntaAPunta(unittest.TestCase):
    def test_croquis_con_seis_fotos(self):
        import carrera
        args = carrera.parse(["--sim", "--go", "--no-gui", "--mudo", "--no-guardar", "--modelo", "mbf",
                              "--mundo", "croquis+6"])
        m, robot = carrera.crear_sim(args)
        r = carrera.correr(robot, args, sim=True, gui=False, mundo=m, max_t=180)
        self.assertTrue(r["exito"], r["log"][-4:])
        self.assertEqual(r["palabra"], m.qr)
        self.assertEqual(r["choques"], 0)
        self.assertLess(r["tiempo"], 150)            # croquis fiel: la buscada no es la primera


class NombreEnGaleria(unittest.TestCase):
    def test_sin_tildes_ni_mayusculas(self):
        import carrera
        self.assertEqual(carrera.nombre_en_galeria("alvaro garcia", ["Alvaro García", "Ana Uno"]), "Alvaro García")
        self.assertEqual(carrera.nombre_en_galeria("Alvaro_Garcia", ["Alvaro García"]), "Alvaro García")
        with self.assertRaises(SystemExit):
            carrera.nombre_en_galeria("Nadie", ["Alvaro García", "Ana Uno"])


if __name__ == "__main__":
    unittest.main()


class MundosDelLaboratorio(unittest.TestCase):
    def test_lab_pone_la_meta_en_distintas_paredes(self):
        paredes = set()
        for k in range(1, 9):
            m = construir(f"lab:{k}+6")
            xs = {round(p[0], 1) for _, _, p in m.rostros}
            ys = {round(p[1], 1) for _, _, p in m.rostros}
            paredes.add("der" if len(xs) == 1 else ("arriba/abajo" if len(ys) == 1 else "?"))
        self.assertIn("arriba/abajo", paredes)
        self.assertIn("der", paredes)

    def test_aspecto_del_laboratorio(self):
        from mundos import CARTON, SUELO_LAB
        m = construir("croquis+6")
        self.assertEqual(m.suelo, SUELO_LAB)
        self.assertTrue(any(tuple(int(c) for c in p.color) == CARTON for p in m.paredes
                            if hasattr(p, "color")))


class FlechasDeColor(unittest.TestCase):
    def test_flecha_de_color_se_lee_por_su_forma(self):
        """En el laboratorio las senales pueden ser de color: lo que cuenta es la forma."""
        from flechas import DetectorFlechas
        from mundos import COLORES_SENAL, textura_flecha
        det = DetectorFlechas(forma=True)
        ok = 0
        for i, col in enumerate(COLORES_SENAL):
            for d in ("izq", "der"):
                tex = textura_flecha(d, np.random.default_rng(i), estilo="giro", color=col)
                img = np.full((250, 250, 3), (95, 140, 178), np.uint8)            # pared de carton
                img[160:] = 115                                                     # piso gris
                img[100:150, 100:150] = cv2.resize(tex, (50, 50), interpolation=cv2.INTER_AREA)
                ok += any(f.dir == d for f in det(cv2.GaussianBlur(img, (3, 3), 0)))
        self.assertEqual(ok, 8)


class Laberinto(unittest.TestCase):
    def test_laberintos_largos_con_senales_y_meta(self):
        for k in range(1, 6):
            m = construir(f"laberinto:{k}+6")
            self.assertEqual(len(m.rostros), 6)
            self.assertTrue(all(abs(p[0] - 5.6) < 1e-6 for _, _, p in m.rostros))   # fotos en la pared derecha
            self.assertLess(m.inicio[0], 0.8)                                          # A en la columna izquierda
            self.assertGreaterEqual(sum("flecha" in c.tipo for c in m.carteles), 2)   # senales en los giros
        self.assertEqual(construir("laberinto:1:6:5:0.7+6").nombre.split("+")[0], "laberinto:1:6:5:0.7")
