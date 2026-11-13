"""
Pruebas sin robot del reconocimiento facial (reconocimiento/).
  python -m unittest discover tests -v      (desde la raiz del repositorio)
"""
import unittest

import cv2
import numpy as np

from reconocimiento import (ROSTROS, Galeria, Reconocedor, Votacion, alinear, galeria_path, nombre_bonito,
                            peso_por_tamano)
from reconocimiento.degradar import simular_camara


def una_foto():
    carpeta = sorted(d for d in ROSTROS.iterdir() if d.is_dir())[0]
    return nombre_bonito(carpeta.name), carpeta.name, sorted(carpeta.glob("*.jpg"))[0]


class VotacionTests(unittest.TestCase):
    def test_lider_y_decision(self):
        v = Votacion(["Ana", "Beto", "Carla"], min_fotos=2, prob=0.97)
        self.assertFalse(v.decidido())
        v.agregar(np.array([0.6, 0.2, 0.1]))
        self.assertFalse(v.decidido())                 # una sola foto no alcanza
        v.agregar(np.array([0.55, 0.25, 0.1]))
        self.assertTrue(v.decidido())
        self.assertEqual(v.lider()[0], "Ana")

    def test_empate_no_decide_antes_del_maximo(self):
        v = Votacion(["Ana", "Beto"], min_fotos=2, max_fotos=4)
        for _ in range(3):
            v.agregar(np.array([0.4, 0.4]))
        self.assertFalse(v.decidido())
        v.agregar(np.array([0.4, 0.4]))
        self.assertTrue(v.decidido())                  # al maximo de fotos decide el lider (conjunto cerrado)

    def test_fotos_chicas_pesan_menos(self):
        self.assertLess(peso_por_tamano(14), peso_por_tamano(30))
        self.assertEqual(peso_por_tamano(80), 1.0)
        v = Votacion(["Ana", "Beto"])
        v.agregar(np.array([0.2, 0.5]), peso=peso_por_tamano(12))     # foto lejana dice Beto
        v.agregar(np.array([0.6, 0.2]), peso=peso_por_tamano(60))     # foto cercana dice Ana
        self.assertEqual(v.lider()[0], "Ana")


class GaleriaTests(unittest.TestCase):
    def test_galeria_versionada_coincide_con_las_fotos(self):
        g = Galeria.cargar(galeria_path("mbf"))
        carpetas = sorted(nombre_bonito(d.name) for d in ROSTROS.iterdir() if d.is_dir())
        self.assertEqual(sorted(g.nombres), carpetas)
        self.assertEqual(len(g.emb), len(g.etiqueta))
        np.testing.assert_allclose(np.linalg.norm(g.emb, axis=1), 1.0, atol=1e-3)

    def test_similitud_por_persona(self):
        g = Galeria(["A", "B"], np.eye(3, dtype=np.float32)[:3], np.array([0, 0, 1]), ["a1", "a2", "b1"])
        s = g.similitudes(np.array([0, 0, 1.0], np.float32))
        self.assertAlmostEqual(float(s[1]), 1.0)
        s = g.similitudes(np.array([0, 0, 1.0], np.float32), excluir=np.array([False, False, True]))
        self.assertLess(float(s[1]), 0.0)              # excluida: la persona B queda sin fotos


class ReconocedorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nombre, cls.carpeta, cls.foto = una_foto()
        cls.rec = Reconocedor("mbf", excluir=(f"{cls.carpeta}/{cls.foto.name}",))

    def test_reconoce_una_foto_que_no_esta_en_la_galeria(self):
        img = cv2.imread(str(self.foto))
        r = self.rec.mejor(img)
        self.assertIsNotNone(r)
        self.assertEqual(r.nombre, self.nombre)

    def test_reconoce_como_la_veria_el_robot(self):
        img = cv2.imread(str(self.foto))
        cara = self.rec.det(img)[0]
        rng = np.random.default_rng(0)
        v = Votacion(self.rec.nombres)
        for _ in range(3):
            cuadro, _ = simular_camara(img, cara.caja, 40, rng)       # cara de 40 px en 250x250, impresa, JPEG
            r = self.rec.mejor(cuadro)
            self.assertIsNotNone(r)
            v.agregar(r.sims, peso_por_tamano(r.cara.ancho))
        self.assertEqual(v.lider()[0], self.nombre)

    def test_alineacion_112(self):
        img = cv2.imread(str(self.foto))
        self.assertEqual(alinear(img, self.rec.det(img)[0]).shape, (112, 112, 3))

    def test_sin_cara(self):
        self.assertIsNone(self.rec.mejor(np.full((250, 250, 3), 128, np.uint8)))


if __name__ == "__main__":
    unittest.main()
