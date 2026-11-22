"""Senales del 21/11 (PDF del profe): disco AZUL con flecha (ida: cuenta), disco ROJO con flecha (vuelta: no),
discos lisos y prohibido (no), y la BANDERA a cuadros de la meta (foto del laboratorio). Sin falsos positivos
con un QR ni con textos de cajas."""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from bandera import DetectorBandera                                       # noqa: E402
from flechas import DetectorFlechas                                       # noqa: E402
import mundos                                                             # noqa: E402

HERE = Path(__file__).parent


def _en_fondo(hoja, lado, rng, ancho=None):
    fondo = np.full((250, 250, 3), 150, np.uint8)
    w = ancho or lado
    sc = cv2.resize(hoja, (w, lado), interpolation=cv2.INTER_AREA)
    m = max(4, int(lado * 0.25))
    x0, y0 = int(rng.integers(m + 5, 240 - w - m)), int(rng.integers(m + 5, 240 - lado - m))
    fondo[y0 - m:y0 + lado + m, x0 - m:x0 + w + m] = 235
    fondo[y0:y0 + lado, x0:x0 + w] = sc
    return cv2.GaussianBlur(fondo, (3, 3), 0.6)


def _disco(color, flecha, rng, direccion="der"):
    tex = mundos.textura_disco(direccion, rng, color, flecha, lado_m=0.2)
    return tex


def test_disco_azul_con_flecha_se_detecta_y_rojo_lisos_prohibido_no():
    rng = np.random.default_rng(3)
    det = DetectorFlechas()
    ok = 0
    for lado in (20, 28, 40, 60):
        for direccion in ("der", "izq"):
            fs = det(_en_fondo(_disco("azul", True, rng, direccion), lado, rng))
            if fs and fs[0].dir == direccion and fs[0].color == "azul":
                ok += 1
            assert not any(f.dir != direccion for f in fs), (lado, direccion, fs)
    assert ok >= 6
    for hoja in (_disco("rojo", True, rng), _disco("azul", False, rng), _disco("rojo", False, rng),
                 mundos.textura_prohibido(rng, 0.2)):
        for lado in (20, 32, 50, 80):
            assert det(_en_fondo(hoja, lado, rng)) == [], (lado,)


def test_bandera_del_laboratorio_se_detecta_y_qr_texto_no():
    det = DetectorBandera()
    foto = cv2.imread(str(HERE / "datos_prueba" / "bandera_lab_2026-11-21.jpg"))
    assert foto is not None
    im = cv2.resize(foto, (250, 187), interpolation=cv2.INTER_AREA)
    assert det(im), "la bandera del lab a 250 px"
    rng = np.random.default_rng(1)
    hoja = mundos.textura_bandera(rng)
    for ancho in (45, 60, 90, 115):        # 115 px ~ 0.9 m (donde se para a mirar); mas cerca no hace falta
        alto = int(ancho * hoja.shape[0] / hoja.shape[1])
        fondo = np.full((250, 250, 3), mundos.CARTON, np.uint8)              # hoja pegada en una caja
        fondo[40:40 + alto, 30:30 + ancho] = cv2.resize(hoja, (ancho, alto), interpolation=cv2.INTER_AREA)
        assert det(cv2.GaussianBlur(fondo, (3, 3), 0.7)), ancho
    # QR y texto: no
    qr = cv2.imread(str(HERE / "datos_prueba" / "qr_prueba.png"))
    for W in (60, 120):
        q = cv2.resize(qr, (W, W), interpolation=cv2.INTER_AREA)
        fondo = np.full((W + 60, W + 60, 3), 140, np.uint8)
        fondo[30:-30, 30:-30] = q
        assert det(fondo) == []
    for txt in ("TAMBO", "ESTE LADO", "FRAGIL x12"):
        fondo = np.full((250, 250, 3), 140, np.uint8)
        fondo[80:140, 30:220] = 235
        cv2.putText(fondo, txt, (35, 125), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3, cv2.LINE_AA)
        assert det(cv2.GaussianBlur(fondo, (3, 3), 0.7)) == [], txt


def test_bandera_sin_carton_alrededor_no_cuenta():
    """(22/11) la bandera de verdad es una hoja blanca sobre carton: la misma hoja sobre una pared gris no cuenta."""
    det = DetectorBandera(sobre_carton=True)                            # como requisito (en la carrera es confianza)
    hoja = mundos.textura_bandera(np.random.default_rng(2))
    alto = int(80 * hoja.shape[0] / hoja.shape[1])
    for fondo_col, esperado in ((mundos.CARTON, True), ((150, 150, 150), False), ((200, 200, 200), False)):
        fondo = np.full((250, 250, 3), fondo_col, np.uint8)
        fondo[60:60 + alto, 60:140] = cv2.resize(hoja, (80, alto), interpolation=cv2.INTER_AREA)
        assert bool(det(cv2.GaussianBlur(fondo, (3, 3), 0.7))) is esperado, fondo_col
