"""
Simula como ve la camara del robot una foto impresa: para evaluar el reconocimiento sin robot y
para aumentar la galeria (galeria.py --aumentar).

La camara del TurtleBot 4 (preview de la OAK-D) da 250x250 px; una cara impresa de ~15 cm ocupa
~50 px a 1 m y ~25 px a 2 m. Ademas la foto impresa pierde contraste, toma el color de la luz,
se ve algo inclinada y el JPEG del puente le quita detalle.
"""
import cv2
import numpy as np


def efecto_impresion(img, rng):
    """Contraste y color de una foto impresa bajo luz de aula."""
    f = img.astype(np.float32)
    contraste = rng.uniform(0.65, 0.95)
    brillo = rng.uniform(-25, 30)
    f = (f - 128.0) * contraste + 128.0 + brillo
    f *= rng.uniform(0.9, 1.1, size=3)                       # tinte de la luz (B, G, R)
    gamma = rng.uniform(0.8, 1.25)
    f = 255.0 * (np.clip(f, 0, 255) / 255.0) ** gamma
    return np.clip(f, 0, 255).astype(np.uint8)


def simular_camara(img, caja, ancho_px, rng, lado=250, inclinacion=0.12, ruido=3.0, jpeg=(60, 90)):
    """Encuadra la foto `img` (con la cara en `caja` = x, y, w, h) como la veria el robot: la cara
    queda de `ancho_px` de ancho, en una posicion al azar de un cuadro de `lado` x `lado`.
    Devuelve (cuadro BGR, caja esperada de la cara en el cuadro)."""
    x, y, w, h = caja
    s = ancho_px / max(w, 1.0)
    cx, cy = (x + w / 2) * s, (y + h / 2) * s
    # destino de la cara dentro del cuadro (sin salirse)
    m = max(ancho_px * 0.8, 10)
    tx, ty = rng.uniform(m, lado - m), rng.uniform(m, lado - m)
    # semejanza (escala + traslacion) + algo de perspectiva (foto no del todo de frente)
    A = np.array([[s, 0, tx - cx], [0, s, ty - cy], [0, 0, 1]], np.float64)
    k = inclinacion
    P = np.array([[1 + rng.uniform(-k, k) * 0.3, rng.uniform(-k, k) * 0.3, 0],
                  [rng.uniform(-k, k) * 0.3, 1, 0],
                  [rng.uniform(-k, k) / lado, rng.uniform(-k, k) / lado, 1]], np.float64)
    C = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]], np.float64)
    Ci = np.array([[1, 0, -tx], [0, 1, -ty], [0, 0, 1]], np.float64)
    H = C @ P @ Ci @ A                                        # perspectiva alrededor de la cara
    fondo = int(rng.uniform(60, 200))
    interp = cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR
    if s < 0.5:                                               # reducir antes evita el aliasing
        pre = cv2.resize(img, None, fx=s * 2, fy=s * 2, interpolation=cv2.INTER_AREA)
        H = H @ np.diag([1 / (s * 2), 1 / (s * 2), 1])
        src = pre
    else:
        src = img
    out = cv2.warpPerspective(src, H, (lado, lado), flags=interp, borderMode=cv2.BORDER_CONSTANT,
                              borderValue=(fondo, fondo, fondo))
    out = efecto_impresion(out, rng)
    sigma = rng.uniform(0.3, 0.9)
    out = cv2.GaussianBlur(out, (0, 0), sigma)
    out = np.clip(out.astype(np.float32) + rng.normal(0, ruido, out.shape), 0, 255).astype(np.uint8)
    q = int(rng.uniform(*jpeg))
    ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, q])
    out = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    esperada = (tx - ancho_px / 2, ty - h * s / 2, ancho_px, h * s)
    return out, esperada


def baja_resolucion(cara112, ancho_px, rng):
    """Cara alineada 112x112 -> como quedaria si en la imagen original midiera `ancho_px`."""
    s = max(ancho_px, 8) / 112.0 * 1.6                        # la caja de YuNet es ~1/1.6 del recorte
    chica = cv2.resize(cara112, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    chica = efecto_impresion(chica, rng)
    ok, buf = cv2.imencode(".jpg", chica, [cv2.IMWRITE_JPEG_QUALITY, int(rng.uniform(60, 90))])
    chica = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return cv2.resize(chica, (112, 112), interpolation=cv2.INTER_CUBIC)


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    return inter / max(aw * ah + bw * bh - inter, 1e-9)
