"""
Simula como veria la camara del robot una foto impresa pegada en una pared de CARTON del laberinto,
a varias distancias, y prueba el detector de caras (YuNet) sobre esa imagen. Sin robot.

Modelo (camara pinhole):
  - imagen 250x250 (la preview que manda el puente), campo de vision --fov grados (por defecto 42:
    la OAK-D Lite tiene ~69 grados en horizontal a 16:9; la preview cuadrada recorta a ~42. Estimado, no medido);
  - foto impresa de --ancho-foto m (por defecto 0.20) con borde blanco, en una pared de carton,
    vista de frente o girada --angulo grados; centrada en la imagen salvo --desplazamiento;
  - desenfoque, ruido, brillo variable y compresion JPEG calidad 85 (como el puente).

Salida: task-a/camara/simulaciones/<fecha>/deteccion.png (filas = distancias, columnas = personas) + .txt
  python task-a/camara/simular_camara.py
  python task-a/camara/simular_camara.py --distancias 0.4 0.6 1.0 --angulo 30 --personas 10
"""
import argparse
import math
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "auto-1"))
sys.path.insert(0, str(HERE.parent / "reconocimiento"))
from face import FaceDetector                      # noqa: E402
from reconocer import EXTS, load_image              # noqa: E402

N = 250
ROOT = HERE.parents[1] / "data" / "rostros"


def carton(rng, n=N):
    """Textura de carton marron con manchas suaves, fibras y lineas del corrugado."""
    base = np.array([95, 145, 185], np.float32) * rng.uniform(0.85, 1.1)           # BGR marron claro
    img = np.ones((n, n, 3), np.float32) * base
    low = cv2.resize(rng.normal(0, 1, (6, 6)).astype(np.float32), (n, n), interpolation=cv2.INTER_CUBIC)
    img *= (1 + 0.06 * low)[..., None]
    img += rng.normal(0, 5, (n, n, 1))                                             # grano
    for x in range(0, n, int(rng.integers(6, 12))):                                # corrugado tenue
        img[:, x:x + 1] *= 0.96
    grad = np.linspace(rng.uniform(0.8, 1.0), rng.uniform(1.0, 1.15), n, dtype=np.float32)
    img *= grad[None, :, None]                                                     # iluminacion lateral
    return img


def pegar(fondo, foto, d, fov, ancho_m, angulo, dx, dy):
    """Proyecta la foto (con borde blanco) en la pared a `d` m. Devuelve la imagen 250x250 float."""
    b = max(4, foto.shape[1] // 25)
    foto = cv2.copyMakeBorder(foto, b, b, b, b, cv2.BORDER_CONSTANT, value=(235, 235, 235))
    h, w = foto.shape[:2]
    W, H = ancho_m, ancho_m * h / w
    f = (N / 2) / math.tan(math.radians(fov) / 2)
    a = math.radians(angulo)
    pts = []
    for px, py in ((-W / 2, -H / 2), (W / 2, -H / 2), (W / 2, H / 2), (-W / 2, H / 2)):
        X, Z = px * math.cos(a) + dx, d + px * math.sin(a)
        Y = py + dy
        pts.append((N / 2 + f * X / Z, N / 2 + f * Y / Z))
    M = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), np.float32(pts))
    warped = cv2.warpPerspective(foto.astype(np.float32), M, (N, N), flags=cv2.INTER_AREA)
    mask = cv2.warpPerspective(np.ones((h, w), np.float32), M, (N, N), flags=cv2.INTER_AREA)[..., None]
    return fondo * (1 - mask) + warped * mask, f * W / d                           # imagen, ancho de la foto en px


def camara(img, rng):
    """Efectos de la camara real: brillo, desenfoque, ruido, JPEG 85."""
    img = img * rng.uniform(0.75, 1.05)
    img = cv2.GaussianBlur(img, (0, 0), rng.uniform(0.5, 1.0))
    img = np.clip(img + rng.normal(0, 4, img.shape), 0, 255).astype(np.uint8)
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return cv2.imdecode(jpg, cv2.IMREAD_COLOR)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distancias", type=float, nargs="+", default=[0.4, 0.6, 0.8, 1.0, 1.5, 2.0])
    ap.add_argument("--personas", type=int, default=8)
    ap.add_argument("--fov", type=float, default=42.0, help="campo de vision de la imagen 250x250 (grados)")
    ap.add_argument("--ancho-foto", type=float, default=0.20, help="ancho de la foto impresa (m)")
    ap.add_argument("--angulo", type=float, default=0.0, help="giro de la pared respecto a la camara (grados)")
    ap.add_argument("--desplazamiento", type=float, nargs=2, default=[0.0, 0.0], metavar=("DX", "DY"),
                    help="foto desplazada (m) a la derecha / hacia abajo respecto al centro de la camara")
    ap.add_argument("--score", type=float, default=0.8, help="umbral del detector usado en el robot")
    ap.add_argument("--min-ancho", type=float, default=24)
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--data", default=str(ROOT))
    ap.add_argument("--out", default=str(HERE / "simulaciones"))
    args = ap.parse_args()

    rng = np.random.default_rng(args.semilla)
    random.seed(args.semilla)
    carpetas = sorted(p for p in Path(args.data).iterdir() if p.is_dir())
    elegidas = []
    for c in random.sample(carpetas, min(args.personas, len(carpetas))):
        fotos = [p for p in sorted(c.iterdir()) if p.suffix.lower() in EXTS]
        elegidas.append((c.name, random.choice(fotos)))
    fotos = [(n, load_image(p, max_side=1000)) for n, p in elegidas]

    det = FaceDetector(score=0.5, min_width=1)          # umbral bajo: vemos todo y luego aplicamos los dos criterios
    filas, log = [], []
    for d in args.distancias:
        tiles, ok_robot, ok_bajo = [], 0, 0
        for nombre, foto in fotos:
            img, ancho_px = pegar(carton(rng), foto, d, args.fov, args.ancho_foto, args.angulo, *args.desplazamiento)
            img = camara(img, rng)
            caras = det.faces(img)
            best = max(caras, key=lambda c: c[4]) if caras else None
            vista = img.copy()
            if best is None:
                txt, col = "sin cara", (0, 0, 255)
            else:
                x, y, w, h, s = best
                pasa = s >= args.score and w >= args.min_ancho
                ok_robot += pasa
                ok_bajo += True
                col = (0, 200, 0) if pasa else (0, 200, 255)
                cv2.rectangle(vista, (int(x), int(y)), (int(x + w), int(y + h)), col, 2)
                txt = f"{s:.2f}  {w:.0f}px"
            cv2.rectangle(vista, (0, N - 20), (N, N), (0, 0, 0), -1)
            cv2.putText(vista, f"{nombre[:14]}  {txt}", (4, N - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
            tiles.append(vista)
            log.append(f"d={d:.2f} m  {nombre:20s} " + ("sin cara" if best is None else f"score={best[4]:.2f} ancho={best[2]:.0f}px"))
        n = len(fotos)
        banda = np.full((26, N * n, 3), 25, np.uint8)
        cv2.putText(banda, f"{d:.2f} m (foto de {ancho_px:.0f} px de ancho): detecta con umbrales del robot {ok_robot}/{n}, "
                    f"con umbral bajo (0.5) {ok_bajo}/{n}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        filas.append(np.vstack([banda, np.hstack(tiles)]))
        log.append(f"--> {d:.2f} m: robot {ok_robot}/{n}, umbral bajo {ok_bajo}/{n}\n")
        print(f"{d:.2f} m: foto de {ancho_px:.0f} px | detecta (score>={args.score}, ancho>={args.min_ancho:.0f}px) {ok_robot}/{n} | "
              f"con score>=0.5 {ok_bajo}/{n}")

    cab = np.full((54, filas[0].shape[1], 3), 0, np.uint8)
    for i, t in enumerate([f"Simulacion: foto de {args.ancho_foto * 100:.0f} cm en pared de carton, fov {args.fov:.0f} grados, "
                           f"angulo {args.angulo:.0f} grados, imagen 250x250",
                           "verde = detecta con los umbrales del robot | naranja = solo con umbral bajo | rojo = no detecta"]):
        cv2.putText(cab, t, (8, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    out = Path(args.out) / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "deteccion.png"), np.vstack([cab, *filas]))
    (out / "deteccion.txt").write_text("\n".join(log) + "\n" + str(vars(args)) + "\n")
    print(f"guardado en {out}/deteccion.png")


if __name__ == "__main__":
    main()
