"""
Prueba del reconocimiento de gestos con la WEBCAM DE LA PC, SIN robot.
Gestos: señalar con el índice en 7 direcciones (girar), V (avanzar lento), 3 dedos (rápido), mano abierta (STOP).

Uso (desde la raíz del repositorio):
  python task-b/gestos-1/probar_gestos.py                  # webcam 0, ventana en vivo
  python task-b/gestos-1/probar_gestos.py --cam 1          # otra cámara
  python task-b/gestos-1/probar_gestos.py --foto mano.jpg  # una foto del disco (sirve en WSL, sin cámara)

En WSL2 no hay webcam: ejecútalo con el Python de Windows (python.exe, ver README.md).
Teclas: q / Esc salir · s guardar captura.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from gestos import AVANCES, NINGUNO, NOMBRE, STOP, Estabilizador, clasificar, es_senal   # noqa: E402
from mano import abrir_camara, crear_detector, detectar, dibujar_mano, texto     # noqa: E402


def color(g):
    if g == STOP:
        return (0, 0, 255)
    if g in AVANCES:
        return (0, 255, 0)
    if es_senal(g):
        return (0, 200, 255)
    return (160, 160, 160)


def dibujar(img, pts, crudo, info, confirmado, fps=None):
    h, w = img.shape[:2]
    dibujar_mano(img, pts)
    ang = "-" if info.get("angulo") is None else f"{info['angulo']:.0f} deg"
    texto(img, f"crudo: {crudo}  dedos: {info.get('estirados') or '-'}  ang: {ang}", (10, 25))
    if fps:
        texto(img, f"{fps:.0f} fps", (w - 90, 25))
    texto(img, NOMBRE[confirmado], (10, h - 20), 1.3, color(confirmado), 3)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", type=int, default=0, help="índice de la webcam (defecto 0)")
    ap.add_argument("--foto", help="clasificar una imagen del disco en vez de usar la cámara")
    ap.add_argument("--no-espejo", action="store_true", help="no voltear la imagen (por defecto vista espejo)")
    ap.add_argument("--margen", type=float, default=20.0, help="grados de margen de cada dirección al señalar (defecto 20, máx. 22.5)")
    args = ap.parse_args()

    if args.foto:
        img = cv2.imread(args.foto)
        if img is None:
            sys.exit(f"No pude leer {args.foto}")
        if not args.no_espejo:
            img = cv2.flip(img, 1)
        with crear_detector(video=False) as det:
            pts = detectar(det, img)
        gesto, info = clasificar(pts, args.margen)
        print("mano detectada" if pts else "sin mano", "->", gesto, info)
        out = HERE / "capturas" / "gesto_foto.png"
        out.parent.mkdir(exist_ok=True)
        cv2.imwrite(str(out), dibujar(img, pts, gesto, info, gesto))
        print("guardada", out)
        return

    cap = abrir_camara(args.cam)
    estab = Estabilizador()
    ultimo = NINGUNO
    t0 = time.monotonic()
    t_prev, fps = t0, None
    with crear_detector(video=True) as det:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("la cámara no entrega imagen")
                break
            if not args.no_espejo:
                frame = cv2.flip(frame, 1)
            t = time.monotonic()
            pts = detectar(det, frame, int((t - t0) * 1000))
            crudo, info = clasificar(pts, args.margen)
            confirmado = estab.update(crudo)
            if confirmado != ultimo:
                print(f"[{t - t0:6.1f}s] gesto: {confirmado}")
                ultimo = confirmado
            dt, t_prev = t - t_prev, t
            if dt > 0:
                fps = 1 / dt if fps is None else 0.9 * fps + 0.1 / dt
            cv2.imshow("Gestos (q = salir)", dibujar(frame, pts, crudo, info, confirmado, fps))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                out = HERE / "capturas" / f"gesto_{time.strftime('%H%M%S')}.png"
                out.parent.mkdir(exist_ok=True)
                cv2.imwrite(str(out), frame)
                print("guardada", out)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
