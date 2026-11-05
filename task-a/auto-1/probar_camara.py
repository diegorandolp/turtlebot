"""
Prueba de la camara del robot y del detector de rostros, SIN autonomia y sin mover el robot.

Uso (desde la raiz del repositorio, con el entorno virtual activado):
  python task-a/auto-1/probar_camara.py                 # ventana en vivo: caja verde = cara detectada
  python task-a/auto-1/probar_camara.py --una           # una sola foto, la guarda y sale
  python task-a/auto-1/probar_camara.py --foto yo.jpg   # prueba el detector con una foto del disco (sin robot)

El robot debe estar FUERA DEL DOCK (en el dock la camara esta apagada): saca el robot con
`python -m herramientas.monitor --drive` (tecla u) o con --undock aqui (retrocede solo: deja espacio detras).
Teclas de la ventana: q / Esc salir, s guardar la foto actual.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from face import FaceDetector                      # noqa: E402


def anotar(img, det, score_min):
    caras = det.faces(img)
    out = img.copy()
    for x, y, w, h, sc in caras:
        cv2.rectangle(out, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
        cv2.putText(out, f"{sc:.2f}  {w:.0f}px", (int(x), max(int(y) - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return out, caras


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--una", action="store_true", help="una foto, la guarda y sale")
    ap.add_argument("--foto", help="probar el detector con una imagen del disco (no usa el robot)")
    ap.add_argument("--undock", action="store_true", help="sacar el robot del dock primero (retrocede solo)")
    ap.add_argument("--score", type=float, default=0.8, help="umbral del detector (defecto 0.8; baja a 0.5 para depurar)")
    ap.add_argument("--min-ancho", type=float, default=24, help="ancho minimo de la cara en px (defecto 24)")
    ap.add_argument("--out", default=str(HERE / "fotos"))
    args = ap.parse_args()

    det = FaceDetector(score=args.score, min_width=args.min_ancho)
    outdir = Path(args.out)

    if args.foto:
        img = cv2.imread(args.foto)
        if img is None:
            sys.exit(f"No pude leer {args.foto}")
        res, caras = anotar(img, det, args.score)
        print(f"{len(caras)} cara(s):", [(round(w), round(sc, 2)) for _, _, w, _, sc in caras], "(ancho px, score)")
        outdir.mkdir(exist_ok=True)
        cv2.imwrite(str(outdir / "deteccion_foto.png"), res)
        print("guardada", outdir / "deteccion_foto.png")
        return

    from tb4 import TB4Client
    with TB4Client(args.ip) as robot:
        st = robot.wait_ready(8)
        if args.undock:
            print("[..] sacando del dock ...")
            robot.undock()
            st = robot.status()
        print(f"dock={st.docked}  bateria={st.battery * 100:.0f}%")
        if st.docked:
            print("[!] El robot esta EN EL DOCK: la camara esta apagada. Sacalo (--undock) y repite.")
        n, ok = 0, 0
        while True:
            try:
                img = robot.photo()
            except Exception as e:
                print(f"[error] {type(e).__name__}: {e}")
                if args.una:
                    sys.exit(1)
                time.sleep(1.0)
                continue
            n += 1
            res, caras = anotar(img, det, args.score)
            ok += bool(caras)
            if caras:
                print(f"foto {n}: CARA  ancho={[round(c[2]) for c in caras]}px  score={[round(c[4], 2) for c in caras]}")
            else:
                print(f"foto {n}: sin cara  (brillo medio {img.mean():.0f})")
            if args.una:
                outdir.mkdir(exist_ok=True)
                cv2.imwrite(str(outdir / "foto_robot.png"), img)
                cv2.imwrite(str(outdir / "foto_robot_detectada.png"), res)
                print("guardadas en", outdir)
                return
            cv2.imshow("camara del robot (caja verde = cara)", cv2.resize(res, None, fx=2, fy=2))
            k = cv2.waitKey(300) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                outdir.mkdir(exist_ok=True)
                f = outdir / f"foto_{int(time.time())}.png"
                cv2.imwrite(str(f), img)
                print("guardada", f)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
