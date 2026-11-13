"""
Evalua el detector de flechas con carpetas de fotos de la camara del robot, una carpeta por clase.

  python task-a/carrera-1/evaluar_flechas.py --izq fotos/izq --der fotos/der --nada fotos/otras
  python task-a/carrera-1/evaluar_flechas.py --izq ... --der ... --errores errores/   # guarda los fallos

Cada foto de --izq/--der debe tener una flecha de ese sentido; las de --nada, ninguna (paredes,
obstaculos, la foto de B, otras senales). La respuesta por foto es la flecha mas confiable
(izq/der) o "nada". Para juntar fotos: task-a/carrera-1/probar_vision.py (tecla s guarda).
"""
import argparse
import sys
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from flechas import DER, IZQ, DetectorFlechas, dibujar      # noqa: E402

EXT = {".jpg", ".jpeg", ".png", ".bmp"}


def respuesta(det, img):
    fl = [f for f in det(img) if f.dir in (IZQ, DER)]
    return (fl[0].dir, fl) if fl else ("nada", fl)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--izq", nargs="*", default=[])
    ap.add_argument("--der", nargs="*", default=[])
    ap.add_argument("--nada", nargs="*", default=[])
    ap.add_argument("--umbral", type=float, default=None)
    ap.add_argument("--margen", type=float, default=None)
    ap.add_argument("--errores", default=None, help="carpeta donde guardar las fotos mal clasificadas")
    a = ap.parse_args(argv)
    det = DetectorFlechas()
    if a.umbral is not None:
        det.umbral = a.umbral
    if a.margen is not None:
        det.margen = a.margen
    conf = {}
    for clase, dirs in ((IZQ, a.izq), (DER, a.der), ("nada", a.nada)):
        for d in dirs:
            for f in sorted(Path(d).iterdir()):
                if f.suffix.lower() not in EXT:
                    continue
                img = cv2.imread(str(f))
                if img is None:
                    continue
                r, fl = respuesta(det, img)
                conf[(clase, r)] = conf.get((clase, r), 0) + 1
                if a.errores and r != clase:
                    out = Path(a.errores)
                    out.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(out / f"{clase}_como_{r}_{f.stem}.png"), dibujar(img.copy(), fl))
    clases = [IZQ, DER, "nada"]
    print(f"{'real \\ respuesta':>18}" + "".join(f"{c:>8}" for c in clases) + "   acierto")
    for c in clases:
        tot = sum(conf.get((c, r), 0) for r in clases)
        if tot:
            print(f"{c:>18}" + "".join(f"{conf.get((c, r), 0):>8}" for r in clases) + f"   {100 * conf.get((c, c), 0) / tot:5.1f}%")
    flechas = sum(conf.get((c, r), 0) for c in (IZQ, DER) for r in clases)
    if flechas:
        mal = conf.get((IZQ, DER), 0) + conf.get((DER, IZQ), 0)
        print(f"sentido equivocado: {mal} de {flechas} ({100 * mal / flechas:.1f}%)  <- el error peligroso")
    return 0


if __name__ == "__main__":
    sys.exit(main())
