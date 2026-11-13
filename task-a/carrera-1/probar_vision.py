"""
Prueba la vision de carrera-1 con la camara del robot (o la webcam / fotos del disco), SIN mover el
robot: flechas (sentido y puntaje), caras y quien es cada una, y la votacion acumulada.

  python task-a/carrera-1/probar_vision.py                    # robot 192.168.0.103, preview 250x250
  python task-a/carrera-1/probar_vision.py --hd               # fotos HD (camara con camara_rgb.sh --hd)
  python task-a/carrera-1/probar_vision.py --webcam 0         # la webcam de la PC (p. ej. con la foto en la mano)
  python task-a/carrera-1/probar_vision.py --fotos carpeta/   # imagenes del disco

Teclas: s guarda la foto en fotos/<clase>/ (la clase se elige con 1 = izq, 2 = der, 3 = nada, 4 = cara;
asi se arma un set para evaluar_flechas.py) · r reinicia la votacion · q / Esc salir.
Muestra tambien las fotos por segundo: con el robot real conviene >= 4 (si no, no usar --hd).
"""
import argparse
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))

from carrera import ascii                                            # noqa: E402
from flechas import DetectorFlechas, dibujar                         # noqa: E402
from reconocimiento import Reconocedor, Votacion, peso_por_tamano   # noqa: E402

CLASES = {ord("1"): "izq", ord("2"): "der", ord("3"): "nada", ord("4"): "cara"}


def fuente(a):
    if a.fotos:
        files = sorted(p for p in Path(a.fotos).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        for f in files:
            yield cv2.imread(str(f))
        return
    if a.webcam is not None:
        cap = cv2.VideoCapture(a.webcam)
        while True:
            ok, img = cap.read()
            if not ok:
                return
            yield img
    from tb4 import TB4Client
    with TB4Client(a.ip) as robot:
        robot.wait_ready(8)
        if a.undock:
            print("[..] sacando del dock (retrocede solo) ...", robot.undock())
        while True:
            try:
                yield robot.photo(timeout=3.0, hd=a.hd)
            except Exception as e:
                print("[aviso] sin foto:", e)
                time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--hd", action="store_true")
    ap.add_argument("--undock", action="store_true", help="sacar el robot del dock primero (retrocede solo)")
    ap.add_argument("--webcam", type=int, default=None)
    ap.add_argument("--fotos", default=None)
    ap.add_argument("--modelo", default="auto")
    ap.add_argument("--flechas-color", action="store_true")
    ap.add_argument("--out", default=str(HERE / "fotos"))
    a = ap.parse_args()
    rec = Reconocedor(a.modelo)
    det = DetectorFlechas(color=a.flechas_color)
    vot = Votacion(rec.nombres)
    clase, n, t0, fps = "nada", 0, time.monotonic(), 0.0
    print(f"[ok] reconocimiento {rec.modelo}, {len(rec.nombres)} personas. Teclas: 1-4 clase, s guardar, r reiniciar, q salir")
    for img in fuente(a):
        if img is None:
            continue
        n += 1
        dt = time.monotonic() - t0
        t0 = time.monotonic()
        fps = 0.8 * fps + 0.2 / max(dt, 1e-3)
        caras = rec.analizar(img)
        flechas = det(img)
        out = dibujar(img.copy(), flechas)
        for r in caras:
            x, y, w, h = (int(v) for v in r.cara.caja)
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(out, f"{ascii(r.nombre)} {r.sim:.2f} ({w}px)", (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        if caras:
            vot.agregar(caras[0].sims, peso_por_tamano(caras[0].cara.ancho))
        s = 600 / max(out.shape[:2])
        out = cv2.resize(out, None, fx=s, fy=s)
        lineas = [f"{img.shape[1]}x{img.shape[0]}  {fps:4.1f} fotos/s   clase al guardar: {clase}"]
        if vot.n:
            lineas += [f"votacion ({vot.n} fotos): " + ", ".join(f"{ascii(nm)} {p:.2f}" for nm, p in vot.top(3))]
        for i, t in enumerate(lineas):
            cv2.putText(out, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2, cv2.LINE_AA)
        cv2.imshow("probar_vision (carrera-1)", out)
        k = cv2.waitKey(0 if a.fotos else 1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k in CLASES:
            clase = CLASES[k]
        elif k == ord("r"):
            vot.reiniciar()
        elif k == ord("s"):
            d = Path(a.out) / clase
            d.mkdir(parents=True, exist_ok=True)
            f = d / time.strftime(f"%Y%m%d_%H%M%S_{n:04d}.jpg")
            cv2.imwrite(str(f), img)
            print("guardada", f)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
