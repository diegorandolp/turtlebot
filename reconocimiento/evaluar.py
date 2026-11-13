"""
Evaluacion del reconocimiento sin robot: dejar una foto fuera (leave-one-out) y "fotografiarla"
como lo haria la camara del robot (degradar.py: foto impresa, 250x250, cara de N px, JPEG).

  python -m reconocimiento.evaluar                     # modelo mbf, anchos de cara 14..96 px
  python -m reconocimiento.evaluar --modelo r50 --reps 5
  python -m reconocimiento.evaluar --anchos 20 30 40 --frontal

Para cada foto y cada ancho se generan `reps` cuadros distintos. Se mide:
  detecta  % de cuadros donde YuNet encuentra la cara
  1 foto   % de acierto con un solo cuadro (sobre los cuadros con cara detectada)
  voto     % de acierto votando con todos los cuadros de esa foto y ancho (Votacion)
La galeria usada en cada prueba excluye la foto evaluada (y sus versiones aumentadas).
"""
import argparse
import sys
import time

import cv2
import numpy as np

from .degradar import iou, simular_camara
from .galeria import construir, fotos_por_persona
from .rostros import ROSTROS, Detector, Embebedor, Votacion, peso_por_tamano, Reconocedor, alinear


def es_frontal(cara, tol=0.22):
    """La nariz cae cerca del centro entre los ojos: la cara mira a la camara."""
    p = cara.puntos
    ojos = p[1, 0] - p[0, 0]
    return ojos > 1 and abs(p[2, 0] - (p[0, 0] + p[1, 0]) / 2) / ojos < tol


def evaluar(modelo="mbf", anchos=(14, 18, 24, 32, 48, 64, 96), reps=3, aumentar=0, solo_frontal=False,
            semilla=0, carpeta=ROSTROS, avisar=print):
    gal, _ = construir(carpeta, modelo, aumentar, avisar=lambda *_: None)
    det, emb = Detector(), Embebedor(modelo)
    rec = Reconocedor.__new__(Reconocedor)                  # mismo flujo que en el robot, galeria en memoria
    rec.det, rec.emb, rec.gal = det, emb, gal
    rng = np.random.default_rng(semilla)
    personas = fotos_por_persona(carpeta)
    fila = {a: dict(n=0, det=0, ok1=0, n1=0, okv=0, nv=0) for a in anchos}
    errores = {}
    for i, (nombre, fotos) in enumerate(personas):
        for f in fotos:
            img = cv2.imread(str(f))
            caras = det(img) if img is not None else []
            if not caras or (solo_frontal and not es_frontal(caras[0])):
                continue
            fuente = f"{f.parent.name}/{f.name}"
            excluir = np.array([s.split("#")[0] == fuente for s in gal.fuente])
            if excluir.all() or not (gal.etiqueta[~excluir] == i).any():
                continue                                    # la persona no tendria otras fotos
            for a in anchos:
                v = Votacion(gal.nombres, min_fotos=1, max_fotos=99)
                for _ in range(reps):
                    cuadro, esperada = simular_camara(img, caras[0].caja, a, rng)
                    fila[a]["n"] += 1
                    cs = [c for c in det(cuadro) if iou(c.caja, esperada) > 0.25]
                    if not cs:
                        continue
                    fila[a]["det"] += 1
                    r = rec.resultado(cuadro, cs[0], excluir)
                    fila[a]["n1"] += 1
                    fila[a]["ok1"] += r.nombre == nombre
                    if r.nombre != nombre:
                        errores[(nombre, r.nombre)] = errores.get((nombre, r.nombre), 0) + 1
                    v.agregar(r.sims, peso_por_tamano(cs[0].ancho))
                if v.n:
                    fila[a]["nv"] += 1
                    fila[a]["okv"] += v.lider()[0] == nombre
    return fila, errores


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modelo", default="mbf")
    ap.add_argument("--anchos", type=int, nargs="+", default=[14, 18, 24, 32, 48, 64, 96])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--aumentar", type=int, default=0)
    ap.add_argument("--frontal", action="store_true", help="solo fotos donde la persona mira a la camara")
    ap.add_argument("--semilla", type=int, default=0)
    a = ap.parse_args(argv)
    t0 = time.time()
    fila, errores = evaluar(a.modelo, a.anchos, a.reps, a.aumentar, a.frontal, a.semilla)
    print(f"modelo={a.modelo} aumentar={a.aumentar} frontal={a.frontal} reps={a.reps}  ({time.time() - t0:.0f} s)")
    print(f"{'cara px':>8} {'cuadros':>8} {'detecta':>8} {'1 foto':>8} {'voto':>8}")
    for w, r in fila.items():
        pct = lambda x, n: f"{100 * x / n:6.1f}%" if n else "    -  "     # noqa: E731
        print(f"{w:>8} {r['n']:>8} {pct(r['det'], r['n']):>8} {pct(r['ok1'], r['n1']):>8} {pct(r['okv'], r['nv']):>8}")
    if errores:
        top = sorted(errores.items(), key=lambda kv: -kv[1])[:6]
        print("confusiones mas comunes (real -> respuesta: veces):")
        for (r, p), n in top:
            print(f"   {r} -> {p}: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
