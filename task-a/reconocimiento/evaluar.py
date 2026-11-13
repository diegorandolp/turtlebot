"""
Evalua el reconocimiento simulando el RETO (sin robot ni camara), con las fotos de data/rostros.

En el reto nos dicen a QUIEN buscar y en el laberinto hay 4 fotos de personas distintas. Cada ronda:
  - se elige una persona objetivo y 3 distractores al azar;
  - de cada una de las 4 se toma una foto "candidata" (la que estaria pegada en el laberinto);
  - la galeria del objetivo son SUS OTRAS fotos (la candidata se excluye: si no, seria trampa);
  - cada candidata se compara con la galeria del objetivo; se elige la mas parecida.
Acierta si la elegida es la del objetivo. Con --mismo-angulo las 4 candidatas usan la misma posicion
en su carpeta (p. ej. todas la 1a foto), por si las carpetas siguen el mismo orden de angulos.

Salida: task-a/reconocimiento/resultados/<fecha>/resultados.png (una fila por ronda) + resultados.txt

  python task-a/reconocimiento/evaluar.py                      # 10 rondas (semilla 0)
  python task-a/reconocimiento/evaluar.py --rondas 20 --semilla 3 --mismo-angulo
"""
import argparse
import random
import time
from pathlib import Path

import cv2
import numpy as np

from reconocer import COS_THR, EXTS, Recognizer, load_image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1] / "data" / "rostros"
T = 200                                   # lado de cada miniatura (px)
GREEN, RED, GREY, WHITE = (60, 200, 60), (40, 40, 230), (150, 150, 150), (255, 255, 255)


def thumb(img, box=None):
    """Miniatura T x T de la foto (recuadro de la cara dibujado), con bandas negras si no es cuadrada."""
    img = img.copy()
    if box is not None:
        x, y, w, h = [int(v) for v in box]
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), max(2, img.shape[1] // 160))
    s = T / max(img.shape[:2])
    img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    out = np.zeros((T, T, 3), np.uint8)
    y0, x0 = (T - img.shape[0]) // 2, (T - img.shape[1]) // 2
    out[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img
    return out


def label(tile, lines, color, border=None):
    tile = tile.copy()
    cv2.rectangle(tile, (0, T - 18 * len(lines) - 6), (T, T), (0, 0, 0), -1)
    for i, t in enumerate(lines):
        cv2.putText(tile, t, (5, T - 18 * (len(lines) - 1 - i) - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    if border is not None:
        cv2.rectangle(tile, (0, 0), (T - 1, T - 1), border, 5)
    return tile


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rondas", type=int, default=10)
    ap.add_argument("--semilla", type=int, default=0)
    ap.add_argument("--mismo-angulo", action="store_true", help="las 4 candidatas con la misma posicion en su carpeta")
    ap.add_argument("--data", default=str(ROOT))
    ap.add_argument("--out", default=str(HERE / "resultados"))
    args = ap.parse_args()

    r = Recognizer()
    personas = {}
    t0 = time.perf_counter()
    for d in sorted(Path(args.data).iterdir()):
        fotos = [p for p in sorted(d.iterdir()) if p.suffix.lower() in EXTS]
        items = []
        for p in fotos:
            img = load_image(p)
            feat, box = r.embed(img)
            if feat is not None:
                items.append(dict(path=p, feat=feat, thumb=thumb(img, box)))
        if len(items) >= 2:
            personas[d.name] = items
    print(f"{sum(len(v) for v in personas.values())} fotos con cara de {len(personas)} personas en {time.perf_counter() - t0:.0f} s")

    random.seed(args.semilla)
    nombres = sorted(personas)
    filas, aciertos, margenes, log = [], 0, [], []
    for k in range(args.rondas):
        objetivo, *distr = random.sample(nombres, 4)
        if args.mismo_angulo:
            pos = random.randrange(min(len(personas[n]) for n in [objetivo, *distr]))
            elegir = {n: personas[n][pos] for n in [objetivo, *distr]}
        else:
            elegir = {n: random.choice(personas[n]) for n in [objetivo, *distr]}
        galeria = [it for it in personas[objetivo] if it is not elegir[objetivo]]
        G = np.array([it["feat"] for it in galeria])
        orden = [objetivo, *distr]
        random.shuffle(orden)                                   # el objetivo no siempre en la misma columna
        sims = {n: float(np.max(G @ elegir[n]["feat"])) for n in orden}
        elegida = max(sims, key=sims.get)
        ok = elegida == objetivo
        aciertos += ok
        rival = max(s for n, s in sims.items() if n != objetivo)
        margenes.append(sims[objetivo] - rival)

        ref = label(galeria[0]["thumb"], [f"BUSCAR: {objetivo[:18]}", f"galeria: {len(galeria)} fotos"], WHITE)
        celdas = [ref, np.full((T, 12, 3), 40, np.uint8)]
        for n in orden:
            es_obj = n == objetivo
            color = GREEN if es_obj else GREY
            borde = (GREEN if ok else RED) if n == elegida else None
            texto = [f"{n[:18]}", f"sim {sims[n]:.2f}" + ("  <- ELEGIDA" if n == elegida else "")]
            celdas.append(label(elegir[n]["thumb"], texto, color, borde))
        fila = np.hstack(celdas)
        banda = np.full((26, fila.shape[1], 3), 25, np.uint8)
        cv2.putText(banda, f"Ronda {k + 1}: {'ACIERTO' if ok else 'FALLO'}  (margen objetivo - mejor distractor = {sims[objetivo] - rival:+.2f})",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, GREEN if ok else RED, 1, cv2.LINE_AA)
        filas.append(np.vstack([banda, fila]))
        linea = f"ronda {k + 1:2d} {'OK   ' if ok else 'FALLO'} objetivo={objetivo:20s} elegida={elegida:20s} " + \
                " ".join(f"{n}={s:.2f}" for n, s in sims.items())
        log.append(linea)
        print(linea)

    resumen = (f"acierto: {aciertos}/{args.rondas}  |  margen medio {np.mean(margenes):+.2f}, minimo {np.min(margenes):+.2f}  |  "
               f"{'mismo angulo' if args.mismo_angulo else 'fotos al azar'}, semilla {args.semilla}")
    print("\n" + resumen)
    cab = np.full((40, filas[0].shape[1], 3), 0, np.uint8)
    cv2.putText(cab, resumen, (8, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
    out = Path(args.out) / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "resultados.png"), np.vstack([cab, *filas]))
    (out / "resultados.txt").write_text("\n".join(log + ["", resumen, f"umbral coseno de referencia: {COS_THR}"]) + "\n")
    print(f"guardado en {out}/resultados.png")


if __name__ == "__main__":
    main()
