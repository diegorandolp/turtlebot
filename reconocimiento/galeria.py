"""
Construye la galeria de rostros: un vector ArcFace por foto de datos/rostros/<Nombre>/*.jpg.

  python -m reconocimiento.galeria                  # -> reconocimiento/modelos/galeria_mbf.npz
  python -m reconocimiento.galeria --modelo r50     # ArcFace R50 (si esta en ~/.insightface)
  python -m reconocimiento.galeria --montaje caras.jpg   # ademas guarda las caras alineadas, para revisar

Para agregar a alguien: crea datos/rostros/Nombre_Apellido/ con 3-10 fotos (de frente y algo de
lado, con buena luz) y vuelve a correr este comando. El nombre que se anuncia es el de la carpeta
con espacios en vez de '_'.

--aumentar agrega, por cada foto, versiones de baja resolucion (como las veria el robot de lejos):
ayuda cuando la cara de B ocupa pocos pixeles.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

from .degradar import baja_resolucion
from .rostros import ROSTROS, Detector, Embebedor, Galeria, alinear, galeria_path, nombre_bonito

EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def fotos_por_persona(carpeta):
    personas = []
    for d in sorted(p for p in Path(carpeta).iterdir() if p.is_dir()):
        fotos = sorted(f for f in d.iterdir() if f.suffix.lower() in EXT)
        if fotos:
            personas.append((nombre_bonito(d.name), fotos))
    return personas


def caras_alineadas(personas, det, avisar=print):
    """-> lista de (indice de persona, archivo, cara 112x112) con la cara mas grande de cada foto."""
    out = []
    for i, (nombre, fotos) in enumerate(personas):
        for f in fotos:
            img = cv2.imread(str(f))
            if img is None:
                avisar(f"[aviso] no se pudo leer {f}")
                continue
            caras = det(img)
            if not caras:
                avisar(f"[aviso] sin cara en {f}")
                continue
            out.append((i, f, alinear(img, caras[0])))
    return out


def construir(carpeta=ROSTROS, modelo="mbf", aumentar=0, semilla=0, avisar=print):
    personas = fotos_por_persona(carpeta)
    if not personas:
        raise SystemExit(f"No hay fotos en {carpeta}/<Nombre>/")
    det, emb = Detector(), Embebedor(modelo)
    al = caras_alineadas(personas, det, avisar)
    rng = np.random.default_rng(semilla)
    imgs, etq, fuente = [], [], []
    for i, f, c in al:
        imgs.append(c)
        etq.append(i)
        fuente.append(f"{f.parent.name}/{f.name}")
        for _ in range(aumentar):
            imgs.append(baja_resolucion(c, rng.uniform(18, 48), rng))
            etq.append(i)
            fuente.append(f"{f.parent.name}/{f.name}#baja")
    g = Galeria([n for n, _ in personas], emb(imgs), np.array(etq), fuente, modelo)
    return g, al


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rostros", default=str(ROSTROS), help="carpeta con una subcarpeta por persona")
    ap.add_argument("--modelo", default="mbf", help="mbf (13 MB, en el repo) | r50 | ruta a un .onnx ArcFace")
    ap.add_argument("--aumentar", type=int, default=0, help="versiones de baja resolucion por foto")
    ap.add_argument("--salida", default=None, help="archivo .npz (defecto reconocimiento/modelos/galeria_<modelo>.npz)")
    ap.add_argument("--montaje", default=None, help="guardar una imagen con todas las caras alineadas")
    a = ap.parse_args(argv)

    g, al = construir(a.rostros, a.modelo, a.aumentar)
    salida = Path(a.salida) if a.salida else galeria_path(a.modelo)
    g.guardar(salida)
    print(f"[ok] {len(g.nombres)} personas, {len(al)} fotos con cara, {len(g.emb)} vectores -> {salida}")
    for i, n in enumerate(g.nombres):
        print(f"     {n:<22} {int((g.etiqueta == i).sum())} vectores")
    # parecido entre personas distintas: si dos se confunden, conviene agregar fotos
    cent = np.array([g.emb[g.etiqueta == i].mean(0) for i in range(len(g.nombres))])
    cent /= np.linalg.norm(cent, axis=1, keepdims=True)
    S = cent @ cent.T - 2 * np.eye(len(cent))
    i, j = np.unravel_index(np.argmax(S), S.shape)
    print(f"     par mas parecido: {g.nombres[i]} / {g.nombres[j]} (similitud {S[i, j]:.2f})")
    if a.montaje:
        tiles = [c for _, _, c in al]
        cols = 12
        filas = [np.hstack(tiles[k:k + cols] + [np.zeros_like(tiles[0])] * (cols - len(tiles[k:k + cols])))
                 for k in range(0, len(tiles), cols)]
        cv2.imwrite(a.montaje, np.vstack(filas))
        print(f"     montaje -> {a.montaje}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
