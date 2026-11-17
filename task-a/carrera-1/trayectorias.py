"""Dibuja, vista desde arriba, la trayectoria de cada corrida de un banco (`banco_pruebas.py --json X --opciones
--traza`): el mundo (cajas, señales, fotos), A, el recorrido (azul al empezar, rojo al final) y el resultado.

  python task-a/carrera-1/trayectorias.py resultados.json trayectorias.png
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mundos import construir  # noqa: E402


def dibujar(r, S=70):
    m = construir(r["mundo"], persona=r.get("persona"))
    W = max(max(p.a[0], p.b[0]) for p in m.paredes)
    H = max(max(p.a[1], p.b[1]) for p in m.paredes)
    img = np.full((int(H * S) + 40, int(W * S) + 20, 3), 255, np.uint8)

    def P(x, y):
        return int(x * S) + 10, int((H - y) * S) + 30

    for p in m.paredes:
        cv2.line(img, P(*p.a), P(*p.b), (60, 90, 140), 2)
    for c in m.carteles:
        if c.tipo.startswith("flecha"):
            cx, cy = (np.array(c.a) + np.array(c.b)) / 2
            cv2.putText(img, c.tipo[7:], P(cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    for _, _, (rx, ry) in m.rostros:
        cv2.circle(img, P(rx, ry), 5, (200, 0, 200), -1)
    x0, y0, th0 = m.inicio
    c, s = math.cos(th0), math.sin(th0)
    tr = r.get("traza") or []
    pts = [P(x0 + c * f[2] - s * f[3], y0 + s * f[2] + c * f[3]) for f in tr]
    for i in range(1, len(pts)):
        col = (int(255 * i / len(pts)), 0, int(255 * (1 - i / len(pts))))
        cv2.line(img, pts[i - 1], pts[i], col, 2)
    cv2.circle(img, P(x0, y0), 7, (0, 160, 0), -1)
    ok = r.get("exito")
    txt = f"{r['mundo']}  {'OK' if ok else 'FALLA'} {r.get('tiempo') or 0:.0f} s  choques {r.get('choques')}"
    cv2.putText(img, txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 0) if ok else (0, 0, 220), 1)
    return img


def main():
    datos = json.load(open(sys.argv[1]))
    corridas = [r for v in datos.values() for r in v]
    imgs = [dibujar(r) for r in corridas if r.get("estado") != "ERROR"]
    h = max(i.shape[0] for i in imgs)
    w = max(i.shape[1] for i in imgs)
    imgs = [cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, w - i.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255))
            for i in imgs]
    cols = 4
    while len(imgs) % cols:
        imgs.append(np.full((h, w, 3), 255, np.uint8))
    filas = [np.hstack(imgs[i:i + cols]) for i in range(0, len(imgs), cols)]
    cv2.imwrite(sys.argv[2], np.vstack(filas))
    print("[ok]", sys.argv[2], len(corridas), "corridas")


if __name__ == "__main__":
    main()
