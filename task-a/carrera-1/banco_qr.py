"""
Banco del lector de QR (qr.py) en el simulador: fotos frente a la meta (6 fotos con QR) a 0.6-2 m, de
frente y en angulo, preview y HD. Mide en que % de fotos se lee el QR de la persona buscada (y se le
asigna una cara), cuantos QR se asignan a una cara equivocada, y el tiempo.

  python task-a/carrera-1/banco_qr.py              # con el lector que haya (WeChat si esta opencv-contrib)
  python task-a/carrera-1/banco_qr.py qr_otro      # otro modulo con buscar_qrs (para comparar versiones)
"""
import importlib
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
import numpy as np                                                   # noqa: E402
from mundos import construir                                          # noqa: E402
from simulador import SimRobot                                        # noqa: E402
from reconocimiento import Reconocedor                                # noqa: E402
mod = importlib.import_module(sys.argv[1] if len(sys.argv) > 1 else "qr")
print("lector:", mod.lector() if hasattr(mod, "lector") else "?")
rec = None
filas = {}
for mundo in ["croquis+6", "denso:3+6", "denso:11+6", "denso:7+6:0.3"]:
    m = construir(mundo)
    if rec is None:
        rec = Reconocedor("mbf")
    r = SimRobot(m, foto_hd=(1280, 720, 66.0))
    rostros = m.rostros
    for (nombre, texto, (fx, fy)) in rostros[1:5]:
        for d in (0.6, 1.0, 1.5, 2.0):
            for ang in (0.0, 0.35):
                r.wx = fx - d * math.cos(ang); r.wy = fy + d * math.sin(ang); r.wth = -ang * 0.6
                for hd in (False, True):
                    img = r.photo(hd=hd)
                    caras = rec.analizar(img)[:6]
                    t0 = time.perf_counter()
                    qs = mod.buscar_qrs(img, [c.cara.caja for c in caras])
                    dt = (time.perf_counter() - t0) * 1000
                    k = (hd, d)
                    f = filas.setdefault(k, dict(n=0, ok=0, lei=0, mal=0, leidos=0, t=[]))
                    f["n"] += 1; f["t"].append(dt)
                    real = {t: n for n, t, _ in rostros}
                    for q in qs:
                        f["leidos"] += 1
                        if q.texto == texto:
                            f["lei"] += 1
                            if q.cara is not None: f["ok"] += 1
                        if q.cara is not None and caras[q.cara].margen > 0.08 and real.get(q.texto) != caras[q.cara].nombre:
                            f["mal"] += 1
print(f"{'cam':5}{'d':>5}{'leido':>8}{'leido+cara':>12}{'leidos/foto':>13}{'asig. mal':>11}{'ms med':>8}{'ms max':>8}")
for (hd, d), f in sorted(filas.items()):
    print(f"{'HD' if hd else 'prev':5}{d:5.1f}{100*f['lei']/f['n']:7.0f}%{100*f['ok']/f['n']:11.0f}%{f['leidos']/f['n']:13.1f}{f['mal']:11d}{np.median(f['t']):8.0f}{max(f['t']):8.0f}")
