"""
Banco de pruebas: corre la carrera completa en muchos circuitos aleatorios del simulador (sin
ventana, en paralelo) y resume exito, tiempos, choques y errores. Es la medida de que la estrategia
generaliza a circuitos que no conoce, con y sin flechas.

  python task-a/carrera-1/banco_pruebas.py                         # 20 circuitos con flechas + los mismos sin
  python task-a/carrera-1/banco_pruebas.py --n 50 --desde 100      # semillas 100..149
  python task-a/carrera-1/banco_pruebas.py --mundos demo sala --opciones --temprano
  python task-a/carrera-1/banco_pruebas.py --json resultados.json  # ademas guarda cada corrida
  python task-a/carrera-1/banco_pruebas.py --videos videos/        # VIDEO de cada corrida que falla o choca
  python task-a/carrera-1/banco_pruebas.py --videos videos/ --videos-todos   # ...o de todas
  python task-a/carrera-1/banco_pruebas.py --mundos croquis+6 denso:1+6 denso:2+6     # 6 fotos con QR en la meta
  python task-a/carrera-1/banco_pruebas.py --mundos croquis denso:1 denso:2 denso:3 --duro
                                                                   # dificil: odometria con sesgo, camara lenta

Exito = vuelve a A (a menos de 35 cm) con el nombre correcto. La foto de B es de la galeria pero se
excluye al reconocer: es una foto que el reconocedor nunca vio.
"""
import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def duro(mundo):
    """Condiciones peores que las normales, distintas (pero repetibles) para cada mundo: odometria con
    error sistematico de hasta +-4 %, mas ruido, menos fotos por segundo y mas retraso de la camara."""
    import zlib
    import numpy as np
    r = np.random.default_rng(zlib.crc32(mundo.encode()))
    return ["--odom-sesgo", f"{r.uniform(-0.04, 0.04):.3f}", "--odom-ruido", "0.02",
            "--cam-periodo", f"{r.uniform(0.5, 0.75):.2f}", "--cam-latencia", f"{r.uniform(0.45, 0.7):.2f}"]


def una(mundo, opciones, max_t, videos=None, videos_todos=False, modo_duro=False):
    import carrera
    extra = duro(mundo) if modo_duro else []
    args = carrera.parse(["--sim", "--go", "--no-gui", "--mudo", "--no-guardar", "--mundo", mundo] + extra + opciones)
    if videos:
        args.video = str(Path(videos) / (mundo.replace(":", "_") + ".mp4"))
    m, robot = carrera.crear_sim(args)
    t = time.time()
    try:
        res = carrera.correr(robot, args, sim=True, gui=False, mundo=m, max_t=max_t)
    except Exception as e:                                  # un fallo de codigo no tumba el banco
        import traceback
        res = dict(mundo=mundo, exito=False, estado="ERROR", error=f"{type(e).__name__}: {e}", tiempo=None,
                   traza_error=traceback.format_exc().strip().splitlines()[-6:],
                   choques=None, acierto=None, en_a=None, nombre=None, persona=m.persona, log=[])
    res["cpu_s"] = round(time.time() - t, 1)
    if modo_duro:
        res["duro"] = " ".join(extra)
    v = res.get("video")
    if v and res.get("exito") and not (res.get("choques") or 0) and not videos_todos:
        Path(v).unlink(missing_ok=True)                     # solo se guardan los que hay que mirar
        res["video"] = None
    return res


def resumen(rs, titulo):
    n = len(rs)
    ok = [r for r in rs if r.get("exito")]
    t = sorted(r["tiempo"] for r in ok)
    mal = [r for r in rs if r.get("estado") == "HECHO" and not r.get("acierto")]
    print(f"\n== {titulo}: {len(ok)}/{n} exitos ({100 * len(ok) / max(n, 1):.0f}%)")
    if t:
        print(f"   tiempo (exitos): mediana {t[len(t) // 2]:.1f} s, min {t[0]:.1f}, max {t[-1]:.1f}")
        tb = sorted(r["tiempo_b"] for r in ok if r.get("tiempo_b"))
        if tb:
            print(f"   reconocida a los (exitos): mediana {tb[len(tb) // 2]:.1f} s, max {tb[-1]:.1f}")
    print(f"   choques: {sum(r.get('choques') or 0 for r in rs)} en total, "
          f"{sum(1 for r in rs if (r.get('choques') or 0) > 0)} corridas con choque")
    print(f"   nombre equivocado: {len(mal)}" + "".join(f"  [{r['mundo']}: {r['persona']} -> {r['nombre']}]" for r in mal))
    con_qr = [r for r in rs if "qr_ok" in r]
    if con_qr:
        qr_mal = [r for r in con_qr if r.get("palabra") and not r.get("qr_ok")]
        sin = [r for r in con_qr if r.get("estado") == "HECHO" and not r.get("palabra")]
        print(f"   QR: {sum(1 for r in con_qr if r.get('qr_ok'))}/{len(con_qr)} bien, {len(qr_mal)} con la palabra "
              f"EQUIVOCADA, {len(sin)} volvieron sin leerlo"
              + "".join(f"  [{r['mundo']}: '{r.get('palabra')}']" for r in qr_mal))
    for r in rs:
        if not r.get("exito"):
            ult = r.get("log")[-1] if r.get("log") else r.get("error", "")
            print(f"   FALLA {r['mundo']:<18} estado={r['estado']:<9} choques={r.get('choques')} t={r.get('tiempo')}  {ult}")
    for r in rs:
        if r.get("video"):
            print(f"   video {r['mundo']:<18} -> {r['video']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=20, help="circuitos aleatorios")
    ap.add_argument("--desde", type=int, default=1, help="primera semilla")
    ap.add_argument("--mundos", nargs="*", default=None, help="mundos concretos en vez de aleatorios")
    ap.add_argument("--solo-con", action="store_true", help="solo con flechas")
    ap.add_argument("--solo-sin", action="store_true", help="solo sin flechas")
    ap.add_argument("--max-t", type=float, default=180.0, help="s maximos por corrida (limite de la competencia: 3 min)")
    ap.add_argument("--procesos", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--json", default=None)
    ap.add_argument("--videos", default=None, help="carpeta: video de cada corrida que falla o choca")
    ap.add_argument("--videos-todos", action="store_true", help="con --videos: guardar el video de todas")
    ap.add_argument("--duro", action="store_true",
                    help="condiciones dificiles: odometria con sesgo +-4%%, camara lenta y con retraso")
    ap.add_argument("--opciones", nargs=argparse.REMAINDER, default=[], help="opciones para carrera.py (al final)")
    a = ap.parse_args(argv)
    if a.mundos:
        grupos = {"mundos": a.mundos}
    else:
        sem = range(a.desde, a.desde + a.n)
        grupos = {}
        if not a.solo_sin:
            grupos["con flechas"] = [f"aleatorio:{s}" for s in sem]
        if not a.solo_con:
            grupos["sin flechas"] = [f"aleatorio:{s}:sin" for s in sem]
    t0 = time.time()
    todos = {}
    with ProcessPoolExecutor(a.procesos) as ex:
        futs = {g: [ex.submit(una, m, a.opciones, a.max_t, a.videos, a.videos_todos, a.duro) for m in ms]
                for g, ms in grupos.items()}
        for g, fs in futs.items():
            todos[g] = [f.result() for f in fs]
    for g, rs in todos.items():
        resumen(rs, g)
    print(f"\n({sum(len(v) for v in todos.values())} corridas en {time.time() - t0:.0f} s, {a.procesos} procesos)")
    if a.json:
        Path(a.json).write_text(json.dumps(todos, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
