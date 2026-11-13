"""
Banco de pruebas: corre la carrera completa en muchos circuitos aleatorios del simulador (sin
ventana, en paralelo) y resume exito, tiempos, choques y errores. Es la medida de que la estrategia
generaliza a circuitos que no conoce, con y sin flechas.

  python task-a/carrera-1/banco_pruebas.py                         # 20 circuitos con flechas + los mismos sin
  python task-a/carrera-1/banco_pruebas.py --n 50 --desde 100      # semillas 100..149
  python task-a/carrera-1/banco_pruebas.py --mundos demo sala --opciones --temprano
  python task-a/carrera-1/banco_pruebas.py --json resultados.json  # ademas guarda cada corrida

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


def una(mundo, opciones, max_t):
    import carrera
    from mundos import construir
    from simulador import SimRobot
    args = carrera.parse(["--sim", "--go", "--no-gui", "--mudo", "--no-guardar", "--mundo", mundo] + opciones)
    m = construir(mundo)
    robot = SimRobot(m, hfov=args.hfov, camara_ok=not args.camara_caida,
                     foto_hd=(1280, 720, args.hfov_hd) if args.hd else None)
    t = time.time()
    try:
        res = carrera.correr(robot, args, sim=True, gui=False, mundo=m, max_t=max_t)
    except Exception as e:                                  # un fallo de codigo no tumba el banco
        res = dict(mundo=mundo, exito=False, estado="ERROR", error=f"{type(e).__name__}: {e}", tiempo=None,
                   choques=None, acierto=None, en_a=None, nombre=None, persona=m.persona, log=[])
    res["cpu_s"] = round(time.time() - t, 1)
    return res


def resumen(rs, titulo):
    n = len(rs)
    ok = [r for r in rs if r.get("exito")]
    t = sorted(r["tiempo"] for r in ok)
    mal = [r for r in rs if r.get("estado") == "HECHO" and not r.get("acierto")]
    print(f"\n== {titulo}: {len(ok)}/{n} exitos ({100 * len(ok) / max(n, 1):.0f}%)")
    if t:
        print(f"   tiempo (exitos): mediana {t[len(t) // 2]:.1f} s, min {t[0]:.1f}, max {t[-1]:.1f}")
    print(f"   choques: {sum(r.get('choques') or 0 for r in rs)} en total, "
          f"{sum(1 for r in rs if (r.get('choques') or 0) > 0)} corridas con choque")
    print(f"   nombre equivocado: {len(mal)}" + "".join(f"  [{r['mundo']}: {r['persona']} -> {r['nombre']}]" for r in mal))
    for r in rs:
        if not r.get("exito"):
            ult = r.get("log")[-1] if r.get("log") else r.get("error", "")
            print(f"   FALLA {r['mundo']:<18} estado={r['estado']:<9} choques={r.get('choques')} t={r.get('tiempo')}  {ult}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=20, help="circuitos aleatorios")
    ap.add_argument("--desde", type=int, default=1, help="primera semilla")
    ap.add_argument("--mundos", nargs="*", default=None, help="mundos concretos en vez de aleatorios")
    ap.add_argument("--solo-con", action="store_true", help="solo con flechas")
    ap.add_argument("--solo-sin", action="store_true", help="solo sin flechas")
    ap.add_argument("--max-t", type=float, default=240.0)
    ap.add_argument("--procesos", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--json", default=None)
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
        futs = {g: [ex.submit(una, m, a.opciones, a.max_t) for m in ms] for g, ms in grupos.items()}
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
