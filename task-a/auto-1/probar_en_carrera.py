"""
Prueba el explorador de auto-1 (explorar.run, sin cambios) en el simulador de carrera-1: camara
renderizada con la foto real de una persona en B, LiDAR con ruido y lecturas perdidas, odometria con
ruido, freno del puente (tb4.protocol.SafetyFilter) y cuenta de choques. La cara se busca con el
FaceDetector (YuNet) de verdad, no con un "sensor de cara" simulado.

Uso (desde la raiz del repositorio):
  python task-a/auto-1/probar_en_carrera.py                         # demo-sin-flechas y sala, sin ventana
  python task-a/auto-1/probar_en_carrera.py --mundos sala --gui     # con ventanas (mapa + mundo real + camara)
  python task-a/auto-1/probar_en_carrera.py --aleatorios 10         # aleatorio:1:sin .. aleatorio:10:sin
  python task-a/auto-1/probar_en_carrera.py --mundos sala --cell 1 --radio 0.17 --volver
Las opciones que no son de este script pasan a explorar.py (--cell, --radio, --vel, --volver, ...).

Mundos (task-a/carrera-1/mundos.py): demo, demo-sin-flechas, sala, aleatorio:N, aleatorio:N:sin.
Por cada mundo guarda en task-a/auto-1/pruebas_carrera/<fecha>/: <mundo>_real.png (recorrido real
desde arriba, la foto de B en magenta), <mundo>_mapa.png (el mapa del SLAM) y resumen.txt.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
# primero el simulador de carrera-1, despues auto-1 (explorar mete slam-1 y la raiz del repo)
sys.path.insert(0, str(REPO / "task-a" / "carrera-1"))
from simulador import SimRobot          # noqa: E402
from mundos import construir            # noqa: E402
sys.path.insert(0, str(HERE))
import explorar                         # noqa: E402
from face import FaceDetector           # noqa: E402

HFOV = 40.0          # grados de la preview 250x250 (supuesto de carrera-1, sin medir en el robot)


class RobotConVentana(SimRobot):
    """SimRobot que, con --gui, muestra tambien el mundo real y la ultima foto de la camara."""
    gui = False
    _ultima = None

    def photo(self, *a, **k):
        img = super().photo(*a, **k)
        self._ultima = img
        return img

    def advance(self, dt):
        super().advance(dt)
        if self.gui and int(self.time / dt) % 3 == 0:
            cv2.imshow("simulador: mundo real", self.vista())
            if self._ultima is not None:
                cv2.imshow("simulador: camara", cv2.resize(self._ultima, (500, 500), interpolation=cv2.INTER_NEAREST))


def una(nombre, extra, gui, max_t, ruido, out):
    m = construir(nombre)
    robot = RobotConVentana(m, hfov=HFOV, odom_ruido=ruido)
    robot.gui = gui
    robot.start = m.inicio                                  # explorar.run lo usa en el resumen
    args = explorar.parse(["--sim", "--go"] + extra)
    args.vel = min(args.vel, 0.20)
    det = FaceDetector()
    caras = []                                              # pose real de cada foto en que se detecto una cara

    def detect(img):
        ok = det(img)
        if ok:
            caras.append(robot.verdad())
        return ok

    slug = nombre.replace(":", "_")
    res = explorar.run(robot, args, sim=True, gui=gui, detect=detect, max_time=max_t,
                       out_png=out / f"{slug}_mapa.png")
    cv2.imwrite(str(out / f"{slug}_real.png"), robot.vista())
    x, y, _ = robot.verdad()
    bx, by = m.b
    ax, ay, _ = m.inicio
    d_b = math.hypot(x - bx, y - by)
    estado = res.get("state")
    # B confirmado por el explorador Y alguna deteccion ocurrio cerca de la foto real (no un falso positivo)
    cerca = any(math.hypot(cx - bx, cy - by) < 2.0 for cx, cy, _ in caras)
    return dict(mundo=nombre, estado=estado, encontro_B=(res.get("b_xy") is not None and cerca),
                persona=m.persona, t=round(res.get("time", 0.0), 1), choques=robot.collisions,
                fotos=robot.fotos, fotos_con_cara=len(caras), dist_a_B=round(d_b, 2),
                dist_a_A=round(math.hypot(x - ax, y - ay), 2), log=res.get("log", [])[-3:])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mundos", nargs="+", default=None)
    ap.add_argument("--aleatorios", type=int, default=0, help="ademas, aleatorio:1:sin .. aleatorio:N:sin")
    ap.add_argument("--gui", action="store_true", help="ventanas en vivo (mas lento)")
    ap.add_argument("--max-t", type=float, default=600.0, help="s simulados maximos por mundo")
    ap.add_argument("--ruido", type=float, default=0.01, help="ruido de odometria del simulador")
    a, extra = ap.parse_known_args()
    mundos = list(a.mundos or ([] if a.aleatorios else ["demo-sin-flechas", "sala"]))
    mundos += [f"aleatorio:{i}:sin" for i in range(1, a.aleatorios + 1)]
    out = HERE / "pruebas_carrera" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    lineas = [f"opciones de explorar: {extra}"]
    for n in mundos:
        r = una(n, extra, a.gui, a.max_t, a.ruido, out)
        txt = (f"{r['mundo']:<18} {'OK  ' if r['encontro_B'] else 'FALLA'} {r['estado']:<8} t={r['t']:>6}s "
               f"a {r['dist_a_B']:.2f} m de B, {r['dist_a_A']:.2f} m de A, choques={r['choques']}, "
               f"fotos con cara {r['fotos_con_cara']}/{r['fotos']}  | {r['log'][-1] if r['log'] else ''}")
        print(txt, flush=True)
        lineas.append(txt)
    ok = sum("OK  " in l for l in lineas)
    lineas.append(f"\n{ok}/{len(mundos)} encontraron B")
    (out / "resumen.txt").write_text("\n".join(lineas) + "\n")
    print(lineas[-1].strip(), f"\nimagenes y resumen en {out}")


if __name__ == "__main__":
    main()
