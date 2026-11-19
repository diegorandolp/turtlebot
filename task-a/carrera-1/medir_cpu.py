"""
Cuanto CPU necesita carrera-1, pieza por pieza. Sirve para decidir si todo puede correr EN el TurtleBot
(Raspberry Pi) en vez de en la laptop. No necesita robot: usa fotos y scans del simulador.

  python3 task-a/carrera-1/medir_cpu.py            # en la PC o, por SSH, en la Raspberry del robot
  python3 task-a/carrera-1/medir_cpu.py --hd       # con fotos HD (1280x720)

Imprime los ms por llamada de cada pieza y el % de UN nucleo que usa la carrera a su ritmo normal
(fotos cada 0.15 s, LiDAR a 7 Hz, control a 20 Hz). La Raspberry Pi 4 tiene 4 nucleos, pero ROS, el
LiDAR, la camara y el puente ya usan parte: conviene que la carrera quede bastante por debajo de 1 nucleo.
"""
import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for p in (HERE, HERE.parent / "auto-1", HERE.parent / "slam-1", ROOT):
    sys.path.insert(0, str(p))


def medir(f, n, *a):
    f(*a)                                                        # la primera llamada carga cosas: no cuenta
    t = time.perf_counter()
    for _ in range(n):
        f(*a)
    return (time.perf_counter() - t) / n * 1000.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hd", action="store_true")
    ap.add_argument("--n", type=int, default=30, help="repeticiones por pieza")
    ap.add_argument("--hilos", type=int, default=1, help="hilos de OpenCV (defecto 1: lo mas parecido a un nucleo)")
    a = ap.parse_args(argv)
    cv2.setNumThreads(a.hilos)

    import carrera
    from flechas import DetectorFlechas
    from mundos import construir
    from navegador import carriles, ventana_dinamica
    from reconocimiento import Reconocedor
    from simulador import SimRobot
    from slam_core import make_engine
    from planner import Planner

    m = construir("demo")
    robot = SimRobot(m, foto_hd=(1280, 720, 66.0) if a.hd else None)
    # pararse a ~0.9 m delante de la foto de B, mirandola (para que haya cara en las fotos)
    k = next(k for k in m.carteles if k.tipo == "rostro")
    (ax, ay), (bx, by) = k.a, k.b
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    nx, ny = -(by - ay), bx - ax                                 # normal del cartel (hacia donde mira)
    L = float(np.hypot(nx, ny))
    robot.wx, robot.wy = cx + 0.9 * nx / L, cy + 0.9 * ny / L
    fotos = []
    for j in range(8):
        robot.wth = float(np.arctan2(cy - robot.wy, cx - robot.wx)) + (j - 4) * 0.03
        fotos.append(robot.photo(hd=a.hd))
    scan = robot.scan()
    rec = Reconocedor("mbf", espejo=False)                            # como la carrera
    det_fl = DetectorFlechas()
    caras = rec.det(fotos[4])
    print(f"maquina: {os.uname().machine}, {os.cpu_count()} nucleos, OpenCV {cv2.__version__}, hilos {a.hilos}")
    print(f"foto {fotos[0].shape[1]}x{fotos[0].shape[0]}, caras en la foto de prueba: {len(caras)}\n")

    t = {}
    t["detectar caras (YuNet)"] = medir(lambda: rec.det(fotos[4]), a.n)
    if caras:
        t["reconocer 1 cara (ArcFace mbf)"] = medir(lambda: rec.resultado(fotos[4], caras[0]), a.n)
    t["detectar flechas"] = medir(lambda: det_fl(fotos[4]), a.n)
    t["carriles (LiDAR)"] = medir(lambda: carriles(scan), a.n)
    sx, sy = scan.points()
    ok = np.hypot(sx, sy) > 0.12
    t["ventana dinamica"] = medir(lambda: ventana_dinamica(sx[ok], sy[ok], 0.3, 0.3, 1.8), a.n)
    eng = make_engine("odom", size_m=carrera.MAP_M, pixels=carrera.MAP_PX)
    pose = robot.pose()
    t["mapa: integrar 1 scan"] = medir(lambda: eng.update(scan, pose), a.n)
    for _ in range(5):
        eng.update(scan, pose)
    mapa = eng.map_image()
    pl = Planner(carrera.MAP_M, carrera.MAP_PX, cell=2, robot_radius=0.17, margin=0.05)
    t["ruta en el mapa (Dijkstra)"] = medir(lambda: pl.plan_to(mapa, (0.0, 0.0), (1.0, 0.5)), max(5, a.n // 3))

    for k, v in t.items():
        print(f"  {k:<32} {v:8.1f} ms")
    # ritmo de la carrera: fotos cada 0.15 s (caras + flechas, y reconocer cuando hay cara), LiDAR 7 Hz,
    # control 20 Hz (ventana dinamica), ruta cada 2 s en la vuelta
    foto = t["detectar caras (YuNet)"] + t["detectar flechas"] + t.get("reconocer 1 cara (ArcFace mbf)", 0.0)
    por_s = (foto * (1 / 0.15) + (t["carriles (LiDAR)"] + t["mapa: integrar 1 scan"]) * 7
             + t["ventana dinamica"] * 20 + t["ruta en el mapa (Dijkstra)"] * 0.5)
    print(f"\n  por foto (caras + flechas + reconocer): {foto:.0f} ms -> como maximo {1000 / max(foto, 1e-3):.1f} fotos/s")
    print(f"  carrera a ritmo normal: {por_s / 10:.0f} % de un nucleo (sin ventana ni video)")
    return t


if __name__ == "__main__":
    main()
