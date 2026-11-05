"""
SLAM 2D en la PC con el robot por UDP (Tarea A, slam-1): conduces el robot con el teclado por el
laberinto y el mapa se va construyendo en una ventana. No necesita ROS.

Uso (desde la raiz del repositorio, con el puente corriendo en el robot):
  python3 task-a/slam-1/slam_run.py --drive               # conducir y mapear
  python3 task-a/slam-1/slam_run.py --drive --undock      # primero sacarlo del dock
  python3 task-a/slam-1/slam_run.py                       # solo mapear (otra PC conduce)
  python3 task-a/slam-1/slam_run.py --engine odom         # comparar: odometria pura, sin BreezySLAM

Teclas (ventana enfocada):
  w/s adelante/atras (mantener)   a/d girar   espacio parar   +/- velocidad
  u sacar del dock   l ver/ocultar el LiDAR en vivo (amarillo)   m guardar el mapa ahora   q / Esc salir (guarda)
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))                      # slam_core
sys.path.insert(0, str(HERE.parents[1]))           # raiz del repo: tb4

from slam_core import make_engine                  # noqa: E402
from tb4 import TB4Client                          # noqa: E402

HOLD = 0.5                    # s que dura una pulsacion (mismo criterio que herramientas/monitor)
KEYS = {ord("w"): (1, 0), ord("s"): (-1, 0), ord("a"): (0, 1), ord("d"): (0, -1)}
MAX_AGE = 0.5                 # s: no se usa un scan/pose mas viejo


def render(engine, trail, view, status, hz, driving, scan=None):
    img = engine.map_image()
    n = img.shape[0]
    res_m = None
    img = cv2.cvtColor(np.flipud(img), cv2.COLOR_GRAY2BGR)       # y hacia arriba
    scale = max(1, view // n)
    img = cv2.resize(img, (n * scale, n * scale), interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return int((n / 2 + x / engine.res) * scale), int((n - 1 - (n / 2 + y / engine.res)) * scale)

    for a, b in zip(trail, trail[1:]):
        cv2.line(img, px(*a), px(*b), (200, 120, 0), 1)
    x, y, th = engine.position()
    p = px(x, y)
    tip = (int(p[0] + 14 * math.cos(th)), int(p[1] - 14 * math.sin(th)))
    if scan is not None:                                  # LiDAR en vivo, llevado al marco del mapa
        sx, sy = scan.points()
        c, s = math.cos(th), math.sin(th)
        for wx, wy in zip(x + c * sx - s * sy, y + s * sx + c * sy):
            cv2.circle(img, px(wx, wy), 1, (0, 255, 255), -1)
    cv2.circle(img, p, 5, (0, 0, 255), -1)
    cv2.arrowedLine(img, p, tip, (0, 0, 255), 2, tipLength=0.5)

    lines = [f"{engine.name}  {hz:.1f} Hz  {'CONDUCIENDO' if driving else 'solo mapa'}",
             f"x={x:+.2f} y={y:+.2f} m  rumbo={math.degrees(th):+.0f} deg"]
    if status is not None:
        lines.append(f"bateria {status.battery * 100:.0f}%  frente libre {status.front_clear:.2f} m"
                     + ("  SIN LIDAR (dock?)" if status.scan_stale else "")
                     + (f"  CONDUCTOR: {'SI' if status.is_driver else 'NO (otra PC manda)'}" if driving else ""))
    for i, t in enumerate(lines):
        cv2.putText(img, t, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1, cv2.LINE_AA)
    return img


def save_map(engine, outdir, size_m):
    outdir = Path(outdir) / time.strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    raw = engine.map_image()
    cv2.imwrite(str(outdir / "mapa.png"), np.flipud(raw))        # y hacia arriba
    np.save(outdir / "mapa.npy", raw)
    x, y, th = engine.position()
    (outdir / "mapa.json").write_text(json.dumps({
        "motor": engine.name, "pixels": int(raw.shape[0]), "metros_por_pixel": size_m / raw.shape[0],
        "tamano_m": size_m, "origen": "centro del mapa = inicio del robot; x adelante, y izquierda",
        "filas": "fila = y (en mapa.npy); mapa.png esta volteado para y hacia arriba",
        "valores": "0 ocupado, 127 desconocido, 255 libre", "pose_final_m_rad": [x, y, th]}, indent=2))
    print(f"[ok] mapa guardado en {outdir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--drive", action="store_true", help="conducir con el teclado")
    ap.add_argument("--undock", action="store_true", help="sacar el robot del dock al empezar")
    ap.add_argument("--engine", choices=("auto", "breezy", "odom"), default="auto")
    ap.add_argument("--map-m", type=float, default=20.0, help="lado del mapa en m (defecto 20)")
    ap.add_argument("--pixels", type=int, default=500, help="lado del mapa en pixeles (defecto 500 = 4 cm/px)")
    ap.add_argument("--view", type=int, default=800, help="tamano de la ventana en px")
    ap.add_argument("--vel", type=float, default=0.12, help="velocidad lineal al conducir (m/s)")
    ap.add_argument("--out", default=str(HERE / "mapas"), help="carpeta donde guardar los mapas")
    args = ap.parse_args()

    engine = make_engine(args.engine, size_m=args.map_m, pixels=args.pixels)
    engine.res = args.map_m / args.pixels
    print(f"[..] motor: {engine.name}; mapa {args.map_m:.0f} m, {engine.res * 100:.1f} cm/px")

    vel, wvel = args.vel, 0.8
    key, key_t = None, 0.0
    trail, last_rx, hz, t_last = [], None, 0.0, time.monotonic()
    show_scan = True

    with TB4Client(args.ip) as robot:
        robot.wait_ready(8)
        if args.undock:
            print("[..] sacando del dock ...")
            robot.undock()
        try:
            while True:
                scan, pose = robot.scan(), robot.pose()
                if (scan is not None and pose is not None and scan.received != last_rx
                        and scan.age < MAX_AGE and pose.age < MAX_AGE):
                    engine.update(scan, pose)
                    last_rx = scan.received
                    now = time.monotonic()
                    hz = 0.8 * hz + 0.2 / max(now - t_last, 1e-3)
                    t_last = now
                    x, y, _ = engine.position()
                    if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
                        trail.append((x, y))

                if args.drive:
                    v, w = (0.0, 0.0)
                    if key in KEYS and time.monotonic() - key_t < HOLD:
                        v, w = KEYS[key][0] * vel, KEYS[key][1] * wvel
                    robot.move(v, w)

                live = scan if (show_scan and scan is not None and scan.age < MAX_AGE) else None
                cv2.imshow("SLAM slam-1", render(engine, trail, args.view, robot.status(), hz, args.drive, live))
                k = cv2.waitKey(30) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k in KEYS:
                    key, key_t = k, time.monotonic()
                elif k == ord(" "):
                    key = None
                elif k in (ord("+"), ord("=")):
                    vel = min(vel + 0.02, 0.25)
                elif k == ord("-"):
                    vel = max(vel - 0.02, 0.04)
                elif k == ord("u"):
                    print("[..] sacando del dock ...")
                    robot.undock()
                elif k == ord("l"):
                    show_scan = not show_scan
                elif k == ord("m"):
                    save_map(engine, args.out, args.map_m)
        except KeyboardInterrupt:
            pass
        finally:
            robot.stop()
            save_map(engine, args.out, args.map_m)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
