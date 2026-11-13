"""
Exploracion autonoma hasta encontrar un rostro (punto B) - Tarea A, auto-1.

El robot parte de A (el punto donde lo enciendes/sacas del dock), construye un mapa con SLAM
(BreezySLAM, task-a/slam-1) y va solo hacia las zonas aun desconocidas hasta que la camara ve una
cara: ese sitio es B. Opcionalmente despues vuelve a A. Todo corre en la PC; el robot recibe
move(v, w) por UDP y conserva su freno de emergencia por LiDAR.

Uso (desde la raiz del repositorio, con el entorno virtual activado):
  python task-a/auto-1/explorar.py --undock            # arranca EN PAUSA: pulsa g para empezar
  python task-a/auto-1/explorar.py --go --volver       # empieza ya y regresa a A al encontrar B
  python task-a/auto-1/explorar.py --sim               # prueba en el simulador, sin robot

Teclas (ventana enfocada):
  g empezar/reanudar     espacio PAUSA (parada inmediata)     l ver/ocultar LiDAR
  m guardar mapa         q / Esc salir (parada y guarda el mapa)
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))                          # planner, explorer, face, sim
sys.path.insert(0, str(HERE.parent / "slam-1"))        # slam_core, slam_run
sys.path.insert(0, str(HERE.parents[1]))               # raiz del repo: tb4

import slam_run                                        # noqa: E402  (render, save_map)
from coverage import Coverage                           # noqa: E402
from explorer import (BARRIDO, BUSCANDO, CONFIRMAR, EXPLORAR, HECHO, INSPECCIONAR, PAUSA, SIN_B, VOLVER,   # noqa: E402
                      Explorer)
from face import FaceDetector, FaceWatcher, ThreadedWatcher              # noqa: E402
from planner import Planner                             # noqa: E402
from slam_core import make_engine                       # noqa: E402

MAX_AGE = 0.5
ACTIVOS = BUSCANDO + (CONFIRMAR,)                      # estados en los que la camara pide fotos
COLORS = {PAUSA: (0, 200, 255), EXPLORAR: (0, 255, 0), CONFIRMAR: (255, 0, 255), BARRIDO: (255, 200, 0), INSPECCIONAR: (0, 200, 255),
          VOLVER: (255, 200, 0), HECHO: (0, 255, 0), SIN_B: (0, 0, 255)}


def draw(engine, ex, trail, status, hz, scan, view, show_scan, watcher=None):
    img = slam_run.render(engine, trail, view, status, hz, False, scan if show_scan else None)
    n = engine.n
    scale = max(1, view // n)
    cov = ex.cov
    cov_txt = ""
    if cov is not None:
        mp = engine.map_image()
        surface, W, _ = cov.weights(mp)
        if cov.pl.m * cov.pl.f == n:                                           # zonas ya vistas por la camara: tinte verde
            ov = cv2.resize(cov.seen.astype(np.uint8), (n * scale, n * scale), interpolation=cv2.INTER_NEAREST)[::-1].astype(bool)
            tint = img.copy()
            tint[ov] = (0, 170, 0)
            img = cv2.addWeighted(img, 0.78, tint, 0.22, 0)
        pend = np.argwhere((W >= cov.ws * 0.99) & surface)                     # superficies aun sin ver: puntos rojos
        for r, c in pend[::2]:
            x, y = cov.pl.to_xy(r, c)
            cv2.circle(img, (int((n / 2 + x / engine.res) * scale), int((n - 1 - (n / 2 + y / engine.res)) * scale)), 1, (0, 0, 255), -1)
        vistas, total = cov.stats(mp)
        cov_txt = f"cobertura camara: {vistas}/{total} superficies ({100 * vistas // max(total, 1)}%)  barridos={ex.n_sweeps}"

    def px(x, y):
        return int((n / 2 + x / engine.res) * scale), int((n - 1 - (n / 2 + y / engine.res)) * scale)

    if ex.view is not None:
        vx, vy, vh = ex.view
        cv2.arrowedLine(img, px(vx, vy), px(vx + 0.35 * math.cos(vh), vy + 0.35 * math.sin(vh)), (0, 200, 255), 2, tipLength=0.4)   # punto de vista objetivo
    pl = ex.plan
    if pl is not None and pl.frontier_xy is not None:
        for fx, fy in pl.frontier_xy:
            cv2.circle(img, px(fx, fy), 1, (255, 255, 0), -1)               # fronteras (cian)
    for a, b in zip(ex.path, ex.path[1:]):
        cv2.line(img, px(*a), px(*b), (0, 220, 0), 1)                        # ruta (verde)
    if ex.goal is not None:
        cv2.drawMarker(img, px(*ex.goal), (0, 200, 0), cv2.MARKER_CROSS, 10, 2)
    cv2.drawMarker(img, px(*ex.home), (255, 255, 255), cv2.MARKER_DIAMOND, 10, 1)    # A
    if ex.b_xy is not None:
        cv2.drawMarker(img, px(*ex.b_xy), (255, 0, 255), cv2.MARKER_STAR, 14, 2)      # B
    if watcher is not None:                                                    # estado de la camara
        if watcher.errors and watcher.last_error and watcher.photos == 0:
            cam, col = f"CAMARA SIN IMAGEN ({watcher.last_error})", (0, 0, 255)
        elif ex.state in ACTIVOS:
            cam, col = f"camara: {watcher.photos} fotos, {watcher.errors} errores, cara en ultima foto: {'SI' if watcher.last_seen else 'no'}", (200, 200, 200)
        else:
            cam, col = "camara: NO busca caras en PAUSA/HECHO (pulsa g)", (0, 200, 255)
        cv2.putText(img, cam, (8, img.shape[0] - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    if cov_txt:
        cv2.putText(img, cov_txt, (8, img.shape[0] - 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 255, 120), 1, cv2.LINE_AA)
    cv2.rectangle(img, (0, img.shape[0] - 28), (img.shape[1], img.shape[0]), (30, 30, 30), -1)
    note = ex.log[-1] if ex.log else ""
    cv2.putText(img, f"{ex.state}  fronteras={pl.n_frontier_cells if pl else 0}  {note}",
                (8, img.shape[0] - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[ex.state], 1, cv2.LINE_AA)
    return img


def guardar_mapa(engine, carpeta, size_m, ultimo=True):
    """Guarda el mapa del SLAM en <carpeta>/<fecha>/ y, si `ultimo`, tambien en <carpeta>/ultimo/
    (el que carga --mapa ultimo). Devuelve la carpeta con fecha."""
    import json
    import shutil
    raw = engine.map_image()
    dest = Path(carpeta) / time.strftime("%Y%m%d_%H%M%S")
    dest.mkdir(parents=True, exist_ok=True)
    np.save(dest / "mapa.npy", raw)
    cv2.imwrite(str(dest / "mapa.png"), np.flipud(raw))
    (dest / "mapa.json").write_text(json.dumps({
        "pixels": int(raw.shape[0]), "tamano_m": size_m, "metros_por_pixel": size_m / raw.shape[0],
        "origen": "centro = punto A (donde arranco el robot), x adelante, y izquierda",
        "nota": "para reutilizarlo, el robot debe arrancar en el MISMO punto A y con la MISMA orientacion"}, indent=2))
    if ultimo:
        u = Path(carpeta) / "ultimo"
        shutil.rmtree(u, ignore_errors=True)
        shutil.copytree(dest, u)
    print(f"[ok] mapa guardado en {dest}" + (f" (y en {Path(carpeta) / 'ultimo'})" if ultimo else ""))
    return dest


def cargar_mapa(ruta, carpeta, pixels):
    """--mapa ultimo | <carpeta con mapa.npy> | <archivo .npy>."""
    p = Path(carpeta) / "ultimo" if ruta == "ultimo" else Path(ruta)
    if p.is_dir():
        p = p / "mapa.npy"
    if not p.exists():
        sys.exit(f"[error] no encuentro el mapa {p}. Primero haz un intento de mapeo (ver README).")
    m = np.load(p)
    if m.shape != (pixels, pixels):
        sys.exit(f"[error] el mapa {p} es de {m.shape[0]} px y ahora se usan {pixels}: usa el mismo --pixels/--map-m")
    print(f"[ok] mapa previo cargado: {p}")
    return m


def combinar(prior, live, occ_thr=90, unk=(100, 150)):
    """Mapa para planificar: el del intento anterior (completo) + lo que el LiDAR ve ahora:
    paredes/obstaculos nuevos y zonas que el mapa previo no conocia."""
    out = prior.copy()
    nuevo = live < occ_thr
    out[nuevo] = live[nuevo]
    desconocido = (prior > unk[0]) & (prior < unk[1])
    out[desconocido] = live[desconocido]
    return out


def run(robot, args, sim=False, gui=True, detect=None, max_time=1800.0, dt=0.15, out_png=None):
    """Bucle principal. Devuelve un resumen. `robot` = TB4Client o SimRobot."""
    engine = make_engine(args.engine, size_m=args.map_m, pixels=args.pixels,
                         sigma_xy_mm=args.sigma_xy, sigma_theta_deg=args.sigma_th)
    planner = Planner(args.map_m, args.pixels, cell=args.cell, robot_radius=args.radio)
    cov = None if args.sin_cobertura else Coverage(planner, fov_deg=args.cam_fov, rng=args.cam_rango)
    if getattr(args, "mapas_dir", None) is None:          # los mapas del simulador no se mezclan con los reales
        args.mapas_dir = str(HERE / ("mapas_sim" if sim else "mapas"))
    prior = cargar_mapa(args.mapa, args.mapas_dir, args.pixels) if args.mapa else None
    if prior is not None and cov is None:
        print("[aviso] --mapa sin cobertura de camara no sirve de nada: quita --sin-cobertura")
    ex = Explorer(planner, cov=cov, vmax=args.vel, volver=args.volver, inspeccion_directa=prior is not None)
    watcher = FaceWatcher()
    tw = None if sim else ThreadedWatcher(watcher, robot.photo, detect)    # robot real: camara en otro hilo
    if args.go:
        ex.start()
    trail, last_rx, hz, t_prev, show_scan, last_photos = [], None, 0.0, None, True, 0
    t_start = robot.time if sim else time.monotonic()
    finished_at, summary = None, {}

    try:
        while True:
            now = robot.time if sim else time.monotonic()
            scan, pose = robot.scan(), robot.pose()
            if (scan is not None and pose is not None and scan.received != last_rx
                    and scan.age < MAX_AGE and pose.age < MAX_AGE):
                engine.update(scan, pose)
                last_rx = scan.received
                if t_prev is not None:
                    hz = 0.8 * hz + 0.2 / max(now - t_prev, 1e-3)
                t_prev = now
            pos = engine.position()
            if not trail or math.hypot(pos[0] - trail[-1][0], pos[1] - trail[-1][1]) > 0.05:
                trail.append(pos[:2])

            if tw is not None:
                tw.active = ex.state in ACTIVOS
            if ex.state in ACTIVOS:                                # buscar el rostro
                res = tw.status if tw is not None else watcher.update(now, robot.photo, detect)
                if res == "confirmed":
                    ex.face_confirmed(pos)
                elif res == "candidate":
                    ex.face_candidate()
                elif ex.state == CONFIRMAR:
                    ex.face_rejected()
            if cov is not None and watcher.photos != last_photos and scan is not None:
                cov.mark(pos, scan)                                # esta foto cubrio el cono frente al robot
                last_photos = watcher.photos
            lw = watcher.last_ok_t
            cam_ok = (now - lw < 6.0) if lw is not None else (watcher.errors == 0 or now - t_start < 10.0)

            mapa = engine.map_image() if prior is None else combinar(prior, engine.map_image())
            v, w = ex.step(now, mapa, pos, scan, photos=watcher.photos, cam_ok=cam_ok)
            robot.move(v, w)
            if sim:
                robot.advance(dt)

            if gui:
                cv2.imshow("auto-1 exploracion", draw(engine, ex, trail, robot.status(), hz, scan, args.view, show_scan, watcher))
                k = cv2.waitKey(1 if sim else 30) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k == ord(" "):
                    ex.pause()
                    robot.stop()
                elif k == ord("g"):
                    if ex.state == PAUSA:
                        ex.start()
                elif k == ord("l"):
                    show_scan = not show_scan
                elif k == ord("m"):
                    slam_run.save_map(engine, args.out, args.map_m)
            elif not sim:
                time.sleep(0.05)
            if not sim and not gui:
                pass

            if ex.state in (HECHO, SIN_B):
                finished_at = finished_at or now
                if now - finished_at > 1.0:
                    break
            if now - t_start > max_time:
                ex.log.append("tiempo maximo alcanzado")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if tw is not None:
            tw.close()
        robot.stop()
        vistas, total = (cov.stats(engine.map_image()) if cov is not None else (0, 0))
        summary = dict(state=ex.state, b_xy=ex.b_xy, barridos=ex.n_sweeps, fotos=watcher.photos, cobertura=(vistas, total), pos=engine.position(), time=(robot.time if sim else time.monotonic()) - t_start,
                       log=ex.log, bad_goals=len(ex.bad_goals))
        if sim:
            ex_, ey_ = robot.wx - robot.start[0], robot.wy - robot.start[1]
            summary.update(collisions=robot.collisions, true_pos=(robot.wx, robot.wy),
                           slam_err=math.hypot(engine.position()[0] - ex_, engine.position()[1] - ey_))
        if out_png and gui is False:
            img = draw(engine, ex, trail, robot.status(), hz, None, args.view, False)
            cv2.imwrite(str(out_png), img)
        if not args.no_guardar:
            # el mapa de un intento de mapeo pasa a ser el "ultimo"; el de una inspeccion (--mapa) se guarda
            # con fecha pero no lo sustituye (empieza de cero y puede estar incompleto)
            guardar_mapa(engine, args.mapas_dir, args.map_m, ultimo=prior is None)
        if gui:
            cv2.destroyAllWindows()
    return summary


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--sim", action="store_true", help="simulador en vez del robot real")
    ap.add_argument("--no-gui", action="store_true", help="sin ventana (solo con --sim)")
    ap.add_argument("--go", action="store_true", help="empezar sin esperar la tecla g (por defecto arranca en pausa)")
    ap.add_argument("--volver", action="store_true", help="al encontrar B, regresar a A")
    ap.add_argument("--undock", action="store_true", help="sacar del dock al empezar")
    ap.add_argument("--vel", type=float, default=0.12, help="velocidad maxima (m/s, defecto 0.12)")
    ap.add_argument("--radio", type=float, default=0.20, help="radio de seguridad del robot (m)")
    ap.add_argument("--cell", type=int, default=2, help="pixeles del mapa por celda de planificacion")
    ap.add_argument("--engine", choices=("auto", "breezy", "odom"), default="auto")
    ap.add_argument("--sigma-xy", type=float, default=100.0, help="BreezySLAM: busqueda en posicion (mm)")
    ap.add_argument("--sigma-th", type=float, default=20.0, help="BreezySLAM: busqueda en rumbo (grados)")
    ap.add_argument("--cam-fov", type=float, default=60.0, help="campo de vision horizontal de la camara (grados)")
    ap.add_argument("--cam-rango", type=float, default=1.5, help="distancia maxima a la que se detecta una cara/foto (m)")
    ap.add_argument("--sin-cobertura", action="store_true", help="desactiva barridos e inspeccion (solo fronteras)")
    ap.add_argument("--mundo", default="laberinto", help="(--sim) laberinto | lateral")
    ap.add_argument("--sin-camara", action="store_true", help="(--sim) simula la camara caida")
    ap.add_argument("--map-m", type=float, default=20.0)
    ap.add_argument("--pixels", type=int, default=500)
    ap.add_argument("--view", type=int, default=1000, help="tamano de la ventana en px")
    ap.add_argument("--max-time", type=float, default=1800.0, help="s maximos antes de rendirse")
    ap.add_argument("--out", default=str(HERE.parent / "slam-1" / "mapas"), help="carpeta de la tecla m")
    ap.add_argument("--mapa", nargs="?", const="ultimo", default=None,
                    help="intentos 2 y 3: carga el mapa de un intento anterior (por defecto el ultimo) e inspecciona "
                         "directamente. El robot debe arrancar en el MISMO punto A y orientacion")
    ap.add_argument("--mapas-dir", default=None, help="donde se guardan/cargan los mapas (defecto task-a/auto-1/mapas)")
    ap.add_argument("--no-guardar", action="store_true", help="no guardar el mapa al terminar")
    ap.add_argument("--ruido", type=float, default=0.0, help="(--sim) ruido de odometria")
    ap.add_argument("--sesgo", type=float, default=0.0, help="(--sim) error sistematico de odometria, p. ej. 0.05")
    ap.add_argument("--score", type=float, default=0.8, help="confianza minima del detector de caras (defecto 0.8)")
    ap.add_argument("--min-ancho", type=float, default=24, help="ancho minimo de la cara en px de 250 (defecto 24)")
    ap.add_argument("--png", default=None, help="(--sim --no-gui) guardar la imagen final del mapa")
    return ap.parse_args(argv)


def main():
    args = parse()
    args.vel = min(args.vel, 0.20)                       # tope de seguridad
    if args.mapas_dir is None:                           # los mapas del simulador no se mezclan con los reales
        args.mapas_dir = str(HERE / ("mapas_sim" if args.sim else "mapas"))
    if args.sim:
        from sim import SimRobot
        robot = SimRobot(world=args.mundo, odom_noise=args.ruido, odom_bias=args.sesgo, cam_fov_deg=args.cam_fov,
                         cam_range=args.cam_rango, camera_ok=not args.sin_camara)
        detect = lambda img: img.mean() > 200            # noqa: E731  el simulador pinta de blanco la "cara"
        res = run(robot, args, sim=True, gui=not args.no_gui, detect=detect, max_time=args.max_time, out_png=args.png)
    else:
        from tb4 import TB4Client
        detect = FaceDetector(score=args.score, min_width=args.min_ancho)
        with TB4Client(args.ip) as robot:
            robot.wait_ready(8)
            if args.undock:
                print("[..] sacando del dock (retrocede solo: deja espacio detras) ...")
                robot.undock()
            print("[ok] listo. Ventana enfocada: g = empezar, espacio = pausa, q = salir.")
            res = run(robot, args, sim=False, gui=True, detect=detect, max_time=args.max_time)
    print("[fin]", res)


if __name__ == "__main__":
    main()
