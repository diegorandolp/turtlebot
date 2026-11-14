"""
Laberinto con senales (Tarea A, laberinto-1): ida -> reconocer la cara -> leer el QR -> vuelta -> anunciar el mensaje.

  IDA        exploracion autonoma (SLAM + fronteras + cobertura de camara, de auto-1). La camara marca en el mapa las
             senales por COLOR: AZUL = camino de ida (el robot prefiere las fronteras de esa zona), ROJO = vuelta
             (se anota, no hace falta). Un callejon sin salida no se "atasca": el robot retrocede a la frontera libre
             detectada mas conveniente y sigue buscando.
  CARA       cada foto vista se apunta en el mapa; el robot va de frente a cada una y SFace dice quien es.
  QR         ante la persona buscada lee el QR de debajo (gira un poco / se acerca si no lo logra) y lo GUARDA.
  VUELTA     ruta mas corta por el mapa ya conocido hasta A (sin callejones).
  ANUNCIO    en A dice el mensaje (voz si hay espeak/spd-say, y siempre en pantalla y en resultado.json).

Uso (desde la raiz del repositorio, con el venv activado):
  python task-a/laberinto-1/laberinto.py --sim --mundo lab_desvios --buscar F3 --hd
  python task-a/laberinto-1/laberinto.py --undock --buscar "Nombre Apellido" --hd     # ROBOT REAL (arranca en pausa: g)
Resto de opciones: las de auto-1/explorar.py (--vel, --radio, --cam-fov, --umbral-reco ...).
Teclas: g empezar, espacio PAUSA, l LiDAR, m guardar mapa, q salir.
"""
import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
AUTO1 = HERE.parents[0] / "auto-1"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(AUTO1))
sys.path.insert(0, str(HERE.parents[0] / "slam-1"))
sys.path.insert(0, str(HERE.parents[1]))

import explorar as ex1                                           # noqa: E402  (draw, GuardaFoto, guardar_mapa ...)
from caras import RegistroCaras, estimar_cara                    # noqa: E402
from cobertura import Coverage                                   # noqa: E402
from explorer import (ACERCARSE, BARRIDO, BUSCANDO, CONFIRMAR, EXPLORAR, HECHO, INSPECCIONAR, PAUSA, SEGUIR_PARED,  # noqa: E402
                      SIN_B, VERIFICAR, VOLVER, Explorer, front_clear)
from face import FaceDetector, FaceWatcher, ThreadedWatcher       # noqa: E402
from planificador import PlannerLab                              # noqa: E402
from qr import FaseQR, leer_qr                                   # noqa: E402
from senales import RegistroSenales, detectar_senales            # noqa: E402
from slam_core import make_engine                                # noqa: E402

MAX_AGE = 0.5
ACTIVOS = BUSCANDO + (CONFIRMAR,)
COL = {"azul": (255, 90, 0), "rojo": (0, 0, 255)}


class Visor(ex1.GuardaFoto):
    """GuardaFoto de auto-1 (caras) + deteccion de senales de color en la MISMA foto."""

    def __init__(self, detect):
        super().__init__(detect)
        self.sen, self.nfotos, self.ancho, self.alto = [], 0, 250, 250

    def __call__(self, img):
        self.sen, self.ancho, self.alto = detectar_senales(img), img.shape[1], img.shape[0]
        self.nfotos += 1
        return super().__call__(img)


def senal_fiable(s, pos, scan, fov_deg, ancho, alto, tam_m=None, max_deg=30.0, tol=0.35):
    """Ubica una senal en el mapa o devuelve None si la deteccion no es de fiar:
      - la mancha toca el borde de la foto (recortada: tamano y centro erroneos)
      - esta a mas de `max_deg` del eje de la camara (el angulo se estima mal en los bordes)
      - con `tam_m` (lado real de la senal, m): la distancia por tamano en la imagen y la del LiDAR deben coincidir
        (si no, el LiDAR midio otra pared y la senal quedaria mal ubicada)."""
    x, y, w, h = s["caja"]
    if x <= 1 or y <= 1 or x + w >= ancho - 1 or y + h >= alto - 1:
        return None
    f = (ancho / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    if abs(math.degrees(math.atan2(ancho / 2.0 - (x + w / 2.0), f))) > max_deg:
        return None
    est = estimar_cara(s["caja"], pos, scan, fov_deg, ancho_img=ancho)
    if est is None:
        return None
    if tam_m:
        d_cam, d_lidar = f * tam_m / max(w, 1.0), math.hypot(est[0] - pos[0], est[1] - pos[1])
        if abs(d_lidar - d_cam) > tol * d_cam:
            return None
    return est


def anunciar(mensaje, carpeta=None):
    """Dice el mensaje: pantalla (siempre), voz si hay sintetizador en la PC, y archivo."""
    print("\n" + "=" * 64 + f"\n   MENSAJE:  {mensaje}\n" + "=" * 64)
    for cmd in ("spd-say", "espeak-ng", "espeak", "say"):
        if shutil.which(cmd):
            try:
                subprocess.run([cmd, mensaje], timeout=30, check=False)
            except Exception:
                pass
            break
    if carpeta:
        Path(carpeta, "mensaje.txt").write_text(mensaje + "\n")


def dibujar(img, engine, registro_s, qr_txt, fase):
    n = engine.n
    scale = max(1, img.shape[0] // n)
    for s in registro_s.items:
        c = COL[s["color"]]
        p = (int((n / 2 + s["x"] / engine.res) * scale), int((n - 1 - (n / 2 + s["y"] / engine.res)) * scale))
        ok = s["vistas"] >= registro_s.need
        cv2.circle(img, p, 7, c, -1 if ok else 1)
        if ok:
            cv2.putText(img, ("ida" if s["color"] == "azul" else "vuelta") + (f" #{s['orden']}" if s["orden"] else ""),
                        (p[0] + 9, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
    txt = f"fase: {fase}" + (f"   QR: {qr_txt}" if qr_txt else "")
    cv2.putText(img, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return img


def run(robot, args, sim=False, gui=True, detect=None, reconocer=None, max_time=1800.0, dt=0.15, carpeta=None):
    engine = make_engine(args.engine, size_m=args.map_m, pixels=args.pixels,
                         sigma_xy_mm=args.sigma_xy, sigma_theta_deg=args.sigma_th)
    planner = PlannerLab(args.map_m, args.pixels, cell=args.cell, robot_radius=args.radio, frontera=args.frontera,
                         peso_ida=args.peso_ida)
    cov = None if args.sin_cobertura else Coverage(planner, fov_deg=args.cam_fov, rng=args.cam_rango)
    ex = Explorer(planner, cov=cov, vmax=args.vel, volver=True, estrategia="fronteras")
    watcher = FaceWatcher()
    foto = (lambda: robot.photo(hd=True)) if args.hd else robot.photo
    tw = None if sim else ThreadedWatcher(watcher, foto, detect)
    if args.go:
        ex.start()
    registro, registro_s = RegistroCaras(), RegistroSenales(need=3)
    trail, last_rx, hz, t_prev, show_scan = [], None, 0.0, None, True
    det_n, nf_vistos, verif = 0, 0, None
    qr, qr_t, qr_txt, fase = None, -1e9, None, "IDA"
    t_start = robot.time if sim else time.monotonic()
    finished_at, anunciado = None, False
    mapa_png = None

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

            buscando = qr is None and qr_txt is None
            if tw is not None:
                tw.active = ex.state in ACTIVOS and qr is None
            if ex.state in ACTIVOS and qr is None and tw is None:
                watcher.update(now, foto, detect)

            # --- senales de color: apuntarlas en el mapa; la AZUL mas reciente orienta la busqueda ---
            if detect.nfotos != nf_vistos and scan is not None:
                nf_vistos = detect.nfotos
                for s in list(detect.sen):
                    est = senal_fiable(s, pos, scan, args.cam_fov, detect.ancho, detect.alto, args.senal_m)
                    if est is None:
                        continue
                    sg, nueva = registro_s.agregar(s["color"], *est)
                    if nueva:
                        ex._note(f"senal {s['color'].upper()} #{sg['orden']} en ({sg['x']:.2f},{sg['y']:.2f})")
                        if s["color"] == "azul":
                            planner.ida_anchor = (sg["x"], sg["y"])
                            ex.goal = None                      # re-elegir frontera ya con la nueva pista
                            ex._invalidate()

            # --- caras: apuntar, ir de frente, reconocer (igual que auto-1 con --buscar) ---
            if detect.n != det_n and scan is not None:
                det_n = detect.n
                for caja in list(detect.cajas):
                    ancho = detect.last.shape[1] if detect.last is not None else 250
                    est = estimar_cara(caja, pos, scan, args.cam_fov, ancho_img=ancho)
                    if est is not None:
                        c, nueva = registro.agregar(*est)
                        if nueva:
                            ex._note(f"cara vista en ({c['x']:.2f},{c['y']:.2f}) ({registro.pendientes()} pendientes)")
            if qr is None and qr_txt is None:
                if ex.state == VERIFICAR:
                    if verif is None:
                        verif = (now, watcher.photos, detect.n)
                    t0v, p0v, n0v = verif
                    c = registro.actual
                    if detect.n > n0v and watcher.photos - p0v >= 2:
                        nombre, simil = reconocer()
                        if nombre == args.buscar:
                            c["estado"], c["nombre"] = "objetivo", nombre
                            ex._note(f"es {nombre} ({simil:.2f}): leo el QR")
                            qr, fase = FaseQR(), "QR"
                            ex.b_xy = (pos[0], pos[1])
                        else:
                            c["estado"], c["nombre"] = "revisada", nombre
                            ex.face_otra_persona(nombre, simil)
                            ex.terminar_cara()
                        verif = None
                    elif now - t0v > 6.0:
                        c["estado"] = "revisada"
                        ex._note("desde el punto de vista no veo ninguna cara: la doy por revisada")
                        ex.terminar_cara()
                        verif = None
                else:
                    verif = None
                    if ex.state == ACERCARSE and ex.cara_fallida:
                        c = registro.actual
                        c["intentos"] = c.get("intentos", 0) + 1
                        if c["intentos"] >= 4:
                            c["estado"] = "inalcanzable"
                            ex._note("tras 4 intentos no llego a la cara: la descarto")
                        else:
                            c["espera"] = now + 20.0
                        ex.terminar_cara()
                    elif ex.state in (EXPLORAR, INSPECCIONAR, BARRIDO, SEGUIR_PARED):
                        c = registro.siguiente(pos, now)
                        if c is not None:
                            registro.actual = c
                            ex._note(f"voy a ver de cerca la cara en ({c['x']:.2f},{c['y']:.2f})")
                            ex.ir_a_cara(registro.punto_de_vista(c))

            if cov is not None and watcher.photos != getattr(run, "_lp", 0) and scan is not None:
                cov.mark(pos, scan)
                run._lp = watcher.photos
            lw = watcher.last_ok_t
            cam_ok = (now - lw < 6.0) if lw is not None else (watcher.errors == 0 or now - t_start < 10.0)

            v, w = ex.step(now, engine.map_image(), pos, scan, photos=watcher.photos, cam_ok=cam_ok)

            # --- fase QR: controla el robot hasta leer el codigo (o rendirse) ---
            if qr is not None:
                frame = None
                if now - qr_t > 0.4:
                    qr_t = now
                    try:
                        frame = robot.photo(hd=True)
                    except Exception:
                        frame = None
                v, w = qr.step(now, frame, pos, scan, front_clear(scan))
                if qr.terminada:
                    qr_txt = qr.texto if qr.ok else ""
                    ex._note(f"QR leido: {qr.texto}" if qr.ok else "no pude leer el QR")
                    qr, fase = None, "VUELTA"
                    ex.face_confirmed(pos, nombre=args.buscar)      # -> VOLVER
                    v, w = 0.0, 0.0
            elif ex.state == VOLVER:
                fase = "VUELTA"
            robot.move(v, w)
            if sim:
                robot.advance(dt)

            if gui:
                img = ex1.draw(engine, ex, trail, robot.status(), hz, scan, args.view, show_scan, watcher, registro)
                cv2.imshow("laberinto", dibujar(img, engine, registro_s, qr_txt, fase))
                if sim:
                    cv2.imshow("simulador: mundo real", robot.render_world())
                k = cv2.waitKey(1 if sim else 30) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k == ord(" "):
                    ex.pause()
                    robot.stop()
                elif k == ord("g") and ex.state == PAUSA:
                    ex.start()
                elif k == ord("l"):
                    show_scan = not show_scan
                elif k == ord("m"):
                    ex1.guardar_mapa(engine, args.mapas_dir, args.map_m, ultimo=False)
            elif not sim:
                time.sleep(0.05)

            if ex.state in (HECHO, SIN_B):
                finished_at = finished_at or now
                if now - finished_at > 1.0:
                    break
            if now - t_start > max_time:
                ex._note("tiempo maximo alcanzado")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if tw is not None:
            tw.close()
        robot.stop()
        mensaje = qr_txt if qr_txt else None
        de_vuelta = ex.state == HECHO and ex.b_xy is not None
        res = dict(estado=ex.state, persona=args.buscar if ex.b_xy else None, mensaje=mensaje, de_vuelta_en_A=de_vuelta,
                   pos=engine.position(), tiempo=(robot.time if sim else time.monotonic()) - t_start,
                   senales=[{k: (round(v, 2) if isinstance(v, float) else v) for k, v in s.items()}
                            for s in registro_s.confirmadas()], log=ex.log)
        if sim:
            res.update(colisiones=robot.collisions, pos_real=(robot.wx, robot.wy))
        carpeta = Path(carpeta or HERE / "corridas" / time.strftime("%Y%m%d_%H%M%S"))
        carpeta.mkdir(parents=True, exist_ok=True)
        (carpeta / "resultado.json").write_text(json.dumps(res, indent=2, ensure_ascii=False, default=str))
        img = dibujar(ex1.draw(engine, ex, trail, robot.status(), hz, None, args.view, False, None, registro),
                      engine, registro_s, qr_txt, ex.state)
        cv2.imwrite(str(carpeta / "mapa.png"), img)
        if not args.no_guardar:
            ex1.guardar_mapa(engine, args.mapas_dir, args.map_m, ultimo=False)
        if gui:
            cv2.destroyAllWindows()
        res["carpeta"] = str(carpeta)
    if res["mensaje"] and res["de_vuelta_en_A"]:
        anunciar(res["mensaje"], carpeta)
    elif res["persona"] and res["de_vuelta_en_A"]:
        print("\n[!] Volvi a A pero no pude leer el QR.")
    return res


def parse(argv=None):
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--senal-m", type=float, default=None,
                    help="lado real de la senal (m): descarta detecciones cuya distancia por tamano no cuadra con el LiDAR "
                         "(simulador: 0.18; robot real: mide tu senal, o no lo pongas)")
    ap.add_argument("--peso-ida", type=float, default=2.0, help="cuanto pesa la cercania a la ultima senal azul (defecto 2)")
    mio, resto = ap.parse_known_args(argv)
    if "-h" in resto or "--help" in resto:
        print(__doc__)
        sys.exit(0)
    a = ex1.parse(resto)
    a.peso_ida, a.senal_m = mio.peso_ida, mio.senal_m
    return a


def main():
    args = parse()
    args.vel = min(args.vel, 0.20)
    if not args.buscar:
        sys.exit("[error] falta --buscar NOMBRE (la persona de la foto; en el simulador F1..F4)")
    args.volver = True
    if args.mapas_dir is None:
        args.mapas_dir = str(HERE / ("mapas_sim" if args.sim else "mapas"))
    if args.sim:
        args.senal_m = args.senal_m or 0.18
        from sim_lab import SimDetectorLab, SimLab
        robot = SimLab(world=args.mundo, odom_noise=args.ruido, odom_bias=args.sesgo, cam_fov_deg=args.cam_fov,
                       cam_range=args.cam_rango, camera_ok=not args.sin_camara)
        detect = Visor(SimDetectorLab())
        reco = lambda: (f"F{robot.face_seen + 1}", 0.9)
        res = run(robot, args, sim=True, gui=not args.no_gui, detect=detect, reconocer=reco, max_time=args.max_time)
    else:
        from tb4 import TB4Client
        detect = Visor(FaceDetector(score=args.score, min_width=args.min_ancho))
        reco = ex1.reconocedor_real(args.galeria, args.buscar, args.umbral_reco, detect)
        with TB4Client(args.ip) as robot:
            robot.wait_ready(8)
            if args.undock:
                print("[..] sacando del dock (retrocede solo: deja espacio detras) ...")
                robot.undock()
            print("[ok] listo. Ventana enfocada: g = empezar, espacio = pausa, q = salir.")
            res = run(robot, args, sim=False, gui=True, detect=detect, reconocer=reco, max_time=args.max_time)
    print("[fin]", {k: v for k, v in res.items() if k != "log"})


if __name__ == "__main__":
    main()
