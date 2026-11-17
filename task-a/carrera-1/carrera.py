"""
Tarea A (carrera-1): ir de A a B esquivando obstaculos, reconocer a la persona de la foto de B y
volver a A anunciando su nombre, en el menor tiempo posible. Todo corre en la PC; el robot recibe
move(v, w) por UDP y conserva su freno de emergencia por LiDAR.

Uso (desde la raiz del repositorio, con el entorno virtual activado):
  python task-a/carrera-1/carrera.py --sim                      # simulador, mundo "demo"; g = empezar
  python task-a/carrera-1/carrera.py --sim --mundo aleatorio:5 --go --no-gui
  python task-a/carrera-1/carrera.py --undock                   # ROBOT REAL: arranca EN PAUSA; g = empezar
  python task-a/carrera-1/carrera.py --go                       # robot real, empieza sin esperar la tecla

  python task-a/carrera-1/carrera.py --sim --mundo croquis --go --video   # + video del recorrido (MP4)

Reglas del laboratorio: en la meta hay varias fotos (~6), cada una con un QR en una esquina de abajo. Se
le dice al robot a quien buscar; lee el QR de esa foto, vuelve a A y dice la palabra:
  python task-a/carrera-1/carrera.py --undock --buscar "Nombre Apellido"          # ROBOT REAL
  python task-a/carrera-1/carrera.py --sim --mundo croquis+6 --go --video          # simulador: 6 fotos con QR

Teclas (ventana enfocada): g empezar/reanudar · espacio PAUSA (parada inmediata) · q / Esc salir
Cada corrida queda en task-a/carrera-1/corridas/ (que paso y cuando, para el informe). Con --video
tambien el video del recorrido en corridas/videos/ (camara, mapa, vista real y estado), con el robot
real o en el simulador.
"""
import argparse
import json
import math
import sys
import time
import unicodedata
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for p in (HERE, HERE.parent / "auto-1", HERE.parent / "slam-1", ROOT):
    sys.path.insert(0, str(p))

from coverage import Coverage                                               # noqa: E402  (auto-1)
from explorer import Explorer                                               # noqa: E402  (auto-1)
from flechas import DetectorFlechas                                         # noqa: E402
from navegador import (ACERCAR, EXPLORAR, FILA, HECHO, IDA, LEER_QR, PAUSA, RECONOCER, VOLVER,   # noqa: E402
                       Carrera, Config)
from percepcion import CamaraHilo, Objetivo, Percepcion, Senales           # noqa: E402
from planner import Planner                                                 # noqa: E402  (auto-1)
from relocalizar import Relocalizador                                      # noqa: E402
from mirada import CoberturaFrontal, MiradaLateral                         # noqa: E402
from reconocimiento import Reconocedor, Votacion, peso_por_tamano          # noqa: E402
from reconocimiento.voz import anunciar                                     # noqa: E402
from slam_core import make_engine                                           # noqa: E402  (slam-1)
from video import Grabador, ruta_auto                                       # noqa: E402

MAP_M, MAP_PX = 20.0, 500
COLORES = {PAUSA: (0, 200, 255), IDA: (0, 255, 0), ACERCAR: (255, 0, 255), RECONOCER: (255, 0, 255),
           VOLVER: (255, 200, 0), HECHO: (0, 255, 0), EXPLORAR: (0, 200, 255), LEER_QR: (255, 255, 0),
           FILA: (255, 160, 0)}


def ascii(s):
    """cv2.putText no dibuja tildes."""
    return unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode()


def componer(base, ref, ahora):
    """Pose del mapa `base` (tomada cuando la odometria marcaba `ref`) + lo que se movio desde `ref`
    hasta `ahora` segun la odometria. Asi la pose esta al dia entre scans del LiDAR."""
    dx, dy = ahora.x - ref.x, ahora.y - ref.y
    c, s = math.cos(-ref.yaw), math.sin(-ref.yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    bx, by, bth = base
    th = bth + math.atan2(math.sin(ahora.yaw - ref.yaw), math.cos(ahora.yaw - ref.yaw))
    return (bx + math.cos(bth) * lx - math.sin(bth) * ly, by + math.sin(bth) * lx + math.cos(bth) * ly,
            math.atan2(math.sin(th), math.cos(th)))


def _plano(texto):
    t = unicodedata.normalize("NFKD", texto).encode("ascii", "ignore").decode()
    return " ".join(t.replace("_", " ").lower().split())


def nombre_en_galeria(nombre, nombres):
    """El nombre tal como esta en la galeria ('alvaro garcia' -> 'Alvaro García'); sin mayusculas, tildes ni
    guiones bajos. Si no esta (o es ambiguo), SystemExit con la lista: mejor saberlo antes de arrancar."""
    exactos = [n for n in nombres if _plano(n) == _plano(nombre)]
    if len(exactos) == 1:
        return exactos[0]
    parecidos = [n for n in nombres if _plano(nombre) in _plano(n)]
    if len(parecidos) == 1:
        return parecidos[0]
    raise SystemExit(f"[error] --buscar '{nombre}' no esta (o es ambiguo) en la galeria. Nombres:\n  "
                     + "\n  ".join(sorted(set(nombres))))


def armar(args, excluir=()):
    rec = Reconocedor(args.modelo, excluir=excluir)
    forma = not args.flechas_solo_negras                                 # las senales del lab pueden ser de color
    per = Percepcion(rec, DetectorFlechas(color=args.flechas_color, forma=forma), hfov=args.hfov, hfov_hd=args.hfov_hd,
                     max_caras=6 if args.buscar else 3)
    engine = make_engine(args.engine, size_m=MAP_M, pixels=MAP_PX)
    pl = Planner(MAP_M, MAP_PX, cell=2, robot_radius=0.17, margin=0.05)
    frontal = args.mirada_desde is not None or not args.cobertura_simple
    cov = CoberturaFrontal(pl, fov_deg=args.hfov, rng=1.5) if frontal else Coverage(pl, fov_deg=args.hfov, rng=1.5)
    exp = Explorer(pl, cov=cov, vmax=min(args.vel, 0.22))
    vot = Votacion(rec.nombres)
    buscar = nombre_en_galeria(args.buscar, rec.nombres) if args.buscar else ""
    if buscar and not args.sin_qr:
        from qr import lector
        print(f"[ok] busco a {buscar}; lector de QR: {lector()}"
              + ("" if lector() == "wechat" else " (con opencv-contrib-python se lee desde mas lejos)"))
    cfg = Config(vmax=args.vel, vmax_vuelta=min(0.31, args.vel + 0.02), temprano=args.temprano,
                 usar_flechas=not args.sin_flechas, buscar=buscar, leer_qr=not args.sin_qr)
    pl_fino = Planner(MAP_M, MAP_PX, cell=2, robot_radius=0.10, margin=0.0)
    nav = Carrera(cfg, pl, vot, Objetivo(vot), Senales(min_n=2), exp, planner_fino=pl_fino)
    if args.mirada_desde is not None:
        nav.mirada = MiradaLateral(cov, desde_s=args.mirada_desde)
    return rec, per, engine, nav, cov


# ---------------------------------------------------------------- ventana

def panel_camara(ultima, nav, lado=400):
    if ultima is None:
        img = np.zeros((lado, lado, 3), np.uint8)
        cv2.putText(img, "sin imagen", (20, lado // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img
    img, caras, flechas = ultima[:3]
    qrs = ultima[3] if len(ultima) > 3 else []
    s = lado / img.shape[1]
    out = cv2.resize(img, (lado, int(img.shape[0] * s)), interpolation=cv2.INTER_NEAREST)
    for o in flechas:
        x, y, w, h = (int(v * s) for v in o.caja)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 140, 255), 2)
        cv2.putText(out, o.dir, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
    for o in caras:
        x, y, w, h = (int(v * s) for v in o.caja)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        txt = f"{o.px:.0f}px"
        if o.res is not None:
            txt = f"{ascii(o.res.nombre)} {o.res.sim:.2f}"
        cv2.putText(out, txt, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    for o in qrs:
        x, y, w, h = (int(v * s) for v in o.caja)
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.putText(out, o.texto[:24], (x, min(out.shape[0] - 4, y + h + 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 0), 1, cv2.LINE_AA)
    if nav.cfg.buscar:
        lineas = [f"BUSCO: {ascii(nav.cfg.buscar)}"]
        lineas.append(f"QR: {ascii(nav.palabra)}" if nav.palabra else "QR: (sin leer)")
        cv2.rectangle(out, (0, 0), (out.shape[1], 40), (0, 0, 0), -1)
        for i, t in enumerate(lineas):
            cv2.putText(out, t, (6, 16 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255) if i == 1 and nav.palabra else (255, 255, 255), 1, cv2.LINE_AA)
        q = getattr(nav, "qr_vista", None)
        if q is not None and q.size:
            lado = 110
            qv = cv2.resize(q, (lado, lado), interpolation=cv2.INTER_NEAREST)
            y0 = out.shape[0] - lado - 4
            out[y0:y0 + lado, out.shape[1] - lado - 4:out.shape[1] - 4] = qv
            cv2.rectangle(out, (out.shape[1] - lado - 5, y0 - 1), (out.shape[1] - 3, y0 + lado), (0, 255, 255), 2)
    if nav.vot is not None and nav.vot.n:
        for i, (n, p) in enumerate(nav.vot.top(3)):
            cv2.rectangle(out, (6, out.shape[0] - 60 + 18 * i), (6 + int(150 * p), out.shape[0] - 46 + 18 * i), (255, 0, 255), -1)
            cv2.putText(out, f"{ascii(n)} {p:.2f}", (160, out.shape[0] - 47 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return out


def panel_mapa(engine, nav, pos, lado=400, radio_m=5.0):
    mp = np.flipud(engine.map_image())
    n = mp.shape[0]
    res = engine.res
    cx, cy = pos[0], pos[1]
    if nav.trail:
        t = np.array(nav.trail)
        cx, cy = (t[:, 0].min() + t[:, 0].max()) / 2, (t[:, 1].min() + t[:, 1].max()) / 2
        radio_m = max(radio_m, (t.max(0) - t.min(0)).max() / 2 + 1.5)
    c0, r0 = int(n / 2 + (cx - radio_m) / res), int(n / 2 - (cy + radio_m) / res)
    k = int(2 * radio_m / res)
    c0, r0 = max(0, min(n - k, c0)), max(0, min(n - k, r0))
    crop = cv2.cvtColor(mp[r0:r0 + k, c0:c0 + k], cv2.COLOR_GRAY2BGR)
    s = lado / k
    img = cv2.resize(crop, (lado, lado), interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return int((n / 2 + x / res - c0) * s), int((n / 2 - y / res - r0) * s)

    for a, b in zip(nav.trail, nav.trail[1:]):
        cv2.line(img, px(*a), px(*b), (200, 120, 0), 1)
    for a, b in zip(nav.ruta, nav.ruta[1:]):
        cv2.line(img, px(*a), px(*b), (0, 200, 0), 2)
    if nav.sen is not None:
        for sg in nav.sen.lista:
            col = (150, 150, 150) if sg.usada else ((0, 140, 255) if sg.confirmada(nav.sen.min_n) else (0, 220, 255))
            cv2.circle(img, px(*sg.xy), 6, col, 2)
            cv2.putText(img, sg.dir(), (px(*sg.xy)[0] + 7, px(*sg.xy)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
    cv2.drawMarker(img, px(*nav.home), (0, 160, 0), cv2.MARKER_DIAMOND, 14, 2)
    for c in nav.candidatas:                       # con --buscar: cada foto vista (verde = la buscada, gris = otra)
        if c.xy is None:
            continue
        col = {"objetivo": (0, 200, 0), "otra": (150, 150, 150)}.get(c.estado, (0, 220, 255))
        cv2.circle(img, px(*c.xy), 7, col, 2)
        cv2.putText(img, f"#{c.id}" + (" QR" if c.id in nav.qr_de else ""), (px(*c.xy)[0] - 30, px(*c.xy)[1] - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
    if nav.obj is not None and nav.obj.xy is not None:
        cv2.drawMarker(img, px(*nav.obj.xy), (255, 0, 255), cv2.MARKER_STAR, 16, 2)
    x, y, th = pos
    p = px(x, y)
    cv2.circle(img, p, max(3, int(0.17 / res * s)), (0, 0, 255), 2)
    cv2.line(img, p, (int(p[0] + 18 * math.cos(th)), int(p[1] - 18 * math.sin(th))), (0, 0, 255), 2)
    if nav.libres is not None and nav.phi is not None and nav.estado not in (PAUSA, HECHO, RECONOCER):
        a = th + nav.phi
        cv2.line(img, p, (int(p[0] + 35 * math.cos(a)), int(p[1] - 35 * math.sin(a))), (0, 255, 255), 1)
    return img


def ventana(engine, nav, pos, ultima, robot, sim, t_rel, camtxt):
    cam = panel_camara(ultima, nav)
    mapa = panel_mapa(engine, nav, pos)
    paneles = [cam, mapa]
    if sim:
        v = robot.vista(px_m=60)
        s = 400 / max(v.shape[:2])
        v = cv2.resize(v, (int(v.shape[1] * s), int(v.shape[0] * s)))
        pad = np.full((400, 400, 3), 245, np.uint8)
        pad[:v.shape[0], :v.shape[1]] = v
        paneles.append(pad)
    top = np.hstack([cv2.copyMakeBorder(p, 0, 400 - p.shape[0], 0, 0, cv2.BORDER_CONSTANT) if p.shape[0] < 400 else p[:400]
                     for p in paneles])
    bar = np.full((70, top.shape[1], 3), 30, np.uint8)
    est = f"{nav.estado}  t={t_rel:5.1f}s"
    if nav.cfg.buscar and not nav.nombre:
        est += f"   BUSCO: {ascii(nav.cfg.buscar)}"
    if nav.nombre:
        est += f"   PERSONA: {ascii(nav.nombre)} ({nav.prob:.2f})"
    if nav.palabra:
        est += f"   QR: {ascii(nav.palabra)}"
    cv2.putText(bar, est, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORES.get(nav.estado, (255, 255, 255)), 2, cv2.LINE_AA)
    cv2.putText(bar, ascii(nav.log[-1]) if nav.log else "g = empezar, espacio = pausa, q = salir", (8, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(bar, camtxt, (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1, cv2.LINE_AA)
    out = np.vstack([top, bar])
    final = anuncio(nav)
    if nav.estado == HECHO and final:
        txt = ("LA PALABRA ES: " + ascii(nav.palabra)) if nav.palabra else ascii(nav.nombre).upper()
        ancho = max(400, 22 * len(txt) + 20)
        cv2.rectangle(out, (0, 0), (min(ancho, out.shape[1]), 56), (0, 120, 0), -1)
        cv2.putText(out, txt, (10, 40), cv2.FONT_HERSHEY_DUPLEX, 1.1 if len(txt) < 18 else 0.8, (255, 255, 255), 2,
                    cv2.LINE_AA)
    return out


def anuncio(nav):
    """Lo que se dice al llegar a A: la palabra del QR (reglas del laboratorio) o, sin QR, el nombre."""
    if nav.palabra:
        return f"La palabra es {nav.palabra}"
    if nav.nombre:
        return f"La persona es {nav.nombre}"
    return ""


def cuadro_video(vista, robot, sim, v, w, choques_previos=0):
    """La ventana + lo que importa al revisar el video: choques (y un borde rojo mientras choca), el freno
    del puente y la velocidad pedida."""
    out = vista.copy()
    h, W = out.shape[:2]
    txt = f"v={v:+.2f} m/s  w={w:+.2f} rad/s"
    if sim:
        choques = robot.collisions - choques_previos
        frena = bool(robot.flags & 1)                                   # FLAG_FRONT_BLOCKED del puente
        col = (0, 0, 255) if choques else (0, 200, 0)
        cv2.putText(out, f"CHOQUES {choques}", (W - 230, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
        if frena:
            cv2.putText(out, "FRENO", (W - 330, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2, cv2.LINE_AA)
        if robot._en_choque:
            cv2.rectangle(out, (0, 0), (W - 1, h - 1), (0, 0, 255), 8)
    cv2.putText(out, txt, (W - 330, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------- bucle

def correr(robot, args, sim=False, gui=True, mundo=None, max_t=300.0, dt=0.05):
    """Una corrida completa. `robot` = TB4Client o SimRobot. Devuelve un resumen (dict)."""
    if args.cam_periodo is None:
        args.cam_periodo = 0.3 if args.hd else 0.15                  # renderizar HD cuesta mas
    excluir = ()
    if sim and mundo is not None:
        excluir = tuple(mundo.fotos_excluir) or ((mundo.foto_b,) if mundo.foto_b else ())
        if mundo.rostros and not args.buscar:
            args.buscar = mundo.persona                                  # simulador: buscar a la del mundo
    rec, per, engine, nav, cov = armar(args, excluir)
    if args.traza:
        nav.traza = []                                                   # (t, estado, x, y, yaw, h_pref, phi, v, w)
    reloj = (lambda: robot.time) if sim else time.monotonic
    sens = dict(pos=(0.0, 0.0, 0.0), scan=None, th_scan=0.0, w=0.0)       # lo ultimo, tambien para el hilo de la camara
    cam = None if sim else CamaraHilo(robot, per, lambda: (sens["pos"], sens["scan"], sens["th_scan"], sens["w"]),
                                      hd=args.hd)
    pos_scan, odo_scan = (0.0, 0.0, 0.0), None
    reloc = None if getattr(args, "sin_relocalizar", False) else Relocalizador()
    pendientes, t_cam, t_scan = [], 0.0, 0.0
    fotos, ultima, last_rx = 0, None, None
    anunciado, t_hecho = False, None
    t0 = reloj()
    vista = None
    grab, v, w, pos = None, 0.0, 0.0, (0.0, 0.0, 0.0)
    if getattr(args, "video", None):
        destino = args.video
        if destino == "auto":
            destino = ruta_auto(HERE / "corridas" / "videos", (mundo.nombre if (sim and mundo) else "robot"))
        grab = Grabador(destino, fps=args.video_fps, bloquear=sim, h264=not args.video_sin_h264)
    if args.go:
        nav.empezar(t0, 0.0)
    t_go_archivo = 0.0
    try:
        while True:
            now = reloj()
            # ---- sensores ----
            if sim:
                if now >= t_scan:
                    sens["scan"], t_scan = robot.scan(), now + 1 / 7.0       # el LiDAR del TB4 gira a ~7 Hz
                scan = sens["scan"]
            else:
                scan = robot.scan()
                sens["scan"] = scan
            pose = robot.pose()
            if (scan is not None and pose is not None and scan.received != last_rx
                    and (sim or (scan.age < 0.5 and pose.age < 0.5))):
                engine.update(scan, pose)
                last_rx = scan.received
                pos_scan, odo_scan = engine.position(), pose
                # ---- volver EXACTO a A: scan de referencia en A y alineacion al volver (relocalizar.py) ----
                if reloc is not None and nav.t_inicio is not None:
                    if reloc.ref is None and math.hypot(pos_scan[0], pos_scan[1]) < 0.05:
                        reloc.referencia(scan, pos_scan)
                    elif nav.estado == VOLVER:
                        h = reloc.corregir(scan, pos_scan, nav.home)
                        if h is not None:
                            dh = math.hypot(h[0] - nav.home[0], h[1] - nav.home[1])
                            if reloc.n_ok == 1 and dh > 0.05:
                                nav.nota(now, f"LiDAR: A esta {dh:.2f} m de donde dice la odometria; corrijo")
                            k = 1.0 if reloc.n_ok == 1 else 0.5
                            nav.home = (nav.home[0] + k * (h[0] - nav.home[0]), nav.home[1] + k * (h[1] - nav.home[1]))
            # pose actual: la del mapa en el ultimo scan + lo que la odometria movio desde entonces
            pos = componer(pos_scan, odo_scan, pose) if (odo_scan is not None and pose is not None) else pos_scan
            sens.update(pos=pos, th_scan=pos_scan[2], w=pose.w if pose is not None else 0.0)
            # ---- camara ----
            buscar = nav.estado not in (PAUSA, VOLVER, HECHO)
            ver = nav.estado == VOLVER                                   # la vuelta: fotos solo para la ventana/video
            leer_qr = nav.quiere_qr                                      # frente a la meta: buscar QR (y en HD)
            if sim:
                if (buscar or ver) and now >= t_cam:
                    hd = args.hd or (leer_qr and not args.qr_sin_hd)
                    img = robot.photo(hd=hd)
                    if ver:
                        caras, flechas, qrs = [], [], []
                    else:
                        caras, flechas, qrs = per.procesar(img, pos, scan, now, sens["w"], pos_scan[2], leer_qr=leer_qr)
                    pendientes.append((now + args.cam_latencia, now, img, caras, flechas, qrs))
                    t_cam = now + (0.3 if ver else (max(args.cam_periodo, 0.3) if hd else args.cam_periodo))
                listos = [p[1:] for p in pendientes if p[0] <= now]
                pendientes = [p for p in pendientes if p[0] > now]
                cam_ok = robot.camara_ok
            else:
                cam.activa = buscar or ver
                cam.solo_ver = ver
                cam.qr, cam.hd_qr = leer_qr, not args.qr_sin_hd
                listos = cam.recoger()
                cam_ok = cam.t_ok is not None and now - cam.t_ok < 6.0
            for t_foto, img, caras, flechas, qrs in listos:
                fotos += 1
                ultima = (img, caras, flechas, qrs)
                for o in caras:
                    nav.ver_cara(now, o, peso_por_tamano(o.px))
                for o in flechas:
                    nav.ver_flecha(now, o)
                for o in qrs:
                    nav.ver_qr(now, o)
                if nav.palabra and nav.qr_vista is None:                 # la foto del QR leido, para la ventana
                    for o in qrs:
                        if o.texto == nav.palabra:
                            x, y, w_, h_ = o.caja
                            m = int(0.25 * max(w_, h_))
                            nav.qr_vista = img[max(0, y - m):y + h_ + m, max(0, x - m):x + w_ + m].copy()
                if nav.estado in (IDA, EXPLORAR) and scan is not None:
                    if isinstance(cov, CoberturaFrontal):
                        # lo que la camara mira DE FRENTE durante la ida y el plan B: asi el plan B sabe que
                        # paredes falta mirar (una pared vista de refilon no cuenta: ahi no se ve una cara)
                        cov.mark(pos, scan, img=engine.map_image())
                    elif nav.estado == EXPLORAR:
                        cov.mark(pos, scan)
            # ---- control ----
            mapa = engine.map_image() if nav.quiere_mapa else None
            v, w = nav.paso(now, pos, scan, mapa, fotos, cam_ok, pos_scan=pos_scan)
            robot.move(v, w)
            if sim:
                robot.advance(dt)
            if nav.estado == HECHO and not anunciado:
                anunciado, t_hecho = True, now
                texto = anuncio(nav)
                print(f"\n==== {texto} ====\n", flush=True)
                if not args.mudo:                                        # la voz no lee bien "_" ni "-"
                    anunciar(texto.replace("_", " ").replace("-", " "), imprimir=False)
            # ---- ventana (y video) ----
            grabar = grab is not None and grab.toca(now)
            if gui or grabar or (args.png and nav.estado == HECHO):
                camtxt = (f"fotos {fotos}" + ("" if sim else f", errores {cam.errores} {cam.ultimo_error}")
                          + f"  | flechas vistas {len(nav.sen.lista)}  | sitios {nav.vis.n()}")
                t_rel = now - (nav.t_inicio if nav.t_inicio is not None else now)
                vista = ventana(engine, nav, pos, ultima, robot, sim, t_rel, camtxt)
            if grabar:
                grab.agregar(cuadro_video(vista, robot, sim, v, w), now)
            if gui:
                cv2.imshow("carrera-1 (Tarea A)", vista)
                k = cv2.waitKey(1 if sim else 20) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord(" "):
                    nav.pausar()
                    robot.stop()
                elif k == ord("g"):
                    nav.empezar(now, pos[2])
            elif not sim:
                time.sleep(0.02)
            if args.go_archivo and nav.estado == PAUSA and now >= t_go_archivo:
                t_go_archivo = now + 0.5                                 # sin ventana (robot): la salida la da un archivo
                ga = Path(args.go_archivo)
                if ga.exists():
                    ga.unlink()
                    nav.empezar(now, pos[2])
                    print("[go] salida dada por", ga, flush=True)
            if t_hecho is not None and (not gui or sim) and now - t_hecho > 1.0:
                break
            if nav.t_inicio is not None and now - nav.t_inicio > max_t:
                nav.nota(now, "tiempo maximo: me rindo")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if cam is not None:
            cam.close()
        robot.stop()
        if gui and not sim:
            cv2.destroyAllWindows()
        if args.png:
            if vista is None or nav.estado != HECHO:
                camtxt = f"fotos {fotos}"
                vista = ventana(engine, nav, engine.position(), ultima, robot, sim, 0.0, camtxt)
            cv2.imwrite(str(args.png), vista)
        ruta_video = None
        if grab is not None:
            try:
                if nav.estado == HECHO or vista is None:                       # el final con el nombre en grande
                    t_rel = (nav.t_fin or reloj()) - (nav.t_inicio or t0)
                    vista = ventana(engine, nav, pos, ultima, robot, sim, t_rel, f"fotos {fotos}")
                grab.agregar(cuadro_video(vista, robot, sim, 0.0, 0.0), reloj())
                grab.sostener(2.0)
                ruta_video = grab.cerrar()
            except Exception as e:                                             # el video nunca tumba la corrida
                print(f"[video] {type(e).__name__}: {e}", file=sys.stderr)
    t_ini = nav.t_inicio if nav.t_inicio is not None else t0
    res = dict(estado=nav.estado, nombre=nav.nombre, prob=round(nav.prob, 3), buscar=nav.cfg.buscar or None,
               palabra=nav.palabra, qrs_leidos=len(nav.qr_de), fotos_meta=len(nav.candidatas),
               tiempo=round((nav.t_fin or reloj()) - t_ini, 2),
               tiempo_b=round(nav.t_b - t_ini, 2) if nav.t_b else None, fotos=fotos,
               flechas=[(s.dir(), s.n, s.usada) for s in nav.sen.lista], log=nav.log)
    if grab is not None:
        res["video"] = str(ruta_video) if ruta_video else None
    if nav.traza is not None:
        res["traza"] = [[c if (c is None or isinstance(c, str)) else round(float(c), 3) for c in fila]
                        for fila in nav.traza[::4]]
        res["caras_vistas"] = nav.obj.n
        res["b_estimado"] = None if nav.obj.xy is None else [round(float(v), 2) for v in nav.obj.xy]
    if sim:
        x, y, _ = robot.verdad()
        res.update(mundo=mundo.nombre if mundo else "", persona=mundo.persona if mundo else "",
                   acierto=(nav.nombre == mundo.persona) if mundo else None, choques=robot.collisions,
                   en_a=math.hypot(x - mundo.inicio[0], y - mundo.inicio[1]) < 0.35 if mundo else None)
        if mundo is not None and mundo.qr:
            res["qr_ok"] = nav.palabra == mundo.qr                       # reglas del laboratorio: la palabra del QR
        res["exito"] = bool(res["estado"] == HECHO and res["acierto"] and res["en_a"] and res.get("qr_ok", True))
    return res


def guardar(res, carpeta=HERE / "corridas"):
    carpeta.mkdir(exist_ok=True)
    f = carpeta / (time.strftime("%Y%m%d_%H%M%S") + ("_" + res.get("mundo", "robot").replace(":", "-")) + ".json")
    f.write_text(json.dumps(res, ensure_ascii=False, indent=1))
    return f


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.104",
                    help="IP del robot (defecto 192.168.0.104; 127.0.0.1 si corre EN el robot)")
    ap.add_argument("--sim", action="store_true", help="simulador en vez del robot real")
    ap.add_argument("--mundo", default="demo",
                    help="(--sim) demo | demo-sin-flechas | sala | aleatorio:N[:sin] | croquis[:W:H] | denso:N[:W:H]; "
                         "+K al final de croquis/denso: K fotos con QR en la meta (p. ej. croquis+6)")
    ap.add_argument("--buscar", default=None, metavar="NOMBRE",
                    help="persona a buscar entre las fotos de la meta (como en datos/rostros, sin importar tildes); "
                         "lee el QR de su foto y anuncia la palabra. En el simulador con +K: la del mundo")
    ap.add_argument("--sin-qr", action="store_true", help="con --buscar: no leer el QR (anuncia el nombre)")
    ap.add_argument("--qr-sin-hd", action="store_true",
                    help="no pedir fotos HD para leer el QR (con HD se lee desde ~1.4 m; sin HD, desde ~0.5 m)")
    ap.add_argument("--persona", default=None, help="(--sim) quien esta en la foto de B (defecto: al azar)")
    ap.add_argument("--no-gui", action="store_true", help="sin ventana (con el robot real solo se para con Ctrl+C)")
    ap.add_argument("--go", action="store_true", help="empezar sin esperar la tecla g")
    ap.add_argument("--go-archivo", default=None, metavar="RUTA",
                    help="sin ventana (en el robot): empezar cuando aparezca este archivo (robot/desplegar_carrera.sh --go)")
    ap.add_argument("--undock", action="store_true", help="sacar del dock al empezar (retrocede solo)")
    ap.add_argument("--vel", type=float, default=0.30, help="velocidad maxima en m/s (defecto 0.30; el robot llega a 0.31)")
    ap.add_argument("--temprano", action="store_true", help="volver apenas se sabe quien es (sin terminar de acercarse a B)")
    ap.add_argument("--sin-flechas", action="store_true", help="ignorar las flechas")
    ap.add_argument("--flechas-color", action="store_true", help="(con --flechas-solo-negras) aceptar tambien tinta de color")
    ap.add_argument("--flechas-solo-negras", action="store_true",
                    help="flechas solo de tinta oscura (por defecto cuenta la FORMA, de cualquier color)")
    ap.add_argument("--modelo", default="auto", help="reconocimiento: auto (r50 si esta instalado, si no mbf) | mbf | r50")
    # OAK-D Lite (TurtleBot 4 Lite): color IMX214, HFOV 69 grados (hoja de datos de Luxonis). La HD 1280x720 conserva
    # ese ancho; la preview 250x250 es el recorte cuadrado central de la 16:9: 2*atan(tan(34.5)*720/1280) = 42 grados.
    # Son valores de catalogo: medirlos en el laboratorio (probar_vision.py frente a una regla a 1 m).
    ap.add_argument("--hfov", type=float, default=42.0, help="campo de vision horizontal de la preview 250x250, en grados")
    ap.add_argument("--hd", action="store_true",
                    help="fotos HD 1280x720: ve la cara y las flechas ~3 veces mas lejos (camara con camara_rgb.sh --hd)")
    ap.add_argument("--hfov-hd", type=float, default=69.0, help="campo de vision horizontal de la foto HD, en grados")
    ap.add_argument("--engine", choices=("auto", "breezy", "odom"), default="odom", help="mapa: odometria (defecto) o BreezySLAM")
    ap.add_argument("--max-t", type=float, default=180.0, help="s maximos de la corrida (el limite del profe: 3 min)")
    ap.add_argument("--cam-periodo", type=float, default=None, help="(--sim) s entre fotos (defecto 0.15, con --hd 0.3)")
    ap.add_argument("--cam-latencia", type=float, default=0.15, help="(--sim) s de retraso de cada foto")
    ap.add_argument("--camara-caida", action="store_true", help="(--sim) la camara no da imagenes")
    ap.add_argument("--mudo", action="store_true", help="no anunciar en voz alta (solo pantalla)")
    ap.add_argument("--no-guardar", action="store_true", help="no guardar el registro de la corrida")
    ap.add_argument("--png", default=None, help="guardar la ventana final en esta imagen")
    ap.add_argument("--traza", action="store_true", help="guardar en el registro la traza del navegador (depurar)")
    ap.add_argument("--video", nargs="?", const="auto", default=None,
                    help="grabar el video del recorrido (MP4). Sin ruta: corridas/videos/<fecha>_<mundo>.mp4")
    ap.add_argument("--video-fps", type=float, default=10.0, help="cuadros por segundo del video (defecto 10)")
    ap.add_argument("--video-sin-h264", action="store_true", help="no convertir a H.264 con ffmpeg (queda MPEG-4)")
    ap.add_argument("--odom-sesgo", type=float, default=0.0,
                    help="(--sim) error sistematico de la odometria, p. ej. 0.03 = las ruedas miden 3%% de mas")
    ap.add_argument("--odom-ruido", type=float, default=0.01, help="(--sim) ruido de la odometria (defecto 0.01)")
    ap.add_argument("--cobertura-simple", action="store_true",
                    help="plan B con la cobertura de auto-1 (cuenta como vista una pared mirada de refilon)")
    ap.add_argument("--mirada-desde", type=float, default=None,
                    help="mirar paredes laterales cercanas durante la ida a partir de estos s de carrera")
    ap.add_argument("--sin-relocalizar", action="store_true",
                    help="no corregir A con el LiDAR al volver (solo odometria, como antes)")
    return ap.parse_args(argv)


def crear_sim(args):
    """Mundo y robot simulado segun las opciones (--mundo, --persona, --hd, --odom-*)."""
    from mundos import ROSTROS, construir
    from simulador import SimRobot
    persona = args.persona
    if persona is None and args.buscar and "+" in args.mundo:     # varias fotos: que la buscada este en la meta
        carpetas = [d.name.replace("_", " ") for d in ROSTROS.iterdir() if d.is_dir()]
        persona = nombre_en_galeria(args.buscar, carpetas)
    mundo = construir(args.mundo, persona)
    hd = args.hd or bool(mundo.qr and not args.qr_sin_hd)                # con QR: la camara HD existe
    robot = SimRobot(mundo, hfov=args.hfov, camara_ok=not args.camara_caida,
                     foto_hd=(1280, 720, args.hfov_hd) if hd else None,
                     odom_ruido=args.odom_ruido, odom_sesgo=args.odom_sesgo)
    return mundo, robot


def main(argv=None):
    args = parse(argv)
    args.vel = min(args.vel, 0.31)
    if args.sim:
        mundo, robot = crear_sim(args)
        if mundo.rostros:
            print(f"[sim] mundo {mundo.nombre}: {len(mundo.rostros)} fotos con QR en la meta; la buscada es "
                  f"{mundo.persona} (QR '{mundo.qr}'); las fotos de la meta no estan en la galeria")
        else:
            print(f"[sim] mundo {mundo.nombre}: en B esta {mundo.persona} ({mundo.foto_b}, excluida de la galeria)")
        res = correr(robot, args, sim=True, gui=not args.no_gui, mundo=mundo, max_t=args.max_t)
    else:
        from tb4 import TB4Client
        with TB4Client(args.ip) as robot:
            robot.wait_ready(8)
            if args.undock:
                print("[..] sacando del dock (retrocede solo: deja espacio detras) ...")
                print("[ok]", robot.undock())
            print("[ok] listo. Ventana enfocada: g = empezar, espacio = pausa, q = salir.")
            res = correr(robot, args, sim=False, gui=not args.no_gui, max_t=args.max_t)
    if not args.no_guardar:
        print("[ok] registro:", guardar(res))
    if res.get("video"):
        print("[ok] video:", res["video"])
    for linea in res.pop("log"):
        print("   ", linea)
    print("[fin]", res)
    return res


if __name__ == "__main__":
    main()
