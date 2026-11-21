"""
Tarea B: conducir el TurtleBot 4 con gestos captados por la WEBCAM DE LA PC.

  señalar con el índice        -> SOLO GIRA hacia ahí y se queda quieto (relativo a su frente):
                                  der/izq 90°, diagonales arriba 45°, diagonales abajo 135°, abajo 180°
                                  (el índice hacia arriba no hace nada)
  ✌️  V (2 dedos arriba)         -> AVANZA lento
  ✊ puño                       -> AVANZA rápido
  ✋ mano abierta (cualquier dir.) -> STOP (se para y espera, también en pleno giro)
  sin mano                      -> sigue haciendo lo mismo

Mantener una señal no repite el giro; bajar la mano y volver a señalar, sí. Señalar mientras avanza lo
para y gira. Avanzar mientras gira: termina el giro y luego avanza.

Uso (desde la raíz del repositorio; en WSL usa el Python de Windows, python.exe, por la webcam):
  python task-b/gestos-1/navegar_gestos.py --sim               # SIMULADOR, sin robot (pruébalo primero)
  python task-b/gestos-1/navegar_gestos.py                     # ROBOT REAL en 192.168.0.103
  python task-b/gestos-1/navegar_gestos.py 192.168.0.103 --undock

Caras y QR: mientras conduces, la CÁMARA DEL ROBOT busca una cara, la reconoce (reconocimiento/) y lee su QR
(task-a/carrera-1/qr.py). Al decidir, anuncia el nombre y el QR. NO mueve el robot: en la Tarea B todo el
movimiento es manual, con gestos (también volver al inicio). --sin-caras: solo gestos.

Teclas: ESPACIO = parada de emergencia · n = buscar otra cara · q / Esc salir.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

from control import PARADO, Control                                              # noqa: E402
from gestos import NOMBRE, STOP, Estabilizador, clasificar                        # noqa: E402
from mano import abrir_camara, crear_detector, detectar, dibujar_mano, texto      # noqa: E402
from probar_gestos import color                                                   # noqa: E402

V_TOPE = 0.31     # m/s, tope de fábrica del TurtleBot 4 (el puente tampoco deja más)


def panel_odometria(lado, rastro, pose, ctrl, robot, escala=60.0):
    """Vista desde arriba centrada en el robot: rastro, rumbo actual (blanco) y objetivo (verde)."""
    img = np.full((lado, lado, 3), 40, np.uint8)
    c = lado // 2
    for k in range(-10, 11):                                   # cuadrícula de 1 m
        if pose is not None:
            gx = int(c + (k - (pose.x % 1.0)) * escala)
            gy = int(c - (k - (pose.y % 1.0)) * escala)
            cv2.line(img, (gx, 0), (gx, lado), (60, 60, 60), 1)
            cv2.line(img, (0, gy), (lado, gy), (60, 60, 60), 1)
    if pose is None:
        texto(img, "sin odometria", (10, 25), color=(0, 0, 255))
        return img

    def px(x, y):
        return int(c + (x - pose.x) * escala), int(c - (y - pose.y) * escala)

    if len(rastro) > 1:
        cv2.polylines(img, [np.array([px(x, y) for x, y in rastro], np.int32)], False, (0, 200, 255), 2)
    cv2.circle(img, (c, c), int(0.17 * escala), (200, 200, 200), 2)
    cv2.arrowedLine(img, (c, c), (int(c + 0.4 * escala * math.cos(pose.yaw)), int(c - 0.4 * escala * math.sin(pose.yaw))),
                    (255, 255, 255), 2, tipLength=0.3)
    if ctrl.rumbo is not None and ctrl.estado != PARADO:
        cv2.arrowedLine(img, (c, c), (int(c + 0.6 * escala * math.cos(ctrl.rumbo)), int(c - 0.6 * escala * math.sin(ctrl.rumbo))),
                        (0, 255, 0), 2, tipLength=0.25)
    st = robot.status()
    if st is not None:
        fc = "libre" if math.isinf(st.front_clear) else f"{st.front_clear:.2f} m"
        texto(img, f"frente: {fc}", (10, lado - 40), color=(0, 0, 255) if st.front_blocked else (255, 255, 255))
        if st.scan_stale:
            texto(img, "SIN LIDAR: no avanza", (10, lado - 65), color=(0, 0, 255))
        if st.battery >= 0:
            texto(img, f"bateria {st.battery * 100:.0f}%", (10, lado - 15))
    return img


def cartel(vista, persona, qr):
    """Banda abajo a la derecha (sobre el panel): persona reconocida y, cuando se lee, el QR."""
    h, w = vista.shape[:2]
    x0, alto = w // 2, 70
    cv2.rectangle(vista, (x0, h - alto), (w, h), (0, 0, 0), -1)
    texto(vista, f"Persona: {persona} reconocida", (x0 + 8, h - 42), 0.6, (0, 255, 0))
    texto(vista, f"QR reconocido: {qr}" if qr else "QR: buscando...", (x0 + 8, h - 14), 0.6,
          (0, 255, 0) if qr else (0, 200, 255))


def miniatura(vista, vigia, ancho=200):
    """La última foto de la cámara del robot, con las caras, en la esquina superior derecha del panel."""
    if vigia is None or vigia.ultima is None:
        return
    img, cajas, nombre = vigia.ultima
    img = img.copy()
    for x, y, w, h in cajas:
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
    alto = int(img.shape[0] * ancho / img.shape[1])
    if alto >= vista.shape[0] or ancho >= vista.shape[1]:
        return
    vista[0:alto, -ancho:] = cv2.resize(img, (ancho, alto))
    if nombre:
        texto(vista, nombre, (vista.shape[1] - ancho + 5, alto - 8), 0.5, (0, 255, 0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--sim", action="store_true", help="usar el simulador de task-a en vez del robot")
    ap.add_argument("--undock", action="store_true", help="sacar el robot del dock primero (retrocede solo)")
    ap.add_argument("--cam", type=int, default=0, help="índice de la webcam (defecto 0)")
    ap.add_argument("--no-espejo", action="store_true", help="no voltear la imagen (por defecto vista espejo)")
    ap.add_argument("--lento", type=float, default=0.15, help="m/s con V (defecto 0.15)")
    ap.add_argument("--rapido", type=float, default=0.31, help="m/s con puño (defecto 0.31, el máximo)")
    ap.add_argument("--buscar", metavar="NOMBRE", help="persona a buscar, p. ej. Chiara_Lazaro (o directamente --Chiara_Lazaro)")
    ap.add_argument("--sin-caras", action="store_true", help="no buscar caras ni QR (solo gestos)")
    ap.add_argument("--hd", action="store_true", help="fotos HD de la cámara del robot (ve caras más lejos, más lento)")
    ap.add_argument("--margen", type=float, default=20.0, help="grados de margen de cada dirección al señalar (defecto 20, máx. 22.5)")
    args, resto = ap.parse_known_args()
    for x in resto:                                     # --Nombre_Apellido = --buscar Nombre_Apellido
        if x.startswith("--") and len(x) > 2 and not args.buscar:
            args.buscar = x[2:]
        else:
            ap.error(f"argumento desconocido: {x}")
    if max(args.lento, args.rapido) > V_TOPE:
        sys.exit(f"Velocidad máxima permitida: {V_TOPE} m/s")

    if args.sim:
        sys.path.insert(0, str(ROOT / "task-a" / "auto-1"))
        from sim import SimRobot
        robot = SimRobot()
    else:
        from tb4 import TB4Client
        robot = TB4Client(args.ip)
        print(f"conectando con {args.ip} ...")
        robot.wait_ready()

    vigia = None
    if not args.sin_caras:
        from vigia import Vigia
        from reconocimiento.voz import anunciar
        print("cargando reconocimiento facial ...")
        try:
            vigia = Vigia(robot, hd=args.hd, buscado=args.buscar)
        except ValueError as e:
            robot.close()
            sys.exit(str(e))
        if vigia.buscado:
            print(f"buscando a: {vigia.buscado}")
        vigia.start()

    if args.undock and not args.sim:
        print("undock:", robot.undock())

    cap = abrir_camara(args.cam)
    anunciado_persona = anunciado_qr = None

    def decir(msg):
        anunciar(msg)
        if not args.sim:
            try:
                robot.decir(msg)
            except Exception:
                pass
    estab = Estabilizador()
    ctrl = Control(v_lento=args.lento, v_rapido=args.rapido)
    rastro = []
    ultimo_txt = None
    t0 = t_prev = time.monotonic()
    print("listo: el robot está PARADO. Señalar = girar, V = avanzar lento, puño = rápido, mano abierta = STOP.")
    try:
        with crear_detector(video=True) as det:
            while True:
                ok, frame = cap.read()
                if not ok:
                    print("la cámara no entrega imagen: paro el robot")
                    break
                if not args.no_espejo:
                    frame = cv2.flip(frame, 1)
                t = time.monotonic()
                dt, t_prev = t - t_prev, t

                pts = detectar(det, frame, int((t - t0) * 1000))
                crudo, info = clasificar(pts, args.margen)
                gesto = estab.update(crudo)

                pose = robot.pose()
                if not args.sim and pose is not None and pose.age > 0.5:
                    pose = None                                 # odometría vieja: no fiarse
                if vigia is not None:
                    quien, qr = vigia.persona(), vigia.qr()
                    if quien and anunciado_persona != quien:
                        anunciado_persona = quien
                        decir(f"Persona reconocida: {quien}")
                    if qr and anunciado_qr != qr:
                        anunciado_qr = qr
                        decir(f"Codigo QR reconocido: {qr}")

                v, w = ctrl.paso(gesto, pose.yaw if pose else None)
                v = min(v, V_TOPE)
                robot.move(v, w) if (v or w) else robot.stop()
                if args.sim:
                    robot.advance(min(dt, 0.1))

                if pose is not None and (not rastro or math.hypot(pose.x - rastro[-1][0], pose.y - rastro[-1][1]) > 0.02):
                    rastro.append((pose.x, pose.y))
                    del rastro[:-2000]
                txt = ctrl.texto()
                if txt != ultimo_txt:
                    print(f"[{t - t0:6.1f}s] gesto {gesto:8s} -> {txt}")
                    ultimo_txt = txt

                h, w_img = frame.shape[:2]
                dibujar_mano(frame, pts)
                texto(frame, f"crudo: {crudo}  dedos: {info.get('estirados') or '-'}", (10, 25))
                texto(frame, NOMBRE[gesto], (10, h - 55), 1.2, color(gesto), 3)
                texto(frame, txt, (10, h - 20), 0.8, (0, 0, 255) if ctrl.estado == PARADO else (0, 255, 0), 2)
                if vigia is not None:
                    texto(frame, vigia.estado + (f"  [{vigia.error}]" if vigia.error else
                                         f"  [foto {vigia.demora * 1000:.0f} ms]" if vigia.demora else ""), (10, 50), 0.6, (255, 255, 0))
                if args.sim:
                    texto(frame, "SIMULADOR", (w_img - 140, 25), color=(0, 255, 255))
                vista = np.hstack([frame, panel_odometria(h, rastro, pose, ctrl, robot)])
                if anunciado_persona:
                    cartel(vista, anunciado_persona, anunciado_qr)
                miniatura(vista, vigia)
                cv2.imshow("Tarea B - gestos (ESPACIO = parada, n = otra cara, q = salir)", vista)

                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord("n") and vigia is not None:
                    vigia.reiniciar()
                    anunciado_persona = anunciado_qr = None
                    print("buscando otra cara")
                if k == ord(" "):
                    ctrl.parar()
                    ctrl.ultimo = gesto                         # si sigue señalando no arranca: hay que señalar de nuevo
                    robot.stop()
                    print("PARADA DE EMERGENCIA (tecla)")
    finally:
        if vigia is not None:
            vigia.detener()
        robot.stop()
        robot.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
