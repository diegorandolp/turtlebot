"""
Monitor del TurtleBot 4 por UDP (grupo 3): LiDAR visto desde arriba + estado + camara.
Python puro (numpy + opencv-python); no necesita ROS. Requiere el puente en el robot.

Uso (desde la raiz del repositorio):
  python3 -m herramientas.monitor                    # solo mirar (no toma el control)
  python3 -m herramientas.monitor --drive            # ademas conducir con el teclado
  python3 -m herramientas.monitor 192.168.0.103 --range 3

Vista: el robot es el cuadro blanco, el frente apunta hacia ARRIBA, anillos cada 1 m.
Las lineas delimitan el pasillo que vigila el freno de seguridad del puente; se ponen
rojas cuando el puente bloquea el avance o el retroceso.

Teclas (con la ventana enfocada):
  p      foto de la camara (otra ventana)          q / Esc  salir
  Solo con --drive:
  w/s    adelante / atras (mantener)    a/d  girar     espacio  parar
  +/-    velocidad lineal               u    sacar del dock     k  volver al dock
"""
import argparse
import math
import threading
import time

import cv2
import numpy as np

from tb4 import TB4Client
from tb4 import protocol as P

SIZE = 720
HOLD = 0.5            # s: la tecla se considera "mantenida" (auto-repeat del teclado)
HALF_WIDTH = 0.20     # igual que --half-width del puente
STOP_DIST = 0.28      # igual que --stop-dist del puente
KEYS = {ord("w"): (1, 0), ord("s"): (-1, 0), ord("a"): (0, 1), ord("d"): (0, -1)}


def draw(scan, status, max_range):
    img = np.zeros((SIZE, SIZE, 3), np.uint8)
    c = SIZE // 2
    k = (SIZE / 2 - 10) / max_range                      # px por metro

    def px(x, y):                                        # x adelante -> arriba, y izquierda -> izquierda
        return int(c - y * k), int(c - x * k)

    for m in range(1, int(max_range) + 1):
        cv2.circle(img, (c, c), int(m * k), (60, 60, 60), 1)
    cv2.line(img, (c, 0), (c, SIZE), (45, 45, 45), 1)
    cv2.line(img, (0, c), (SIZE, c), (45, 45, 45), 1)

    if status is not None:
        for sign, blocked in ((1, status.front_blocked), (-1, status.rear_blocked)):
            color = (0, 0, 255) if blocked else (0, 140, 255)
            far = sign * max_range
            for y in (-HALF_WIDTH, HALF_WIDTH):
                cv2.line(img, px(0, y), px(far, y), color, 1)
            cv2.line(img, px(sign * STOP_DIST, -HALF_WIDTH), px(sign * STOP_DIST, HALF_WIDTH), color, 2)

    if scan is not None:
        x, y = scan.points()
        u = np.clip((c - y * k).astype(int), 1, SIZE - 2)
        v = np.clip((c - x * k).astype(int), 1, SIZE - 2)
        near = np.hypot(x, y) < 0.5
        for color, sel in (((0, 200, 0), ~near), ((0, 0, 255), near)):
            for dv in (-1, 0, 1):
                for du in (-1, 0, 1):
                    img[v[sel] + dv, u[sel] + du] = color

    cv2.rectangle(img, (c - 6, c - 6), (c + 6, c + 6), (255, 255, 255), -1)
    cv2.line(img, (c, c - 6), (c, c - 18), (255, 255, 255), 2)   # frente
    return img


def text_lines(robot, status, scan, pose, speed, drive, msg):
    if status is None:
        return ["Esperando al puente..."]
    dist = lambda d: "libre" if math.isinf(d) else f"{d:.2f} m"
    lines = [f"bateria {status.battery * 100:.0f}%   dock: {status.docked}   PCs: {status.n_clients}",
             f"libre al frente {dist(status.front_clear)}   atras {dist(status.rear_clear)}"]
    alerts = [n for f, n in ((status.front_blocked, "FRENTE BLOQUEADO"), (status.rear_blocked, "ATRAS BLOQUEADO"),
                             (status.scan_stale, "SIN LIDAR (en el dock?)")) if f]
    if alerts:
        lines.append(" | ".join(alerts))
    if pose is not None:
        lines.append(f"odom x={pose.x:+.2f} y={pose.y:+.2f} yaw={math.degrees(pose.yaw):+.0f}  v={pose.v:+.2f}")
    if scan is not None:
        lines.append(f"scan {len(scan.ranges)} pts, edad {scan.age * 1000:.0f} ms")
    if drive:
        lines.append(f"CONDUCES: {'si' if status.is_driver else 'no'}   vel {speed:.2f} m/s (+/-)")
    if status.age > 1.5:
        lines.append(f"SIN DATOS DEL ROBOT hace {status.age:.1f} s")
    if msg:
        lines.append(msg)
    return lines


def main():
    ap = argparse.ArgumentParser(description="Monitor UDP del TurtleBot 4")
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--port", type=int, default=P.DEFAULT_PORT)
    ap.add_argument("--key", default=P.DEFAULT_KEY)
    ap.add_argument("--range", type=float, default=3.0, dest="max_range", help="metros mostrados")
    ap.add_argument("--drive", action="store_true", help="permitir conducir con el teclado")
    ap.add_argument("--speed", type=float, default=0.10, help="m/s inicial")
    ap.add_argument("--turn", type=float, default=0.8, help="rad/s al girar")
    args = ap.parse_args()

    win = "TurtleBot 4 - grupo 3"
    state = {"msg": "", "photo": None}

    def background(name, fn):
        def run():
            state["msg"] = f"{name}..."
            try:
                result = fn()
                if isinstance(result, np.ndarray):
                    state["photo"] = result
                    state["msg"] = f"{name}: {result.shape[1]}x{result.shape[0]}"
                else:
                    state["msg"] = f"{name}: {result}"
            except Exception as e:
                state["msg"] = f"{name} fallo: {e}"
        threading.Thread(target=run, daemon=True).start()

    with TB4Client(args.ip, args.port, args.key) as robot:
        speed, direction, last_key = args.speed, (0, 0), 0.0
        cv2.namedWindow(win)
        while True:
            status, scan, pose = robot.status(), robot.scan(), robot.pose()
            img = draw(scan, status, args.max_range)
            for i, line in enumerate(text_lines(robot, status, scan, pose, speed, args.drive, state["msg"])):
                cv2.putText(img, line, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imshow(win, img)
            if state["photo"] is not None:
                cv2.imshow("Camara", state["photo"])
                state["photo"] = None

            key = cv2.waitKey(30) & 0xFF
            now = time.monotonic()
            if key in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("p"):
                background("foto", robot.photo)
            if args.drive:
                if key in KEYS:
                    direction, last_key = KEYS[key], now
                elif key == ord(" "):
                    direction = (0, 0)
                    robot.stop()
                elif key in (ord("+"), ord("=")):
                    speed = min(0.31, speed + 0.05)
                elif key == ord("-"):
                    speed = max(0.05, speed - 0.05)
                elif key == ord("u"):
                    background("undock", robot.undock)
                elif key == ord("k"):
                    background("dock", robot.dock)
                if direction != (0, 0) and now - last_key > HOLD:
                    direction = (0, 0)
                    robot.stop()
                if direction != (0, 0):
                    robot.move(direction[0] * speed, direction[1] * args.turn)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
