"""
Visor del lidar del TurtleBot 4 (topic /scan) desde la laptop, dentro de WSL.
Vista desde arriba: el robot es el cuadro blanco del centro, el frente apunta hacia arriba.
Puntos verdes = obstaculos; rojos = muy cerca del frente. Anillos grises cada 1 m.

Uso (WSL, laptop en la red del robot):
  python3 ver_lidar_tb4.py
  python3 ver_lidar_tb4.py --rot 90      # si el frente real sale de lado, gira la vista
  python3 ver_lidar_tb4.py --range 6     # metros mostrados

Salir: tecla q / Esc, o cerrar la ventana.
Solo LEE /scan: no mueve ni configura nada en el robot.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SIZE = 720
NEAR_FRONT_M = 0.5   # rojo si hay algo mas cerca que esto en el frente
FRONT_DEG = 30       # semiancho del "frente"


def render(msg, max_range=4.0, rot_deg=0.0, size=SIZE):
    """LaserScan -> imagen BGR (numpy). Devuelve (imagen, dist_minima_m o None)."""
    img = np.zeros((size, size, 3), np.uint8)
    c = size // 2
    scale = (size / 2 - 10) / max_range  # px por metro

    yy, xx = np.ogrid[:size, :size]
    d = np.hypot(xx - c, yy - c) / scale
    for m in range(1, int(max_range) + 1):
        img[np.abs(d - m) < 1.0 / scale] = (70, 70, 70)
    img[c, :] = (70, 70, 70)
    img[:, c] = (70, 70, 70)

    r = np.asarray(msg.ranges, dtype=np.float32)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment + np.radians(rot_deg)
    ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= min(msg.range_max, max_range))
    r, a = r[ok], a[ok]

    x = r * np.cos(a)            # adelante
    y = r * np.sin(a)            # izquierda
    u = np.clip((c - y * scale).astype(int), 1, size - 2)
    v = np.clip((c - x * scale).astype(int), 1, size - 2)

    near = (np.abs(np.degrees(np.arctan2(y, x))) < FRONT_DEG) & (x > 0) & (r < NEAR_FRONT_M)
    for color, sel in (((0, 200, 0), ~near), ((0, 0, 255), near)):
        for dv in (-1, 0, 1):
            for du in (-1, 0, 1):
                img[v[sel] + dv, u[sel] + du] = color

    img[c - 6:c + 7, c - 6:c + 7] = (255, 255, 255)   # robot
    img[c - 14:c - 6, c - 1:c + 2] = (255, 255, 255)  # marca de "frente"

    return img, (float(r.min()) if len(r) else None)


def main():
    ap = argparse.ArgumentParser(description="Visor lidar TurtleBot 4")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--range", type=float, default=4.0, dest="max_range")
    ap.add_argument("--rot", type=float, default=0.0, help="grados para girar la vista")
    args = ap.parse_args()

    import cv2

    rclpy.init()
    node = rclpy.create_node("ver_lidar_tb4")
    state = {"msg": None, "new": False}

    def on_scan(m):
        state["msg"], state["new"] = m, True

    node.create_subscription(LaserScan, args.topic, on_scan, qos_profile_sensor_data)

    win = "Lidar TurtleBot 4"
    cv2.namedWindow(win)
    blank = np.zeros((SIZE, SIZE, 3), np.uint8)
    cv2.putText(blank, f"Esperando {args.topic} ...", (40, SIZE // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)
    cv2.imshow(win, blank)

    t_start, warned = time.monotonic(), False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["new"]:
                state["new"] = False
                img, dmin = render(state["msg"], args.max_range, args.rot)
                txt = f"mas cercano: {dmin:.2f} m" if dmin is not None else "sin lecturas"
                cv2.putText(img, txt, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 255, 255), 2)
                cv2.imshow(win, img)
            elif state["msg"] is None and not warned and time.monotonic() - t_start > 8:
                warned = True
                print(f"Sin datos en {args.topic}. Revisa: misma red, mismo ROS_DOMAIN_ID que el robot, "
                      "ROS_STATIC_PEERS, y 'ros2 topic hz /scan'.")
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
