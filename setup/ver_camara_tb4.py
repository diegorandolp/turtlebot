"""
Visor de la camara OAK-D del TurtleBot 4 desde la laptop, dentro de WSL.
Lee el topic de imagen por ROS. No toca nada en el robot.

Uso (WSL, laptop en la red del robot):
  python3 ver_camara_tb4.py                       # JPEG comprimido (recomendado por Wi-Fi)
  python3 ver_camara_tb4.py --topic /oakd/rgb/preview/image_raw --scale 3   # imagen cruda

Si el topic termina en /compressed se usa sensor_msgs/CompressedImage; si no, sensor_msgs/Image.

Salir: tecla q / Esc, o cerrar la ventana.
Nota: el video por Wi-Fi puede verse entrecortado; es normal si la senal es floja.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def to_bgr(msg):
    """sensor_msgs/Image -> array numpy listo para mostrar (BGR o gris)."""
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        raise ValueError(f"encoding no soportado: {msg.encoding}")
    h, w = msg.height, msg.width
    arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(h, msg.step)[:, :w * ch]
    arr = arr.reshape(h, w, ch)
    if msg.encoding == "rgb8":
        arr = arr[:, :, ::-1]
    elif msg.encoding == "rgba8":
        arr = arr[:, :, 2::-1]
    elif msg.encoding == "bgra8":
        arr = arr[:, :, :3]
    elif msg.encoding == "mono8":
        arr = arr[:, :, 0]
    return np.ascontiguousarray(arr)


def main():
    ap = argparse.ArgumentParser(description="Visor camara TurtleBot 4")
    ap.add_argument("--topic", default="/oakd/rgb/preview/image_raw/compressed")
    ap.add_argument("--scale", type=float, default=2.0, help="factor de aumento")
    args = ap.parse_args()

    import cv2

    rclpy.init()
    node = rclpy.create_node("ver_camara_tb4")
    state = {"msg": None, "new": False}

    def on_image(m):
        state["msg"], state["new"] = m, True

    compressed = args.topic.endswith("/compressed")
    if compressed:
        decode = lambda m: cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
    else:
        decode = to_bgr
    node.create_subscription(CompressedImage if compressed else Image, args.topic, on_image,
                             qos_profile_sensor_data)

    win = "Camara TurtleBot 4"
    cv2.namedWindow(win)
    t_start, warned = time.monotonic(), False
    frames, t_fps, fps = 0, time.monotonic(), 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["new"]:
                state["new"] = False
                img = decode(state["msg"])
                if img is None:  # JPEG corrupto (paquete perdido por Wi-Fi)
                    continue
                if args.scale != 1.0:
                    img = cv2.resize(img, None, fx=args.scale, fy=args.scale,
                                     interpolation=cv2.INTER_LINEAR)
                frames += 1
                now = time.monotonic()
                if now - t_fps >= 1.0:
                    fps, frames, t_fps = frames / (now - t_fps), 0, now
                cv2.putText(img, f"{fps:.0f} fps", (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)
                cv2.imshow(win, img)
            elif state["msg"] is None and not warned and time.monotonic() - t_start > 8:
                warned = True
                print(f"Sin imagen en {args.topic}. Revisa: misma red, mismo ROS_DOMAIN_ID que el robot, "
                      "ROS_STATIC_PEERS y 'ros2 topic list | grep image'.")
            key = cv2.waitKey(1) & 0xFF
            try:
                closed = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1
            except cv2.error:
                closed = False
            if key in (ord("q"), 27) or (state["msg"] is not None and closed):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
