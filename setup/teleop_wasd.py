"""
Teleoperacion WASD del TurtleBot 4 desde la laptop (dentro de WSL).
Sin paquetes ROS2, sin 'ros2 topic pub': un solo script que usa rclpy.

Uso (en WSL, con la laptop en la red del robot):
  python3 teleop_wasd.py            # controla el robot real
  python3 teleop_wasd.py --dry      # solo muestra lo que enviaria, NO publica
  python3 teleop_wasd.py --topic /cmd_vel_unstamped

Controles (mantener la tecla apretada para avanzar):
  w / s : adelante / atras        a / d : girar izquierda / derecha
  q / e : adelante girando izquierda / derecha
  u / j : subir / bajar velocidad lineal     i / k : subir / bajar velocidad angular
  espacio o x : detener           Ctrl+C : salir

Seguridad:
  - Arranca lento (START_*) y tu subes la velocidad con u/i; tope LIMIT_* (maximo del robot).
  - Si pasan HOLD_TIMEOUT segundos sin tecla, el robot se detiene solo.
  - Al salir (o si falla algo) se envia velocidad cero.
  - Atras el Create3 no tiene sensores: ve despacio y mirando el camino.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

START_LINEAR = 0.1    # m/s   velocidad inicial (lenta a proposito)
START_ANGULAR = 1.0   # rad/s
STEP_LINEAR = 0.05
STEP_ANGULAR = 0.2
LIMIT_LINEAR = 0.46   # 0.31 es el maximo de fabrica; >0.31 solo con safety_override: full en el Create3
LIMIT_ANGULAR = 1.9   # tope = maximo del TurtleBot 4 (1.9 rad/s)
HOLD_TIMEOUT = 0.6   # s sin tecla -> parar (cubre el retardo inicial del auto-repeat)
RATE_HZ = 20
DISCOVERY_WAIT = 8.0  # s esperando que el robot aparezca

# tecla -> (factor lineal, factor angular)
KEYS = {
    "w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1),
    "q": (1, 1), "e": (1, -1),
}
STOP_KEYS = " x"


class Sender:
    """Publica Twist o TwistStamped segun lo que use el topic del robot."""

    def __init__(self, topic, dry):
        self.dry = dry
        if dry:
            return
        import rclpy
        from geometry_msgs.msg import Twist, TwistStamped

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("teleop_wasd")

        tname = self._wait(lambda: self._topic_type(topic), "el topic " + topic)
        self.stamped = tname.endswith("TwistStamped")
        self.msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.node.create_publisher(self.msg_type, topic, 10)

        self._wait(lambda: self.node.count_subscribers(topic) > 0,
                   "alguien escuchando en " + topic)
        print(f"Conectado: {topic} ({tname}), "
              f"{self.node.count_subscribers(topic)} suscriptor(es)")

    def _topic_type(self, topic):
        for name, types in self.node.get_topic_names_and_types():
            if name == topic:
                return types[0]
        return None

    def _wait(self, fn, what):
        end = time.monotonic() + DISCOVERY_WAIT
        while time.monotonic() < end:
            res = fn()
            if res:
                return res
            time.sleep(0.2)
        self.close()
        sys.exit(f"No aparece {what}. Revisa: misma red, ping al robot, "
                 "mismo ROS_DOMAIN_ID que el robot y ROS_STATIC_PEERS.")

    def send(self, linear, angular):
        if self.dry:
            return
        msg = self.msg_type()
        body = msg
        if self.stamped:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            body = msg.twist
        body.linear.x = float(linear)
        body.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        if self.dry:
            return
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


def read_keys(fd):
    """Devuelve todas las teclas pendientes (sin bloquear)."""
    data = b""
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        data += chunk
    return data.decode(errors="ignore").lower()


def show(linear, angular, max_l, max_a):
    sys.stdout.write(f"\r\x1b[K lineal={linear:+.2f} m/s  angular={angular:+.2f} rad/s"
                     f"   [vel: {max_l:.2f} m/s, {max_a:.2f} rad/s]")
    sys.stdout.flush()


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    parser = argparse.ArgumentParser(description="Teleop WASD TurtleBot 4")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--dry", action="store_true", help="no publicar, solo mostrar")
    args = parser.parse_args()

    if not sys.stdin.isatty():
        sys.exit("Necesita una terminal interactiva (ejecutalo directo en WSL).")

    sender = Sender(args.topic, args.dry)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print("w/s adelante/atras | a/d girar | q/e adelante+giro | espacio/x parar | Ctrl+C salir")
    print("u/j velocidad lineal +/- | i/k velocidad angular +/-")
    if args.dry:
        print("[MODO DRY: no se publica nada]")

    max_l, max_a = START_LINEAR, START_ANGULAR
    direction = (0, 0)
    linear = angular = 0.0
    last_key = 0.0
    shown = (0.0, 0.0, max_l, max_a)
    period = 1.0 / RATE_HZ
    try:
        tty.setcbreak(fd)  # Ctrl+C sigue funcionando
        show(0.0, 0.0, max_l, max_a)
        while True:
            t0 = time.monotonic()
            for k in read_keys(fd):
                if k in KEYS:
                    direction = KEYS[k]
                    last_key = t0
                elif k in STOP_KEYS:
                    direction = (0, 0)
                elif k == "u":
                    max_l = clamp(max_l + STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif k == "j":
                    max_l = clamp(max_l - STEP_LINEAR, STEP_LINEAR, LIMIT_LINEAR)
                elif k == "i":
                    max_a = clamp(max_a + STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)
                elif k == "k":
                    max_a = clamp(max_a - STEP_ANGULAR, STEP_ANGULAR, LIMIT_ANGULAR)

            if direction != (0, 0) and t0 - last_key > HOLD_TIMEOUT:
                direction = (0, 0)
            linear, angular = direction[0] * max_l, direction[1] * max_a

            if linear or angular:
                sender.send(linear, angular)
            elif shown[:2] != (0.0, 0.0):
                sender.stop()

            if shown != (linear, angular, max_l, max_a):
                show(linear, angular, max_l, max_a)
                shown = (linear, angular, max_l, max_a)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sender.stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sender.close()
            print("\nDetenido. Adios.")


if __name__ == "__main__":
    main()
