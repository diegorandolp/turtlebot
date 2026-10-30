"""
Ejemplo de maniobra con el puente UDP: avanzar hasta un obstaculo, girar a la derecha
y desplazarse lateralmente. Base para la navegacion de la Tarea A.

Uso (desde la raiz del repositorio):
  python3 -m ejemplos.esquivar                 # usa el robot fuera del dock
  python3 -m ejemplos.esquivar --undock        # primero lo saca del dock
  python3 -m ejemplos.esquivar --lado izquierda --lateral 0.4

Pasos:
  1. Avanza a --vel hasta que el pasillo del robot tenga --parar-a m libres
     (antes de que actue el freno de emergencia del puente, a 0.28 m).
  2. Gira 90 grados hacia --lado usando la odometria.
  3. Avanza --lateral m manteniendo el rumbo, si hay sitio libre.
Ctrl+C detiene el robot en cualquier momento.
"""
import argparse
import math
import time

from tb4 import TB4Client

LOOP = 0.05            # s entre comandos (20 Hz)
SAFETY_MARGIN = 0.30   # m extra libres que pedimos antes de desplazarnos


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wait_lidar(robot, timeout=20.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s, sc = robot.status(), robot.scan()
        if s and not s.scan_stale and sc is not None and sc.age < 0.5:
            return
        time.sleep(0.2)
    raise RuntimeError("el LiDAR no publica (sigue en el dock?)")


def approach(robot, stop_at, speed, max_dist=1.5, timeout=20.0):
    """Avanza recto hasta tener stop_at m libres. Devuelve lo recorrido (m)."""
    p0 = robot.pose()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s, p = robot.status(), robot.pose()
        dist = math.hypot(p.x - p0.x, p.y - p0.y)
        if s.front_clear <= stop_at:
            robot.stop()
            return dist
        if dist > max_dist:
            robot.stop()
            raise RuntimeError(f"recorrio {dist:.2f} m sin encontrar el obstaculo")
        robot.move(speed, 0.0)
        time.sleep(LOOP)
    robot.stop()
    raise RuntimeError("tiempo agotado acercandose al obstaculo")


def turn(robot, angle, tol_deg=2.0, timeout=10.0):
    """Gira 'angle' rad (+ izquierda, - derecha) en el sitio con control proporcional sobre la odometria."""
    target = wrap(robot.pose().yaw + angle)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        err = wrap(target - robot.pose().yaw)
        if abs(err) < math.radians(tol_deg):
            robot.stop()
            time.sleep(0.3)
            return math.degrees(wrap(target - robot.pose().yaw))
        w = clamp(1.5 * err, -0.6, 0.6)
        if abs(w) < 0.15:
            w = math.copysign(0.15, w)
        robot.move(0.0, w)
        time.sleep(LOOP)
    robot.stop()
    raise RuntimeError("tiempo agotado girando")


def drive(robot, distance, speed, timeout=15.0):
    """Avanza 'distance' m en linea recta corrigiendo el rumbo. Devuelve lo recorrido (m)."""
    p0 = robot.pose()
    heading = p0.yaw
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s, p = robot.status(), robot.pose()
        dist = math.hypot(p.x - p0.x, p.y - p0.y)
        remaining = distance - dist
        if remaining <= 0.005:
            break
        if s.front_blocked:
            robot.stop()
            raise RuntimeError(f"el puente freno por un obstaculo tras {dist:.2f} m")
        v = clamp(remaining * 1.5, 0.05, speed)          # frena suave al llegar
        w = clamp(1.0 * wrap(heading - p.yaw), -0.3, 0.3)  # mantiene el rumbo
        robot.move(v, w)
        time.sleep(LOOP)
    robot.stop()
    time.sleep(0.5)
    p = robot.pose()
    return math.hypot(p.x - p0.x, p.y - p0.y)


def main():
    ap = argparse.ArgumentParser(description="Avanzar hasta un obstaculo y desplazarse a un lado")
    ap.add_argument("ip", nargs="?", default="192.168.0.103")
    ap.add_argument("--undock", action="store_true", help="sacar primero el robot del dock")
    ap.add_argument("--lado", choices=["derecha", "izquierda"], default="derecha")
    ap.add_argument("--lateral", type=float, default=0.50, help="m a desplazarse hacia el lado")
    ap.add_argument("--parar-a", type=float, default=0.35, help="m libres a los que parar delante del obstaculo")
    ap.add_argument("--vel", type=float, default=0.10, help="m/s")
    args = ap.parse_args()

    with TB4Client(args.ip) as robot:
        robot.wait_ready()
        time.sleep(1.0)
        if args.undock and robot.status().docked:
            print("undock:", robot.undock())
        wait_lidar(robot)

        s = robot.status()
        print(f"1) libre al frente: {s.front_clear:.2f} m")
        if s.front_clear <= args.parar_a:
            print("   ya esta a la distancia de parada, no avanzo")
        else:
            d = approach(robot, args.parar_a, args.vel)
            print(f"   avanzo {d * 100:.0f} cm y paro con {robot.status().front_clear:.2f} m libres")

        sign = -1 if args.lado == "derecha" else 1
        err = turn(robot, sign * math.pi / 2)
        print(f"2) giro 90 grados a la {args.lado} (error final {err:+.1f} grados)")

        free = robot.status().front_clear
        need = args.lateral + SAFETY_MARGIN
        if free < need:
            print(f"3) NO me desplazo: hay {free:.2f} m libres y necesito {need:.2f} m")
            return
        d = drive(robot, args.lateral, max(args.vel, 0.12))
        p = robot.pose()
        print(f"3) avanzo {d * 100:.1f} cm hacia la {args.lado}; pose final "
              f"({p.x:.2f}, {p.y:.2f}, {math.degrees(p.yaw):.0f} grados)")


if __name__ == "__main__":
    main()
