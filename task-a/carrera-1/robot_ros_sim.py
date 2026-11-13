#!/usr/bin/env python3
"""
El simulador de carrera-1 hecho robot de ROS 2: publica lo mismo que el TurtleBot 4 (/scan, /odom,
/battery_state, /dock_status y las camaras /oakd/rgb/preview/image_raw y /oakd/rgb/image_raw) y obedece
/cmd_vel (TwistStamped). Con el puente real (robot/tb4_bridge.py) delante, el programa de la PC no nota
la diferencia: asi se prueba el camino completo PC -> UDP -> puente -> ROS 2 sin el robot.

Lo mas facil es usar sim_ros.sh, que lo arranca junto con el puente en un dominio ROS aislado:
  task-a/carrera-1/sim_ros.sh demo                  # luego, en otra terminal:
  python task-a/carrera-1/carrera.py 127.0.0.1 --go

Necesita ROS 2 Jazzy (en esta PC: el distrobox ros-jazzy) y numpy/opencv.
"""
import argparse
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image, LaserScan

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from mundos import construir                     # noqa: E402
from simulador import SimRobot                   # noqa: E402


class RobotSim(Node):
    def __init__(self, a):
        super().__init__("robot_sim")
        self.a = a
        self.mundo = construir(a.mundo, a.persona)
        self.sim = SimRobot(self.mundo, hfov=a.hfov, foto_hd=(1280, 720, a.hfov_hd), freno=False)
        self.get_logger().info(f"Mundo {self.mundo.nombre}: en B esta {self.mundo.persona} ({self.mundo.foto_b})")
        self.p_scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.p_odom = self.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
        self.p_bat = self.create_publisher(BatteryState, "/battery_state", qos_profile_sensor_data)
        self.p_prev = self.create_publisher(Image, "/oakd/rgb/preview/image_raw", qos_profile_sensor_data)
        self.p_hd = self.create_publisher(Image, "/oakd/rgb/image_raw", qos_profile_sensor_data)
        self.p_dock = None
        try:
            from irobot_create_msgs.msg import DockStatus
            self.DockStatus = DockStatus
            self.p_dock = self.create_publisher(DockStatus, "/dock_status", qos_profile_sensor_data)
        except ImportError:
            pass
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, 10)
        self.t_real0 = self.t_fis = time.monotonic()
        self.lock = threading.Lock()
        self.create_timer(0.02, self.fisica)
        self.create_timer(1 / 7.0, self.pub_scan)
        self.create_timer(1 / 20.0, self.pub_odom)
        self.create_timer(1.0, self.pub_estado)
        self.choques = 0
        self.vivo = True
        threading.Thread(target=self.loop_camara, daemon=True).start()   # renderizar no frena la fisica

    def stamp(self):
        return self.get_clock().now().to_msg()

    def on_cmd(self, msg):
        self.sim.move(msg.twist.linear.x, msg.twist.angular.z)

    def fisica(self):
        now = time.monotonic()
        dt, self.t_fis = min(now - self.t_fis, 0.1), now                 # al ritmo del reloj real
        with self.lock:
            self.sim.advance(dt)
        if self.sim.collisions != self.choques:
            self.choques = self.sim.collisions
            self.get_logger().warn(f"CHOQUE ({self.choques})")

    def pub_scan(self):
        s = self.sim.scan()
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.stamp(), "rplidar_link"
        off = math.radians(self.a.lidar_offset_deg)              # el LiDAR del TB4 va girado: el puente lo corrige
        m.angle_min = float(s.angles[0] - off)
        m.angle_increment = float(s.angles[1] - s.angles[0])
        m.angle_max = float(m.angle_min + m.angle_increment * (len(s.angles) - 1))
        m.range_min, m.range_max = 0.15, 12.0
        r = np.where(np.isfinite(s.ranges), s.ranges, np.inf)
        m.ranges = [float(v) for v in r]
        self.p_scan.publish(m)

    def pub_odom(self):
        p = self.sim.pose()
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = self.stamp(), "odom", "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = p.x, p.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(p.yaw / 2), math.cos(p.yaw / 2)
        m.twist.twist.linear.x, m.twist.twist.angular.z = self.sim.v, self.sim.w
        self.p_odom.publish(m)

    def _imagen(self, img):
        m = Image()
        m.header.stamp, m.header.frame_id = self.stamp(), "oakd_rgb_camera_optical_frame"
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = "bgr8", img.shape[1] * 3
        m.data = img.tobytes()
        return m

    def loop_camara(self):
        while self.vivo:
            t0 = time.monotonic()
            for pub, hd in ((self.p_prev, False), (self.p_hd, True)):
                if pub.get_subscription_count():                 # solo se renderiza si alguien mira
                    with self.lock:
                        x, y, th = self.sim.verdad()
                    cam = self.sim.cam_hd if hd else self.sim.cam
                    pub.publish(self._imagen(cam.foto(x, y, th)))
            time.sleep(max(0.0, 1 / self.a.fps - (time.monotonic() - t0)))

    def pub_estado(self):
        b = BatteryState()
        b.header.stamp, b.percentage = self.stamp(), 0.9
        self.p_bat.publish(b)
        if self.p_dock is not None:
            d = self.DockStatus()
            d.header.stamp, d.is_docked = self.stamp(), False
            self.p_dock.publish(d)
        x, y, _ = self.sim.verdad()
        ax, ay, _ = self.mundo.inicio
        self.get_logger().info(f"t={time.monotonic() - self.t_real0:5.1f}s  a {math.hypot(x - ax, y - ay):.2f} m de A, "
                               f"choques {self.sim.collisions}", throttle_duration_sec=5.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mundo", nargs="?", default="demo")
    ap.add_argument("--persona", default=None)
    ap.add_argument("--fps", type=float, default=15.0, help="cuadros por segundo de las camaras")
    ap.add_argument("--hfov", type=float, default=40.0, help="campo de vision de la preview (grados)")
    ap.add_argument("--hfov-hd", type=float, default=66.0, help="campo de vision de la imagen HD (grados)")
    ap.add_argument("--lidar-offset-deg", type=float, default=90.0, help="giro del LiDAR (como el robot real)")
    a = ap.parse_args()
    rclpy.init()
    node = RobotSim(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vivo = False
        x, y, _ = node.sim.verdad()
        ax, ay, _ = node.mundo.inicio
        print(f"[robot_sim] fin: a {math.hypot(x - ax, y - ay):.2f} m de A, choques {node.sim.collisions}, "
              f"persona en B: {node.mundo.persona}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
