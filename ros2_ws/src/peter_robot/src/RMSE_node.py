#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import numpy as np
import subprocess
import re
import time

#NOTA: se integró este código al principal de la red para evitar la ejecución de muchos códigos al mismo tiempo pero
#esto afectó gravemente el rendimiento de la red, por lo tanto lo volví a separar


class RMSEPublisher(Node):

    def __init__(self):
        super().__init__('rmse_publisher')

        # Publisher final
        self.rmse_pub = self.create_publisher(Float64, '/rmse_ct', 10)

        # Goal y start
        self.goal = np.array([2.0, 0.0])
        self.start = np.array([0.0, 0.0])

        self.direction = self.goal - self.start
        self.direction_norm = np.linalg.norm(self.direction)

        # buffers
        self.errors_ct = []

        # control de tiempo
        self.start_time = time.time()
        self.duration = 70.0  # duración de la prueba (segundos)
        self.published = False

        # timer
        self.timer = self.create_timer(0.1, self.step)

        self.get_logger().info("RMSE node started")

    # -----------------------------
    # lectura gazebo
    # -----------------------------
    def get_peter_pose(self):
        cmd = [
            "ign", "topic",
            "-e",
            "-t", "/world/default/pose/info",
            "-n", "1"
        ]

        try:
            output = subprocess.check_output(cmd, timeout=0.5).decode("utf-8")
        except subprocess.TimeoutExpired:
            return None

        pattern = r'name: "peter".*?position\s*{\s*x:\s*([-\d.eE]+)\s*y:\s*([-\d.eE]+)\s*z:\s*([-\d.eE]+)'
        match = re.search(pattern, output, re.DOTALL)

        if match:
            return np.array(list(map(float, match.groups())))

        return None

    # -----------------------------
    # error cross track
    # -----------------------------
    def cross_track_error(self, p):
        p2 = p[:2]

        dx = self.direction[0]
        dy = self.direction[1]

        px = p2[0] - self.start[0]
        py = p2[1] - self.start[1]

        cross = dx * py - dy * px

        return abs(cross) / self.direction_norm

    # -----------------------------
    # loop principal
    # -----------------------------
    def step(self):

        pose = self.get_peter_pose()

        if pose is None:
            #self.get_logger().info("No se encontró pose")
            return

        ct = self.cross_track_error(pose)
        self.errors_ct.append(ct)

        elapsed = time.time() - self.start_time

        # debug opcional
        # self.get_logger().info(f"t={elapsed:.1f}s | CT={ct:.3f}")
        # self.get_logger().info(f"X {pose[0]:.4f}")
        # self.get_logger().info(f"Y {pose[1]:.4f}")
        # self.get_logger().info(f"Z {pose[2]:.4f}")

        # -----------------------------
        # FINAL DE PRUEBA
        # -----------------------------
        if elapsed >= self.duration and not self.published:

            rmse_ct = np.sqrt(np.mean(np.array(self.errors_ct) ** 2))

            msg = Float64()
            msg.data = float(rmse_ct)

            self.rmse_pub.publish(msg)

            self.get_logger().info(f"FINAL RMSE CT = {rmse_ct:.4f}")

            self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = RMSEPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()