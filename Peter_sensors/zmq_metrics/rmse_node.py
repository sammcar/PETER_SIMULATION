#!/usr/bin/env python3
"""
rmse_node.py — RMSE de error cross-track en trayectoria recta.

En simulación la pose venía del CLI de Gazebo (gz topic).
En el robot físico se suscribe al tópico ZMQ /pose que debe publicar
un publisher de odometría/localización (a implementar más adelante).

Prueba: trayectoria recta de start=[0,0] a goal=[2,0], duración 20 s.
Publica el RMSE final en /rmse_ct (Float64) al terminar.
"""

import time

import numpy as np

from transport import (
    Node, spin,
    Float64, PoseStamped,
)
from topics import TOPIC_RMSE_CT, TOPIC_POSE


class RMSEPublisher(Node):

    def __init__(self, duration=20.0):
        super().__init__('rmse_publisher')

        self.goal      = np.array([2.0, 0.0])
        self.start     = np.array([0.0, 0.0])
        self.direction = self.goal - self.start
        self.direction_norm = np.linalg.norm(self.direction)

        self.errors_ct  = []
        self.start_time = time.time()
        self.duration   = duration
        self.published  = False

        # Publisher
        self.rmse_pub = self.create_publisher(Float64, TOPIC_RMSE_CT, 10)

        # Subscribe to /pose published by the odometry node
        self.create_subscription(PoseStamped, TOPIC_POSE, self._cb_pose, 10)

        self.get_logger().info(
            f"RMSE node started — waiting for poses on '{TOPIC_POSE}' "
            f"(duration: {self.duration} s)"
        )

    # ── Cross-track error ──────────────────────────────────────────────────

    def _cross_track_error(self, x, y):
        dx, dy = self.direction
        px = x - self.start[0]
        py = y - self.start[1]
        cross = dx * py - dy * px
        return abs(cross) / self.direction_norm

    # ── Pose callback (replaces the gz topic subprocess) ──────────────────

    def _cb_pose(self, msg):
        if self.published:
            return

        ct = self._cross_track_error(msg.x, msg.y)
        self.errors_ct.append(ct)

        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f"t={elapsed:.1f}s | CT={ct:.3f} | X={msg.x:.4f} Y={msg.y:.4f}"
        )

        if elapsed >= self.duration:
            rmse_ct = float(np.sqrt(np.mean(np.array(self.errors_ct) ** 2)))

            out      = Float64()
            out.data = rmse_ct
            self.rmse_pub.publish(out)

            self.get_logger().info(f"FINAL RMSE CT = {rmse_ct:.4f}")
            self.published = True


def main():
    node = RMSEPublisher()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
