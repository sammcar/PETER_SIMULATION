#!/usr/bin/env python3
"""
peter_teleop_stdin.py
Teleoperación por teclado vía stdin (sin pynput/pty).
Diseñado para ejecutarse desde el host WSL donde la captura de stdin es fiable.

Teclas:
  w / ↑  — avanzar
  s / ↓  — retroceder
  a / ←  — girar izquierda
  d / →  — girar derecha
  q      — desplazamiento lateral izquierda
  e      — desplazamiento lateral derecha
  espacio — detener
  x      — salir

Uso (host WSL):
  python3 ros2_ws/src/peter_robot/src/peter_teleop_stdin.py

O con ROS sourced:
  ros2 run peter_robot peter_teleop_stdin
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
────────────────────────────────────────
  Peter Teleop — stdin (host WSL)
────────────────────────────────────────
  w / ↑    avanzar
  s / ↓    retroceder
  a / ←    girar izquierda
  d / →    girar derecha
  q        lateral izquierda
  e        lateral derecha
  ESPACIO  detener
  x        salir
────────────────────────────────────────
"""

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1.0
LATERAL_SPEED = 0.5

KEY_BINDINGS = {
    'w':      (LINEAR_SPEED,  0.0,           0.0),
    's':      (-LINEAR_SPEED, 0.0,           0.0),
    'a':      (0.0,           0.0,           ANGULAR_SPEED),
    'd':      (0.0,           0.0,           -ANGULAR_SPEED),
    'q':      (0.0,           LATERAL_SPEED, 0.0),
    'e':      (0.0,           -LATERAL_SPEED, 0.0),
    ' ':      (0.0,           0.0,           0.0),
    # Arrow keys (escape sequences)
    '\x1b[A': (LINEAR_SPEED,  0.0,           0.0),   # ↑
    '\x1b[B': (-LINEAR_SPEED, 0.0,           0.0),   # ↓
    '\x1b[C': (0.0,           0.0,           -ANGULAR_SPEED),  # →
    '\x1b[D': (0.0,           0.0,           ANGULAR_SPEED),   # ←
}


def get_key(fd: int) -> str:
    """Lee un carácter o secuencia de escape del terminal."""
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        # Posible secuencia de escape (flecha)
        try:
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        except Exception:
            return ch
    return ch


class TeleopStdin(Node):

    def __init__(self):
        super().__init__('peter_teleop_stdin')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('[TeleopStdin] Publicando en /cmd_vel')

    def publish(self, lin_x: float, lin_y: float, ang_z: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.linear.y = lin_y
        msg.angular.z = ang_z
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopStdin()

    print(HELP)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            key = get_key(fd)
            if key in ('x', '\x03'):  # x o Ctrl-C
                break
            if key in KEY_BINDINGS:
                lin_x, lin_y, ang_z = KEY_BINDINGS[key]
                node.publish(lin_x, lin_y, ang_z)
            # Ignorar teclas desconocidas sin error
    except Exception as exc:
        node.get_logger().error(f'[TeleopStdin] Error: {exc}')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.publish(0.0, 0.0, 0.0)  # stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
