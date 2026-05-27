#!/usr/bin/env python3
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

MENU = """
PETER TELEOP :D
---------------------------
To move around:
   u    i    o
   j    k    l
   m    ,    .

To change mode:
---------------------------
 z: Movil type H mode
 x: Omnidirectional mode
 c: Quadruped mode

q/a : increase/decrease max speeds by 10%
w/s : increase/decrease only linear speed by 10%
e/d : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


def get_key(settings, timeout=0.1):
    """Read one character from stdin in raw mode. Returns '' on timeout."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.get_logger().info("\033[92mRunning Peter Teleop by Sam! :D\033[0m")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/peter_mode', 10)

        self.speed = 2.0
        self.turn = 2.0
        self.max_speed = 3.5
        self.min_speed = 0.1
        self.max_turn = 3.5
        self.min_turn = 0.1
        self.current_mode = 'C'

    def display_menu(self):
        print(MENU)
        print("currently:\tspeed {:.1f}\tturn {:.1f}\tmode {}".format(
            self.speed, self.turn, self.current_mode))

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.current_mode = mode

    def adjust_speed(self, linear_factor, angular_factor):
        self.speed = min(max(self.speed * linear_factor, self.min_speed), self.max_speed)
        self.turn = min(max(self.turn * angular_factor, self.min_turn), self.max_turn)

    def handle_key(self, key):
        """Process a single character key. Returns True if the menu should refresh."""
        movement = {
            'i': (self.speed,  0.0,       0.0),
            ',': (-self.speed, 0.0,       0.0),
            'j': (0.0,         self.speed, 0.0),
            'l': (0.0,        -self.speed, 0.0),
            'u': (0.0,         0.0,        self.turn),
            'o': (0.0,         0.0,       -self.turn),
            'm': (0.0,         0.0,        self.turn),
            '.': (0.0,         0.0,       -self.turn),
            'k': (0.0,         0.0,        0.0),
        }
        mode = {'z': 'H', 'x': 'X', 'c': 'C'}
        speed_adj = {
            'q': (1.1, 1.1), 'a': (0.9, 0.9),
            'w': (1.1, 1.0), 's': (0.9, 1.0),
            'e': (1.0, 1.1), 'd': (1.0, 0.9),
        }

        if key in movement:
            self.publish_twist(*movement[key])
        elif key in mode:
            self.publish_mode(mode[key])
        elif key in speed_adj:
            self.adjust_speed(*speed_adj[key])
        else:
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    node.display_menu()

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':  # Ctrl+C
                break
            if key and node.handle_key(key):
                node.display_menu()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.publish_twist()  # Detener robot al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
