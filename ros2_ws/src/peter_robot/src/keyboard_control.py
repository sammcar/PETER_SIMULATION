#!/usr/bin/env python3
# Teleop simple por stdin para ROS2 (Twist)
# Uso: source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
#       python3 ~/peter_teleop_stdin.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, tty, termios

class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.speed = 0.2
        self.turn = 1.0
        self.create_timer(0.1, self._publish)

    def _publish(self):
        msg = Twist()
        msg.linear.x = float(self.linear)
        msg.angular.z = float(self.angular)
        self.pub.publish(msg)

def print_help():
    print("\nPETER TELEOP :D")
    print("---------------------------")
    print("To move around:")
    print("   u    i    o")
    print("   j    k    l")
    print("   m    ,    .")
    print("\nTo change mode:")
    print("---------------------------")
    print(" z: Movil type H mode")
    print(" x: Omnidirectional mode")
    print(" c: Quadruped mode")
    print("\nq/a : increase/decrease max speeds by 10%")
    print("w/s : increase/decrease only linear speed by 10%")
    print("e/d : increase/decrease only angular speed by 10%")
    print("CTRL-C or q to quit\n")

def main():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = SimpleTeleop()
    print_help()
    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch == 'i':
                    node.linear = node.speed; node.angular = 0.0
                elif ch == 'k':
                    node.linear = -node.speed; node.angular = 0.0
                elif ch == 'j':
                    node.angular = node.turn; node.linear = 0.0
                elif ch == 'l':
                    node.angular = -node.turn; node.linear = 0.0
                elif ch == 'u':
                    node.linear = node.speed; node.angular = node.turn
                elif ch == 'o':
                    node.linear = node.speed; node.angular = -node.turn
                elif ch == 'm':
                    node.linear = -node.speed; node.angular = 0.0
                elif ch == ',':
                    node.linear = -node.speed; node.angular = node.turn
                elif ch == '.':
                    node.linear = -node.speed; node.angular = -node.turn
                elif ch == ' ':
                    node.linear = 0.0; node.angular = 0.0
                elif ch == 'q':
                    break
                elif ch == 'a':
                    node.speed *= 0.9; print(f"speed {node.speed:.2f}")
                elif ch == 'w':
                    node.speed *= 1.1; print(f"speed {node.speed:.2f}")
                elif ch == 's':
                    node.speed *= 0.9; print(f"speed {node.speed:.2f}")
                elif ch == 'e':
                    node.turn *= 1.1; print(f"turn {node.turn:.2f}")
                elif ch == 'd':
                    node.turn *= 0.9; print(f"turn {node.turn:.2f}")
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
