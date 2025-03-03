#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/peter_mode', 10)

        # Velocidades predeterminadas
        self.speed = 2.0  # Velocidad lineal
        self.turn = 2.0   # Velocidad angular

        # Limite para velocidad lineal
        self.max_speed = 3.5
        self.min_speed = 0.1

        # Limite para velocidad angular
        self.max_turn = 3.5
        self.min_turn = 0.1

        # Modo Actual
        self.current_mode = 'C'

    def display_menu(self):
        """Limpia la terminal y muestra el menú de teleoperación."""

        os.system('cls' if os.name == 'nt' else 'clear')
        self.get_logger().info("\033[92mRunning Peter Teleop by Sam! :D\033[0m")
        
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
        print("\nCTRL-Z to quit")
        print("\ncurrently:\tspeed {:.1f}\tturn {:.1f}\tmode {}".format(self.speed, self.turn, self.current_mode))

    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publica el mensaje Twist con los valores dados."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def publish_mode(self, mode):
        """Publica el modo actual."""
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        self.current_mode = mode

    def on_press(self, key):
        """Callback para capturar teclas presionadas."""
        try:
            key_char = key.char

            movement_commands = {
                'i': lambda: self.publish_twist(linear_x=self.speed),  # Adelante
                ',': lambda: self.publish_twist(linear_x=-self.speed),  # Atrás
                'j': lambda: self.publish_twist(linear_y=self.speed),  # Izquierda
                'l': lambda: self.publish_twist(linear_y=-self.speed),  # Derecha
                'u': lambda: self.publish_twist(angular_z=self.turn),  # Gira izquierda
                'o': lambda: self.publish_twist(angular_z=-self.turn),  # Gira derecha
                'm': lambda: self.publish_twist(angular_z=self.turn),  # Gira izquierda
                '.': lambda: self.publish_twist(angular_z=-self.turn)  # Gira derecha
            }

            mode_commands = {
                'z': lambda: self.publish_mode('H'),   # Modo H móvil
                'x': lambda: self.publish_mode('X'),  # Modo omnidireccional
                'c': lambda: self.publish_mode('C')  # Modo cuadrúpedo
            }

            speed_commands = {
                'q': lambda: self.adjust_speed(1.1, 1.1),  # Incrementa ambas velocidades
                'a': lambda: self.adjust_speed(0.9, 0.9),  # Reduce ambas velocidades
                'w': lambda: self.adjust_speed(1.1, 1.0),  # Incrementa solo lineal
                's': lambda: self.adjust_speed(0.9, 1.0),  # Reduce solo lineal
                'e': lambda: self.adjust_speed(1.0, 1.1),   # Incrementa solo angular
                'd': lambda: self.adjust_speed(1.0, 0.9)   # Incrementa solo angular
            }

            if key_char in movement_commands:
                movement_commands[key_char]()
            elif key_char in mode_commands:
                mode_commands[key_char]()
            elif key_char in speed_commands:
                speed_commands[key_char]()
            else:
                self.publish_twist()

            self.display_menu()

        except AttributeError:
            pass

    def adjust_speed(self, linear_factor, angular_factor):
        """Ajusta las velocidades lineales y angulares."""
        self.speed = min(max(self.speed * linear_factor, self.min_speed), self.max_speed)
        self.turn = min(max(self.turn * angular_factor, self.min_turn), self.max_turn)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = Teleop()
    teleop_node.display_menu()
    listener = keyboard.Listener(on_press=teleop_node.on_press)
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
