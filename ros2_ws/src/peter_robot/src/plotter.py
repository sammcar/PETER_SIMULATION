#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from threading import Thread
from time import time, strftime
import sys

class CmdVelPlotter(Node):
    def __init__(self):
        super().__init__('plotter')

        self.time_stamps = []
        self.linear_x = []
        self.linear_y = []
        self.angular_z = []

        self.start_time = time()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Nodo plotter iniciado. Presiona Enter para graficar. Usa Ctrl+C para cerrar sin graficar.')

    def cmd_vel_callback(self, msg):
        t = time() - self.start_time
        self.time_stamps.append(t)
        self.linear_x.append(msg.linear.x)
        self.linear_y.append(msg.linear.y)
        self.angular_z.append(msg.angular.z)

    def plot_data(self):
        if not self.time_stamps:
            self.get_logger().warn('No se registraron datos.')
            return

        # Crear la gráfica
        plt.figure()
        plt.plot(self.time_stamps, self.linear_x, label='linear.x')
        plt.plot(self.time_stamps, self.linear_y, label='linear.y')
        plt.plot(self.time_stamps, self.angular_z, label='angular.z')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Valor')
        plt.title('Evolución de cmd_vel')
        plt.legend()
        plt.grid(True)

        # Guardar como imagen PNG
        timestamp = strftime("%Y%m%d_%H%M%S")
        filename = f'cmd_vel_plot_{timestamp}.png'
        plt.savefig(filename)
        self.get_logger().info(f'📊 Gráfica guardada como {filename}')

        # Mostrar gráfica
        plt.show()


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CmdVelPlotter()

    thread = Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    try:
        while True:
            input("🔁 Presiona Enter para graficar y guardar imagen...\n")
            node.plot_data()
    except KeyboardInterrupt:
        pass  # Ctrl+C simplemente cierra el nodo
    finally:
        rclpy.shutdown()
        thread.join()
        sys.exit(0)

if __name__ == '__main__':
    main()
