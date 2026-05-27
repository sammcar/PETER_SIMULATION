#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

#Nota: el indice de robustez se calcula haciendo múltiples pruebas por lo tanto es necesario que sea un nodo
#separado que se ejecuta siempre

class RIListener(Node):

    def __init__(self):
        super().__init__('ri_listener')

        self.rmse_0 = None
        self.rmse_5 = None
        self.rmse_10 = None

        self.step = 0  # controla el orden

        self.create_subscription(Float64, '/rmse_ct', self.callback, 10)

        self.timer = self.create_timer(0.5, self.compute_ri)

    def callback(self, msg):

        if self.step == 0:
            self.rmse_0 = msg.data
            self.get_logger().info(f"RMSE0 recibido: {self.rmse_0}")
            self.step += 1

        elif self.step == 1:
            self.rmse_5 = msg.data
            self.get_logger().info(f"RMSE5 recibido: {self.rmse_5}")
            self.step += 1

        elif self.step == 2:
            self.rmse_10 = msg.data
            self.get_logger().info(f"RMSE10 recibido: {self.rmse_10}")
            self.step += 1

    def compute_ri(self):

        if None in (self.rmse_0, self.rmse_5, self.rmse_10):
            return

        ri = 1.0 - (
            ((self.rmse_5 - self.rmse_0) +
             (self.rmse_10 - self.rmse_0)) /
            (2.0 * self.rmse_0)
        )

        self.get_logger().info(
            f"\nRMSE0={self.rmse_0:.3f}"
            f"\nRMSE5={self.rmse_5:.3f}"
            f"\nRMSE10={self.rmse_10:.3f}"
            f"\nRI={ri:.3f}"
        )

        # opcional: evitar recomputar
        self.step = 99


def main():
    rclpy.init()
    node = RIListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()