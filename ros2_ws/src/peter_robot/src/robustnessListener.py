#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray


#Nota: el indice de robustez se calcula haciendo múltiples pruebas por lo tanto es necesario que sea un nodo 
#separado que se ejecuta siempre

class RIListener(Node):

    def __init__(self):

        super().__init__('ri_listener')

        # RMSEs por nivel de ruido
        self.rmse_0 = None
        self.rmse_1 = None
        self.rmse_2 = None

        # último nivel de ruido recibido
        self.current_noise_idx = None

        # subscripciones
        self.create_subscription(
            Float64,
            '/rmse_ct',
            self.rmse_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/Metrics',
            self.metrics_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.compute_ri)

    # ---------------------------------------------------------
    # recibe métricas
    # ---------------------------------------------------------

    def metrics_callback(self, msg):

        try:
            self.current_noise_idx = int(msg.data[4])

            self.get_logger().info(
                f"Nivel de ruido recibido: {self.current_noise_idx}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Error leyendo /Metrics: {e}"
            )

    # ---------------------------------------------------------
    # recibe RMSE
    # ---------------------------------------------------------

    def rmse_callback(self, msg):

        if self.current_noise_idx is None:
            self.get_logger().warn(
                "Todavía no se ha recibido el nivel de ruido"
            )
            return

        rmse = msg.data

        if self.current_noise_idx == 0:

            self.rmse_0 = rmse

            self.get_logger().info(
                f"RMSE ruido 0 guardado: {rmse}"
            )

        elif self.current_noise_idx == 1:

            self.rmse_1 = rmse

            self.get_logger().info(
                f"RMSE ruido 1 guardado: {rmse}"
            )

        elif self.current_noise_idx == 2:

            self.rmse_2 = rmse

            self.get_logger().info(
                f"RMSE ruido 2 guardado: {rmse}"
            )

    # ---------------------------------------------------------
    # calcula robustness index
    # ---------------------------------------------------------

    def compute_ri(self):

        if None in (self.rmse_0, self.rmse_1, self.rmse_2):
            return

        # evitar división por cero
        if self.rmse_0 == 0.0:

            self.get_logger().error(
                "RMSE base es 0, no se puede calcular RI"
            )

            return

        ri = 1.0 - (
            (
                (self.rmse_1 - self.rmse_0) +
                (self.rmse_2 - self.rmse_0)
            ) /
            (2.0 * self.rmse_0)
        )

        self.get_logger().info(
            f"\nRMSE0={self.rmse_0:.3f}"
            f"\nRMSE1={self.rmse_1:.3f}"
            f"\nRMSE2={self.rmse_2:.3f}"
            f"\nRI={ri:.3f}"
        )

        # opcional:
        # reiniciar después de calcular

        # self.rmse_0 = None
        # self.rmse_1 = None
        # self.rmse_2 = None


def main():

    rclpy.init()

    node = RIListener()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()