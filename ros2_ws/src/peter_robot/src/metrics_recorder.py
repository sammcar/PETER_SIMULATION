#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float64


class MetricsRecorder(Node):

    def __init__(self):
        super().__init__('metrics_recorder')

        # --------------------------------------------------
        # Ruta de salida
        # --------------------------------------------------
        output_dir = os.path.expanduser(
            '~/docker_simulation_ws/PETER_SIMULATION/ros2_ws/src/peter_robot/docs/resultados/pruebas'
        )

        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.csv_path = os.path.join(
            output_dir,
            f'metrics_{timestamp}.csv'
        )

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'time_s',

            'latency',
            'firing_var',
            'temp_consistency',
            'lambda_efficiency',

            'Tresponse',
            'Tswitch',
            'roll_rms',
            'pitch_rms',
            'active_noise_level',

            'rmse_ct'
        ])

        self.get_logger().info(f'Guardando CSV en: {self.csv_path}')

        # --------------------------------------------------
        # Variables
        # --------------------------------------------------

        self.latency = None
        self.firing_var = None
        self.temp_consistency = None
        self.lambda_efficiency = None

        self.Tresponse = None
        self.Tswitch = None
        self.roll_rms = None
        self.pitch_rms = None
        self.active_noise_level = None

        self.rmse_ct = None

        # --------------------------------------------------
        # Subscripciones
        # --------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            '/experiment/metrics',
            self.metrics_cb,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/Metrics',
            self.metrics2_cb,
            10
        )

        self.create_subscription(
            Float64,
            '/rmse_ct',
            self.rmse_cb,
            10
        )

        # Guardar a 5 Hz
        self.timer = self.create_timer(
            0.2,
            self.save_row
        )

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def metrics_cb(self, msg):

        if len(msg.data) >= 4:
            self.latency = msg.data[0]
            self.firing_var = msg.data[1]
            self.temp_consistency = msg.data[2]
            self.lambda_efficiency = msg.data[3]

    def metrics2_cb(self, msg):

        if len(msg.data) >= 5:
            self.Tresponse = msg.data[0]
            self.Tswitch = msg.data[1]
            self.roll_rms = msg.data[2]
            self.pitch_rms = msg.data[3]
            self.active_noise_level = msg.data[4]

    def rmse_cb(self, msg):
        self.rmse_ct = msg.data

    # --------------------------------------------------
    # Escritura CSV
    # --------------------------------------------------

    def save_row(self):

        t = self.get_clock().now().nanoseconds * 1e-9

        self.csv_writer.writerow([
            round(t, 3),

            self.latency,
            self.firing_var,
            self.temp_consistency,
            self.lambda_efficiency,

            self.Tresponse,
            self.Tswitch,
            self.roll_rms,
            self.pitch_rms,
            self.active_noise_level,

            self.rmse_ct
        ])

        self.csv_file.flush()

    def destroy_node(self):

        self.csv_file.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MetricsRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()