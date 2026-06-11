#!/usr/bin/env python3
"""
imu_ina_recorder.py — Graba en CSV los datos publicados por imu_ina_node.py
                       junto con el último comando enviado al robot.

Suscripciones (ZMQ):
  /imu/data   (Imu)              — orientación (quat), accel lineal, vel angular
  /ina/data   (Float32MultiArray) — [current_A, voltage_V, power_W]
  /robot_cmd  (String)            — último carácter enviado al ESP (i , j l u o k z x c …)

Salida:
  CSV con timestamp en <output_dir>/imu_ina_<timestamp>.csv  (5 Hz)

Columna robot_cmd:
  Refleja el último comando recibido en /robot_cmd. Útil para correlacionar
  consumo de corriente con modo o dirección de movimiento.
"""

import argparse
import csv
import os
from datetime import datetime

from transport import Node, spin, Imu, Float32MultiArray, String
from topics import TOPIC_IMU, TOPIC_INA, TOPIC_ROBOT_CMD


class ImuInaRecorder(Node):

    def __init__(self, output_dir: str):
        super().__init__('imu_ina_recorder')

        os.makedirs(output_dir, exist_ok=True)

        timestamp     = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(output_dir, f'imu_ina_{timestamp}.csv')

        self.csv_file   = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_s',
            'qx', 'qy', 'qz', 'qw',
            'ax', 'ay', 'az',
            'gx', 'gy', 'gz',
            'current_A', 'voltage_V', 'power_W',
            'robot_cmd',
        ])

        self.get_logger().info(f'Guardando CSV en: {self.csv_path}')

        # ── State ──────────────────────────────────────────────────────────
        self.qx = self.qy = self.qz = self.qw = None
        self.ax = self.ay = self.az = None
        self.gx = self.gy = self.gz = None
        self.current_A = self.voltage_V = self.power_W = None
        self.robot_cmd = None

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(Imu,              TOPIC_IMU,       self.imu_cb, 10)
        self.create_subscription(Float32MultiArray, TOPIC_INA,       self.ina_cb, 10)
        self.create_subscription(String,            TOPIC_ROBOT_CMD, self.cmd_cb, 10)

        self.create_timer(0.2, self.save_row)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def imu_cb(self, msg):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z
        self.gx = msg.angular_velocity.x
        self.gy = msg.angular_velocity.y
        self.gz = msg.angular_velocity.z

    def ina_cb(self, msg):
        if len(msg.data) >= 3:
            self.current_A = msg.data[0]
            self.voltage_V = msg.data[1]
            self.power_W   = msg.data[2]

    def cmd_cb(self, msg):
        self.robot_cmd = msg.data

    # ── CSV write ──────────────────────────────────────────────────────────

    def save_row(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            round(t, 3),
            self.qx, self.qy, self.qz, self.qw,
            self.ax, self.ay, self.az,
            self.gx, self.gy, self.gz,
            self.current_A, self.voltage_V, self.power_W,
            self.robot_cmd,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='IMU + INA recorder — ZMQ → CSV')
    parser.add_argument('--output-dir', default=os.path.expanduser('~/peter_experiments/pruebas'),
                        help='Directorio de salida del CSV')
    args = parser.parse_args()

    node = ImuInaRecorder(output_dir=args.output_dir)
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
