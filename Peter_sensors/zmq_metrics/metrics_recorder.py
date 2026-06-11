#!/usr/bin/env python3
"""
metrics_recorder.py — Agregador CSV de todas las métricas en tiempo real.

Suscripciones (ZMQ):
  /experiment/metrics  (Float32MultiArray[4]) — latency, firing_var, temp_consistency, lambda
  /Metrics             (Float32MultiArray[5]) — Tresponse, Tswitch, roll_rms, pitch_rms, noise
  /rmse_ct             (Float64)              — RMSE cross-track final

Salida:
  CSV con timestamp en ~/peter_experiments/metrics_<timestamp>.csv  (5 Hz)
"""

import argparse
import csv
import os
from datetime import datetime

from transport import (
    Node, spin,
    Float32MultiArray, Float64,
)
from topics import (
    TOPIC_EXPERIMENT_METRICS,
    TOPIC_METRICS,
    TOPIC_RMSE_CT,
)


class MetricsRecorder(Node):

    def __init__(self, output_dir=None, experiment='unknown', test=1):
        super().__init__('metrics_recorder')

        if output_dir is None:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = os.path.expanduser(f'~/peter_experiments/{experiment}_{ts}')
        os.makedirs(output_dir, exist_ok=True)

        self.csv_path = os.path.join(output_dir, 'combined_metrics.csv')

        self.csv_file   = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_s',
            'latency', 'firing_var', 'temp_consistency', 'lambda_efficiency',
            'Tresponse', 'Tswitch', 'roll_rms', 'pitch_rms', 'active_noise_level',
            'rmse_ct',
        ])

        self.get_logger().info(f'Guardando CSV en: {self.csv_path}')

        # ── State ──────────────────────────────────────────────────────────
        self.latency           = None
        self.firing_var        = None
        self.temp_consistency  = None
        self.lambda_efficiency = None

        self.Tresponse         = None
        self.Tswitch           = None
        self.roll_rms          = None
        self.pitch_rms         = None
        self.active_noise_level = None

        self.rmse_ct = None

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, TOPIC_EXPERIMENT_METRICS, self.metrics_cb,  10)
        self.create_subscription(Float32MultiArray, TOPIC_METRICS,            self.metrics2_cb, 10)
        self.create_subscription(Float64,           TOPIC_RMSE_CT,            self.rmse_cb,     10)

        self.create_timer(0.2, self.save_row)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def metrics_cb(self, msg):
        if len(msg.data) >= 4:
            self.latency           = msg.data[0]
            self.firing_var        = msg.data[1]
            self.temp_consistency  = msg.data[2]
            self.lambda_efficiency = msg.data[3]

    def metrics2_cb(self, msg):
        if len(msg.data) >= 5:
            self.Tresponse          = msg.data[0]
            self.Tswitch            = msg.data[1]
            self.roll_rms           = msg.data[2]
            self.pitch_rms          = msg.data[3]
            self.active_noise_level = msg.data[4]

    def rmse_cb(self, msg):
        self.rmse_ct = msg.data

    # ── CSV write ──────────────────────────────────────────────────────────

    def save_row(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            round(t, 3),
            self.latency, self.firing_var, self.temp_consistency, self.lambda_efficiency,
            self.Tresponse, self.Tswitch, self.roll_rms, self.pitch_rms, self.active_noise_level,
            self.rmse_ct,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Metrics Recorder')
    parser.add_argument('--output-dir',  default=None,      help='Directorio de salida (absoluto)')
    parser.add_argument('--experiment',  default='unknown',  help='Nombre del experimento')
    parser.add_argument('--test',        type=int, default=1, help='Número de prueba')
    args = parser.parse_args()

    node = MetricsRecorder(
        output_dir=args.output_dir,
        experiment=args.experiment,
        test=args.test,
    )
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
