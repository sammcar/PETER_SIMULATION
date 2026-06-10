#!/usr/bin/env python3
"""
neural_recorder.py — Captura de actividad neuronal y cálculo de métricas.

Métricas calculadas:
  - Latencia de decisión   : tiempo desde aparición del estímulo hasta primer comando motor.
  - Varianza de disparo    : varianza de las tasas de disparo en ventana deslizante.
  - Consistencia temporal  : correlación de vectores de actividad entre ciclos consecutivos.
  - Eficiencia λ           : eficiencia de resolución de conflictos entre estímulos simultáneos.

Suscripciones (ZMQ):
  neuron_activity    (Float32MultiArray) — actividades de todas las neuronas
  /cmd_vel           (Twist)             — comando de velocidad motor
  /bounding_box/red  (Float32MultiArray) — presencia estímulo rojo
  /bounding_box/blue (Float32MultiArray) — presencia estímulo azul
  /peter_mode        (String)            — modo actual del robot

Publicaciones (ZMQ):
  /experiment/metrics (Float32MultiArray) — [latency, firing_var, temp_consistency, lambda]
  /experiment/status  (String)            — resumen JSON de métricas actuales
"""

import argparse
import csv
import json
import os
import time
from collections import deque
from datetime import datetime

import numpy as np

from transport import (
    Node, spin,
    Float32MultiArray, Twist, String,
)
from topics import (
    TOPIC_NEURON_ACTIVITY,
    TOPIC_CMD_VEL,
    TOPIC_BBOX_RED,
    TOPIC_BBOX_BLUE,
    TOPIC_PETER_MODE,
    TOPIC_EXPERIMENT_METRICS,
    TOPIC_EXPERIMENT_STATUS,
)


class NeuralRecorder(Node):

    def __init__(self, experiment_type='unknown', window_size=50,
                 output_dir=None, save_csv=True):
        super().__init__('neural_recorder')

        self._exp_type        = experiment_type
        self._window          = window_size
        self._explicit_outdir = output_dir is not None
        self._output_dir      = output_dir or os.path.expanduser('~/peter_experiments')
        self._save_csv        = save_csv

        # ── Sliding buffers ────────────────────────────────────────────────
        self._activity_buffer  = deque(maxlen=self._window)
        self._cmd_vel_buffer   = deque(maxlen=self._window)
        self._timestamp_buffer = deque(maxlen=self._window)

        # ── Stimulus state ─────────────────────────────────────────────────
        self._stimulus_appeared_time     = None
        self._first_cmd_after_stimulus   = False
        self._latency_history            = []
        self._red_present                = False
        self._blue_present               = False
        self._prev_activity              = None
        self._mode                       = 'C'

        # ── Accumulated metrics ────────────────────────────────────────────
        self._firing_variance      = 0.0
        self._temporal_consistency = 0.0
        self._lambda_efficiency    = 0.0
        self._last_latency         = float('nan')

        # ── CSV logging ────────────────────────────────────────────────────
        if self._save_csv:
            if self._explicit_outdir:
                session_dir = self._output_dir
            else:
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                session_dir = os.path.join(self._output_dir, f'{self._exp_type}_{ts}')
            os.makedirs(session_dir, exist_ok=True)
            csv_path = os.path.join(session_dir, 'neural_metrics.csv')
            self._csv_file   = open(csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'sim_time_s', 'latency_s', 'firing_variance',
                'temporal_consistency', 'lambda_efficiency',
                'mode', 'red_present', 'blue_present',
            ])
            self.get_logger().info(f'[NeuralRecorder] CSV: {csv_path}')
        else:
            self._csv_writer = None
            self._csv_file   = None

        # ── Subscriptions ──────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, TOPIC_NEURON_ACTIVITY, self._cb_activity, 50)
        self.create_subscription(Twist,             TOPIC_CMD_VEL,         self._cb_cmd_vel,  50)
        self.create_subscription(Float32MultiArray, TOPIC_BBOX_RED,        self._cb_red,     100)
        self.create_subscription(Float32MultiArray, TOPIC_BBOX_BLUE,       self._cb_blue,    100)
        self.create_subscription(String,            TOPIC_PETER_MODE,      self._cb_mode,     10)

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_metrics = self.create_publisher(Float32MultiArray, TOPIC_EXPERIMENT_METRICS, 10)
        self._pub_status  = self.create_publisher(String,            TOPIC_EXPERIMENT_STATUS,  10)

        self.create_timer(0.2, self._publish_metrics)

        self.get_logger().info(f'[NeuralRecorder] Iniciado — experimento: {self._exp_type}')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cb_activity(self, msg):
        arr = np.array(msg.data, dtype=np.float32)
        self._activity_buffer.append(arr)
        self._timestamp_buffer.append(time.time())
        self._prev_activity = arr

    def _cb_cmd_vel(self, msg):
        lin = msg.linear.x
        lat = msg.linear.y
        ang = msg.angular.z
        mag = abs(lin) + abs(lat) + abs(ang)
        self._cmd_vel_buffer.append(mag)

        if self._stimulus_appeared_time is not None and not self._first_cmd_after_stimulus:
            if mag > 0.01:
                latency = time.time() - self._stimulus_appeared_time
                self._latency_history.append(latency)
                self._last_latency = latency
                self._first_cmd_after_stimulus = True
                self.get_logger().info(f'[NeuralRecorder] Latencia de decisión: {latency:.4f} s')

    def _cb_red(self, msg):
        if len(msg.data) >= 2:
            was_present    = self._red_present
            self._red_present = msg.data[1] > 1.0
            if self._red_present and not was_present:
                self._on_stimulus_appeared()

    def _cb_blue(self, msg):
        if len(msg.data) >= 2:
            was_present       = self._blue_present
            self._blue_present = msg.data[1] > 1.0
            if self._blue_present and not was_present:
                self._on_stimulus_appeared()

    def _cb_mode(self, msg):
        self._mode = msg.data

    # ── Helpers ────────────────────────────────────────────────────────────

    def _on_stimulus_appeared(self):
        self._stimulus_appeared_time   = time.time()
        self._first_cmd_after_stimulus = False

    def _compute_firing_variance(self):
        if len(self._activity_buffer) < 2:
            return 0.0
        matrix = np.vstack(self._activity_buffer)
        return float(np.mean(np.var(matrix, axis=0)))

    def _compute_temporal_consistency(self):
        if len(self._activity_buffer) < 2:
            return 0.0
        buf = list(self._activity_buffer)
        correlations = []
        for i in range(1, len(buf)):
            a, b = buf[i - 1], buf[i]
            na, nb = np.linalg.norm(a), np.linalg.norm(b)
            if na < 1e-9 or nb < 1e-9:
                correlations.append(0.0)
            else:
                correlations.append(float(np.dot(a, b) / (na * nb)))
        return float(np.mean(correlations)) if correlations else 0.0

    def _compute_lambda_efficiency(self):
        if not self._cmd_vel_buffer or not self._activity_buffer:
            return 0.0
        if not (self._red_present and self._blue_present):
            return 1.0
        total_intensity = float(np.linalg.norm(np.array(self._activity_buffer[-1])))
        if total_intensity < 1e-9:
            return 0.0
        return min(1.0, float(self._cmd_vel_buffer[-1]) / (total_intensity + 1e-9))

    # ── Periodic publish ───────────────────────────────────────────────────

    def _publish_metrics(self):
        self._firing_variance      = self._compute_firing_variance()
        self._temporal_consistency = self._compute_temporal_consistency()
        self._lambda_efficiency    = self._compute_lambda_efficiency()

        now_s = self.get_clock().now().nanoseconds * 1e-9

        metrics_msg      = Float32MultiArray()
        lat_val          = float(self._last_latency) if not np.isnan(self._last_latency) else -1.0
        metrics_msg.data = [lat_val, float(self._firing_variance),
                            float(self._temporal_consistency), float(self._lambda_efficiency)]
        self._pub_metrics.publish(metrics_msg)

        status = {
            'exp':                self._exp_type,
            'time':               round(now_s, 2),
            'latency_s':          round(self._last_latency, 4) if not np.isnan(self._last_latency) else None,
            'latency_mean_s':     round(float(np.mean(self._latency_history)), 4) if self._latency_history else None,
            'firing_variance':    round(self._firing_variance, 6),
            'temporal_consistency': round(self._temporal_consistency, 4),
            'lambda_efficiency':  round(self._lambda_efficiency, 4),
            'mode':               self._mode,
            'red_present':        self._red_present,
            'blue_present':       self._blue_present,
        }
        status_msg      = String()
        status_msg.data = json.dumps(status)
        self._pub_status.publish(status_msg)

        if self._csv_writer is not None:
            self._csv_writer.writerow([
                round(now_s, 3),
                round(self._last_latency, 4) if not np.isnan(self._last_latency) else '',
                round(self._firing_variance, 6),
                round(self._temporal_consistency, 4),
                round(self._lambda_efficiency, 4),
                self._mode,
                int(self._red_present),
                int(self._blue_present),
            ])
            self._csv_file.flush()

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Neural Recorder')
    parser.add_argument('--experiment-type', default='unknown')
    parser.add_argument('--window-size',     type=int, default=50)
    parser.add_argument('--output-dir',      default=os.path.expanduser('~/peter_experiments'))
    parser.add_argument('--no-csv',          action='store_true')
    args = parser.parse_args()

    node = NeuralRecorder(
        experiment_type=args.experiment_type,
        window_size=args.window_size,
        output_dir=args.output_dir,
        save_csv=not args.no_csv,
    )
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
