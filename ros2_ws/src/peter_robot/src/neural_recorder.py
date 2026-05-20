#!/usr/bin/env python3
"""
neural_recorder.py
Nodo de instrumentación para captura de actividad neuronal y cálculo de métricas.

Métricas calculadas:
  - Latencia de decisión   : tiempo desde aparición de estímulo hasta primer comando motor.
  - Varianza de disparo    : varianza de las tasas de disparo neuronal en ventana deslizante.
  - Consistencia temporal  : correlación de vectores de actividad entre ciclos consecutivos.
  - Eficiencia λ           : eficiencia de resolución de conflictos entre estímulos simultáneos.

Suscripciones:
  /neuron_activity    (Float32MultiArray) — actividades de todas las neuronas
  /cmd_vel            (Twist)             — comando de velocidad motor
  /bounding_box/red   (Float32MultiArray) — presencia estímulo rojo
  /bounding_box/blue  (Float32MultiArray) — presencia estímulo azul
  /peter_mode         (String)            — modo actual del robot

Publicaciones:
  /experiment/metrics (Float32MultiArray) — [latency, firing_var, temp_consistency, lambda]
  /experiment/status  (String)            — resumen JSON de métricas actuales

Los datos se guardan en ~/peter_experiments/<timestamp>/ en formato CSV.
"""

import csv
import json
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class NeuralRecorder(Node):

    def __init__(self):
        super().__init__('neural_recorder')

        # ---- Parámetros ----
        self.declare_parameter('experiment_type', 'unknown')
        self.declare_parameter('window_size', 50)
        self.declare_parameter('output_dir', os.path.expanduser('~/peter_experiments'))
        self.declare_parameter('save_csv', True)

        self._exp_type = self.get_parameter('experiment_type').get_parameter_value().string_value
        self._window = self.get_parameter('window_size').get_parameter_value().integer_value
        self._output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._save_csv = self.get_parameter('save_csv').get_parameter_value().bool_value

        # ---- Buffers deslizantes ----
        self._activity_buffer: deque = deque(maxlen=self._window)
        self._cmd_vel_buffer: deque = deque(maxlen=self._window)
        self._timestamp_buffer: deque = deque(maxlen=self._window)

        # ---- Estado de estímulos ----
        self._stimulus_appeared_time: float = None  # instante en que llegó el estímulo
        self._first_cmd_after_stimulus: bool = False  # si ya se registró latencia
        self._latency_history: list = []
        self._red_present: bool = False
        self._blue_present: bool = False
        self._prev_activity: np.ndarray = None
        self._mode: str = 'C'

        # ---- Métricas acumuladas ----
        self._firing_variance: float = 0.0
        self._temporal_consistency: float = 0.0
        self._lambda_efficiency: float = 0.0
        self._last_latency: float = float('nan')

        # ---- CSV logging ----
        if self._save_csv:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._session_dir = os.path.join(
                self._output_dir, f'{self._exp_type}_{ts}'
            )
            os.makedirs(self._session_dir, exist_ok=True)
            csv_path = os.path.join(self._session_dir, 'metrics.csv')
            self._csv_file = open(csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'sim_time_s',
                'latency_s',
                'firing_variance',
                'temporal_consistency',
                'lambda_efficiency',
                'mode',
                'red_present',
                'blue_present',
            ])
            self.get_logger().info(f'[NeuralRecorder] CSV: {csv_path}')
        else:
            self._csv_writer = None
            self._csv_file = None

        # ---- Suscriptores ----
        self.create_subscription(
            Float32MultiArray, 'neuron_activity', self._cb_activity, 50
        )
        self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, 50
        )
        self.create_subscription(
            Float32MultiArray, '/bounding_box/red', self._cb_red, 100
        )
        self.create_subscription(
            Float32MultiArray, '/bounding_box/blue', self._cb_blue, 100
        )
        self.create_subscription(
            String, '/peter_mode', self._cb_mode, 10
        )

        # ---- Publicadores ----
        self._pub_metrics = self.create_publisher(Float32MultiArray, '/experiment/metrics', 10)
        self._pub_status = self.create_publisher(String, '/experiment/status', 10)

        # ---- Timer de publicación (5 Hz) ----
        self._timer = self.create_timer(0.2, self._publish_metrics)

        self.get_logger().info(
            f'[NeuralRecorder] Iniciado — experimento: {self._exp_type}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_activity(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        now = time.time()
        self._activity_buffer.append(arr)
        self._timestamp_buffer.append(now)
        self._prev_activity = arr

    def _cb_cmd_vel(self, msg: Twist):
        lin = msg.linear.x
        lat = msg.linear.y
        ang = msg.angular.z
        mag = abs(lin) + abs(lat) + abs(ang)
        self._cmd_vel_buffer.append(mag)

        # Medir latencia: si hubo estímulo y aún no se registró el primer comando
        if self._stimulus_appeared_time is not None and not self._first_cmd_after_stimulus:
            if mag > 0.01:
                latency = time.time() - self._stimulus_appeared_time
                self._latency_history.append(latency)
                self._last_latency = latency
                self._first_cmd_after_stimulus = True
                self.get_logger().info(
                    f'[NeuralRecorder] Latencia de decisión: {latency:.4f} s'
                )

    def _cb_red(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            area = msg.data[1]
            was_present = self._red_present
            self._red_present = area > 1.0
            if self._red_present and not was_present:
                self._on_stimulus_appeared()

    def _cb_blue(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            area = msg.data[1]
            was_present = self._blue_present
            self._blue_present = area > 1.0
            if self._blue_present and not was_present:
                self._on_stimulus_appeared()

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _on_stimulus_appeared(self):
        """Reinicia el temporizador de latencia cuando aparece un nuevo estímulo."""
        self._stimulus_appeared_time = time.time()
        self._first_cmd_after_stimulus = False

    def _compute_firing_variance(self) -> float:
        """
        Varianza de disparo: promedio de la varianza por neurona en la ventana temporal.
        Con N muestras de actividad (cada una es un vector de neuronas),
        calculamos la varianza temporal de cada neurona y promediamos.
        """
        if len(self._activity_buffer) < 2:
            return 0.0
        matrix = np.vstack(self._activity_buffer)  # shape (T, N_neurons)
        per_neuron_var = np.var(matrix, axis=0)
        return float(np.mean(per_neuron_var))

    def _compute_temporal_consistency(self) -> float:
        """
        Consistencia temporal: correlación media entre vectores de actividad
        en ciclos consecutivos dentro de la ventana.

        Valor ∈ [-1, 1]:  1.0 = perfectamente consistente.
        """
        if len(self._activity_buffer) < 2:
            return 0.0
        buf = list(self._activity_buffer)
        correlations = []
        for i in range(1, len(buf)):
            a = buf[i - 1]
            b = buf[i]
            norm_a = np.linalg.norm(a)
            norm_b = np.linalg.norm(b)
            if norm_a < 1e-9 or norm_b < 1e-9:
                correlations.append(0.0)
            else:
                correlations.append(float(np.dot(a, b) / (norm_a * norm_b)))
        return float(np.mean(correlations)) if correlations else 0.0

    def _compute_lambda_efficiency(self) -> float:
        """
        Eficiencia de resolución de conflictos λ.

        Definición operativa:
          λ = (decisión tomada) / (intensidad total de estímulos en conflicto)

        - Si solo hay un estímulo activo: λ = 1.0 (no hay conflicto).
        - Si hay dos estímulos simultáneos: λ = cmd_mag / (R_intensity + B_intensity).
          Valores altos (→1) indican que el sistema tomó una decisión clara y eficiente
          a pesar del conflicto; valores bajos indican parálisis o indecisión.
        - Normalizado a [0, 1].
        """
        if len(self._cmd_vel_buffer) == 0 or len(self._activity_buffer) == 0:
            return 0.0

        both_present = self._red_present and self._blue_present
        if not both_present:
            return 1.0  # sin conflicto

        # Intensidad de estímulo = norma del vector de actividad reciente
        recent_activity = np.array(self._activity_buffer[-1], dtype=np.float32)
        total_intensity = float(np.linalg.norm(recent_activity))
        if total_intensity < 1e-9:
            return 0.0

        cmd_mag = float(self._cmd_vel_buffer[-1])
        lam = min(1.0, cmd_mag / (total_intensity + 1e-9))
        return lam

    # ------------------------------------------------------------------
    # Publicación periódica
    # ------------------------------------------------------------------

    def _publish_metrics(self):
        self._firing_variance = self._compute_firing_variance()
        self._temporal_consistency = self._compute_temporal_consistency()
        self._lambda_efficiency = self._compute_lambda_efficiency()

        now_s = self.get_clock().now().nanoseconds * 1e-9

        # Float32MultiArray: [latency, firing_var, temp_consistency, lambda]
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            float(self._last_latency) if not np.isnan(self._last_latency) else -1.0,
            float(self._firing_variance),
            float(self._temporal_consistency),
            float(self._lambda_efficiency),
        ]
        self._pub_metrics.publish(metrics_msg)

        # JSON status
        status = {
            'exp': self._exp_type,
            'sim_time': round(now_s, 2),
            'latency_s': round(self._last_latency, 4)
            if not np.isnan(self._last_latency) else None,
            'latency_mean_s': round(float(np.mean(self._latency_history)), 4)
            if self._latency_history else None,
            'firing_variance': round(self._firing_variance, 6),
            'temporal_consistency': round(self._temporal_consistency, 4),
            'lambda_efficiency': round(self._lambda_efficiency, 4),
            'mode': self._mode,
            'red_present': self._red_present,
            'blue_present': self._blue_present,
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self._pub_status.publish(status_msg)

        # CSV row
        if self._csv_writer is not None:
            self._csv_writer.writerow([
                round(now_s, 3),
                round(self._last_latency, 4)
                if not np.isnan(self._last_latency) else '',
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


def main(args=None):
    rclpy.init(args=args)
    node = NeuralRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
