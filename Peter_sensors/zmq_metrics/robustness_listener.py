#!/usr/bin/env python3
"""
robustness_listener.py — Calcula el Índice de Robustez (RI).

Recoge el RMSE cross-track bajo 3 niveles de ruido y calcula:
  RI = 1 - (((RMSE1 - RMSE0) + (RMSE2 - RMSE0)) / (2 * RMSE0))

Suscripciones (ZMQ):
  /rmse_ct  (Float64)           — RMSE final de cada prueba
  /Metrics  (Float32MultiArray) — índice del nivel de ruido en data[4]

Nota: el RI requiere 3 pruebas independientes (ruido 0, 1 y 2).
      Ejecutar este nodo durante toda la sesión de pruebas.
"""

from transport import Node, spin, Float64, Float32MultiArray
from topics import TOPIC_RMSE_CT, TOPIC_METRICS


class RIListener(Node):

    def __init__(self):
        super().__init__('ri_listener')

        self.rmse_0 = None
        self.rmse_1 = None
        self.rmse_2 = None
        self.current_noise_idx = None

        self.create_subscription(Float64,           TOPIC_RMSE_CT, self.rmse_callback,    10)
        self.create_subscription(Float32MultiArray, TOPIC_METRICS, self.metrics_callback, 10)

        self.create_timer(0.5, self.compute_ri)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def metrics_callback(self, msg):
        try:
            self.current_noise_idx = int(msg.data[4])
            self.get_logger().info(f"Nivel de ruido recibido: {self.current_noise_idx}")
        except Exception as e:
            self.get_logger().error(f"Error leyendo {TOPIC_METRICS}: {e}")

    def rmse_callback(self, msg):
        if self.current_noise_idx is None:
            self.get_logger().warn("Todavía no se ha recibido el nivel de ruido")
            return

        rmse = msg.data
        if self.current_noise_idx == 0:
            self.rmse_0 = rmse
            self.get_logger().info(f"RMSE ruido 0 guardado: {rmse:.4f}")
        elif self.current_noise_idx == 1:
            self.rmse_1 = rmse
            self.get_logger().info(f"RMSE ruido 1 guardado: {rmse:.4f}")
        elif self.current_noise_idx == 2:
            self.rmse_2 = rmse
            self.get_logger().info(f"RMSE ruido 2 guardado: {rmse:.4f}")

    # ── RI computation ─────────────────────────────────────────────────────

    def compute_ri(self):
        if None in (self.rmse_0, self.rmse_1, self.rmse_2):
            return

        if self.rmse_0 == 0.0:
            self.get_logger().error("RMSE base es 0, no se puede calcular RI")
            return

        ri = 1.0 - (
            ((self.rmse_1 - self.rmse_0) + (self.rmse_2 - self.rmse_0))
            / (2.0 * self.rmse_0)
        )

        self.get_logger().info(
            f"\nRMSE0={self.rmse_0:.3f}"
            f"\nRMSE1={self.rmse_1:.3f}"
            f"\nRMSE2={self.rmse_2:.3f}"
            f"\nRI   ={ri:.3f}"
        )


def main():
    node = RIListener()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
