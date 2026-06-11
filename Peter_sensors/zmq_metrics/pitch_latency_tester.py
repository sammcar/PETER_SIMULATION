#!/usr/bin/env python3
import csv
import math
import time
from datetime import datetime

from transport import Node, spin, Imu, String
from topics import TOPIC_IMU, TOPIC_PETER_MODE


class PitchLatencyTester(Node):
    def __init__(self, repetitions=25, high_value=12.0, low_value=0.0, hold_time=2.0, timeout=10.0):
        super().__init__("pitch_latency_tester")

        self.pub = self.create_publisher(Imu, TOPIC_IMU, 10)

        self.repetitions = repetitions
        self.high_value  = float(high_value)
        self.low_value   = float(low_value)
        self.hold_time   = float(hold_time)
        self.timeout     = float(timeout)

        self.trial_index = 0
        self.pending_high = True
        self.phase = "start"
        self.phase_start = time.time()

        self.current_mode = None

        self.waiting_for_change = False
        self.stimulus_time = None

        self.latencies = []

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"pitch_latency_{ts}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["trial", "stimulus", "latency_s", "status"])

        self.create_subscription(String, TOPIC_PETER_MODE, self._mode_cb, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Iniciado. CSV: {self.csv_path}")

    def _mode_cb(self, msg):
        mode = msg.data
        if self.current_mode is None:
            self.current_mode = mode
            return

        if self.waiting_for_change and mode != self.current_mode:
            latency = time.time() - self.stimulus_time
            self.latencies.append(latency)
            self.csv_writer.writerow([
                self.trial_index + 1,
                self.high_value if self.pending_high else self.low_value,
                round(latency, 6),
                "OK",
            ])
            self.csv_file.flush()
            self.get_logger().info(f"Ensayo {self.trial_index + 1}: latencia {latency:.4f} s")
            self.waiting_for_change = False
            self.phase = "hold"
            self.phase_start = time.time()

        self.current_mode = mode

    def _publish_value(self, value):
        # Codifica pitch_degrees como cuaternión de rotación pura en Y:
        #   qw=cos(θ/2), qy=sin(θ/2), qx=qz=0
        # imu_callback calcula: pitch = abs(degrees(arcsin(2*(qw*qy - qz*qx)))) = abs(value)
        # y roll = 180 - arctan2(0, 1) = 180° (robot plano)
        theta = math.radians(float(value))
        msg = Imu()
        msg.orientation.w = math.cos(theta / 2)
        msg.orientation.x = 0.0
        msg.orientation.y = math.sin(theta / 2)
        msg.orientation.z = 0.0
        self.pub.publish(msg)
        self.stimulus_time = time.time()
        self.waiting_for_change = True

    def _tick(self):
        now = time.time()

        if self.trial_index >= self.repetitions:
            self._finish()
            return

        if self.phase == "start":
            value = self.high_value if self.pending_high else self.low_value
            self._publish_value(value)
            self.phase = "wait"
            self.get_logger().info(
                f"Ensayo {self.trial_index + 1}/{self.repetitions}: publicado {value}"
            )

        elif self.phase == "wait":
            if self.waiting_for_change and (now - self.stimulus_time) > self.timeout:
                self.csv_writer.writerow([
                    self.trial_index + 1,
                    self.high_value if self.pending_high else self.low_value,
                    "",
                    "TIMEOUT",
                ])
                self.csv_file.flush()
                self.get_logger().warning(f"Ensayo {self.trial_index + 1}: timeout")
                self._advance_trial()

        elif self.phase == "hold":
            if now - self.phase_start >= self.hold_time:
                self._advance_trial()

    def _advance_trial(self):
        self.pending_high = not self.pending_high
        self.trial_index += 1
        self.phase = "start"
        self.phase_start = time.time()
        self.waiting_for_change = False

    def _finish(self):
        if self.latencies:
            avg = sum(self.latencies) / len(self.latencies)
            self.get_logger().info(
                f"Finalizado. Latencia media: {avg:.4f} s en {len(self.latencies)} ensayos válidos"
            )
        else:
            self.get_logger().warning("Finalizado sin latencias válidas.")
        self.destroy_node()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    node = PitchLatencyTester(
        repetitions=25,
        high_value=12.0,
        low_value=0.0,
        hold_time=10.0,
        timeout=10.0,
    )
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
