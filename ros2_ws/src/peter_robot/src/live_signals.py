#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import matplotlib.pyplot as plt
from collections import deque

MAX_DATA =25

class NeuronSignalPlotter(Node):
    def __init__(self):
        super().__init__('neuron_signal_plotter')

        self.time_data = deque(maxlen=MAX_DATA)
        self.gpe_data = [deque(maxlen=MAX_DATA) for _ in range(3)]
        self.dec1_data = [deque(maxlen=MAX_DATA) for _ in range(5)]
        self.dec2_data = [deque(maxlen=MAX_DATA) for _ in range(3)]

        self.current_mode = "Sin datos"
        self.t_counter = 0

        self.sub_activity = self.create_subscription(Float32MultiArray, 'neuron_activity', self.activity_callback, 10)
        self.sub_mode = self.create_subscription(String, '/peter_mode', self.mode_callback, 10)

        plt.ion()
        self.fig, (self.ax_gpe, self.ax_dec1, self.ax_dec2, self.ax_mode) = plt.subplots(4, 1, figsize=(8, 10))
        self.fig.tight_layout(pad=3.0)

    def activity_callback(self, msg):
        self.t_counter += 1
        self.time_data.append(self.t_counter)

        self.gpe_data[0].append(msg.data[3])
        self.gpe_data[1].append(msg.data[4])
        self.gpe_data[2].append(msg.data[5])

        for i, idx in enumerate(range(12, 17)):
            self.dec1_data[i].append(msg.data[idx])

        for i, idx in enumerate(range(26, 29)):
            self.dec2_data[i].append(msg.data[idx])

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def update_plot(self):
        # GPe
        self.ax_gpe.clear()
        self.ax_gpe.plot(self.time_data, self.gpe_data[0], 'r-', label='GPe R')
        self.ax_gpe.plot(self.time_data, self.gpe_data[1], 'g-', label='GPe G')
        self.ax_gpe.plot(self.time_data, self.gpe_data[2], 'b-', label='GPe B')
        self.ax_gpe.set_title("GPe")
        self.ax_gpe.legend(loc='upper right')

        # Autoajuste de límites Y
        all_gpe = list(self.gpe_data[0]) + list(self.gpe_data[1]) + list(self.gpe_data[2])
        if all_gpe:
            ymin, ymax = min(all_gpe), max(all_gpe)
            padding = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
            self.ax_gpe.set_ylim(ymin - padding, ymax + padding)

        # Decisión 1
        self.ax_dec1.clear()
        for i in range(5):
            self.ax_dec1.plot(self.time_data, self.dec1_data[i], label=f"Neuron {i}")
        self.ax_dec1.set_title("Decisión 1")
        self.ax_dec1.legend(loc='upper right')

        all_dec1 = [v for dq in self.dec1_data for v in dq]
        if all_dec1:
            ymin, ymax = min(all_dec1), max(all_dec1)
            padding = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
            self.ax_dec1.set_ylim(ymin - padding, ymax + padding)

        # Decisión 2
        self.ax_dec2.clear()
        for i in range(3):
            self.ax_dec2.plot(self.time_data, self.dec2_data[i], label=f"Neuron {14+i}")
        self.ax_dec2.set_title("Decisión 2")
        self.ax_dec2.legend(loc='upper right')

        all_dec2 = [v for dq in self.dec2_data for v in dq]
        if all_dec2:
            ymin, ymax = min(all_dec2), max(all_dec2)
            padding = (ymax - ymin) * 0.1 if ymax != ymin else 0.1
            self.ax_dec2.set_ylim(ymin - padding, ymax + padding)

        # Modo actual
        self.ax_mode.clear()
        self.ax_mode.text(0.5, 0.5, f"Modo: {self.current_mode}", fontsize=14, ha='center', va='center')
        self.ax_mode.axis('off')

        plt.pause(0.001)


def main():
    rclpy.init()
    node = NeuronSignalPlotter()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
