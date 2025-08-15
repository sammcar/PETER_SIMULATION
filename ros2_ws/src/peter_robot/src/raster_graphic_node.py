#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time

# ==== Booleans de control ====
PLOT_GANGLIOS_BASALES = False
PLOT_LOCOMOCION = True
PLOT_DECISION_MARCHA = True
PLOT_RED_LIDAR = False

class NeuronPlotter(Node):
    def __init__(self):
        super().__init__('neuron_plotter')
        # en __init__
        self.imu_series = None   # se inicializa al primer mensaje
        self.imu_samples = 0     # contador de muestras recibidas
        self.IMU_FREQ_HZ = 10.0  # <<< ajusta a la frecuencia real del tópico 'imu_activity'


        self.data_matrix = []  # Guardará la historia temporal
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'neuron_activity',
            self.listener_callback,
            10
        )

        self.subscription_imu = self.create_subscription(
            Float32MultiArray,
            'imu_activity',
            self.listener_callback_imu,
            10
        )

        self.get_logger().info("🧠 Nodo plotter iniciado. Presiona Enter para mostrar gráficas.")

    def listener_callback_imu(self, msg):
        # Inicializa estructura al primer mensaje
        if self.imu_series is None:
            n = len(msg.data)
            self.imu_series = [ [] for _ in range(n) ]

        # Tamaño inconsistente -> descartar
        if len(msg.data) != len(self.imu_series):
            self.get_logger().warn(f"⚠️ IMU con {len(msg.data)} elementos, esperado {len(self.imu_series)}; se ignora este mensaje.")
            return

        # Acumula valores
        for i, v in enumerate(msg.data):
            self.imu_series[i].append(float(v))
        self.imu_samples += 1

    def listener_callback(self, msg):
        if len(msg.data) == 85:
            self.data_matrix.append(msg.data)
        else:
            self.get_logger().warn(f"⚠️ Mensaje recibido con {len(msg.data)} elementos (esperado: 85)")

    def plot_raster(self):
        if not self.data_matrix:
            self.get_logger().warn("No se recibieron datos.")
            return

        data_array = np.array(self.data_matrix).T  # Filas = neuronas, columnas = tiempo

        # ==== Función auxiliar para plotear submódulos ====
        # ==== Frecuencia de muestreo (Hz) ====

        FREQ_HZ = 10.0  # <-- pon aquí la frecuencia real de publicación del tópico

        def plot_module(submodules, module_title, custom_labels=False):
            fig, axs = plt.subplots(len(submodules), 1, figsize=(10, 2.5*len(submodules)))
            fig.suptitle(module_title)
            if len(submodules) == 1:
                axs = [axs]  # Asegurar lista si solo hay un subplot

            num_samples = data_array.shape[1]
            time_axis = np.arange(num_samples) / FREQ_HZ  # tiempo en segundos

            for i, (start, end, labels, name) in enumerate(submodules):
                raster = data_array[start:end, :]  # Filas = neuronas del rango

                im = axs[i].imshow(
                    raster,
                    aspect='auto',
                    cmap='gist_yarg',
                    interpolation='nearest',
                    vmin=0,
                    vmax=np.max(raster) if raster.size else 1.0,
                    extent=[time_axis[0], time_axis[-1], 0, raster.shape[0]]  # eje X en segundos
                )

                # Eje Y: etiquetas de neuronas
                if labels:
                    axs[i].set_yticks(range(len(labels)))
                    axs[i].set_yticklabels(labels)
                else:
                    axs[i].set_yticks(range(end - start))
                    axs[i].set_yticklabels([str(n+1) for n in range(start, end)])

                axs[i].set_title(name, fontsize=10)
                axs[i].set_ylabel("Neurona")
                axs[i].set_xlabel("Tiempo (s)")
                fig.colorbar(im, ax=axs[i], orientation='vertical')

            plt.tight_layout()


        # ==== Módulo 1: Ganglios Basales ====
        if PLOT_GANGLIOS_BASALES:
            submodules = [
                (0, 3, ['B','G','R'], "GPi"),
                (3, 6, ['B','G','R'], "GPe"),
                (6, 9, ['B','G','R'], "STN"),
                (9, 12, ['B','G','R'], "STR"),
            ]
            plot_module(submodules, "Módulo Ganglios Basales")

        # ==== Módulo 2: Locomoción ====
        if PLOT_LOCOMOCION:
            submodules = [
                (15, 26, [f"X{i}" for i in range(3, 14)], "Locomoción 1"),
                (29, 30, ["X17"], "Locomoción 2"),
            ]
            plot_module(submodules, "Módulo Locomoción")


        # ==== Módulo 3: Decisión de Marcha ====
        if PLOT_DECISION_MARCHA:
            submodules = [
                (12, 17, [f"X{i}" for i in range(0, 5)], "Decisión 1"),
                (26, 29, [f"X{i}" for i in range(14, 17)], "Decisión 2"),
            ]
            plot_module(submodules, "Módulo Decisión de Marcha")

        # ==== Módulo 4: Red Lidar ====
        if PLOT_RED_LIDAR:
            submodules = [
                (32, 37, [f"L{i}" for i in range(5)], "Lidar"),
                (37, 53, [str(i) for i in range(1,17)], "Input"),
                (53, 69, [str(i) for i in range(1,17)], "Response"),
                (69, 85, [str(i) for i in range(1,17)], "Auxiliar"),
            ]
            plot_module(submodules, "Módulo Red Lidar")

        if IMU and self.imu_series is not None and self.imu_samples > 0:
            # Eje X a partir de la frecuencia de muestreo del suscriptor IMU
            t_imu = np.arange(self.imu_samples) / self.IMU_FREQ_HZ  # tiempo en segundos

            plt.figure(figsize=(10, 6))
            for i, serie in enumerate(self.imu_series):
                # Asegura longitud coherente si alguna lista quedó desfasada
                m = min(len(serie), len(t_imu))
                if m > 0:
                    plt.plot(t_imu[:m], serie[:m], label=f'IMU {i}')
            plt.xlabel('Tiempo (s)')
            plt.ylabel('Valor')
            plt.title('IMU activity')
            plt.legend(loc='upper right')
            plt.grid(True)


        plt.show()


def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = NeuronPlotter()

    thread = Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()

    try:
        while True:
            input("🔁 Presiona Enter para mostrar gráficas...\n")
            node.plot_raster()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()
