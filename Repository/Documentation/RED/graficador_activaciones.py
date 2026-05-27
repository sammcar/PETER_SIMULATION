import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class GraficaActivaciones(Node):
    def __init__(self):
        super().__init__('graficador_activaciones')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/activaciones_neuronales',
            self.activaciones_callback,
            10)
        
        # Número total de neuronas y configuración de cuadrantes
        self.n_neuronas = 60
        self.n_por_cuadrante = self.n_neuronas // 4
        
        # Definir direcciones preferidas en 4 cuadrantes
        cuadrante_1 = np.linspace(0, 90, self.n_por_cuadrante, endpoint=False)
        cuadrante_2 = np.linspace(90, 180, self.n_por_cuadrante, endpoint=False)
        cuadrante_3 = np.linspace(180, 270, self.n_por_cuadrante, endpoint=False)
        cuadrante_4 = np.linspace(270, 360, self.n_por_cuadrante, endpoint=False)
        
        self.Directions_deg = np.concatenate([cuadrante_1, cuadrante_2, cuadrante_3, cuadrante_4])

        # Configuración de la gráfica
        plt.ion()  # Modo interactivo
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.barras = self.ax.bar(self.Directions_deg, np.zeros(self.n_neuronas), color=['r']*15 + ['g']*15 + ['b']*15 + ['purple']*15)
        
        self.ax.set_ylim(0, 1)  # Límite de activación
        self.ax.set_xlabel('Dirección preferida (°)')
        self.ax.set_ylabel('Nivel de activación')
        self.ax.set_title('Activaciones de neuronas por cuadrante')

    def activaciones_callback(self, msg):
        activaciones = np.array(msg.data)
        
        # Actualizar las barras de la gráfica
        for i, bar in enumerate(self.barras):
            bar.set_height(activaciones[i])

        plt.draw()
        plt.pause(0.1)  # Pausa para actualizar la gráfica


def main(args=None):
    rclpy.init(args=args)
    node = GraficaActivaciones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
