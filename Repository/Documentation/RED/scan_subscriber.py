import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, '/activaciones_neuronales', 10)
        
        # Número total de neuronas y configuración de cuadrantes
        self.n_neuronas = 60  
        self.n_por_cuadrante = self.n_neuronas // 4  
        
        # Definir direcciones preferidas en 4 cuadrantes
        cuadrante_1 = np.linspace(0, 90, self.n_por_cuadrante, endpoint=False)
        cuadrante_2 = np.linspace(90, 180, self.n_por_cuadrante, endpoint=False)
        cuadrante_3 = np.linspace(180, 270, self.n_por_cuadrante, endpoint=False)
        cuadrante_4 = np.linspace(270, 360, self.n_por_cuadrante, endpoint=False)
        
        # Unir todas las direcciones y convertir a radianes
        self.Directions_deg = np.concatenate([cuadrante_1, cuadrante_2, cuadrante_3, cuadrante_4])
        self.Directions_rad = np.radians(self.Directions_deg)
        
        # Convertir a vectores unitarios
        self.Om = np.vstack((np.cos(self.Directions_rad), np.sin(self.Directions_rad)))
        
        # Parámetros de la gaussiana
        self.sigma = 0.06  
        self.umbral = 0.95  

    def gausiana(self, theta, omega):
        return np.exp((np.dot(theta, omega) - 1) / (2 * (self.sigma**2)))

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        activaciones_totales = np.zeros(self.n_neuronas)  
        detected = False  

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment  

            if np.isnan(r) or np.isinf(r):
                continue  

            detected = True  
            vector_input = np.array([np.cos(angle), np.sin(angle)])
            
            activaciones = np.array([self.gausiana(vector_input, self.Om[:, j]) for j in range(self.n_neuronas)])
            activaciones_totales = np.maximum(activaciones_totales, activaciones)  # Guardar las activaciones más altas
            
        # Publicar activaciones
        msg_activaciones = Float32MultiArray()
        msg_activaciones.data = activaciones_totales.tolist()
        self.publisher.publish(msg_activaciones)

        if not detected:
            self.get_logger().info("No se detectó ningún obstáculo.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


