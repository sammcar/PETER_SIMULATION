#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

# Definir límites para filtrar áreas publicadas
AREA_MIN = 500    # Área mínima aceptada para evitar ruido
AREA_MAX = 300000  # Área máxima aceptada para evitar falsos positivos

class CamaraNodo(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()

        # Suscripción al tópico de la cámara
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Tópico de la cámara
            self.listener_callback,
            10
        )

        # Publicadores para cada color
        self.publisher_red = self.create_publisher(Float32MultiArray, '/bounding_box/red', 10)
        self.publisher_blue = self.create_publisher(Float32MultiArray, '/bounding_box/blue', 10)
        #self.publisher_green = self.create_publisher(Float32MultiArray, '/bounding_box/green', 10)

        # Variables para promediar bounding boxes
        self.boxes_a = np.zeros((8, 2))  # Matriz para suavizar valores de altura y ancho
        self.aux_m = 0  # Auxiliar para controlar el promedio
        self.last_detection = {'red': False, 'blue': False, 
                               #'green': False
                               }  # Estado previo de cada color

        # Variables para la visualización en otro hilo
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True  # Se cerrará cuando termine el programa
        self.display_thread.start()

        self.get_logger().info("Nodo de visualización de cámara iniciado.")

    def publish_bounding(self, publisher, x, w, h, detected):
        """
        Publica los datos de la bounding box en el tópico correspondiente.
        Si no hay detección válida, publica [0.0, 0.0].
        """
        msg = Float32MultiArray()
        if detected:
            x_center = x + (w / 2)  # Cálculo del centroide en X
            x_mapped = float(180 - (x_center / 639) * 180)  # Mapeo del centroide al rango 0-180
            area = float(w * h)

            # Filtrar áreas fuera del rango permitido
            if AREA_MIN <= area <= AREA_MAX:
                msg.data = [x_mapped, area]
            else:
                msg.data = [0.0, 0.0]  # Publicar 0 si el área es inválida
        else:
            msg.data = [0.0, 0.0]  # Publicar 0 si no hay detección

        publisher.publish(msg)

    def remove_small_objects(self, image, min_size):
        """
        Filtra objetos pequeños eliminando componentes conectadas menores a min_size.
        """
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_size:
                labels[labels == label] = 0
        return np.uint8(labels > 0) * 255

    def filter_and_draw(self, frame, hsv, lower_color, upper_color, color_bgr, publisher, colorname):
        #print(f"\nProcesando color: {colorname}")
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Debug: Ver cuántos pixeles blancos hay en la máscara inicial
        initial_white_pixels = cv2.countNonZero(mask)
        #print(f"Pixeles detectados (máscara inicial): {initial_white_pixels}")

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        mask = self.remove_small_objects(mask, min_size=700)

        # Debug: Pixeles tras filtrado morfológico
        final_white_pixels = cv2.countNonZero(mask)
        #print(f"Pixeles detectados (tras filtrado morfológico): {final_white_pixels}")

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        filtered_contours = [c for c in contours if 30 < cv2.boundingRect(c)[3] < 500]
        #print(f"Número de contornos filtrados por altura: {len(filtered_contours)}")

        if filtered_contours:
            max_contour = max(filtered_contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            #print(f"Área del contorno más grande: {area}")

            if AREA_MIN <= area <= AREA_MAX:
                x, y, w, h = cv2.boundingRect(max_contour)
                #print(f"Bounding box: x={x}, y={y}, w={w}, h={h}")
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)

                self.publish_bounding(publisher, x, w, h, detected=True)
                self.last_detection[colorname] = True
            else:
                #print(f"Área ({area}) fuera de rango permitido ({AREA_MIN}-{AREA_MAX}).")
                self.publish_bounding(publisher, 0, 0, 0, detected=False)
                self.last_detection[colorname] = False
        else:
            #print("No se encontraron contornos válidos.")
            self.publish_bounding(publisher, 0, 0, 0, detected=False)
            self.last_detection[colorname] = False

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_filters = {
            'red': ((0, 65, 70), (10, 255, 215)),
            'blue': ((80, 0, 40), (179, 255, 255)),
            #'green': ((35, 80, 75), (75, 255, 255)),
        }

        # Procesa y dibuja bounding boxes sobre la imagen frame
        self.filter_and_draw(frame, hsv, *color_filters['red'], (0, 0, 255), self.publisher_red, "red")
        self.filter_and_draw(frame, hsv, *color_filters['blue'], (255, 0, 0), self.publisher_blue, "blue")
        #self.filter_and_draw(frame, hsv, *color_filters['green'], (0, 255, 0), self.publisher_green, "green")

        # 🔥 COPIA AQUÍ, justo después de procesar y dibujar bounding boxes
        with self.frame_lock:
            self.latest_frame = frame.copy()


    def display_loop(self):
        """
        Hilo dedicado a la visualización de la imagen usando OpenCV.
        """
        while rclpy.ok():
            with self.frame_lock:
                if self.latest_frame is not None:
                    display_frame = self.latest_frame.copy()
                else:
                    display_frame = None

            if display_frame is not None:
                cv2.imshow('Video', display_frame)
            # Si se presiona 'q' se sale de la visualización
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

def main(args=None):
    """
    Función principal que inicializa el nodo y ejecuta el bucle ROS2.
    """
    rclpy.init(args=args)
    viewer = CamaraNodo()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
