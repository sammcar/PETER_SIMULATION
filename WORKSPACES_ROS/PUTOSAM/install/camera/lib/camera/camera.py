#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class CamaraNodo(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Tópico de la cámara
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Nodo de visualización de cámara iniciado.")


        self.boxes_a = np.zeros((8,2)) #matriz para promediarar
        self.aux_m = 0 #auxiliar para controlar el promedio o

    def publish_bounding(self):
        msg = Int32MultiArray()
        msg.data = self.array_data
        #self.publisher_bbox.publish(msg)  # Publica la bounding box
        self.get_logger().info('Publishing array: {}'.format(msg.data))

    def remove_small_objects(self, image, min_size):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_size:
                labels[labels == label] = 0
        return np.uint8(labels > 0) * 255

    
    def listener_callback(self, msg):
        #try:

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # Lee el frame de la cámara
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)  # Tamaño requerido del lider

        filtered_contours = []  # Arreglo de contornos filtrados

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cambio el frame a hsv

        # ---------------------FILTRO DE COLOR ------ ------------------------------------------------------
        # (En caso de creer que no funciona bien, muestre la imagen binarizada y use el código CameraAdjust

        # -------------------------- GREEN -------------------------------

        hue_min_green = 5
        saturation_min_green = 80
        value_min_green = 80
        hue_max_green = 80
        saturation_max_green = 255
        value_max_green = 255
        
        lower_green = np.array([hue_min_green, saturation_min_green, value_min_green])
        upper_green = np.array([hue_max_green, saturation_max_green, value_max_green])

        # -------------------------- RED --------------------------------


        hue_min_red = 0
        saturation_min_red = 0
        value_min_red = 0
        hue_max_red = 0
        saturation_max_red = 255
        value_max_red = 255
        
        lower_red = np.array([hue_min_red, saturation_min_red, value_min_red])
        upper_red = np.array([hue_max_red, saturation_max_red, value_max_red])

        # ----------------------------- BLUE -------------------------------
        
        hue_min_blue = 0
        saturation_min_blue = 0
        value_min_blue = 0
        hue_max_blue = 0
        saturation_max_blue = 255
        value_max_blue = 255
        
        lower_blue = np.array([hue_min_blue, saturation_min_blue, value_min_blue])
        upper_blue = np.array([hue_max_blue, saturation_max_blue, value_max_blue])

        # Filtrar el color verde
        # Máscaras del filtro de color para solo obtener como parte del frame el color correspondiente --------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_green, upper_green)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_green, upper_green)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        _, binarized_image = cv2.threshold(cv2.cvtColor(res, cv2.COLOR_BGR2GRAY), 1, 255,
                                        cv2.THRESH_OTSU)  # Hago un threshold para obtener una imagen binarizada
                                        # # Definir el kernel para la operación de erosión y dilatación
        kernel = np.ones((3,3),np.uint8)  # Puedes ajustar el tamaño del kernel según tus necesidades

        # # Aplicar la erosión
        binarized_image = cv2.erode(binarized_image, kernel, iterations = 1)

        # # Aplicar la dilatación
        binarized_image = cv2.dilate(binarized_image, kernel, iterations = 1)
        binarized_image = self.remove_small_objects(binarized_image, min_size=700)
        contours, _ = cv2.findContours(binarized_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Obtengo los contornos de la imagen binarizada

        # Condicional si encuentra contornos ---->
        if contours:
            # ---------------FILTRO DE SIMETRÍA DE LA BOUNDING BOX ---------------
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if ((y + h)-y)>30 and ((y + h)-y)<500:  # Rango de los contornos aceptados
                    filtered_contours.append(contour)

            # Si hay contornos filtrados -----------------------------------
            if filtered_contours:
                max_contour = max(filtered_contours, key=cv2.contourArea)  # Encuentra el contorno con mayor área

                area = cv2.contourArea(max_contour)
                if (area > 100):
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0),
                                2)  # Dibujo la bounding box en el frmae

                    #print("h: ",h)
                    # AJUSTE PARA QUE LA SALIDA DE LAS BOUNDING BOX SEAN IGUAL A YOLO
    
                    # ----------------------------------------------------------
                    self.boxes_a[self.aux_m, :] = [h, w] 
                    boxes = np.mean(self.boxes_a, axis=0)
                    self.array_data = [int(boxes[0]), int(boxes[1])]
                    self.aux_m = (self.aux_m + 1)*(self.aux_m<7)
                    self.publish_bounding()  # Publica la bounding box
                    print(int(boxes[0]))
        # Mostrar el fotograma en una ventana
        cv2.imshow('Video', frame)
                        
        # # Esperar a que se presione la tecla 'q' para salir del bucle
        cv2.waitKey(1)
        

        ros_image_msg = self.bridge.cv2_to_imgmsg(frame)
        #self.publisher_image.publish(ros_image_msg) FALTA

        #except Exception as e:
        #    self.get_logger().error(f"Error al procesar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    viewer = CamaraNodo()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

