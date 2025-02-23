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
        self.bridge = CvBridge()
        
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Tópico de la cámara
            self.listener_callback,
            10
        )

        # Publicadores para cada color
        self.publisher_red = self.create_publisher(Int32MultiArray, '/bounding_box/red', 10)
        self.publisher_blue = self.create_publisher(Int32MultiArray, '/bounding_box/blue', 10)
        self.publisher_green = self.create_publisher(Int32MultiArray, '/bounding_box/green', 10)

        self.get_logger().info("Nodo de visualización de cámara iniciado.")

    def publish_bounding(self, publisher, x, y, w, h, colorname):
        msg = Int32MultiArray()
        msg.data = [x, y, w, h]
        self.get_logger().info(f'Array {colorname}: {[x, y, w, h]}')
        publisher.publish(msg)

    def remove_small_objects(self, image, min_size):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_size:
                labels[labels == label] = 0
        return np.uint8(labels > 0) * 255

    def filter_and_draw(self, frame, hsv, lower_color, upper_color, color_bgr, publisher, colorname):
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = self.remove_small_objects(mask, min_size=700)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if area > 100:
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
                self.publish_bounding(publisher, x, y, w, h, colorname)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Filtros de color (HSV)
        color_filters = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'blue': ((80, 0, 150), (255, 255, 255)),
            'green': ((5, 80, 80), (80, 255, 255)),
        }

        # Detectar y dibujar las bounding boxes
        self.filter_and_draw(frame, hsv, *color_filters['red'], (0, 0, 255), self.publisher_red, "Red")
        self.filter_and_draw(frame, hsv, *color_filters['blue'], (255, 0, 0), self.publisher_blue, "Blue")
        self.filter_and_draw(frame, hsv, *color_filters['green'], (0, 255, 0), self.publisher_green, "Green")

        cv2.imshow('Video', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = CamaraNodo()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
