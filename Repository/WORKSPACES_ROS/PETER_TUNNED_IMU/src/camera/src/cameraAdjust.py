#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.camera = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',  # Tópico de la cámara
            self.image_callback,
            10
        )
        self.publisher_bounding_box = self.create_publisher(Int32MultiArray, '/camera/bounding_box', 10)  # Publisher de la bounding box

        # Publisher de imagen
        self.bridge = CvBridge()
        self.topicNameFrames = 'topic_camera_image'  # Tópico del frame
        self.queueSize = 20
        self.publisher_image = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        
        self.i = 0  # contador de publicaciones

    def remove_small_objects(self, image, min_size):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_size:
                labels[labels == label] = 0
        return np.uint8(labels > 0) * 255

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # Lee el frame de la cámara
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)  # Tamaño requerido del frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_h_dark = cv2.getTrackbarPos('LowerD H', 'Trackbars')
        lower_s_dark = cv2.getTrackbarPos('LowerD S', 'Trackbars')
        lower_v_dark = cv2.getTrackbarPos('LowerD V', 'Trackbars')
        upper_h_dark = cv2.getTrackbarPos('UpperD H', 'Trackbars')
        upper_s_dark = cv2.getTrackbarPos('UpperD S', 'Trackbars')
        upper_v_dark = cv2.getTrackbarPos('UpperD V', 'Trackbars')

        lower_h_light = cv2.getTrackbarPos('LowerL H', 'Trackbars')
        lower_s_light = cv2.getTrackbarPos('LowerL S', 'Trackbars')
        lower_v_light = cv2.getTrackbarPos('LowerL V', 'Trackbars')
        upper_h_light = cv2.getTrackbarPos('UpperL H', 'Trackbars')
        upper_s_light = cv2.getTrackbarPos('UpperL S', 'Trackbars')
        upper_v_light = cv2.getTrackbarPos('UpperL V', 'Trackbars')

        lower_red_dark = np.array([lower_h_dark, lower_s_dark, lower_v_dark])
        upper_red_dark = np.array([upper_h_dark, upper_s_dark, upper_v_dark])

        lower_red_light = np.array([lower_h_light, lower_s_light, lower_v_light])
        upper_red_light = np.array([upper_h_light, upper_s_light, upper_v_light])

        # Threshold the HSV image to get only red colors
        mask_dark = cv2.inRange(hsv, lower_red_dark, upper_red_dark)
        mask_light = cv2.inRange(hsv, lower_red_light, upper_red_light)

        mask = cv2.bitwise_or(mask_light, mask_dark)

        res = cv2.bitwise_and(hsv, hsv, mask=mask)

        _, binarized_image = cv2.threshold(cv2.cvtColor(res, cv2.COLOR_BGR2GRAY), 1, 255, cv2.THRESH_OTSU)
        binarized_image = self.remove_small_objects(binarized_image, min_size=100)        

        # Find contours in the mask
        contours, _ = cv2.findContours(binarized_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            # Draw the bounding box on the original image
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            M = cv2.moments(max_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

            bounding_box_array = [x, y, w, h, cx, cy]

            msg = Int32MultiArray(data=bounding_box_array)
            self.publisher_bounding_box.publish(msg)

        cv2.imshow('Frame', binarized_image) 
        cv2.waitKey(1)


        #ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame) 
        #self.publisher_image.publish(ROS2ImageMessage)

        self.get_logger().info("Received camera frame.")

class CamaraNodo(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.camera_subscription = self.create_subscription(
            Image,
            'topic_camera_image',  # Tópico de la cámara
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Nodo de visualización de cámara iniciado.")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Vista de la Cámara - Peter Robot', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

def nothing(x):
    pass

def main(args=None):
    rclpy.init(args=args)

    cv2.namedWindow('Frame')
    cv2.namedWindow('Trackbars')

    # Create trackbars for adjusting lower and upper bounds of red color
    cv2.createTrackbar('LowerD H', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('LowerD S', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('LowerD V', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperD H', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperD S', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperD V', 'Trackbars', 0, 255, nothing)

    cv2.createTrackbar('LowerL H', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('LowerL S', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('LowerL V', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperL H', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperL S', 'Trackbars', 0, 255, nothing)
    cv2.createTrackbar('UpperL V', 'Trackbars', 0, 255, nothing)

    camera_publisher = CameraPublisher()
    camera_viewer = CamaraNodo()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_publisher)
    executor.add_node(camera_viewer)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        camera_publisher.destroy_node()
        camera_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
