import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CamaraNodo(Node):
    def __init__(self):
        super().__init__('camara_nodo')
        self.bridge = CvBridge()
        self.last_frame = None
        # Suscribirse al tópico de la cámara (cola de tamaño 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        # Convertir el mensaje de ROS Image a un frame de OpenCV (formato BGR8)
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

def main(args=None):
    rclpy.init(args=args)
    node = CamaraNodo()
    try:
        # **Inicializar ventanas de visualización y controles una sola vez**
        cv2.namedWindow("Camara", cv2.WINDOW_NORMAL)    # Ventana para la imagen de la cámara
        cv2.namedWindow("Mascara", cv2.WINDOW_NORMAL)   # Ventana para la máscara filtrada
        # (Opcional) Ventana y trackbars para ajustar filtros HSV:
        cv2.namedWindow("Filtros HSV", cv2.WINDOW_NORMAL)
        # Crear barras de control HSV (min y max para H, S, V)
        cv2.createTrackbar("H min", "Filtros HSV", 0, 179, lambda x: None)
        cv2.createTrackbar("H max", "Filtros HSV", 179, 179, lambda x: None)
        cv2.createTrackbar("S min", "Filtros HSV", 0, 255, lambda x: None)
        cv2.createTrackbar("S max", "Filtros HSV", 255, 255, lambda x: None)
        cv2.createTrackbar("V min", "Filtros HSV", 0, 255, lambda x: None)
        cv2.createTrackbar("V max", "Filtros HSV", 255, 255, lambda x: None)

        # Bucle principal: correr mientras ROS2 esté activo
        while rclpy.ok():
            # Procesar un ciclo de mensajes ROS (no bloqueante)
            rclpy.spin_once(node, timeout_sec=0.01)  
            # Si hay un frame disponible, procesarlo
            if node.last_frame is not None:
                frame = node.last_frame  # Tomar la última imagen recibida
                # Obtener valores actuales de los trackbars
                h_min = cv2.getTrackbarPos("H min", "Filtros HSV")
                h_max = cv2.getTrackbarPos("H max", "Filtros HSV")
                s_min = cv2.getTrackbarPos("S min", "Filtros HSV")
                s_max = cv2.getTrackbarPos("S max", "Filtros HSV")
                v_min = cv2.getTrackbarPos("V min", "Filtros HSV")
                v_max = cv2.getTrackbarPos("V max", "Filtros HSV")
                # Convertir imagen a HSV y aplicar filtro por rangos
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower = (h_min, s_min, v_min)
                upper = (h_max, s_max, v_max)
                mask = cv2.inRange(hsv, lower, upper)
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # opcional: convertir mask a BGR para mostrar en color
                # Mostrar la imagen original y la máscara filtrada en sus ventanas
                cv2.imshow("Camara", frame)
                cv2.imshow("Mascara", mask_bgr)
            # Mostrar las ventanas y esperar 1 ms para procesar eventos de teclado/ventana
            key = cv2.waitKey(1) & 0xFF
            # Si se pulsa ESC (código 27), salir del bucle
            if key == 27:
                node.get_logger().info("Esc presionado, cerrando...")
                break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción manual, cerrando...")
    finally:
        # **Limpiar recursos: cerrar ventanas y apagar nodo ROS2**
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
