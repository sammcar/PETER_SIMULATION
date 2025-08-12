#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu, LaserScan
from ros_gz_interfaces.msg import Contacts
import time
from collections import deque 
import re 

class NetworkPublisher(Node):

    def __init__(self):
        super().__init__('network_publisher')
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/peter_mode', 10)


        # Velocidades predeterminadas
        self.speed = 5.0  # Velocidad lineal
        self.turn = 5.0   # Velocidad angular

        # Limite para velocidad lineal
        self.max_speed = 6.5
        self.min_speed = 0.1

        # Limite para velocidad angular
        self.max_turn = 6.5
        self.min_turn = 0.1

        # Modo Inicial
        self.current_mode = 'C'

        # Intensidad de estimulo
        self.areaBoundingBoxR = 0.0
        self.areaBoundingBoxG = 0.0
        self.areaBoundingBoxB = 0.0

        # Posicion de estimulo
        self.posR = 0.0
        self.posG = 0.0
        self.posB = 0.0

        # Suscriptores
        # self.create_subscription(Float32MultiArray, '/bounding_box/red', self.red_callback, 100)
        # self.create_subscription(Float32MultiArray, '/bounding_box/green', self.green_callback, 100)
        # self.create_subscription(Float32MultiArray, '/bounding_box/blue', self.blue_callback, 100)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaRU', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaRD', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaLU', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaLD', self.bumper_callback, 10)
        self.accel_z_history = deque(maxlen=50)

        # Constantes y variables de dinámica neuronal
        self.initctes()

        # Timer para llamar la función de control cada segundo
        self.timer = self.create_timer(0.2, self.run_network)

    def initctes(self):

        #-------------------------N E U R O N A S--------------------------------------#

        self.Gpi = np.zeros((3, 2)) # Globo Pálido Interno
        self.Gpe = np.zeros((3, 2)) # Globo Pálido Externo
        self.STN = np.zeros((3, 2)) # Subtalámico
        self.STR = np.zeros((6, 2)) # Estriado
        self.z = np.zeros((20, 2)) # Red Sam/Espitia
        self.lidar = np.zeros((5,2)) # Wta Lidar
        self.Response = np.zeros((16,2))
        self.Aux = np.zeros((16,2))


        #------------------------- C O N S T A N T E S --------------------------------------#

        #DEBUGGING

        self.MOVEMENT = True


        #--------------IMU --------------
        #
        self.filtered_accel_z = 0
        self.accel_z_buffer = deque(maxlen=10)
        self.high_passed_accel_z = []
        self.alpha_dev = 0.9
        self.filtered_accel_z = 0

        self.ignore_imu = False
        self.ignore_start_time = None
        self.ignore_duration = 3.0  # segundos a ignorar



        self.ang_p = 90 # Posicion Frente del Robot
        self.ang_s = 90 # Posicion del estimulo
        self.epsilem = 0.1 # Tolerancia
        self.dt = 1 # Intervalo de Integracion
        self.cte = 1 # Constante de Avance
        self.Area = 100000 # Area Limite

        self.roll=0
        self.pitch=0
        self.std_dev_accel_z = 0
    

        self.w = 10 # Peso Sinaptico
        self.j = 2 # Peso Sinaptico

        self.A = 5 # Parametro Saturacion Naka
        self.Sigma = 0.3 # Parametro Inhibicion Naka
        self.SigmaIMU = 1.5 # Parametro Inhibicion Naka

        self.tau = 1 # Tao Neuronas Z
        self.tauMotor = 2 # Tao Neuronas Z
        self.TaoGpi = 1 # Tao Ganglios
        self.TaoGpe = 2 # Tao Ganglios
        self.TaoSTN = 2 # Tao Ganglios
        self.TaoSTR = 1 # Tao Ganglios

        self.Usigma_az = 3.5 #Umbral de variación estándar de un IMU
        self.Upitch = 15 #Umbral pitch
        self.Uroll = 15 #Umbral roll

        # 1) Pesos para Input -> Response (inverso)
        self.W_input_to_response = np.fliplr(np.eye(16))
        # 2) Inhibición lateral en Response
        self.weights_r_r = -np.ones((16,16)) + np.eye(16)
        # 3) Conexiones triangulares Response -> Aux
        self.W_response_to_aux = np.triu(np.ones((16,16)))

    #------------------------- LIDAR --------------------------------------#

        # Número total de neuronas y configuración de cuadrantes
        self.n_neuronas = 16  
        self.n_por_cuadrante = self.n_neuronas // 4

        self.activaciones_totales = np.zeros(self.n_neuronas)

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

        detected = False
        self.activaciones_totales = np.zeros(self.n_neuronas)  

        # Definir los límites de detección (en metros)
        r_min = 0.2  # Distancia mínima de detección (ejemplo: 20 cm)
        r_max = 1.0  # Distancia máxima de detección (ejemplo: 1 metro)

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment  

            # Filtrar valores no válidos o fuera del rango permitido
            if np.isnan(r) or np.isinf(r) or r < r_min or r > r_max:
                continue  

            detected = True  
            vector_input = np.array([np.cos(angle), np.sin(angle)])
            
            activaciones = np.array([self.gausiana(vector_input, self.Om[:, j]) for j in range(self.n_neuronas)]) * 130 / (1000 * r)
            self.activaciones_totales = np.maximum(self.activaciones_totales, activaciones)  # Guardar las activaciones más altas

        # Mensaje opcional si no se detecta nada dentro del rango
        # if not detected:
        #     self.get_logger().info("No se detectó ningún obstáculo dentro del rango especificado.")



    def run_network(self):

        self.std_dev_accel_z = np.std(self.high_passed_accel_z)
        
        
        
        # Detectar terreno rocoso
        if self.std_dev_accel_z > 3.53:  # Umbral ajustable
            self.get_logger().info("Terreno rocoso detectado 🚧")
        else:
            self.get_logger().info("Terreno liso 🛣️")

        
        #------------------------E S T I M U L O -------------------------------------#

        # Comportamiento esperado: Huye del depredador (Red) en movil H. Caza a la presa (Blue) en cuadrupedo.
        # Evita al obstaculo (Green) en modo omnidireccional. Si no hay estimulo avanza en modo movil H.
        # Estimulo en frente es posicion entre 70 y 110. Estimulo a la izquierda son valores entre 111 y 180.
        # Estimulo a la derecha son valores entre 0 y 69

        cmd_lineal = 0.0 # Comando de Velocidad Lineal
        cmd_ang = 0.0 # Comando de Velocidad Angular
        cmd_lateral = 0.0 # Comando de Velocidad Lateral

        #------------------------- D I N A M I C A --------------------------------------#

        self.lidar[0, 1] = self.lidar[0, 0] + (self.dt / 10) * (-self.lidar[0, 0] + (np.sum(self.activaciones_totales[0:4]) + np.sum(self.activaciones_totales[12:16]) - self.lidar[1, 0]))
        self.lidar[1, 1] = self.lidar[1, 0] + (self.dt / 10) * (-self.lidar[1, 0] + (np.sum(self.activaciones_totales[4:12]) - self.lidar[0, 0]))
        self.lidar[2, 1] = self.lidar[2, 0] + (self.dt / 10) * (-self.lidar[2, 0] + (np.sum(self.activaciones_totales[0:8]) - self.lidar[3, 0]))
        self.lidar[3, 1] = self.lidar[3, 0] + (self.dt / 10) * (-self.lidar[3, 0] + (np.sum(self.activaciones_totales[8:16]) - self.lidar[2, 0]))

        
        for r in range(16):

            self.Response[r, 1] = self.Response[r, 0] + (self.dt/5) * (-self.Response[r, 0] + max(0, (self.W_input_to_response @ self.activaciones_totales)[r] + self.weights_r_r[r, :] @ self.Response[:, 0]))
            self.Aux[r, 1] = self.Aux[r, 0] + (self.dt/5) * (-self.Aux[r, 0] + max(0, self.W_response_to_aux[r, :] @ self.Response[:, 1]))

        self.lidar[4,1] = self.lidar[4, 0] + (self.dt / self.tau) * (-self.lidar[4, 0] + max(0, (np.sum(self.Aux[:,0]))))


        R = self.areaBoundingBoxR/500
        if self.lidar[4,0]*10 > self.epsilem: G = self.lidar[4,0]*10 
        else: G = 0
        B = self.areaBoundingBoxB/500

        print("R: ", str(R))
        print("G: ", str(G))
        print("B: ", str(B)) 

        self.STN[0, 1] = np.clip((self.STN[0, 0] + (1/self.TaoSTN)*(-self.STN[0, 0]*5 + R - self.Gpi[0,0] - self.Gpe[1,0] - self.Gpe[2,0] -1.0)),0, None)
        self.STN[1, 1] = np.clip((self.STN[1, 0] + (1/self.TaoSTN)*(-self.STN[1, 0]*5 + G - self.Gpi[1,0] - self.Gpe[0,0] - self.Gpe[2,0] -1.0)),0, None)
        self.STN[2, 1] = np.clip((self.STN[2, 0] + (1/self.TaoSTN)*(-self.STN[2, 0]*5 + B - self.Gpi[2,0] - self.Gpe[0,0] - self.Gpe[1,0] -1.0)),0, None)
        
        self.Gpi[0, 1] = np.clip((self.Gpi[0, 0] + (1/self.TaoGpi)*(-self.Gpi[0, 0] + self.STN[1,0] + self.STN[2,0] - self.Gpe[0,0] - self.STR[0,0])),0, None)
        self.Gpi[1, 1] = np.clip((self.Gpi[1, 0] + (1/self.TaoGpi)*(-self.Gpi[1, 0] + self.STN[0,0] + self.STN[2,0] - self.Gpe[1,0] - self.STR[1,0])),0, None)
        self.Gpi[2, 1] = np.clip((self.Gpi[2, 0] + (1/self.TaoGpi)*(-self.Gpi[2, 0] + self.STN[0,0] + self.STN[1,0] - self.Gpe[2,0] - self.STR[2,0])),0, None)
        
        self.Gpe[0, 1] = np.clip((self.Gpe[0, 0] + (1/self.TaoGpe)*(-self.Gpe[0, 0] + self.STN[0,0])),0, None)
        self.Gpe[1, 1] = np.clip((self.Gpe[1, 0] + (1/self.TaoGpe)*(-self.Gpe[1, 0] + self.STN[1,0])),0, None)
        self.Gpe[2, 1] = np.clip((self.Gpe[2, 0] + (1/self.TaoGpe)*(-self.Gpe[2, 0] + self.STN[2,0])),0, None)
        
        self.STR[0, 1] = np.clip((self.STR[0, 0] + (1/self.TaoSTR)*(-self.STR[0, 0] + self.STN[0,0])),0, None)
        self.STR[1, 1] = np.clip((self.STR[1, 0] + (1/self.TaoSTR)*(-self.STR[1, 0] + self.STN[1,0])),0, None)
        self.STR[2, 1] = np.clip((self.STR[2, 0] + (1/self.TaoSTR)*(-self.STR[2, 0] + self.STN[2,0])),0, None)

        if self.Gpe[0,1] > 1.5 and R > 0.5:
            self.ang_s = self.posR
        elif self.Gpe[1,1] > 1.5 and G > 0.5:
            self.ang_s = 90*(self.lidar[0,0]>0.5) + 170*(self.lidar[2,0]>0.5) + 10*(self.lidar[3,0]>0.5)
        elif self.Gpe[2,1] > 1.5 and B > 0.5:
            self.ang_s = self.posB
        else:
            self.ang_s = 90.0

        # ------IMPLEMENTACIÒN MÒDULO IMU ----------

        self.z[0, 1] = self.z[0, 0] + (self.dt / self.tau) * (-self.z[0, 0] + (self.A * max(0, (5*self.std_dev_accel_z - 5*self.Usigma_az ))**2) / (self.SigmaIMU**2 + (-self.z[0,0] + self.std_dev_accel_z - self.Usigma_az )**2))
        self.z[1, 1] = self.z[1, 0] + (self.dt / self.tau) * (-self.z[1, 0] + (self.A * max(0, (self.pitch - self.Upitch ))**2) / (self.SigmaIMU**2 + (-self.z[1,0] + self.pitch - self.Upitch )**2))
        self.z[2, 1] = self.z[2, 0] + (self.dt / self.tau) * (-self.z[2, 0] + (self.A * max(0, (self.roll - self.Uroll ))**2) / (self.SigmaIMU**2 + (-self.z[2,0] + self.roll - self.Uroll )**2))

        self.z[3, 1] = self.z[3, 0] + (self.dt / self.tau) * (-self.z[3, 0] + max(0, (self.Gpe[2,0] )))
        self.z[4, 1] = self.z[4, 0] + (self.dt / self.tau) * (-self.z[4, 0] + max(0, (self.Gpe[1, 0] + self.j * self.Gpe[0, 0])))

        self.z[5, 1] = self.z[5, 0] + (self.dt / self.tau) * (-self.z[5, 0] + max(0, (self.ang_s - self.ang_p) - 20))
        self.z[6, 1] = self.z[6, 0] + (self.dt / self.tau) * (-self.z[6, 0] + max(0, (self.ang_p - self.ang_s) - 20))

        self.z[7, 1] = self.z[7, 0] + (self.dt / self.tau) * (-self.z[7, 0] + max(0, (self.z[5, 0] + self.z[3, 0] - self.w*self.z[4, 0])))
        self.z[8, 1] = self.z[8, 0] + (self.dt / self.tau) * (-self.z[8, 0] + max(0, (self.z[5, 0] + self.z[4, 0] - self.w*self.z[3, 0])))
        self.z[9, 1] = self.z[9, 0] + (self.dt / self.tau) * (-self.z[9, 0] + max(0, (self.z[4, 0] + self.z[6, 0] - self.w*self.z[3, 0])))
        self.z[10, 1] = self.z[10, 0] + (self.dt / self.tau) * (-self.z[10, 0] + max(0, (self.z[3, 0] + self.z[6, 0] - self.w*self.z[4, 0])))

        self.z[11, 1] = self.z[11, 0] + (self.dt / self.tau) * (-self.z[11, 0] + max(0, (self.z[7, 0] + self.z[9, 0])))
        self.z[12, 1] = self.z[12, 0] + (self.dt / self.tau) * (-self.z[12, 0] + max(0, (self.z[10, 0] + self.z[8, 0])))
        self.z[13, 1] = self.z[13, 0] + (self.dt / self.tau) * (-self.z[13, 0] + max(0, (-self.w*abs(cmd_ang)*self.z[11, 0] - self.w*abs(cmd_ang)*self.z[12, 0] -self.w*self.z[17,0] + self.cte)))

        self.z[14, 1] = self.z[14, 0] + (self.dt / self.tau) * (-self.z[14, 0] + (self.A * max(0, (100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[2, 0] - self.w*self.z[15, 0] - self.w*self.z[16, 0] ))**2) / (self.Sigma**2 + (100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[2, 0] - self.w*self.z[15, 0] - self.w*self.z[16, 0] )**2))
        self.z[15, 1] = self.z[15, 0] + (self.dt / self.tau) * (-self.z[15, 0] + (self.A * max(0, (-self.Gpe[1,0] -0.5*self.cte + self.z[3, 0]*self.w + 15*self.z[0,0] + 0.7*self.z[1,0] + 0.7*self.z[2,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[16, 0] ))**2) / (self.Sigma**2 + (-self.Gpe[1,0]-0.5*self.cte + self.z[3, 0]*2 + 20*self.z[0,0] + 0.7*self.z[1,0] + 0.7*self.z[2,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[16, 0] )**2))
        self.z[16, 1] = self.z[16, 0] + (self.dt / self.tau) * (-self.z[16, 0] + (self.A * max(0, (self.z[4, 0] - self.w*self.Gpe[1,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[15, 0]*1.5 + self.cte ))**2) / (self.Sigma**2 + (self.z[4, 0] - self.w*self.Gpe[1,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[15, 0]*1.5 + self.cte )**2))
        
        self.z[17, 1] = self.z[17, 0] + (self.dt / self.tau) * (-self.z[17, 0] + max(0, (self.Gpe[2,0] - self.Area)))

        cmd_ang = (self.z[11,0]*(self.Gpe[1,0] < 0.5)) - (self.z[12,0]*(self.Gpe[1,0]<0.5))
        cmd_lateral = (-self.lidar[2,0]*1.5 + self.lidar[3,0]*1.5 + self.z[11,0]*(self.Gpe[1,0] > 0.5)) - (self.z[12,0]*(self.Gpe[1,0] > 0.5))
        cmd_lineal = -self.lidar[0,0]*1.5 + self.lidar[1,0]*1.5 +  self.z[13,0] -self.j*self.z[4,0]*(self.z[5,0] < self.epsilem and self.z[6,0] < self.epsilem)

        for i in range(len(self.z)): self.z[i, 0] = self.z[i,1]
        for i in range(len(self.lidar)): self.lidar[i, 0] = self.lidar[i,1]
        for i in range(len(self.Response)): self.Response[i, 0] = self.Response[i,1]
        for i in range(len(self.Aux)): self.Aux[i, 0] = self.Aux[i,1]
        for i in range(len(self.STN)): self.STN[i, 0] = self.STN[i,1]
        for i in range(len(self.Gpi)): self.Gpi[i, 0] = self.Gpi[i,1]
        for i in range(len(self.Gpe)): self.Gpe[i, 0] = self.Gpe[i,1]
        for i in range(len(self.STR)): self.STR[i, 0] = self.STR[i,1]


        # ------------------- PRINTS -------------------------


        self.get_logger().info(
                        f"GpeR: {self.Gpe[0,1]}\n"
                        f"GpeG: {self.Gpe[1,1]}\n"
                        f"GpeB: {self.Gpe[2,1]}\n"
                        f"ang_p: {self.ang_p}\n"
                        f"ang_s: {self.ang_s}\n"
                        f"3: {self.z[3,1]}\n"
                        f"4: {self.z[4,1]}\n"
                        f"5: {self.z[5,1]}\n"
                        f"6: {self.z[6,1]}\n"
                        f"7: {self.z[7,1]}\n"
                        f"8: {self.z[8,1]}\n"
                        f"9: {self.z[9,1]}\n"
                        f"10: {self.z[10,1]}\n"
                        f"11: {self.z[11,1]}\n"
                        f"12: {self.z[12,1]}\n"
                        f"13: {self.z[13,1]}\n"
                        f"14: {self.z[14,1]}\n"
                        f"15: {self.z[15,1]}\n"
                        f"16: {self.z[16,1]}\n"
                        f"0: {self.z[0,1]}\n"
                        f"1: {self.z[1,1]}\n"
                        f"2: {self.z[2,1]}\n"
                        f"roll: {self.roll}\n"
                        f"pitch: {self.pitch}\n"
                        f"VIBRACION: {self.std_dev_accel_z}"
                        )

        #------------------------- P U B L I C A C I O N --------------------------------------#
        
        if self.MOVEMENT:
            if self.epsilem < cmd_ang:
                self.publish_twist(angular_z=self.turn)  # Gira izquierda
                print("Giro Izquierda")
            elif cmd_ang < -self.epsilem:
                self.publish_twist(angular_z=-self.turn)  # Gira derecha
                print("Giro Derecha")
            else:        
                #-------------------------------------------------------------------

                if self.epsilem < cmd_lineal:
                    self.publish_twist(linear_x=self.speed)  # Adelante
                    print("Avanza")
                elif cmd_lineal < -self.epsilem:
                    self.publish_twist(linear_x=-self.speed) # Atrás
                    print("Retrocede")
                elif self.z[17,1] > 0.5:
                    self.publish_twist() # Stop
                    print("Stop")
                else:
                    pass
                #-------------------------------------------------------------------
                if self.epsilem < cmd_lateral:
                    self.publish_twist(linear_y=self.speed) # Izquierda
                    print("Desp. Izquierda")
                elif cmd_lateral < -self.epsilem:
                    self.publish_twist(linear_y=-self.speed) # Derecha
                    print("Desp. Derecha")
                else:
                    pass
                #-------------------------------------------------------------------



            if self.z[15,1] > 0.5:
                self.publish_mode('C')
                print("Cuadrupedo")
            elif self.z[16,1] > 0.5:
                self.publish_mode('H')
                print("Móvil H")
            elif self.z[14,1] > 0.5:
                self.publish_mode('X')
                print("Móvil X")
            else:
                pass

    #------------------------- F U N C I O N E S    A U X I L I A R E S --------------------------------------#
        # Callbacks de cada estímulo
    def red_callback(self, msg):
        if len(msg.data) >= 2:
            self.posR = msg.data[0]  # Posición en el rango de 0 a 180
            self.areaBoundingBoxR = msg.data[1]  # Área aproximada
            # self.get_logger().info(f'Red - Pos: {self.posR}, Área: {self.areaBoundingBoxR}')

    def green_callback(self, msg):
        if len(msg.data) >= 2:
            self.posG = msg.data[0]
            self.areaBoundingBoxG = msg.data[1]
            # self.get_logger().info(f'Green - Pos: {self.posG}, Área: {self.areaBoundingBoxG}')

    def blue_callback(self, msg):
        if len(msg.data) >= 2:
            self.posB = msg.data[0]
            self.areaBoundingBoxB = msg.data[1]
            # self.get_logger().info(f'Blue - Pos: {self.posB}, Área: {self.areaBoundingBoxB}')

    def bumpRU_callback(self,msg):
        pass

    def bumpRD_callback(self,msg):
        pass

    def bumpLU_callback(self,msg):
        pass

    def bumpLD_callback(self,msg):
        pass
    
    def imu_callback(self, msg):
        # Extraer cuaterniones
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Convertir cuaterniones a ángulos de Euler (Roll y Pitch)
        self.roll = 180 - abs(np.degrees((np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)))))
        self.pitch = abs(np.degrees((np.arcsin(2 * (qw * qy - qz * qx)))) )

        
        z = msg.linear_acceleration.z
        self.filtered_accel_z = self.alpha_dev * self.filtered_accel_z + (1 - self.alpha_dev) * z
        high_pass = z - self.filtered_accel_z
        self.high_passed_accel_z.append(high_pass)

        if len(self.accel_z_buffer) > 1:
            self.std_dev_accel_z = np.std(self.accel_z_buffer)
        else:
            self.std_dev_accel_z = 0.0

        # Mostrar información
        # self.get_logger().info(f"Roll: {self.roll:.2f}°, Pitch: {self.pitch:.2f}°, Aceleración Z: {self.accel_z:.2f} m/s², STD Z: {std_dev_accel_z:.4f}")

    def lidar_callback(self, msg):
        """Callback para el LiDAR: almacena el primer valor del array de rangos."""
        self.lidar_data = msg.ranges[0] if msg.ranges else None

    # def bumpRU_callback(self, msg):
    #     """Callback para el bumper de la TibiaRU."""
    #     self.bumpRU_data = len(msg.contacts) > 0

    # def bumpRD_callback(self, msg):
    #     """Callback para el bumper de la TibiaRD."""
    #     self.bumpRD_data = len(msg.contacts) > 0

    # def bumpLU_callback(self, msg):
    #     """Callback para el bumper de la TibiaLU."""
    #     self.bumpLU_data = len(msg.contacts) > 0

    # def bumpLD_callback(self, msg):
    #     """Callback para el bumper de la TibiaLD."""
    #     self.bumpLD_data = len(msg.contacts) > 0


    def publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publica el mensaje Twist con los valores dados."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def publish_mode(self, mode):
        """Publica el modo actual."""
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        if self.current_mode != mode:
            self.ignore_imu = True
            self.ignore_start_time = time.time()
    
        self.current_mode = mode

    






#------------------------- M A I N --------------------------------------#

def main(args=None):
    rclpy.init(args=args)
    node = NetworkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()