#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float64
from sensor_msgs.msg import Imu, LaserScan
from ros_gz_interfaces.msg import Contacts
from collections import deque 
import re 

class NetworkPublisher(Node):

    def __init__(self):
        super().__init__('network_publisher')
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/peter_mode', 10)

        self.GPEr = self.create_publisher(Float64, '/Gpe_Hostil', 10)
        self.GPEg = self.create_publisher(Float64, '/Gpe_Obstaculo', 10)
        self.GPEb = self.create_publisher(Float64, '/Gpe_Apetente', 10)

        self.publisher_ = self.create_publisher(Float32MultiArray, 'neuron_activity', 10)
        self.publisher_imu = self.create_publisher(Float32MultiArray, 'imu_activity', 10)

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

        # self.create_subscription(Contacts, '/bumper/TibiaRU', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaRD', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaLU', self.bumper_callback, 10)
        # self.create_subscription(Contacts, '/bumper/TibiaLD', self.bumper_callback, 10)
        self.create_subscription(Float32MultiArray, '/bounding_box/red', self.red_callback, 100)
        self.create_subscription(Float32MultiArray, '/bounding_box/blue', self.blue_callback, 100)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.accel_z_history = deque(maxlen=50)
        # Constantes y variables de dinámica neuronal
        self.initctes()

        # Timer para llamar la función de control cada segundo
        self.timer = self.create_timer(0.2, self.run_network)

    def initctes(self):

        #-------------------------N E U R O N A S--------------------------------------#

        self.Gpi = np.zeros((3, 2)) # Globo Pálido Interno
        self.Gpe = np.zeros((3, 2)) # Globo Pálido Externo
        self.StN = np.zeros((3, 2)) # Subtalámico
        self.StR = np.zeros((3, 2)) # Estriado
        self.z = np.zeros((20, 2)) # Red Sam/Espitia
        self.lidar = np.zeros((5,2)) # Wta Lidar
        self.Response = np.zeros((16,2))
        self.Aux = np.zeros((16,2))

        #------------------------- C O N S T A N T E S --------------------------------------#

        #DEBUGGING

        self.MOVEMENT = True

        #--------------IMU --------------
        
        self.ignore_imu = False
        self.ignore_timer = time.time()
        self.ignore_duration = 4.2

        # Antes de tu callback, inicializa buffers y filtros:
        self.accel_buffer = deque(maxlen=50)     # Ventana de 50 muestras
        self.accel_std = 0.0
        self.accel_std2 = 0.0

        #Logica que será neuronal
        self.terrainchanger = False
        self.terrain_timer = 0.0  # Guarda el tiempo de inicio

        self.low_accel_counter = 0
        self.low_accel_threshold = 0.5
        self.low_accel_limit = 100
        self.low_accel_flag = False

        self.ang_p = 90 # Posicion Frente del Robot
        self.ang_s = 90 # Posicion del estimulo
        self.epsilem = 0.01 # Tolerancia
        self.dt = 1 # Intervalo de Integracion
        self.cte = 3 # Constante de Avance
        self.Area = 28 # Area Limite Tuneada segun iluminacion

        self.roll = 0.0
        self.pitch = 0.0
        self.std_dev_accel_z = 0.0
        self.accel_z_history = []
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

        #self.Usigma_az = 3.27 #PARA CASO PLANO-RUGOSO-PLANO
        self.Usigma_az = 3.7 #PARA CASO PLANO-INLINADO
        self.Upitch = 7.8 #Umbral pitch
        self.Uroll = 270 #Umbral roll

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
        #     print("No se detectó ningún obstáculo dentro del rango especificado.")

    def map_with_limits(self, value, in_min, in_max, out_min, out_max):
        """Mapea un valor de un rango de entrada a uno de salida, con límites aplicados."""
        if value <= in_min:
            return out_min
        elif value >= in_max:
            return out_max
        else:
            # Interpolación lineal
            m = (out_max - out_min) / (in_max - in_min)
            b = out_min - m * in_min
            return m * value + b
        
    def limit(self, value, max):
        """Limita un valor."""

        if value > max: 
            value = max
        elif value <-max:
            value = -max
        else:
            pass

        return value

    def run_network(self):
        #------------------------E S T I M U L O -------------------------------------#

        # Comportamiento esperado: Huye del depredador (Red) en movil H. Caza a la presa (Blue) en cuadrupedo.
        # Evita al obstaculo (Green) en modo omnidireccional. Si no hay estimulo avanza en modo movil H.
        # Estimulo en frente es posicion entre 70 y 110. Estimulo a la izquierda son valores entre 111 y 180.
        # Estimulo a la derecha son valores entre 0 y 69

        cmd_lineal = 0.0 # Comando de Velocidad Lineal
        cmd_ang = 0.0 # Comando de Velocidad Angular
        cmd_lateral = 0.0 # Comando de Velocidad Lateral

        #------------------------- D I N A M I C A --------------------------------------#

        self.lidar[0, 1] = self.lidar[0, 0] + (self.dt / 10) * (-self.lidar[0, 0] + (np.sum(self.activaciones_totales[0:4]) + np.sum(self.activaciones_totales[12:16]) - self.lidar[1, 0])) # Frente
        self.lidar[1, 1] = self.lidar[1, 0] + (self.dt / 10) * (-self.lidar[1, 0] + (np.sum(self.activaciones_totales[4:12]) - self.lidar[0, 0])) # Atras
        self.lidar[2, 1] = self.lidar[2, 0] + (self.dt / 10) * (-self.lidar[2, 0] + (np.sum(self.activaciones_totales[0:8]) - self.lidar[3, 0])) # Izquierda
        self.lidar[3, 1] = self.lidar[3, 0] + (self.dt / 10) * (-self.lidar[3, 0] + (np.sum(self.activaciones_totales[8:16]) - self.lidar[2, 0])) # Derecha

        for r in range(16):

            self.Response[r, 1] = self.Response[r, 0] + (self.dt/5) * (-self.Response[r, 0] + max(0, (self.W_input_to_response @ self.activaciones_totales)[r] + self.weights_r_r[r, :] @ self.Response[:, 0]))
            self.Aux[r, 1] = self.Aux[r, 0] + (self.dt/5) * (-self.Aux[r, 0] + max(0, self.W_response_to_aux[r, :] @ self.Response[:, 1]))

        self.lidar[4,1] = self.lidar[4, 0] + (self.dt / self.tau) * (-self.lidar[4, 0]*1.1 + max(0, (np.sum(self.Aux[:,0]))))


        R = self.areaBoundingBoxR/500
        #R = 2 #Descomentar para probar inclinacion
        if self.lidar[4,0]*15 > 0.2: G = self.lidar[4,0]*15
        else: G = 0
        B = self.areaBoundingBoxB/500


        self.StN[0, 1] = np.clip((self.StN[0, 0] + (1/self.TaoSTN)*(-self.StN[0, 0] + R - self.Gpi[0,0] - self.Gpe[1,0] - self.Gpe[2,0] -1.0)),0, None)
        self.StN[1, 1] = np.clip((self.StN[1, 0] + (1/self.TaoSTN)*(-self.StN[1, 0] + G - self.Gpi[1,0] - self.Gpe[0,0] - self.Gpe[2,0] -1.0)),0, None)
        self.StN[2, 1] = np.clip((self.StN[2, 0] + (1/self.TaoSTN)*(-self.StN[2, 0] + B*0.7 - self.Gpi[2,0] - self.Gpe[0,0] - self.Gpe[1,0] -1.0)),0, None)
        
        self.Gpi[0, 1] = np.clip((self.Gpi[0, 0] + (1/self.TaoGpi)*(-self.Gpi[0, 0] + self.StN[1,0] + self.StN[2,0] - self.Gpe[0,0] - self.StR[0,0])),0, None)
        self.Gpi[1, 1] = np.clip((self.Gpi[1, 0] + (1/self.TaoGpi)*(-self.Gpi[1, 0] + self.StN[0,0] + self.StN[2,0] - self.Gpe[1,0] - self.StR[1,0])),0, None)
        self.Gpi[2, 1] = np.clip((self.Gpi[2, 0] + (1/self.TaoGpi)*(-self.Gpi[2, 0] + self.StN[0,0] + self.StN[1,0] - self.Gpe[2,0] - self.StR[2,0])),0, None)
        
        self.Gpe[0, 1] = np.clip((self.Gpe[0, 0] + (1/self.TaoGpe)*(-self.Gpe[0, 0] + self.StN[0,0])),0, None)
        self.Gpe[1, 1] = np.clip((self.Gpe[1, 0] + (1/self.TaoGpe)*(-self.Gpe[1, 0] + self.StN[1,0]*5)),0, None)
        self.Gpe[2, 1] = np.clip((self.Gpe[2, 0] + (1/self.TaoGpe)*(-self.Gpe[2, 0] + self.StN[2,0]*0.5)),0, None)
        
        self.StR[0, 1] = np.clip((self.StR[0, 0] + (1/self.TaoSTR)*(-self.StR[0, 0] + self.StN[0,0])),0, None)
        self.StR[1, 1] = np.clip((self.StR[1, 0] + (1/self.TaoSTR)*(-self.StR[1, 0] + self.StN[1,0])),0, None)
        self.StR[2, 1] = np.clip((self.StR[2, 0] + (1/self.TaoSTR)*(-self.StR[2, 0] + self.StN[2,0])),0, None)

        if self.Gpe[0,1] > 1.5 and R > 0.5:
            self.ang_s = self.posR
        # elif self.Gpe[1,1] > 0.5 and G > 0.5:
        #     self.ang_s = 180*(self.lidar[2,1] > 0.1) + 90*(self.lidar[0,1] > 0.1) + self.ang_s*(self.lidar[4,1]<0.1)
        elif self.Gpe[2,1] > 1.5 and B > 0.5:
            self.ang_s = self.posB
        else:
            self.ang_s = 90*(self.lidar[4,1]<0.3)

        #self.ang_s = 90 #Descomentar para probar terreno inclinado

        # ------IMPLEMENTACIÒN MÒDULO IMU ----------

        self.z[0, 1] = self.z[0, 0] + (self.dt / self.tau) * (-self.z[0, 0] + (self.A * max(0, (self.std_dev_accel_z - self.Usigma_az ))**2) / (self.SigmaIMU**2 + (-self.z[0,0] + self.std_dev_accel_z - self.Usigma_az )**2))
        self.z[1, 1] = self.z[1, 0] + (self.dt / self.tau) * (-self.z[1, 0] + (self.A * max(0, (self.pitch - self.Upitch ))**2) / (self.SigmaIMU**2 + (-self.z[1,0] + self.pitch - self.Upitch )**2))
        self.z[2, 1] = self.z[2, 0] + (self.dt / self.tau) * (-self.z[2, 0] + (self.A * max(0, (self.roll - self.Uroll ))**2) / (self.SigmaIMU**2 + (-self.z[2,0] + self.roll - self.Uroll )**2))
        self.z[3, 1] = self.z[3, 0] + (self.dt / self.tau) * (-self.z[3, 0] + max(0, (self.Gpe[2,0] )))
        self.z[4, 1] = self.z[4, 0] + (self.dt / self.tau) * (-self.z[4, 0] + max(0, (self.Gpe[1, 0] + self.j * self.Gpe[0, 0])))

        self.z[5, 1] = self.z[5, 0] + (self.dt / self.tau) * (-self.z[5, 0] + max(0, self.lidar[2,0]*160 + (self.lidar[4,1]<0.3)*((self.ang_s - self.ang_p) - 20)))
        self.z[6, 1] = self.z[6, 0] + (self.dt / self.tau) * (-self.z[6, 0] + max(0, self.lidar[3,0]*160 + (self.lidar[4,1]<0.3)*((self.ang_p - self.ang_s) - 20)))

        self.z[7, 1] = self.z[7, 0] + (self.dt / self.tau) * (-self.z[7, 0] + max(0, (self.z[5, 0] + self.z[3, 0] - self.w*self.z[4, 0])))
        self.z[8, 1] = self.z[8, 0] + (self.dt / self.tau) * (-self.z[8, 0] + max(0, (self.z[5, 0] + self.z[4, 0] - self.w*self.z[3, 0])))
        self.z[9, 1] = self.z[9, 0] + (self.dt / self.tau) * (-self.z[9, 0] + max(0, (self.z[4, 0] + self.z[6, 0] - self.w*self.z[3, 0])))
        self.z[10, 1] = self.z[10, 0] + (self.dt / self.tau) * (-self.z[10, 0] + max(0, (self.z[3, 0] + self.z[6, 0] - self.w*self.z[4, 0])))

        self.z[11, 1] = self.z[11, 0] + (self.dt / self.tau) * (-self.z[11, 0] + max(0, (self.z[7, 0] + self.z[9, 0])))
        self.z[12, 1] = self.z[12, 0] + (self.dt / self.tau) * (-self.z[12, 0] + max(0, (self.z[10, 0] + self.z[8, 0])))
        self.z[13, 1] = self.z[13, 0] + (self.dt / self.tau) * (-self.z[13, 0] + max(0, (-self.w*abs(cmd_ang)*self.z[11, 0] - self.w*abs(cmd_ang)*self.z[12, 0] -2*self.w*self.z[17,0] + self.cte + self.Gpe[2,0])))

        self.z[14, 1] = self.z[14, 0] + (self.dt / self.tau) * (-self.z[14, 0] + (self.A * max(0, (100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[2, 0] - self.w*self.z[15, 0] - self.w*self.z[16, 0] ))**2) / (self.Sigma**2 + (100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[2, 0] - self.w*self.z[15, 0] - self.w*self.z[16, 0] )**2))
        self.z[15, 1] = self.z[15, 0] + (self.dt / self.tau) * (-self.z[15, 0] + (self.A * max(0, (-self.Gpe[1,0]*100 -self.cte + self.z[4, 0]*self.w + 2*self.z[0,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[16, 0] ))**2) / (self.Sigma**2 + (-self.Gpe[1,0]*100 -self.cte + self.z[4, 0]*self.w + 2*self.z[0,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[16, 0] )**2))
        self.z[16, 1] = self.z[16, 0] + (self.dt / self.tau) * (-self.z[16, 0] + (self.A * max(0, (self.z[3, 0] - 100*self.Gpe[1,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[15, 0]*1.5 + 5*self.z[1,0] + 5*self.z[2,0] + self.cte ))**2) / (self.Sigma**2 + (self.z[3, 0] - 100*self.Gpe[1,0] - self.w*self.z[14, 0]*1.5 - self.w*self.z[15, 0]*1.5 + 5*self.z[1,0] + 5*self.z[2,0] + self.cte )**2))
        
        self.z[17, 1] = self.z[17, 0] + (self.dt / self.tau) * (-self.z[17, 0] + max(0, (self.Gpe[2,0] - self.Area)))

        cmd_ang = (self.z[11,0]*(self.lidar[4,0] < 0.3)) - (self.z[12,0]*(self.lidar[4,0]<0.3))
        cmd_lateral = (self.lidar[2,0]*1.5 + self.lidar[3,0]*1.5 + self.z[11,0]*((self.Gpe[1,0] > 0.5)and(self.Gpe[2,0]<0.5))) - (self.z[12,0]*((self.Gpe[1,0] > 0.5)and(self.Gpe[2,0]<0.5)))
        cmd_lineal = self.lidar[0,0]*1.5 - self.lidar[1,0]*1.5 + self.z[13,0] -self.j*self.z[4,0]*(self.z[5,0] < self.epsilem and self.z[6,0] < self.epsilem)

        for i in range(len(self.z)): self.z[i, 0] = self.z[i,1]*(self.z[i,1]>self.epsilem)
        for i in range(len(self.lidar)): self.lidar[i, 0] = self.lidar[i,1]*(self.lidar[i,1]>self.epsilem)
        for i in range(len(self.Response)): self.Response[i, 0] = self.Response[i,1]*(self.Response[i,1]>self.epsilem)
        for i in range(len(self.Aux)): self.Aux[i, 0] = self.Aux[i,1]*(self.Aux[i,1]>self.epsilem)
        for i in range(len(self.StN)): self.StN[i, 0] = self.StN[i,1]*(self.StN[i,1]>self.epsilem)
        for i in range(len(self.Gpi)): self.Gpi[i, 0] = self.Gpi[i,1]*(self.Gpi[i,1]>self.epsilem)
        for i in range(len(self.Gpe)): self.Gpe[i, 0] = self.Gpe[i,1]*(self.Gpe[i,1]>self.epsilem)
        for i in range(len(self.StR)): self.StR[i, 0] = self.StR[i,1]*(self.StR[i,1]>self.epsilem)

        # ------------------- PRINTS -------------------------
        
        print("R: ", str(R))
        print("G: ", str(G))
        print("B: ", str(B)) 

        print(
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
                        f"17: {self.z[17,1]}\n"
                        f"0: {self.z[0,1]}\n"
                        f"1: {self.z[1,1]}\n"
                        f"2: {self.z[2,1]}\n"
                        f"roll: {self.roll}\n"
                        f"pitch: {self.pitch}\n"
                        f"STD total: {self.accel_std:.3f}"
                        )
        
        print("cmd_ang: ", str(cmd_ang))
        print("cmd_lineal: ", str(cmd_lineal))
        print("cmd_lateral: ", str(cmd_lateral))

        print("lidar frente: ", str(self.lidar[0,0]))
        print("lidar atras: ", str(self.lidar[1,0]))
        print("lidar izquierda: ", str(self.lidar[2,0]))
        print("lidar derecha:", str(self.lidar[3,0]))
        print("lidar 4:", str(self.lidar[4,0]))

        cmd_ang = self.limit(cmd_ang, 1)
        cmd_lineal = self.limit(cmd_lineal, 5)
        cmd_lateral = self.limit(cmd_lateral, 5)

    #------------------------- TERRAIN CHANGER -----------------------------

        if self.accel_std > self.Usigma_az and not self.terrainchanger:
        #if (self.accel_std > self.Usigma_az or self.low_accel_flag)and not self.terrainchanger:  # Descomentar si el robot se queda atascado mucho
            print("Terreno rocoso detectado 🚧")
            self.terrainchanger = True
            self.std_dev_accel_z = 6
            self.terrain_timer = time.time()  # Guardar momento de activación

        # Si está activo, verificar si pasaron 8 segundos
        if self.terrainchanger:
            elapsed = time.time() - self.terrain_timer
            if elapsed < 16:
                print("Terreno rocoso detectado 🚧")
                self.std_dev_accel_z = 6
            else:
                self.terrainchanger = False  # Volver a estado normal
                print("Terreno liso 🛣️")
                self.std_dev_accel_z = 0

        # Si no está activo y no hay vibración, mensaje normal
        elif not self.terrainchanger:
            self.std_dev_accel_z = 0
            print("Terreno liso 🛣️")

        if self.pitch > self.Upitch: print("Terreno inclinado")
        else: print("Terreno NO inclinado")

        #------------------------- P U B L I C A C I O N --------------------------------------#
        
        self.publicarGPE(self.Gpe[0,1], self.Gpe[1,1], self.Gpe[2,1])
        
        # -------------------- PUBLICACIÓN (una sola vez por ciclo) --------------------
        
        if self.MOVEMENT:
            # 1) calcular magnitudes filtradas por umbral
            ang = cmd_ang     if abs(cmd_ang)     > self.epsilem else 0.0
            lat = cmd_lateral if abs(cmd_lateral) > self.epsilem else 0.0
            lin = cmd_lineal  if abs(cmd_lineal)  > self.epsilem else 0.0

            # 2) resolver prioridad y construir UN solo Twist
            tw_ang, tw_lat, tw_lin = 0.0, 0.0, 0.0

            if self.z[17,1] > 0.25:
                # STOP: todo a cero
                print("Stop")
            elif ang != 0.0:
                # prioridad 1: giro
                tw_ang = ang
                print("Giro Izquierda" if ang > 0 else "Giro Derecha")
            elif lat != 0.0 and self.z[16,1] < 0.5:
                # prioridad 2: desplazamiento lateral
                tw_lat = lat
                print("Desp. Izquierda" if lat > 0 else "Desp. Derecha")
            elif lin != 0.0:
                # prioridad 3: avance/retroceso
                tw_lin = lin
                print("Avanza" if lin > 0 else "Retrocede")
            else:
                # sin comando significativo → queda todo en 0
                pass

            # 3) publicar una sola vez
            self.publish_twist(linear_x=tw_lin, linear_y=tw_lat, angular_z=tw_ang)
            # ------------------------------------------------------------------------------

            # modos
            if self.z[15,1] > 0.5:
                self.publish_mode('C'); print("Cuadrupedo")
            elif self.z[16,1] > 0.5:
                self.publish_mode('H'); print("Móvil H")
            elif self.z[14,1] > 0.5:
                self.publish_mode('X'); print("Móvil X")

            self.publish_data()
            self.publish_imu()

    #------------------------- F U N C I O N E S    A U X I L I A R E S --------------------------------------#

    from std_msgs.msg import Float64

    def publicarGPE(self, gper=0.0, gpeg=0.0, gpeb=0.0):
        msg_r = Float64()
        msg_g = Float64()
        msg_b = Float64()
        
        msg_r.data = gper
        msg_g.data = gpeg
        msg_b.data = gpeb

        self.GPEr.publish(msg_r)
        self.GPEg.publish(msg_g)
        self.GPEb.publish(msg_b)

        # Callbacks de cada estímulo
    def red_callback(self, msg):
        if len(msg.data) >= 2:
            self.posR = msg.data[0]  # Posición en el rango de 0 a 180
            self.areaBoundingBoxR = msg.data[1]  # Área aproximada
            # print(f'Red - Pos: {self.posR}, Área: {self.areaBoundingBoxR}')

    def green_callback(self, msg):
        if len(msg.data) >= 2:
            self.posG = msg.data[0]
            self.areaBoundingBoxG = msg.data[1]
            # print(f'Green - Pos: {self.posG}, Área: {self.areaBoundingBoxG}')

    def blue_callback(self, msg):
        if len(msg.data) >= 2:
            self.posB = msg.data[0]
            self.areaBoundingBoxB = msg.data[1]
            # print(f'Blue - Pos: {self.posB}, Área: {self.areaBoundingBoxB}')
    
    def imu_callback(self, msg):
        # Extraer cuaterniones
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Calcular ángulos Euler
        self.roll = 180 - abs(np.degrees(np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))))
        self.pitch = abs(np.degrees(np.arcsin(2*(qw*qy - qz*qx))))

        # === MÉTODOS DE VIBRACIÓN ===

        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_mag = np.sqrt(ax**2 + ay**2 + az**2)

        # --- Guardar en buffers ---
        self.accel_buffer.append(accel_mag)

        if self.ignore_imu:
            if time.time() - self.ignore_timer < self.ignore_duration:
                # Mientras dure el tiempo, forzar el valor a 0 o al último
                self.accel_std = 0.0  # o self.last_accel_std si prefieres el anterior
                return
            else:
                # Termina la ignorancia
                self.ignore_imu = False

            # Contar lecturas bajas consecutivas
        if self.accel_std < self.low_accel_threshold:
            self.low_accel_counter += 1
        else:
            self.low_accel_counter = 0  # Reinicia si rompe la secuencia

        # Activar bandera si llegó al límite
        if self.low_accel_counter >= self.low_accel_limit:
            self.low_accel_flag = True
        else:
            self.low_accel_flag = False

        # --- STD magnitud total ---
        self.accel_std = np.std(self.accel_buffer) if len(self.accel_buffer) > 1 else 0.0
        self.accel_std2 = np.std(self.accel_buffer) if len(self.accel_buffer) > 1 else 0.0

        # Mostrar información
        # print(f"Roll: {self.roll:.2f}°, Pitch: {self.pitch:.2f}°, Aceleración Z: {self.accel_z:.2f} m/s², STD Z: {std_dev_accel_z:.4f}")

    def publish_twist(self, linear_x=None, linear_y=None, angular_z=None):
        if not hasattr(self, "_twist"):
            self._twist = Twist()  # estado persistente

        if linear_x is not None:
            self._twist.linear.x = float(linear_x)
        if linear_y is not None:
            self._twist.linear.y = float(linear_y)
        if angular_z is not None:
            self._twist.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(self._twist)

    def publish_mode(self, mode):
        """Publica el modo actual."""
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        print(f"IGNOREIMU: {self.ignore_imu}")
        if self.current_mode != mode and (self.current_mode == "C" or self.current_mode == "H"): 
            self.ignore_timer = time.time()  
            self.ignore_imu = True
        self.current_mode = mode

    def publish_data(self):

        parts = [
            np.asarray(self.Gpi)[:, 1],
            np.asarray(self.Gpe)[:, 1],
            np.asarray(self.StN)[:, 1],
            np.asarray(self.StR)[:, 1],
            np.asarray(self.z)[:, 1],
            np.asarray(self.lidar)[:, 1],
            np.asarray(self.activaciones_totales),
            np.asarray(self.Response)[:, 1],
            np.asarray(self.Aux)[:, 1],
        ]

        vec = np.concatenate([p.ravel() for p in parts]).astype(np.float32)
        msg = Float32MultiArray()
        msg.data = vec.tolist()
        self.publisher_.publish(msg)

    def publish_imu(self):

        parts = [
            np.asarray(self.accel_std2),
            np.asarray(self.pitch),
        ]

        vec = np.concatenate([p.ravel() for p in parts]).astype(np.float32)
        msg = Float32MultiArray()
        msg.data = vec.tolist()
        self.publisher_imu.publish(msg)

#------------------------- M A I N --------------------------------------#

def main(args=None):
    rclpy.init(args=args)
    node = NetworkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()