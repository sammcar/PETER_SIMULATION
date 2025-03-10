#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pandas as pd
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
        
class NetworkPublisher(Node):

    def __init__(self):
        super().__init__('network_publisher')
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/peter_mode', 10)

        # Velocidades predeterminadas
        self.speed = 3.0  # Velocidad lineal
        self.turn = 3.0   # Velocidad angular

        # Limite para velocidad lineal
        self.max_speed = 3.5
        self.min_speed = 0.1

        # Limite para velocidad angular
        self.max_turn = 3.5
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
        self.create_subscription(Float32MultiArray, '/bounding_box/red', self.red_callback, 10)
        self.create_subscription(Float32MultiArray, '/bounding_box/green', self.green_callback, 10)
        self.create_subscription(Float32MultiArray, '/bounding_box/blue', self.blue_callback, 10)


        # Constantes y variables de dinámica neuronal
        self.initctes()

        # Timer para llamar la función de control cada segundo
        self.timer = self.create_timer(1.0, self.run_network)




    def initctes(self):


        #-------------------------N E U R O N A S--------------------------------------#

        self.Gpi = np.zeros((3, 2)) # Globo Pálido Interno
        self.Gpe = np.zeros((3, 2)) # Globo Pálido Externo
        self.STN = np.zeros((3, 2)) # Subtalámico
        self.STR = np.zeros((6, 2)) # Estriado
        self.z = np.zeros((18, 2)) # Red Sam/Espitia


        #------------------------- C O N S T A N T E S --------------------------------------#

        self.ang_p = 90 # Posicion Frente del Robot
        self.epsilem = 0.1 # Tolerancia
        self.dt = 1 # Intervalo de Integracion
        self.cte = 1 # Constante de Avance
        self.Area = 100000 # Area Limite

        self.w = 10 # Peso Sinaptico
        self.j = 2 # Peso Sinaptico

        self.A = 1 # Parametro Saturacion Naka
        self.B = 0.5 # Parametro Inhibicion Naka

        self.tau = 5 # Tao Neuronas Z
        self.TaoGpi = 1 # Tao Ganglios
        self.TaoGpe = 2 # Tao Ganglios
        self.TaoSTN = 2 # Tao Ganglios
        self.TaoSTR = 1 # Tao Ganglios

    def run_network(self):


        #------------------------E S T I M U L O -------------------------------------#

        # Comportamiento esperado: Huye del depredador (Red) em movil H. Caza a la presa (Blue) en cuadrupedo.
        # Evita al obstaculo (Green) en modo omnidireccional. Si no hay estimulo avanza en modo omnidireccional.
        # Estimulo en frente es posicion entre 70 y 110. Estimulo a la izquierda son valores entre 111 y 180.
        # Estimulo a la derecha son valores entre 0 y 69


        cmd_lineal = 0.0 # Comando de Velocidad Lineal
        cmd_ang = 0.0 # Comando de Velocidad Angular
        cmd_lateral = 0.0 # Comando de Velocidad Lateral

        #------------------------- D I N A M I C A --------------------------------------#

        R = self.areaBoundingBoxR
        G = self.areaBoundingBoxG
        B = self.areaBoundingBoxB

        self.STN[0, 1] = np.clip((self.STN[0, 0] + (1/self.TaoSTN)*(-self.STN[0, 0] + R - self.Gpi[0,0] - self.Gpe[1,0] - self.Gpe[2,0])),0, None)
        self.STN[1, 1] = np.clip((self.STN[1, 0] + (1/self.TaoSTN)*(-self.STN[1, 0] + G - self.Gpi[1,0] - self.Gpe[0,0] - self.Gpe[2,0])),0, None)
        self.STN[2, 1] = np.clip((self.STN[2, 0] + (1/self.TaoSTN)*(-self.STN[2, 0] + B - self.Gpi[2,0] - self.Gpe[0,0] - self.Gpe[1,0])),0, None)
        
        self.Gpi[0, 1] = np.clip((self.Gpi[0, 0] + (1/self.TaoGpi)*(-self.Gpi[0, 0] + self.STN[1,0] + self.STN[2,0] - self.Gpe[0,0] - self.STR[0,0])),0, None)
        self.Gpi[1, 1] = np.clip((self.Gpi[1, 0] + (1/self.TaoGpi)*(-self.Gpi[1, 0] + self.STN[0,0] + self.STN[2,0] - self.Gpe[1,0] - self.STR[1,0])),0, None)
        self.Gpi[2, 1] = np.clip((self.Gpi[2, 0] + (1/self.TaoGpi)*(-self.Gpi[2, 0] + self.STN[0,0] + self.STN[1,0] - self.Gpe[2,0] - self.STR[2,0])),0, None)
        
        self.Gpe[0, 1] = np.clip((self.Gpe[0, 0] + (1/self.TaoGpe)*(-self.Gpe[0, 0] + self.STN[0,0])),0, None)
        self.Gpe[1, 1] = np.clip((self.Gpe[1, 0] + (1/self.TaoGpe)*(-self.Gpe[1, 0] + self.STN[1,0])),0, None)
        self.Gpe[2, 1] = np.clip((self.Gpe[2, 0] + (1/self.TaoGpe)*(-self.Gpe[2, 0] + self.STN[2,0])),0, None)
        
        self.STR[0, 1] = np.clip((self.STR[0, 0] + (1/self.TaoSTR)*(-self.STR[0, 0] + self.j*self.STN[0,0])),0, None)
        self.STR[1, 1] = np.clip((self.STR[1, 0] + (1/self.TaoSTR)*(-self.STR[1, 0] + self.STN[1,0])),0, None)
        self.STR[2, 1] = np.clip((self.STR[2, 0] + (1/self.TaoSTR)*(-self.STR[2, 0] + self.STN[2,0])),0, None)

        if self.Gpe[0,1] > 0.5:
            ang_s = self.posR
        elif self.Gpe[1,1] > 0.5:
            ang_s = self.posG
        elif self.Gpe[2,1] > 0.5:
            ang_s = self.posB
        else:
            ang_s = 90.0

        self.z[3, 1] = self.z[3, 0] + (self.dt / self.tau) * (-self.z[3, 0] + max(0, (self.Gpe[2,0])))
        self.z[4, 1] = self.z[4, 0] + (self.dt / self.tau) * (-self.z[4, 0] + max(0, (self.Gpe[1, 0] + self.j * self.Gpe[0, 0])))

        self.z[5, 1] = self.z[5, 0] + (self.dt / self.tau) * (-self.z[5, 0] + max(0, (ang_s - self.ang_p) - 20))
        self.z[6, 1] = self.z[6, 0] + (self.dt / self.tau) * (-self.z[6, 0] + max(0, (self.ang_p - ang_s) - 20))

        self.z[7, 1] = self.z[7, 0] + (self.dt / self.tau) * (-self.z[7, 0] + max(0, (self.z[5, 0] + self.z[3, 0] - self.w*self.z[4, 0])))
        self.z[8, 1] = self.z[8, 0] + (self.dt / self.tau) * (-self.z[8, 0] + max(0, (self.z[5, 0] + self.z[4, 0] - self.w*self.z[3, 0])))
        self.z[9, 1] = self.z[9, 0] + (self.dt / self.tau) * (-self.z[9, 0] + max(0, (self.z[4, 0] + self.z[6, 0] - self.w*self.z[3, 0])))
        self.z[10, 1] = self.z[10, 0] + (self.dt / self.tau) * (-self.z[10, 0] + max(0, (self.z[3, 0] + self.z[6, 0] - self.w*self.z[4, 0])))

        self.z[11, 1] = self.z[11, 0] + (self.dt / self.tau) * (-self.z[11, 0] + max(0, (self.z[7, 0] + self.z[9, 0])))
        self.z[12, 1] = self.z[12, 0] + (self.dt / self.tau) * (-self.z[12, 0] + max(0, (self.z[10, 0] + self.z[8, 0])))
        self.z[13, 1] = self.z[13, 0] + (self.dt / self.tau) * (-self.z[13, 0] + max(0, (-self.w*abs(cmd_ang)*self.z[11, 0] - self.w*abs(cmd_ang)*self.z[12, 0] -self.w*self.z[17,0] + self.cte)))

        self.z[14, 1] = self.z[14, 0] + (self.dt / self.tau) * (-self.z[14, 0] + (self.A * max(0, (self.cte + 100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[1, 0] - self.w*self.Gpi[2, 0]))**2) / (B**2 + (self.cte + 100*self.Gpe[1,0] - self.w*self.Gpi[0, 0] - self.w*self.Gpi[1, 0] - self.w*self.Gpi[2, 0])**2))
        self.z[15, 1] = self.z[15, 0] + (self.dt / self.tau) * (-self.z[15, 0] + (self.A * max(0, (self.z[3, 0]))**2) / (B**2 + (self.z[3, 0])**2))
        self.z[16, 1] = self.z[16, 0] + (self.dt / self.tau) * (-self.z[16, 0] + (self.A * max(0, (self.z[4, 0] - self.j*self.Gpe[1,0]))**2) / (B**2 + (self.z[4, 0] - self.j*self.Gpe[1,0])**2))
        self.z[17, 1] = self.z[17, 0] + (self.dt / self.tau) * (-self.z[17, 0] + max(0, (self.Gpe[2,0] - self.Area))) 

        cmd_ang = (self.z[11,0]*(self.Gpe[1,0] < 0.5)) - (self.z[12,0]*(self.Gpe[1,0]<0.5))
        cmd_lateral = (self.z[11,0]*(self.Gpe[1,0] > 0.5)) - (self.z[12,0]*(self.Gpe[1,0] > 0.5))
        cmd_lineal = self.z[13,0] -self.j*self.z[4,0]*(self.z[5,0] < self.epsilem and self.z[6,0] < self.epsilem)

        for i in range(len(self.z)): self.z[i, 0] = self.z[i,1]
        for i in range(len(self.STN)): self.STN[i, 0] = self.STN[i,1]
        for i in range(len(self.Gpi)): self.Gpi[i, 0] = self.Gpi[i,1]
        for i in range(len(self.Gpe)): self.Gpe[i, 0] = self.Gpe[i,1]
        for i in range(len(self.STR)): self.STR[i, 0] = self.STR[i,1]

        #------------------------- P U B L I C A C I O N --------------------------------------#

        if self.epsilem < cmd_ang:
            self.publish_twist(angular_z=self.turn)  # Gira izquierda
            print("Giro Izquierda")
        elif cmd_ang < -self.epsilem:
            self.publish_twist(angular_z=-self.turn)  # Gira derecha
            print("Giro Derecha")
        else:
            pass
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
        if self.z[14,1] > 0.5:
            self.publish_mode('X')
            print("Móvil X")
        elif self.z[4,1] > 0.5:
            self.publish_mode('H')
            print("Móvil H")
        elif self.z[3,1] > 0.5:
            self.publish_mode('C')
            print("Cuadrupedo")
        else:
            pass

    #------------------------- F U N C I O N E S    A U X I L I A R E S --------------------------------------#
        # Callbacks de cada estímulo
    def red_callback(self, msg):
        if len(msg.data) >= 2:
            self.posR = msg.data[0]  # Posición en el rango de 0 a 180
            self.areaBoundingBoxR = msg.data[1]  # Área aproximada
            self.get_logger().info(f'Red - Pos: {self.posR}, Área: {self.areaBoundingBoxR}')

    def green_callback(self, msg):
        if len(msg.data) >= 2:
            self.posG = msg.data[0]
            self.areaBoundingBoxG = msg.data[1]
            self.get_logger().info(f'Green - Pos: {self.posG}, Área: {self.areaBoundingBoxG}')

    def blue_callback(self, msg):
        if len(msg.data) >= 2:
            self.posB = msg.data[0]
            self.areaBoundingBoxB = msg.data[1]
            self.get_logger().info(f'Blue - Pos: {self.posB}, Área: {self.areaBoundingBoxB}')


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
