#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pandas as pd
import numpy as np

        
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

        #-------------------------E S T I M U L O -------------------------------------#

        # Comportamiento esperado: Huye del depredador (Red) em movil H. Caza a la presa (Blue) en cuadrupedo.
        # Evita al obstaculo (Green) en modo omnidireccional. Si no hay estimulo avanza en modo omnidireccional.
        # Estimulo en frente es posicion entre 70 y 110. Estimulo a la izquierda son valores entre 111 y 180.
        # Estimulo a la derecha son valores entre 0 y 69


        # Intensidad de estimulo
        areaBoundingBoxR = 0.0
        areaBoundingBoxG = 0.0
        areaBoundingBoxB = 0.0

        # Posicion de estimulo
        posR = 0.0
        posG = 0.0
        posB = 0.0

        #-------------------------N E U R O N A S--------------------------------------#

        GPi = np.zeros((3, 2)) # Globo Pálido Interno
        GPe = np.zeros((3, 2)) # Globo Pálido Externo
        STN = np.zeros((3, 2)) # Subtalámico
        STR = np.zeros((6, 2)) # Estriado
        z = np.zeros((18, 2)) # Red Sam/Espitia

        #------------------------- C O N S T A N T E S --------------------------------------#

        ang_p = 90 # Posicion Frente del Robot
        epsilem = 0.1 # Tolerancia
        dt = 1 # Intervalo de Integracion
        cte = 1 # Constante de Avance
        Area = 100000 # Area Limite

        w = 10 # Peso Sinaptico
        j = 2 # Peso Sinaptico

        A = 1 # Parametro Saturacion Naka
        B = 0.5 # Parametro Inhibicion Naka

        tau = 5 # Tao Neuronas Z
        TaoGPi = 1 # Tao Ganglios
        TaoGPe = 2 # Tao Ganglios
        TaoSTN = 2 # Tao Ganglios
        TaoSTR = 1 # Tao Ganglios

        cmd_lineal = 0.0 # Comando de Velocidad Lineal
        cmd_ang = 0.0 # Comando de Velocidad Angular
        cmd_lateral = 0.0 # Comando de Velocidad Lateral

        #------------------------- D I N A M I C A --------------------------------------#

        while(True):

            R = areaBoundingBoxR
            G = areaBoundingBoxG
            B = areaBoundingBoxB

            STN[0, 1] = np.clip((STN[0, 0] + (1/TaoSTN)*(-STN[0, 0] + R - GPi[0,0] - GPe[1,0] - GPe[2,0])),0, None)
            STN[1, 1] = np.clip((STN[1, 0] + (1/TaoSTN)*(-STN[1, 0] + G - GPi[1,0] - GPe[0,0] - GPe[2,0])),0, None)
            STN[2, 1] = np.clip((STN[2, 0] + (1/TaoSTN)*(-STN[2, 0] + B - GPi[2,0] - GPe[0,0] - GPe[1,0])),0, None)
            
            GPi[0, 1] = np.clip((GPi[0, 0] + (1/TaoGPi)*(-GPi[0, 0] + STN[1,0] + STN[2,0] - GPe[0,0] - STR[0,0])),0, None)
            GPi[1, 1] = np.clip((GPi[1, 0] + (1/TaoGPi)*(-GPi[1, 0] + STN[0,0] + STN[2,0] - GPe[1,0] - STR[1,0])),0, None)
            GPi[2, 1] = np.clip((GPi[2, 0] + (1/TaoGPi)*(-GPi[2, 0] + STN[0,0] + STN[1,0] - GPe[2,0] - STR[2,0])),0, None)
            
            GPe[0, 1] = np.clip((GPe[0, 0] + (1/TaoGPe)*(-GPe[0, 0] + STN[0,0])),0, None)
            GPe[1, 1] = np.clip((GPe[1, 0] + (1/TaoGPe)*(-GPe[1, 0] + STN[1,0])),0, None)
            GPe[2, 1] = np.clip((GPe[2, 0] + (1/TaoGPe)*(-GPe[2, 0] + STN[2,0])),0, None)
            
            STR[0, 1] = np.clip((STR[0, 0] + (1/TaoSTR)*(-STR[0, 0] + j*STN[0,0])),0, None)
            STR[1, 1] = np.clip((STR[1, 0] + (1/TaoSTR)*(-STR[1, 0] + STN[1,0])),0, None)
            STR[2, 1] = np.clip((STR[2, 0] + (1/TaoSTR)*(-STR[2, 0] + STN[2,0])),0, None)

            if GPe[0,1] > 0.5:
                ang_s = posR
            elif GPe[1,1] > 0.5:
                ang_s = posG
            elif GPe[2,1] > 0.5:
                ang_s = posB
            else:
                ang_s = 90.0

            z[3, 1] = z[3, 0] + (dt / tau) * (-z[3, 0] + max(0, (GPe[2,0])))
            z[4, 1] = z[4, 0] + (dt / tau) * (-z[4, 0] + max(0, (GPe[1, 0] + j * GPe[0, 0])))

            z[5, 1] = z[5, 0] + (dt / tau) * (-z[5, 0] + max(0, (ang_s - ang_p) - 20))
            z[6, 1] = z[6, 0] + (dt / tau) * (-z[6, 0] + max(0, (ang_p - ang_s) - 20))

            z[7, 1] = z[7, 0] + (dt / tau) * (-z[7, 0] + max(0, (z[5, 0] + z[3, 0] - w*z[4, 0])))
            z[8, 1] = z[8, 0] + (dt / tau) * (-z[8, 0] + max(0, (z[5, 0] + z[4, 0] - w*z[3, 0])))
            z[9, 1] = z[9, 0] + (dt / tau) * (-z[9, 0] + max(0, (z[4, 0] + z[6, 0] - w*z[3, 0])))
            z[10, 1] = z[10, 0] + (dt / tau) * (-z[10, 0] + max(0, (z[3, 0] + z[6, 0] - w*z[4, 0])))

            z[11, 1] = z[11, 0] + (dt / tau) * (-z[11, 0] + max(0, (z[7, 0] + z[9, 0])))
            z[12, 1] = z[12, 0] + (dt / tau) * (-z[12, 0] + max(0, (z[10, 0] + z[8, 0])))
            z[13, 1] = z[13, 0] + (dt / tau) * (-z[13, 0] + max(0, (-w*abs(cmd_ang)*z[11, 0] - w*abs(cmd_ang)*z[12, 0] -w*z[17,0] + cte)))

            z[14, 1] = z[14, 0] + (dt / tau) * (-z[14, 0] + (A * max(0, (cte + 100*GPe[1,0] - w*GPi[0, 0] - w*GPi[1, 0] - w*GPi[2, 0]))**2) / (B**2 + (cte + 100*GPe[1,0] - w*GPi[0, 0] - w*GPi[1, 0] - w*GPi[2, 0])**2))
            z[15, 1] = z[15, 0] + (dt / tau) * (-z[15, 0] + (A * max(0, (z[3, 0]))**2) / (B**2 + (z[3, 0])**2))
            z[16, 1] = z[16, 0] + (dt / tau) * (-z[16, 0] + (A * max(0, (z[4, 0] - j*GPe[1,0]))**2) / (B**2 + (z[4, 0] - j*GPe[1,0])**2))
            z[17, 1] = z[17, 0] + (dt / tau) * (-z[17, 0] + max(0, (GPe[2,0] - Area))) 

            cmd_ang = (z[11,0]*(GPe[1,0] < 0.5)) - (z[12,0]*(GPe[1,0]<0.5))
            cmd_lateral = (z[11,0]*(GPe[1,0] > 0.5)) - (z[12,0]*(GPe[1,0] > 0.5))
            cmd_lineal = z[13,0] -j*z[4,0]*(z[5,0] < epsilem and z[6,0] < epsilem)

            for i in range(len(z)): z[i, 0] = z[i,1]
            for i in range(len(STN)): STN[i, 0] = STN[i,1]
            for i in range(len(GPi)): GPi[i, 0] = GPi[i,1]
            for i in range(len(GPe)): GPe[i, 0] = GPe[i,1]
            for i in range(len(STR)): STR[i, 0] = STR[i,1]

            #------------------------- P U B L I C A C I O N --------------------------------------#

            if epsilem < cmd_ang:
                self.publish_twist(angular_z=self.turn)  # Gira izquierda
                print("Giro Izquierda")
            elif cmd_ang < -epsilem:
                self.publish_twist(angular_z=-self.turn)  # Gira derecha
                print("Giro Derecha")
            else:
                pass
            #-------------------------------------------------------------------
            if epsilem < cmd_lineal:
                self.publish_twist(linear_x=self.speed)  # Adelante
                print("Avanza")
            elif cmd_lineal < -epsilem:
                self.publish_twist(linear_x=-self.speed) # Atrás
                print("Retrocede")
            elif z[17,1] > 0.5:
                self.publish_twist() # Stop
                print("Stop")
            else:
                pass
            #-------------------------------------------------------------------
            if epsilem < cmd_lateral:
                self.publish_twist(linear_y=self.speed) # Izquierda
                print("Desp. Izquierda")
            elif cmd_lateral < -epsilem:
                self.publish_twist(linear_y=-self.speed) # Derecha
                print("Desp. Derecha")
            else:
                pass
            #-------------------------------------------------------------------
            if z[14,1] > 0.5:
                self.publish_mode('X')
                print("Móvil X")
            elif z[4,1] > 0.5:
                self.publish_mode('H')
                print("Móvil H")
            elif z[3,1] > 0.5:
                self.publish_mode('C')
                print("Cuadrupedo")
            else:
                pass

    #------------------------- F U N C I O N E S    A U X I L I A R E S --------------------------------------#
            
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
