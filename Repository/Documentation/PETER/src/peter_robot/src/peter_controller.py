#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')

        self.get_logger().info("\033[92mPeter Controller by Sam Activated! :D\033[0m")

        # Publicadores para las posiciones y velocidades
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/gazebo_velocity_controllers/commands', 10)

        # Suscriptores
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/peter_mode', self.peter_mode_callback, 10)

        # Inicialización de articulaciones y velocidades
        self.joint_positions = [0.0] * 12  # 12 articulaciones
        self.joint_velocities = [0.0] * 4  # 4 ruedas

        # Variables
        self.increment = 0.1
        self.increment_velocity = 0.5
        self.state = 'C'
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.target_positions = [0.0] * 12
        self.target_velocities = [0.0] * 4

        # Timer
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        self.update_positions()
        self.update_velocities()
        position_msg = Float64MultiArray()
        position_msg.data = self.joint_positions
        self.position_publisher_.publish(position_msg)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = self.joint_velocities
        self.velocity_publisher_.publish(velocity_msg)

    def update_positions(self):
        """Actualiza las posiciones de las articulaciones hacia sus objetivos de forma incremental."""
        for i in range(len(self.joint_positions)):
            if self.joint_positions[i] < self.target_positions[i]:
                self.joint_positions[i] = min(self.joint_positions[i] + self.increment, self.target_positions[i])
            elif self.joint_positions[i] > self.target_positions[i]:
                self.joint_positions[i] = max(self.joint_positions[i] - self.increment, self.target_positions[i])

    def update_velocities(self):
        """Actualiza las velocidades de las ruedas hacia sus objetivos de forma incremental."""
        for i in range(len(self.joint_velocities)):
            if self.joint_velocities[i] < self.target_velocities[i]:
                self.joint_velocities[i] = min(self.joint_velocities[i] + self.increment_velocity, self.target_velocities[i])
            elif self.joint_velocities[i] > self.target_velocities[i]:
                self.joint_velocities[i] = max(self.joint_velocities[i] - self.increment_velocity, self.target_velocities[i])

    def cmd_vel_callback(self, msg):
        """Callback para actualizar los comandos de velocidad."""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

        if self.state == 'X':  # Modo omnidireccional
            self.target_velocities = [
                self.linear_x + self.angular_z + self.linear_y,  # RU
                self.linear_x - self.angular_z - self.linear_y,  # LU
                self.linear_x + self.angular_z - self.linear_y,  # RD
                self.linear_x - self.angular_z + self.linear_y   # LD
            ]
        elif self.state == 'H':  # Modo móvil tipo H
            self.target_velocities = [
                self.linear_x + self.angular_z,  # RU
                self.linear_x - self.angular_z,  # LU
                self.linear_x + self.angular_z,  # RD
                self.linear_x - self.angular_z   # LD
            ]
        else:  # Modo C (reposo en las ruedas)
            self.target_velocities = [0.0] * 4

    def peter_mode_callback(self, msg):
        """Callback para actualizar el modo."""
        mode = msg.data.upper()
        if mode in ['C', 'X', 'H']:
            self.state = mode
            self.update_target_positions()

    def update_target_positions(self):
        """Actualiza las posiciones objetivo de las articulaciones según el modo."""
        if self.state == 'C':  # Modo cuadrúpedo

            # /////////////////////////// MODIFICAR AQUI PARA LA CAMINATA EN MODO CUADRUPEDO ///////////////////////////////
            self.target_positions = [0.0] * 12
            #///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            
        elif self.state == 'X':  # Modo omnidireccional
            self.target_positions = [
                0.785, 0.0, -1.40,  # CoxisRU, FemurRU, TibiaRU
                -0.785, 0.0, -1.40,  # CoxisLU, FemurLU, TibiaLU
                -0.785, 0.0, -1.40,  # CoxisRD, FemurRD, TibiaRD
                0.785, 0.0, -1.40   # CoxisLD, FemurLD, TibiaLD
            ]
        elif self.state == 'H':  # Modo móvil tipo H
            self.target_positions = [
                0.0, 0.0, -1.40,  # CoxisRU, FemurRU, TibiaRU
                0.0, 0.0, -1.40,  # CoxisLU, FemurLU, TibiaLU
                0.0, 0.0, -1.40,  # CoxisRD, FemurRD, TibiaRD
                0.0, 0.0, -1.40   # CoxisLD, FemurLD, TibiaLD
            ]

def main(args=None):
    rclpy.init(args=args)
    joint_position_publisher = JointPositionPublisher()

    try:
        rclpy.spin(joint_position_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
