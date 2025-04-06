#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import time
from rclpy.executors import MultiThreadedExecutor
import threading
from ros_gz_interfaces.msg import Contacts
import re
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

# Robot parameters
LENGTH_A = 45.0
LENGTH_C = 68.5
LENGTH_B = 152.0
LENGTH_SIDE = 120.0
Z_ABSOLUTE = 26.0
servo_service_en = False
# Movement constants
Z_DEFAULT = LENGTH_B
Z_UP = 102.0
DIAG = math.sqrt(((LENGTH_A + LENGTH_C) ** 2) / 2)
SIDE = LENGTH_A + LENGTH_C
# Movement variables
site_now = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for current positions
site_expect = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for expected positions
temp_speed = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for temporary speeds
move_speed = 8.0
speed_multiple = 2.0
SPOT_TURN_SPEED = 4.0
LEG_MOVE_SPEED = 8.0
BODY_MOVE_SPEED = 3.0
STAND_SEAT_SPEED = 1.0
rest_counter = 0  # volatile not needed in Python
KEEP = 255.0

# Turn calculations
TEMP_A = math.sqrt((2 * DIAG + LENGTH_SIDE) ** 2 + DIAG ** 2)
TEMP_B = 2 * (0 + DIAG) + LENGTH_SIDE
TEMP_C = math.sqrt((2 * DIAG + LENGTH_SIDE) ** 2 + (2 * 0 + DIAG + LENGTH_SIDE) ** 2)
TEMP_ALPHA = math.acos((TEMP_A ** 2 + TEMP_B ** 2 - TEMP_C ** 2) / (2 * TEMP_A * TEMP_B))
TURN_X1 = (TEMP_A - LENGTH_SIDE) / 2
TURN_Y1 = 0 + DIAG / 2
TURN_X0 = TURN_X1 - TEMP_B * math.cos(TEMP_ALPHA)
TURN_Y0 = TEMP_B * math.sin(TEMP_ALPHA) - TURN_Y1 - LENGTH_SIDE

class JointPositionPublisher(Node):

    def iniciarCuadrupedo(self):
        global site_expect, site_now

        site_expect = [[DIAG,DIAG,Z_DEFAULT],
                        [SIDE,0,Z_DEFAULT],
                        [DIAG,DIAG,Z_DEFAULT],
                        [SIDE,0,Z_DEFAULT]]
        site_now = [[DIAG,DIAG,Z_DEFAULT],
                        [SIDE,0,Z_DEFAULT],
                        [DIAG,DIAG,Z_DEFAULT],
                        [SIDE,0,Z_DEFAULT]]
        self.target_positions = [0.0,   0.0, 0.0, 
                                 0.785, 0.0, 0.0,
                                 0.0,   0.0, 0.0,
                                -0.785, 0.0, 0.0]

    def __init__(self):
        super().__init__('joint_position_publisher')

        self.get_logger().info("\033[92mPeter Controller by Sam Activated! :D\033[0m")

        # Publicadores para las posiciones y velocidades
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/gazebo_joint_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/gazebo_velocity_controllers/commands', 10)

        # Suscriptores
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/peter_mode', self.peter_mode_callback, 10)
        self.create_subscription(Contacts, '/bumper/TibiaRU', self.bumper_callback, 10)
        self.create_subscription(Contacts, '/bumper/TibiaRD', self.bumper_callback, 10)
        self.create_subscription(Contacts, '/bumper/TibiaLU', self.bumper_callback, 10)
        self.create_subscription(Contacts, '/bumper/TibiaLD', self.bumper_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
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
        self.leg_ok = False
        self.leg = 0

        # Control dirección Omnidireccional
        self.current_angle = 0.0  # Último ángulo roll del IMU 
        self.target_angle = None   # Ángulo objetivo cuando ω = 0, antes de ir en linea recta
        self.kp = 0.35  # Ganancia proporcional
        self.inicio = False

        # Publisher de trayectoria
        self.path_pub = self.create_publisher(Path, "/trajectory", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Buffer y listener para transformaciones TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.ticker = self.create_timer(0.02, self.ticker_callback)
        self.on_air = [False,False,False,False]
        self.machine = 0
        # Start the state machine execution
        self.past = 'C'
        self.iniciarCuadrupedo()
        self.run_state_machine()

    def run_state_machine(self):
        """Non-blocking state machine execution loop."""
        while rclpy.ok():
            if self.state == 'C':
                self.process_state()
            rclpy.spin_once(self, timeout_sec=0.1)  # Keep ROS timers and callbacks responsive

    def process_state(self):
        global servo_service_en, site_now
        """Handles state transitions without blocking ROS execution."""
        #self.get_logger().info(f"Processing state: {self.machine}")

        if self.machine == 1:
            if site_now[2][1] != 0 and site_now[3][1] != 0:
                self.setFrontStep()
            servo_service_en = True
            self.step_forward(1)
        
        elif self.machine == 2:
            if site_now[2][1] != 0 and site_now[3][1] != 0:
                self.setFrontStep()
            servo_service_en = True
            self.step_back(1)
        
        elif self.machine == 3:
            if site_now[2][0] != 0 and site_now[0][0] != 0:
                self.setSideStep()
            servo_service_en = True
            self.step_left(1)

        elif self.machine == 4:
            if site_now[2][0] != 0 and site_now[0][0] != 0:
                self.setSideStep()
            servo_service_en = True
            self.step_right(1)

        elif self.machine == 5:
            if site_now[2][1] != 0 and site_now[3][1] != 0:
                self.setFrontStep()
            servo_service_en = True
            self.turn_left(1)

        elif self.machine == 6:
            if site_now[2][1] != 0 and site_now[3][1] != 0:
                self.setFrontStep()
            servo_service_en = True
            self.turn_right(1)

        elif self.machine == 7:
            self.setCenter()
            self.state = 'C'
            servo_service_en = True

        elif self.machine == 8:
            servo_service_en = False
        
        else:
            pass

    def inverseKinematics(self,x, y, z, a1=LENGTH_A, a2=LENGTH_C, a3=LENGTH_B):
        """
        Calculate joint angles theta1, theta2, and theta3 for a 3-DOF robot arm
        given the desired end-effector position (x, y, z) and link lengths a1, a2, and a3.
        """
        # Calculate theta1 (rotation around the base in the x-y plane)
        theta1 = math.atan2(y, x)

        # Calculate intermediate distances r and s
        r = math.sqrt(x**2 + y**2) - a1
        s = z

        # Calcular d, la distancia desde el hombro hasta la muñeca
        d = math.sqrt(r**2 + s**2)

        # Verificar si la posición deseada está al alcance
        if d > (a2 + a3) or d < abs(a2 - a3):
            print("Error: Posición fuera de alcance")
            return None

        # Calcular theta2 usando la ley de cosenos y el arctangent corregido
        cos_theta2 = (a2**2 + d**2 - a3**2) / (2 * a2 * d)
        theta2 = math.atan2(s, r) - math.acos(min(1, max(-1, cos_theta2)))  # Asegurar el dominio entre -1 y 1

        # Calcular theta3 usando la ley de cosenos con ajuste de ángulo interno
        cos_theta3 = (a2**2 + a3**2 - d**2) / (2 * a2 * a3)
        theta3 = math.pi - math.acos(min(1, max(-1, cos_theta3)))  # Asegurar el dominio entre

        # Convert angles from radians to degrees for readability
        # theta1_deg = math.degrees(theta1)
        # theta2_deg = math.degrees(theta2)
        # theta3_deg = math.degrees(theta3)

        return theta1, theta2, theta3

    def wait_all_reach(self):
        """
        Waits for all legs to reach their target positions by calling wait_reach
        for each leg (0 to 3).
        """
        for leg in range(4):  # Iterate over legs 0 to 3
            #self.get_logger().info("ESPERANDO PATA"+str(leg))
            self.wait_reach(leg)
    
    def wait_reach(self, leg):
        """
        Waits for a specific leg to reach its target position.

        Args:
        leg (int): The leg index to check (0 to 3).
        """
        global site_now, site_expect, servo_service_en  

        while not (site_now[leg][0] == site_expect[leg][0] and
                  site_now[leg][1] == site_expect[leg][1] and
                site_now[leg][2] == site_expect[leg][2]):

            #self.get_logger().info(f"Checking leg {leg} position: {site_now[leg]}")
            #self.get_logger().info(f"Desired leg {leg} position: {site_expect[leg]}")
            #self.get_logger().info(f"SERVICE: {servo_service_en}")
            rclpy.spin_once(self, timeout_sec=0.005)  # Allow callbacks to run
            #time.sleep(0.3)  # Let ROS process callbacks without spin_once()

    def set_site(self, leg, x, y, z):
        """
        Sets one of the endpoints' target positions.
        Updates temp_speed[4][3] with the calculated speeds.
        Non-blocking function.

        Args:
            leg (int): The leg index to update.
            x (float): Target x position, or KEEP to maintain current position.
            y (float): Target y position, or KEEP to maintain current position.
            z (float): Target z position, or KEEP to maintain current position.
        """
        global site_now, temp_speed, move_speed, speed_multiple, site_expect, KEEP

        # Initialize distance variables
        length_x, length_y, length_z = 0.0, 0.0, 0.0

        # Calculate distance to target in each axis if a new target is provided (not KEEP)
        if x != KEEP:
            length_x = x - site_now[leg][0]
        if y != KEEP:
            length_y = y - site_now[leg][1]
        if z != KEEP:
            length_z = z - site_now[leg][2]

        # Calculate the total distance to the new target position
        length = math.sqrt(length_x**2 + length_y**2 + length_z**2)

        # Avoid division by zero
        if length > 0:
            # Set speed for each axis, proportional to the distance in each direction
            temp_speed[leg][0] = length_x / length * move_speed * speed_multiple
            temp_speed[leg][1] = length_y / length * move_speed * speed_multiple
            temp_speed[leg][2] = length_z / length * move_speed * speed_multiple
        else:
            # If the length is zero, the speed remains zero
            temp_speed[leg][0] = 0.0
            temp_speed[leg][1] = 0.0
            temp_speed[leg][2] = 0.0

        # Update the target position (site_expect) if a new target is provided
        if x != KEEP:
            site_expect[leg][0] = x
        if y != KEEP:
            site_expect[leg][1] = y
        if z != KEEP:
            site_expect[leg][2] = z
        """
        PATA DELANTERA IZQUIERDA: 1
        PATA DELANTERA DERECHA: 0
        PATA TRASERA IZQUIERDA: 3
        PATA TRASERA DERECHA: 2
        """
    
    def step_forward(self, step):
        global move_speed  # Assuming move_speed is a global variable

        move_speed = LEG_MOVE_SPEED  # Set movement speed to leg movement speed
        while step > 0:  # Repeat until the specified number of steps is complete
            step -= 1

            if site_now[1][1] == 0:  # PASO CON LA IZQUIERDA
                # Step 1: Lift and move legs 2 and 1 forward
                self.set_site(1, SIDE, 0, Z_UP)  # Lift leg 2
                self.wait_all_reach()
                self.set_site(1, DIAG, 2 * DIAG, Z_UP)  # Move leg 2 forward
                self.wait_all_reach()
                self.on_air[1] = True
                self.set_site(1, DIAG, 2 * DIAG, Z_DEFAULT)  # Lower leg 2
                self.wait_all_reach()
                self.on_air[1] = False

                move_speed = BODY_MOVE_SPEED  # Slow down for body movement

                # Move the body forward by adjusting the grounded legs
                # =============IZQUIERDA=================
                # DELANTERA
                self.set_site(1, DIAG, DIAG, Z_DEFAULT)
                # TRASERA
                self.set_site(3, DIAG, DIAG, Z_DEFAULT)
                # ===============DERECHA=================
                # DELANTERA
                self.set_site(0, SIDE, 0, Z_DEFAULT)
                # TRASERA
                self.set_site(2, DIAG, 2 * DIAG, Z_DEFAULT)

                self.wait_all_reach()

                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement

                # Step 2: Lift and move leg 1 back to its starting position
                self.set_site(2, DIAG, 2 * DIAG, Z_UP)  # Lift leg 1
                self.wait_all_reach()
                self.set_site(2, SIDE, 0, Z_UP)  # Move leg 1 back
                self.wait_all_reach()
                self.on_air[2] = True
                self.set_site(2, SIDE, 0, Z_DEFAULT)  # Lower leg 1
                self.wait_all_reach()
                self.on_air[2] = False

            else:  # PASO CON LA DERECHA
                # Alternate step: Lift and move legs 0 and 3 forward
                self.set_site(0, SIDE, 0, Z_UP)  # Lift leg 3
                self.wait_all_reach()
                self.set_site(0, SIDE, 2 * DIAG, Z_UP)  # Move leg 3 forward
                self.wait_all_reach()
                self.on_air[0] = True
                self.set_site(0, SIDE, 2 * DIAG, Z_DEFAULT)  # Lower leg 3
                self.wait_all_reach()
                self.on_air[0] = False

                move_speed = BODY_MOVE_SPEED  # Slow down for body movement

                # Move the body forward by adjusting the grounded legs
                # ===============DERECHA=================
                # DELANTERA
                self.set_site(0, DIAG, DIAG, Z_DEFAULT)
                # TRASERA
                self.set_site(2, DIAG, DIAG, Z_DEFAULT)
                # =============IZQUIERDA=================
                # DELANTERA
                self.set_site(1, SIDE, 0, Z_DEFAULT)
                # TRASERA
                self.set_site(3, DIAG, 2 * DIAG, Z_DEFAULT)

                self.wait_all_reach()

                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement

                # Lift and move leg 3 back to its starting position
                self.set_site(3, DIAG, 2 * DIAG, Z_UP)  # Lift leg 1
                self.wait_all_reach()
                self.set_site(3, SIDE, 0, Z_UP)  # Move leg 1 back
                self.wait_all_reach()
                self.on_air[3] = True
                self.set_site(3, SIDE, 0, Z_DEFAULT)  # Lower leg 1
                self.wait_all_reach()
                self.on_air[3] = False

    def step_back(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED  # Set movement speed to leg movement speed
    
        while step > 0:
            step -= 1
            if site_now[2][1] == 0:
                # Phase 1: Move legs 3 and 0
                self.set_site(2, SIDE, 0, Z_UP)
                self.wait_all_reach()
                self.set_site(2, DIAG, 2 * DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(2, DIAG, 2 * DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED  # Slow down for body movement
    
                # Adjust grounded legs to move the body forward
                self.set_site(2, DIAG, DIAG, Z_DEFAULT)
                self.set_site(0, DIAG, DIAG, Z_DEFAULT)
                self.set_site(3, SIDE, 0, Z_DEFAULT)
                self.set_site(1, DIAG, 2 * DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement
    
                self.set_site(1, DIAG, 2 * DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(1, SIDE, 0, Z_UP)
                self.wait_all_reach()
                self.set_site(1, SIDE, 0, Z_DEFAULT)
                self.wait_all_reach()
            else:
                # Phase 2: Move legs 1 and 2
                self.set_site(3, SIDE, 0, Z_UP)
                self.wait_all_reach()
                self.set_site(3, DIAG, 2 * DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(3, DIAG, 2 * DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED  # Slow down for body movement
    
                self.set_site(3, DIAG, DIAG, Z_DEFAULT)
                self.set_site(1, DIAG, DIAG, Z_DEFAULT)
                self.set_site(2, SIDE, 0, Z_DEFAULT)
                self.set_site(0, SIDE, 2 * DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement
    
                self.set_site(0, SIDE, 2 * DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(0, SIDE, 0, Z_UP)
                self.wait_all_reach()
                self.set_site(0, SIDE, 0, Z_DEFAULT)
                self.wait_all_reach()
    
    def step_right(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED
    
        while step > 0:
            step -= 1
            if site_now[2][0] == 0:
                self.set_site(2, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(2, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(2, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(2, DIAG, DIAG, Z_DEFAULT)
                self.set_site(0, 0, SIDE, Z_DEFAULT)
                self.set_site(3, DIAG, DIAG, Z_DEFAULT)
                self.set_site(1, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(1, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(1, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(1, 0 , SIDE, Z_DEFAULT)
                self.wait_all_reach()
            
            else:
                self.set_site(0, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(0, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(0, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(0, DIAG, DIAG, Z_DEFAULT)
                self.set_site(2, 0, SIDE, Z_DEFAULT)
                self.set_site(1, DIAG, DIAG, Z_DEFAULT)
                self.set_site(3, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(3, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(3, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(3, 0, SIDE, Z_DEFAULT)
                self.wait_all_reach()
    
    def step_left(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED
    
        while step > 0:
            step -= 1
            if site_now[1][0] == 0:
                self.set_site(1, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(1, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(1, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(1, DIAG, DIAG, Z_DEFAULT)
                self.set_site(3, 0, SIDE, Z_DEFAULT)
                self.set_site(0, DIAG, DIAG, Z_DEFAULT)
                self.set_site(2, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(2, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(2, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(2, 0, SIDE, Z_DEFAULT)
                self.wait_all_reach()
            else:
                self.set_site(3, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(3, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(3, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(3, DIAG, DIAG, Z_DEFAULT)
                self.set_site(1, 0, SIDE, Z_DEFAULT)
                self.set_site(2, DIAG, DIAG, Z_DEFAULT)
                self.set_site(0, 2 * DIAG, DIAG, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(0, 2 * DIAG, DIAG, Z_UP)
                self.wait_all_reach()
                self.set_site(0, 0, SIDE, Z_UP)
                self.wait_all_reach()
                self.set_site(0, 0, SIDE, Z_DEFAULT)
                self.wait_all_reach()

    def turn_left(self,step):
        global move_speed
        move_speed = SPOT_TURN_SPEED  # Configurar velocidad de giro
    
        while step > 0:
            step -= 1
        
        if site_now[3][1] == 0:
            self.set_site(3, SIDE, 0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(3, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, TURN_X0, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, SIDE, 0, Z_UP)
            self.set_site(0, SIDE, 0, Z_DEFAULT)
            self.set_site(3, DIAG, DIAG, Z_DEFAULT)
            self.set_site(1, DIAG, DIAG, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, SIDE, 0, Z_DEFAULT)
            self.wait_all_reach()
        
        else:
            self.set_site(0, SIDE, 0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_UP)
            self.set_site(3, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, TURN_X0, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, DIAG, DIAG, Z_DEFAULT)
            self.set_site(3, SIDE, 0, Z_DEFAULT)
            self.set_site(1, SIDE, 0, Z_UP)
            self.set_site(0, SIDE, DIAG, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, SIDE, 0, Z_DEFAULT)
            self.wait_all_reach()

    def turn_right(self,step):
        global move_speed
        move_speed = SPOT_TURN_SPEED  # Configurar velocidad de giro
    
        while step > 0:
            step -= 1
        
        if site_now[1][1] == 0:
            self.set_site(1, SIDE, 0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0, TURN_Y0, Z_UP)
            self.set_site(2, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, SIDE, 0, Z_UP)
            self.set_site(2, SIDE, 0, Z_DEFAULT)
            self.set_site(1, DIAG, DIAG, Z_DEFAULT)
            self.set_site(3, DIAG, DIAG, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, SIDE, 0, Z_DEFAULT)
            self.wait_all_reach()
        
        else:
            self.set_site(2, SIDE, 0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0, TURN_Y0, Z_UP)
            self.set_site(1, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(3, TURN_X0, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, DIAG, DIAG, Z_DEFAULT)
            self.set_site(0, SIDE, DIAG, Z_DEFAULT)
            self.set_site(1, SIDE, 0, Z_DEFAULT)
            self.set_site(3, SIDE, 0, Z_UP)
            self.wait_all_reach()

            self.set_site(3, SIDE, 0, Z_DEFAULT)
            self.wait_all_reach()

    def timer_callback(self):
        self.update_positions()
        self.update_velocities()
        # Diccionario con los valores actuales de las articulaciones
        joint_position_dict = {
            'CoxisRU_joint': self.joint_positions[0],
            'FemurRU_joint': self.joint_positions[1],
            'TibiaRU_joint': self.joint_positions[2],
            'CoxisLU_joint': self.joint_positions[3],
            'FemurLU_joint': self.joint_positions[4],
            'TibiaLU_joint': self.joint_positions[5],
            'CoxisRD_joint': self.joint_positions[6],
            'FemurRD_joint': self.joint_positions[7],
            'TibiaRD_joint': self.joint_positions[8],
            'CoxisLD_joint': self.joint_positions[9],
            'FemurLD_joint': self.joint_positions[10],
            'TibiaLD_joint': self.joint_positions[11],

            # Bumpers con posición fija
            'BumperRU_joint': 0.0,
            'BumperLU_joint': 0.0,
            'BumperRD_joint': 0.0,
            'BumperLD_joint': 0.0,
        }

        # Orden estricto según configuración de gazebo_joint_controller
        joint_order = [
            'CoxisRU_joint', 'FemurRU_joint', 'TibiaRU_joint', 'BumperRU_joint',
            'CoxisLU_joint', 'FemurLU_joint', 'TibiaLU_joint', 'BumperLU_joint',
            'CoxisRD_joint', 'FemurRD_joint', 'TibiaRD_joint', 'BumperRD_joint',
            'CoxisLD_joint', 'FemurLD_joint', 'TibiaLD_joint', 'BumperLD_joint'
        ]

        # Construcción del mensaje
        position_msg = Float64MultiArray()
        position_msg.data = [joint_position_dict[name] for name in joint_order]
        self.position_publisher_.publish(position_msg)

        # Velocidades no necesitan cambio (solo 12 articulaciones móviles)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = self.joint_velocities
        self.velocity_publisher_.publish(velocity_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


        # Publicar trayectoria
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("odom", "base_link", now)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.path_msg.poses.append(pose)
            self.path_msg.header.stamp = pose.header.stamp
            self.path_pub.publish(self.path_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass  # Evita error si la transformación aún no está disponible

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

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        sin_yaw = 2.0 * (qw * qz + qx * qy)
        cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(sin_yaw, cos_yaw)

        if self.inicio == False:
            self.target_angle = yaw
            self.inicio = True

        self.current_angle = yaw  # Almacena el ángulo yaw del IMU

        # Si ω se vuelve cero, fijar el target_angle con el último roll del IMU
        if self.angular_z != 0.0:
            self.target_angle = self.current_angle

        if self.state == 'X':  # Modo omnidireccional
            #ALGORITMO DE CONTROL PROPORCIONAL NO LINEAL (ACOTADO)
            #Si se está moviendo sin rotación, aplicar corrección proporcional de ángulo
            if self.angular_z == 0 and self.target_angle is not None:
                error_angle = math.degrees(self.target_angle - self.current_angle)
                # Si el error es mayor a 20°, aplicar corrección fuerte
                if abs(error_angle) > 20:
                    correction = 1.0 if error_angle > 0 else -1.0 
                else:
                    correction = self.kp * error_angle

                #self.get_logger().info(f"correction: {correction}")  # Mostrar en grados

                self.target_velocities = [
                    self.linear_x + self.angular_z + self.linear_y + correction,  # RU
                    -self.linear_x + self.angular_z + self.linear_y + correction, # LU
                    self.linear_x + self.angular_z - self.linear_y + correction,  # RD
                    -self.linear_x + self.angular_z - self.linear_y + correction  # LD
                ]

        # self.get_logger().info(f"Yaw actual: {yaw}°")  # Mostrar en grados
        # self.get_logger().info(f"current: {self.current_angle}")  # Mostrar en grados
        # self.get_logger().info(f"target: {self.target_angle}")  # Mostrar en grados

    def cmd_vel_callback(self, msg):
        global servo_service_en
        """Callback para actualizar los comandos de velocidad."""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        servo_service_en = False

        if self.state == 'X':  # Modo omnidireccional

            self.target_velocities = [
                self.linear_x + self.angular_z + self.linear_y,  # RU
                -self.linear_x + self.angular_z + self.linear_y, # LU
                self.linear_x + self.angular_z - self.linear_y,  # RD
                -self.linear_x + self.angular_z - self.linear_y  # LD
            ]
        
        elif self.state == 'H':  # Modo móvil tipo H

            self.target_velocities = [
                self.linear_x + self.angular_z,  # RU (Rueda Derecha Delantera)
                -self.linear_x + self.angular_z, # LU (Rueda Izquierda Delantera)
                self.linear_x + self.angular_z,  # RD (Rueda Derecha Trasera)
                -self.linear_x + self.angular_z  # LD (Rueda Izquierda Trasera)
            ]

        else:  # Modo C (reposo en las ruedas)
            self.target_velocities = [0.0] * 4
            servo_service_en = True
            if self.linear_x > 0:
                self.machine = 1
                return
            elif self.linear_x < 0:
                self.machine = 2
                return
            elif self.linear_y > 0:
                self.machine = 3
                return
            elif self.linear_y < 0:
                self.machine = 4
                return
            elif self.angular_z > 0:
                self.machine = 5
                return
            elif self.angular_z < 0:
                self.machine = 6
                return
            else:
                self.machine = 8

        self.update_target_positions()
            
    def ticker_callback(self):
        """
        Timer callback to move endpoints toward expected positions in a straight line.
        """
        # Check if the servo service is enabled
        if not servo_service_en:
            return

        # Loop through each leg to update positions
        for i in range(4):  # imax = 4
            # Update each coordinate (x, y, z) based on speed
            for j in range(3):
                # Move incrementally if far from the target, or snap to target if close
                if abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]):
                    site_now[i][j] += temp_speed[i][j]
                else:
                    site_now[i][j] = site_expect[i][j]

            # Convert the current Cartesian position to polar coordinates (joint angles)
            #self.inverseKinematics(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2])
            #self.get_logger().info(f"x:{site_now[i][0]}\ny:{site_now[i][1]}\nz:{site_now[i][2]}")
            alpha, beta, gamma = self.inverseKinematics(site_now[i][0], site_now[i][1], site_now[i][2])
            # Adjust angles for specific legs
            alpha = alpha - 0.785
            #beta = beta
            gamma = -gamma + 1.57
            #gamma = -gamma +2.1
            if i in (1, 3):  # Adjust alpha for legs 0 and 2
                beta = - beta
                gamma = - gamma
                if i == 1:
                    alpha = -alpha
            if i == 2:
                alpha = - alpha

            # if i == 0:
            #self.get_logger().info(f"Alpha:{alpha*57.295}\nBeta:{beta*57.295}\nGamma:{gamma*57.295}")
            self.target_positions[3*i:3*i+3] = [alpha,beta,gamma]

    def peter_mode_callback(self, msg):
        global servo_service_en
        """Callback para actualizar el modo."""
        mode = msg.data.upper()
        if mode in ['C', 'X', 'H']:
            self.state = mode
            servo_service_en = False
            self.update_target_positions()

    def update_target_positions(self):
        """Actualiza las posiciones objetivo de las articulaciones según el modo."""
        global servo_service_en
        if self.state == 'C':  # Modo cuadrúpedo
            servo_service_en = True
            if self.past != 'C':
            # /////////////////////////// MODIFICAR AQUI PARA LA CAMINATA EN MODO CUADRUPEDO ///////////////////////////////
                self.iniciarCuadrupedo()
            #///////////////////////////////////////////////////////////////////////////////////////////////////////////////
            self.past = 'C'

        elif self.state == 'X':  # Modo omnidireccional

            if self.linear_x != 0.0:
                self.target_positions = [
                0.0, 0.8, 2.30,  # CoxisRU, FemurRU, TibiaRU
                0.0, -0.8, -2.30,  # CoxisLU, FemurLU, TibiaLU
                0.0, 0.9, 2.30,  # CoxisRD, FemurRD, TibiaRD
                0.0, -0.8, -2.30   # CoxisLD, FemurLD, TibiaLD
                ]

            elif self.linear_y != 0.0:
                self.target_positions = [
                1.57, 0.8, 2.30,  # CoxisRU, FemurRU, TibiaRU
                -1.42, -0.8, -2.30,  # CoxisLU, FemurLU, TibiaLU
                -1.42, 0.9, 2.30,  # CoxisRD, FemurRD, TibiaRD
                1.46, -0.8, -2.30   # CoxisLD, FemurLD, TibiaLD
            ]
            else:    
                self.target_positions = [
                    0.7, 0.8, 2.30,  # CoxisRU, FemurRU, TibiaRU
                    -0.7, -0.8, -2.30,  # CoxisLU, FemurLU, TibiaLU
                    -0.7, 0.8, 2.10,  # CoxisRD, FemurRD, TibiaRD
                    0.7, -0.8, -2.30   # CoxisLD, FemurLD, TibiaLD
                ]

            self.past = 'X'

        elif self.state == 'H':  # Modo móvil tipo H
            self.target_positions = [
                0.0, 0.8, 2.30,  # CoxisRU, FemurRU, TibiaRU
                0.0, -0.8, -2.30,  # CoxisLU, FemurLU, TibiaLU
                0.0, 0.9, 2.30,  # CoxisRD, FemurRD, TibiaRD
                0.0, -0.8, -2.30   # CoxisLD, FemurLD, TibiaLD
            ]
            self.past = 'H'

    def setCenter(self):
        global site_now
        for i in range(4):
            self.set_site(i,site_now[i][0],site_now[i][1],Z_UP)
            self.wait_reach(i)
            self.set_site(i,DIAG,DIAG,Z_UP)
            self.wait_reach(i)
            self.set_site(i,DIAG,DIAG,Z_DEFAULT)
            self.wait_reach(i)
            # #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
            # self.set_site(1, DIAG, DIAG , Z_DEFAULT)
            # self.set_site(3, DIAG, DIAG , Z_DEFAULT)
            # #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
            # self.set_site(0, SIDE, DIAG, Z_DEFAULT)
            # self.set_site(2, DIAG, DIAG, Z_DEFAULT)
            # #self.wait_all_reach()

    def setFrontStep(self):
        self.setCenter()
        self.set_site(1,DIAG,DIAG,Z_UP)
        self.wait_reach(1)
        self.set_site(1,SIDE,0,Z_UP)
        self.wait_reach(1)
        self.set_site(1,SIDE,0,Z_DEFAULT)
        self.wait_reach(1)

        self.set_site(3,DIAG,DIAG,Z_UP)
        self.wait_reach(3)
        self.set_site(3,SIDE,0,Z_UP)
        self.wait_reach(3)
        self.set_site(3,SIDE,0,Z_DEFAULT)
        self.wait_reach(3)

        # #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
        # self.set_site(1, SIDE, 0 , Z_DEFAULT)
        # self.set_site(3, SIDE, 0 , Z_DEFAULT)
        # #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
        # self.set_site(0, SIDE, DIAG, Z_DEFAULT)
        # self.set_site(2, DIAG, DIAG, Z_DEFAULT)
        # #self.wait_all_reach()

    def setSideStep(self):
        self.setCenter()
        self.set_site(1,DIAG,DIAG,Z_UP)
        self.wait_reach(1)
        self.set_site(1,0,SIDE,Z_UP)
        self.wait_reach(1)
        self.set_site(1,0,SIDE,Z_DEFAULT)
        self.wait_reach(1)

        self.set_site(0,DIAG,DIAG,Z_UP)
        self.wait_reach(0)
        self.set_site(0,0,SIDE,Z_UP)
        self.wait_reach(0)
        self.set_site(0,0,SIDE,Z_DEFAULT)
        self.wait_reach(0)

        # #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
        # self.set_site(1, 0, SIDE , Z_DEFAULT)
        # self.set_site(3, DIAG, DIAG , Z_DEFAULT)
        # #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
        # self.set_site(0, 0, SIDE, Z_DEFAULT)
        # self.set_site(2, DIAG, DIAG, Z_DEFAULT)
        # #self.wait_all_reach()

    def bumper_callback(self, msg):
        global site_expect, site_now
        """
        Process all collisions detected by the bumper sensor.
        Extracts and prints all force.z values from each contact and wrench.
        """
        total_collisions = len(msg.contacts)
        if total_collisions == 0:
            return  # No collisions detected

        #self.get_logger().info(f"Detected {total_collisions} collisions.")

        for contact in msg.contacts:
            collision1_name = contact.collision1.name
            collision2_name = contact.collision2.name
            
            leg_name = self.extract_leg_name(collision1_name)
            
            if leg_name == 'BumperRU':
                leg_index = 0
            elif leg_name == 'BumperLU':
                leg_index = 1
            elif leg_name == 'BumperRD':
                leg_index = 2
            elif leg_name == 'BumperLD':
                leg_index = 3    
            # Each contact can have multiple wrenches (force/torque readings)
            for i, wrench in enumerate(contact.wrenches):
                force1_z = wrench.body_1_wrench.force.z
                force2_z = wrench.body_2_wrench.force.z

                # Log collision details
                # self.get_logger().info(f"Collision {i+1}: {collision1_name} <-> {collision2_name}")
                # self.get_logger().info(f"  - Force Z (Body 1): {force1_z}")
                # self.get_logger().info(f"  - Force Z (Body 2): {force2_z}")

                # Detect ground support (force.z < 0)
                
                # if force1_z < -3.0 and self.on_air[leg_index]:
                #     site_now[leg_index][0] = site_expect[leg_index][0]
                #     site_now[leg_index][1] = site_expect[leg_index][1]
                #     site_now[leg_index][2] = site_expect[leg_index][2]
                #     # self.get_logger().info(f"  ✅ Ground support detected from {collision1_name} with force.z = {force1_z}")
                # if force2_z < -3.0 and self.on_air[leg_index]:
                #     site_now[leg_index][0] = site_expect[leg_index][0]
                #     site_now[leg_index][1] = site_expect[leg_index][1]
                #     site_now[leg_index][2] = site_expect[leg_index][2]
                #     # self.get_logger().info(f"  ✅ Ground support detected from {collision2_name} with force.z = {force2_z}")

    def extract_leg_name(self,collision_name):
        """Extracts leg name from collision string (e.g., 'BumperRU' from 'peter::BumperRU_link::BumperRU_link_collision')"""
        match = re.search(r"peter::(Bumper\w+)_link", collision_name)
        if match:
            return match.group(1)  # Extract the 'BumperRU' part
        return None  # Return None if no match is found

def main(args=None):
    rclpy.init(args=args)
    joint_position_publisher = JointPositionPublisher()

    # Use MultiThreadedExecutor for concurrent callback execution
    executor = MultiThreadedExecutor()
    executor.add_node(joint_position_publisher)

    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # Let the timer drive execution - don't block here
        while rclpy.ok():
            time.sleep(0.1)  # Prevents high CPU usage while letting the timer work
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure proper shutdown
        executor.shutdown()
        executor_thread.join()
        joint_position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
