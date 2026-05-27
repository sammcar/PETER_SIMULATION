#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
import math
import time
from rclpy.executors import MultiThreadedExecutor
import threading

# Robot parameters
LENGTH_A = 45.0
LENGTH_C = 45.0
LENGTH_B = 90.0
LENGTH_SIDE = 65.0
Z_ABSOLUTE = 26.0
servo_service_en = False
# Movement constants
Z_DEFAULT = LENGTH_B
Z_UP = 60
Z_BOOT = LENGTH_B
X_DEFAULT = math.sqrt(((LENGTH_A + LENGTH_C) ** 2) / 2)
X_OFFSET = 0
Y_START = 0
X_START = 0
Y_STEP = X_DEFAULT
X_STEP = Y_STEP
Y_DEFAULT = X_DEFAULT

# Movement variables
site_now = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for current positions
site_expect = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for expected positions
temp_speed = [[0.0 for _ in range(3)] for _ in range(4)]  # 4x3 array for temporary speeds
move_speed = 8.0
speed_multiple = 1.0
SPOT_TURN_SPEED = 4.0
LEG_MOVE_SPEED = 8.0
BODY_MOVE_SPEED = 3.0
STAND_SEAT_SPEED = 1.0
rest_counter = 0  # volatile not needed in Python
KEEP = 255.0

# Turn calculations
TEMP_A = math.sqrt((2 * X_DEFAULT + LENGTH_SIDE) ** 2 + Y_STEP ** 2)
TEMP_B = 2 * (Y_START + Y_STEP) + LENGTH_SIDE
TEMP_C = math.sqrt((2 * X_DEFAULT + LENGTH_SIDE) ** 2 + (2 * Y_START + Y_STEP + LENGTH_SIDE) ** 2)
TEMP_ALPHA = math.acos((TEMP_A ** 2 + TEMP_B ** 2 - TEMP_C ** 2) / (2 * TEMP_A * TEMP_B))
TURN_X1 = (TEMP_A - LENGTH_SIDE) / 2
TURN_Y1 = Y_START + Y_STEP / 2
TURN_X0 = TURN_X1 - TEMP_B * math.cos(TEMP_ALPHA)
TURN_Y0 = TEMP_B * math.sin(TEMP_ALPHA) - TURN_Y1 - LENGTH_SIDE

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
        self.leg_ok = False
        self.leg = 0

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.ticker = self.create_timer(0.05, self.ticker_callback)

        self.machine = 0
        # Start the state machine execution
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
        self.get_logger().info(f"Processing state: {self.machine}")

        if self.machine == 1:
            if site_now[2][1] != Y_START and site_now[3][1] != Y_START:
                self.setFrontStep()
            servo_service_en = True
            self.step_forward(1)
        
        elif self.machine == 2:
            if site_now[2][1] != Y_START and site_now[3][1] != Y_START:
                self.setFrontStep()
            servo_service_en = True
            self.step_back(1)
        
        elif self.machine == 3:
            if site_now[2][0] != X_START and site_now[0][0] != X_START:
                self.setSideStep()
            servo_service_en = True
            self.step_left(1)

        elif self.machine == 4:
            if site_now[2][0] != X_START and site_now[0][0] != X_START:
                self.setSideStep()
            servo_service_en = True
            self.step_right(1)

        elif self.machine == 5:
            if site_now[2][1] != Y_START and site_now[3][1] != Y_START:
                self.setFrontStep()
            servo_service_en = True
            self.turn_left(1)

        elif self.machine == 6:
            if site_now[2][1] != Y_START and site_now[3][1] != Y_START:
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

    # def wait_all_reach(self):
    #     """
    #     Waits for all legs to reach their target positions by calling wait_reach
    #     for each leg (0 to 3).
    #     """
    #     #for leg in range(4):  # Iterate over legs 0 to 3
        
    #     self.get_logger().info("ESPERANDO PATA"+str(self.leg))
    #     self.wait_reach(self.leg)
        
    #     if self.leg_ok:
    #         self.leg += 1
    #         self.leg_ok = False
    #         if self.leg == 4:
    #             self.leg = 0

    # def wait_reach(self, leg):
    #     """
    #     Waits for a specific leg to reach its target position.

    #     Args:
    #     leg (int): The leg index to check (0 to 3).
    #     """
    #     global site_now, site_expect, servo_service_en  

    #     self.get_logger().info(f"Checking leg {leg} position: {site_now[leg]}")
    #     self.get_logger().info(f"Desired leg {leg} position: {site_expect[leg]}")
    #     self.get_logger().info(f"SERVICE: {servo_service_en}")
    #     time.sleep(0.03)  # Let ROS process callbacks without spin_once()
        
    #     while not (site_now[leg][0] == site_expect[leg][0] and
    #               site_now[leg][1] == site_expect[leg][1] and
    #             site_now[leg][2] == site_expect[leg][2]):
    #         return
        
    #     self.leg_ok = True

    def wait_all_reach(self):
        """
        Waits for all legs to reach their target positions by calling wait_reach
        for each leg (0 to 3).
        """
        for leg in range(4):  # Iterate over legs 0 to 3
            self.get_logger().info("ESPERANDO PATA"+str(leg))
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

            self.get_logger().info(f"Checking leg {leg} position: {site_now[leg]}")
            self.get_logger().info(f"Desired leg {leg} position: {site_expect[leg]}")
            self.get_logger().info(f"SERVICE: {servo_service_en}")
            rclpy.spin_once(self, timeout_sec=0.1)  # Allow callbacks to run
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

            if site_now[1][1] == Y_START:  # PASO CON LA IZQUIERDA
                # Step 1: Lift and move legs 2 and 1 forward
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)  # Lift leg 2
                self.wait_all_reach()
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)  # Move leg 2 forward
                self.wait_all_reach()
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)  # Lower leg 2
                self.wait_all_reach()

                move_speed = BODY_MOVE_SPEED  # Slow down for body movement

                # Move the body forward by adjusting the grounded legs
                # =============IZQUIERDA=================
                # DELANTERA
                self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                # TRASERA
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                # ===============DERECHA=================
                # DELANTERA
                self.set_site(0, X_DEFAULT - X_OFFSET, Y_START, Z_DEFAULT)
                # TRASERA
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)

                self.wait_all_reach()

                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement

                # Step 2: Lift and move leg 1 back to its starting position
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)  # Lift leg 1
                self.wait_all_reach()
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)  # Move leg 1 back
                self.wait_all_reach()
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)  # Lower leg 1
                self.wait_all_reach()
            else:  # PASO CON LA DERECHA
                # Alternate step: Lift and move legs 0 and 3 forward
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)  # Lift leg 3
                self.wait_all_reach()
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)  # Move leg 3 forward
                self.wait_all_reach()
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)  # Lower leg 3
                self.wait_all_reach()

                move_speed = BODY_MOVE_SPEED  # Slow down for body movement

                # Move the body forward by adjusting the grounded legs
                # ===============DERECHA=================
                # DELANTERA
                self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                # TRASERA
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                # =============IZQUIERDA=================
                # DELANTERA
                self.set_site(1, X_DEFAULT - X_OFFSET, Y_START, Z_DEFAULT)
                # TRASERA
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)

                self.wait_all_reach()

                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement

                # Lift and move leg 3 back to its starting position
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)  # Lift leg 1
                self.wait_all_reach()
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)  # Move leg 1 back
                self.wait_all_reach()
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)  # Lower leg 1
                self.wait_all_reach()

    def step_back(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED  # Set movement speed to leg movement speed
    
        while step > 0:
            step -= 1
            if site_now[2][1] == Y_START:
                # Phase 1: Move legs 3 and 0
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED  # Slow down for body movement
    
                # Adjust grounded legs to move the body forward
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
                self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement
    
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
                self.wait_all_reach()
            else:
                # Phase 2: Move legs 1 and 2
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED  # Slow down for body movement
    
                self.set_site(3, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
                self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + 2 * Y_STEP, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED  # Restore speed for leg movement
    
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START + 2 * Y_STEP, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
                self.wait_all_reach()
    
    def step_right(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED
    
        while step > 0:
            step -= 1
            if site_now[2][0] == X_START:
                self.set_site(2, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(2, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(0, X_START, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(3, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(1, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(1, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_START + X_OFFSET, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
            
            else:
                self.set_site(0, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(0, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(2, X_START, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(1, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(3, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(3, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_START + X_OFFSET, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
    def step_left(self,step):
        global move_speed
        move_speed = LEG_MOVE_SPEED
    
        while step > 0:
            step -= 1
            if site_now[1][0] == X_START:
                self.set_site(1, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(1, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(1, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(3, X_START, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(0, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(2, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(2, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(2, X_START + X_OFFSET, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
            else:
                self.set_site(3, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(3, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = BODY_MOVE_SPEED
    
                self.set_site(3, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(1, X_START, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(2, X_DEFAULT, Y_START + Y_STEP, Z_DEFAULT)
                self.set_site(0, X_START + 2 * X_STEP, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()
    
                move_speed = LEG_MOVE_SPEED
    
                self.set_site(0, X_START + 2 * X_STEP, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_START, Y_DEFAULT, Z_UP)
                self.wait_all_reach()
                self.set_site(0, X_START + X_OFFSET, Y_DEFAULT, Z_DEFAULT)
                self.wait_all_reach()

    def turn_left(self,step):
        global move_speed
        move_speed = SPOT_TURN_SPEED  # Configurar velocidad de giro
    
        while step > 0:
            step -= 1
        
        if site_now[3][1] == Y_START:
            self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.wait_all_reach()
        
        else:
            self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.set_site(3, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.set_site(3, X_DEFAULT - X_OFFSET, Y_START, Z_DEFAULT)
            self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.set_site(0, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.wait_all_reach()

    def turn_right(self,step):
        global move_speed
        move_speed = SPOT_TURN_SPEED  # Configurar velocidad de giro
    
        while step > 0:
            step -= 1
        
        if site_now[1][1] == Y_START:
            self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.set_site(2, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(1, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(3, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(1, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(2, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.set_site(1, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.set_site(3, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.wait_all_reach()
        
        else:
            self.set_site(2, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.wait_all_reach()

            self.set_site(0, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.set_site(1, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(2, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(0, TURN_X1 - X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(2, TURN_X0 - X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.set_site(1, TURN_X1 + X_OFFSET, TURN_Y1, Z_DEFAULT)
            self.set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_DEFAULT)
            self.wait_all_reach()

            self.set_site(3, TURN_X0 + X_OFFSET, TURN_Y0, Z_UP)
            self.wait_all_reach()

            self.set_site(2, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_DEFAULT)
            self.set_site(1, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_UP)
            self.wait_all_reach()

            self.set_site(3, X_DEFAULT + X_OFFSET, Y_START, Z_DEFAULT)
            self.wait_all_reach()

    def timer_callback(self):
        self.get_logger().info("Running timer")
        self.update_positions()
        #self.ticker_callback()
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
        global servo_service_en
        """Callback para actualizar los comandos de velocidad."""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        servo_service_en = False

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
            servo_service_en = True
            if self.linear_x > 0:
                self.machine = 1
            elif self.linear_x < 0:
                self.machine = 2
            elif self.linear_y > 0:
                self.machine = 3
            elif self.linear_y < 0:
                self.machine = 4
            elif self.angular_z > 0:
                self.machine = 5
            elif self.angular_z < 0:
                self.machine = 6
            else:
                self.machine = 8
            
    def ticker_callback(self):
        self.get_logger().info("Running TICKER, leg: "+str(self.leg))

        """
        Timer callback to move endpoints toward expected positions in a straight line.
        """
        # Check if the servo service is enabled
        if not servo_service_en:
            return

        # Temporary variables for joint angles
        #alpha, beta, gamma = 0.0, 0.0, 0.0

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
            if i in (1, 2):  # Adjust alpha for legs 0 and 2
                alpha = - alpha

            # Send the angles to the leg
            #anglesToLeg(i, alpha, beta, gamma)
            #alpha [0 , pi/2]
            #beta [-pi/4 , pi/4]
            #gamma [-pi/2, pi/8]
            beta = beta #+ math.pi/4
            gamma = gamma - 1.57

            
            self.get_logger().info(f"Alpha:{alpha*57.295}\nBeta:{beta*57.295}\nGamma:{gamma*57.295}")
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
        global site_expect, site_now
        """Actualiza las posiciones objetivo de las articulaciones según el modo."""
        if self.state == 'C':  # Modo cuadrúpedo

            # /////////////////////////// MODIFICAR AQUI PARA LA CAMINATA EN MODO CUADRUPEDO ///////////////////////////////
            site_expect = [[X_DEFAULT,Y_DEFAULT,Z_DEFAULT],
                           [X_DEFAULT,Y_START,Z_DEFAULT],
                           [X_DEFAULT,Y_DEFAULT,Z_DEFAULT],
                           [X_DEFAULT,Y_START,Z_DEFAULT]]
            site_now = [[X_DEFAULT,Y_DEFAULT,Z_DEFAULT],
                           [X_DEFAULT,Y_START,Z_DEFAULT],
                           [X_DEFAULT,Y_DEFAULT,Z_DEFAULT],
                           [X_DEFAULT,Y_START,Z_DEFAULT]]
            self.target_positions = [0.785,0.,0.,0.,0.,0.,-0.785,0.,0.,0.,0.,0.]
            pass
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

    def setFrontStep(self):
        #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
        self.set_site(1, X_DEFAULT - X_OFFSET, Y_START , Z_BOOT)
        self.set_site(3, X_DEFAULT + X_OFFSET, Y_START , Z_BOOT)
        #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
        self.set_site(0, X_DEFAULT - X_OFFSET, Y_START + Y_STEP, Z_BOOT)
        self.set_site(2, X_DEFAULT + X_OFFSET, Y_START + Y_STEP, Z_BOOT)
        #self.wait_all_reach()

    def setSideStep(self):
        #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
        self.set_site(1, X_START - X_OFFSET, Y_DEFAULT , Z_BOOT)
        self.set_site(3, X_DEFAULT + X_OFFSET, Y_DEFAULT , Z_BOOT)
        #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
        self.set_site(0, X_START - X_OFFSET, Y_DEFAULT, Z_BOOT)
        self.set_site(2, X_DEFAULT + X_OFFSET, Y_DEFAULT, Z_BOOT)
        #self.wait_all_reach()

    def setCenter(self):
        #LADO IZQUIERDO, PATAS 1(DELANTERA) Y 3(TRASERA)
        self.set_site(1, X_DEFAULT, Y_DEFAULT , Z_BOOT)
        self.set_site(3, X_DEFAULT, Y_DEFAULT , Z_BOOT)
        #LADO DERECHO, PATAS 0(DELANTERA) Y 2(TRASERA)
        self.set_site(0, X_DEFAULT, Y_DEFAULT, Z_BOOT)
        self.set_site(2, X_DEFAULT, Y_DEFAULT, Z_BOOT)
        #self.wait_all_reach()
"""
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
"""

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
