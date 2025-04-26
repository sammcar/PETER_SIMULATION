#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk

class JointSliderPublisher(Node):
    def __init__(self, slider_values):
        super().__init__('joint_slider_publisher')
        
        # Publicador para comandos de posición (todas las articulaciones)
        self.position_publisher = self.create_publisher(
            Float64MultiArray, 
            '/gazebo_joint_controller/commands', 
            10
        )
        # Publicador para comandos de velocidad (por ejemplo, para 4 ruedas)
        self.velocity_publisher = self.create_publisher(
            Float64MultiArray, 
            '/gazebo_velocity_controllers/commands', 
            10
        )
        
        # Lista compartida con la interfaz para leer los valores de posición
        self.slider_values = slider_values

    def publish_joint_commands(self):
        msg = Float64MultiArray()
        # Copia de los valores actuales de los sliders de posición
        msg.data = self.slider_values[:]
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Publicando Posición: {msg.data}")

    def publish_velocity_command(self, command_value):
        msg = Float64MultiArray()
        # Crea un array de 4 elementos con el valor proporcionado
        msg.data = [command_value] * 4
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publicando Velocidad: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    # Lista con los nombres de las articulaciones para posición
    joint_names = [
        "CoxisRU_joint",
        "FemurRU_joint",
        "TibiaRU_joint",
        "CoxisLU_joint",
        "FemurLU_joint",
        "TibiaLU_joint",
        "CoxisRD_joint",
        "FemurRD_joint",
        "TibiaRD_joint",
        "CoxisLD_joint",
        "FemurLD_joint",
        "TibiaLD_joint"
    ]
    
    # Creamos una lista con 12 valores iniciales para las articulaciones de posición
    joint_values = [0.0] * len(joint_names)

    # Creamos el nodo ROS2 que publicará los mensajes
    node = JointSliderPublisher(joint_values)

    # Creamos la ventana principal de Tkinter
    root = tk.Tk()
    root.title("Control de Articulaciones")

    # Marco para los sliders de posición
    frame_positions = tk.Frame(root)
    frame_positions.pack(side=tk.TOP, padx=10, pady=10)

    # Se crea un slider para cada articulación con su nombre
    sliders = []
    for i, joint in enumerate(joint_names):
        frame = tk.Frame(frame_positions)
        frame.pack(pady=2)
        label = tk.Label(frame, text=joint)
        label.pack(side=tk.LEFT, padx=5)
        slider = tk.Scale(frame, from_=-1.5, to=1.5, resolution=0.01,
                          orient=tk.HORIZONTAL, length=300)
        slider.pack(side=tk.LEFT)
        slider.set(0.0)
        sliders.append(slider)

    # Marco para los botones de velocidad
    frame_velocity = tk.Frame(root)
    frame_velocity.pack(side=tk.TOP, padx=10, pady=10)
    
    # Botón para publicar velocidad 1.0
    def publish_velocity_1():
        node.publish_velocity_command(1.0)
    
    btn_velocity_1 = tk.Button(frame_velocity, text="Publicar Velocidad 1.0", command=publish_velocity_1)
    btn_velocity_1.pack(side=tk.LEFT, padx=5)

    # Botón para publicar velocidad 0.0
    def publish_velocity_0():
        node.publish_velocity_command(0.0)
    
    btn_velocity_0 = tk.Button(frame_velocity, text="Publicar Velocidad 0.0", command=publish_velocity_0)
    btn_velocity_0.pack(side=tk.LEFT, padx=5)

    # Función que actualiza los valores de los sliders y publica los comandos de posición
    def update_and_publish():
        for i, slider in enumerate(sliders):
            joint_values[i] = slider.get()
        node.publish_joint_commands()
        # Se programa la siguiente actualización cada 100 ms
        root.after(100, update_and_publish)

    root.after(100, update_and_publish)

    try:
        root.mainloop()  # Ejecuta la interfaz gráfica
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
