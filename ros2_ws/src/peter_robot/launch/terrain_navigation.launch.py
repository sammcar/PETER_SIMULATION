#!/usr/bin/env python3
"""
terrain_navigation.launch.py
Archivo de lanzamiento unificado para la evaluación en terrenos irregulares (Familia C1).
Hereda la infraestructura de inicialización de modelo, puentes y controladores del
entorno base de un solo estímulo, integrando el monitor de estabilidad.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    PACKAGE_NAME = 'peter_robot'
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    # 1. Declaración exhaustiva de argumentos enviados por el test_manager
    args = [
        DeclareLaunchArgument('world_name', default_value='terrain'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('trial_index', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
    ]

    # 2. Carga y procesamiento del modelo cinemático (URDF/Xacro)
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    if not os.path.exists(xacro_path):
        # Fallback de respaldo por consistencia de estructura de archivos
        xacro_path = os.path.join(pkg_share, 'urdf', 'peter_robot.urdf.xacro')
        
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Nodo del Publicador de Estado del Robot
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Inclusión del entorno físico de simulación de Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 5. Generación de la entidad del robot en Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_platform',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z')
        ],
        output='screen'
    )

    # 6. Puentes de comunicación ROS 2 <-> Gazebo Transport
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/peter/stability_data@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # 7. Controlador de Locomoción y Spawners de ROS 2 Control
    peter_controller = Node(
        package=PACKAGE_NAME,
        executable='peter_controller',
        name='peter_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    load_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_forward = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='screen'
    )

    # 8. Red Neuronal de Selección de Marchas
    neural_network = Node(
        package=PACKAGE_NAME,
        executable='neural_network',
        name='neural_network',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. Infraestructura de Telemetría e Instrumentación (Fase Post-Warmup: 10.5 s)
    telemetry_nodes = TimerAction(period=10.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='neural_recorder',
            name='neural_recorder',
            output='screen',
            parameters=[{
                'experiment_type': 'terrain_navigation',
                'use_sim_time': True
            }]
        ),
        Node(
            package=PACKAGE_NAME,
            executable='metrics_recorder',
            name='metrics_recorder',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package=PACKAGE_NAME,
            executable='peter_stability_monitor',
            name='stability_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

    return LaunchDescription(
        args + [
            robot_state_pub,
            gazebo,
            spawn_robot,
            ros_gz_bridge,
            peter_controller,
            load_joint_state,
            load_forward,
            neural_network,
            telemetry_nodes
        ]
    )