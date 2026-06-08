#!/usr/bin/env python3
"""
terrain_navigation.launch.py
Archivo de lanzamiento unificado para la evaluación en terrenos irregulares (Familia C1).
Incorpora auto-descubrimiento del archivo URDF/Xacro para evitar crashes.
"""

import os
import glob
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    PACKAGE_NAME = 'peter_robot'
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    # 1. Argumentos exigidos por el test_manager
    args = [
        DeclareLaunchArgument('world_name', default_value='terrain'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('trial_index', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
    ]

    # 2. Rutas de recursos para Gazebo Sim
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'), ':',
            os.path.join(pkg_share, 'urdf')
        ]
    )

    # 3. AUTO-DESCUBRIMIENTO DEL MODELO CINEMÁTICO (Evita el XacroException)
    urdf_dir = os.path.join(pkg_share, 'urdf')
    xacro_files = glob.glob(os.path.join(urdf_dir, '*.xacro'))
    
    # Fallback en caso de que el xacro esté en un paquete "description"
    if not xacro_files:
        try:
            desc_share = get_package_share_directory('peter_description')
            xacro_files = glob.glob(os.path.join(desc_share, 'urdf', '*.xacro'))
        except Exception:
            pass
            
    if not xacro_files:
        raise FileNotFoundError(f"¡CRÍTICO! No se encontró el archivo .xacro del robot en {urdf_dir}.")
        
    xacro_path = xacro_files[0]  # Toma automáticamente el archivo correcto
    
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Inclusión de Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 5. Generación del Robot y Puentes
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

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # 6. Spawners de ROS 2 Control (Iguales a single_stimulus)
    load_joint_state = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    load_forward = Node(package='controller_manager', executable='spawner', arguments=['forward_position_controller'], output='screen')
    load_head = Node(package='controller_manager', executable='spawner', arguments=['head_segment_controller'], output='screen')
    load_velocity = Node(package='controller_manager', executable='spawner', arguments=['velocity_controller'], output='screen')

    # 7. Nodos de Control y Red Neuronal
    peter_controller = Node(
        package=PACKAGE_NAME,
        executable='peter_controller',
        name='peter_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    neural_network = Node(
        package=PACKAGE_NAME,
        executable='neural_network',
        name='neural_network',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Instrumentación y Estabilidad (10.5 segundos post-warmup)
    telemetry_nodes = TimerAction(period=10.5, actions=[
        Node(package=PACKAGE_NAME, executable='neural_recorder', name='neural_recorder', output='screen', 
             parameters=[{'experiment_type': 'terrain_navigation', 'use_sim_time': True}]),
        Node(package=PACKAGE_NAME, executable='metrics_recorder', name='metrics_recorder', output='screen', 
             parameters=[{'use_sim_time': True}]),
        Node(package=PACKAGE_NAME, executable='peter_stability_monitor', name='stability_monitor', output='screen', 
             parameters=[{'use_sim_time': True}])
    ])

    return LaunchDescription(
        args + [
            set_model_path,
            robot_state_pub,
            gazebo,
            spawn_robot,
            ros_gz_bridge,
            ros_gz_image_bridge,
            load_joint_state,
            load_forward,
            load_head,
            load_velocity,
            peter_controller,
            neural_network,
            telemetry_nodes
        ]
    )