#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'peter_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Declarar TODOS los argumentos que envía el test_manager para evitar crashes
    args = [
        DeclareLaunchArgument('world_name', default_value='terrain'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('trial_index', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
    ]

    # 2. Inicializar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world_name': LaunchConfiguration('world_name'), 'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 3. Controladores y Red Neuronal
    control_nodes = [
        Node(package=pkg_name, executable='peter_controller', name='peter_controller', output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=['forward_position_controller'], output='screen'),
        Node(package=pkg_name, executable='neural_network', name='neural_network', output='screen', parameters=[{'use_sim_time': True}])
    ]

    # 4. Nodos de MÉTRICAS Y ESTABILIDAD (Sincronizados a los 10.5s)
    telemetry_nodes = TimerAction(period=10.5, actions=[
        Node(package=pkg_name, executable='neural_recorder', name='neural_recorder', output='screen', 
             parameters=[{'experiment_type': 'terrain_navigation', 'use_sim_time': True}]),
        Node(package=pkg_name, executable='metrics_recorder', name='metrics_recorder', output='screen', 
             parameters=[{'use_sim_time': True}]),
        # AQUI ACTIVAMOS EL STABILITY MONITOR
        Node(package=pkg_name, executable='peter_stability_monitor', name='stability_monitor', output='screen', 
             parameters=[{'use_sim_time': True}])
    ])

    return LaunchDescription(args + [gazebo] + control_nodes + [telemetry_nodes])