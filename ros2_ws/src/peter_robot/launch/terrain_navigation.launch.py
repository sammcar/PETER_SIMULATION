#!/usr/bin/env python3
"""
terrain_navigation.launch.py
Archivo de lanzamiento unificado para la evaluación en terrenos irregulares (Familia C1).
Sigue estrictamente la arquitectura base de inicialización, rutas de recursos y ganchos
de ciclo de vida de single_stimulus.launch.py para garantizar estabilidad en Gazebo Sim.
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    PACKAGE_NAME = 'peter_robot'
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    # 1. Argumentos de Lanzamiento exigidos por el test_manager
    args = [
        DeclareLaunchArgument('world_name', default_value='terrain'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('trial_index', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
    ]

    # 2. Configuración de Rutas de Recursos de Gazebo (Idéntico a single_stimulus)
    # Esto evita el cierre prematuro de Gazebo al resolver las mallas del robot
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'), ':',
            os.path.join(pkg_share, 'urdf')
        ]
    )

    # 3. Procesamiento del Modelo Cinemático (URDF/Xacro)
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    if not os.path.exists(xacro_path):
        xacro_path = os.path.join(pkg_share, 'urdf', 'peter_robot.urdf.xacro')
        
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Inclusión de Gazebo Base (Utiliza la lógica interna de gazebo.launch.py)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 5. Entidad del Robot y Puentes de Comunicación
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

    # 6. Controladores de Articulaciones de ROS 2 Control (Spawners de single_stimulus)
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

    load_head = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_segment_controller'],
        output='screen',
        condition=launch.conditions.IfCondition('false') # Desactivado si no hay segmento superior activo
    ) if hasattr(os, 'launch') else None # Fallback seguro de importación

    # Re-mapeo directo por compatibilidad con la firma funcional de spawners
    load_head = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen', name='head_spawner_bypass')

    load_velocity = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen'
    )

    # 7. Controlador Principal y Red Neuronal Basal
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

    # 8. Infraestructura de Telemetría e Instrumentación (Fase Post-Warmup: 10.5 s)
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
        # Monitor de Estabilidad Cinemática Habilitado para Terreno Irregular
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
            set_model_path,
            robot_state_pub,
            gazebo,
            spawn_robot,
            ros_gz_bridge,
            ros_gz_image_bridge,
            load_joint_state,
            load_forward,
            load_velocity,
            peter_controller,
            neural_network,
            telemetry_nodes
        ]
    )