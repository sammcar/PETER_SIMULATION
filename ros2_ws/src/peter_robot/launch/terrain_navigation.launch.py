#!/usr/bin/env python3
"""
terrain_navigation.launch.py
Archivo de lanzamiento unificado para la evaluación en terrenos irregulares (Familia C1).
Copia EXACTA de la arquitectura funcional de single_stimulus.launch.py, 
adaptada para el mundo de terreno y con el monitor de estabilidad activado.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'peter_robot'

def _resolve_world(context, *args, **kwargs):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    worlds_dir = os.path.join(pkg_share, 'worlds')
    world_name = context.launch_configurations['world_name']

    for ext in ('.world', '.sdf'):
        candidate = os.path.join(worlds_dir, world_name + ext)
        if os.path.exists(candidate):
            return [
                SetLaunchConfiguration('world_path', candidate),
                LogInfo(msg=f'[terrain_navigation] World: {candidate}'),
            ]

    fallback = os.path.join(worlds_dir, 'terrain.world')
    return [
        SetLaunchConfiguration('world_path', fallback),
        LogInfo(msg=f'[terrain_navigation] World "{world_name}" not found, using fallback.'),
    ]

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # ---- Model path (GAZEBO_MODEL_PATH) ----
    model_path = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '/usr/share/gazebo/models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{model_path}:{existing_model_path}',
    )

    # ---- Robot description (usando tu xacro real) ----
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'peter.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ---- Declare arguments ----
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='terrain'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('trial_index', default_value='0'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
        DeclareLaunchArgument('noise_level_idx', default_value='0'),
        DeclareLaunchArgument('illum_direction', default_value='-1'),
    ]

    resolve_world = OpaqueFunction(function=_resolve_world)

    # ---- Core nodes ----
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Gazebo se lanza con las librerías nativas de ros_gz_sim igual que en tu archivo original
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r '] + [LaunchConfiguration('world_path')],
            'use_sim_time': 'true',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'peter_urdf',
            '-name', 'peter',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', 'gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
    )

    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
    )

    # ---- Controllers (Usando tu ExecuteProcess exacto) ----
    load_joint_state = TimerAction(period=5.0, actions=[
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen')
    ])

    load_forward = TimerAction(period=6.0, actions=[
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_joint_controller'], output='screen')
    ])

    load_head = TimerAction(period=7.0, actions=[
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'head_controller'], output='screen')
    ])

    load_velocity = TimerAction(period=8.0, actions=[
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_velocity_controllers'], output='screen')
    ])

    peter_controller = TimerAction(period=9.0, actions=[
        Node(package=PACKAGE_NAME, executable='peter_controller', name='peter_controller', output='screen')
    ])

# ---- Establecer modo H (Híbrido) ----
    set_hybrid_mode = TimerAction(period=14.5, actions=[
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/peter_mode', 'std_msgs/msg/String', '{data: "H"}'],
            output='screen'
        )
    ])

    # ---- Application nodes ----
    neural_network = TimerAction(period=19.5, actions=[
        Node(
            package=PACKAGE_NAME, 
            executable='fsm_arbitration', 
            name='fsm_arbitration', 
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
             )
    ])

    camera_node = TimerAction(period=13.0, actions=[
        Node(package=PACKAGE_NAME, executable='camera_node', name='camera_node', output='screen')
    ])

    # ---- Metrics & Stability Monitor ----
    neural_recorder = TimerAction(period=13.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='neural_recorder',
            name='neural_recorder',
            output='screen',
            parameters=[{'experiment_type': 'terrain_navigation', 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ])

    RMSE_node = TimerAction(period=13.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='RMSE_node',
            name='RMSE_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ])

    metrics_recorder = TimerAction(period=13.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='metrics_recorder',
            name='metrics_recorder',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ])

    stability_monitor = TimerAction(period=13.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='peter_stability_monitor',
            name='stability_monitor',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ])

    return LaunchDescription(
        args
        + [
            set_model_path,
            resolve_world,
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
            set_hybrid_mode,
            neural_network,
            camera_node,
            neural_recorder,
            metrics_recorder,
            RMSE_node,
            stability_monitor
        ]
    )