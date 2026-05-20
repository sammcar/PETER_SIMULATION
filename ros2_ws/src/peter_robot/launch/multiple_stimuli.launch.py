#!/usr/bin/env python3
"""
multiple_stimuli.launch.py
Launch file para "Pruebas con múltiples estímulos".

Argumentos disponibles:
  world_name      : nombre del world file en worlds/ (default: multiple_stimuli)
  spawn_red       : lanzar estímulo hostil rojo      (default: true)
  spawn_blue      : lanzar estímulo apetente azul    (default: true)
  spawn_green     : lanzar obstáculo verde           (default: false)
  red_x, red_y    : posición del estímulo rojo       (default: 4.0,  2.0)
  blue_x, blue_y  : posición del estímulo azul       (default: 4.0, -2.0)
  green_x, green_y: posición del obstáculo verde     (default: 3.0,  0.0)
  robot_x, robot_y: pose inicial del robot           (default: 0.0,  0.0)
  record_metrics  : activar nodo de métricas         (default: true)
  use_sim_time    : usar reloj de simulación         (default: true)
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
                LogInfo(msg=f'[multiple_stimuli] World: {candidate}'),
            ]

    fallback = os.path.join(worlds_dir, 'multiple_stimuli.world')
    return [
        SetLaunchConfiguration('world_path', fallback),
        LogInfo(msg=f'[multiple_stimuli] World "{world_name}" not found, using fallback.'),
    ]


def _spawn_stimuli(context, *args, **kwargs):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    models_dir = os.path.join(pkg_share, 'models')
    actions = []

    stimuli = [
        ('spawn_red',   'red',   'red_stimulus',   'red_x',   'red_y',   '0.5'),
        ('spawn_blue',  'blue',  'blue_stimulus',  'blue_x',  'blue_y',  '0.5'),
        ('spawn_green', 'green', 'green_obstacle', 'green_x', 'green_y', '0.5'),
    ]

    for flag, model, entity, x_arg, y_arg, z_default in stimuli:
        if context.launch_configurations.get(flag, 'false').lower() != 'true':
            continue
        sdf_file = os.path.join(models_dir, f'{model}.sdf')
        if not os.path.exists(sdf_file):
            actions.append(LogInfo(msg=f'[multiple_stimuli] SDF not found: {sdf_file}'))
            continue
        x = context.launch_configurations.get(x_arg, '4.0')
        y = context.launch_configurations.get(y_arg, '0.0')
        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{entity}',
                arguments=[
                    '-file', sdf_file,
                    '-name', entity,
                    '-x', x, '-y', y, '-z', z_default,
                ],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # ---- Model path ----
    model_path = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '/usr/share/gazebo/models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{model_path}:{existing_model_path}',
    )

    # ---- Robot description ----
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'peter.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ---- Declare arguments ----
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='multiple_stimuli'),
        DeclareLaunchArgument('spawn_red',   default_value='true',
                              description='Spawn red (hostile) stimulus'),
        DeclareLaunchArgument('spawn_blue',  default_value='true',
                              description='Spawn blue (appetitive) stimulus'),
        DeclareLaunchArgument('spawn_green', default_value='false',
                              description='Spawn green (obstacle) stimulus'),
        DeclareLaunchArgument('red_x',   default_value='4.0'),
        DeclareLaunchArgument('red_y',   default_value='2.0'),
        DeclareLaunchArgument('blue_x',  default_value='4.0'),
        DeclareLaunchArgument('blue_y',  default_value='-2.0'),
        DeclareLaunchArgument('green_x', default_value='3.0'),
        DeclareLaunchArgument('green_y', default_value='0.0'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='1.2'),
        DeclareLaunchArgument('record_metrics', default_value='true'),
    ]

    resolve_world = OpaqueFunction(function=_resolve_world)
    spawn_stimuli = OpaqueFunction(function=_spawn_stimuli)

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

    # ---- Controllers ----
    load_joint_state = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_state_broadcaster'],
            output='screen',
        )
    ])

    load_forward = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'gazebo_joint_controller'],
            output='screen',
        )
    ])

    load_head = TimerAction(period=7.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'head_controller'],
            output='screen',
        )
    ])

    load_velocity = TimerAction(period=8.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'gazebo_velocity_controllers'],
            output='screen',
        )
    ])

    # ---- Application nodes ----
    neural_network = TimerAction(period=9.0, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='red_neuronal',
            name='red_neuronal',
            output='screen',
        )
    ])

    camera_node = TimerAction(period=10.0, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='camera_node',
            name='camera_node',
            output='screen',
        )
    ])

    # ---- Metrics recorder ----
    neural_recorder = TimerAction(period=10.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='neural_recorder',
            name='neural_recorder',
            output='screen',
            parameters=[{
                'experiment_type': 'multiple_stimuli',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
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
            spawn_stimuli,
            ros_gz_bridge,
            ros_gz_image_bridge,
            load_joint_state,
            load_forward,
            load_head,
            load_velocity,
            neural_network,
            camera_node,
            neural_recorder,
        ]
    )
