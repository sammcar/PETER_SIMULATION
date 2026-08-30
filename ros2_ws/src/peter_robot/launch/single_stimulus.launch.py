#!/usr/bin/env python3
"""
single_stimulus.launch.py
Launch file para "Pruebas con único estímulo".

Argumentos disponibles:
  world_name      : nombre del world file en worlds/ (default: single_stimulus)
  stimulus_type   : "red" | "blue" | "green"          (default: red)
  stimulus_x      : coordenada X del estímulo          (default: 4.0)
  stimulus_y      : coordenada Y del estímulo          (default: 0.0)
  stimulus_z      : coordenada Z del estímulo          (default: 0.5)
  robot_x         : pose inicial del robot X           (default: 0.0)
  robot_y         : pose inicial del robot Y           (default: 0.0)
  robot_z         : pose inicial del robot Z           (default: 1.2)
  record_metrics  : activar nodo de métricas neuronales (default: true)
  use_sim_time    : usar reloj de simulación           (default: true)
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
                LogInfo(msg=f'[single_stimulus] World: {candidate}'),
            ]

    fallback = os.path.join(worlds_dir, 'single_stimulus.world')
    return [
        SetLaunchConfiguration('world_path', fallback),
        LogInfo(msg=f'[single_stimulus] World "{world_name}" not found, using fallback.'),
    ]


def _spawn_stimulus(context, *args, **kwargs):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    stype = context.launch_configurations['stimulus_type']
    sx = context.launch_configurations['stimulus_x']
    sy = context.launch_configurations['stimulus_y']
    sz = context.launch_configurations['stimulus_z']
    models_dir = os.path.join(pkg_share, 'models')
    sdf_file = os.path.join(models_dir, f'{stype}.sdf')

    if not os.path.exists(sdf_file):
        return [LogInfo(msg=f'[single_stimulus] Stimulus SDF not found: {sdf_file}')]

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_stimulus',
            arguments=[
                '-file', sdf_file,
                '-name', f'{stype}_stimulus',
                '-x', sx, '-y', sy, '-z', sz,
            ],
            output='screen',
        )
    ]


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
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('world_name', default_value='single_stimulus',
                              description='World file name (no extension)'),
        DeclareLaunchArgument('stimulus_type', default_value='red',
                              description='Stimulus model: red | blue | green'),
        DeclareLaunchArgument('stimulus_x', default_value='4.0',
                              description='Stimulus X position'),
        DeclareLaunchArgument('stimulus_y', default_value='0.0',
                              description='Stimulus Y position'),
        DeclareLaunchArgument('stimulus_z', default_value='0.5',
                              description='Stimulus Z position'),
        DeclareLaunchArgument('robot_x', default_value='0.0',
                              description='Robot initial X'),
        DeclareLaunchArgument('robot_y', default_value='0.0',
                              description='Robot initial Y'),
        DeclareLaunchArgument('robot_z', default_value='1.2',
                              description='Robot initial Z (spawn height)'),
        DeclareLaunchArgument('record_metrics', default_value='true',
                              description='Launch neural recorder node'),
        DeclareLaunchArgument('noise_level_idx', default_value='0',
                              description='Noise level index (0-4), forwarded to red_neuronal as ROS param "nl"'),
        DeclareLaunchArgument('ablation_mode', default_value='full',
                              description='Ablation mode forwarded to red_neuronal: full | no_lateral_inhibition | threshold_only'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Gazebo server-only (-s), sin GUI Qt. Evita depender de X11/Wayland; '
                                          'necesario para tandas automatizadas (test_manager.py). true|false'),
    ]

    resolve_world = OpaqueFunction(function=_resolve_world)
    spawn_stimulus = OpaqueFunction(function=_spawn_stimulus)

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

    # -s = servidor sin GUI (evita el crash de Qt/xcb cuando no hay display X11
    # utilizable, ej. contenedores sin X forwarding funcional o hosts Wayland
    # sin XWayland en ':0'). -r = arranca la simulacion corriendo de una vez.
    gz_flags = PythonExpression([
        "'-s -r ' if '", LaunchConfiguration('headless'), "'.lower() == 'true' else '-r '"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': [gz_flags] + [LaunchConfiguration('world_path')],
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

    # ---- Controllers (delayed for Gazebo to settle) ----
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

    peter_controller = TimerAction(period=9.0, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='peter_controller',
            name='peter_controller',
            output='screen',
        )
    ])


    # ---- Application nodes ----
    neural_network = TimerAction(period=9.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='red_neuronal',
            name='red_neuronal',
            output='screen',
            parameters=[{
                'nl': ParameterValue(LaunchConfiguration('noise_level_idx'), value_type=int),
                'ablation_mode': LaunchConfiguration('ablation_mode'),
            }],
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

    # ---- Metrics recorder (optional) ----
    neural_recorder = TimerAction(period=10.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='neural_recorder',
            name='neural_recorder',
            output='screen',
            parameters=[{
                'experiment_type': 'single_stimulus',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )
    ])

    # Nodo unificado de telemetría de tu compañero (se lanza en paralelo)
    metrics_recorder = TimerAction(period=10.5, actions=[
        Node(
            package=PACKAGE_NAME,
            executable='metrics_recorder',
            name='metrics_recorder',
            output='screen',
            parameters=[{
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
            spawn_stimulus,
            ros_gz_bridge,
            ros_gz_image_bridge,
            load_joint_state,
            load_forward,
            load_head,
            load_velocity,
            peter_controller,
            neural_network,
            camera_node,
            neural_recorder,
            metrics_recorder,  # <── INYECTADO AQUÍ
        ]
    )
