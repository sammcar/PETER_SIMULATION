#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
    ExecuteProcess,
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
                LogInfo(msg=f'[transition_evaluation] World: {candidate}'),
            ]

    fallback = os.path.join(worlds_dir, 'empty.sdf')
    return [
        SetLaunchConfiguration('world_path', fallback),
        LogInfo(msg=f'[transition_evaluation] World "{world_name}" not found, using fallback empty.sdf.'),
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

    # ---- Robot description (Tu xacro real e inicialización exacta) ----
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'peter.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file).toxml()

    # ---- Declare arguments ----
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='empty'),
        DeclareLaunchArgument('robot_x', default_value='0.0'),
        DeclareLaunchArgument('robot_y', default_value='0.0'),
        DeclareLaunchArgument('robot_z', default_value='0.3'),
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

    # Spawner para meter el robot en la simulación física
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'peter',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z')
        ],
        output='screen'
    )

    # Puentes de Comunicación (Bridges) ROS2 <-> Gazebo Sim
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/peter/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/peter/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/empty/model/peter/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/peter/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/peter/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
        ],
        remappings=[
            ('/world/empty/model/peter/joint_state', '/joint_states'),
            ('/model/peter/cmd_vel', '/cmd_vel'),
            ('/model/peter/odometry', '/odom')
        ],
        output='screen'
    )

    # Controladores de las articulaciones (Joint Control Spawners)
    load_joint_state = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_forward = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_joint_controller'],
                output='screen'
            )
        ]
    )

    load_velocity = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_velocity_controllers'],
                output='screen'
            )
        ]
    )

    # --- NODOS DEL CONTROL DEL ROBOT Y MONITOREO ---
    # Ajustamos los tiempos para que inicien DESPUÉS de que los controladores estén activos (t > 8.0s)
    
    peter_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package=PACKAGE_NAME,
                executable='peter_controller',
                name='peter_controller',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        ]
    )

    stability_monitor = TimerAction(
        period=12.0,
        actions=[
            Node(
                package=PACKAGE_NAME,
                executable='peter_stability_monitor',
                name='stability_monitor',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            )
        ]
    )

    return LaunchDescription(
        args
        + [
            set_model_path,
            resolve_world,
            robot_state_pub,
            gazebo,
            spawn_robot,
            ros_gz_bridge,
            load_joint_state,
            load_forward,
            load_velocity,
            peter_controller,
            stability_monitor
        ]
    )