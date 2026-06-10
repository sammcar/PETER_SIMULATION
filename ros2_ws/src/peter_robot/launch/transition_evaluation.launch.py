#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'peter_robot'

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    
    # ---- Model path (GAZEBO_MODEL_PATH) ----
    model_path = os.path.join(pkg_share, 'models')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '/usr/share/gazebo/models')
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f'{model_path}:{existing_model_path}',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world_name', default_value='empty', description='World file name (without extension)'
    )

    # Configuración de archivos de soporte e inicialización de controladores
    xacro_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'peter.urdf.xacro'
    )
    config_joint_controllers = os.path.join(pkg_share, 'config', 'joint_controllers.yaml')

    # ---- Core nodes ----
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': xacro.process_file(xacro_file).toxml(),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Lanzar Simulador Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world_name'), '.sdf']}.items()
    )

    # Spawner para meter el robot en la simulación física
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'peter', '-z', '0.3'],
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

    # Controladores de las articulaciones (Joint Controllers)
    load_joint_state = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"], output="screen"
    )
    load_forward = Node(
        package="controller_manager", executable="spawner",
        arguments=["forward_position_controller"], output="screen"
    )
    load_velocity = Node(
        package="controller_manager", executable="spawner",
        arguments=["velocity_controller"], output="screen"
    )

    # --- NODOS DEL CONTROL DEL ROBOT Y MONITOREO (Sin extensión .py) ---
    
    peter_controller = TimerAction(
        period=5.0,
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
        period=7.0,
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

    return LaunchDescription([
        set_model_path,
        declare_use_sim_time_cmd,
        declare_world_cmd,
        robot_state_pub,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        load_joint_state,
        load_forward,
        load_velocity,
        peter_controller,
        stability_monitor
    ])