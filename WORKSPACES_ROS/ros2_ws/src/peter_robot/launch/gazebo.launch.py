import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import TextSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'peter_robot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    world_file_path = 'empty.world'
    world_path = os.path.join(pkg_share, 'worlds',  world_file_path)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_path = os.path.join(get_package_share_directory('peter_robot'))
    xacro_file = os.path.join(pkg_path,'urdf','peter.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
    'world',
    default_value=default_world,
    description='World to load'
    )
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )



    peter_controller = Node(
        package='peter_robot',
        executable='peter_controller',
        name='peter_controller',
        output='screen'
    )

    camera_node = Node(
        package='peter_robot',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen' 
    )

    load_forward_command_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
            'gazebo_joint_controller'],
        output='screen'
    )
    
    cabeza_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
            'head_controller'],
        output='screen'
    )


    # Cargar el controlador de velocidad para las ruedas
    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gazebo_velocity_controllers'],
        output='screen'
    )


    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
   # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-entity', 'peter_urdf',
                '-name', 'peter',
                '-z', '0.8'],
        output='screen'
    )


    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items())

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    return LaunchDescription([
        declare_use_sim_time_cmd,
        node_robot_state_publisher,
        world_arg,
        gazebo,
        spawn_entity,
        load_joint_state_controller,
        load_forward_command_controller,
        load_velocity_controller,
        cabeza_controller,
        ros_gz_bridge,
        ros_gz_image_bridge,
        peter_controller,
        camera_node
    ])
