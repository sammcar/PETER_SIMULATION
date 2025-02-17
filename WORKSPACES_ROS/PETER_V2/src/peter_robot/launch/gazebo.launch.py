import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'peter_robot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Declarar argumento para el mundo
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Nombre del archivo de mundo a cargar'
    )

    world_file = LaunchConfiguration('world')
    
    # Usar PathJoinSubstitution para construir la ruta del mundo de forma segura
    world_path = PathJoinSubstitution([pkg_share, 'worlds', world_file])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock si es true'
    )

    pkg_path = os.path.join(get_package_share_directory('peter_robot'))
    xacro_file = os.path.join(pkg_path,'urdf','peter.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        name='joint_state_publisher'
    )

    peter_controller = Node(
        package='peter_robot',
        executable='peter_controller',
        name='peter_controller',
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

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gazebo_velocity_controllers'],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", 
                   "-entity", 'peter_urdf',
                   "-x", '0.0',
                   "-y", '0.0',
                   "-z", '0.8',
                   "-Y", '0.0'],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', world_path, '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,  # Agregar argumento del mundo
        node_robot_state_publisher,
        gazebo,
        spawn,
        start_joint_state_publisher_cmd, 
        load_joint_state_controller,
        load_forward_command_controller,
        load_velocity_controller,
        peter_controller
    ])
