#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = 'peter_robot'

def set_world_path(context, *args, **kwargs):
    """ Busca el mundo correcto y guarda la ruta en 'world_path' dentro del launch """
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    worlds_dir = os.path.join(pkg_share, 'worlds')
    world_name = context.launch_configurations['world_name']
    
    world_path_sdf = os.path.join(worlds_dir, f'{world_name}.sdf')
    world_path_world = os.path.join(worlds_dir, f'{world_name}.world')

    if os.path.exists(world_path_sdf):
        selected_world = world_path_sdf
    elif os.path.exists(world_path_world):
        selected_world = world_path_world
    else:
        selected_world = os.path.join(worlds_dir, 'empty.sdf')
        print(f'⚠️ Mundo "{world_name}" no encontrado. Cargando "empty.sdf" por defecto.')

    return [
        SetLaunchConfiguration('world_path', selected_world),
        LogInfo(msg=f'✅ Mundo seleccionado: {selected_world}')
    ]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Configuración de variables de entorno de recursos
    model_path = os.path.join(pkg_share, 'models')
    default_gazebo_model_path = '/usr/share/gazebo/models'
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', default_gazebo_model_path)
    combined_model_path = f"{model_path}:{existing_model_path}"

    set_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=combined_model_path
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Name of the world file (without extension)'
    )

    set_world_path_action = OpaqueFunction(function=set_world_path)

    # Procesamiento e inyección del URDF original
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    xacro_file = os.path.join(pkg_path, 'urdf', 'peter.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': use_sim_time}]
    )

    # Instanciación de Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r '] + [LaunchConfiguration('world_path')],
            'use_sim_time': 'true',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Spawner físico con nomenclatura exacta de entidad
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-entity', 'peter_urdf',
                '-name', 'peter',
                '-z', '1.2'],
        output='screen'
    )

    # Puentes de comunicación basados en archivos de configuración YAML nativos
    bridge_params = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'gz_bridge.yaml')
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

    # Carga secuencial cronometrada de controladores de hardware (ros2_control)
    load_joint_state_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    load_forward_command_controller = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_joint_controller'],
                output='screen'
            )
        ]
    )

    cabeza_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'head_controller'],
                output='screen'
            )
        ]
    )

    load_velocity_controller = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gazebo_velocity_controllers'],
                output='screen'
            )
        ]
    )

    # Ejecución sincronizada de los nodos lógicos de control y observación externa
    peter_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package=PACKAGE_NAME,
                executable='peter_controller',
                name='peter_controller',
                output='screen'
            )
        ]
    )

    camera_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package=PACKAGE_NAME,
                executable='camera_node',
                name='camera_node',
                output='screen'
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
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        set_model_path_env,
        set_world_path_action,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        load_joint_state_controller,
        load_forward_command_controller,
        cabeza_controller,
        load_velocity_controller,
        peter_controller,
        camera_node,
        stability_monitor
    ])