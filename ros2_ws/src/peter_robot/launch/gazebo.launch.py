import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetLaunchConfiguration, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def set_world_path(context, *args, **kwargs):
    """ Busca el mundo correcto y guarda la ruta en 'world_path' dentro del launch """
    package_name = 'peter_robot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    worlds_dir = os.path.join(pkg_share, 'worlds')

    world_name = context.launch_configurations['world_name']
    
    # Buscar el archivo correspondiente en worlds/ (SDF o .world)
    world_path_sdf = os.path.join(worlds_dir, f'{world_name}.sdf')
    world_path_world = os.path.join(worlds_dir, f'{world_name}.world')

    # Seleccionar el archivo que exista
    if os.path.exists(world_path_sdf):
        selected_world = world_path_sdf
    elif os.path.exists(world_path_world):
        selected_world = world_path_world
    else:
        selected_world = os.path.join(worlds_dir, 'depot.sdf')
        print(f'⚠️ Mundo "{world_name}" no encontrado. Cargando "depot.sdf" por defecto.')

    return [
        SetLaunchConfiguration('world_path', selected_world),
        LogInfo(msg=f'✅ Mundo seleccionado: {selected_world}')
    ]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = 'peter_robot'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # 🔹 Definir model_path antes de usarlo (apunta a la carpeta instalada)
    model_path = os.path.join(pkg_share, 'models')
    default_gazebo_model_path = '/usr/share/gazebo/models'  # Path por defecto en Ubuntu
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', default_gazebo_model_path)
    combined_model_path = f"{model_path}:{existing_model_path}"

    print(f"📦 GAZEBO_MODEL_PATH seteado a: {combined_model_path}")

    set_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=combined_model_path
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_path = os.path.join(get_package_share_directory('peter_robot'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'peter.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    declare_world_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Name of the world file (without extension, must be in worlds/ folder)'
    )

    set_world_path_action = OpaqueFunction(function=set_world_path)

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    peter_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='peter_robot',
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
                package='peter_robot',
                executable='camera_node',
                name='camera_node',
                output='screen'
            )
        ]
    )

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


    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-entity', 'peter_urdf',
                '-name', 'peter',
                '-z', '1.2'],
        output='screen'
    )

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

    # spawn_green_sphere = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-file', os.path.join(pkg_path, 'models', 'green.sdf'),
    #         '-name', 'green',
    #         '-x', '4.0', '-y', '0.0', '-z', '0.5'
    #     ],
    #     output='screen'
    # )

    spawn_blue_sphere = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_path, 'models', 'blue.sdf'),
            '-name', 'blue_sphere',
            '-x', '4.0', '-y', '-2.0', '-z', '0.5'
        ],
        output='screen'
    )

    spawn_red_sphere = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_path, 'models', 'red.sdf'),
            '-name', 'red_sphere',
            '-x', '4.0', '-y', '2.0', '-z', '0.5'
        ],
        output='screen'
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
        #spawn_green_sphere,
        #spawn_red_sphere,
        #spawn_blue_sphere,
        camera_node
    ])
