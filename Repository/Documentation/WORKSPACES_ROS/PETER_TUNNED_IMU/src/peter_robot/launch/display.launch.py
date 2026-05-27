
# Este codigo se encarga de iniciar RVIZ, junto con los nodos y topicos nativos de ROS cuya funcion final es visualizar un robot descrito por archivos URDF.

import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('peter_robot'))
    xacro_file = os.path.join(pkg_path,'urdf','peter.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                             description='Flag to enable joint_state_publisher_gui'),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        node_robot_state_publisher,
        rviz_node
    ])