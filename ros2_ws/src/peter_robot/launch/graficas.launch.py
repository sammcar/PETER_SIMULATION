import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('peter_robot')

    rviz_config = os.path.join(pkg_share, 'launch', 'rviz_configuration.rviz')
    plotjuggler_config = os.path.join(pkg_share, 'launch', 'plotjuggler_config.xml')

    rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    plotjuggler_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'plotjuggler', 'plotjuggler',
            '-l', plotjuggler_config
        ],
        output='screen'
    )

    return LaunchDescription([
        #rviz_cmd,
        plotjuggler_cmd,
    ])
