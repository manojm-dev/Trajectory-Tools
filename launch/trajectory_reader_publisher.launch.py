import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('trajectory_tools'),
        'config',
        'trajectory_reader_publisher.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_tools',
            executable='trajectory_reader_publisher',
            name='trajectory_reader_publisher',
            output='screen',
            parameters=[config]
        )
    ])