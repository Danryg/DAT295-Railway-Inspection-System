from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = os.path.join(
        get_package_share_directory('test_world_gazebo')
    )
    config_file = os.path.join(package_share_dir, 'config', 'ekf_config.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])
