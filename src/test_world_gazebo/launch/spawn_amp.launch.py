import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    base_directory = 'test_world_gazebo'
    model_path = os.path.join(os.getcwd(), "src", base_directory, "worlds", "models", "amp")

    urdf_path = os.path.join(
        model_path,
        'amp.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amp',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', '3.11'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)




    return ld