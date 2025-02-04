#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    base_directory = "test_world_gazebo"
    launch_file_dir = os.path.join(
        get_package_share_directory(base_directory), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    model_path = os.path.join(os.getcwd(), "src", base_directory, "worlds", "models")

    os.environ["GAZEBO_MODEL_PATH"] = os.path.abspath(model_path)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # AMP spawn positions
    amp_spawn_x = "1.06"
    amp_spawn_y = "-56.37"
    amp_spawn_z = "0.7"

    # TurtleBot spawn positions
    tb_spawn_x = '4.06'
    tb_spawn_y = '0.0'
    tb_spawn_z = '0.2'

    # Unique LaunchConfigurations for AMP
    amp_x_pose = LaunchConfiguration("amp_x_pose", default=amp_spawn_x)
    amp_y_pose = LaunchConfiguration("amp_y_pose", default=amp_spawn_y)
    amp_z_pose = LaunchConfiguration("amp_z_pose", default=amp_spawn_z)

    # Unique LaunchConfigurations for TurtleBot
    tb_x_pose = LaunchConfiguration("tb_x_pose", default=tb_spawn_x)
    tb_y_pose = LaunchConfiguration("tb_y_pose", default=tb_spawn_y)
    tb_z_pose = LaunchConfiguration("tb_z_pose", default=tb_spawn_z)

    world = os.path.join(
        get_package_share_directory(base_directory), "worlds", "brand_new_world.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "x_pose": tb_x_pose,
            "y_pose": tb_y_pose,
            "z_pose": tb_z_pose,
        }.items(),
    )

    spawn_amp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_amp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "x_pose": amp_x_pose,
            "y_pose": amp_y_pose,
            "z_pose": amp_z_pose,
        }.items(),
    )

    camera_node = Node(
        package="camera_handler",
        executable="picture_taker",
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_amp_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(robot_state_publisher_amp_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(camera_node)

    return ld

