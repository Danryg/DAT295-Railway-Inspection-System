import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  package_name = 'test_world_gazebo'
  bringup_dir = get_package_share_directory(package_name)

  world_file_path = 'worlds/brand_new_world.world'

  ############ You do not need to change anything below this line #############

  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
  pkg_share = FindPackageShare(package=package_name).find(package_name)

  default_world_path = os.path.join(pkg_share, world_file_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path


  headless = LaunchConfiguration('headless')
  use_sim_time = 'true'


  use_simulator = 'True'
  use_robot_state_pub = 'True'

  headless = LaunchConfiguration('headless')
  world = LaunchConfiguration('world')
  pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
          'y': LaunchConfiguration('y_pose', default='-0.50'),
          'z': LaunchConfiguration('z_pose', default='0.01'),
          'R': LaunchConfiguration('roll', default='0.00'),
          'P': LaunchConfiguration('pitch', default='0.00'),
          'Y': LaunchConfiguration('yaw', default='0.00')}
  robot_name = 'turtlebot3_waffle'
  robot_sdf = os.path.join(bringup_dir, 'worlds', 'waffle.model'),



  use_simulator = LaunchConfiguration('use_simulator')

  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    'use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_world_cmd = DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            description='World to load into Gazebo'
        )

  launch_file_dir = os.path.join(get_package_share_directory('test_world_gazebo'), 'launch')
  x_pose = LaunchConfiguration('x_pose', default='0.0')
  y_pose = LaunchConfiguration('y_pose', default='0.0')
  spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_amp.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
   )

  # Start Gazebo client
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))


  urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
  with open(urdf, 'r') as infp:
      robot_description = infp.read()

  start_robot_state_publisher_cmd = Node(
      condition=IfCondition(use_robot_state_pub),
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': robot_description}],
  )

  start_gazebo_spawner_cmd = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      output='screen',
      arguments=[
          '-entity', robot_name,
          '-file', robot_sdf,
          '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
          '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_use_world_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)

#new
  ld.add_action(declare_use_robot_state_pub_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_turtlebot_cmd)

  #new

  ld.add_action(start_gazebo_spawner_cmd)
  ld.add_action(start_robot_state_publisher_cmd)

  return ld
