import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
 
  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  package_name = 'test_world_gazebo'
  # robot_name_in_model = 'hubert'
#   rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'
  sdf_file_path = 'models/model.sdf'
  world_file_path = 'worlds/v1_simple_world.world'
     
  # Pose where we want to spawn the robot
  # spawn_x_val = '0.0'
  # spawn_y_val = '0.0'
  # spawn_z_val = '0.0'
  # spawn_yaw_val = '0.00'
 
  ############ You do not need to change anything below this line #############
   
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package=package_name).find(package_name)
  default_urdf_model_path = os.path.join(pkg_share, sdf_file_path)
  default_world_path = os.path.join(pkg_share, world_file_path)
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
   
  # Launch configuration variables specific to simulation
  # gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  # namespace = LaunchConfiguration('namespace')
#   rviz_config_file = LaunchConfiguration('rviz_config_file')
  sdf_model = LaunchConfiguration('sdf_model')
  # use_namespace = LaunchConfiguration('use_namespace')
  # use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  # use_rviz = LaunchConfiguration('use_rviz')
  # use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  # world = LaunchConfiguration('world')
   
  # Declare the launch arguments  
  # declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
  #   name='gui',
  #   default_value='True',
  #   description='Flag to enable joint_state_publisher_gui')
     
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
             
#   declare_rviz_config_file_cmd = DeclareLaunchArgument(
#     name='rviz_config_file',
#     default_value=default_rviz_config_path,
#     description='Full path to the RVIZ config file to use')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
 
  declare_sdf_model_path_cmd = DeclareLaunchArgument(
    name='sdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot sdf file')
     
  # declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
  #   name='use_robot_state_pub',
  #   default_value='True',
  #   description='Whether to start the robot state publisher')
 
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_use_world_cmd = DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            # choices=['moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        )
   
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  # start_robot_state_publisher_cmd = Node(
  #   package='robot_state_publisher',
  #   executable='robot_state_publisher',
  #   parameters=[{'robot_description': Command(['xacro ', sdf_model])}])
 
  # Publish the joint states of the robot
  # start_joint_state_publisher_cmd = Node(
  #   package='joint_state_publisher',
  #   executable='joint_state_publisher',
  #   name='joint_state_publisher',
  #   condition=UnlessCondition(gui))
  
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
   )
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'velodyne', 
                # '-topic', 'robot_description',
                '-file', sdf_model,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                    '-Y', '0.0'],
                    output='screen')
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  # ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_world_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_sdf_model_path_cmd)
  # ld.add_action(declare_use_robot_state_pub_cmd)  
  # ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
 
  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  # ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_joint_state_publisher_cmd)
 
  return ld