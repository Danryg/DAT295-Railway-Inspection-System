# DAT295-Railway-Inspection-System

## Launching the drone
Do the prerequisites to build the drone: 
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
1. Navigate to PX4-Autopilot
2. For only the drone
    ```
    make px4_sitl_default gazebo-classic_iris_velodyne_cam
    ```  
3. For drone in a world 
    ```
        make px4_sitl gazebo-classic_iris_velodyne_cam__v1_simple_world
    ```
## Running MicroXRCEAgent (drivers between PX4 and ROS2)

Before running the drone or the teleop, make sure you have MicroXRCEDDSAgent working. 
follow this guide: https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html 

Then launch the MicrXRCEDDSAgent:
```
MicroXRCEAgent udp4 -p 8888
```


## Teleop for the drone
```
ros2 run px4_ros_com velocity_control.py
```
```
ros2 run px4_ros_com control.py
```

## Docking for the drone
```
ros2 launch ros2_aruco aruco_recognition.launch
```
```
ros2 run docking docking_node
```

# TurtleBot with LiDAR object detection

## TB3 with velodyne in the world
```
ros2 launch test_world_gazebo amp_and_tb_3d_lidar_launch.py
```

## Start Euclidean clustering
```
ros2 launch lidar_object_detection euclidean_cluster_launch.py namespace:=cluster02 input_pointcloud:=/velodyne_points euclidean_param_path:=(path to yaml file)
```

# Turtlebot + mapping

## TB3 in our world
```
ros2 launch test_world_gazebo turtlebot_launch.py
```

## Slam toolbox
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

## Nav2 
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

## Rviz2
```
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## Map saver
```
ros2 run nav2_map_server map_saver_cli -f current_map
```

# AMP speed controller topic
## publish to a float32 to the /amp_robot/speed topic in order to control the speed via the controller
## an example of how to do this can be found in the input_handler.py and speed_controller.py files in ~/DAT295-Railway-Inspection-System/src/test_world_gazebo/test_world_gazebo

# System Requirements
1. Ubuntu 22.04
2. ROS2 Humble
3. Gazebo Classic (11)
4. Numpy < 2
5. OpenCV >= 4.7 
