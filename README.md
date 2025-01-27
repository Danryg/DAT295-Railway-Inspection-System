# DAT295-Railway-Inspection-System

## Know problems
1. gz error
    1. Remove gz-harmonic and associated files
    2. install ros-humble-gazebo-ros
2. Other erros
    1. git rm -r --cached .
    2. sudo apt autoremove

## Running the program/s

1. git clone -b sim_dev --recurse-submodules $git_link
2. To run the drone
    1. Navigate to PX4-Autopilot
    2. For only the drone
        ```
        make px4_sitl_default gazebo-classic_iris_velodyne_cam
        ```  
    3. For drone in world 
        ```
        make px4_sitl gazebo-classic_iris_velodyne_cam__v1_simple_world
        ```
3.  If drone is not to be used run this directly
    ```  
    colcon build
    ```
4. 
    ``` 
    source install/setup.bash
    ```
    If there is an error run it again
5.  Change the worlds in ```gazebo.launch.py``` 

    To run the world/s
    1.   Copy the folders in test_world_gazebo/worlds/models to ~/.gazebo/models
    2.
       ```
       ros2 launch test_world_gazebo gazebo.launch.py
       ```
## Running MicroXRCEAgent (drivers between PX4 and ROS2)
From the root directory
```
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Then to run it
```
MicroXRCEAgent udp4 -p 8888
```
## Building a new package
Navigate to src/ 

For C++ type build
```
ros2 pkg create $PKG_NAME --build-type ament_cmake
```
For python type build
```
ros2 pkg create $PKG_NAME --build-type ament_python
```

# RTABMAP with the Drone and Teleop
This is a quick guide to set up and run the drone with RTABMAP and teleop navigation. The mapping will kind of work, need to get the timestamps of the pointclouds and set a better transform between the imu and the lidar. 

## Micro-XRCE-DDS-Agent
Before running the drone or the teleop, make sure you have MicroXRCEDDSAgent working. 
follow this guide: https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html 

Then launch the MicrXRCEDDSAgent:
```MicroXRCEAgent udp4 -p 8888```

## Launching the drone
Do the prerequisites to build the drone: 
```bash ./PX4-Autopilot/Tools/setup/ubuntu.sh```
Then launch the drone: 
In PX4-Autopilot run:
```make px4_sitl gazebo-classic_iris_velodyne_cam__test``` to run the drone in a test world with some trees. 

## Building RTABMAP
From DAT295-Railway-Inspection_System: 

```rosdep update && rosdep install --from-paths src --ignore-src -r -y```

```export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)```
```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```
```source install/setup.bash```

This will build all necessary packages including rtabmap for slam and px4_ros_com for the teleop. 

## Setting transform 
```ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base iris_imu_link```

## Launching RTABMap - 3D lidar example
```ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points imu_topic:=/imu frame_id:=base```

## Teleop
```ros2 run px4_ros_com velocity_control.py```
```ros2 run px4_ros_com control.py```


# Turtlebot + mapping

## TB3 in our world
```ros2 launch test_world_gazebo turtlebot_launch.py```

## Slam toolbox
```ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True```

## Nav2 
```ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True```

## Rviz2
```ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz```


# AMP speed controller topic
## publish to a float32 to the /amp_robot/speed topic in order to control the speed via the controller
## an example of how to do this can be found in the input_handler.py and speed_controller.py files in ~/DAT295-Railway-Inspection-System/src/test_world_gazebo/test_world_gazebo
