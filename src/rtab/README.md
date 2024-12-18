# RTABMAP with the Drone and Teleop
This is a quick guide to set up and run the drone with RTABMAP and teleop navigation. The mapping will kind of work, need to get the timestamps of the pointclouds and set a better transform between the imu and the lidar. 
## Building RTABMAP
From DAT295-Railway-Inspection_System: 

´´´rosdep update && rosdep install --from-paths src --ignore-src -r -y```
´´´export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)```
´´´colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release```
´´´source install/setup.bash´´´
This will build all necessary packages including rtabmap for slam and px4_ros_com for the teleop. 


## Micro-XRCE-DDS-Agent
Before running the drone or the teleop, make sure you have MicroXRCEDDSAgent working. 
follow this guide: https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html 

Then launch the MicrXRCEDDSAgent:
´´´MicroXRCEDDSAgent udp4 -p 8888´´´


## Launching the drone
Then launch the drone: 
In PX4-Autopilot run:
´´´make px5_sitl gazebo-classic_iris_velodyne_cam__test´´´ to run the drone in a test world with some trees. 

## Setting transform 
´´´ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base iris_imu_link´´´

## Launching RTABMap - 3D lidar example
´´´ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points imu_topic:=/imu frame_id:=base´´´

## Teleop
´´´ros2 run px4_ros_com velocity_control.py´´´
´´´ros2 run px4_ros_com control.py´´´
