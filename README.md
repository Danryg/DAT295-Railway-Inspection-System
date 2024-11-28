# DAT295-Railway-Inspection-System

## Know problems
1. gz error
    1. Remove gz-harmonic and associated files
    2. install ros-humble-gazebo-ros
    3. git rm -r --cached .
    4. sudo apt autoremove
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
3.  ```  
    colcon build
    ```
4.  To run the worlds
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
