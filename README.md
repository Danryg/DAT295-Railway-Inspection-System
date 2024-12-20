# DAT295-Railway-Inspection-System

The driving is not perfect but works

## Running the program/s

1. Running the amp in gazebo after building

```bash
colcon build --packages-ignore px4 px4_msgs px4_ros_com microxrcedds_agent
source install/setup.bash
```

Copy the models to .Gazebo folder if changed
```bash
bash scripts/copy_models.sh
```

Then to launch gazebo
```bash
ros2 launch test_world_gazebo empty_world.launch.py
```

2. Driving the amp
To make the amp drive
```bash
ros2 run test_world_gazebo drive
```

To make the amp stop
```bash
ros2 run test_world_gazebo stop
```
