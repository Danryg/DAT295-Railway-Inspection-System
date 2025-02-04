cmake_minimum_required(VERSION 3.5)
project(object_detection_eval)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# Declare the executable
add_executable(model_position_node src/model_position.cpp)

# Link dependencies
ament_target_dependencies(model_position_node rclcpp gazebo_msgs geometry_msgs)

# Install the executable
install(TARGETS
  model_position_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
