cmake_minimum_required(VERSION 3.5)
project(object_detection_eval)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create a shared library for ModelPosition
add_library(model_position_lib src/model_position.cpp)
ament_target_dependencies(model_position_lib rclcpp gazebo_msgs geometry_msgs)

# Define executables (nodes)
add_executable(model_position_node src/model_position_node.cpp)
target_link_libraries(model_position_node model_position_lib)

add_executable(filtered_model_position_node src/filtered_model_position.cpp)
target_link_libraries(filtered_model_position_node model_position_lib)

# Link dependencies
ament_target_dependencies(model_position_lib rclcpp gazebo_msgs geometry_msgs)
ament_target_dependencies(model_position_node rclcpp gazebo_msgs geometry_msgs)
ament_target_dependencies(filtered_model_position_node rclcpp gazebo_msgs geometry_msgs)

# Install the executables
install(TARGETS
  model_position_node
  filtered_model_position_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

