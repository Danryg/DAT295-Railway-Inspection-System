cmake_minimum_required(VERSION 3.5)
project(object_detection_eval)

# ============================================
# ✅ Step 1: Find Required Dependencies
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================
# ✅ Step 2: Generate ROS 2 Messages (First)
# ============================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBoxModel.msg"
  "msg/ModelBoundingBox.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ============================================
# ✅ Step 3: Ensure Message Headers Are Found
# ============================================
include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# ============================================
# ✅ Step 4: Build Libraries and Executables
# ============================================

# Create a shared library for ModelPosition
add_library(model_position_lib src/model_position.cpp)
target_link_libraries(model_position_lib "${cpp_typesupport_target}")
ament_target_dependencies(model_position_lib rclcpp gazebo_msgs geometry_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(model_position_lib "${cpp_typesupport_target}")

# Create a shared library for the Gazebo World Plugin
add_library(bounding_box_world_plugin SHARED src/bounding_box_world_plugin.cpp)
target_link_libraries(bounding_box_world_plugin "${cpp_typesupport_target}")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_target_dependencies(bounding_box_world_plugin rclcpp gazebo_dev std_msgs)
target_link_libraries(bounding_box_world_plugin "${cpp_typesupport_target}")

# Define executables (nodes)
add_executable(model_position_node src/model_position_node.cpp)
target_link_libraries(model_position_node model_position_lib)

add_executable(filtered_model_position_node src/filtered_model_position.cpp)
target_link_libraries(filtered_model_position_node model_position_lib)

# Link dependencies
ament_target_dependencies(model_position_lib rclcpp gazebo_msgs geometry_msgs std_msgs)
ament_target_dependencies(model_position_node rclcpp gazebo_msgs geometry_msgs std_msgs)
ament_target_dependencies(filtered_model_position_node rclcpp gazebo_msgs geometry_msgs std_msgs)
ament_target_dependencies(bounding_box_world_plugin rclcpp gazebo_dev std_msgs)

# ============================================
# ✅ Step 5: Install Targets
# ============================================
install(TARGETS model_position_lib
    LIBRARY DESTINATION lib
)

install(TARGETS
  model_position_node
  filtered_model_position_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bounding_box_world_plugin
  LIBRARY DESTINATION lib
)

# ============================================
# ✅ Step 6: Call `ament_package()` Last
# ============================================
ament_package()
