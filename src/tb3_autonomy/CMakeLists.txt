cmake_minimum_required(VERSION 3.8)
project(tb3_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(behaviortree_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Install directories
install(DIRECTORY
  bt_xml launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(
    tb3_behaviors
    PACKAGE_DIR python/tb3_behaviors)

# Install Python scripts
install(PROGRAMS
    scripts/autonomy_node.py
    scripts/test_move_base.py
    scripts/test_vision.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
