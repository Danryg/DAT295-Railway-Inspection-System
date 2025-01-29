cmake_minimum_required(VERSION 3.8)
project(behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(behaviortree_ros2 REQUIRED)
# find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_DEPS
    # behaviortree_ros2
    std_msgs
    std_srvs
    # btcpp_ros2_interfaces 
    irobot_create_msgs
  )

# add_executable(sample_bt_executor src/sample_bt_executor.cpp)
# ament_target_dependencies(sample_bt_executor ${THIS_PACKAGE_DEPS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  src/turtle_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/behavior.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
