cmake_minimum_required(VERSION 3.8)
project(lbx_franka_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

# Python modules and executables are handled by setup.py for this package if it's Python-based
# If it has C++ nodes, add them here:
# add_executable(my_cpp_node src/my_cpp_node.cpp)
# ament_target_dependencies(my_cpp_node rclcpp ...)
# install(TARGETS my_cpp_node DESTINATION lib/${PROJECT_NAME})

# For pure Python packages, setup.py handles installation.
# For C++ packages or mixed, ensure Python parts are handled by setup.py and C++ by CMake.

# If this package primarily provides launch files and configurations (now centralized),
# much of the Python-specific installation might be minimal or handled via setup.py.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 