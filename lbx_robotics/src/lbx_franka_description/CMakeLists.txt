cmake_minimum_required(VERSION 3.8)
project(lbx_franka_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch files if any
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# Install config files if any
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# Install meshes if any
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 