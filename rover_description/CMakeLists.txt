cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories with proper structure
install(
  DIRECTORY
    urdf
    config  
    launch
    scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable scripts
install(PROGRAMS
    scripts/wait_for_ros2_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install meshes directory only if it exists and is not empty
file(GLOB MESH_FILES "meshes/*")
if(MESH_FILES)
  install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
