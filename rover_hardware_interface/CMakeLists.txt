cmake_minimum_required(VERSION 3.8)
project(rover_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Add the source file for the hardware interface plugin
add_library(rover_hardware_interface SHARED
  src/rover_hardware_interface.cpp
)

# Set the include directories
target_include_directories(rover_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against required libraries
ament_target_dependencies(rover_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# This is important! It registers the hardware interface as a plugin
# that ros2_control can find and load.
pluginlib_export_plugin_description_file(hardware_interface src/rover_hardware_interface.xml)

install(TARGETS
  rover_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  "src/rover_hardware_interface.xml"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
