cmake_minimum_required(VERSION 3.8)
project(vips_driver)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- Find Asio Manually ---
find_path(ASIO_INCLUDE_DIR NAMES asio.hpp
    HINTS /usr/include /usr/local/include
    DOC "Path to Asio header files")

if(NOT ASIO_INCLUDE_DIR)
    message(FATAL_ERROR "Asio header files (asio.hpp) not found. Please install libasio-dev.")
else()
    message(STATUS "Found Asio headers at: ${ASIO_INCLUDE_DIR}")
endif()
# --- End Find Asio ---

# Add the executable for the VIPS driver node
add_executable(vips_node src/vips_driver_node.cpp)

# Add the include directories
target_include_directories(vips_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ASIO_INCLUDE_DIR}
)

# Link against the required ROS 2 libraries
ament_target_dependencies(vips_node
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
)

# Install the executable to the package's lib directory
install(TARGETS
  vips_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the package.xml file
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Run default linters
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  ament_add_gtest(test_vips_parsing test/test_vips_parsing.cpp)
  target_include_directories(test_vips_parsing PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ASIO_INCLUDE_DIR}
  )
  ament_target_dependencies(test_vips_parsing
    "rclcpp"
    "sensor_msgs"
    "nav_msgs"
    "tf2"
    "tf2_ros"
  )
endif()

# Mark the package for ament indexing
ament_package()
