cmake_minimum_required(VERSION 3.5)
project(simple_marker_detector)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_cmake
find_package(ament_cmake REQUIRED)
# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
# ROS2 interfaces
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# OpenCV 4
find_package(OpenCV 4 REQUIRED)

# Include directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Executables
set(EXECUTABLE_0 simple_marker_detector)

# Executable 0
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)
target_link_libraries(${EXECUTABLE_0} ${OpenCV_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_0}
  rclcpp
  message_filters
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs)

# Install targets
install(TARGETS ${EXECUTABLE_0} DESTINATION lib/${PROJECT_NAME})
# Install directories
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
