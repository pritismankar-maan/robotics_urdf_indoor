cmake_minimum_required(VERSION 3.5)
project(my_odometry)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ------------------------
# Find ROS2 packages
# ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ------------------------
# Find Python3 for matplotlib-cpp
# ------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# ------------------------
# Include directories
# ------------------------
include_directories(
  include
  ${Python3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../matplotlib-cpp
)

# ------------------------
# Odometry node
# ------------------------
add_executable(odom_node scripts/odom_node.cpp)
ament_target_dependencies(odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# ------------------------
# Live plot node (header-only matplotlib-cpp)
# ------------------------
add_executable(live_plot scripts/live_plot.cpp)
ament_target_dependencies(live_plot
  rclcpp
  sensor_msgs
  nav_msgs
)
# Only include directories; do NOT link libraries to avoid plain/keyword conflict
target_include_directories(live_plot PRIVATE
  ${Python3_INCLUDE_DIRS}
)

# Link Python library to resolve undefined Py_* symbols
target_link_libraries(live_plot ${Python3_LIBRARIES})

# ------------------------
# Install targets
# ------------------------
install(TARGETS
  odom_node
  live_plot
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# Install launch folder
# ------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
