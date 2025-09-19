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
find_package(Eigen3 REQUIRED)

# ------------------------
# Include directories
# ------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ------------------------
# Odometry node
# ------------------------
add_executable(odom_node src/odom_node.cpp)
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
# Install targets
# ------------------------
install(TARGETS
  odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# Install launch folder
# ------------------------
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
