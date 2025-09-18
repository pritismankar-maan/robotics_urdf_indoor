cmake_minimum_required(VERSION 3.5)
project(my_fusion)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories (if any headers in include/)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable (C++ node in scripts/ekf_fusion_node.cpp)
add_executable(ekf_fusion_node scripts/ekf_fusion_node.cpp)

# Link dependencies
ament_target_dependencies(ekf_fusion_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# Install the executable
install(TARGETS ekf_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other resources if needed
install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
