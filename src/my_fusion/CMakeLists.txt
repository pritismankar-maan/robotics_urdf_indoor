cmake_minimum_required(VERSION 3.5)
project(my_fusion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(ekf_fusion_node
  src/ekf_fusion_node.cpp
)

ament_target_dependencies(ekf_fusion_node
  rclcpp nav_msgs sensor_msgs visualization_msgs tf2 tf2_geometry_msgs
)

install(TARGETS ekf_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
