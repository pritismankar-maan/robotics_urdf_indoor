#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <vector>

class EKFFusionNode : public rclcpp::Node
{
public:
  EKFFusionNode();

private:
  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fused_point_pub_;

  // Parameters
  std::string odom_topic_, imu_topic_, fused_topic_, frame_id_;
  double max_dt_ = 0.1;

  // EKF state
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_, Q_, R_;

  double imu_omega_ = 0.0;
  rclcpp::Time last_time_;
  bool first_odom_ = true;
  int next_fused_point_id_ = 0;

  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  inline double normalizeYaw(double yaw);
};
