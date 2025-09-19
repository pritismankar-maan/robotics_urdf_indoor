#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double v_yaw{0.0};
};

struct Segment
{
  Pose2D start;
  Pose2D end;
};

class OdomNode : public rclcpp::Node
{
public:
  OdomNode();

private:
  // ---- Callbacks ----
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ---- Publishers / helpers ----
  void publishOdom(const rclcpp::Time &stamp);
  void publishImuOdom(const rclcpp::Time &stamp);
  void publishLidarOdom(const rclcpp::Time &stamp);

  void publishTF(const rclcpp::Time &stamp);
  void publishimuTF(const rclcpp::Time &stamp);
  void publishlidarTF(const rclcpp::Time &stamp);

  void updatePose(const Pose2D &new_pose, const rclcpp::Time &stamp);
  double normalizeYaw(double yaw);

  // ---- Members ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_lidar_pub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lidar_point_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  Pose2D current_pose_;
  Pose2D imu_pose_;
  Pose2D lidar_pose_;

  double wheel_radius_{0.05};
  double wheel_separation_{0.30};

  double last_left_pos_{0.0};
  double last_right_pos_{0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  int next_marker_points_id_{0};

  double imu_vx_{0.0};
  double imu_vy_{0.0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  int next_imu_point_id_{0};

  std::vector<Eigen::Vector2d> prev_scan_;
  rclcpp::Time last_lidar_time_{0, 0, RCL_ROS_TIME};
  double lidar_vx_{0.0};
  double lidar_vy_{0.0};
  double lidar_v_yaw_{0.0};
  int next_lidar_point_id_{0};
};
