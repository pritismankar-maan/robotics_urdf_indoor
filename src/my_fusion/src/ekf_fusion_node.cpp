#include "my_fusion/ekf_fusion_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

using std::placeholders::_1;

namespace
{
inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}
} // namespace

EKFFusionNode::EKFFusionNode() : Node("ekf_fusion_node")
{
  // ---- Parameters ----
  this->declare_parameter<std::string>("odom_topic", "/odom_est");
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("lidar_topic", "/odom_lidar_fused");
  this->declare_parameter<std::string>("fused_topic", "/odom_fused");
  this->declare_parameter<std::string>("frame_id", "odom");
  this->declare_parameter<std::vector<double>>("Q", {0.001, 0, 0,
                                                     0, 0.001, 0,
                                                     0, 0, 0.002});
  this->declare_parameter<std::vector<double>>("R", {0.2, 0, 0,
                                                     0, 0.2, 0,
                                                     0, 0, 0.2});
  this->declare_parameter<std::vector<double>>("P_init", {0.01, 0, 0,
                                                          0, 0.01, 0,
                                                          0, 0, 0.01});
  this->declare_parameter<double>("max_dt", 0.1);

  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("fused_topic", fused_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("max_dt", max_dt_);

  std::vector<double> qv, rv, pv;
  this->get_parameter("Q", qv);
  this->get_parameter("R", rv);
  this->get_parameter("P_init", pv);

  Q_ = Eigen::Map<Eigen::Matrix3d>(qv.data());
  R_ = Eigen::Map<Eigen::Matrix3d>(rv.data());
  P_ = Eigen::Map<Eigen::Matrix3d>(pv.data());

  x_ = Eigen::Vector3d::Zero();

  // ---- Subs & pubs ----
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, std::bind(&EKFFusionNode::odomCallback, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50, std::bind(&EKFFusionNode::imuCallback, this, _1));
  lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_topic_, 20, std::bind(&EKFFusionNode::lidarOdomCallback, this, _1));

  fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(fused_topic_, 20);
  fused_point_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("fused_points", 1);
}

// ======================== Callbacks ==============================

void EKFFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_omega_ = msg->angular_velocity.z;
}

void EKFFusionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time now = msg->header.stamp;

  if (first_odom_)
  {
    last_enc_time_ = now;
    first_odom_ = false;
    return;
  }

  double dt = (now - last_enc_time_).seconds();
  if (dt <= 0.0 || dt > max_dt_)
  {
    last_enc_time_ = now;
    return;
  }
  last_enc_time_ = now;

  double v = std::hypot(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y);
  double theta = x_(2);
  double omega = imu_omega_;

  // --- Prediction step ---
  x_(0) += v * dt * std::cos(theta);
  x_(1) += v * dt * std::sin(theta);
  x_(2) = normalizeYaw(x_(2) + omega * dt);

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -v * dt * std::sin(theta);
  F(1, 2) = v * dt * std::cos(theta);

  P_ = F * P_ * F.transpose() + Q_;

  last_v_ = v;
  last_omega_ = omega;

  publishFused(now);
}

void EKFFusionNode::lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time now = msg->header.stamp;

  if (first_lidar_)
  {
    prev_lidar_x_ = msg->pose.pose.position.x;
    prev_lidar_y_ = msg->pose.pose.position.y;
    prev_lidar_yaw_ = yawFromQuat(msg->pose.pose.orientation);
    last_lidar_time_ = now;
    first_lidar_ = false;
    return;
  }

  double dt = (now - last_lidar_time_).seconds();
  if (dt <= 0.0 || dt > max_dt_)
  {
    last_lidar_time_ = now;
    return;
  }
  last_lidar_time_ = now;

  double lx = msg->pose.pose.position.x;
  double ly = msg->pose.pose.position.y;
  double lyaw = yawFromQuat(msg->pose.pose.orientation);

  Eigen::Vector3d z(lx, ly, lyaw);

  // Standard EKF measurement update
  lidarUpdate(z, now);
}

void EKFFusionNode::lidarUpdate(const Eigen::Vector3d &z,
                                const rclcpp::Time &stamp)
{
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  Eigen::Vector3d y = z - x_;
  y(2) = normalizeYaw(y(2)); // wrap yaw innovation
  Eigen::Matrix3d R_used = R_ * 100;
  Eigen::Matrix3d S = H * P_ * H.transpose() + R_used;
  Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();

  x_ = x_ + K * y;
  x_(2) = normalizeYaw(x_(2));

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  P_ = (I - K * H) * P_;

  publishFused(stamp);
}

// ======================== Helpers ==============================

void EKFFusionNode::publishFused(const rclcpp::Time &stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id_;
  odom.pose.pose.position.x = x_(0);
  odom.pose.pose.position.y = x_(1);

  tf2::Quaternion q;
  q.setRPY(0, 0, x_(2));
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = last_v_;
  odom.twist.twist.angular.z = last_omega_;

  // Fill covariance (only first 3x3 of pose.covariance)
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      odom.pose.covariance[i * 6 + j] = (i < 3 && j < 3) ? P_(i, j) : 0.0;

  fused_pub_->publish(odom);

  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id_;
  m.ns = "fused_path";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = m.scale.y = m.scale.z = 0.05;
  m.color.r = 1.0;
  m.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = x_(0);
  p.y = x_(1);
  p.z = 0.0;
  m.points.push_back(p);
  fused_point_pub_->publish(m);
}

double EKFFusionNode::yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double EKFFusionNode::normalizeYaw(double yaw)
{
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

// ======================== Main ==============================

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFFusionNode>());
  rclcpp::shutdown();
  return 0;
}

