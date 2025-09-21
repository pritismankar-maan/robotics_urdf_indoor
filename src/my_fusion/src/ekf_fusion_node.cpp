#include "my_fusion/ekf_fusion_node.hpp"

EKFFusionNode::EKFFusionNode() : Node("ekf_fusion_node")
{
  // --- Load parameters ---
  this->declare_parameter<std::string>("odom_topic", "/odom_est");
  this->declare_parameter<std::string>("imu_topic", "/imu/data");
  this->declare_parameter<std::string>("fused_topic", "/odom_fused");
  this->declare_parameter<std::string>("frame_id", "odom");

  this->declare_parameter<std::vector<double>>("Q", {0.001,0,0,0,0.001,0,0,0,0.001});
  this->declare_parameter<std::vector<double>>("R", {0.05,0,0,0,0.05,0,0,0,0.05});
  this->declare_parameter<std::vector<double>>("P_init", {0.01,0,0,0,0.01,0,0,0,0.01});
  this->declare_parameter<double>("max_dt", 0.1);

  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("fused_topic", fused_topic_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("max_dt", max_dt_);

  std::vector<double> Q_vec, R_vec, P_vec;
  this->get_parameter("Q", Q_vec);
  this->get_parameter("R", R_vec);
  this->get_parameter("P_init", P_vec);

  Q_ = Eigen::Map<Eigen::Matrix3d>(Q_vec.data());
  R_ = Eigen::Map<Eigen::Matrix3d>(R_vec.data());
  P_ = Eigen::Map<Eigen::Matrix3d>(P_vec.data());
  x_ = Eigen::Vector3d::Zero();

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&EKFFusionNode::odomCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10, std::bind(&EKFFusionNode::imuCallback, this, std::placeholders::_1));

  // Publishers
  fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_topic_, 10);
  fused_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fused_points", 10);
}

void EKFFusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // trusting only the yaw from IMU
  imu_omega_ = msg->angular_velocity.z;
}

void EKFFusionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time now = msg->header.stamp;

  if (first_odom_) {
    last_time_ = now;
    first_odom_ = false;
    return;
  }

  double dt = (now - last_time_).seconds();
  if (dt <= 0.0 || dt > max_dt_) {
    last_time_ = now;
    return;
  }
  last_time_ = now;

  // Prediction
  double v = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  double omega = imu_omega_;
  double theta = x_(2);

  x_(0) += v * dt * std::cos(theta);
  x_(1) += v * dt * std::sin(theta);
  x_(2) = normalizeYaw(x_(2) + omega * dt);

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0,2) = -v * dt * std::sin(theta);
  F(1,2) =  v * dt * std::cos(theta);
  P_ = F * P_ * F.transpose() + Q_;

  // Update
  Eigen::Vector3d z(msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    tf2::getYaw(msg->pose.pose.orientation));

  Eigen::Vector3d y;
  y(0) = z(0) - x_(0);
  y(1) = z(1) - x_(1);
  y(2) = normalizeYaw(z(2) - x_(2));

  Eigen::Matrix3d K = P_ * (P_ + R_).inverse();
  x_ += K * y;
  x_(2) = normalizeYaw(x_(2));
  P_ = (Eigen::Matrix3d::Identity() - K) * P_;

  // Publish fused odom
  nav_msgs::msg::Odometry fused;
  fused.header.stamp = now;
  fused.header.frame_id = frame_id_;
  fused.pose.pose.position.x = x_(0);
  fused.pose.pose.position.y = x_(1);
  tf2::Quaternion q;
  q.setRPY(0, 0, x_(2));
  fused.pose.pose.orientation = tf2::toMsg(q);
  fused.twist.twist.linear.x = v;
  fused.twist.twist.angular.z = omega;

  for (int i=0;i<6;i++)
    for (int j=0;j<6;j++)
      fused.pose.covariance[i*6+j] = (i<3 && j<3) ? P_(i,j) : 0.0;

  fused_pub_->publish(fused);

  // Publish marker
  visualization_msgs::msg::Marker mp;
  mp.header.stamp = now;
  mp.header.frame_id = frame_id_;
  mp.ns = "fused_points";
  mp.id = next_fused_point_id_++;
  mp.type = visualization_msgs::msg::Marker::POINTS;
  mp.action = visualization_msgs::msg::Marker::ADD;
  mp.scale.x = mp.scale.y = 0.05;
  mp.color.r = 1.0;
  mp.color.a = 1.0;
  geometry_msgs::msg::Point p;
  p.x = x_(0); p.y = x_(1); p.z = 0.0;
  mp.points.push_back(p);
  fused_point_pub_->publish(mp);
}

double EKFFusionNode::normalizeYaw(double yaw)
{
  while (yaw > M_PI) yaw -= 2.0*M_PI;
  while (yaw < -M_PI) yaw += 2.0*M_PI;
  return yaw;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFFusionNode>());
  rclcpp::shutdown();
  return 0;
}
