#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odom_node"),
    wheel_radius_(0.05),     // from your URDF/SDF
    wheel_separation_(0.30),
    x_(0.0), y_(0.0), yaw_(0.0),
    last_time_(0,0,RCL_ROS_TIME)
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&OdomNode::jointCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&OdomNode::imuCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_est", 10);
    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_trajectory", 10);

    // Set up Marker for RViz
    traj_marker_.header.frame_id = "odom";
    traj_marker_.ns = "trajectory";
    traj_marker_.id = 0;
    traj_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    traj_marker_.action = visualization_msgs::msg::Marker::ADD;
    traj_marker_.scale.x = 0.02;
    traj_marker_.color.r = 1.0;
    traj_marker_.color.g = 0.0;
    traj_marker_.color.b = 0.0;
    traj_marker_.color.a = 1.0;
  }

private:
  // ---- Callbacks ----
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.size() < 2 || msg->position.size() < 2) return;

    double left_pos = msg->position[0];
    double right_pos = msg->position[1];

    rclcpp::Time now = msg->header.stamp;
    if (last_time_.nanoseconds() == 0) {
      last_left_pos_ = left_pos;
      last_right_pos_ = right_pos;
      last_time_ = now;
      return;
    }

    double dt = (now - last_time_).seconds();
    if (dt <= 0) return;

    double d_left = (left_pos - last_left_pos_) * wheel_radius_;
    double d_right = (right_pos - last_right_pos_) * wheel_radius_;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / wheel_separation_;

    yaw_ += d_theta;
    x_ += d_center * std::cos(yaw_);
    y_ += d_center * std::sin(yaw_);

    publishOdom(now);
    publishTrajectory(now);

    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;
    last_time_ = now;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Optional: use imu yaw for fusion later
    (void)msg;
  }

  // ---- Publishers ----
  void publishOdom(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0,0,yaw_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom);
  }

  void publishTrajectory(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::Point p;
    p.x = x_;
    p.y = y_;
    p.z = 0.0;
    traj_marker_.header.stamp = stamp;
    traj_marker_.points.push_back(p);
    traj_pub_->publish(traj_marker_);
  }

  // ---- Members ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;

  visualization_msgs::msg::Marker traj_marker_;

  double wheel_radius_;
  double wheel_separation_;

  double x_, y_, yaw_;
  double last_left_pos_{0.0}, last_right_pos_{0.0};
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
