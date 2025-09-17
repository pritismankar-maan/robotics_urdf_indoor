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
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

struct Segment
{
  Pose2D start;
  Pose2D end;
};

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odom_node"),
    wheel_radius_(0.05),      // m
    wheel_separation_(0.30),  // m
    current_pose_{0.0, 0.0, 0.0},
    last_time_(0, 0, RCL_ROS_TIME),
    next_marker_id_(0),
    next_marker_points_id_(0)
  {
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&OdomNode::jointCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&OdomNode::imuCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_est", 10);
    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_segments", 10);
    point_pub_ = create_publisher<visualization_msgs::msg::Marker>("odom_points", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
	    // --- find indexes of the drive wheels in this message ---
      auto it_l = std::find(msg->name.begin(), msg->name.end(), "left_wheel_joint");
      auto it_r = std::find(msg->name.begin(), msg->name.end(), "right_wheel_joint");
      if (it_l == msg->name.end() || it_r == msg->name.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "JointState without drive wheels, skipping");
        return;
      }

      const size_t idx_l = std::distance(msg->name.begin(), it_l);
      const size_t idx_r = std::distance(msg->name.begin(), it_r);
      if (idx_l >= msg->position.size() || idx_r >= msg->position.size()) return;

      double left_pos = msg->position[idx_l];
      double right_pos = msg->position[idx_r];

      // --- sanity check: ignore bogus zero reset (both wheels exactly 0) ---
      if (std::fabs(left_pos) < 1e-9 && std::fabs(right_pos) < 1e-9) {
        RCLCPP_DEBUG(get_logger(), "Skipping 0/0 wheel state");
        return;
      }

      rclcpp::Time now = msg->header.stamp;
      if (last_time_.nanoseconds() == 0) {
        last_left_pos_ = left_pos;
        last_right_pos_ = right_pos;
        last_time_ = now;
        return;
      }

      double dt = (now - last_time_).seconds();
      if (dt <= 0) return;

      // --- compute wheel travel ---
      double d_left = (left_pos - last_left_pos_) * wheel_radius_;
      double d_right = (right_pos - last_right_pos_) * wheel_radius_;

      // if wheels didn't move, don’t publish a new segment
      if (std::fabs(d_left) < 1e-6 && std::fabs(d_right) < 1e-6) return;

      double d_center = (d_left + d_right) / 2.0;
      double d_theta = (d_right - d_left) / wheel_separation_;

      Pose2D new_pose;
      new_pose.x   = current_pose_.x + d_center * std::cos(current_pose_.yaw + d_theta / 2.0);
      new_pose.y   = current_pose_.y + d_center * std::sin(current_pose_.yaw + d_theta / 2.0);
      new_pose.yaw = current_pose_.yaw + d_theta;

      updatePose(new_pose, now);

      last_left_pos_  = left_pos;
      last_right_pos_ = right_pos;
      last_time_      = now;
    }


  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
  {
    // placeholder for future IMU fusion
  }

  void updatePose(const Pose2D &new_pose, const rclcpp::Time &stamp)
  {
    Segment seg{current_pose_, new_pose};
    current_pose_ = new_pose;

    publishOdom(stamp);
    publishSegment(seg, stamp);
    publishTF(stamp);

    double dx = seg.end.x - seg.start.x;
    double dy = seg.end.y - seg.start.y;
    double dist = std::hypot(dx, dy);
    double dyaw = seg.end.yaw - seg.start.yaw;
    RCLCPP_DEBUG(this->get_logger(), "Δx=%.4f Δy=%.4f dist=%.4f Δyaw=%.4f",
                 dx, dy, dist, dyaw);
  }

  void publishOdom(const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = current_pose_.x;
    odom.pose.pose.position.y = current_pose_.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom);
  }

  void publishSegment(const Segment &seg, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "odom";
    m.ns = "trajectory_segments";
    m.id = next_marker_id_++;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.r = 1.0;
    m.color.a = 1.0;

    geometry_msgs::msg::Point p1;
    p1.x = seg.start.x;
    p1.y = seg.start.y;
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = seg.end.x;
    p2.y = seg.end.y;
    p2.z = 0.0;

    m.points.push_back(p1);
    m.points.push_back(p2);

    traj_pub_->publish(m);
    
    visualization_msgs::msg::Marker m_point;
    m_point.header.stamp = stamp;
    m_point.header.frame_id = "odom";
    m_point.ns = "trajectory_points";
    m_point.id = next_marker_points_id_++;
    m_point.type = visualization_msgs::msg::Marker::POINTS;
    m_point.action = visualization_msgs::msg::Marker::ADD;
    m_point.scale.x = 0.05;  // point diameter
    m_point.scale.y = 0.05;
    m_point.color.g = 1.0;   // green points
    m_point.color.a = 1.0;

    geometry_msgs::msg::Point p;
    p.x = seg.end.x;  // add the new_pose as a point
    p.y = seg.end.y;
    p.z = 0.0;
    m_point.points.push_back(p);

    point_pub_->publish(m_point);
    
  }

  void publishTF(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = current_pose_.x;
    t.transform.translation.y = current_pose_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.yaw);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
  }

  // --- members ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double wheel_radius_;
  double wheel_separation_;

  Pose2D current_pose_;
  double last_left_pos_{0.0};
  double last_right_pos_{0.0};
  rclcpp::Time last_time_;
  int next_marker_id_;
  int next_marker_points_id_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
