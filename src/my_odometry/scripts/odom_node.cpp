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

#include "sensor_msgs/msg/laser_scan.hpp"
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Pose2D
{
  double x;
  double y;
  double yaw;
  // velocities
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
      
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&OdomNode::lidarCallback, this, _1));
    lidar_point_pub_ = create_publisher<visualization_msgs::msg::Marker>("lidar_points", 10);


    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_est", 10);
    odom_imu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_imu_est", 10);
    odom_lidar_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_lidar_est", 10);
    traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_segments", 10);
    point_pub_ = create_publisher<visualization_msgs::msg::Marker>("odom_points", 10);
    imu_point_pub_ = create_publisher<visualization_msgs::msg::Marker>("imu_points", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
  
  // need a first stamp
    rclcpp::Time stamp(msg->header.stamp);

  // first measurement
  if (last_lidar_time_.nanoseconds() == 0) {
    last_lidar_time_ = stamp;
    return;
  }

  double dt = (stamp - last_lidar_time_).seconds();
  if (dt <= 0.0) return;
  
  // Skip if dt is too large (sensor pause, old bag message, etc.)
  if (dt > 0.5) {   // adjust to your rate
      RCLCPP_WARN(get_logger(),
                  "Skipping Lidar SVD: dt=%.3f s", dt);
      last_lidar_time_ = stamp;
      return;
  }  
  
  // --- 1) Convert to XY and downsample ---
  std::vector<Eigen::Vector2d> pts;
  const size_t step = 5;   // take every 5th beam
  for (size_t i=0; i<msg->ranges.size(); i+=step) {
    float r = msg->ranges[i];
    if (!std::isfinite(r) || r<0.05 || r>msg->range_max) continue;
    float ang = msg->angle_min + i*msg->angle_increment;
    pts.emplace_back(r*std::cos(ang), r*std::sin(ang));
  }
  if (prev_scan_.empty()) { prev_scan_ = pts; return; }

  // --- 2) Associate: nearest neighbour ---
  std::vector<Eigen::Vector2d> src, dst;
  for (auto &p : pts) 
  {
    double best_d = 0.4;   // max assoc distance [m]
    int best_idx = -1;
    
    for (size_t j=0;j<prev_scan_.size();++j)
    {  
      double d = (p - prev_scan_[j]).squaredNorm();
      if (d < best_d*best_d)
      { 
        best_d = std::sqrt(d); 
        best_idx = j; 
      }
    }
    
    if (best_idx>=0)
    { 
      src.push_back(prev_scan_[best_idx]); 
      dst.push_back(p); 
    }
  }
  
  if (src.size()<3) 
  { 
    prev_scan_ = pts; 
    return; 
  }

  // --- 3) SVD solve for rotation + translation ---
  Eigen::Vector2d mu_src= Eigen::Vector2d::Zero(); 
  Eigen::Vector2d mu_dst=Eigen::Vector2d::Zero();
  
  for (size_t i=0;i<src.size();++i)
  { 
    mu_src+=src[i]; 
    mu_dst+=dst[i]; 
  }
  
  mu_src/=src.size(); 
  mu_dst/=dst.size();
  
  Eigen::Matrix2d W=Eigen::Matrix2d::Zero();
  
  for (size_t i=0;i<src.size();++i)
  {
    W += (src[i]-mu_src)*(dst[i]-mu_dst).transpose();
  }
  
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Eigen::Matrix2d R = svd.matrixV()*svd.matrixU().transpose();
  
  if (R.determinant()<0) R.col(1) *= -1;
  Eigen::Vector2d t = mu_dst - R*mu_src;

  // --- 4) Update pose ---
  double dtheta = std::atan2(R(1,0), R(0,0));
  lidar_pose_.yaw += dtheta;
  lidar_pose_.yaw = normalizeYaw(lidar_pose_.yaw);
  
  lidar_pose_.x   += t.x()*std::cos(lidar_pose_.yaw)
                     - t.y()*std::sin(lidar_pose_.yaw);
  lidar_pose_.y   += t.x()*std::sin(lidar_pose_.yaw)
                     + t.y()*std::cos(lidar_pose_.yaw);

  double c = std::cos(lidar_pose_.yaw - dtheta);
  double s = std::sin(lidar_pose_.yaw - dtheta);
  lidar_vx_ = c*t.x()/dt - s*t.y()/dt;
  lidar_vy_ = s*t.x()/dt + c*t.y()/dt;
  lidar_v_yaw_ = dtheta / dt;
  
  visualization_msgs::msg::Marker mp;
mp.header.stamp = stamp;
mp.header.frame_id = "odom";
mp.ns = "lidar_points";
mp.id = next_lidar_point_id_++;
mp.type = visualization_msgs::msg::Marker::POINTS;
mp.action = visualization_msgs::msg::Marker::ADD;
mp.scale.x = 0.05;  // point diameter
mp.scale.y = 0.05;
mp.color.r = 1.0;   // red points
mp.color.a = 1.0;

geometry_msgs::msg::Point p;
p.x = lidar_pose_.x;
p.y = lidar_pose_.y;
p.z = 0.0;

lidar_point_pub_->publish(mp);
  publishLidarOdom(msg->header.stamp);
  mp.points.push_back(p);
  publishlidarTF(msg->header.stamp);

  prev_scan_ = pts;
}
  
  void publishlidarTF(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "lidar_link";
    t.transform.translation.x = lidar_pose_.x;
    t.transform.translation.y = lidar_pose_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, imu_pose_.yaw);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
  }
  
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
      new_pose.yaw = normalizeYaw(new_pose.yaw);

// --- compute velocities ---
      bool use_joint_velocity = false;
      double v_left = 0.0, v_right = 0.0;
      if (!msg->velocity.empty() && idx_l < msg->velocity.size() && idx_r < msg->velocity.size()) {
        v_left = msg->velocity[idx_l];
        v_right = msg->velocity[idx_r];
        if (std::fabs(v_left) > 1e-9 || std::fabs(v_right) > 1e-9) {
          use_joint_velocity = true;
        }  
      }

      if (use_joint_velocity) {
        double v = (v_left + v_right) / 2.0 * wheel_radius_;
        double omega = (v_right - v_left) / wheel_separation_;
        new_pose.vx = v * std::cos(current_pose_.yaw);
        new_pose.vy = v * std::sin(current_pose_.yaw);
        new_pose.v_yaw = omega;
      } else {
        new_pose.vx = d_center / dt * std::cos(current_pose_.yaw + d_theta/2.0);
        new_pose.vy = d_center / dt * std::sin(current_pose_.yaw + d_theta/2.0);
        new_pose.v_yaw = d_theta / dt;
      }      
      
      updatePose(new_pose, now);

      last_left_pos_  = left_pos;
      last_right_pos_ = right_pos;
      last_time_      = now;
    }


  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // need a first stamp
    rclcpp::Time stamp(msg->header.stamp);

  // first measurement
  if (last_imu_time_.nanoseconds() == 0) {
    last_imu_time_ = stamp;
    return;
  }

  double dt = (stamp - last_imu_time_).seconds();
  if (dt <= 0.0) return;
  
  // Skip if dt is too large (sensor pause, old bag message, etc.)
  if (dt > 2) {   // 50 ms for a 20 Hz sensor, adjust to your rate
      RCLCPP_WARN(get_logger(),
                  "Skipping IMU integration: dt=%.3f s", dt);
      last_imu_time_ = stamp;
      return;
  }

  // integrate yaw
  double wz = msg->angular_velocity.z;
  imu_pose_.yaw += wz * dt;
  imu_pose_.yaw = normalizeYaw(imu_pose_.yaw);

  // body accel -> world
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double c = std::cos(imu_pose_.yaw);
  double s = std::sin(imu_pose_.yaw);
  double ax_w =  c*ax - s*ay;
  double ay_w =  s*ax + c*ay;

  imu_vx_ += ax_w * dt;
  imu_vy_ += ay_w * dt;
  imu_pose_.x += imu_vx_ * dt;
  imu_pose_.y += imu_vy_ * dt;
  imu_pose_.v_yaw = wz;  // angular velocity from IMU


  last_imu_time_ = stamp;

  // publish as POINTS
  visualization_msgs::msg::Marker mp;
  mp.header.stamp = stamp;
  mp.header.frame_id = "odom";
  mp.ns = "imu_points";
  mp.id = next_imu_point_id_++;
  mp.type = visualization_msgs::msg::Marker::POINTS;
  mp.action = visualization_msgs::msg::Marker::ADD;
  mp.scale.x = 0.05;
  mp.scale.y = 0.05;
  mp.color.b = 1.0;
  mp.color.a = 1.0;

  geometry_msgs::msg::Point p;
  p.x = imu_pose_.x;
  p.y = imu_pose_.y;
  p.z = 0.0;
  mp.points.push_back(p);

  publishImuOdom(stamp); 	
  imu_point_pub_->publish(mp);
  publishimuTF(stamp);
  
  }

  void publishImuOdom(const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "imu_link";
    odom.pose.pose.position.x = imu_pose_.x;
    odom.pose.pose.position.y = imu_pose_.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, imu_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);
    
     // --- twist (from integrated velocity) ---
    odom.twist.twist.linear.x = imu_vx_;
    odom.twist.twist.linear.y = imu_vy_;
    odom.twist.twist.angular.z = imu_pose_.v_yaw;  // we need to track integrated yaw rate

    // --- pose covariance ---
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.02*0.02; // x
    odom.pose.covariance[7]  = 0.02*0.02; // y
    odom.pose.covariance[35] = 0.05*0.05; // yaw (z rotation)

    // --- twist covariance ---
    for (int i = 0; i < 36; i++) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0]  = 0.02*0.02; // vx
    odom.twist.covariance[7]  = 0.02*0.02; // vy
    odom.twist.covariance[35] = 0.05*0.05; // v_yaw

    odom_imu_pub_->publish(odom);
  }
  
  void publishLidarOdom(const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "lidar_link";
    odom.pose.pose.position.x = lidar_pose_.x;
    odom.pose.pose.position.y = lidar_pose_.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, lidar_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);
    
    // --- twist (from integrated velocity) ---
    odom.twist.twist.linear.x = lidar_vx_;
    odom.twist.twist.linear.y = lidar_vy_;
    odom.twist.twist.angular.z = lidar_v_yaw_;  // we need to track integrated yaw rate

    // --- pose covariance ---
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = 0.05*0.05; // x
    odom.pose.covariance[7]  = 0.05*0.05; // y
    odom.pose.covariance[35] = 0.1*0.1; // yaw (z rotation)

    // --- twist covariance ---
    for (int i = 0; i < 36; i++) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0]  = 0.05*0.05; // vx
    odom.twist.covariance[7]  = 0.05*0.05; // vy
    odom.twist.covariance[35] = 0.1*0.1; // v_yaw

    odom_lidar_pub_->publish(odom);
  }
  
  void publishimuTF(const rclcpp::Time &stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "imu_link";
    t.transform.translation.x = imu_pose_.x;
    t.transform.translation.y = imu_pose_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, imu_pose_.yaw);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
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
    
    // twist
    odom.twist.twist.linear.x = current_pose_.vx;
    odom.twist.twist.linear.y = current_pose_.vy;
    odom.twist.twist.angular.z = current_pose_.v_yaw;

    // pose covariance (example small numbers)
    for (int i=0;i<36;i++) odom.pose.covariance[i]=0.0;
    odom.pose.covariance[0] = 0.01*0.01;
    odom.pose.covariance[7] = 0.01*0.01;
    odom.pose.covariance[35]= 0.01*0.01;

    for (int i=0;i<36;i++) odom.twist.covariance[i]=0.0;
    odom.twist.covariance[0] = 0.01*0.01;
    odom.twist.covariance[7] = 0.01*0.01;
    odom.twist.covariance[35]= 0.01*0.01;

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

  inline double normalizeYaw(double yaw)
  {
    while (yaw > M_PI) 
    {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) 
    {
      yaw += 2.0 * M_PI;
    }
    return yaw;
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_point_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_lidar_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lidar_point_pub_;
  std::vector<Eigen::Vector2d> prev_scan_;
  Pose2D lidar_pose_{0.0, 0.0, 0.0};
  rclcpp::Time last_lidar_time_{0,0,RCL_ROS_TIME};
  double lidar_vx_{0.0};
  double lidar_vy_{0.0};
  double lidar_v_yaw_{0.0};
  int next_lidar_point_id_{0};


  double wheel_radius_;
  double wheel_separation_;

  Pose2D current_pose_;
  double last_left_pos_{0.0};
  double last_right_pos_{0.0};
  rclcpp::Time last_time_;
  int next_marker_id_;
  int next_marker_points_id_;

  Pose2D imu_pose_{0.0, 0.0, 0.0};
  double imu_vx_{0.0};
  double imu_vy_{0.0};
  rclcpp::Time last_imu_time_{0,0,RCL_ROS_TIME};
  int next_imu_point_id_{0};

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}
