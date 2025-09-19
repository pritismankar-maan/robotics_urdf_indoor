#include "my_odometry/odom_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

OdomNode::OdomNode()
: Node("odom_node"),
  wheel_radius_(0.05),
  wheel_separation_(0.30)
{
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&OdomNode::jointCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 50, std::bind(&OdomNode::imuCallback, this, _1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&OdomNode::lidarCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_est", 10);
    odom_imu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_imu_est", 10);
    odom_lidar_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_lidar_est", 10);
    //traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_segments", 10);
    point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("odom_points", 10);
    imu_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("imu_points", 10);
    lidar_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_points", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

// -------------------
// Helpers
// -------------------
inline double OdomNode::normalizeYaw(double yaw)
{
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

void OdomNode::updatePose(const Pose2D &new_pose, const rclcpp::Time &stamp)
{
    //Segment seg{current_pose_, new_pose};
    current_pose_ = new_pose;

    publishOdom(stamp);
    //publishSegment(seg, stamp);
    publishTF(stamp);

    //double dx = seg.end.x - seg.start.x;
    //double dy = seg.end.y - seg.start.y;
    //double dist = std::hypot(dx, dy);
    //double dyaw = seg.end.yaw - seg.start.yaw;
    //RCLCPP_DEBUG(this->get_logger(), "Δx=%.4f Δy=%.4f dist=%.4f Δyaw=%.4f",
    //             dx, dy, dist, dyaw);
}

// -------------------
// TF Publishers
// -------------------
void OdomNode::publishTF(const rclcpp::Time &stamp)
{
    // publish TF Needed for viz
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

void OdomNode::publishimuTF(const rclcpp::Time &stamp)
{
    // Fill and send transform 
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

void OdomNode::publishlidarTF(const rclcpp::Time &stamp)
{
    // Fill and send transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "lidar_link";
    t.transform.translation.x = lidar_pose_.x;
    t.transform.translation.y = lidar_pose_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, lidar_pose_.yaw);
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
}

// -------------------
// Joint State Callback
// -------------------
void OdomNode::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    auto it_l = std::find(msg->name.begin(), msg->name.end(), "left_wheel_joint");
    auto it_r = std::find(msg->name.begin(), msg->name.end(), "right_wheel_joint");
    if (it_l == msg->name.end() || it_r == msg->name.end()) 
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "JointState without drive wheels, skipping");
      return;
    }

    const size_t idx_l = std::distance(msg->name.begin(), it_l);
    const size_t idx_r = std::distance(msg->name.begin(), it_r);

    if (idx_l >= msg->position.size() || idx_r >= msg->position.size()) return;

    double left_pos = msg->position[idx_l];
    double right_pos = msg->position[idx_r];

    // sanity check: ignore bogus zero reset (both wheels exactly 0)
    if (std::fabs(left_pos) < 1e-9 && std::fabs(right_pos) < 1e-9) 
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "JointState Wheel Position at fake zero, skipping");
      return;
    }
    
    // For the first value
    rclcpp::Time now = msg->header.stamp;
    if (last_time_.nanoseconds() == 0) {
        last_left_pos_ = left_pos;
        last_right_pos_ = right_pos;
        last_time_ = now;
        return;
    }

    double dt = (now - last_time_).seconds();
    if (dt <= 0) return;

    // Get wheel travel/distance
    double d_left = (left_pos - last_left_pos_) * wheel_radius_;
    double d_right = (right_pos - last_right_pos_) * wheel_radius_;

    // if wheels didn't move, don’t publish
    if (std::fabs(d_left) < 1e-6 && std::fabs(d_right) < 1e-6) return;

    // get how much the base link's center has moved
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / wheel_separation_;

    // Calculate coordinate in world frame
    Pose2D new_pose;
    new_pose.x = current_pose_.x + d_center * std::cos(current_pose_.yaw + d_theta/2.0);
    new_pose.y = current_pose_.y + d_center * std::sin(current_pose_.yaw + d_theta/2.0);
    new_pose.yaw = normalizeYaw(current_pose_.yaw + d_theta);

    // Get Twist/velocities - Few sanity checks
    bool use_joint_velocity = false;
    double v_left = 0.0, v_right = 0.0;
    if (!msg->velocity.empty() && idx_l < msg->velocity.size() && idx_r < msg->velocity.size()) {
        v_left = msg->velocity[idx_l];
        v_right = msg->velocity[idx_r];
        // If not zero reset (bogus), proceed
        if (std::fabs(v_left) > 1e-9 || std::fabs(v_right) > 1e-9) use_joint_velocity = true;
    }

    // Calculate Twist velocity
    if (use_joint_velocity) {
        double v = (v_left + v_right) / 2.0 * wheel_radius_;
        double omega = (v_right - v_left) / wheel_separation_;
        new_pose.vx = v * std::cos(current_pose_.yaw);
        new_pose.vy = v * std::sin(current_pose_.yaw);
        new_pose.v_yaw = omega;
    } else {
        // else get velocity from Encoder's position data
        new_pose.vx = d_center/dt * std::cos(current_pose_.yaw + d_theta/2.0);
        new_pose.vy = d_center/dt * std::sin(current_pose_.yaw + d_theta/2.0);
        new_pose.v_yaw = d_theta / dt;
    }

    
    // Fill marker
    visualization_msgs::msg::Marker mp;
    mp.header.stamp = now;
    mp.header.frame_id = "odom";
    mp.ns = "enc_points";
    mp.id = next_marker_points_id_++;
    mp.type = visualization_msgs::msg::Marker::POINTS;
    mp.action = visualization_msgs::msg::Marker::ADD;
    mp.scale.x = 0.05;
    mp.scale.y = 0.05;
    mp.color.g = 1.0;
    mp.color.a = 1.0;

    geometry_msgs::msg::Point p;
    p.x = new_pose.x;
    p.y = new_pose.y;
    p.z = 0.0;
    mp.points.push_back(p);
    
    // publish point, Transform (needed for rviz) and Markers
    updatePose(new_pose, now);
    point_pub_->publish(mp);
    publishTF(now);

    // Update for next iteration
    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;
    last_time_ = now;
}

// -------------------
// IMU Callback
// -------------------
void OdomNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    rclcpp::Time stamp(msg->header.stamp);

    // For first timestamp
    if (last_imu_time_.nanoseconds() == 0) {
        last_imu_time_ = stamp;
        return;
    }

    // Get timegap
    double dt = (stamp - last_imu_time_).seconds();
    
    // Ignore if timegap between two IMUs is too large
    if (dt <= 0.0 || dt > 2.0) {
        RCLCPP_WARN(get_logger(),
                  "Skipping IMU integration: dt=%.3f s", dt);
        last_imu_time_ = stamp;
        return;
    }

    // Fill position's yaw
    double wz = msg->angular_velocity.z;
    imu_pose_.yaw = normalizeYaw(imu_pose_.yaw + wz*dt);

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;

    // Convert accel into World frame
    double c = std::cos(imu_pose_.yaw);
    double s = std::sin(imu_pose_.yaw);
    double ax_w = c*ax - s*ay;
    double ay_w = s*ax + c*ay;

    // Update twist and position data to be published later
    imu_vx_ += ax_w*dt;
    imu_vy_ += ay_w*dt;
    imu_pose_.x += imu_vx_*dt;
    imu_pose_.y += imu_vy_*dt;
    imu_pose_.v_yaw = wz;

    last_imu_time_ = stamp;

    // Fill marker
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

    // Publish Imu's Odom, Marker and corresponding TF for viz
    publishImuOdom(stamp);
    imu_point_pub_->publish(mp);
    publishimuTF(stamp);
}

// -------------------
// Lidar Callback
// -------------------
void OdomNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    rclcpp::Time stamp(msg->header.stamp);

    if (last_lidar_time_.nanoseconds() == 0) {
        last_lidar_time_ = stamp;
        return;
    }

    double dt = (stamp - last_lidar_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
        last_lidar_time_ = stamp;
        return;
    }

    // Convert ranges to points
    std::vector<Eigen::Vector2d> pts;
    const size_t step = 5;
    for (size_t i = 0; i < msg->ranges.size(); i += step) {
        float r = msg->ranges[i];
        if (!std::isfinite(r) || r < 0.05 || r > msg->range_max) continue;
        float ang = msg->angle_min + i*msg->angle_increment;
        pts.emplace_back(r*std::cos(ang), r*std::sin(ang));
    }

    if (prev_scan_.empty()) {
        prev_scan_ = pts;
        return;
    }

    // Nearest neighbor association
    std::vector<Eigen::Vector2d> src, dst;
    for (auto &p: pts) {
        double best_d = 0.4;
        int best_idx = -1;
        for (size_t j = 0; j < prev_scan_.size(); ++j) {
            double d = (p - prev_scan_[j]).squaredNorm();
            if (d < best_d*best_d) { best_d = std::sqrt(d); best_idx = j; }
        }
        if (best_idx >= 0) { src.push_back(prev_scan_[best_idx]); dst.push_back(p); }
    }

    if (src.size() < 3) { prev_scan_ = pts; return; }

    // SVD
    Eigen::Vector2d mu_src = Eigen::Vector2d::Zero();
    Eigen::Vector2d mu_dst = Eigen::Vector2d::Zero();
    for (size_t i=0;i<src.size();++i){ mu_src += src[i]; mu_dst += dst[i]; }
    mu_src /= src.size(); mu_dst /= dst.size();

    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (size_t i=0;i<src.size();++i){ W += (src[i]-mu_src)*(dst[i]-mu_dst).transpose(); }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R = svd.matrixV()*svd.matrixU().transpose();
    if (R.determinant()<0) R.col(1) *= -1;
    Eigen::Vector2d t = mu_dst - R*mu_src;

    double dtheta = std::atan2(R(1,0), R(0,0));
    lidar_pose_.yaw = normalizeYaw(lidar_pose_.yaw + dtheta);
    lidar_pose_.x += t.x()*std::cos(lidar_pose_.yaw) - t.y()*std::sin(lidar_pose_.yaw);
    lidar_pose_.y += t.x()*std::sin(lidar_pose_.yaw) + t.y()*std::cos(lidar_pose_.yaw);

    double c = std::cos(lidar_pose_.yaw - dtheta);
    double s = std::sin(lidar_pose_.yaw - dtheta);
    lidar_vx_ = c*t.x()/dt - s*t.y()/dt;
    lidar_vy_ = s*t.x()/dt + c*t.y()/dt;
    lidar_v_yaw_ = dtheta/dt;

    // Publish marker
    visualization_msgs::msg::Marker mp;
    mp.header.stamp = stamp;
    mp.header.frame_id = "odom";
    mp.ns = "lidar_points";
    mp.id = next_lidar_point_id_++;
    mp.type = visualization_msgs::msg::Marker::POINTS;
    mp.action = visualization_msgs::msg::Marker::ADD;
    mp.scale.x = 0.05;
    mp.scale.y = 0.05;
    mp.color.r = 1.0;
    mp.color.a = 1.0;

    geometry_msgs::msg::Point p;
    p.x = lidar_pose_.x;
    p.y = lidar_pose_.y;
    p.z = 0.0;
    mp.points.push_back(p);

    publishLidarOdom(stamp);
    lidar_point_pub_->publish(mp);
    publishlidarTF(stamp);

    prev_scan_ = pts;
}

// ---------------------------
// Encoder Odometry Publishers
// ---------------------------
void OdomNode::publishOdom(const rclcpp::Time &stamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    // Fill position data
    odom.pose.pose.position.x = current_pose_.x;
    odom.pose.pose.position.y = current_pose_.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Fill velocity related data
    odom.twist.twist.linear.x = current_pose_.vx;
    odom.twist.twist.linear.y = current_pose_.vy;
    odom.twist.twist.angular.z = current_pose_.v_yaw;

    // Hardcode covariance
    for (int i=0;i<36;i++){ odom.pose.covariance[i]=0.0; odom.twist.covariance[i]=0.0; }
    odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.01*0.01;
    odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[35] = 0.01*0.01;

    // publish Encoder related Odom
    odom_pub_->publish(odom);
}

void OdomNode::publishImuOdom(const rclcpp::Time &stamp)
{
    // Fill Odom structure
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "imu_link";
    
    // Fill position and Orientation
    odom.pose.pose.position.x = imu_pose_.x;
    odom.pose.pose.position.y = imu_pose_.y;
    tf2::Quaternion q;
    q.setRPY(0,0,imu_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Fill velocity related data
    odom.twist.twist.linear.x = imu_vx_;
    odom.twist.twist.linear.y = imu_vy_;
    odom.twist.twist.angular.z = imu_pose_.v_yaw;

    // Hardcode covariance (need to be extracted from actual sensors)
    for (int i=0;i<36;i++)
    { 
      odom.pose.covariance[i]=0.0; 
      odom.twist.covariance[i]=0.0; 
    }
    odom.pose.covariance[0]=odom.pose.covariance[7]=0.02*0.02; 
    odom.pose.covariance[35]=0.05*0.05;
    odom.twist.covariance[0]=odom.twist.covariance[7]=0.02*0.02;
    odom.twist.covariance[35]=0.05*0.05;   

    // Publish IMU Odom
    odom_imu_pub_->publish(odom);
}

void OdomNode::publishLidarOdom(const rclcpp::Time &stamp)
{
    // Fill Odom structure
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "lidar_link";
    
    // Fill position and Orientation
    odom.pose.pose.position.x = lidar_pose_.x;
    odom.pose.pose.position.y = lidar_pose_.y;
    tf2::Quaternion q;
    q.setRPY(0,0,lidar_pose_.yaw);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Fill velocity related data
    odom.twist.twist.linear.x = lidar_vx_;
    odom.twist.twist.linear.y = lidar_vy_;
    odom.twist.twist.angular.z = lidar_v_yaw_;

    // Hardcode covariance (need to be extracted from actual sensors)
    for (int i=0;i<36;i++)
    { 
      odom.pose.covariance[i]=0.0; 
      odom.twist.covariance[i]=0.0; 
    }
    
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

// -------------------
// Visualization Segment
// -------------------
//void OdomNode::publishSegment(const Segment &seg, const rclcpp::Time &stamp)
//{
//    visualization_msgs::msg::Marker marker;
//    marker.header.stamp = stamp;
//    marker.header.frame_id = "odom";
//    marker.ns = "odom_segments";
//    marker.id = next_marker_id_++;
//    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//    marker.action = visualization_msgs::msg::Marker::ADD;
//    marker.scale.x = 0.02;
//    marker.color.r = 1.0;
//    marker.color.a = 1.0;

//    geometry_msgs::msg::Point p1, p2;
//    p1.x = seg.start.x; p1.y = seg.start.y; p1.z = 0;
//    p2.x = seg.end.x;   p2.y = seg.end.y;   p2.z = 0;
//    marker.points.push_back(p1);
//    marker.points.push_back(p2);

//    traj_pub_->publish(marker);
//}

// -------------------
// Main
// -------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

