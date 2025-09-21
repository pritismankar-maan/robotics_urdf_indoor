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

struct Pose2D {
    double x=0, y=0, yaw=0;
    double vx=0, vy=0, v_yaw=0;
};

class LidarOdomNode : public rclcpp::Node {
public:
    LidarOdomNode()
        : Node("lidar_odometry_node"),
          wheel_radius_(0.05),
          wheel_separation_(0.30)
    {
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&LidarOdomNode::jointCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 50, std::bind(&LidarOdomNode::imuCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarOdomNode::lidarCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_lidar_fused", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    Pose2D lidar_pose_;
    Pose2D last_encoder_pose_;
    double wheel_radius_;
    double wheel_separation_;
    rclcpp::Time last_lidar_time_;
    rclcpp::Time last_encoder_time_;
    std::vector<Eigen::Vector2d> prev_scan_;
    
    // -------------------
    // Utilities
    // -------------------
    inline double normalizeYaw(double yaw) {
        while (yaw > M_PI) yaw -= 2.0*M_PI;
        while (yaw < -M_PI) yaw += 2.0*M_PI;
        return yaw;
    }

    void publishOdom(const rclcpp::Time &stamp, const Pose2D &pose, double cov_factor=1.0) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0,0,pose.yaw);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = pose.vx;
        odom.twist.twist.linear.y = pose.vy;
        odom.twist.twist.angular.z = pose.v_yaw;

        // simple covariance scaling
        for (int i=0;i<36;i++){ odom.pose.covariance[i]=0; odom.twist.covariance[i]=0; }
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.05*cov_factor;
        odom.pose.covariance[35] = 0.1*cov_factor;
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.05*cov_factor;
        odom.twist.covariance[35] = 0.1*cov_factor;

        odom_pub_->publish(odom);

        // TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = pose.x;
        t.transform.translation.y = pose.y;
        t.transform.translation.z = 0;
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    }

    // -------------------
    // Callbacks
    // -------------------
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
	// Convert ROS message time to rclcpp::Time
	rclcpp::Time current_time(msg->header.stamp);

	if (last_encoder_time_.nanoseconds() == 0) {
	last_encoder_time_ = current_time;
	if (!msg->position.empty()) {
	    last_encoder_pose_.x = 0.0;
	    last_encoder_pose_.y = 0.0;
	    last_encoder_pose_.yaw = 0.0;
	}
	return;
	}

	double dt = (current_time - last_encoder_time_).seconds();
	if (dt <= 0) return;

	double left = msg->position[0]*wheel_radius_;
	double right = msg->position[1]*wheel_radius_;
	double d_center = (left+right)/2.0;
	double d_theta = (right-left)/wheel_separation_;

	last_encoder_pose_.x += d_center*std::cos(last_encoder_pose_.yaw + d_theta/2.0);
	last_encoder_pose_.y += d_center*std::sin(last_encoder_pose_.yaw + d_theta/2.0);
	last_encoder_pose_.yaw = normalizeYaw(last_encoder_pose_.yaw + d_theta);

	last_encoder_time_ = current_time;
    }


    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Could optionally be used for angular correction
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	    rclcpp::Time stamp(msg->header.stamp);
	    if (last_lidar_time_.nanoseconds() == 0) {
		last_lidar_time_ = stamp;
		prev_scan_.clear();
		for (size_t i=0;i<msg->ranges.size();i+=2){  // denser sampling
		    double r = msg->ranges[i];
		    if (!std::isfinite(r)) continue;
		    double ang = msg->angle_min + i*msg->angle_increment;
		    prev_scan_.push_back(Eigen::Vector2d(r*std::cos(ang), r*std::sin(ang)));
		}
		return;
	    }

	    double dt = (stamp - last_lidar_time_).seconds();
	    if (dt <= 0 || dt > 0.5) return;

	    // Convert current scan
	    std::vector<Eigen::Vector2d> curr_scan;
	    for (size_t i=0;i<msg->ranges.size();i+=2){
		double r = msg->ranges[i];
		if (!std::isfinite(r)) continue;
		double ang = msg->angle_min + i*msg->angle_increment;
		curr_scan.emplace_back(r*std::cos(ang), r*std::sin(ang));
	    }
	    if (curr_scan.empty() || prev_scan_.empty()) return;

	    // Tighter ICP association
	    std::vector<Eigen::Vector2d> src, dst;
	    double max_assoc_dist = 0.2; // tighter threshold
	    for (size_t i=0;i<curr_scan.size();i++){
		double min_d = max_assoc_dist;
		int best_idx = -1;
		for (size_t j=0;j<prev_scan_.size();j++){
		    double d = (curr_scan[i]-prev_scan_[j]).squaredNorm();
		    if (d<min_d*min_d) {
		        min_d = std::sqrt(d);
		        best_idx = j;
		    }
		}
		if (best_idx>=0) {
		    src.push_back(prev_scan_[best_idx]);
		    dst.push_back(curr_scan[i]);
		}
	    }

	    if (src.size()<5) return;  // need more points

	    // Compute SVD transform
	    Eigen::Vector2d mu_src = Eigen::Vector2d::Zero(), mu_dst = Eigen::Vector2d::Zero();
	    for (size_t i=0;i<src.size();i++){ mu_src+=src[i]; mu_dst+=dst[i]; }
	    mu_src/=src.size(); mu_dst/=dst.size();

	    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
	    for (size_t i=0;i<src.size();i++){ W += (src[i]-mu_src)*(dst[i]-mu_dst).transpose(); }

	    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	    Eigen::Matrix2d R = svd.matrixV()*svd.matrixU().transpose();
	    if (R.determinant()<0) R.col(1)*=-1;

	    Eigen::Vector2d t = mu_dst - R*mu_src;
	    // Flip scan motion if lidar X is reversed
	    t = -t;
	    double dtheta = std::atan2(R(1,0), R(0,0));

	    // Update pose
	    //lidar_pose_.yaw = normalizeYaw(lidar_pose_.yaw + dtheta);
	    //Eigen::Vector2d trans_rotated;
	    //trans_rotated.x() = t.x()*std::cos(lidar_pose_.yaw) - t.y()*std::sin(lidar_pose_.yaw);
	    //trans_rotated.y() = t.x()*std::sin(lidar_pose_.yaw) + t.y()*std::cos(lidar_pose_.yaw);
	    //lidar_pose_.x += trans_rotated.x();
	    //lidar_pose_.y += trans_rotated.y();
	
	double new_yaw = normalizeYaw(lidar_pose_.yaw + dtheta);
	double yaw_before = lidar_pose_.yaw;     // save current heading
	lidar_pose_.yaw = new_yaw;

	Eigen::Vector2d trans_rotated;
	trans_rotated.x() = t.x()*cos(yaw_before) - t.y()*sin(yaw_before);
	trans_rotated.y() = t.x()*sin(yaw_before) + t.y()*cos(yaw_before);

	lidar_pose_.x += trans_rotated.x();
	lidar_pose_.y += trans_rotated.y();
	
	RCLCPP_INFO(this->get_logger(),
    		"ICP result: dtheta=%.3f  t=(%.3f, %.3f)",
    					dtheta, t.x(), t.y());



	    // Covariance scaling for slip
	    double wheel_move = std::hypot(last_encoder_pose_.vx, last_encoder_pose_.vy);
	    double cov_factor = (wheel_move < 1e-3) ? 5.0 : 1.0;

	    publishOdom(stamp, lidar_pose_, cov_factor);

	    prev_scan_ = curr_scan;
	    last_lidar_time_ = stamp;
    } 

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

