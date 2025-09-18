// File: my_fusion/scripts/ekf_fusion_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <vector>

class EKFFusionNode : public rclcpp::Node
{
public:
    EKFFusionNode() : Node("ekf_fusion_node")
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

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&EKFFusionNode::odomCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, std::bind(&EKFFusionNode::imuCallback, this, std::placeholders::_1));

        // --- Publisher ---
        fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_topic_, 10);

        //last_time_ = this->now();
        first_odom_ = true;
    }

private:
    // --- ROS ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;

    // --- EKF State ---
    Eigen::Vector3d x_;        // [x, y, theta]
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;

    double imu_omega_ = 0.0;
    rclcpp::Time last_time_;
    bool first_odom_;
    double max_dt_ = 0.1;

    // --- Parameters ---
    std::string odom_topic_, imu_topic_, fused_topic_, frame_id_;

    // --- Callbacks ---
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_omega_ = msg->angular_velocity.z;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rclcpp::Time now = msg->header.stamp;

        if (first_odom_) {
	  last_time_ = now;
	  first_odom_ = false;
	  return;  // just initialise time, don’t run prediction yet
        }

        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > max_dt_) {
	  last_time_ = now;     // avoid runaway
	  return;
        }
        last_time_ = now;

        // --- Prediction ---
        double v = std::sqrt(std::pow(msg->twist.twist.linear.x,2) +
                             std::pow(msg->twist.twist.linear.y,2));
        double omega = imu_omega_;

        double theta = x_(2);
        x_(0) += v * dt * std::cos(theta);
        x_(1) += v * dt * std::sin(theta);
        x_(2) += omega * dt;

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,2) = -v * dt * std::sin(theta);
        F(1,2) = v * dt * std::cos(theta);

        P_ = F * P_ * F.transpose() + Q_;

        // --- Measurement Update ---
        Eigen::Vector3d z(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          tf2::getYaw(msg->pose.pose.orientation));

        Eigen::Matrix3d K = P_ * (P_ + R_).inverse();
        x_ = x_ + K * (z - x_);
        P_ = (Eigen::Matrix3d::Identity() - K) * P_;

        // --- Publish fused odom ---
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

        // --- Fill covariance (6x6) ---
        for(int i=0;i<6;i++)
            for(int j=0;j<6;j++)
                fused.pose.covariance[i*6 + j] = (i<3 && j<3) ? P_(i,j) : 0.0;

        fused_pub_->publish(fused);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

