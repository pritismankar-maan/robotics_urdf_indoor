#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Dense>

class EKFFusionNode : public rclcpp::Node
{
public:
    EKFFusionNode();

private:
    // ----- Callbacks -----
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lidarUpdate(const Eigen::Vector3d &z, const rclcpp::Time &stamp);

    // ----- Helper functions -----
    void publishFused(const rclcpp::Time &stamp);
    double yawFromQuat(const geometry_msgs::msg::Quaternion &q);
    double normalizeYaw(double yaw);

    // ----- ROS interfaces -----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fused_point_pub_;

    // ----- Parameters -----
    std::string odom_topic_;
    std::string imu_topic_;
    std::string lidar_topic_;
    std::string fused_topic_;
    std::string frame_id_;
    double max_dt_;

    // ----- EKF state -----
    Eigen::Vector3d x_;      // [x, y, yaw]
    Eigen::Matrix3d P_;      // covariance
    Eigen::Matrix3d Q_;      // process noise
    Eigen::Matrix3d R_;      // measurement noise

    // ----- Last measurements -----
    double last_v_{0.0};
    double last_omega_{0.0};
    double imu_omega_{0.0};
    rclcpp::Time last_enc_time_;
    rclcpp::Time last_lidar_time_;
    bool first_odom_{true};
    bool first_lidar_{true};

    // ----- Previous lidar pose -----
    double prev_lidar_x_{0.0};
    double prev_lidar_y_{0.0};
    double prev_lidar_yaw_{0.0};
};

