#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "matplotlibcpp.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>

namespace plt = matplotlibcpp;

class LivePlot : public rclcpp::Node {
public:
    LivePlot() : Node("live_plot"), running_(true) {
        // ROS2 subscriptions
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu_points", 10, std::bind(&LivePlot::imu_cb, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_points", 10, std::bind(&LivePlot::odom_cb, this, std::placeholders::_1));

        // Matplotlib interactive mode
        plt::backend("Qt5Agg");
        plt::ion();
        plt::figure_size(800, 600);
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::title("Live Odometry + IMU Plot");

        // Start plotting thread
        plot_thread_ = std::thread([this]() { this->plot_loop(); });
    }

    ~LivePlot() {
        running_ = false;
        if (plot_thread_.joinable()) {
            plot_thread_.join();
        }
    }

private:
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_x_.push_back(msg->linear_acceleration.x);
        imu_y_.push_back(msg->linear_acceleration.y);

        // Keep vectors from growing indefinitely (optional)
        if (imu_x_.size() > max_points_) {
            imu_x_.erase(imu_x_.begin());
            imu_y_.erase(imu_y_.begin());
        }
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_x_.push_back(msg->pose.pose.position.x);
        odom_y_.push_back(msg->pose.pose.position.y);

        if (odom_x_.size() > max_points_) {
            odom_x_.erase(odom_x_.begin());
            odom_y_.erase(odom_y_.begin());
        }
    }

    void plot_loop() {
        while (rclcpp::ok() && running_) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                plt::clf();
                if (!imu_x_.empty()) plt::plot(imu_x_, imu_y_, "r-");
                if (!odom_x_.empty()) plt::plot(odom_x_, odom_y_, "b-");
                plt::legend();
                plt::draw();
                plt::pause(0.05);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ROS2 subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Data storage
    std::vector<double> imu_x_, imu_y_;
    std::vector<double> odom_x_, odom_y_;
    std::mutex mutex_;
    const size_t max_points_ = 1000;  // limit stored points

    // Plotting thread
    std::thread plot_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivePlot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
