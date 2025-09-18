#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading

class LivePlot(Node):
    def __init__(self):
        super().__init__('live_plot')
        self.enc_x, self.enc_y = [], []
        self.imu_x, self.imu_y = [], []

        self.sub_enc = self.create_subscription(
            Odometry, '/odom_est', self.enc_cb, 10)
        self.sub_imu = self.create_subscription(
            Odometry, '/odom_imu_est', self.imu_cb, 10)

        # start a background thread for matplotlib
        self.plot_thread = threading.Thread(target=self.plot_loop, daemon=True)
        self.plot_thread.start()

    def enc_cb(self, msg):
        self.enc_x.append(msg.pose.pose.position.x)
        self.enc_y.append(msg.pose.pose.position.y)

    def imu_cb(self, msg):
        self.imu_x.append(msg.pose.pose.position.x)
        self.imu_y.append(msg.pose.pose.position.y)

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            ax.clear()
            ax.plot(self.enc_x, self.enc_y, 'g-', label='Encoders')
            ax.plot(self.imu_x, self.imu_y, 'b-', label='IMU')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.legend()
            plt.pause(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = LivePlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

