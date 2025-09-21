#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityDriver(Node):
    """
    Publish Twist messages to drive the robot along a rectangular path.
    Each segment = straight line for a few seconds, then turn.
    """

    def __init__(self):
        super().__init__("velocity_driver")
        self.pub = self.create_publisher(Twist, "/diff_cont/cmd_vel_unstamped", 10)

        # define a harcoded sequence of (linear_x, angular_z, duration)
        self.commands = [
            (0.5, 0.0, 5.2),    # forward - down
            (0.0, 0.8, 2.0),    # turn left
            (0.5, 0.0, 5.8),    # forward - to the left
            (0.0, 0.75, 1.8),    # turn left
            (0.5, 0.0, 11.0),    # forward - up
            (0.0, 0.75, 1.8),    # turn left
            (0.5, 0.0, 9.0),    # forward back to start - to the right
            (0.0, 0.8, 2.0),    # turn left
            (0.5, 0.0, 10.5),    # forward - down
            (0.0, 0.8, 2.0),    # turn left
            (0.5, 0.0, 4.0),    # forward - to the left
            (0.0, 0.75, 1.5),    # turn left
            (0.5, 0.0, 4.2),    # forward - up towards origin
            (0.0, 0.0, 0.0),    # stop
        ]
        self.index = 0
        self.elapsed = 0.0

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.loop)
        self.get_logger().info("VelocityDriver started")

    def loop(self):
        if self.index >= len(self.commands):
            return

        # extract vel command and duration
        lin, ang, dur = self.commands[self.index]
        
        # keep publishing certain velocity command in loop
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)

        self.elapsed += self.timer_period
        # if timer is more than 'dur', go to next command, reset the time
        if self.elapsed >= dur:
            self.index += 1
            self.elapsed = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = VelocityDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

