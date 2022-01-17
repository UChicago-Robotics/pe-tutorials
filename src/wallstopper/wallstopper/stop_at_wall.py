#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallStopper(Node):
    def __init__(self):
        super().__init__('stop_at_wall')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)


    def scan_callback(self, data):
        distance = data.ranges[0]
        if (distance > data.range_max):
            distance = data.range_max
        if (distance < data.range_min):
            distance = data.range_min
        
        ideal_dist = 0.3
        error = 0.5 * (distance - ideal_dist)

        vel = Twist()
        vel.linear.x = error

        self.publisher_.publish(vel)

        self.get_logger().info(f"Received scan: {data.ranges[0]}")


def main(args=None):
    rclpy.init(args=args)
    wallstopper = WallStopper()
    rclpy.spin(wallstopper)

    wallstopper.destroy_node() # Optional
    rclpy.shutdown()

if __name__ == "__main__":
    main()
