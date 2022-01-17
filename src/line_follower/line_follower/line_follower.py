#!/usr/bin/env python3
import rclpy
import cv_bridge
import cv2
import numpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = cv_bridge.CvBridge()

        cv2.namedWindow("window", 1)

        self.image_sub = self.create_subscription(Image, 
                '/camera/image_raw', self.image_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.twist = Twist()

        self.last_time = None

    def image_callback(self, msg):
        curr_time = self.get_clock().now()
        delta = curr_time if self.last_time is None else (curr_time - self.last_time)

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([ 10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # +-----------------------------------------------------------------+
            # |                         Start here!                             |
            # +-----------------------------------------------------------------+
            err = w/2 - cx
            k_p = 1.0 / 100.0
            k_i = 1.0 / 100.0 # unused
            k_d = 1.0 / 100.0 # unused
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = k_p * err
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("window", image)
        cv2.waitKey(3)

        self.last_time = curr_time

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

