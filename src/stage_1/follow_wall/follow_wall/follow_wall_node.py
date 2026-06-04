#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.MOVE_FORWARD = 0
        self.TURN_LEFT = 1
        self.WALL_FOLLOW = 2
        self.STOPPED = 3

        self.state = self.MOVE_FORWARD

        self.get_logger().info("Node Started")

    def clean_range(self, value):
        if math.isinf(value) or math.isnan(value):
            return 10.0
        return value

    def scan_callback(self, msg):

        front_distance = self.clean_range(msg.ranges[0])

        right_distance = self.clean_range(msg.ranges[270])

        right_front_distance = self.clean_range(msg.ranges[315])

        vel_msg = Twist()

        if self.state == self.MOVE_FORWARD:

            if front_distance >= 0.5:

                vel_msg.linear.x = 0.15

                self.get_logger().info(
                    f"Moving Forward | Front={front_distance:.2f}"
                )

            else:

                self.get_logger().info(
                    "Wall reached -> Start turning left"
                )

                self.state = self.TURN_LEFT

        elif self.state == self.TURN_LEFT:

            vel_msg.angular.z = 0.3

            self.get_logger().info(
                f"Turning | Right={right_distance:.2f} "
                f"RightFront={right_front_distance:.2f}"
            )

            # Wall now on right side
            wall_on_right = right_distance < 0.6 and right_distance > 0.3

            # Parallel condition
            parallel_to_wall = right_front_distance < 0.7 and right_front_distance > 0.6

            if wall_on_right and parallel_to_wall:

                self.get_logger().info(
                    "90 degree turn completed"
                )

                vel_msg.angular.z = 0.0

                self.state = self.WALL_FOLLOW

        elif self.state == self.WALL_FOLLOW:

            desired_rf = 0.62

            error = desired_rf - right_front_distance

            kp = 1.5

            vel_msg.linear.x = 0.12

            vel_msg.angular.z = kp * error

            if vel_msg.angular.z > 0.5:
                vel_msg.angular.z = 0.5

            if vel_msg.angular.z < -0.5:
                vel_msg.angular.z = -0.5

            self.get_logger().info(
                f"Wall Follow | RF={right_front_distance:.2f} "
                f"Error={error:.2f}"
            )

            if front_distance <= 0.5:
                self.state = self.TURN_LEFT

        self.cmd_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()