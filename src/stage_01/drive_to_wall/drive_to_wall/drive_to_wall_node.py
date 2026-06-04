#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DriveToWallNode(Node):
    def __init__(self):
        super().__init__('drive_to_wall_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Drive to Wall Node started')

    def scan_callback(self, msg):
        self.get_logger().info(f'Scan received with {len(msg.ranges)} readings')

        front_distance = msg.ranges[0]
        self.get_logger().info(f'Front distance: {front_distance:.2f} m')
       
        vel_msg = Twist()

        if front_distance < 1.0:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            self.get_logger().info('stopped! wall detected at < 1m')

        else:
            vel_msg.linear.x = 0.5
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            self.get_logger().info('moving forward: linear.x = 0.5')
        
        self.publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()