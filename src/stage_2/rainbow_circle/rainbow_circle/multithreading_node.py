#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

class RainbowCircle(Node):
    def __init__(self):
        super().__init__('rainbow_circle')
        self.publishers_= self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
            )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
            )
        
        self.get_logger().info('circle publisher started')

        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.client_group = MutuallyExclusiveCallbackGroup()


        self.set_pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen',
            callback_group=self.client_group
        )

        self.main_timer = self.create_timer(
            0.2,
            self.loop,
            callback_group=self.timer_group
        )


        self.rainbow_colors = [
        #   ( r , g, b)
            (255, 0, 0),      # red
            (255, 127, 0),    # orange
            (255, 255, 0),    # yellow
            (0, 255, 0),      # green
            (0, 0, 255),      # blue
            (75, 0, 130),     # indigo
            (148, 0, 211),    # violet
        ]

        self.color_index = 0


    def timer_callback(self):
        msg = Twist()

        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publishers_.publish(msg)

    def loop(self):
        r, g, b = self.rainbow_colors[self.color_index]

        req = SetPen.Request()
        req.r = r        
        req.g = g
        req.b = b
        req.width = 4
        req.off = 0

        self.get_logger().info(f'Setting pen color: ({r}, {g}, {b})')

        future = self.set_pen_client.call_async(req)  

        self.color_index = (self.color_index + 1) % len(self.rainbow_colors)


def main(args=None):
    rclpy.init(args=args)
    node = RainbowCircle()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
