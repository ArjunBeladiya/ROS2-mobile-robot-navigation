#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


class TFFollower(Node):
    def __init__(self):
        super().__init__('tf_follower')

        self.target_frame = 'target_pose'
        self.map_frame = 'map'

        # Listens to TF in the background
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Connects to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Connected to Nav2. Following target_pose.')

        # Send a fresh goal often enough that the robot never fully stops
        # to wait for the next one
        self.timer = self.create_timer(0.3, self.follow_target)

    def follow_target(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.target_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not get target_pose: {e}')
            return

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = tf.transform.translation.x
        goal.pose.position.y = tf.transform.translation.y
        goal.pose.orientation = tf.transform.rotation

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        self.nav_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()