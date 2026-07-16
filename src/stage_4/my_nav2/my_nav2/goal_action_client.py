import random
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class GoalActionClient(Node):
    def __init__(self):
        super().__init__('goal_action_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = random.uniform(-2.0, 2.0)
        goal_msg.pose.pose.position.y = random.uniform(-2.0, 2.0)

        yaw = random.uniform(-math.pi, math.pi)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Sending goal...')
        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {fb.distance_remaining:.2f} m')

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GoalActionClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()