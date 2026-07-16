import random
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.timer = self.create_timer(10,self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = random.uniform(-2.0, 2.0)
        goal.pose.position.y = random.uniform(-2.0, 2.0)
        goal.pose.position.z = 0.0

        yaw = random.uniform(-math.pi, math.pi)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher_.publish(goal)
        self.get_logger().info(
            f'Published goal: x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}, yaw={math.degrees(yaw):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()