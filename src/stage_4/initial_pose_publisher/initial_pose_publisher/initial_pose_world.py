import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_world")
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info('Waiting for AMCL subscriber on /initialpose...')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.publisher_.publish(msg)
        self.get_logger().info('Initial pose published.')
        self.published = True
        self.timer.cancel()
        # Give the message time to propagate before shutting down
        self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()