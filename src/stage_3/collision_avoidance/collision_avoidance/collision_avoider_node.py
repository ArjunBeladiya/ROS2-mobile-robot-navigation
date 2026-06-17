#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
from custom_interfaces.action import TravelNoCrashing
from custom_interfaces.msg import DangerZones

WHITE  = 0
GREEN  = 1
YELLOW = 2
RED    = 3

ZONE_NAMES = {WHITE: 'WHITE', GREEN: 'GREEN', YELLOW: 'YELLOW', RED: 'RED'}

SPEED_FULL = 0.2    
SPEED_SLOW = 0.08   
SPEED_AVOID = 0.03  

TURN_SPEED = 0.6

FEEDBACK_PERIOD_SEC = 3.0
CONTROL_PERIOD_SEC  = 0.1


class CollisionAvoiderNode(Node):
    def __init__(self):
        super().__init__('collision_avoider_node')

        self._cb_group = ReentrantCallbackGroup()

        self.current_zones = DangerZones()
        self.zones_received = False

        self.start_x = None
        self.start_y = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.have_odom = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.danger_sub = self.create_subscription(
            DangerZones, '/danger_zones', self.danger_callback, 10,
            callback_group=self._cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self._cb_group
        )

        self.action_server = ActionServer(
            self,
            TravelNoCrashing,
            '/drive_no_crashing',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info('Collision Avoider Node started, action /drive_no_crashing ready')

    def danger_callback(self, msg: DangerZones):
        self.current_zones = msg
        self.zones_received = True

    def odom_callback(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        self.have_odom = True

    def goal_callback(self, goal_request):
        if goal_request.target_distance <= 0.0:
            self.get_logger().warn('Rejecting goal: target_distance must be > 0')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_distance = goal_handle.request.target_distance
        self.get_logger().info(f'Starting drive: target_distance={target_distance:.2f} m')

        wait_start = time.time()
        while (not self.have_odom or not self.zones_received) and time.time() - wait_start < 5.0:
            time.sleep(0.05)

        self.start_x = self.last_x
        self.start_y = self.last_y
        start_time = self.get_clock().now()
        last_feedback_time = start_time

        most_dangerous_zone = 'WHITE'
        highest_danger_value = WHITE

        result = TravelNoCrashing.Result()
        feedback_msg = TravelNoCrashing.Feedback()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_robot()
                    result.traveled_distance = self.get_traveled_distance()
                    result.most_dangerous_zone = most_dangerous_zone
                    result.highest_danger_value = highest_danger_value
                    return result

                traveled = self.get_traveled_distance()
                remaining = target_distance - traveled

                if remaining <= 0.0:
                    break

                fl = self.current_zones.front_left
                fr = self.current_zones.front_right

                all_zones = [
                    fl, fr,
                    self.current_zones.back_left,
                    self.current_zones.back_right,
                ]
                run_worst = max(all_zones)
                if run_worst > highest_danger_value:
                    highest_danger_value = run_worst
                    most_dangerous_zone = self.zone_label(self.current_zones, run_worst)

                worst_front = max(fl, fr)
                linear_speed, angular_speed = self.compute_command(fl, fr, worst_front)

                if remaining < 0.1 and worst_front < RED:
                    linear_speed = min(linear_speed, SPEED_SLOW)

                self.publish_cmd(linear_speed, angular_speed)

                now = self.get_clock().now()
                if (now - last_feedback_time).nanoseconds >= FEEDBACK_PERIOD_SEC * 1e9:
                    elapsed = now - start_time
                    feedback_msg.traveled_distance = traveled
                    feedback_msg.time_traveled = Duration(
                        sec=int(elapsed.nanoseconds // 1_000_000_000),
                        nanosec=int(elapsed.nanoseconds % 1_000_000_000)
                    )
                    goal_handle.publish_feedback(feedback_msg)
                    last_feedback_time = now
                    self.get_logger().info(
                        f'Feedback: traveled={traveled:.2f}m, '
                        f'front_zone={ZONE_NAMES.get(worst_front)}, '
                        f'lin={linear_speed:.2f}, ang={angular_speed:.2f}'
                    )

                time.sleep(CONTROL_PERIOD_SEC)

        finally:
            self.stop_robot()

        traveled_final = self.get_traveled_distance()
        result.traveled_distance = traveled_final
        result.most_dangerous_zone = most_dangerous_zone
        result.highest_danger_value = int(highest_danger_value)

        goal_handle.succeed()
        self.get_logger().info(
            f'Goal reached: traveled={traveled_final:.2f}m, '
            f'worst_zone={most_dangerous_zone} ({highest_danger_value})'
        )
        return result

    def compute_command(self, fl: int, fr: int, worst_front: int):
        """
        Returns (linear_speed, angular_speed) based on front-left/front-right
        danger zones.

        WHITE/GREEN -> full speed, straight
        YELLOW      -> reduced speed, straight
        RED         -> crawl forward + turn away from the more dangerous side
        """
        if worst_front == RED:
            if fl > fr:
                angular = -TURN_SPEED  
            elif fr > fl:
                angular = TURN_SPEED    
            else:
                angular = TURN_SPEED    
            return SPEED_AVOID, angular

        if worst_front == YELLOW:
            return SPEED_SLOW, 0.0

        return SPEED_FULL, 0.0

    def zone_label(self, zones: DangerZones, value: int) -> str:
        if zones.front_left == value:
            return 'front_left'
        if zones.front_right == value:
            return 'front_right'
        if zones.back_left == value:
            return 'back_left'
        if zones.back_right == value:
            return 'back_right'
        return 'unknown'

    def get_traveled_distance(self) -> float:
        if self.start_x is None:
            return 0.0
        dx = self.last_x - self.start_x
        dy = self.last_y - self.start_y
        return (dx ** 2 + dy ** 2) ** 0.5

    def publish_cmd(self, linear_speed: float, angular_speed: float = 0.0):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoiderNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()