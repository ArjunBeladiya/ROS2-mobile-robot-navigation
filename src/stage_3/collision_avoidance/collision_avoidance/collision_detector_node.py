#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interfaces.msg import DangerZones
from custom_interfaces.srv import SetZoneSize

DEFAULT_CRITICAL = 0.5
DEFAULT_WARN     = 0.75
DEFAULT_SAFE     = 1.0

WHITE  = 0
GREEN  = 1
YELLOW = 2
RED    = 3

def level_name(level: int) -> str:
    return {RED: "RED", YELLOW: "YELLOW", GREEN: "GREEN", WHITE: "WHITE"}.get(level, "UNKNOWN")


class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detector_node')

        self.red_threshold    = DEFAULT_CRITICAL
        self.yellow_threshold = DEFAULT_WARN
        self.green_threshold  = DEFAULT_SAFE

        self.publisher_ = self.create_publisher(DangerZones, '/danger_zones', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.zone_srv = self.create_service(
            SetZoneSize, '/set_zone_size', self.set_zone_size_callback
        )

        self.get_logger().info('Collision Detector Node started')
        self.get_logger().info(
            f'Initial zones — critical={self.red_threshold}m  '
            f'warn={self.yellow_threshold}m  safe={self.green_threshold}m'
        )

    def set_zone_size_callback(self, request, response):
        critical = request.critical
        warn     = request.warn
        safe     = request.safe

        if not (0 < critical < warn < safe):
            response.success = False
            response.message = (
                f'Invalid zone sizes: critical={critical}, warn={warn}, safe={safe}. '
                'Requirement: 0 < critical < warn < safe.'
            )
            self.get_logger().warn(response.message)
            return response

        self.red_threshold    = critical
        self.yellow_threshold = warn
        self.green_threshold  = safe

        response.success = True
        response.message = (
            f'Zone sizes updated — critical={critical}m  '
            f'warn={warn}m  safe={safe}m'
        )
        self.get_logger().info(response.message)
        return response

    def classify(self, distance: float) -> int:
        if distance <= self.red_threshold:    return RED
        if distance <= self.yellow_threshold: return YELLOW
        if distance <= self.green_threshold:  return GREEN
        return WHITE

    def get_avg_range(self, scan: LaserScan, start: int, end: int) -> float:
        valid = [
            r for r in scan.ranges[start:end]
            if not math.isnan(r) and not math.isinf(r)
            and scan.range_min <= r <= scan.range_max
        ]
        return sum(valid) / len(valid) if valid else scan.range_max

    def scan_callback(self, scan: LaserScan):
        total   = len(scan.ranges)
        quarter = total // 4

        front_left_dist  = self.get_avg_range(scan, 0,           quarter)
        back_left_dist   = self.get_avg_range(scan, quarter,     quarter * 2)
        back_right_dist  = self.get_avg_range(scan, quarter * 2, quarter * 3)
        front_right_dist = self.get_avg_range(scan, quarter * 3, total)

        msg = DangerZones()
        msg.front_right = self.classify(front_right_dist)
        msg.front_left  = self.classify(front_left_dist)
        msg.back_left   = self.classify(back_left_dist)
        msg.back_right  = self.classify(back_right_dist)

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'FR={level_name(msg.front_right)} ({front_right_dist:.2f}m) | '
            f'FL={level_name(msg.front_left)}  ({front_left_dist:.2f}m) | '
            f'BR={level_name(msg.back_right)}  ({back_right_dist:.2f}m) | '
            f'BL={level_name(msg.back_left)}   ({back_left_dist:.2f}m)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()