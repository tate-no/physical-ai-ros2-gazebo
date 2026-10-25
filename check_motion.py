"""Read-only ROS diagnostic; run in a sourced Humble shell. Never publishes."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from color_target_nav.control import sector
from color_target_nav.avoidance import ScanGuard


def main():
    rclpy.init()
    node = Node('color_nav_read_only_check')
    latest = {}
    subscriptions = []
    for name, topic, message in (
        ('scan', '/scan', LaserScan),
        ('desired', '/color_nav/desired_cmd', Twist),
        ('drive', '/burger_test/cmd_vel', Twist),
        ('odom', '/burger_test/odom', Odometry),
        ('status', '/color_nav/status', String),
    ):
        subscriptions.append(node.create_subscription(
            message, topic, lambda msg, key=name: latest.update({key: msg}),
            qos_profile_sensor_data))
    try:
        end = time.monotonic() + 4.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.2)
        report = {'missing_topics': [key for key in
                  ('scan', 'desired', 'drive', 'odom', 'status') if key not in latest]}
        if 'status' in latest:
            report['mission'] = json.loads(latest['status'].data)
        if 'scan' in latest:
            msg = latest['scan']

            def measure(low, high):
                return sector(msg.ranges, msg.angle_min, msg.angle_increment,
                              low, high, msg.range_min, msg.range_max)

            report['clearance_metres'] = {
                'front': measure(-.35, .35),
                'left_side': measure(.35, 2.5),
                'right_side': measure(-2.5, -.35),
                'rear_left': measure(2.5, math.pi),
                'rear_right': measure(-math.pi, -2.5),
            }
            report['scan_frame'] = msg.header.frame_id
            guard = ScanGuard(msg.ranges, msg.angle_min, msg.angle_increment,
                              msg.range_min, msg.range_max)
            report['surroundings_m'] = guard.clearance
            # Works with the installed v3/v4 too: no update to the driving
            # controller is needed to run this read-only diagnostic.
            def blockers(predicate):
                points = sorted((p for p in guard.points if predicate(*p)),
                                key=lambda p: math.hypot(*p))[:6]
                return [{'x': round(x, 3), 'y': round(y, 3)} for x, y in points]
            report['path_checks'] = {
                'scan_valid': guard.valid, 'reverse_clear': guard.reverse_safe(),
                'turn_clear': guard.turn_safe(), 'forward_clear': guard.forward_safe(),
                'reverse_blockers': blockers(lambda x, y: -.36 <= x <= .10 and abs(y) <= .14),
                'turn_blockers': blockers(lambda x, y: math.hypot(x, y) <= .28),
                'forward_blockers': blockers(lambda x, y: -.14 <= x <= .38 and abs(y) <= .14),
            }
        for key in ('desired', 'drive'):
            if key in latest:
                msg = latest[key]
                report[key] = {'linear_m_s': msg.linear.x, 'angular_rad_s': msg.angular.z}
        if 'odom' in latest:
            msg = latest['odom']
            report['actual_velocity'] = {'linear_m_s': msg.twist.twist.linear.x,
                                         'angular_rad_s': msg.twist.twist.angular.z}
            report['position'] = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}
        print(json.dumps(report, indent=2, allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
