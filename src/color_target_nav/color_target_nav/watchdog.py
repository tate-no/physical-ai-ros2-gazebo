"""Independent command watchdog for Gazebo Classic's latched velocity command."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from geometry_msgs.msg import Twist


class Watchdog(Node):
    def __init__(self):
        super().__init__('color_nav_watchdog')
        self.last = float('-inf')
        self.command = Twist()
        self.pub = self.create_publisher(Twist, '/burger_test/cmd_vel', 1)
        self.create_subscription(Twist, '/color_nav/desired_cmd', self.receive,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_timer(.05, self.tick)

    def receive(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.command = Twist()
            self.command.linear.x = max(-.04, min(.08, msg.linear.x))
            self.command.angular.z = max(-.35, min(.35, msg.angular.z))
            self.last = time.monotonic()

    def tick(self):
        self.pub.publish(self.command if time.monotonic() - self.last < .3 else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
