import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FrontDistance(Node):
    def __init__(self):
        super().__init__('front_distance')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.finished = False

    def scan_callback(self, msg):
        distances = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= math.radians(15):
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    distances.append(distance)

        if distances:
            print(f'正面の最短距離：{min(distances):.3f} m')
        else:
            print('正面に有効な距離データがありません')

        self.finished = True


def main():
    rclpy.init()
    node = FrontDistance()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
