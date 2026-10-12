import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.start_time = time.monotonic()
        self.last_scan_time = time.monotonic()
        self.watchdog = self.create_timer(0.1, self.check_scan_timeout)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

    def check_scan_timeout(self):
        if time.monotonic() - self.last_scan_time > 1.0:
            self.publisher.publish(Twist())
            print('LiDAR通信途絶：停止指令を送信')

    def scan_callback(self, msg):
        self.last_scan_time = time.monotonic()
        distances = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(math.degrees(angle)) <= 10:
                if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                    distances.append(distance)

        if not distances:
            print('正面の距離：不明')
            self.publisher.publish(Twist())
            return

        nearest = min(distances)
        print(f'正面の最短距離：{nearest:.2f} m')

        if time.monotonic() - self.start_time >= 5.0:
            self.publisher.publish(Twist())
            print('5秒経過：停止指令を送信')
            return

        if nearest <= 0.5:
            print('判定：停止が必要')
            stop = Twist()
            self.publisher.publish(stop)
            print('停止指令を送信しました')
        else:
            print('判定：停止不要')
            move = Twist()
            move.linear.x = 0.05
            self.publisher.publish(move)
            print('低速前進指令を送信しました')


def main():
    rclpy.init()
    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
