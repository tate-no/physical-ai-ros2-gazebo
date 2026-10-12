import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from tf2_ros import TransformException


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.finished = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_callback(self, msg):
        if self.finished:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, Time()
            )
        except TransformException:
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        points = []
        clusters = []
        current = []
        threshold = 0.15

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not (
                msg.range_min <= distance <= msg.range_max
            ):
                if current:
                    clusters.append(current)
                    current = []
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            point = (i, x, y)

            if current:
                _, px, py = current[-1]
                if math.hypot(x - px, y - py) > threshold:
                    clusters.append(current)
                    current = []

            current.append(point)
            points.append(point)

        if current:
            clusters.append(current)

        # 0度と360度の境界にまたがる物体を結合
        if len(clusters) > 1:
            first = clusters[0]
            last = clusters[-1]
            _, fx, fy = first[0]
            _, lx, ly = last[-1]
            if (first[0][0] == 0
                    and last[-1][0] == len(msg.ranges) - 1
                    and math.hypot(fx - lx, fy - ly) <= threshold):
                clusters[0] = last + first
                clusters.pop()

        print(f'有効な測定点数：{len(points)}')
        print(f'検出グループ数：{len(clusters)}')

        for number, cluster in enumerate(clusters, 1):
            x = sum(p[1] for p in cluster) / len(cluster)
            y = sum(p[2] for p in cluster) / len(cluster)
            distance = math.hypot(x, y)
            angle = math.degrees(math.atan2(y, x))

            world_x = tx + x * math.cos(yaw) - y * math.sin(yaw)
            world_y = ty + x * math.sin(yaw) + y * math.cos(yaw)

            print(
                f'障害物{number}: 測定点={len(cluster)} '
                f'距離={distance:.3f}m '
                f'角度={angle:.2f}度 '
                f'相対座標=({x:.3f}, {y:.3f}) '
                f'ワールド座標=({world_x:.3f}, {world_y:.3f})'
            )

        self.finished = True


def main():
    rclpy.init()
    node = ObjectDetector()

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
