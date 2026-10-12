import math
import time
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


BOX_POSITIONS = {'red': (3.0, 0.0), 'blue': (3.0, 0.8), 'yellow': (3.0, -0.8)}

class TurnToBox(Node):
    def __init__(self, color):
        super().__init__('turn_to_box')
        self.color = color
        self.target_x, self.target_y = BOX_POSITIONS[color]
        self.turn_speed = 0.15
        self.angle_tolerance = math.radians(2.0)
        self.max_turn_seconds = 16.0
        self.turn_start_time = None
        self.turn_finished = False
        self.last_odom_time = None
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)
        self.exit_timer = self.create_timer(0.1, self.exit_callback)

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

    def watchdog_callback(self):
        if self.turn_finished:
            return
        if self.turn_start_time is None:
            return
        now = time.monotonic()
        if now - self.turn_start_time >= self.max_turn_seconds:
            self.turn_finished = True
            self.stop_robot()
            print('16秒経過：停止指令を送信')
            return
        if self.last_odom_time is None or now - self.last_odom_time > 1.0:
            self.turn_finished = True
            self.stop_robot()
            print('位置情報が途切れました：停止指令を送信')

    def exit_callback(self):
        if self.turn_finished:
            for _ in range(3):
                self.stop_robot()
                time.sleep(0.05)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

    def odom_callback(self, msg):
        if self.turn_finished:
            return
        self.last_odom_time = time.monotonic()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        print(f'現在位置：X={x:.3f}, Y={y:.3f}')
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        target = math.atan2(self.target_y-y, self.target_x-x)
        print(f'現在の向き：{math.degrees(yaw):.2f}度')
        print(f'{self.color}の箱の方向：{math.degrees(target):.2f}度')
        error = math.atan2(math.sin(target-yaw), math.cos(target-yaw))
        print(f'あと旋回する角度：{math.degrees(error):.2f}度')
        if self.turn_start_time is None:
            self.turn_start_time = time.monotonic()

        elapsed = time.monotonic() - self.turn_start_time
        if elapsed >= self.max_turn_seconds:
            self.turn_finished = True
            print('時間制限に達しました：旋回終了')

        if abs(error) <= self.angle_tolerance:
            self.turn_finished = True
            print('目標角度に到達しました')

        if self.turn_finished:
            self.stop_robot()
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = max(
            -self.turn_speed,
            min(self.turn_speed, 0.8 * error)
        )
        self.publisher.publish(cmd)


def main():
    color = sys.argv[1].lower() if len(sys.argv) > 1 else "blue"
    print(f"指定された箱の色：{color}")
    if color not in BOX_POSITIONS:
        print("エラー：red、blue、yellow のいずれかを指定してください")
        return
    rclpy.init()
    node = TurnToBox(color)
    try:
        while rclpy.ok() and not node.turn_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.stop_robot()
                time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
