import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

TARGET = sys.argv[1].upper() if len(sys.argv) > 1 else 'RED'
if TARGET not in ('RED', 'BLUE', 'YELLOW'):
    raise SystemExit('RED / BLUE / YELLOW を指定してください')

RANGES = {
    'RED': [((0, 80, 40), (10, 255, 255)),
            ((170, 80, 40), (179, 255, 255))],
    'BLUE': [((100, 80, 40), (130, 255, 255))],
    'YELLOW': [((20, 80, 40), (40, 255, 255))]
}

class ColorControl(Node):
    def __init__(self):
        super().__init__('burger_color_final')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Twist, '/burger_test/cmd_vel', 10)
        self.create_subscription(
            Image, '/burger_camera/color_camera/image_raw',
            self.callback, 10)
        self.count = 0
        self.finished = False
        self.next_action = time.monotonic() + 1.0
        print(f'目標色: {TARGET} / 最大10回', flush=True)

    def stop(self):
        if not rclpy.ok():
            return
        for _ in range(20):
            if not rclpy.ok():
                break
            try:
                self.pub.publish(Twist())
            except Exception:
                break
            time.sleep(0.05)

    def pulse(self, speed):
        cmd = Twist()
        cmd.angular.z = speed
        try:
            self.pub.publish(cmd)
            time.sleep(0.1)
        finally:
            self.stop()

    def callback(self, msg):
        if self.finished or time.monotonic() < self.next_action:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for low, high in RANGES[TARGET]:
            mask |= cv2.inRange(
                hsv, np.array(low), np.array(high))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print('目標色が見えません。停止', flush=True)
            self.finished = True
            self.stop()
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) <= 30:
            print('目標色が見えません。停止', flush=True)
            self.finished = True
            self.stop()
            return

        x, y, w, h = cv2.boundingRect(largest)
        center = x + w // 2
        error = center - frame.shape[1] // 2

        print(f'{TARGET}: x={center}, ずれ={error:+d}px, '
              f'回転回数={self.count}', flush=True)

        if abs(error) <= 10:
            print('目標色が中央です。完了！', flush=True)
            self.finished = True
            self.stop()
            return

        if self.count >= 10:
            print('最大10回に到達。停止', flush=True)
            self.finished = True
            self.stop()
            return

        speed = -0.2 if error > 0 else 0.2
        self.count += 1
        self.next_action = time.monotonic() + 2.0
        print(f'短時間回転 {self.count}/10: {speed}',
              flush=True)
        self.pulse(speed)

def main():
    rclpy.init()
    node = ColorControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
