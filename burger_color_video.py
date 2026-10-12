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

RANGES = {
    'RED': [((0, 80, 40), (10, 255, 255)),
            ((170, 80, 40), (179, 255, 255))],
    'BLUE': [((100, 80, 40), (130, 255, 255))],
    'YELLOW': [((20, 80, 40), (40, 255, 255))]
}

if TARGET not in RANGES:
    raise SystemExit('RED / BLUE / YELLOW を指定してください')


class ColorControl(Node):
    def __init__(self):
        super().__init__('burger_color_video')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Twist, '/burger_test/cmd_vel', 10)
        self.create_subscription(
            Image,
            '/burger_camera/color_camera/image_raw',
            self.callback, 10)
        self.start_time = time.monotonic()
        self.last_image = self.start_time
        self.finished = False
        self.timer = self.create_timer(0.1, self.safety_check)
        print(f'動画用自動旋回開始：{TARGET}', flush=True)

    def stop(self):
        self.pub.publish(Twist())

    def safety_check(self):
        if self.finished:
            return
        now = time.monotonic()
        if now - self.start_time > 30:
            print('30秒経過：安全停止', flush=True)
            self.finished = True
            self.stop()
        elif now - self.last_image > 2:
            print('カメラ画像が途絶えました：安全停止', flush=True)
            self.finished = True
            self.stop()

    def callback(self, msg):
        if self.finished:
            return

        self.last_image = time.monotonic()
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
            print('目標色が見えません：停止', flush=True)
            self.finished = True
            self.stop()
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) <= 30:
            print('目標色が見えません：停止', flush=True)
            self.finished = True
            self.stop()
            return

        x, y, w, h = cv2.boundingRect(largest)
        center = x + w // 2
        error = center - frame.shape[1] // 2

        if abs(error) <= 10:
            print(f'{TARGET}: ずれ={error:+d}px', flush=True)
            print('目標色が中央です。自動停止・完了！', flush=True)
            self.finished = True
            self.stop()
            return

        cmd = Twist()

        if abs(error) > 80:
            speed = 0.5
        elif abs(error) > 30:
            speed = 0.3
        else:
            speed = 0.15

        cmd.angular.z = -speed if error > 0 else speed
        self.pub.publish(cmd)

        print(
            f'{TARGET}: ずれ={error:+d}px / '
            f'旋回速度={cmd.angular.z:+.2f}',
            flush=True)


def main():
    rclpy.init()
    node = ColorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
