import rclpy
import time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(
            Twist, '/burger_test/cmd_vel', 10
        )
        self.create_subscription(
            Image,
            '/burger_camera/color_camera/image_raw',
            self.callback,
            10
        )
        self.rotation_done = False
        self.last_log = self.get_clock().now()
        print('3色認識を開始しました！')

    def stop_robot(self):
        stop = Twist()
        for _ in range(40):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

    def rotate_once(self, angular_speed):
        move = Twist()
        move.angular.z = angular_speed

        try:
            start = time.monotonic()
            while time.monotonic() - start < 0.1:
                self.cmd_pub.publish(move)
                time.sleep(0.05)
        finally:
            self.stop_robot()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ranges = {
            'RED': [((0, 80, 40), (10, 255, 255)),
                    ((170, 80, 40), (179, 255, 255))],
            'BLUE': [((100, 80, 40), (130, 255, 255))],
            'YELLOW': [((20, 80, 40), (40, 255, 255))]
        }

        results = []

        for name, limits in ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

            for lower, upper in limits:
                mask |= cv2.inRange(
                    hsv, np.array(lower), np.array(upper)
                )

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area > 30:
                    x, y, w, h = cv2.boundingRect(largest)
                    cx = x + w // 2
                    cy = y + h // 2
                    error = cx - 320

                    if abs(error) <= 10:
                        direction = '中央・停止'
                    elif error > 0:
                        direction = '負の角速度'
                    else:
                        direction = '正の角速度'

                    results.append(
                        f'{name}: ({cx}, {cy}), '
                        f'ずれ={error:+d}px, 判定={direction}'
                    )

                    if name == 'RED' and not self.rotation_done:
                        self.rotation_done = True
                        if abs(error) > 10:
                            speed = -0.2 if error > 0 else 0.2
                            print('赤い箱へ短時間回転:', speed)
                            self.rotate_once(speed)
                        else:
                            print('赤い箱は中央付近です')
                            self.stop_robot()

        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds > 1_000_000_000:
            print('検出結果:', ', '.join(results) if results else 'なし')
            self.last_log = now

def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
