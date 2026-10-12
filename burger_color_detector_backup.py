import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            '/burger_camera/color_camera/image_raw',
            self.callback,
            10
        )
        self.last_log = self.get_clock().now()
        print('3色認識を開始しました！')

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
                    results.append(
                        f'{name}: ({cx}, {cy})'
                    )

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
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
