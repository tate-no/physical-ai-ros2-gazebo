import time
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

RANGES = {
    'RED': [((0, 80, 40), (10, 255, 255)),
            ((170, 80, 40), (179, 255, 255))],
    'BLUE': [((100, 80, 40), (130, 255, 255))],
    'YELLOW': [((20, 80, 40), (40, 255, 255))]
}


class ColorRecorder(Node):
    def __init__(self):
        super().__init__('burger_color_record')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Twist, '/burger_test/cmd_vel', 10)
        self.create_subscription(
            Image, '/burger_camera/color_camera/image_raw',
            self.callback, 10)
        self.lock = threading.Lock()
        self.target = None
        self.started = 0.0
        self.last_image = time.monotonic()
        self.last_command = time.monotonic()
        self.timer = self.create_timer(0.1, self.safety_check)

    def stop(self):
        self.pub.publish(Twist())

    def set_target(self, color):
        with self.lock:
            self.stop()
            self.target = color
            self.started = time.monotonic()
            self.last_image = self.started
            self.last_command = self.started
            print(f'\n>>> 目標変更：{color} <<<', flush=True)

    def finish(self, message):
        print(message, flush=True)
        self.target = None
        self.stop()
        print('次の色を入力できます。', flush=True)

    def safety_check(self):
        with self.lock:
            if self.target is None:
                self.stop()
                return
            now = time.monotonic()
            if now - self.started > 30:
                self.finish('30秒経過：安全停止')
            elif now - self.last_image > 2:
                self.finish('カメラ画像が途絶えました：安全停止')
            elif now - self.last_command > 1:
                self.stop()

    def callback(self, msg):
        with self.lock:
            if self.target is None:
                return

            self.last_image = time.monotonic()
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for low, high in RANGES[self.target]:
                mask |= cv2.inRange(
                    hsv, np.array(low), np.array(high))

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                self.finish('目標色が見えません：停止')
                return

            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) <= 30:
                self.finish('目標色が見えません：停止')
                return

            x, y, w, h = cv2.boundingRect(largest)
            center = x + w // 2
            error = center - frame.shape[1] // 2

            if abs(error) <= 10:
                self.finish(
                    f'{self.target}: ずれ={error:+d}px '
                    '→ 中央到達・自動停止！')
                return

            if abs(error) > 80:
                speed = 0.5
            elif abs(error) > 30:
                speed = 0.3
            else:
                speed = 0.15

            cmd = Twist()
            cmd.angular.z = -speed if error > 0 else speed
            self.pub.publish(cmd)
            self.last_command = time.monotonic()

            print(
                f'{self.target}: ずれ={error:+d}px / '
                f'旋回速度={cmd.angular.z:+.2f}',
                flush=True)


def main():
    rclpy.init()
    node = ColorRecorder()
    worker = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    worker.start()

    print('=== 3色連続指示・撮影モード ===', flush=True)
    print('RED / BLUE / YELLOW を入力', flush=True)
    print('終了は EXIT', flush=True)

    try:
        while rclpy.ok():
            color = input('\n目標色を入力 > ').strip().upper()
            if color == 'EXIT':
                break
            if color in RANGES:
                node.set_target(color)
            else:
                print('RED / BLUE / YELLOW を指定してください')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        with node.lock:
            node.target = None
            node.stop()
        rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
