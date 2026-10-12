import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('safe_rotate')
pub = node.create_publisher(Twist, '/burger_test/cmd_vel', 10)

move = Twist()
move.angular.z = 0.2
stop = Twist()

try:
    print('0.5秒間の回転指令を開始します')
    start = time.monotonic()

    while time.monotonic() - start < 0.5:
        pub.publish(move)
        time.sleep(0.05)

finally:
    print('停止指令を送信します')
    for _ in range(40):
        pub.publish(stop)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()
    print('制御終了')
