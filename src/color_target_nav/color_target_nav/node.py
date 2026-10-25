"""ROS adapter: bounded sensor age, nonblocking callbacks, physical contacts."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
from .control import Mission, sector
from .perception import detect_region, VisionLock
from .avoidance import ScanGuard, SpaceCheck


class Navigator(Node):
    def __init__(self):
        super().__init__('color_target_nav')
        defaults = {'targets': ['RED', 'BLUE', 'YELLOW'], 'repeat': True,
                    'target_timeout': 300.0, 'sensor_timeout': 0.7,
                    'image_topic': '/burger_camera/color_camera/image_raw',
                    'scan_topic': '/scan', 'contact_topic': '/color_nav/contacts',
                    'odom_topic': '/burger_test/odom',
                    'cmd_topic': '/color_nav/desired_cmd', 'horizontal_fov': 1.57,
                    'min_area': 80.0, 'space_confirm_scans': 3,
                    'automatic_retries': 2, 'retry_wait': 3.0, 'retry_window': 30.0,
                    'vision_confirm_frames': 3, 'recovery_timeout': 90.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.param = lambda name: self.get_parameter(name).value
        self.mission = Mission(self.param('targets'), time.monotonic(),
                               self.param('target_timeout'), self.param('repeat'),
                               self.param('automatic_retries'), self.param('retry_wait'),
                               self.param('retry_window'), self.param('space_confirm_scans'),
                               self.param('recovery_timeout'))
        self.bridge = CvBridge()
        self.vision = VisionLock(self.param('vision_confirm_frames'))
        self.bearing = None
        self.target_bounds = None
        self.image_at = self.scan_at = self.contact_at = float('-inf')
        self.odom_at = float('-inf')
        self.pose = None
        self.guard = None
        self.scan_sequence = 0
        self.space_check = SpaceCheck(self.param('space_confirm_scans'))
        self.left = self.right = None
        self.last_report_at = 0.0
        self.last_state = None
        self.distances = (None, None, None)
        self.contacts = set()
        self.pub = self.create_publisher(Twist, self.param('cmd_topic'), 10)
        self.status = self.create_publisher(String, '/color_nav/status', 10)
        self.create_subscription(Image, self.param('image_topic'), self.image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.param('scan_topic'), self.scan, qos_profile_sensor_data)
        self.create_subscription(ContactsState, self.param('contact_topic'), self.contact, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.param('odom_topic'), self.odometry, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('controller_version=recovery_v9_open_space_escape; '
                               f'order={" -> ".join(self.mission.targets)}; repeat={self.mission.repeat}; '
                               f'camera_frames={self.vision.required}; space_scans={self.space_check.required}')

    def image(self, msg):
        try:
            detection = detect_region(self.bridge.imgmsg_to_cv2(msg, 'bgr8'),
                                      self.mission.target, self.param('min_area'),
                                      self.param('horizontal_fov'))
            locked = self.vision.update(self.mission.target, self.mission.vision_epoch, detection)
            self.bearing = locked.bearing if locked is not None else None
            self.target_bounds = (locked.low, locked.high) if locked is not None else None
            self.image_at = time.monotonic()
        except Exception as error:
            self.vision.reset(self.mission.target, self.mission.vision_epoch)
            self.bearing = self.target_bounds = None
            self.image_at = float('-inf')
            self.get_logger().error(str(error))

    def scan(self, msg):
        self.scan_sequence += 1
        self.guard = ScanGuard(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, self.scan_sequence)
        def distance(lo, hi):
            return sector(msg.ranges, msg.angle_min, msg.angle_increment,
                          lo, hi, msg.range_min, msg.range_max)
        side_values = (distance(0.35, 2.5), distance(-2.5, -0.35))
        self.left, self.right = side_values
        rear_values = (distance(2.5, 3.142), distance(-3.142, -2.5))
        self.distances = (distance(-0.35, 0.35),
                          min(rear_values) if None not in rear_values else None,
                          min(side_values) if None not in side_values else None)
        self.scan_at = time.monotonic()

    def odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            self.odom_at = float('-inf')
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = p.x, p.y, yaw
        self.odom_at = time.monotonic()

    def contact(self, msg):
        contacts = set()
        for state in msg.states:
            names = [state.collision1_name.split('::')[0],
                     state.collision2_name.split('::')[0]]
            contacts.update(n for n in names if n != 'burger_camera_main')
        self.contacts.update(contacts)
        self.contact_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = all(0 <= now - t < self.param('sensor_timeout')
                    for t in (self.image_at, self.scan_at, self.contact_at, self.odom_at))
        old_epoch = self.mission.vision_epoch
        linear, angular = self.mission.step(now, fresh, self.bearing,
                                            *self.distances, self.contacts,
                                            pose=self.pose, guard=self.guard,
                                            left=self.left, right=self.right,
                                            target_bounds=self.target_bounds)
        self.contacts.clear()
        if old_epoch != self.mission.vision_epoch:
            self.vision.reset(self.mission.target, self.mission.vision_epoch)
            self.bearing = None
            self.target_bounds = None
        motion = ('reverse' if linear < 0 else
                  'align' if angular != 0 and self.mission.state == 'ALIGN' else
                  'turn' if angular != 0 else
                  'forward' if linear > 0 and self.mission.state != 'APPROACH' else None)
        phase = self.mission.state
        if not fresh:
            self.space_check.reset()
        elif motion is not None:
            if not self.space_check.ready(self.guard, motion, self.bearing, self.target_bounds):
                linear, angular = 0.0, 0.0
                phase = 'CHECK_SPACE'
        else:
            self.space_check.reset()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        self.pub.publish(cmd)
        if self.last_state != phase or (now - self.last_report_at >= 2 and
                (self.mission.state in ('RETREAT', 'WAIT_RETRY') or self.mission.state.startswith('RECOVER_'))):
            self.get_logger().info(f'{phase}/{self.mission.state}: {self.mission.reason}; '
                                   f'target={self.mission.target}, touched={self.mission.completed_targets}, '
                                   f'laps={self.mission.laps}, camera={self.vision.count}/{self.vision.required}; '
                                   f'space={self.space_check.confirmed}/{self.space_check.required}; '
                                   f'linear={linear:.3f}, angular={angular:.3f}; '
                                   f'paths={self.guard.path_report() if self.guard is not None else None}')
            self.last_state, self.last_report_at = phase, now
        self.status.publish(String(data=json.dumps({'state': self.mission.state,
                            'index': self.mission.index, 'target': self.mission.target
                            if self.mission.state != 'DONE' else None,
                            'sensors_fresh': fresh, 'controller_version': 'recovery_v9_open_space_escape',
                            'repeat': self.mission.repeat, 'target_order': self.mission.targets,
                            'completed_targets': self.mission.completed_targets,
                            'laps': self.mission.laps, 'vision_epoch': self.mission.vision_epoch,
                            'camera_confirmed': self.bearing is not None,
                            'camera_frames': self.vision.count,
                            'camera_frames_required': self.vision.required,
                            'camera_relocations': self.mission.camera_relocations,
                            'motion_phase': phase,
                            'space_check': {'motion': self.space_check.motion,
                                             'confirmed': self.space_check.confirmed,
                                             'required': self.space_check.required,
                                             'reason': self.space_check.reason},
                            'surroundings_m': self.guard.clearance if self.guard is not None else None,
                            'path_checks': self.guard.path_report() if self.guard is not None else None,
                            'reason': self.mission.reason,
                            'failure_reason': self.mission.failure_reason,
                            'relocation_count': self.mission.relocation_count,
                            'departure_clear_scans': self.mission.departure_clear_scans,
                            'recovery_count': self.mission.recovery_count,
                            'recovery_triggers': self.mission.recovery_triggers,
                            'target_bearing_rad': self.bearing,
                            'retry_count': self.mission.retry_count,
                            'retry_cause': self.mission.retry_cause,
                            'retry_safe_scans': self.mission.retry_safe_scans,
                            'sensor_age_s': {name: round(now - stamp, 3) if math.isfinite(stamp) else None
                                             for name, stamp in (('image', self.image_at),
                                             ('scan', self.scan_at), ('contact', self.contact_at),
                                             ('odom', self.odom_at))},
                            'clearance_m': dict(zip(('front', 'rear', 'sides'), self.distances)),
                            'linear_m_s': linear, 'angular_rad_s': angular})))


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
