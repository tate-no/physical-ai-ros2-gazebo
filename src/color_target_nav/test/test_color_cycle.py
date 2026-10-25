"""Synthetic camera + 360-ray LiDAR + kinematics, NOT Gazebo physics."""
import math
import unittest

import cv2
import numpy as np

from color_target_nav.avoidance import SpaceCheck, angle_difference
from color_target_nav.control import Mission
from color_target_nav.perception import detect_region, VisionLock
from test_avoidance import scan_scene


def camera(pose, objects):
    width, height = 320, 240
    focal = width / (2 * math.tan(1.57 / 2))
    sx = pose[0] + .05 * math.cos(pose[2])
    sy = pose[1] + .05 * math.sin(pose[2])
    frame = np.zeros((height, width, 3), np.uint8)
    colors = {'RED': (0, 0, 255), 'BLUE': (255, 0, 0),
              'YELLOW': (0, 255, 255), 'GRAY': (120, 120, 120)}
    ordered = sorted(objects, key=lambda obj: math.hypot(obj[1] - sx, obj[2] - sy), reverse=True)
    for name, x, y, radius in ordered:
        distance = math.hypot(x - sx, y - sy)
        bearing = angle_difference(math.atan2(y - sy, x - sx), pose[2])
        half = math.asin(min(.99, radius / max(.001, distance)))
        low, high = bearing - half, bearing + half
        if low > .785 or high < -.785:
            continue
        low, high = max(-.785, low), min(.785, high)
        left = round(width / 2 - focal * math.tan(high))
        right = round(width / 2 - focal * math.tan(low))
        vertical = max(2, min(height // 2, round(focal * radius / max(.01, distance))))
        cv2.rectangle(frame, (left, height // 2 - vertical),
                       (right, height // 2 + vertical), colors[name], -1)
    return frame


def physical_contacts(pose, objects):
    contacts = set()
    for name, x, y, radius in objects:
        dx, dy = x - pose[0], y - pose[1]
        bx = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
        by = -dx * math.sin(pose[2]) + dy * math.cos(pose[2])
        # Front contact bumper, from the derived SDF.
        px, py = max(.09, min(.19, bx)), max(-.07, min(.07, by))
        if math.hypot(bx - px, by - py) <= radius:
            contacts.add(name.lower() + '_box' if name != 'GRAY' else 'gray_obstacle')
    return contacts


def run_cycle(objects, laps=2, steps=14000):
    mission = Mission(['RED', 'BLUE', 'YELLOW'], 0, timeout=300, repeat=True)
    vision = VisionLock()
    space = SpaceCheck()
    pose = (0.0, 0.0, 0.0)
    circles = [(x, y, radius) for _, x, y, radius in objects]
    touched = []
    transitions = []
    last_state = None
    dt = .1
    for i in range(steps):
        now = i * dt
        frame = camera(pose, objects)
        detection = detect_region(frame, mission.target)
        locked = vision.update(mission.target, mission.vision_epoch, detection)
        bearing = locked.bearing if locked is not None else None
        bounds = (locked.low, locked.high) if locked is not None else None
        guard, front, rear, left, right = scan_scene(pose, circles)
        previous_epoch, previous_completed, previous_target = mission.vision_epoch, mission.completed_targets, mission.target
        v, w = mission.step(now, True, bearing, front, rear, min(left, right),
                            physical_contacts(pose, objects), pose, guard, left, right, bounds)
        if previous_epoch != mission.vision_epoch:
            vision.reset(mission.target, mission.vision_epoch)
        if previous_completed != mission.completed_targets:
            touched.append(previous_target)
        if mission.state != last_state:
            transitions.append((round(now, 1), mission.state, mission.target, mission.reason,
                                tuple(round(value, 3) for value in pose),
                                round(bearing, 3) if bearing is not None else None))
            last_state = mission.state
        if mission.laps >= laps or mission.state == 'FAILED':
            break
        motion = ('reverse' if v < 0 else 'align' if w and mission.state == 'ALIGN' else 'turn' if w else
                  'forward' if v > 0 and mission.state != 'APPROACH' else None)
        if motion is not None and not space.ready(guard, motion, bearing, bounds):
            v, w = 0.0, 0.0
        elif motion is None:
            space.reset()
        pose = (pose[0] + v * math.cos(pose[2]) * dt,
                pose[1] + v * math.sin(pose[2]) * dt, pose[2] + w * dt)
    return mission, touched, transitions


class ColorCycleTests(unittest.TestCase):
    def test_existing_demo_positions_repeat_two_laps_with_obstacle(self):
        objects = [('RED', 3, 0, .10), ('BLUE', 3, 1, .10),
                   ('YELLOW', 3, -1, .10), ('GRAY', 1.5, .8, .15)]
        mission, touched, transitions = run_cycle(objects, laps=2)
        self.assertEqual(touched, ['RED', 'BLUE', 'YELLOW'] * 2, str(transitions[-20:]))
        self.assertEqual(mission.laps, 2)

    def test_two_complete_color_cycles_with_synthetic_camera_and_lidar(self):
        objects = [('RED', 1.3, 0, .10), ('BLUE', 1.3, .9, .10), ('YELLOW', .1, 1.2, .10)]
        mission, touched, transitions = run_cycle(objects)
        self.assertEqual(touched, ['RED', 'BLUE', 'YELLOW'] * 2, str(transitions[-15:]))
        self.assertEqual(mission.laps, 2)
        self.assertEqual(mission.target, 'RED')
        self.assertIn('REACQUIRE', [row[1] for row in transitions])

    def test_obstacle_avoidance_returns_to_camera_and_same_target_then_touch(self):
        objects = [('RED', 1.3, 0, .10), ('BLUE', 1.3, .9, .10),
                   ('YELLOW', .1, 1.2, .10), ('GRAY', .65, .10, .08)]
        mission, touched, transitions = run_cycle(objects, laps=2)
        self.assertEqual(touched, ['RED', 'BLUE', 'YELLOW'] * 2, str(transitions[-20:]))
        states = [row[1] for row in transitions]
        self.assertTrue(any(state.startswith('RECOVER_') for state in states))
        self.assertIn('REACQUIRE', states)
        self.assertEqual(mission.laps, 2)


if __name__ == '__main__':
    unittest.main()
