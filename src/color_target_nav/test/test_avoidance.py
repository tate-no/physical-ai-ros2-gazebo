"""Ray-cast synthetic scenes and odometry-driven closed-loop recovery tests."""
import math
import itertools
import unittest

from color_target_nav.avoidance import ScanGuard, Recovery, SpaceCheck, angle_difference
from color_target_nav.control import Mission, sector


SCAN_SEQUENCES = itertools.count(1)


def scan_scene(pose, circles=()):
    """360 rays against circles; shapes are fixed in world coordinates."""
    x, y, yaw = pose
    sx, sy = x - .032 * math.cos(yaw), y - .032 * math.sin(yaw)
    ranges = []
    for i in range(360):
        a = yaw + i * math.tau / 360
        dx, dy = math.cos(a), math.sin(a)
        nearest = float('inf')
        for cx, cy, radius in circles:
            vx, vy = cx - sx, cy - sy
            projection = vx * dx + vy * dy
            discriminant = radius**2 - (vx**2 + vy**2 - projection**2)
            if discriminant < 0:
                continue
            distance = projection - math.sqrt(discriminant)
            if .03 <= distance <= 3.5:
                nearest = min(nearest, distance)
        ranges.append(nearest)
    guard = ScanGuard(ranges, 0, math.tau / 360, .03, 3.5, next(SCAN_SEQUENCES))

    def measure(low, high):
        return sector(ranges, 0, math.tau / 360, low, high, .03, 3.5)

    front = measure(-.35, .35)
    rear = min(measure(2.5, math.pi), measure(-math.pi, -2.5))
    left, right = measure(.35, 2.5), measure(-2.5, -.35)
    return guard, front, rear, left, right


class AvoidanceTests(unittest.TestCase):
    def test_side_obstacle_allows_straight_escape_but_blocks_turn_sweep(self):
        guard, _, _, _, _ = scan_scene((0, 0, 0), [(0.04, -.28, .1)])
        self.assertFalse(guard.turn_safe())
        self.assertTrue(guard.reverse_safe())

    def test_confirmed_goal_progress_has_priority_over_unrelated_side_object(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, [(.04, -.28, .1)])
        mission = Mission(['BLUE'], 0)
        v, w = mission.step(1, True, 0, front, rear, min(left, right), set(),
                            pose, guard, left, right, (-.1, .1))
        self.assertGreater(v, 0)
        self.assertEqual(mission.state, 'APPROACH')
        self.assertIsNone(mission.recovery)

    def test_clear_camera_scan_is_not_preempted_by_previous_box_in_front(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, [(.48, 0, .1)])
        mission = Mission(['BLUE'], 0)
        mission.reacquire(pose, 'scan_next_color', guard)
        v, w = mission.step(1, True, None, front, rear, min(left, right),
                            set(), pose, guard, left, right)
        self.assertEqual(v, 0)
        self.assertNotEqual(w, 0)
        self.assertEqual(mission.state, 'REACQUIRE')
        self.assertIsNone(mission.recovery)

    def test_camera_alignment_checks_small_sweep_instead_of_full_circle(self):
        guard, *_ = scan_scene((0, 0, 0), [(.30, .025, .10)])
        self.assertFalse(guard.turn_safe())
        self.assertTrue(guard.alignment_safe(.10, (-.4, .5)))
        check = SpaceCheck()
        self.assertFalse(check.ready(guard, 'align', .10, (-.4, .5)))
        self.assertFalse(guard.alignment_safe(.10, (float('nan'), .5)))

    def test_other_obstacle_in_target_route_still_blocks_approach(self):
        guard, *_ = scan_scene((0, 0, 0), [(.25, .10, .03)])
        self.assertFalse(guard.approach_safe((-.05, .05)))

    def test_reverse_corridor_checks_diagonal_rear_obstacles(self):
        guard, _, _, _, _ = scan_scene((0, 0, 0), [(-.24, .09, .04)])
        self.assertFalse(guard.reverse_safe())

    def test_pass_corridor_blocks_obstacles_beside_front_beam(self):
        guard, front, _, _, _ = scan_scene((0, 0, 0), [(.30, .12, .03)])
        self.assertFalse(guard.forward_safe())

    def test_invalid_or_partial_scan_never_authorizes_motion(self):
        for ranges in ([float('inf')] * 90, [float('nan')] * 360,
                       [0.0] * 360, [float('-inf')] * 360):
            guard = ScanGuard(ranges, 0, math.tau / 360, .03, 3.5)
            self.assertFalse(guard.reverse_safe())
            self.assertFalse(guard.turn_safe())
            self.assertFalse(guard.forward_safe())
        for start, step in ((float('nan'), .01745), (float('inf'), .01745),
                            (0, float('nan')), (0, float('inf')), (0, 1)):
            self.assertFalse(ScanGuard([float('inf')] * 360, start, step, .03, 3.5).valid)

    def test_invalid_scan_blocks_mission_even_if_sector_values_look_clear(self):
        guard = ScanGuard([float('nan')] * 360, 0, math.tau / 360, .03, 3.5)
        mission = Mission(['BLUE'], 0)
        self.assertEqual(mission.step(1, True, 0, 3.5, 3.5, 3.5, set(),
                                      (0, 0, 0), guard, 3.5, 3.5), (0, 0))
        self.assertEqual(mission.reason, 'invalid_or_incomplete_scan')

    def test_unexpected_contact_stops_during_bypass(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, [(.04, -.28, .1)])
        mission = Mission(['BLUE'], 0)
        mission.step(1, True, 0, front, rear, min(left, right), set(), pose, guard, left, right)
        self.assertEqual(mission.step(2, True, 0, front, rear, min(left, right),
                                      {'gray_obstacle'}, pose, guard, left, right), (0, 0))
        self.assertEqual(mission.failure_reason, 'unexpected_contact: gray_obstacle')

    def test_recovery_moves_out_then_turns_and_passes_in_synthetic_scene(self):
        pose = (0.0, 0.0, 0.0)
        obstacles = [(.30, 0, .10)]
        mission = Mission(['BLUE'], 0, timeout=120)
        space_check = SpaceCheck()
        stages = set()
        reversed_distance = 0.0
        passed_distance = 0.0
        dt = .1
        for i in range(450):
            now = i * dt
            guard, front, rear, left, right = scan_scene(pose, obstacles)
            v, w = mission.step(now, True, 0.0, front, rear, min(left, right),
                                set(), pose=pose, guard=guard, left=left, right=right)
            motion = 'reverse' if v < 0 else 'turn' if w else 'forward' if v > 0 else None
            if motion is not None and not space_check.ready(guard, motion):
                v, w = 0, 0
            elif motion is None:
                space_check.reset()
            stages.add(mission.state)
            if mission.recovery_count and mission.recovery is None:
                break
            if v < 0:
                self.assertTrue(guard.reverse_safe())
                self.assertGreater(rear, .3)
                reversed_distance += -v * dt
            if w:
                self.assertTrue(guard.turn_safe())
                self.assertEqual(v, 0)
            if v > 0:
                self.assertTrue(guard.forward_safe())
                passed_distance += v * dt
            pose = (pose[0] + v * math.cos(pose[2]) * dt,
                    pose[1] + v * math.sin(pose[2]) * dt, pose[2] + w * dt)
        self.assertEqual(mission.state, 'REACQUIRE')
        self.assertEqual(mission.recovery_count, 1)
        self.assertGreater(reversed_distance, .1)
        self.assertGreaterEqual(passed_distance, .25)
        self.assertTrue({'RECOVER_BACKUP', 'RECOVER_TURN', 'RECOVER_PASS'} <= stages)

    def test_recovery_does_not_reverse_into_blocked_rear(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, [(0.04, -.28, .1), (-.23, 0, .05)])
        recovery = Recovery(0, pose, 1)
        self.assertEqual(recovery.step(1, pose, guard, front, rear), (0, 0))
        self.assertFalse(recovery.failed)
        self.assertEqual(recovery.step(46, pose, guard, front, rear), (0, 0))
        self.assertTrue(recovery.failed)

    def test_blocked_rear_can_escape_forward_and_return_to_camera_scan(self):
        pose = (0.0, 0.0, 0.0)
        obstacles = [(-.23, 0, .05)]
        mission = Mission(['BLUE'], 0, timeout=120)
        check = SpaceCheck()
        stages = set()
        forward_motion = 0.0
        for i in range(450):
            guard, front, rear, left, right = scan_scene(pose, obstacles)
            v, w = mission.step(i * .1, True, None, front, rear, min(left, right),
                                set(), pose, guard, left, right)
            stages.add(mission.state)
            if mission.recovery_count and mission.recovery is None:
                break
            motion = 'reverse' if v < 0 else 'turn' if w else 'forward' if v > 0 else None
            if motion is not None and not check.ready(guard, motion):
                v, w = 0, 0
            elif motion is None:
                check.reset()
            self.assertGreaterEqual(v, 0, 'must not back into the rear obstacle')
            if v > 0:
                self.assertTrue(guard.forward_safe())
                forward_motion += v * .1
            if w:
                self.assertTrue(guard.turn_safe())
            pose = (pose[0] + v * math.cos(pose[2]) * .1,
                    pose[1] + v * math.sin(pose[2]) * .1, pose[2] + w * .1)
        self.assertEqual(mission.state, 'REACQUIRE')
        self.assertGreater(forward_motion, .1)
        self.assertIn('RECOVER_FORWARD', stages)

    def test_no_forward_escape_when_front_and_rear_are_blocked(self):
        pose = (0, 0, 0)
        guard, front, rear, _, _ = scan_scene(pose, [(-.23, 0, .05), (.30, 0, .05)])
        recovery = Recovery(0, pose, 1)
        self.assertEqual(recovery.step(1, pose, guard, front, rear), (0, 0))
        self.assertEqual(recovery.stage, 'BACKUP')
        paths = guard.path_report()
        self.assertFalse(paths['reverse_clear'])
        self.assertFalse(paths['forward_clear'])
        self.assertTrue(paths['reverse_blockers'])
        self.assertTrue(paths['forward_blockers'])

    def test_recorded_gazebo_blocker_points_select_forward_escape(self):
        # User's stationary scan: right-rear points block reverse and turn,
        # while the forward corridor is clear. Reproject into scan beams.
        points = [(-.186, -.131), (-.076, -.198), (-.117, -.179),
                  (-.147, -.156), (-.162, -.142), (-.109, -.187), (-.100, -.192)]
        ranges = [float('inf')] * 360
        for x, y in points:
            angle = math.atan2(y, x + .032) % math.tau
            index = round(angle / math.tau * 360) % 360
            ranges[index] = min(ranges[index], math.hypot(x + .032, y))
        guard = ScanGuard(ranges, 0, math.tau / 360, .03, 3.5, 1)
        self.assertFalse(guard.reverse_safe())
        self.assertFalse(guard.turn_safe())
        self.assertTrue(guard.forward_safe())
        recovery = Recovery(0, (0, 0, 0), 1)
        self.assertEqual(recovery.step(1, (0, 0, 0), guard, 1.32, .70), (0, 0))
        self.assertEqual(recovery.stage, 'FORWARD')
        self.assertEqual(recovery.step(2, (0, 0, 0), guard, 1.32, .70), (.04, 0))

    def test_choose_turn_direction_using_left_and_right_clearance(self):
        for cy, expected in ((-.28, 1), (.28, -1)):
            pose = (0, 0, 0)
            guard, front, rear, left, right = scan_scene(pose, [(.04, cy, .1)])
            mission = Mission(['BLUE'], 0)
            mission.recover(1, pose, guard, front, rear, left, right, 'test_direction_choice')
            self.assertEqual(mission.recovery.direction, expected)

    def test_freshness_loss_stops_even_during_recovery(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, [(.04, -.28, .1)])
        mission = Mission(['BLUE'], 0)
        mission.step(1, True, 0, front, rear, min(left, right), set(), pose, guard, left, right)
        self.assertEqual(mission.step(2, False, 0, front, rear, min(left, right),
                                      set(), pose, guard, left, right), (0, 0))

    def test_odometry_stall_triggers_recovery_and_reports_timeout(self):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose)
        mission = Mission(['BLUE'], 0, timeout=120, automatic_retries=0)
        mission.step(0, True, 0, front, rear, 3.5, set(), pose, guard, left, right)
        mission.step(11, True, 0, front, rear, 3.5, set(), pose, guard, left, right)
        self.assertEqual(mission.state, 'RECOVER_TURN')
        mission.step(57, True, 0, front, rear, 3.5, set(), pose, guard, left, right)
        self.assertEqual(mission.failure_reason, 'recovery_timeout')

    def test_turn_wraparound_and_recovery_attempt_bound(self):
        self.assertAlmostEqual(angle_difference(-math.pi + .1, math.pi - .1), .2)
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose)
        mission = Mission(['BLUE'], 0, automatic_retries=0)
        mission.recovery_count = 3
        self.assertEqual(mission.recover(1, pose, guard, front, rear, left, right, 'blocked'), (0, 0))
        self.assertEqual(mission.failure_reason, 'recovery_attempt_limit')


class SpaceCheckTests(unittest.TestCase):
    def test_reports_all_eight_directions(self):
        ranges = [float('inf')] * 360
        names = ('front', 'front_left', 'left', 'rear_left', 'rear',
                 'rear_right', 'right', 'front_right')
        for i in range(8):
            ranges[i * 45] = 2 - .2 * i
        guard = ScanGuard(ranges, 0, math.tau / 360, .03, 3.5, 1)
        for i, name in enumerate(names):
            self.assertAlmostEqual(guard.clearance[name], 2 - .2 * i)

    def test_repeated_timer_ticks_do_not_count_as_distinct_scans(self):
        check = SpaceCheck()
        guard, *_ = scan_scene((0, 0, 0))
        for _ in range(50):
            self.assertFalse(check.ready(guard, 'reverse'))
        self.assertEqual(check.confirmed, 1)
        for expected in (2, 3):
            guard, *_ = scan_scene((0, 0, 0))
            self.assertEqual(check.ready(guard, 'reverse'), expected == 3)
        self.assertTrue(check.ready(guard, 'reverse'))

    def test_new_obstacle_stops_reverse_immediately_and_restarts_confirmation(self):
        check = SpaceCheck()
        for _ in range(3):
            guard, *_ = scan_scene((0, 0, 0))
            check.ready(guard, 'reverse')
        blocked, *_ = scan_scene((0, 0, 0), [(-.23, .08, .05)])
        self.assertFalse(check.ready(blocked, 'reverse'))
        self.assertEqual(check.confirmed, 0)
        guard, *_ = scan_scene((0, 0, 0))
        self.assertFalse(check.ready(guard, 'reverse'))
        self.assertEqual(check.confirmed, 1)

    def test_change_of_direction_or_freshness_reset_requires_new_checks(self):
        check = SpaceCheck()
        for _ in range(3):
            guard, *_ = scan_scene((0, 0, 0))
            check.ready(guard, 'reverse')
        self.assertFalse(check.ready(guard, 'turn'))
        self.assertEqual(check.confirmed, 1)
        check.reset()
        self.assertFalse(check.ready(guard, 'reverse'))
        self.assertEqual(check.confirmed, 1)

    def test_unknown_scan_and_invalid_confirmation_settings(self):
        check = SpaceCheck()
        self.assertFalse(check.ready(None, 'reverse'))
        unknown = ScanGuard([float('nan')] * 360, 0, math.tau / 360, .03, 3.5, 1)
        self.assertFalse(check.ready(unknown, 'reverse'))
        self.assertTrue(all(value is None for value in unknown.clearance.values()))
        for required in (0, 1, 11, 3.5):
            with self.assertRaises(ValueError):
                SpaceCheck(required)


if __name__ == '__main__':
    unittest.main()
