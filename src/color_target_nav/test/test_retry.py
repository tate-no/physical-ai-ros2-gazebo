import unittest

from color_target_nav.control import Mission
from test_avoidance import scan_scene


class RetryTests(unittest.TestCase):
    def begin_blocked_recovery(self, mission, circles):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, circles)
        mission.recover(1, pose, guard, front, rear, left, right, 'blocked_test')

    def step(self, mission, now, circles=(), fresh=True, contacts=None):
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose, circles)
        return mission.step(now, fresh, 0, front, rear, min(left, right),
                            contacts or set(), pose, guard, left, right)

    def test_obstacle_disappears_then_retry_resumes_same_target_after_cooldown(self):
        mission = Mission(['BLUE'], 0, timeout=300)
        circles = [(.04, -.28, .1), (-.23, 0, .05)]
        self.begin_blocked_recovery(mission, circles)
        self.step(mission, 47, circles)
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.assertEqual(mission.failure_reason, None)
        self.assertEqual(self.step(mission, 49), (0, 0))
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.assertEqual(self.step(mission, 51), (0, 0))
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.step(mission, 51.2)
        self.step(mission, 51.4)
        self.assertEqual(mission.state, 'REACQUIRE')
        self.assertGreater(self.step(mission, 52)[0], 0)
        self.assertEqual(mission.target, 'BLUE')
        self.assertEqual(mission.retry_count, 1)

    def test_still_blocked_waits_without_blind_reverse_then_stops(self):
        mission = Mission(['BLUE'], 0, timeout=300)
        circles = [(.04, -.28, .1), (-.23, 0, .05), (.30, 0, .05)]
        self.begin_blocked_recovery(mission, circles)
        self.step(mission, 47, circles)
        self.assertEqual(self.step(mission, 51, circles), (0, 0))
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.assertEqual(self.step(mission, 78, circles), (0, 0))
        self.assertEqual(mission.state, 'FAILED')
        self.assertTrue(mission.failure_reason.startswith('retry_no_safe_space'))

    def test_sensor_loss_never_releases_wait_and_wait_deadline_is_bounded(self):
        mission = Mission(['BLUE'], 0, timeout=300)
        mission.wait_or_fail(1, 'blocked', 'RECOVER')
        self.assertEqual(self.step(mission, 5, fresh=False), (0, 0))
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.step(mission, 32, fresh=False)
        self.assertEqual(mission.state, 'FAILED')

    def test_post_contact_retry_keeps_color_order_and_allows_departure_contact(self):
        mission = Mission(['RED', 'BLUE'], 0, timeout=300)
        self.step(mission, 1, contacts={'red_box'})
        # At timeout the robot is still by the touched red box.
        self.step(mission, 47, [(0.30, 0, .10)])
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.assertEqual(mission.target, 'BLUE')
        self.step(mission, 51, [(0.30, 0, .10)], contacts={'red_box'})
        self.step(mission, 51.2, [(0.30, 0, .10)], contacts={'red_box'})
        self.step(mission, 51.4, [(0.30, 0, .10)], contacts={'red_box'})
        self.assertEqual(mission.state, 'RETREAT')
        self.assertLess(self.step(mission, 52, [(0.30, 0, .10)], contacts={'red_box'})[0], 0)
        self.assertEqual(mission.index, 1)

    def test_unexpected_contact_is_terminal_and_never_automatically_retried(self):
        mission = Mission(['RED'], 0)
        self.step(mission, 1, contacts={'gray_obstacle'})
        self.assertEqual(mission.state, 'FAILED')
        self.assertEqual(mission.retry_count, 0)
        self.assertEqual(self.step(mission, 10), (0, 0))
        mission = Mission(['BLUE'], 0)
        mission.wait_or_fail(1, 'blocked', 'RECOVER')
        self.step(mission, 5, contacts={'gray_obstacle'})
        self.assertEqual(mission.state, 'FAILED')

    def test_retry_budget_and_original_target_timeout_are_not_reset(self):
        mission = Mission(['BLUE'], 0, timeout=120)
        for now in (1, 10):
            mission.wait_or_fail(now, 'blocked', 'RECOVER')
            self.step(mission, now + 4)
            self.step(mission, now + 4.2)
            self.step(mission, now + 4.4)
        self.assertEqual(mission.retry_count, 2)
        mission.wait_or_fail(20, 'blocked', 'RECOVER')
        self.assertEqual(mission.state, 'FAILED')
        self.assertTrue(mission.failure_reason.startswith('automatic_retry_limit'))
        mission = Mission(['BLUE'], 0, timeout=5)
        mission.wait_or_fail(1, 'blocked', 'RECOVER')
        self.step(mission, 6)
        self.assertEqual(mission.failure_reason, 'target_timeout')

    def test_recovery_limit_rechecks_then_reacquires_same_color(self):
        mission = Mission(['BLUE'], 0, timeout=300)
        mission.recovery_count = 3
        mission.relocation_count = 2
        self.begin_blocked_recovery(mission, [])
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.assertIsNone(mission.failure_reason)
        for now in (5, 5.2, 5.4):
            self.step(mission, now)
        self.assertEqual(mission.state, 'REACQUIRE')
        self.assertEqual(mission.recovery_count, 0)
        self.assertEqual(mission.retry_count, 1)
        self.assertEqual(mission.target, 'BLUE')
        self.assertEqual(mission.started, 0)
        self.assertGreater(mission.vision_epoch, 0)
        self.assertGreater(self.step(mission, 6)[0], 0)

    def test_retry_configuration_validation(self):
        for count in (-1, 6, 1.5):
            with self.assertRaises(ValueError):
                Mission(['BLUE'], 0, automatic_retries=count)
        with self.assertRaises(ValueError):
            Mission(['BLUE'], 0, retry_wait=3, retry_window=2)

    def test_configurable_recovery_window_for_slow_gazebo(self):
        mission = Mission(['BLUE'], 0, timeout=300, automatic_retries=0, recovery_timeout=90)
        self.step(mission, 0)
        self.step(mission, 11)
        self.step(mission, 57)
        self.assertNotEqual(mission.state, 'FAILED')
        self.step(mission, 102)
        self.assertEqual(mission.failure_reason, 'recovery_timeout')
        with self.assertRaises(ValueError):
            Mission(['BLUE'], 0, recovery_timeout=0)

    def test_retry_needs_distinct_fresh_scans_not_repeated_timer_ticks(self):
        mission = Mission(['BLUE'], 0)
        mission.wait_or_fail(0, 'blocked', 'RECOVER')
        pose = (0, 0, 0)
        guard, front, rear, left, right = scan_scene(pose)
        for i in range(20):
            mission.step(4 + .01*i, True, 0, front, rear, min(left, right),
                         set(), pose, guard, left, right)
        self.assertEqual(mission.retry_safe_scans, 1)
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.step(mission, 5, fresh=False)
        self.assertEqual(mission.retry_safe_scans, 0)
        self.step(mission, 6)
        self.step(mission, 6.2)
        self.assertEqual(mission.state, 'WAIT_RETRY')
        self.step(mission, 6.4)
        self.assertEqual(mission.state, 'REACQUIRE')


if __name__ == '__main__':
    unittest.main()
