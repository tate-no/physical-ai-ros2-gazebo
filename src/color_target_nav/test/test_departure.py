import unittest
from color_target_nav.control import Mission
from test_avoidance import scan_scene

class DepartureTests(unittest.TestCase):
    def step(self, mission, now, pose, contacts=None, guard=None, fresh=True):
        g, front, rear, left, right = scan_scene(pose)
        return mission.step(now, fresh, 0, front, rear, min(left, right),
                            contacts or set(), pose, guard or g, left, right)

    def begin(self):
        m = Mission(['RED', 'BLUE'], 0)
        self.step(m, 1, (0, 0, 0), {'red_box'})
        return m

    def test_clear_scan_without_actual_reverse_does_not_release(self):
        m = self.begin()
        for i in range(5):
            self.assertLess(self.step(m, 2+i, (0, 0, 0))[0], 0)
        self.assertEqual(m.state, 'RETREAT')
        self.assertLess(self.step(m, 8, (0, .25, 0))[0], 0)

    def test_needs_three_distinct_clear_scans_after_reverse(self):
        m = self.begin()
        pose = (-.25, 0, 0)
        guard = scan_scene(pose)[0]
        for i in range(5):
            self.assertEqual(self.step(m, 2+.01*i, pose, guard=guard), (0, 0))
        self.assertEqual(m.departure_clear_scans, 1)
        self.step(m, 3, pose)
        self.assertEqual(m.state, 'RETREAT')
        self.step(m, 4, pose)
        self.assertEqual(m.state, 'REACQUIRE')
        self.assertEqual(m.target, 'BLUE')
        self.assertEqual(m.completed_targets, 1)

    def test_contact_and_stale_scan_reset_departure_confirmation(self):
        m = self.begin()
        pose = (-.25, 0, 0)
        self.step(m, 2, pose)
        self.step(m, 3, pose, {'red_box'})
        self.assertEqual(m.departure_clear_scans, 0)
        self.step(m, 4, pose)
        self.step(m, 5, pose, fresh=False)
        self.assertEqual(m.departure_clear_scans, 0)
        self.assertEqual(m.state, 'RETREAT')

    def test_departure_confirmation_does_not_extend_deadline(self):
        m = self.begin()
        self.step(m, 47, (-.25, 0, 0))
        self.assertEqual(m.state, 'WAIT_RETRY')
        self.assertEqual(m.departure_origin, (0, 0, 0))
