import math
import unittest
from color_target_nav.control import Mission, sector


class ControlTests(unittest.TestCase):
    def step(self, mission, now=1, **changes):
        args = dict(fresh=True, bearing=0.0, front=1.0, rear=1.0, sides=1.0, contacts=set())
        args.update(changes)
        return mission.step(now, **args)

    def test_sensor_loss_and_unknown_ranges_stop(self):
        for changes in ({'fresh': False}, {'front': None}, {'rear': None}, {'sides': None}):
            self.assertEqual(self.step(Mission(['RED'], 0), **changes), (0, 0))

    def test_scan_wrapping_invalid_and_infinity(self):
        self.assertEqual(sector([1, 2, 3, 4], 0, math.pi / 2, -.1, .1, .03, 3.5), 1)
        self.assertIsNone(sector([float('nan')], 0, 1, -.1, .1, .03, 3.5))
        self.assertIsNone(sector([0.01], 0, 1, -.1, .1, .03, 3.5))
        self.assertEqual(sector([float('inf')], 0, 1, -.1, .1, .03, 3.5), 3.5)

    def test_distance_alone_never_completes(self):
        mission = Mission(['RED'], 0)
        self.assertEqual(self.step(mission, front=.25)[0], .015)
        self.assertEqual(mission.index, 0)
        self.assertEqual(self.step(mission, front=.1), (0, 0))

    def test_order_retreat_and_done(self):
        mission = Mission(['RED', 'BLUE'], 0)
        self.step(mission, contacts={'red_box'})
        self.assertEqual(mission.target, 'BLUE')
        self.assertEqual(self.step(mission, now=2, front=.25)[0], -.04)
        self.assertEqual(self.step(mission, now=2, front=.25, rear=.1), (0, 0))
        self.assertEqual(self.step(mission, now=4), (0, 0))
        self.assertEqual(mission.state, 'SEARCH')
        self.step(mission, now=5, contacts={'blue_box'})
        self.assertEqual(mission.state, 'DONE')
        self.assertEqual(self.step(mission, now=6), (0, 0))

    def test_slow_simulation_retreat_waits_for_measured_clearance(self):
        mission = Mission(['RED', 'BLUE'], 0)
        self.step(mission, contacts={'red_box'})
        # Well past the old 2.5 s pulse, the contacted box is still too near.
        self.assertEqual(self.step(mission, now=8, front=.32, sides=.4), (-.04, 0))
        self.assertEqual(mission.state, 'RETREAT')
        self.assertEqual(self.step(mission, now=10, front=.46, sides=.5), (0, 0))
        self.assertEqual(mission.state, 'SEARCH')
        self.assertGreater(self.step(mission, now=11, bearing=.3)[1], 0)

    def test_retreat_blocked_or_blind_stops_and_has_deadline(self):
        mission = Mission(['RED', 'BLUE'], 0, automatic_retries=0)
        self.step(mission, contacts={'red_box'})
        for changes in ({'rear': .2}, {'rear': None}, {'fresh': False}):
            self.assertEqual(self.step(mission, now=5, front=.3, **changes), (0, 0))
        self.assertEqual(self.step(mission, now=47, front=.3, rear=.2), (0, 0))
        self.assertEqual(mission.state, 'FAILED')

    def test_wrong_or_ambiguous_contact_fails(self):
        for contacts in ({'blue_box'}, {'red_box', 'gray_obstacle'}):
            mission = Mission(['RED'], 0)
            self.step(mission, contacts=contacts)
            self.assertEqual(mission.state, 'FAILED')

    def test_repeat_timeout_and_validation(self):
        mission = Mission(['RED'], 0, repeat=True)
        self.step(mission, contacts={'red_box'})
        self.assertEqual(mission.state, 'RETREAT')
        mission = Mission(['RED'], 0, timeout=2)
        self.assertEqual(self.step(mission, now=3), (0, 0))
        self.assertEqual(mission.state, 'FAILED')
        for colors in ([], ['GREEN']):
            with self.assertRaises(ValueError):
                Mission(colors, 0)

    def test_alignment_search_and_obstacle(self):
        mission = Mission(['RED'], 0)
        self.assertGreater(self.step(mission, bearing=.3)[1], 0)
        self.assertLess(self.step(mission, bearing=-.3)[1], 0)
        self.assertEqual(self.step(mission, bearing=None)[0], 0)
        self.assertEqual(self.step(mission, front=.3, bearing=.4)[0], 0)
        self.assertEqual(mission.state, 'AVOID')


if __name__ == '__main__':
    unittest.main()
