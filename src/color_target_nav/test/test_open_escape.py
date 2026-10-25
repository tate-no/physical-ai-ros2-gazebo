import math
import unittest
from color_target_nav.avoidance import OpenSpaceEscape
from color_target_nav.control import Mission
from test_avoidance import scan_scene

class OpenEscapeTests(unittest.TestCase):
    def test_three_recoveries_selects_relocation_without_changing_target(self):
        m = Mission(['YELLOW'], 0, timeout=300)
        m.recovery_count = 3
        pose = (0,0,0)
        g,f,b,l,r = scan_scene(pose)
        self.assertEqual(m.recover(1,pose,g,f,b,l,r,'blocked'), (0,0))
        self.assertIsInstance(m.recovery, OpenSpaceEscape)
        self.assertEqual(m.relocation_count,1)
        self.assertEqual(m.target,'YELLOW')
        self.assertEqual(m.started,0)

    def test_closed_scene_never_selects_fake_open_corridor(self):
        circles = [(math.cos(i*math.tau/16)*.5,math.sin(i*math.tau/16)*.5,.15) for i in range(16)]
        g,*_ = scan_scene((0,0,0),circles)
        self.assertIsNone(OpenSpaceEscape.choose_direction(g))

    def test_escape_moves_out_then_reacquires_in_synthetic_scene(self):
        pose = (0.,0.,0.)
        objects = [(1.,0.,.2),(.6,.6,.2)]
        recovery = OpenSpaceEscape(0,pose,timeout=90)
        distance = 0
        for i in range(1800):
            g,f,b,l,r = scan_scene(pose,objects)
            v,w = recovery.step(i*.05,pose,g,f,b)
            if v:
                self.assertTrue(g.forward_safe() if v>0 else g.reverse_safe())
            if w:
                self.assertTrue(g.turn_safe())
            pose=(pose[0]+v*math.cos(pose[2])*.05,pose[1]+v*math.sin(pose[2])*.05,pose[2]+w*.05)
            distance += abs(v)*.05
            if recovery.done or recovery.failed:
                break
        self.assertTrue(recovery.done, recovery.reason)
        self.assertGreaterEqual(distance,.5)

    def test_blocked_turn_and_deadline_never_force_motion(self):
        escape=OpenSpaceEscape(0,(0,0,0),timeout=2)
        escape.stage='ESCAPE_TURN'
        escape.heading=1.
        g,f,b,*_ = scan_scene((0,0,0),[(.15,.1,.05)])
        self.assertEqual(escape.step(1,(0,0,0),g,f,b),(0,0))
        self.assertEqual(escape.stage,'BACKUP')
        self.assertEqual(escape.step(3,(0,0,0),g,f,b),(0,0))
        self.assertTrue(escape.failed)

    def test_escape_completion_returns_to_same_color_with_new_vision_epoch(self):
        m=Mission(['YELLOW'],0,timeout=300)
        m.recovery_count=3
        pose=(0,0,0)
        g,f,b,l,r=scan_scene(pose)
        m.recover(1,pose,g,f,b,l,r,'blocked')
        m.recovery.done=True
        m.recover(2,pose,g,f,b,l,r,'local_bypass')
        self.assertEqual(m.state,'REACQUIRE')
        self.assertEqual(m.target,'YELLOW')
        self.assertEqual(m.completed_targets,0)
        self.assertGreater(m.vision_epoch,0)

    def test_relocation_budget_does_not_enable_unbounded_escape(self):
        m=Mission(['YELLOW'],0,timeout=300)
        m.recovery_count=3
        m.relocation_count=2
        pose=(0,0,0)
        g,f,b,l,r=scan_scene(pose)
        m.recover(1,pose,g,f,b,l,r,'blocked')
        self.assertEqual(m.state,'WAIT_RETRY')
        self.assertEqual(m.relocation_count,2)
        self.assertEqual(m.started,0)
