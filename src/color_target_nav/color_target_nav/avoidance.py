"""Conservative local recovery using LiDAR points and measured odometry.

Geometry matches burger_contact.sdf: LiDAR x=-0.032 m, rear body x=-0.12,
front bumper x=0.19, half width including wheels 0.10 m. These checks are
local sampled-path checks, not a global planner or a real robot safety system.
"""
import math


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ScanGuard:
    def __init__(self, ranges, angle_min, increment, range_min, range_max, sequence=0):
        self.sequence = sequence
        self.points = []
        names = ('front', 'front_left', 'left', 'rear_left', 'rear',
                 'rear_right', 'right', 'front_right')
        bins = [[] for _ in names]
        self.clearance = dict.fromkeys(names)
        self.valid = (len(ranges) >= 180 and math.isfinite(angle_min)
                      and math.isfinite(increment) and 0 < increment <= .04
                      and 6.1 <= len(ranges) * increment <= 6.5
                      and math.isfinite(range_max) and range_max > range_min > 0)
        if not self.valid:
            return
        for i, distance in enumerate(ranges):
            angle = angle_min + i * increment
            index = int(math.floor((angle + math.pi / 8) / (math.pi / 4))) % 8
            if math.isinf(distance) and distance > 0:
                bins[index].append(range_max)
                continue
            if not math.isfinite(distance) or not range_min <= distance <= range_max:
                self.valid = False
                continue
            bins[index].append(distance)
            self.points.append((distance * math.cos(angle) - .032,
                                distance * math.sin(angle)))
        if self.valid:
            self.clearance = {name: min(values) if values else None
                              for name, values in zip(names, bins)}

    def reverse_safe(self):
        # Include the next 12 cm of travel plus the current body width.
        # The front face is excluded so retreat away from a touched box is
        # possible. Do not back through obstacles beside the body/rear.
        return self.valid and not any(-.36 <= x <= .10 and abs(y) <= .14
                                      for x, y in self.points)

    def turn_safe(self, clearance=.28):
        # The bumper sweeps a larger circle than the wheels/body.
        return self.valid and all(math.hypot(x, y) > clearance for x, y in self.points)

    def approach_safe(self, bounds=None):
        if not self.valid:
            return False
        low, high = bounds if bounds is not None else (-.18, .18)
        if not (math.isfinite(low) and math.isfinite(high) and low <= high):
            return False
        for x, y in self.points:
            if -.14 <= x <= .38 and abs(y) <= .14:
                angle = math.atan2(y, x - .05)
                # Intentional target touch is allowed only in the confirmed
                # color region. Other points in the vehicle path still block.
                if not (x >= .15 and low - .06 <= angle <= high + .06):
                    return False
        return True

    def alignment_safe(self, bearing, bounds=None):
        """Check a short camera correction, not an entire 360-degree turn.

        Near a confirmed goal a full-circle bumper envelope would block
        every final correction. Check the actual small swept rectangle.
        Only the front of that confirmed target can be intentionally touched.
        """
        if not self.valid or bearing is None or not math.isfinite(bearing):
            return False
        low, high = bounds if bounds is not None else (-.18, .18)
        if not (math.isfinite(low) and math.isfinite(high) and low <= high):
            return False
        sweep = max(-.25, min(.25, bearing))
        for x, y in self.points:
            target_point = (abs(bearing) < .15 and x >= .15 and
                            low - .06 <= math.atan2(y, x - .05) <= high + .06)
            if target_point:
                continue
            for i in range(7):
                angle = sweep * i / 6
                px = x * math.cos(angle) + y * math.sin(angle)
                py = -x * math.sin(angle) + y * math.cos(angle)
                if -.14 <= px <= .22 and abs(py) <= .14:
                    return False
        return True

    def forward_safe(self):
        # Used only for bypass/exploration, never to infer target contact.
        return self.valid and not any(-.14 <= x <= .38 and abs(y) <= .14
                                      for x, y in self.points)

    def path_report(self):
        def nearest(predicate):
            points = sorted((p for p in self.points if predicate(*p)),
                            key=lambda p: math.hypot(*p))[:4]
            return [{'x': round(x, 3), 'y': round(y, 3)} for x, y in points]
        return {'scan_valid': self.valid, 'reverse_clear': self.reverse_safe(),
                'turn_clear': self.turn_safe(), 'forward_clear': self.forward_safe(),
                'reverse_blockers': nearest(lambda x, y: -.36 <= x <= .10 and abs(y) <= .14),
                'turn_blockers': nearest(lambda x, y: math.hypot(x, y) <= .28),
                'forward_blockers': nearest(lambda x, y: -.14 <= x <= .38 and abs(y) <= .14)}


class SpaceCheck:
    """Require distinct safe scans before an escape/turn/pass can start."""
    def __init__(self, required=3):
        if not isinstance(required, int) or not 2 <= required <= 10:
            raise ValueError('space_confirm_scans must be an integer from 2 to 10')
        self.required = required
        self.reset()

    def reset(self):
        self.motion = None
        self.sequence = None
        self.confirmed = 0
        self.reason = 'not_checking'

    def ready(self, guard, motion, bearing=None, bounds=None):
        if motion != self.motion:
            self.reset()
            self.motion = motion
        safe = (guard is not None and guard.valid and
                {'reverse': guard.reverse_safe, 'turn': guard.turn_safe,
                 'forward': guard.forward_safe,
                 'align': lambda: guard.alignment_safe(bearing, bounds)}[motion]())
        if not safe:
            self.confirmed = 0
            self.sequence = guard.sequence if guard is not None else None
            self.reason = 'space_blocked_or_unknown'
            return False
        if guard.sequence != self.sequence:
            self.confirmed = min(self.required, self.confirmed + 1)
            self.sequence = guard.sequence
        self.reason = 'space_confirmed' if self.confirmed >= self.required else 'waiting_for_distinct_safe_scans'
        return self.confirmed >= self.required


class Recovery:
    """Back away, turn toward the clearer side, pass, then reacquire color."""
    def __init__(self, now, pose, direction, timeout=45.0, escape_only=False):
        self.started = now
        self.deadline = now + timeout
        self.origin = pose
        self.stage = 'BACKUP'
        self.direction = 1 if direction >= 0 else -1
        self.reason = 'make_room_before_turn'
        self.done = False
        self.failed = False
        self.escape_only = escape_only
        self.turn_origin = None

    def space_ready(self, pose):
        self.origin = pose
        if self.escape_only:
            self.done = True
            self.reason = 'room_ready_for_camera_reacquisition'
        else:
            self.stage = 'TURN'
            if self.turn_origin is None:
                self.turn_origin = pose
            self.reason = 'turn_toward_clearer_side'
        return 0.0, 0.0

    def step(self, now, pose, guard, front, rear):
        if now >= self.deadline:
            self.failed = True
            self.reason = 'recovery_timeout'
            return 0.0, 0.0
        if not guard.valid:
            self.reason = 'invalid_scan'
            return 0.0, 0.0
        travelled = math.hypot(pose[0] - self.origin[0], pose[1] - self.origin[1])
        if self.stage == 'BACKUP':
            if guard.turn_safe(.34) and front > .35:
                return self.space_ready(pose)
            if travelled >= .65:
                self.failed = True
                self.reason = 'no_turning_room_after_backup'
                return 0.0, 0.0
            if rear > .30 and guard.reverse_safe():
                self.reason = 'backing_away_from_obstacle'
                return -.04, 0.0
            if front > .45 and guard.forward_safe():
                self.stage = 'FORWARD'
                self.origin = pose
                self.reason = 'rear_blocked_make_turning_room_ahead'
                return 0.0, 0.0
            self.reason = 'rear_or_swept_path_blocked'
            return 0.0, 0.0
        if self.stage == 'FORWARD':
            # Persist with the chosen escape so noisy rear measurements do
            # not switch the command between forward/backward every scan.
            if guard.turn_safe(.34) and front > .35:
                return self.space_ready(pose)
            if travelled >= .65:
                self.failed = True
                self.reason = 'no_turning_room_after_forward_escape'
                return 0.0, 0.0
            if front > .45 and guard.forward_safe():
                self.reason = 'moving_ahead_to_make_turning_room'
                return .04, 0.0
            self.stage = 'BACKUP'
            self.origin = pose
            self.reason = 'forward_escape_path_blocked_recheck_all_paths'
            return 0.0, 0.0
        if self.stage == 'TURN':
            turned = self.direction * angle_difference(pose[2], self.turn_origin[2])
            if turned >= 1.4:
                self.stage = 'PASS'
                self.origin = pose
                self.reason = 'pass_obstacle'
                return 0.0, 0.0
            if not guard.turn_safe():
                self.stage = 'BACKUP'
                self.origin = pose
                self.reason = 'turn_sweep_blocked'
                return 0.0, 0.0
            return 0.0, .25 * self.direction
        if travelled >= .25 and guard.turn_safe(.34):
            self.done = True
            self.reason = 'turning_room_ready_reacquire_target'
            return 0.0, 0.0
        if travelled >= .90:
            self.failed = True
            self.reason = 'no_camera_turning_room_after_pass'
            return 0.0, 0.0
        if front > .45 and guard.forward_safe():
            return .04, 0.0
        # Replan within the same deadline instead of repeatedly turning into
        # a blocked corridor. Retain direction to avoid left/right oscillation.
        self.stage = 'BACKUP'
        self.origin = pose
        self.turn_origin = None
        self.reason = 'bypass_path_blocked'
        return 0.0, 0.0


class OpenSpaceEscape(Recovery):
    """Leave a repeated local deadlock before reacquiring the same color.

    Select a .8m sampled corridor, turn only with a clear swept body,
    then translate .5m while continuously checking the actual forward path.
    """
    def __init__(self, now, pose, timeout=90.0):
        super().__init__(now, pose, 1, timeout=timeout)
        self.heading = None

    def space_ready(self, pose):
        self.stage = 'SELECT'
        self.origin = pose
        self.heading = None
        self.reason = 'scan_open_corridors_before_escape'
        return 0.0, 0.0

    @staticmethod
    def choose_direction(guard):
        if not guard.valid:
            return None
        candidates = []
        for i in range(16):
            angle = angle_difference(i * math.tau / 16, 0)
            # Leave the original blocked heading instead of selecting it again.
            if abs(angle) < math.pi / 4:
                continue
            c, s = math.cos(angle), math.sin(angle)
            distances = [x*c+y*s for x, y in guard.points
                         if x*c+y*s > 0 and abs(-x*s+y*c) <= .18]
            clearance = min(distances, default=3.5)
            if clearance >= .80:
                candidates.append((min(clearance, 1.5), -abs(angle), angle))
        return max(candidates)[2] if candidates else None

    def step(self, now, pose, guard, front, rear):
        if now >= self.deadline or not guard.valid:
            return super().step(now, pose, guard, front, rear)
        if self.stage in ('BACKUP', 'FORWARD'):
            return super().step(now, pose, guard, front, rear)
        if self.stage == 'SELECT':
            if not guard.turn_safe(.34):
                self.stage = 'BACKUP'
                self.origin = pose
                self.reason = 'escape_needs_turning_room'
                return 0.0, 0.0
            angle = self.choose_direction(guard)
            if angle is None:
                self.reason = 'no_open_escape_corridor_rechecking'
                return 0.0, 0.0
            self.heading = pose[2] + angle
            self.stage = 'ESCAPE_TURN'
            self.reason = 'turn_to_lidar_selected_open_corridor'
            return 0.0, 0.0
        if self.stage == 'ESCAPE_TURN':
            if not guard.turn_safe(.28):
                self.stage = 'BACKUP'
                self.origin = pose
                self.reason = 'escape_turn_blocked_recheck'
                return 0.0, 0.0
            error = angle_difference(self.heading, pose[2])
            if abs(error) <= .08:
                self.stage = 'ESCAPE_PASS'
                self.origin = pose
                self.reason = 'move_out_to_open_space'
                return 0.0, 0.0
            return 0.0, max(-.25, min(.25, error))
        travelled = math.hypot(pose[0]-self.origin[0], pose[1]-self.origin[1])
        if travelled >= .50 and guard.turn_safe(.34):
            self.done = True
            self.reason = 'open_space_reached_reacquire_current_color'
            return 0.0, 0.0
        if travelled >= .75:
            self.failed = True
            self.reason = 'escape_distance_limit_without_turning_room'
            return 0.0, 0.0
        if front > .45 and guard.forward_safe():
            return .04, 0.0
        self.stage = 'BACKUP'
        self.origin = pose
        self.reason = 'escape_corridor_changed_recheck'
        return 0.0, 0.0
