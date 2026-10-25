"""Deterministic reactive controller. Distances are from the LiDAR origin."""
import math
from .avoidance import Recovery, OpenSpaceEscape, angle_difference


def sector(ranges, angle_min, increment, low, high, range_min, range_max):
    values = []
    for i, distance in enumerate(ranges):
        angle = math.atan2(math.sin(angle_min + i * increment),
                           math.cos(angle_min + i * increment))
        if low <= angle <= high:
            if math.isinf(distance) and distance > 0:
                values.append(range_max)
            elif math.isfinite(distance) and range_min <= distance <= range_max:
                values.append(distance)
            else:
                return None  # blind beam in a safety sector: fail closed
    return min(values) if values else None


class Mission:
    def __init__(self, targets, now, timeout=120.0, repeat=False,
                 automatic_retries=2, retry_wait=3.0, retry_window=30.0,
                 retry_confirm_scans=3, recovery_timeout=45.0):
        self.targets = [c.upper() for c in targets]
        if not self.targets or any(c not in ('RED', 'BLUE', 'YELLOW') for c in self.targets):
            raise ValueError('targets must contain RED, BLUE or YELLOW')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive')
        if not isinstance(automatic_retries, int) or not 0 <= automatic_retries <= 5:
            raise ValueError('automatic_retries must be an integer from 0 to 5')
        if not all(math.isfinite(v) and v > 0 for v in (retry_wait, retry_window)) or retry_window <= retry_wait:
            raise ValueError('retry_window must exceed positive retry_wait')
        if not isinstance(retry_confirm_scans, int) or not 2 <= retry_confirm_scans <= 10:
            raise ValueError('retry_confirm_scans must be an integer from 2 to 10')
        if not math.isfinite(recovery_timeout) or recovery_timeout <= 0:
            raise ValueError('recovery_timeout must be positive')
        self.index = 0
        self.started = now
        self.timeout = timeout
        self.repeat = repeat
        self.state = 'SEARCH'
        self.retreat_deadline = 0.0
        self.avoid_until = 0.0
        self.search_ticks = 0
        self.recovery = None
        self.recovery_count = 0
        self.relocation_count = 0
        self.recovery_triggers = []
        self.reason = 'starting'
        self.failure_reason = None
        self.progress_pose = None
        self.progress_at = now
        self.departure_origin = None
        self.departure_clear_scans = 0
        self.departure_scan_sequence = None
        self.retreat_origin = None
        self.automatic_retries = automatic_retries
        self.retry_wait = retry_wait
        self.retry_window = retry_window
        self.retry_count = 0
        self.retry_due = self.retry_deadline = 0.0
        self.retry_resume = None
        self.retry_cause = None
        self.last_contacted = None
        self.retry_confirm_scans = retry_confirm_scans
        self.retry_safe_scans = 0
        self.retry_scan_sequence = None
        self.recovery_timeout = recovery_timeout
        self.vision_epoch = 0
        self.completed_targets = 0
        self.laps = 0
        self.last_target_heading = None
        self.search_last_yaw = None
        self.search_turned = 0.0
        self.search_direction = 1
        self.explore_origin = None
        self.visual_losses = 0
        self.visual_loss_anchor = None
        self.camera_relocations = 0

    def reacquire(self, pose, reason, guard=None):
        self.vision_epoch += 1
        self.state = 'REACQUIRE' if guard is not None else 'SEARCH'
        self.reason = reason
        self.search_last_yaw = pose[2] if pose is not None else None
        self.search_turned = 0.0
        self.explore_origin = None
        if pose is not None and self.last_target_heading is not None:
            self.search_direction = 1 if angle_difference(self.last_target_heading, pose[2]) >= 0 else -1
        else:
            self.search_direction = 1
        return 0.0, 0.0

    def fail(self, reason):
        self.state = 'FAILED'
        self.reason = self.failure_reason = reason
        return 0.0, 0.0

    def wait_or_fail(self, now, reason, resume):
        if self.automatic_retries == 0:
            return self.fail(reason)
        if self.retry_count >= self.automatic_retries:
            return self.fail('automatic_retry_limit: ' + reason)
        self.retry_count += 1
        self.retry_due = now + self.retry_wait
        self.retry_deadline = now + self.retry_window
        self.retry_resume, self.retry_cause = resume, reason
        self.recovery = None
        self.retry_safe_scans = 0
        self.retry_scan_sequence = None
        self.state = 'WAIT_RETRY'
        self.reason = 'stopped_to_recheck_surroundings: ' + reason
        return 0.0, 0.0

    def recover(self, now, pose, guard, front, rear, left, right, reason):
        if pose is None or guard is None:
            self.reason = 'recovery_requires_odometry_and_scan'
            return 0.0, 0.0
        if self.recovery is None:
            if self.recovery_count >= 3:
                if self.automatic_retries == 0 or self.relocation_count >= 2:
                    return self.wait_or_fail(now, 'recovery_attempt_limit', 'RECOVER')
                self.relocation_count += 1
                self.recovery_count = 0
                self.recovery = OpenSpaceEscape(now, pose, timeout=self.recovery_timeout)
                self.recovery_triggers.append('repeated_blockage_open_space_escape')
                self.state = 'RECOVER_BACKUP'
                self.reason = 'repeated_blockage_scan_and_leave_local_area'
                return 0.0, 0.0
            self.recovery_count += 1
            self.recovery_triggers.append(reason)
            self.recovery_triggers = self.recovery_triggers[-12:]
            escape_only = reason in ('camera_scan_needs_turning_room', 'rotation_sweep_blocked')
            direction = 1 if left >= right else -1
            if self.last_target_heading is not None:
                goal_turn = angle_difference(self.last_target_heading, pose[2])
                if abs(goal_turn) > .15 and (left if goal_turn > 0 else right) > .35:
                    direction = 1 if goal_turn > 0 else -1
            self.recovery = Recovery(now, pose, direction,
                                     timeout=self.recovery_timeout, escape_only=escape_only)
        command = self.recovery.step(now, pose, guard, front, rear)
        self.state = 'RECOVER_' + self.recovery.stage
        self.reason = reason + ': ' + self.recovery.reason
        if self.recovery.failed:
            return self.wait_or_fail(now, self.recovery.reason, 'RECOVER')
        if self.recovery.done:
            self.recovery = None
            self.progress_at, self.progress_pose = now, pose
            return self.reacquire(pose, 'camera_reacquire_after_avoidance', guard)
        return command

    @property
    def target(self):
        return self.targets[min(self.index, len(self.targets) - 1)]

    def step(self, now, fresh, bearing, front, rear, sides, contacts,
             pose=None, guard=None, left=None, right=None, target_bounds=None):
        """Return linear and angular speed. Only named physical contact advances."""
        if self.state in ('DONE', 'FAILED'):
            return 0.0, 0.0
        if now - self.started > self.timeout:
            return self.fail('target_timeout')
        if self.state == 'WAIT_RETRY' and now >= self.retry_deadline:
            return self.fail('retry_no_safe_space: ' + self.retry_cause)
        if not fresh or any(v is None for v in (front, rear, sides)):
            self.retry_safe_scans = 0
            self.departure_clear_scans = 0
            self.reason = 'stale_sensor_or_unknown_clearance'
            return 0.0, 0.0
        if guard is not None and not guard.valid:
            self.retry_safe_scans = 0
            self.departure_clear_scans = 0
            self.reason = 'invalid_or_incomplete_scan'
            return 0.0, 0.0
        left = sides if left is None else left
        right = sides if right is None else right
        if self.state == 'WAIT_RETRY':
            allowed = {self.last_contacted} if self.retry_resume == 'RETREAT' and self.last_contacted else set()
            if contacts - allowed:
                return self.fail('unexpected_contact_while_waiting: ' + ','.join(sorted(contacts - allowed)))
            if now >= self.retry_deadline:
                return self.fail('retry_no_safe_space: ' + self.retry_cause)
            if now < self.retry_due:
                self.reason = 'retry_cooldown_rechecking_surroundings'
                return 0.0, 0.0
            can_escape = (pose is not None and guard is not None and
                          ((rear > .30 and guard.reverse_safe()) or
                           (self.retry_resume != 'RETREAT' and front > .45 and guard.forward_safe()) or
                           (not contacts and front > .35 and guard.turn_safe())))
            if not can_escape:
                self.retry_safe_scans = 0
                self.retry_scan_sequence = guard.sequence if guard is not None else None
                self.reason = 'retry_waiting_for_safe_escape_space'
                return 0.0, 0.0
            if guard.sequence != self.retry_scan_sequence:
                self.retry_safe_scans += 1
                self.retry_scan_sequence = guard.sequence
            if self.retry_safe_scans < self.retry_confirm_scans:
                self.reason = f'retry_confirming_space_{self.retry_safe_scans}_of_{self.retry_confirm_scans}'
                return 0.0, 0.0
            self.progress_at, self.progress_pose = now, pose
            if self.retry_resume == 'RETREAT':
                self.state = 'RETREAT'
                self.retreat_origin = pose
                self.departure_clear_scans = 0
                self.departure_scan_sequence = None
                self.retreat_deadline = now + 45.0
            else:
                # A new bounded batch is permitted only after fresh scans
                # confirm escape space. Preserve retry and target-time budgets.
                self.recovery_count = 0
                return self.reacquire(pose, 'retry_space_available_reacquire_current_color', guard)
            self.reason = 'retry_space_available_reconfirm_before_motion'
            # Keep this tick stopped. Node's SpaceCheck must confirm three
            # new scans again before any nonzero command is passed through.
            return 0.0, 0.0
        if self.state == 'RETREAT':
            if contacts - ({self.last_contacted} if self.last_contacted else set()):
                return self.fail('unexpected_contact_during_retreat')
            # A fixed wall-time pulse can move only a few centimetres when
            # Gazebo runs slowly. Require measured clearance before turning.
            clear = (guard.turn_safe(.34) and front >= .38) if guard is not None else (front >= .45 and sides >= .45)
            if guard is not None:
                # Reject a transient clear scan: the robot must actually
                # translate away from the contact before starting a turn.
                origin = self.departure_origin
                reversed_distance = (-(pose[0] - origin[0]) * math.cos(origin[2])
                                     -(pose[1] - origin[1]) * math.sin(origin[2])) if pose is not None and origin is not None else 0.0
                clear = clear and reversed_distance >= .20 and not contacts
                if not clear:
                    self.departure_clear_scans = 0
                    self.departure_scan_sequence = guard.sequence
                elif guard.sequence != self.departure_scan_sequence:
                    self.departure_clear_scans += 1
                    self.departure_scan_sequence = guard.sequence
                if clear:
                    if self.departure_clear_scans >= 3:
                        self.progress_at, self.progress_pose = now, pose
                        return self.reacquire(pose, 'stable_departure_confirmed_scan_next_color', guard)
                    # Hold still while collecting independent confirmation scans.
                    # Keep the normal retreat deadline active.
                    if now < self.retreat_deadline:
                        self.reason = 'confirming_stable_departure_space'
                        return 0.0, 0.0
            elif clear and not contacts:
                self.progress_at, self.progress_pose = now, pose
                return self.reacquire(pose, 'camera_scan_next_color_after_contact', guard)
            if now >= self.retreat_deadline:
                return self.wait_or_fail(now, 'post_contact_retreat_timeout', 'RETREAT')
            if pose is not None and self.retreat_origin is not None:
                moved = math.hypot(pose[0] - self.retreat_origin[0], pose[1] - self.retreat_origin[1])
                if moved >= .65:
                    return self.wait_or_fail(now, 'no_turning_room_after_contact_retreat', 'RETREAT')
            if rear > .3 and (guard is None or guard.reverse_safe()):
                self.reason = 'retreating_until_turning_room_available'
                return -.04, 0.0
            self.reason = 'contact_retreat_rear_path_blocked'
            return 0.0, 0.0
        expected = self.target.lower() + '_box'
        if contacts:
            if contacts == {expected} and bearing is not None and abs(bearing) < 0.15:
                self.relocation_count = 0
                self.completed_targets += 1
                self.index += 1
                if self.index == len(self.targets):
                    self.laps += 1
                    if not self.repeat:
                        self.state = 'DONE'
                        self.reason = 'mission_complete'
                        return 0.0, 0.0
                    self.index = 0
                self.started = now
                self.vision_epoch += 1
                self.last_target_heading = None
                self.visual_losses = 0
                self.visual_loss_anchor = None
                self.camera_relocations = 0
                self.departure_origin = pose
                self.departure_clear_scans = 0
                self.departure_scan_sequence = None
                self.last_contacted = expected
                self.retry_count = 0
                self.retry_cause = self.retry_resume = None
                self.retreat_deadline = now + 45.0
                self.retreat_origin = pose
                self.recovery = None
                self.recovery_count = 0
                self.state = 'RETREAT'
                self.reason = 'target_contact_confirmed'
            else:
                return self.fail('unexpected_contact: ' + ','.join(sorted(contacts)))
            return 0.0, 0.0
        if self.recovery is not None:
            return self.recover(now, pose, guard, front, rear, left, right, 'local_bypass')
        if bearing is not None and pose is not None:
            self.last_target_heading = pose[2] + bearing
        if pose is not None:
            if self.progress_pose is None or math.hypot(pose[0] - self.progress_pose[0], pose[1] - self.progress_pose[1]) >= .02 or abs(angle_difference(pose[2], self.progress_pose[2])) >= .1:
                self.progress_at, self.progress_pose = now, pose
            elif now - self.progress_at > 10:
                return self.recover(now, pose, guard, front, rear, left, right, 'no_measured_progress')
        if guard is not None:
            if bearing is None:
                if self.state not in ('REACQUIRE', 'EXPLORE'):
                    if self.state in ('ALIGN', 'APPROACH') and pose is not None:
                        if self.visual_loss_anchor is None or math.hypot(
                                pose[0] - self.visual_loss_anchor[0],
                                pose[1] - self.visual_loss_anchor[1]) > .10:
                            self.visual_losses = 0
                            self.visual_loss_anchor = pose
                        self.visual_losses += 1
                        if self.visual_losses >= 2:
                            self.visual_losses = 0
                            self.camera_relocations += 1
                            return self.recover(now, pose, guard, front, rear, left, right,
                                                'camera_target_repeatedly_occluded')
                    return self.reacquire(pose, 'camera_scan_current_target', guard)
                if self.state == 'EXPLORE':
                    moved = math.hypot(pose[0] - self.explore_origin[0], pose[1] - self.explore_origin[1])
                    if moved >= .25:
                        return self.reacquire(pose, 'camera_scan_after_exploration', guard)
                    if front > .55 and guard.forward_safe():
                        self.reason = 'explore_then_scan_for_current_color'
                        return .04, 0.0
                    return self.recover(now, pose, guard, front, rear, left, right,
                                        'exploration_corridor_blocked')
                if not guard.turn_safe():
                    return self.recover(now, pose, guard, front, rear, left, right,
                                        'camera_scan_needs_turning_room')
                if pose is not None and self.search_last_yaw is not None:
                    self.search_turned = max(0.0, self.search_turned + self.search_direction *
                                             angle_difference(pose[2], self.search_last_yaw))
                    self.search_last_yaw = pose[2]
                if self.search_turned >= math.tau:
                    if front > .55 and guard.forward_safe():
                        self.state = 'EXPLORE'
                        self.explore_origin = pose
                        self.reason = 'full_camera_turn_completed_explore_clear_space'
                        return 0.0, 0.0
                    return self.recover(now, pose, guard, front, rear, left, right,
                                        'no_target_after_full_camera_turn')
                self.reason = 'rotate_camera_until_current_color_confirmed'
                return 0.0, .25 * self.search_direction
            if abs(bearing) > .08:
                if not guard.alignment_safe(bearing, target_bounds):
                    return self.recover(now, pose, guard, front, rear, left, right,
                                        'rotation_sweep_blocked')
                self.state = 'ALIGN'
                self.reason = 'confirmed_current_color_center_camera'
                return 0.0, max(-.35, min(.35, 1.2 * bearing))
            # A nearby object outside the actual forward path is not a reason
            # to abandon a confirmed target. Still block other corridor points.
            if front < .12 or not guard.approach_safe(target_bounds):
                return self.recover(now, pose, guard, front, rear, left, right,
                                    'obstacle_in_confirmed_target_route')
            self.state = 'APPROACH'
            self.reason = 'confirmed_current_color_approach_until_physical_contact'
            return (.015 if front < .38 else .08), 0.0
        # Rotation is permitted only with clearance around the whole body.
        if sides < 0.20 or rear < 0.20:
            return self.recover(now, pose, guard, front, rear, left, right, 'side_or_rear_clearance')
        if guard is not None and not guard.turn_safe() and (bearing is None or abs(bearing) > .08):
            return self.recover(now, pose, guard, front, rear, left, right, 'rotation_sweep_blocked')
        if now < self.avoid_until:
            self.state = 'AVOID'
            return (0.0, 0.25) if front > 0.20 else (0.0, 0.0)
        if front < 0.45 and (bearing is None or abs(bearing) > 0.15):
            if guard is not None:
                return self.recover(now, pose, guard, front, rear, left, right, 'obstacle_in_front')
            self.avoid_until = now + 2.5
            self.state = 'AVOID'
            return (0.0, 0.25) if front > 0.20 else (0.0, 0.0)
        if bearing is None:
            self.state = 'SEARCH'
            # Scan a full turn, then cautiously explore a short clear corridor.
            self.search_ticks = (self.search_ticks + 1) % 620
            if self.search_ticks > 500 and front > 0.6 and sides > 0.3:
                if guard is None or guard.forward_safe():
                    self.reason = 'explore_clear_corridor'
                    return 0.04, 0.0
            self.reason = 'search_for_color'
            return (0.0, 0.25) if front > 0.20 else (0.0, 0.0)
        if abs(bearing) > 0.08:
            self.state = 'ALIGN'
            self.reason = 'center_target'
            return (0.0, max(-0.35, min(0.35, 1.2 * bearing))) if front > 0.20 else (0.0, 0.0)
        self.state = 'APPROACH'
        self.reason = 'approach_centered_target'
        # Low-speed touch is allowed only with a centered visual target.
        # Modified demo LiDAR min range and bumper geometry cover this region.
        if front < 0.12:
            return self.recover(now, pose, guard, front, rear, left, right, 'front_too_close_without_contact')
        return (0.015 if front < 0.38 else 0.08), 0.0
