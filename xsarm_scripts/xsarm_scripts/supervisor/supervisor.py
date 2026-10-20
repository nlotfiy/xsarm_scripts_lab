"""
Safety supervisor for the Interbotix arm: the single motion entry point for student code.

`SupervisedManipulator` replaces `InterbotixManipulatorXS` in a script. Its `arm` attribute
is an `ArmSupervisor`, which offers the same motion calls as `InterbotixArmXSInterface` but
sends every one of them through `ArmSupervisor._command`, and raises `MotionRejected` instead
of returning `False` when a command is refused.

See docs/supervisor_steps.md for the checks that later tasks add to `_command`.
"""

import math
import os
import socket
import threading
import time

from ament_index_python.packages import get_package_share_directory
import interbotix_common_modules.angle_manipulation as ang
from interbotix_common_modules.common_robot.robot import interbotix_is_up
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointGroupCommand
import modern_robotics as mr
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from xsarm_scripts.supervisor.collision import SelfCollisionModel
from xsarm_scripts.supervisor.envelope import (
    checked_envelope, envelope_path, excess, load_envelope, read_station_message)
from xsarm_scripts.supervisor.geometry import generate_urdf, HELD, RobotGeometry
from xsarm_scripts.supervisor.ik import AnalyticIK, Unreachable
import yaml

# The API refused the command without saying why (task 1.5 replaces this with named causes).
API_REJECT_UNKNOWN = 'API_REJECT_UNKNOWN'
# A blocking move returned and the measured joints were not at the commanded positions.
SUP_NOT_ARRIVED = 'SUP_NOT_ARRIVED'
# The measured joint positions are missing, old, or not numbers, so the arm cannot be checked.
SUP_NO_FEEDBACK = 'SUP_NO_FEEDBACK'
# A target or waypoint is not a finite number.
SUP_INVALID_TARGET = 'SUP_INVALID_TARGET'
# A joint target or waypoint is outside the enforced position limits.
SUP_POSITION_LIMIT = 'SUP_POSITION_LIMIT'
# The move would exceed the enforced joint velocity limit at the peak of its profile.
SUP_VELOCITY_LIMIT = 'SUP_VELOCITY_LIMIT'
# No joint solution within the limits exists for the requested pose or path.
SUP_NO_SOLUTION = 'SUP_NO_SOLUTION'
# During a move the measured joints left the band the commanded motion should keep them in.
SUP_FOLLOWING_ERROR = 'SUP_FOLLOWING_ERROR'
# During a move the measured arm came closer to the table than the measured margin.
SUP_MEASURED_CLEARANCE = 'SUP_MEASURED_CLEARANCE'
# The move would bring the arm closer to the table than the margin allows.
SUP_TABLE_CLEARANCE = 'SUP_TABLE_CLEARANCE'
# The move would bring two links of the arm closer to each other than the margin allows.
SUP_SELF_COLLISION = 'SUP_SELF_COLLISION'

# The move would take a link of the arm outside the work envelope.
SUP_WORK_ENVELOPE = 'SUP_WORK_ENVELOPE'

# The arm is in its sleep pose and was asked to go anywhere but the home pose.
SUP_SLEEP_EXIT = 'SUP_SLEEP_EXIT'
# The software e-stop is pressed: the arm holds and no motion is accepted until it is reset.
SUP_ESTOP = 'SUP_ESTOP'
# During a move a motor drew more current than its limit, for several readings in a row.
SUP_OVERCURRENT = 'SUP_OVERCURRENT'
# During a move the measured arm was outside the work envelope.
SUP_MEASURED_ENVELOPE = 'SUP_MEASURED_ENVELOPE'

# An unchecked move was asked for while the driver is not the simulator.
SUP_NOT_SIMULATED = 'SUP_NOT_SIMULATED'

# The software e-stop: `ros2 run xsarm_scripts estop` publishes its state on this topic.
ESTOP_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def estop_topic(robot_name):
    """Return the topic the software e-stop's state is published on."""
    return f'/{robot_name}/supervisor/estop'


# The work envelope of the arm's computer: `envelope_server` (started by arm.launch.py)
# publishes it, kept for late subscribers like the e-stop's state.
ENVELOPE_QOS = ESTOP_QOS


def envelope_topic(robot_name):
    """Return the topic the arm's computer publishes its work envelope on."""
    return f'/{robot_name}/supervisor/work_envelope'


# Weight of each joint's travel when choosing among the joint solutions of a pose: the joints
# that carry the arm count for more than the wrist.
BRANCH_WEIGHTS = (2.0, 2.0, 2.0, 1.0, 1.0, 1.0)
# A solution is charged for every joint closer than this [rad] to one of its limits.
BRANCH_LIMIT_MARGIN = 0.2

# Times a Cartesian path is planned again over a longer time to bring it under the speed limit.
CARTESIAN_RETIMES = 4

# Fewest points checked along a joint-space move, which is a straight line in joint space
# because all joints share one time-based profile. With the arm's geometry known, more are
# checked wherever the arm moves further than `path_step` between them.
PATH_SAMPLES = 10


def profile_fraction(t, moving_time, accel_time):
    """
    Return how far along a move the motors' time-based profile is after `t` seconds.

    The profile accelerates for `accel_time`, cruises, and decelerates for `accel_time`,
    finishing at `moving_time`; `accel_time` is at most half of `moving_time`.

    :return: fraction of the distance covered, from 0 to 1
    """
    if t <= 0.0:
        return 0.0
    if t >= moving_time:
        return 1.0
    if accel_time <= 0.0:
        return t / moving_time
    cruise = 1.0 / (moving_time - accel_time)
    if t < accel_time:
        return 0.5 * cruise * t * t / accel_time
    if t <= moving_time - accel_time:
        return cruise * (t - 0.5 * accel_time)
    return 1.0 - 0.5 * cruise * (moving_time - t) ** 2 / accel_time


class MotionRejected(Exception):
    """
    A motion command was refused, or the arm did not reach where it was commanded.

    When a command is refused nothing is sent to the arm for it. When the arm did not
    arrive (`SUP_NOT_ARRIVED`), it has been commanded to hold where it was measured. When
    the joint feedback is lost (`SUP_NO_FEEDBACK`), nothing more is sent at all and the
    supervisor refuses every later command; restart the script once the driver is back.
    """

    def __init__(self, code, call, detail=None):
        """
        Describe the refused command.

        :param code: reason code (see sim_harness/reason_codes.py)
        :param call: name of the supervisor method that was called
        :param detail: (optional) dict with the arguments or values behind the refusal
        """
        self.code = code
        self.call = call
        self.detail = dict(detail or {})
        super().__init__(f'{call} rejected ({code}): {self.detail}')


class ArmSupervisor:
    """Wrap an `InterbotixArmXSInterface` so that every motion command passes one gate."""

    # Calls on the wrapped arm that do not move it; anything else is not reachable from here.
    _READ_ONLY = (
        'get_joint_commands', 'get_single_joint_command', 'get_ee_pose_command', 'get_ee_pose',
        'get_joint_positions', 'get_joint_velocities', 'get_joint_efforts',
        'get_number_of_joints',
    )

    def __init__(
            self, arm, arrival_tolerance=0.05, settle_timeout=0.5, feedback_timeout=0.5,
            geometry=None, table_z=0.0, table_margin=0.025, limit_floor=None,
            monitor_period=0.02, following_tolerance=0.1, following_lag=0.25,
            measured_margin=0.005, self_collision=None, self_margin=0.010,
            work_envelope=None, envelope_tolerance=0.020, ik=None, path_step=0.010,
            current_readings=3):
        """
        Construct the supervisor.

        :param arm: the `InterbotixArmXSInterface` to supervise
        :param arrival_tolerance: (optional) largest difference [rad] between a commanded and
            a measured joint position for a blocking move to count as completed. The default
            is twice the largest settled error measured on the real vx300s (0.0245 rad, elbow,
            at the home pose), so it detects a move that did not complete, not gravity droop
        :param settle_timeout: (optional) time [s] the arm is given, after a blocking move
            returns, to come within `arrival_tolerance`
        :param feedback_timeout: (optional) age [s] beyond which the latest joint state
            message counts as lost feedback
        :param geometry: (optional) `RobotGeometry` of the arm; without it the table is not
            checked
        :param table_z: (optional) height [m] of the table surface in the arm's base frame
        :param table_margin: (optional) smallest distance [m] allowed between the table and
            the lowest point of the commanded arm. It has to cover what the command does not
            show: the real vx300s settles up to 16 mm low at 0.4 m reach (finding F15)
        :param limit_floor: (optional) limits enforced on top of those the driver reports:
            `{'position': {joint: [lower, upper]}, 'velocity': rad_per_s}`; the tighter of
            the two applies. It may also give `'current_limit_ma': {joint: mA}`: a move is
            stopped when that joint's motor draws more for `current_readings` readings in a
            row. Joints without a current limit are not watched
        :param monitor_period: (optional) time [s] between checks of the measured arm while
            a blocking joint move is in progress; `None` switches that monitoring off
        :param following_tolerance: (optional) distance [rad] a measured joint may be outside
            the band between where the command is now and where it was `following_lag` ago
        :param following_lag: (optional) longest delay [s] expected between the commanded
            motion and the measured one
        :param measured_margin: (optional) clearance [m] the measured arm must keep from
            the table. A move in progress is stopped when the measured arm is lower than
            the command explains by more than `table_margin - measured_margin`, which keeps
            it this far from the table as long as the command respects `table_margin`
        :param self_collision: (optional) `SelfCollisionModel` of the arm; without it the
            arm is not checked against itself
        :param self_margin: (optional) smallest distance [m] allowed between the capsules of
            two links of the commanded arm. The capsules already stand at least 6 mm off the
            link meshes wherever two links come close, so the links stay further apart
        :param work_envelope: (optional) box in the base frame that the moving links of the
            commanded arm must stay inside: `{'min': [x, y, z], 'max': [x, y, z]}` in metres;
            needs `geometry`
        :param envelope_tolerance: (optional) distance [m] the measured arm may be outside
            the work envelope during a move before the move is stopped. The command is held
            to the envelope exactly; this allows for the arm not being quite where it is
            commanded (up to 16 mm on the real vx300s, finding F15)
        :param ik: (optional) `AnalyticIK` of the arm. With it, pose commands and Cartesian
            moves are solved in closed form and the joint solution is chosen here; without
            it they use the arm interface's own numerical solver
        :param current_readings: (optional) readings in a row over the current limit that
            stop a move: one reading every `monitor_period` during a joint move, one per
            waypoint during a Cartesian move, so that a brief peak does not stop it
        :param path_step: (optional) farthest [m] any point of the arm may travel between
            two poses at which a path is checked. Two links that are `self_margin` apart at
            one checked pose can close by no more than this before the next, so with the
            default (equal to `self_margin`) their capsules cannot meet in between
        """
        self._arm = arm
        self._monitor_period = monitor_period
        self._following_tolerance = following_tolerance
        self._following_lag = following_lag
        self._measured_margin = measured_margin
        info = arm.group_info
        floor = limit_floor or {}
        position = floor.get('position', {})
        self._lower = [max(lo, position.get(name, [-math.inf, math.inf])[0])
                       for name, lo in zip(info.joint_names, info.joint_lower_limits)]
        self._upper = [min(hi, position.get(name, [-math.inf, math.inf])[1])
                       for name, hi in zip(info.joint_names, info.joint_upper_limits)]
        self._speed = [min(v, floor.get('velocity', math.inf))
                       for v in info.joint_velocity_limits]
        currents = floor.get('current_limit_ma') or {}
        self._current_limit = [currents.get(name) for name in info.joint_names]
        self._current_readings = current_readings
        self._over_current = [0] * info.num_joints   # readings in a row over the limit
        self._estopped = False   # set from the e-stop topic, see `set_estop`
        self._geometry = geometry
        self._table_z = table_z
        self._table_margin = table_margin
        self._self_collision = self_collision
        self._self_margin = self_margin
        self._work_envelope = None
        if work_envelope is not None and geometry is not None:
            self._work_envelope = checked_envelope(work_envelope)
        # a new envelope from `set_work_envelope`, as a 1-tuple, taken up at the next command
        self._next_envelope = None
        self._envelope_lock = threading.Lock()
        self._envelope_tolerance = envelope_tolerance
        self._ik = ik
        self._held = None  # object in the gripper, see `attach_object`
        self._path_step = path_step
        self._radii = None  # reach of the arm about each joint, see `_between`
        self._arrival_tolerance = arrival_tolerance
        self._settle_timeout = settle_timeout
        self._feedback_timeout = feedback_timeout
        self._fault = None  # reason code once feedback has been lost; never cleared
        self._rest_offset = [0.0] * info.num_joints  # commanded minus measured, arm at rest
        # The Interbotix arm assumes the motors' profile registers already hold its timing
        # and does not write them at start-up. A script that died during a Cartesian move
        # leaves them at the waypoint timing (0.2 s), so write them here.
        moving_time, accel_time = arm.moving_time, arm.accel_time
        arm.moving_time = arm.accel_time = None
        arm.set_trajectory_time(moving_time, accel_time)
        # the moving time a move gets when it is not given one; `set_trajectory_time` changes it
        self._default_moving_time = moving_time
        self._logger = arm.core.get_node().get_logger()

    def __getattr__(self, name):
        if name in self._READ_ONLY:
            return getattr(self._arm, name)
        raise AttributeError(f"'{name}' is not available through the arm supervisor")

    @property
    def group_info(self):
        """Joint names, limits and sleep positions reported by the driver."""
        return self._arm.group_info

    @property
    def T_sb(self):
        """Commanded end-effector pose in the base frame (4x4 matrix, a copy)."""
        return np.array(self._arm.T_sb)

    @property
    def moving_time(self):
        """Duration [s] of the next move."""
        return self._arm.moving_time

    @property
    def accel_time(self):
        """Acceleration/deceleration time [s] of the next move."""
        return self._arm.accel_time

    def _command(self, call, method, target=None, verified_path=False, **kwargs):
        """
        Send one motion command to the wrapped arm; every public motion method ends here.

        :param call: name of the public method, for logging and for `MotionRejected`
        :param method: bound method of the wrapped arm that carries out the command
        :param target: (optional) joint positions the command moves to in a straight line in
            joint space; checked against the table and the arm itself before anything is
            sent
        :param verified_path: (optional) the move to `target` is known to be free of
            self-collision, so that check is skipped
        :param kwargs: arguments for `method`
        :return: the joint solution for the pose commands, otherwise `None`
        :raises MotionRejected: if the wrapped arm refuses the command, or if the arm is not
            at the commanded position after a blocking move
        """
        self._logger.debug(f'{call}: {kwargs}')
        if self._fault is not None:
            raise MotionRejected(self._fault, call, {'reason': 'feedback was lost earlier'})
        executes = kwargs.get('execute', True)
        if executes and self._estopped:
            raise self._refuse(SUP_ESTOP, call, {'reason': 'the e-stop is pressed'})
        blocking = executes and kwargs.get('blocking', True)
        # a blocking joint move is sent without blocking and watched from here instead
        watched = blocking and target is not None
        if executes:
            measured = self._require_feedback(call)
            start = self._arm.get_joint_commands()
            limit = self._arrival_tolerance
            self._rest_offset = [
                max(-limit, min(limit, c - m)) for c, m in zip(start, measured)]
            if target is not None:
                self._check_sleep_exit(call, start, target)
                asked = moving_time = kwargs.get('moving_time') or self._arm.moving_time
                ramp = kwargs.get('accel_time') or self._arm.accel_time
                needed = self._time_needed(start, target, ramp)
                if needed > moving_time:
                    # too fast for the speed limit: take the time the limit allows instead
                    self._logger.info(f'{call}: slowed to the speed limit, moving time '
                                      f'{moving_time:.2f} s -> {needed:.2f} s')
                    moving_time = needed
                    kwargs = dict(kwargs, moving_time=needed)
                    # arm.py judges the move by the time of the move before it
                    self._arm.set_trajectory_time(moving_time=needed)
                accel_time = min(ramp, moving_time / 2.0)
                self._check_move(call, start, target, moving_time, accel_time, verified_path)
        result = method(**(dict(kwargs, blocking=False) if watched else kwargs))
        solution, accepted = result if isinstance(result, tuple) else (None, result)
        if accepted is False:
            self._logger.error(f'{call} rejected: {kwargs}')
            raise MotionRejected(API_REJECT_UNKNOWN, call, kwargs)
        if watched:
            self._watch_move(call, measured, start, target, moving_time, accel_time)
            if moving_time > asked and self._fault is None:
                # The move is over. The time that was asked for was too short, so it is not
                # kept for the moves that follow: they get the default again. After a move
                # that was not waited for, the longer time is left in place: writing a
                # shorter one could reach the motors before the move's command does.
                self._arm.set_trajectory_time(moving_time=self._default_moving_time)
        if blocking:
            self._check_arrival(call)
        return None if solution is None else [float(q) for q in solution]

    def _check_move(self, call, start, target, moving_time, accel_time, verified_path=False):
        """
        Check a joint-space move before it is sent: limits, speed, table, envelope, itself.

        :raises MotionRejected: with the code of the first check that fails
        """
        self._check_positions(call, target)
        # all joints share one time-based profile: the peak speed is the distance over the
        # time not spent accelerating
        self._check_speeds(call, start, target, moving_time - accel_time)
        path = self._between(start, target, PATH_SAMPLES)
        self._check_table(call, path)
        self._check_envelope(call, path)
        if not verified_path:
            self._check_self_collision(call, path)

    def _between(self, start, target, fewest=1):
        """
        Return the poses at which a straight joint-space move is checked, ending at `target`.

        There are enough of them that no point of the arm, nor of a held object, travels
        more than `path_step` from one to the next: turning joint i by an angle moves a point
        by at most that angle times the reach of the arm about the joint.

        :param fewest: smallest number of poses to return
        :return: list of joint position lists, without `start`
        """
        count = fewest
        if self._path_step and hasattr(self._geometry, 'reach_radii'):
            if self._radii is None:
                self._radii = self._geometry.reach_radii(
                    self._arm.group_info.joint_names, self._held)
            travel = sum(r * abs(t - s) for r, s, t in zip(self._radii, start, target))
            count = max(count, math.ceil(travel / self._path_step))
        return [[s + (t - s) * k / count for s, t in zip(start, target)]
                for k in range(1, count + 1)]

    def _hold(self, measured):
        """
        Command the arm to stay where it was just measured.

        A joint holding a load sits a little off its command, because that error is what
        produces the holding torque. Commanding exactly the measured position would remove
        the torque and let the arm sag further, so the command keeps the offset the arm had
        at rest before the move.
        """
        self._arm._publish_commands(
            positions=[m + o for m, o in zip(measured, self._rest_offset)], blocking=False)

    def _watch_move(self, call, measured, start, target, moving_time, accel_time):
        """
        Wait for a joint move to finish, stopping it if the measured arm goes wrong.

        :param measured: measured joint positions when the move was commanded
        :param start: commanded joint positions before the move
        :param target: commanded joint positions of the move
        :raises MotionRejected: with `SUP_FOLLOWING_ERROR`, `SUP_MEASURED_CLEARANCE`,
            `SUP_MEASURED_ENVELOPE` or `SUP_NO_FEEDBACK`; for all but the last the arm has
            been commanded to hold
        """
        if self._monitor_period is None:
            time.sleep(moving_time)
            return
        floor = self._measured_floor(measured)
        reach = self._measured_reach(measured)
        self._over_current = [0] * len(self._over_current)
        began = time.monotonic()
        while True:
            t = time.monotonic() - began
            if t >= moving_time:
                return
            self._check_estop(call)
            # The motors follow the time-based profile; the simulator moves at constant
            # speed. The band runs from the slower of the two, one lag ago, to the faster now.
            late = t - self._following_lag
            behind = min(profile_fraction(late, moving_time, accel_time),
                         min(max(late / moving_time, 0.0), 1.0))
            ahead = max(profile_fraction(t, moving_time, accel_time), t / moving_time)
            self._check_measured(
                call, self._require_feedback(call), floor, reach,
                [[s + (g - s) * f for s, g in zip(start, target)] for f in (behind, ahead)])
            time.sleep(min(self._monitor_period, moving_time - t))

    def _measured_floor(self, measured):
        """Return the measured clearance [m] below which a move starting here is stopped."""
        if self._geometry is None:
            return None
        return min(self._measured_margin, self._clearance(measured)[0])

    def _check_sleep_exit(self, call, start, target, near=0.1):
        """
        Refuse to take the arm out of its sleep pose anywhere but to the home pose.

        The sleep pose folds the arm onto itself, and only the move between it and the home
        pose has been checked against the link meshes. On the arm, a move straight out of
        the sleep pose to elsewhere left its commanded path by 14 mm while unfolding (the
        shoulder lags as it lifts the arm), so it goes home first, as it comes back by way of
        home (Nima's decision, 2026-10-15).

        :param target: joint positions the move goes to; `None` for a Cartesian move
        :raises MotionRejected: with `SUP_SLEEP_EXIT`
        """
        sleep = [float(q) for q in self._arm.group_info.joint_sleep_positions]
        if max(abs(s) for s in sleep) <= near:
            return   # a sleep pose that is the home pose: nothing to unfold
        if max(abs(q - s) for q, s in zip(start, sleep)) > near:
            return
        if target is not None and (max(abs(t) for t in target) <= near
                                   or max(abs(t - s) for t, s in zip(target, sleep)) <= near):
            return   # to the home pose, or staying in the sleep pose
        raise self._refuse(SUP_SLEEP_EXIT, call, {
            'reason': 'from the sleep pose the arm goes to the home pose first: call '
                      'go_to_home_pose()'})

    def set_estop(self, pressed):
        """
        Take the state of the software e-stop (called from the e-stop topic).

        While it is pressed every motion command is refused with `SUP_ESTOP`, and a move in
        progress is stopped with a hold at its next check. The `estop` program has already
        commanded the hold itself, so the arm holds even if this script is not moving it.
        When it is reset, the commanded position is taken from where the arm is, because the
        arm stopped short of whatever was last commanded.
        """
        was, self._estopped = self._estopped, bool(pressed)
        if was and not self._estopped:
            self._arm.capture_joint_positions()
            self._logger.info('E-stop reset: motion commands are accepted again')
        elif self._estopped and not was:
            self._logger.error('E-stop pressed: holding; motion commands are refused')

    def _check_estop(self, call):
        """Stop a move in progress with a hold if the e-stop has been pressed."""
        if self._estopped:
            self._hold(self._require_feedback(call))
            raise self._refuse(SUP_ESTOP, call, {'reason': 'the e-stop was pressed'})

    def _check_current(self, call, measured):
        """
        Stop a move in progress if a motor has drawn more than its current limit for
        `current_readings` readings in a row.

        A motor pushing against something it should not be touching draws far more current
        than moving the arm through free space does. The driver reports each motor's present
        current [mA] as the joint effort; the simulator reports none, and then nothing is
        checked.

        :raises MotionRejected: with `SUP_OVERCURRENT`, after commanding a hold
        """
        if not any(limit is not None for limit in self._current_limit):
            return
        try:
            currents = [float(c) for c in self._arm.get_joint_efforts()]
        except (IndexError, TypeError, ValueError):
            return   # no current reported
        for i, (current, limit) in enumerate(zip(currents, self._current_limit)):
            over = limit is not None and abs(current) > limit
            self._over_current[i] = self._over_current[i] + 1 if over else 0
            if self._over_current[i] >= self._current_readings:
                self._hold(measured)
                raise self._refuse(SUP_OVERCURRENT, call, {
                    'joint': self._arm.group_info.joint_names[i],
                    'current_ma': round(current), 'limit_ma': limit,
                    'readings': self._over_current[i]})

    def _measured_reach(self, measured):
        """Return how far [m] outside the work envelope a move starting here may be measured."""
        if self._work_envelope is None:
            return None
        names = self._arm.group_info.joint_names
        start = excess(
            self._geometry, self._work_envelope, dict(zip(names, measured)), self._held)[0]
        return max(0.0, start) + self._envelope_tolerance

    def _check_measured(self, call, measured, floor, reach, expected):
        """
        Stop a move in progress if the measured arm is not where the command puts it.

        :param measured: measured joint positions
        :param floor: from `_measured_floor` at the start of the move
        :param reach: from `_measured_reach` at the start of the move
        :param expected: joint position lists the command has passed through within the
            expected lag; each measured joint should lie in the range they span
        :raises MotionRejected: with `SUP_FOLLOWING_ERROR`, `SUP_MEASURED_CLEARANCE` or
            `SUP_MEASURED_ENVELOPE`, after commanding a hold at the measured position
        """
        names = self._arm.group_info.joint_names
        for i, (name, q) in enumerate(zip(names, measured)):
            values = [joints[i] for joints in expected]
            error = max(min(values) - q, q - max(values))
            if error > self._following_tolerance:
                self._hold(measured)
                raise self._refuse(SUP_FOLLOWING_ERROR, call, {
                    'joint': name, 'error': round(error, 4),
                    'tolerance': self._following_tolerance})
        if floor is not None:
            # The arm travels on after a hold (about 0.2 s at its speed, measured on the
            # real vx300s), so waiting until it is near the table is too late. Stop as soon
            # as it is lower than the command explains by more than the droop the table
            # margin allows for, wherever that happens.
            clearance, link = self._clearance(measured)
            commanded = min(self._clearance(joints)[0] for joints in expected)
            allowed = self._table_margin - self._measured_margin
            if clearance < floor - 0.001 or clearance < commanded - allowed - 0.001:
                self._hold(measured)
                raise self._refuse(SUP_MEASURED_CLEARANCE, call, {
                    'clearance_mm': round(clearance * 1000, 1),
                    'commanded_mm': round(commanded * 1000, 1),
                    'allowed_below_command_mm': round(allowed * 1000, 1), 'link': link})
        if reach is not None:
            # The command never leaves the envelope, so a measured arm this far outside it
            # is not where it was sent. It travels on after the hold, like any stop.
            outside, axis, side, link = excess(
                self._geometry, self._work_envelope, dict(zip(names, measured)), self._held)
            if outside > reach + 0.001:
                self._hold(measured)
                raise self._refuse(SUP_MEASURED_ENVELOPE, call, {
                    'outside_mm': round(outside * 1000, 1), 'side': f'{axis} {side}',
                    'limit_m': self._work_envelope[side]['xyz'.index(axis)],
                    'allowed_outside_mm': round(reach * 1000, 1), 'link': link})
        self._check_current(call, measured)

    def _check_positions(self, call, joints, **where):
        """
        Refuse joint positions that are not finite or lie outside the enforced limits.

        :param where: extra detail for `MotionRejected` (e.g. the waypoint number)
        :raises MotionRejected: with `SUP_INVALID_TARGET` or `SUP_POSITION_LIMIT`
        """
        names = self._arm.group_info.joint_names
        for name, q, lower, upper in zip(names, joints, self._lower, self._upper):
            if not math.isfinite(q):
                raise self._refuse(SUP_INVALID_TARGET, call, dict(where, joint=name))
            if not lower - 1e-3 <= q <= upper + 1e-3:
                raise self._refuse(SUP_POSITION_LIMIT, call, dict(
                    where, joint=name, target=round(q, 4), limits=[lower, upper]))

    def _time_needed(self, start, target, accel_time):
        """
        Return the shortest moving time [s] at which a move keeps to the joint speed limits.

        With the time-based profile the peak speed of a joint is its travel over the time not
        spent accelerating, and the acceleration time is at most half the moving time.

        :return: moving time, rounded up to the millisecond the motor register holds
        """
        cruise = max((abs(t - s) / limit for s, t, limit in zip(start, target, self._speed)
                      if limit > 0.0), default=0.0)
        needed = cruise + accel_time if cruise >= accel_time else 2.0 * cruise
        return math.ceil(needed * 1000.0 - 1e-6) / 1000.0

    def _check_speeds(self, call, start, end, duration, **where):
        """
        Refuse a step from `start` to `end` that needs more than the enforced joint speed.

        :param duration: time [s] over which the step is covered at its fastest
        :raises MotionRejected: with `SUP_VELOCITY_LIMIT`
        """
        names = self._arm.group_info.joint_names
        for name, a, b, limit in zip(names, start, end, self._speed):
            speed = abs(b - a) / duration
            if speed > limit * (1.0 + 1e-9):
                raise self._refuse(SUP_VELOCITY_LIMIT, call, dict(
                    where, joint=name, peak_speed=round(speed, 3), limit=round(limit, 3)))

    def _refuse(self, code, call, detail):
        """Log a refusal and return the `MotionRejected` to raise for it."""
        self._logger.error(f'{call} rejected ({code}): {detail}')
        return MotionRejected(code, call, detail)

    def _clearance(self, joints):
        """Return the height [m] of the arm's lowest point above the table, and its link."""
        z, link = self._geometry.lowest_point(dict(zip(self._arm.group_info.joint_names, joints)))
        return z - self._table_z, link

    def _table_slack(self, joints):
        """
        Measure the commanded arm against the table, each part against its own margin.

        The links have to keep `table_margin`; an object in the gripper has the margin it was
        attached with, by default none, so that it can be set down.

        :return: (clearance less margin [m] of the part that has least to spare; that part's
            clearance [m]; its margin [m]; its name)
        """
        clearance, link = self._clearance(joints)
        worst = (clearance - self._table_margin, clearance, self._table_margin, link)
        if self._held is not None:
            q = dict(zip(self._arm.group_info.joint_names, joints))
            clearance = self._geometry.held_lowest(q, self._held) - self._table_z
            margin = self._held['table_margin']
            worst = min(worst, (clearance - margin, clearance, margin, HELD))
        return worst

    def _check_table(self, call, path):
        """
        Refuse a path that brings the commanded arm within the margin of the table.

        A path that starts inside the margin is allowed as long as it does not go further
        inside than where it starts, so the arm can always be moved away from the table.

        :param call: name of the public method being carried out
        :param path: joint position lists the move passes through, without its start
        :raises MotionRejected: with code `SUP_TABLE_CLEARANCE`; nothing has been sent
        """
        if self._geometry is None:
            return
        allowed = min(0.0, self._table_slack(self._arm.get_joint_commands())[0])
        for i, joints in enumerate(path, start=1):
            slack, clearance, margin, link = self._table_slack(joints)
            if slack < allowed - 0.001:
                detail = {
                    'clearance_mm': round(clearance * 1000, 1),
                    'margin_mm': round(margin * 1000, 1),
                    'link': link, 'path_fraction': round(i / len(path), 2)}
                raise self._refuse(SUP_TABLE_CLEARANCE, call, detail)

    def set_work_envelope(self, box):
        """
        Replace the work envelope; the commands that follow are held to the new one.

        A move in progress keeps the envelope it was checked against. Safe to call from any
        thread (the envelope topic's callback calls it).

        :param box: `{'min': [x, y, z], 'max': [x, y, z]}` [m], or `None` for no envelope
        :raises ValueError: if `box` is not a box
        """
        box = None if box is None else checked_envelope(box)
        with self._envelope_lock:
            self._next_envelope = (box,)

    def _check_envelope(self, call, path):
        """
        Refuse a path that takes a moving link of the commanded arm outside the work envelope.

        A path that starts outside the envelope is allowed as long as the arm does not reach
        further out than where it starts, so the arm can always be brought back in.

        :param call: name of the public method being carried out
        :param path: joint position lists the move passes through, without its start
        :raises MotionRejected: with code `SUP_WORK_ENVELOPE`; nothing has been sent
        """
        with self._envelope_lock:
            if self._next_envelope is not None and self._geometry is not None:
                self._work_envelope = self._next_envelope[0]
            self._next_envelope = None
        if self._work_envelope is None:
            return
        names = self._arm.group_info.joint_names
        start = excess(self._geometry, self._work_envelope,
                       dict(zip(names, self._arm.get_joint_commands())), self._held)[0]
        allowed = max(0.0, start)
        for i, joints in enumerate(path, start=1):
            outside, axis, side, link = excess(
                self._geometry, self._work_envelope, dict(zip(names, joints)), self._held)
            if outside > allowed + 0.001:
                detail = {
                    'outside_mm': round(outside * 1000, 1), 'side': f'{axis} {side}',
                    'limit_m': self._work_envelope[side]['xyz'.index(axis)],
                    'link': link, 'path_fraction': round(i / len(path), 2)}
                raise self._refuse(SUP_WORK_ENVELOPE, call, detail)

    def _self_clearance(self, joints):
        """Return the smallest capsule distance of the arm, and the two links it is between."""
        q = dict(zip(self._arm.group_info.joint_names, joints))
        if self._held is None:
            return self._self_collision.clearance(q)
        return self._self_collision.clearance(q, self._held)

    def _check_self_collision(self, call, path):
        """
        Refuse a path that brings two links of the commanded arm within the margin.

        As with the table, a path that starts inside the margin is allowed as long as the
        links do not come closer than where they start, so a folded arm can be unfolded.

        :param call: name of the public method being carried out
        :param path: joint position lists the move passes through, without its start
        :raises MotionRejected: with code `SUP_SELF_COLLISION`; nothing has been sent
        """
        if self._self_collision is None:
            return
        start = self._self_clearance(self._arm.get_joint_commands())
        floor = min(self._self_margin, start[0])
        for i, joints in enumerate(path, start=1):
            clearance, link, other = self._self_clearance(joints)
            if clearance < floor - 0.001:
                detail = {
                    'clearance_mm': round(clearance * 1000, 1),
                    'margin_mm': round(self._self_margin * 1000, 1),
                    'links': [link, other], 'path_fraction': round(i / len(path), 2)}
                raise self._refuse(SUP_SELF_COLLISION, call, detail)

    def _pose_command(self, call, method, **kwargs):
        """
        Carry out a pose command: solve it, check the solution against the table, then send.

        :return: the joint positions [rad] that reach the pose
        :raises MotionRejected: if there is no joint solution or it is refused
        """
        if self._ik is not None:
            return self._solved_pose_command(call, **kwargs)
        if not kwargs['execute']:
            return self._command(call, method, **kwargs)
        solution, solved = method(**dict(kwargs, execute=False))
        if not solved:
            raise self._refuse(SUP_NO_SOLUTION, call, {})
        solution = [float(q) for q in solution]
        # seeded with its own solution, the solver returns it unchanged
        return self._command(call, method, target=solution, **dict(kwargs, custom_guess=solution))

    def _branch_cost(self, joints, reference):
        """Return the cost of a joint solution: weighted travel plus nearness to limits."""
        travel = sum(w * abs(q - r) for w, q, r in zip(BRANCH_WEIGHTS, joints, reference))
        crowding = sum(max(0.0, BRANCH_LIMIT_MARGIN - min(q - lo, hi - q))
                       for q, lo, hi in zip(joints, self._lower, self._upper))
        return travel + crowding

    def _solve(self, call, T_sd, reference, **where):
        """
        Return the joint solutions of a pose that are inside the limits, cheapest first.

        :param reference: joint positions the cost of a solution is measured from
        :raises MotionRejected: with `SUP_NO_SOLUTION` and the reason: the pose is out of
            reach, or every solution has a joint outside its limits
        """
        names = self._arm.group_info.joint_names
        try:
            inside, outside = self._ik.solve_within(T_sd, self._lower, self._upper, reference)
        except Unreachable as problem:
            raise self._refuse(
                SUP_NO_SOLUTION, call, dict(where, reason=f'out of reach: {problem}'))
        if not inside:
            raise self._refuse(SUP_NO_SOLUTION, call, dict(
                where, reason='every joint solution has a joint outside its limits',
                outside={branch.name: names[joint] for branch, joint in outside}))
        return sorted(inside, key=lambda branch: self._branch_cost(branch.joints, reference))

    def _solved_pose_command(
            self, call, custom_guess=None, execute=True, moving_time=None, accel_time=None,
            blocking=True, T_sd=None, **components):
        """
        Carry out a pose command with the closed-form solver.

        Of the joint solutions inside the limits, the one that costs least from where the
        arm is commanded now (or from `custom_guess`) is taken. If the move to it fails a
        check the command is refused; no other solution is tried, so a pose command never
        turns into a reconfiguration of the arm that the script did not ask for. The choice
        depends only on the pose and the reference, and is logged.

        :return: the joint positions [rad] that reach the pose
        :raises MotionRejected: if there is no joint solution or the move to it is refused
        """
        if self._fault is not None:
            raise MotionRejected(self._fault, call, {'reason': 'feedback was lost earlier'})
        if T_sd is None:
            x, y = components.get('x', 0), components.get('y', 0)
            yaw = components.get('yaw')
            T_sd = np.identity(4)
            T_sd[:3, :3] = ang.euler_angles_to_rotation_matrix([
                components.get('roll', 0), components.get('pitch', 0),
                math.atan2(y, x) if yaw is None else yaw])
            T_sd[:3, 3] = [x, y, components.get('z', 0)]
        start = self._arm.get_joint_commands()
        branches = self._solve(call, np.array(T_sd, float), custom_guess or start)
        joints = list(branches[0].joints)
        if not execute:
            return joints
        self._logger.info(f'{call}: joint solution {branches[0].name}')
        self._command(
            call, self._arm.set_joint_positions, target=joints, joint_positions=joints,
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)
        return joints

    def _feedback_age(self):
        """Return the age [s] of the latest joint state message."""
        core = self._arm.core
        with core.js_mutex:
            stamp = Time.from_msg(core.joint_states.header.stamp)
        return (core.get_node().get_clock().now() - stamp).nanoseconds * 1e-9

    def _require_feedback(self, call):
        """
        Return the measured joint positions, or stop if they cannot be trusted.

        Feedback is lost when the latest joint state message is older than the feedback
        timeout (driver stopped), or holds values that are not finite (the driver publishes
        NaN on a failed bus read when `read_failure_behavior` is 1 in the motor config).
        Nothing is sent to the arm in that case, not even a hold: there is no measured
        position to hold at.

        :param call: name of the public method being carried out
        :raises MotionRejected: with code `SUP_NO_FEEDBACK`
        """
        deadline = time.monotonic() + self._feedback_timeout
        detail = None
        while detail is None:
            age = self._feedback_age()
            if age <= self._feedback_timeout:
                break
            if time.monotonic() >= deadline:
                detail = {'reason': 'joint states are stale', 'age': round(age, 3)}
            else:
                time.sleep(0.01)
        if detail is None:
            measured = [float(q) for q in self._arm.get_joint_positions()]
            if all(math.isfinite(q) for q in measured):
                return measured
            detail = {'reason': 'joint states are not finite'}
        self._fault = SUP_NO_FEEDBACK
        self._logger.error(f'{call}: feedback lost, nothing more will be sent: {detail}')
        raise MotionRejected(SUP_NO_FEEDBACK, call, detail)

    def _check_arrival(self, call):
        """
        Check that the measured joints are at the commanded positions after a blocking move.

        If they are not within the tolerance once the settle time has passed, the arm is
        commanded to hold at its measured position, so that the next move cannot start from
        a pose the arm is not in.

        :param call: name of the public method that commanded the move
        :raises MotionRejected: with code `SUP_NOT_ARRIVED`, or `SUP_NO_FEEDBACK`
        """
        deadline = time.monotonic() + self._settle_timeout
        while True:
            commanded = self._arm.get_joint_commands()
            measured = self._require_feedback(call)
            errors = [m - c for m, c in zip(measured, commanded)]
            if max(abs(e) for e in errors) <= self._arrival_tolerance:
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(0.02)
        self._hold(measured)
        detail = {
            'tolerance': self._arrival_tolerance,
            'error': {
                name: round(e, 4)
                for name, e in zip(self._arm.group_info.joint_names, errors)
                if abs(e) > self._arrival_tolerance},
        }
        self._logger.error(f'{call} did not arrive, holding position: {detail}')
        raise MotionRejected(SUP_NOT_ARRIVED, call, detail)

    def attach_object(self, radius, p0, p1, table_margin=0.0):
        """
        Tell the supervisor that the gripper holds an object, so that it is checked too.

        The object is described by a capsule: every point within `radius` of the segment from
        `p0` to `p1`. The points are in the frame of the gripper tip (`ee_gripper_link`, the
        point between the fingertips): x along the fingers away from the wrist, y towards
        the left finger, z up when the gripper is level. For a bottle 0.2 m tall and 0.06 m
        across, held upright by its middle in a level gripper:

            bot.gripper.grasp()
            bot.arm.attach_object(radius=0.03, p0=(0, 0, -0.07), p1=(0, 0, 0.07))

        From then on the object has to stay inside the work envelope and clear of the arm
        (except the gripper), like a link, and no move may take it below the table. It may
        touch the table, so that it can be set down: the links keep their margin from the
        table, the object by default has none. Only the commanded path is checked for the
        object against the table; a real arm that settles low presses it onto the table.
        Call `detach_object` after releasing it.

        :param radius: radius [m] of the capsule
        :param p0: one end of the capsule's axis, (x, y, z) [m] in the gripper tip frame
        :param p1: the other end
        :param table_margin: (optional) smallest distance [m] allowed between the object and
            the table
        :raises ValueError: if the capsule is not three finite numbers per point, a positive
            radius and a margin that is not negative
        """
        ends = [[float(v) for v in point] for point in (p0, p1)]
        if any(len(point) != 3 for point in ends) or not all(
                math.isfinite(v) for point in ends for v in point):
            raise ValueError('p0 and p1 must each be three finite numbers')
        if not (math.isfinite(radius) and radius > 0.0):
            raise ValueError('radius must be a positive number')
        if not (math.isfinite(table_margin) and table_margin >= 0.0):
            raise ValueError('table_margin must be zero or a positive number')
        self._held = {
            'frame': 'ee_gripper_link', 'p0': ends[0], 'p1': ends[1], 'radius': float(radius),
            'table_margin': float(table_margin)}
        self._radii = None
        self._logger.info(f'Held object attached: radius {radius} m, from {ends[0]} to {ends[1]}')

    def detach_object(self):
        """Tell the supervisor that the gripper no longer holds an object."""
        self._held = None
        self._radii = None
        self._logger.info('Held object detached')

    def set_trajectory_time(self, moving_time=None, accel_time=None):
        """Set the duration and acceleration time [s] used by the following moves."""
        self._arm.set_trajectory_time(moving_time, accel_time)
        if moving_time is not None:
            self._default_moving_time = moving_time

    def capture_joint_positions(self):
        """Reset the commanded joints to the measured ones (use after torquing back on)."""
        self._arm.capture_joint_positions()

    def go_to_home_pose(self, moving_time=None, accel_time=None, blocking=True):
        """Command the arm to its Home pose (all joints at zero)."""
        self._command(
            'go_to_home_pose', self._arm.go_to_home_pose,
            target=[0.0] * self._arm.group_info.num_joints,
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def go_to_sleep_pose(self, moving_time=None, accel_time=None, blocking=True):
        """
        Command the arm to its Sleep pose.

        The Sleep pose folds the arm onto itself, closer than the capsule model can tell
        from a collision (the link meshes are 14 mm apart there), so the self-collision
        check would refuse it. The move from the Home pose to the Sleep pose has been checked
        against the meshes instead, and is sent without that check; from any other pose the
        check applies, and the script has to go to the Home pose first.
        """
        at_home = all(
            abs(q) <= self._arrival_tolerance for q in self._arm.get_joint_commands())
        self._command(
            'go_to_sleep_pose', self._arm.go_to_sleep_pose,
            target=[float(q) for q in self._arm.group_info.joint_sleep_positions],
            verified_path=at_home and self._held is None,
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def set_joint_positions(
            self, joint_positions, moving_time=None, accel_time=None, blocking=True):
        """
        Command positions [rad] to all the arm joints.

        :raises MotionRejected: if the command is refused
        """
        joint_positions = [float(q) for q in joint_positions]
        self._command(
            'set_joint_positions', self._arm.set_joint_positions, target=joint_positions,
            joint_positions=joint_positions,
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def set_single_joint_position(
            self, joint_name, position, moving_time=None, accel_time=None, blocking=True):
        """
        Command one joint to a position [rad].

        :raises MotionRejected: if the command is refused
        """
        target = list(self._arm.get_joint_commands())
        if joint_name in self._arm.info_index_map:
            target[self._arm.info_index_map[joint_name]] = float(position)
        self._command(
            'set_single_joint_position', self._arm.set_single_joint_position, target=target,
            joint_name=joint_name, position=float(position),
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def set_ee_pose_matrix(
            self, T_sd, custom_guess=None, execute=True, moving_time=None, accel_time=None,
            blocking=True):
        """
        Command an end-effector pose given as a 4x4 matrix in the base frame.

        :return: the joint positions [rad] that reach the pose
        :raises MotionRejected: if no valid joint solution is found
        """
        return self._pose_command(
            'set_ee_pose_matrix', self._arm.set_ee_pose_matrix,
            T_sd=T_sd, custom_guess=custom_guess, execute=execute,
            moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def set_ee_pose_components(
            self, x=0, y=0, z=0, roll=0, pitch=0, yaw=None, custom_guess=None, execute=True,
            moving_time=None, accel_time=None, blocking=True):
        """
        Command an end-effector pose in the base frame (position [m], angles [rad]).

        :return: the joint positions [rad] that reach the pose
        :raises MotionRejected: if no valid joint solution is found
        """
        return self._pose_command(
            'set_ee_pose_components', self._arm.set_ee_pose_components,
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, custom_guess=custom_guess,
            execute=execute, moving_time=moving_time, accel_time=accel_time, blocking=blocking)

    def set_ee_cartesian_trajectory(
            self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=None, wp_moving_time=0.2,
            wp_accel_time=0.1, wp_period=0.05):
        """
        Command a straight-line displacement of the end effector (position [m], angles [rad]).

        The displacement is relative to the commanded pose, in a frame with the base frame's
        axes turned to the end effector's yaw (the same convention as the Interbotix API).

        :raises MotionRejected: if the path cannot be planned
        """
        self._command(
            'set_ee_cartesian_trajectory', self._stream_cartesian,
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, moving_time=moving_time,
            wp_moving_time=wp_moving_time, wp_accel_time=wp_accel_time, wp_period=wp_period)

    def _plan_cartesian(self, x, y, z, roll, pitch, yaw, moving_time, wp_period):
        """
        Plan the joint waypoints of a straight-line displacement, without sending anything.

        The path is that of `InterbotixArmXSInterface.set_ee_cartesian_trajectory`. With the
        closed-form solver, every waypoint keeps the waist and elbow configuration of the
        first, and takes the wrist solution nearest the waypoint before it.

        :return: the waypoints after the start (a list of joint position lists) and the final
            pose `T_sd`
        :raises MotionRejected: with `SUP_NO_SOLUTION` if the path cannot be planned
        """
        arm = self._arm
        call = 'set_ee_cartesian_trajectory'
        if arm.group_info.num_joints < 6 and (y != 0 or yaw != 0):
            raise self._refuse(SUP_NO_SOLUTION, call, {
                'reason': "'y' and 'yaw' need an arm with 6 joints"})
        rpy = ang.rotation_matrix_to_euler_angles(arm.T_sb[:3, :3])
        T_sy = np.identity(4)
        T_sy[:3, :3] = ang.euler_angles_to_rotation_matrix([0.0, 0.0, rpy[2]])
        T_yb = np.dot(mr.TransInv(T_sy), arm.T_sb)
        rpy[2] = 0.0
        n = int(moving_time / wp_period + 1e-9)   # 0.6 / 0.05 is 11.999... in floating point
        inc = 1.0 / float(n)
        joints = [float(q) for q in arm.joint_commands]
        waypoints = []
        arm_branch = None  # waist and elbow configuration, the same for the whole path
        for i in range(n):
            T_yb[:3, 3] += [inc * x, inc * y, inc * z]
            rpy[0] += inc * roll
            rpy[1] += inc * pitch
            rpy[2] += inc * yaw
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
            T_sd = np.dot(T_sy, T_yb)
            if self._ik is None:
                joints, solved = arm.set_ee_pose_matrix(T_sd, joints, False, blocking=False)
                if not solved:
                    raise self._refuse(
                        SUP_NO_SOLUTION, call, {'planned_fraction': round(i / n, 3)})
            else:
                branches = self._solve(call, T_sd, joints, planned_fraction=round(i / n, 3))
                if arm_branch is not None:
                    branches = [b for b in branches if (b.waist, b.elbow) == arm_branch]
                    if not branches:
                        raise self._refuse(SUP_NO_SOLUTION, call, {
                            'planned_fraction': round(i / n, 3),
                            'reason': 'the path cannot be followed without changing the '
                                      f'arm configuration ({"/".join(arm_branch)})'})
                # the solution that continues the path is the one nearest the last waypoint
                branch = min(branches, key=lambda b: max(
                    abs(q - last) for q, last in zip(b.joints, joints)))
                if arm_branch is None:
                    arm_branch = (branch.waist, branch.elbow)
                    self._logger.info(f'{call}: joint solution {"/".join(arm_branch)}')
                joints = branch.joints
            waypoints.append([float(q) for q in joints])
        return waypoints, T_sd

    def _haste(self, start, waypoints, wp_period):
        """Return the largest ratio of a joint's speed between waypoints to its limit."""
        worst, previous = 0.0, start
        for waypoint in waypoints:
            for a, b, limit in zip(previous, waypoint, self._speed):
                if limit > 0.0:
                    worst = max(worst, abs(b - a) / wp_period / limit)
            previous = waypoint
        return worst

    def _stream_cartesian(
            self, x, y, z, roll, pitch, yaw, moving_time, wp_moving_time, wp_accel_time,
            wp_period):
        """
        Plan a straight-line displacement and send its waypoints to the arm one at a time.

        The waypoints go out as joint group commands, one every `wp_period`, from this thread.
        The driver's own trajectory execution is not used: it cannot be cancelled, and it
        drops a trajectory that arrives while another is executing. Sending from here means
        that when the script stops, no further waypoint is sent. Before every waypoint the
        measured arm is checked against the waypoints already sent and against the table.

        A path that would need more than a joint's speed limit is planned again over the
        longer time the limit allows. The whole path is then checked before the first
        waypoint goes out: positions, the speed each step needs at the waypoint period (which
        catches a jump between joint solutions), clearance above the table, the work
        envelope, and clearance between the links of the arm.

        :return: `True` once the path has been sent
        :raises MotionRejected: if the path cannot be planned or is refused
        """
        arm = self._arm
        self._check_sleep_exit('set_ee_cartesian_trajectory', arm.get_joint_commands(), None)
        if moving_time is None:
            moving_time = arm.moving_time
        accel_time = arm.accel_time
        waypoints, T_sd = self._plan_cartesian(
            x, y, z, roll, pitch, yaw, moving_time, wp_period)
        asked, before = moving_time, None
        for _ in range(CARTESIAN_RETIMES):
            haste = self._haste(arm.get_joint_commands(), waypoints, wp_period)
            # A jump between joint solutions is not cured by more waypoints: if the last
            # round did not help, stop, and the speed check below refuses the path.
            if haste <= 1.0 or (before is not None and haste > 0.5 * before):
                break
            # Too fast for the speed limit: plan again over the time the limit allows. The
            # joints do not move evenly along the path, so a finer plan can show a faster
            # stretch and need another round.
            before = haste
            moving_time = math.ceil(moving_time * haste / wp_period + 1.0) * wp_period
            waypoints, T_sd = self._plan_cartesian(
                x, y, z, roll, pitch, yaw, moving_time, wp_period)
        if moving_time > asked:
            self._logger.info('set_ee_cartesian_trajectory: slowed to the speed limit, moving '
                              f'time {asked:.2f} s -> {moving_time:.2f} s')
        previous = arm.get_joint_commands()
        for i, waypoint in enumerate(waypoints, start=1):
            self._check_positions('set_ee_cartesian_trajectory', waypoint, waypoint=i)
            self._check_speeds(
                'set_ee_cartesian_trajectory', previous, waypoint, wp_period, waypoint=i)
            previous = waypoint
        sent = [arm.get_joint_commands()]
        # the motors move in a straight line in joint space from one waypoint to the next
        path = [pose for a, b in zip(sent + waypoints, waypoints) for pose in self._between(a, b)]
        self._check_table('set_ee_cartesian_trajectory', path)
        self._check_envelope('set_ee_cartesian_trajectory', path)
        self._check_self_collision('set_ee_cartesian_trajectory', path)
        # waypoints the measured arm may still be catching up with
        window = math.ceil((wp_moving_time + self._following_lag) / wp_period) + 1
        measured = self._require_feedback('set_ee_cartesian_trajectory')
        floor, reach = self._measured_floor(measured), self._measured_reach(measured)
        self._over_current = [0] * len(self._over_current)
        arm.set_trajectory_time(wp_moving_time, wp_accel_time)
        try:
            start = time.monotonic()
            for i, waypoint in enumerate(waypoints, start=1):
                time.sleep(max(0.0, start + i * wp_period - time.monotonic()))
                self._check_estop('set_ee_cartesian_trajectory')
                measured = self._require_feedback('set_ee_cartesian_trajectory')
                if self._monitor_period is not None:
                    self._check_measured(
                        'set_ee_cartesian_trajectory', measured, floor, reach, sent[-window:])
                sent.append(waypoint)
                arm.joint_commands = list(waypoint)
                arm.core.pub_group.publish(
                    JointGroupCommand(name=arm.group_name, cmd=waypoint))
            time.sleep(max(0.0, start + moving_time + wp_moving_time - time.monotonic()))
            arm.T_sb = T_sd
        except BaseException:
            arm._update_Tsb()  # stopped part-way: the commanded pose is the last waypoint sent
            raise
        finally:
            # with feedback lost the driver may be gone, and this service call would not return
            if self._fault is None:
                # a time that had to be lengthened is not kept for the moves that follow
                arm.set_trajectory_time(
                    self._default_moving_time if moving_time > asked else asked, accel_time)
        return True


class SupervisedManipulator:
    """Drop-in replacement for `InterbotixManipulatorXS` whose arm is supervised."""

    # arguments passed on to `ArmSupervisor`
    SUPERVISOR_ARGS = (
        'arrival_tolerance', 'settle_timeout', 'feedback_timeout', 'table_z', 'table_margin',
        'monitor_period', 'following_tolerance', 'following_lag', 'measured_margin',
        'self_margin', 'envelope_tolerance', 'path_step')

    def __init__(self, robot_model='vx300s', **kwargs):
        """
        Construct the arm, gripper and supervisor.

        :param robot_model: Interbotix arm model
        :param kwargs: any of `SUPERVISOR_ARGS` (see `ArmSupervisor`), and any other argument
            of `InterbotixManipulatorXS`. `accel_time` defaults to 0.15 s instead of that
            class's 0.3 s: on the real arm it shortens the travel after a hold by a third.
            `work_envelope` is the envelope the arm's computer publishes unless given (see
            `supervisor/envelope.py`); pass `None` to run without one. `envelope_wait` [s]
            (default 3) is how long to wait for that envelope; if none comes (the driver was
            not started with arm.launch.py of xsarm_scripts), this computer's envelope file
            is used, with a warning. `speed_limit` [rad/s]
            lowers the joint speed limit for this script; moves asked for faster are slowed
            to it
        """
        kwargs.setdefault('accel_time', 0.15)
        speed_limit = kwargs.pop('speed_limit', None)
        envelope_given = 'work_envelope' in kwargs
        work_envelope = kwargs.pop('work_envelope', None)
        envelope_wait = kwargs.pop('envelope_wait', 3.0)
        supervisor_args = {k: kwargs.pop(k) for k in self.SUPERVISOR_ARGS if k in kwargs}
        self._bot = InterbotixManipulatorXS(robot_model=robot_model, **kwargs)
        self.arm = None
        self._station_message = self._station_used = None
        if envelope_given:
            source = 'given by the script'
        else:
            work_envelope, source = self._station_envelope(envelope_wait)
        geometry = RobotGeometry(
            generate_urdf(robot_model, kwargs.get('robot_name') or robot_model))
        path = os.path.join(
            get_package_share_directory('xsarm_scripts'), 'config', 'supervisor_limits.yaml')
        with open(path) as f:
            floors = yaml.safe_load(f)
        if robot_model not in floors:
            raise ValueError(f"no limit floor for '{robot_model}' in {path}")
        floor = floors[robot_model]
        if speed_limit is not None:
            # a lower speed for this script only, such as for a first run on the arm
            floor = dict(floor, velocity=min(floor.get('velocity', math.inf), speed_limit))
        self.arm = ArmSupervisor(
            self._bot.arm, geometry=geometry, limit_floor=floor,
            self_collision=SelfCollisionModel.load(geometry, robot_model),
            work_envelope=work_envelope, ik=AnalyticIK(self._bot.arm.robot_des),
            **supervisor_args)
        self._log_envelope(work_envelope, source)
        if self._station_message is not None:
            # a change that came while the supervisor was being built
            self._apply_station_message(self._station_message)
        self.gripper = getattr(self._bot, 'gripper', None)
        # The e-stop's state is kept by its publisher (transient local), so a script started
        # while it is pressed learns so at once.
        self._estop_subscription = self.get_node().create_subscription(
            Bool, estop_topic(self._bot.core.robot_name),
            lambda message: self.arm.set_estop(message.data), ESTOP_QOS)

    def get_node(self):
        """Return the ROS node used by this robot."""
        return self._bot.get_node()

    def _log_envelope(self, box, source):
        if box is None:
            self.get_node().get_logger().warn(f'No work envelope ({source})')
            return
        low, high = box['min'], box['max']
        self.get_node().get_logger().info(
            f'Work envelope ({source}): x {low[0]:.3f} to {high[0]:.3f}, '
            f'y {low[1]:.3f} to {high[1]:.3f}, z {low[2]:.3f} to {high[2]:.3f} m')

    def _apply_station_message(self, text):
        """Pass a changed envelope of the arm's computer on to the supervisor."""
        if text == self._station_used:
            return
        self._station_used = text
        try:
            box, host, path = read_station_message(text)
        except ValueError as error:
            self.get_node().get_logger().error(
                f'{error}; the previous work envelope stays in force')
            return
        self.arm.set_work_envelope(box)
        self._log_envelope(box, f'changed on {host}, {path}; from the next command')

    def _station_envelope(self, wait):
        """
        Return the work envelope the arm's computer publishes, and where it came from.

        The subscription stays: a later change on the arm's computer is passed on to the
        supervisor and applies from the next command. If nothing is published within `wait`
        seconds, this computer's envelope file is returned instead.

        :raises ValueError: if the arm's computer reports an envelope file that is not a box
        """
        node = self.get_node()
        topic = envelope_topic(self._bot.core.robot_name)
        received = threading.Event()

        def receive(message):
            self._station_message = message.data
            received.set()
            if self.arm is not None:
                self._apply_station_message(message.data)

        self._envelope_subscription = node.create_subscription(
            String, topic, receive, ENVELOPE_QOS)
        deadline = time.monotonic() + wait
        while not received.is_set() and time.monotonic() < deadline:
            if interbotix_is_up():
                received.wait(0.05)     # robot_startup() already spins the node
            else:
                rclpy.spin_once(node, timeout_sec=0.05)
        if received.is_set():
            self._station_used = self._station_message
            box, host, path = read_station_message(self._station_used)
            source = f'from {host}, {path}'
            local = envelope_path()
            if os.path.exists(local) and host != socket.gethostname():
                node.get_logger().info(f'{local} on this computer is not used')
            return box, source
        node.get_logger().warn(
            f'No work envelope from the arm\'s computer on {topic} within {wait:.0f} s '
            '(was the driver started with `ros2 launch xsarm_scripts arm.launch.py`?). '
            f'Using this computer\'s file {envelope_path()} instead.')
        return load_envelope(), f'this computer\'s file {envelope_path()}'

    def shutdown(self):
        """Shut the robot's node down, as `InterbotixManipulatorXS.shutdown` does."""
        self._bot.shutdown()

    @property
    def core(self):
        raise AttributeError(
            "'core' is not available through the arm supervisor: commands sent through it "
            '(trajectories, currents, PWM, operating modes, torque) would not be checked')

    def is_simulated(self):
        """
        Return whether the arm behind the driver is the simulator and not the motors.

        Judged from the nodes that are running, not from an argument anyone passed: the
        simulated driver is the node `xs_sdk_sim` in the robot's namespace, the real one is
        `xs_sdk`. If both are seen, or neither, the arm is taken to be real.
        """
        namespace = '/' + self._bot.core.robot_name
        drivers = {name for name, ns in self.get_node().get_node_names_and_namespaces()
                   if ns.rstrip('/') == namespace and name.startswith('xs_sdk')}
        return drivers == {'xs_sdk_sim'}

    def show_unchecked(self, joint_positions, moving_time=None):
        """
        Move the SIMULATED arm to joint positions without any of the supervisor's checks.

        For showing in simulation what a configuration outside the limits looks like. It is
        refused whenever the driver is not the simulator.

        :raises MotionRejected: with `SUP_NOT_SIMULATED` if the arm may be real
        """
        if not self.is_simulated():
            raise MotionRejected(SUP_NOT_SIMULATED, 'show_unchecked', {
                'reason': 'unchecked moves are for the simulator only'})
        self._bot.arm._publish_commands([float(q) for q in joint_positions], moving_time)
