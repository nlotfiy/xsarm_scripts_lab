"""
Closed-form inverse kinematics of the vx300s (task 1.3 in docs/supervisor_steps.md).

The arm has a waist, a shoulder and an elbow that place the wrist centre, and three wrist
axes that meet there, so a pose has at most eight joint solutions: the waist facing the wrist
centre or turned away from it, the elbow on either side of the line from the shoulder to the
wrist centre, and the wrist bent either way. `AnalyticIK.solve` returns every one of them
that exists; it never iterates and never fails to converge.

The dimensions are read from the Modern Robotics description of the arm (`Slist`, `M`), the
one `arm.py` computes its forward kinematics with, so a solution reproduces the pose exactly
in the frame the rest of the API uses. Joint limits are not known here; the caller passes
the limits it enforces.
"""

from dataclasses import dataclass
import math

import modern_robotics as mr
import numpy as np

# below this distance [m] the wrist centre counts as being on the waist axis
SINGULAR = 1e-9
# below this sine the wrist counts as straight; the pose is then met to within this angle [rad]
WRIST_SINGULAR = 1e-6
# a pose this far [m] beyond the arm's reach is still solved, at full stretch
REACH_TOLERANCE = 1e-9


def wrap(angle):
    """Return the angle [rad] in (-pi, pi]."""
    return math.pi - (math.pi - angle) % (2.0 * math.pi)


@dataclass(frozen=True)
class Branch:
    """One joint solution of a pose."""

    joints: tuple   # waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate [rad]
    waist: str      # 'front': facing the wrist centre; 'back': turned away from it
    elbow: str      # 'up' or 'down': the side of the shoulder-wrist line the elbow is on
    wrist: str      # '+' or '-': the sign of the wrist angle

    @property
    def name(self):
        return f'{self.waist}/{self.elbow}/{self.wrist}'


class Unreachable(Exception):
    """The pose has no joint solution, whatever the joint limits."""


class AnalyticIK:
    """Forward and inverse kinematics of a vx300s-type arm."""

    def __init__(self, robot_des):
        """
        Read the arm's dimensions.

        :param robot_des: Modern Robotics description with `Slist` and `M`, from
            `interbotix_xs_modules.xs_robot.mr_descriptions`
        :raises ValueError: if the arm is not built like the vx300s
        """
        self.Slist, self.M = np.array(robot_des.Slist, float), np.array(robot_des.M, float)
        axes = self.Slist[:3].T
        expected = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]]
        if self.Slist.shape != (6, 6) or not np.allclose(axes, expected):
            raise ValueError('no closed-form inverse kinematics for this arm')
        # a point on each axis that is along y, from v = -w x q: q = (v_z, 0, -v_x)
        shoulder, elbow, wrist = (
            np.array([self.Slist[5, i], -self.Slist[3, i]]) for i in (1, 2, 4))
        if not np.allclose(self.M[:3, :3], np.eye(3)) or abs(wrist[1] - elbow[1]) > 1e-9 \
                or abs(shoulder[0]) > 1e-9 or abs(self.M[2, 3] - wrist[1]) > 1e-9:
            raise ValueError('no closed-form inverse kinematics for this arm')
        self._height = shoulder[1]                      # shoulder axis above the base plane
        upper = elbow - shoulder                        # shoulder to elbow at the home pose
        self._upper = float(np.linalg.norm(upper))
        self._upper_angle = math.atan2(upper[1], upper[0])
        self._forearm = float(wrist[0] - elbow[0])      # elbow to wrist centre
        self._tool = float(self.M[0, 3] - wrist[0])     # wrist centre to the end effector

    def fk(self, joints):
        """Return the end-effector pose (4x4, base frame) of a joint configuration."""
        return mr.FKinSpace(self.M, self.Slist, np.array(joints, float))

    def wrist_centre(self, T):
        """Return the point where the three wrist axes meet when the end effector is at T."""
        return T[:3, 3] - self._tool * T[:3, 0]

    def check_reach(self, T):
        """
        Check that the wrist centre of a pose is within the arm's reach.

        :raises Unreachable: saying whether the pose is too far from the shoulder or too
            close to it, and by how much
        """
        centre = self.wrist_centre(T)
        distance = math.hypot(math.hypot(centre[0], centre[1]), centre[2] - self._height)
        longest, shortest = self._upper + self._forearm, abs(self._upper - self._forearm)
        if distance > longest + REACH_TOLERANCE:
            raise Unreachable(f'{(distance - longest) * 1000:.0f} mm beyond full reach '
                              f'(wrist centre {distance:.3f} m from the shoulder, at most '
                              f'{longest:.3f} m)')
        if distance < shortest - REACH_TOLERANCE:
            raise Unreachable(f'{(shortest - distance) * 1000:.0f} mm too close to the '
                              f'shoulder (wrist centre {distance:.3f} m from it, at least '
                              f'{shortest:.3f} m)')
        return distance

    def solve(self, T, reference=None):
        """
        Find every joint solution of a pose, without regard to joint limits.

        Two joint angles are free in special poses and are then taken from `reference`: the
        waist when the wrist centre is on the waist axis, and the forearm roll when the wrist
        is straight (only the sum of forearm roll and wrist rotation is fixed there).

        :param T: desired end-effector pose (4x4, base frame)
        :param reference: (optional) joint positions to take the free angles from; zero
            otherwise
        :return: list of `Branch`, at most eight, each with every angle in (-pi, pi]
        :raises Unreachable: if the wrist centre is out of the arm's reach
        """
        T = np.array(T, float)
        reference = [0.0] * 6 if reference is None else [float(q) for q in reference]
        distance = self.check_reach(T)
        centre = self.wrist_centre(T)
        radius, height = math.hypot(centre[0], centre[1]), centre[2] - self._height
        heading = reference[0] if radius < SINGULAR else math.atan2(centre[1], centre[0])
        cosine = (distance ** 2 - self._upper ** 2 - self._forearm ** 2) / (
            2.0 * self._upper * self._forearm)
        bend = math.acos(min(1.0, max(-1.0, cosine)))   # angle between upper arm and forearm

        branches = []
        for waist, q1, r in (('front', heading, radius), ('back', heading + math.pi, -radius)):
            # seen with the waist turned away, the same bend puts the elbow on the other side
            above = bend if waist == 'front' else -bend
            for elbow, between in (('up', above), ('down', -above)):
                if elbow == 'down' and bend < SINGULAR:
                    continue    # at full stretch both elbow solutions are the same
                # directions in the vertical plane through the waist axis, measured from the
                # horizontal: upper arm at `up`, forearm at `up - between`
                up = math.atan2(height, r) + math.atan2(
                    self._forearm * math.sin(between),
                    self._upper + self._forearm * math.cos(between))
                q2 = self._upper_angle - up
                q3 = between - self._upper_angle
                for wrist, q4, q5, q6 in self._wrist(T, q1, q2 + q3, reference[3]):
                    joints = tuple(wrap(q) for q in (q1, q2, q3, q4, q5, q6))
                    branches.append(Branch(joints, waist, elbow, wrist))
        return branches

    def _wrist(self, T, q1, pitch, roll_reference):
        """Return the wrist solutions (name, q4, q5, q6) for a forearm pointing at `pitch`."""
        c1, s1, c, s = math.cos(q1), math.sin(q1), math.cos(pitch), math.sin(pitch)
        forearm = np.array([[c1 * c, -s1, c1 * s], [s1 * c, c1, s1 * s], [-s, 0.0, c]])
        R = forearm.T @ T[:3, :3]   # = Rx(q4) Ry(q5) Rx(q6)
        sine = math.hypot(R[1, 0], R[2, 0])
        if sine < WRIST_SINGULAR:
            if R[0, 0] < 0:
                return []   # the wrist folded back on the forearm: half a turn, never allowed
            total = math.atan2(R[2, 1], R[1, 1])
            return [('+', roll_reference, 0.0, total - roll_reference)]
        q5 = math.atan2(sine, R[0, 0])
        return [
            ('+', math.atan2(R[1, 0], -R[2, 0]), q5, math.atan2(R[0, 1], R[0, 2])),
            ('-', math.atan2(-R[1, 0], R[2, 0]), -q5, math.atan2(-R[0, 1], -R[0, 2])),
        ]

    def solve_within(self, T, lower, upper, reference=None):
        """
        Find the joint solutions of a pose that respect joint limits.

        :param lower: lowest position [rad] of each joint
        :param upper: highest position [rad] of each joint
        :return: (branches inside the limits; the others, each with the name of the first
            joint that is outside)
        :raises Unreachable: if the wrist centre is out of the arm's reach
        """
        inside, outside = [], []
        for branch in self.solve(T, reference):
            beyond = [i for i, q in enumerate(branch.joints) if not lower[i] <= q <= upper[i]]
            if beyond:
                outside.append((branch, beyond[0]))
            else:
                inside.append(branch)
        return inside, outside
