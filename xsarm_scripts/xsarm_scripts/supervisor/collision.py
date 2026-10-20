"""Self-collision clearance of the arm from its capsule model (task 1.9)."""

import itertools
import os

from ament_index_python.packages import get_package_share_directory
import numpy as np
from xsarm_scripts.supervisor.geometry import HELD
import yaml

# links an object in the gripper touches by being held, so it is not checked against them
GRIPPER_LINKS = ('gripper_link', 'gripper_prop_link', 'gripper_bar_link', 'left_finger_link',
                 'right_finger_link')


def segment_distances(p1, q1, p2, q2):
    """
    Return the distance between each segment p1-q1 and the matching segment p2-q2.

    All arguments are (n, 3) arrays. Closest points are found as in Ericson, "Real-Time
    Collision Detection", 5.1.9, for all pairs at once.
    """
    tiny = 1e-12
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.maximum(np.einsum('ij,ij->i', d1, d1), tiny)
    e = np.maximum(np.einsum('ij,ij->i', d2, d2), tiny)
    b = np.einsum('ij,ij->i', d1, d2)
    c = np.einsum('ij,ij->i', d1, r)
    f = np.einsum('ij,ij->i', d2, r)
    denom = a * e - b * b
    s = np.where(denom > tiny, np.clip((b * f - c * e) / np.maximum(denom, tiny), 0, 1), 0.0)
    t = (b * s + f) / e
    s = np.where(t < 0, np.clip(-c / a, 0, 1), np.where(t > 1, np.clip((b - c) / a, 0, 1), s))
    t = np.clip(t, 0, 1)
    return np.linalg.norm(r + d1 * s[:, None] - d2 * t[:, None], axis=1)


class SelfCollisionModel:
    """Capsules on the arm's links, and the pairs of links to keep apart."""

    def __init__(self, geometry, capsules, allowed):
        """
        Construct the model.

        :param geometry: `RobotGeometry` of the arm
        :param capsules: {link: {'capsules': [{'p0', 'p1', 'radius'}, ...]}}, link frame, as
            written by `fit_capsules`
        :param allowed: pairs of link names that are never checked, as written by `build_acm`
        """
        self._geometry = geometry
        allowed = {frozenset(pair) for pair in allowed}
        frame = {name.split('/')[-1]: name for name in geometry.order}
        self._links = sorted(capsules)
        self._frames = [frame[link] for link in self._links]
        first = np.cumsum([0] + [len(capsules[link]['capsules']) for link in self._links])
        flat = [c for link in self._links for c in capsules[link]['capsules']]
        self._link_of = np.repeat(np.arange(len(self._links)), np.diff(first))
        self._p0 = np.array([c['p0'] for c in flat], float)
        self._p1 = np.array([c['p1'] for c in flat], float)
        radius = np.array([c['radius'] for c in flat], float)
        # every capsule of one link against every capsule of the other, for each checked pair
        pairs = [(i, j)
                 for a, b in itertools.combinations(range(len(self._links)), 2)
                 if frozenset((self._links[a], self._links[b])) not in allowed
                 for i in range(first[a], first[a + 1]) for j in range(first[b], first[b + 1])]
        self._i, self._j = (np.array(side) for side in zip(*pairs))
        self._radii = radius[self._i] + radius[self._j]
        self._radius = radius
        self._not_gripper = np.flatnonzero(
            [self._links[link] not in GRIPPER_LINKS for link in self._link_of])
        # the fingers are checked at their widest opening, whatever the gripper is doing
        self._fingers = {}
        for joint in geometry.urdf.joints:
            if joint.type == 'prismatic':
                self._fingers[joint.name] = max(
                    (joint.limit.lower, joint.limit.upper), key=abs)

    @classmethod
    def load(cls, geometry, robot_model):
        """Construct the model from the capsule and ACM files installed with the package."""
        config = os.path.join(get_package_share_directory('xsarm_scripts'), 'config')
        with open(os.path.join(config, f'{robot_model}_capsules.yaml')) as f:
            capsules = yaml.safe_load(f)[robot_model]
        with open(os.path.join(config, f'{robot_model}_acm.yaml')) as f:
            allowed = [entry['links'] for entry in yaml.safe_load(f)[robot_model]['allowed']]
        return cls(geometry, capsules, allowed)

    def clearance(self, q, held=None):
        """
        Find the smallest distance between the capsules of any checked pair of links.

        :param q: dict of joint name -> position; missing joints are taken as 0
        :param held: (optional) object in the gripper, as `{'frame', 'p0', 'p1', 'radius'}`:
            a capsule in the frame of that link. It is checked against every link except
            those of the gripper, and reported as 'held_object'
        :return: (distance [m], negative if the capsules overlap; link name; link name)
        """
        transforms = self._geometry.link_transforms({**q, **self._fingers})
        rotation = np.array([transforms[frame][:3, :3] for frame in self._frames])
        origin = np.array([transforms[frame][:3, 3] for frame in self._frames])
        p0 = np.einsum('nij,nj->ni', rotation[self._link_of], self._p0) + origin[self._link_of]
        p1 = np.einsum('nij,nj->ni', rotation[self._link_of], self._p1) + origin[self._link_of]
        distance = segment_distances(
            p0[self._i], p1[self._i], p0[self._j], p1[self._j]) - self._radii
        k = int(np.argmin(distance))
        nearest = (float(distance[k]), self._links[self._link_of[self._i[k]]],
                   self._links[self._link_of[self._j[k]]])
        if held is None:
            return nearest
        h0, h1 = self._geometry.held_segment(transforms, held)
        others = self._not_gripper
        to_object = segment_distances(
            p0[others], p1[others], np.tile(h0, (len(others), 1)),
            np.tile(h1, (len(others), 1))) - self._radius[others] - held['radius']
        k = int(np.argmin(to_object))
        if to_object[k] < nearest[0]:
            return float(to_object[k]), self._links[self._link_of[others[k]]], HELD
        return nearest
