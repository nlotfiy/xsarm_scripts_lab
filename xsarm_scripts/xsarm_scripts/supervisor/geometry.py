"""Forward kinematics on the URDF link meshes, used to measure clearance above the table."""

import struct
import subprocess
import warnings
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
from urdf_parser_py.urdf import URDF


# name under which an object held in the gripper is reported
HELD = 'held_object'


def generate_urdf(robot_model, robot_name):
    """Run xacro on the Interbotix description of `robot_model` and return the URDF string."""
    share = get_package_share_directory('interbotix_xsarm_descriptions')
    return subprocess.run(
        ['xacro', f'{share}/urdf/{robot_model}.urdf.xacro', f'robot_name:={robot_name}',
         'use_world_frame:=true'],
        check=True, capture_output=True, text=True,
    ).stdout


def _origin(xyz, rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = rz @ ry @ rx
    T[:3, 3] = xyz
    return T


def _rotation(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


def _load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0]
    if 84 + 50 * n == len(data):  # binary STL
        tri = np.frombuffer(data, offset=84, count=n, dtype=np.dtype(
            [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        return tri['v'].reshape(-1, 3).astype(float)
    pts = [line.split()[1:4] for line in data.decode(errors='ignore').splitlines()
           if line.strip().startswith('vertex')]
    return np.array(pts, float)


def _hull(points):
    """Reduce to convex hull vertices; exact for distance-to-plane queries."""
    try:
        with warnings.catch_warnings():
            # this machine's scipy complains about the installed numpy on import
            warnings.simplefilter('ignore', UserWarning)
            from scipy.spatial import ConvexHull
        return points[ConvexHull(points).vertices]
    except Exception:
        return np.unique(points.round(5), axis=0)


class RobotGeometry:
    """Link meshes and kinematic tree of a robot, loaded from a URDF string."""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        for element in root.findall('ros2_control'):
            root.remove(element)  # not geometry, and the URDF parser prints a warning for it
        self.urdf = URDF.from_xml_string(ET.tostring(root))
        self.root = self.urdf.get_root()
        self.joint_of = {j.child: j for j in self.urdf.joints}
        self.order = []
        self._visit(self.root)
        self.points = {}  # link -> hull vertices in the link frame
        self._meshes = {}  # link -> (file, scale, origin transform or None)
        for link in self.urdf.links:
            geom = link.collision or link.visual
            if geom is None or not hasattr(geom.geometry, 'filename'):
                continue
            path = geom.geometry.filename
            if path.startswith('package://'):
                pkg, rel = path[len('package://'):].split('/', 1)
                path = f'{get_package_share_directory(pkg)}/{rel}'
            elif path.startswith('file://'):
                path = path[len('file://'):]
            scale = np.asarray(geom.geometry.scale or [1, 1, 1], float)
            T = None if geom.origin is None else _origin(geom.origin.xyz, geom.origin.rpy)
            self._meshes[link.name] = (path, scale, T)
            self.points[link.name] = _hull(self._vertices(link.name))
        # links rigidly attached to the root never move, so they are not clearance candidates
        self.fixed_links = {self.root}
        for name in self.order[1:]:
            j = self.joint_of[name]
            if j.type == 'fixed' and j.parent in self.fixed_links:
                self.fixed_links.add(name)
        self.moving_links = [name for name in self.points if name not in self.fixed_links]

    def reach_radii(self, joint_names, held=None):
        """
        Bound how far any point of the arm is from each joint's axis.

        Turning joint i by an angle `a` moves no point of the arm, nor of a held object, by
        more than `radius[i] * a`, whatever the other joints are doing. The bound for a point
        on a link is the length of the chain of joint origins from the joint to that link,
        plus the distance of the point from its link's origin.

        :param joint_names: the joints to bound, in order
        :param held: (optional) object in the gripper, as for `held_lowest`
        :return: list of radii [m]
        """
        parent_joint = {link: self.joint_of[link] for link in self.order[1:]}
        extent = {link: float(np.linalg.norm(points, axis=1).max())
                  for link, points in self.points.items()}
        if held is not None:
            frame = next(name for name in self.order if name.endswith('/' + held['frame']))
            extent[frame] = max(extent.get(frame, 0.0), held['radius'] + max(
                float(np.linalg.norm(held[end])) for end in ('p0', 'p1')))
        radii = dict.fromkeys(joint_names, 0.0)
        for link, reach in extent.items():
            # walk from the link towards the root, adding the offset of each joint passed
            while link in parent_joint:
                joint = parent_joint[link]
                if joint.name in radii:
                    radii[joint.name] = max(radii[joint.name], reach)
                offset = float(np.linalg.norm(joint.origin.xyz)) if joint.origin else 0.0
                if joint.type == 'prismatic':
                    offset += max(abs(joint.limit.lower), abs(joint.limit.upper))
                reach += offset
                link = joint.parent
        return [radii[name] for name in joint_names]

    def _vertices(self, link):
        path, scale, T = self._meshes[link]
        v = _load_stl(path) * scale
        return v if T is None else v @ T[:3, :3].T + T[:3, 3]

    def triangles(self, link):
        """Return the triangles of a link's mesh in the link frame, as an (n, 3, 3) array."""
        return self._vertices(link).reshape(-1, 3, 3)

    def _visit(self, name):
        self.order.append(name)
        for j in self.urdf.joints:
            if j.parent == name:
                self._visit(j.child)

    def link_transforms(self, q):
        """
        Compute the pose of every link in the root frame.

        :param q: dict of joint name -> position [rad or m]; missing joints are taken as 0
        :return: dict of link name -> 4x4 transform
        """
        T = {self.root: np.eye(4)}
        for name in self.order[1:]:
            j = self.joint_of[name]
            Tj = _origin(j.origin.xyz, j.origin.rpy) if j.origin is not None else np.eye(4)
            if j.type in ('revolute', 'continuous'):
                Tj = Tj @ _rotation(j.axis, q.get(j.name, 0.0))
            elif j.type == 'prismatic':
                Tp = np.eye(4)
                Tp[:3, 3] = np.asarray(j.axis, float) * q.get(j.name, 0.0)
                Tj = Tj @ Tp
            T[name] = T[j.parent] @ Tj
        return T

    def held_segment(self, T, held):
        """Return the end points, in the root frame, of the capsule of a held object."""
        frame = T[next(name for name in self.order if name.endswith('/' + held['frame']))]
        return (frame[:3, :3] @ np.asarray(held['p0'], float) + frame[:3, 3],
                frame[:3, :3] @ np.asarray(held['p1'], float) + frame[:3, 3])

    def held_lowest(self, q, held):
        """
        Find the lowest point of an object in the gripper.

        :param held: the object, as `{'frame', 'p0', 'p1', 'radius'}`: a capsule in the frame
            of that link
        :return: z [m] in the root frame
        """
        p0, p1 = self.held_segment(self.link_transforms(q), held)
        return float(min(p0[2], p1[2]) - held['radius'])

    def lowest_point(self, q):
        """
        Find the lowest point of the moving links.

        :param q: dict of joint name -> position
        :return: (z [m] in the root frame, link name)
        """
        T = self.link_transforms(q)
        best = (np.inf, None)
        for name in self.moving_links:
            z = float((self.points[name] @ T[name][2, :3] + T[name][2, 3]).min())
            if z < best[0]:
                best = (z, name)
        return best

    def extent(self, q, held=None):
        """
        Find the box, aligned with the root frame, that just contains the moving links.

        :param q: dict of joint name -> position
        :param held: (optional) object in the gripper, as for `held_lowest`
        :return: (lowest x, y, z [m]; highest x, y, z [m]; the link at each of the lowest;
            the link at each of the highest)
        """
        T = self.link_transforms(q)
        low, high = np.full(3, np.inf), np.full(3, -np.inf)
        low_link, high_link = [None] * 3, [None] * 3
        if held is not None:
            p0, p1 = self.held_segment(T, held)
            low, high = np.minimum(p0, p1) - held['radius'], np.maximum(p0, p1) + held['radius']
            low_link, high_link = [HELD] * 3, [HELD] * 3
        for name in self.moving_links:
            points = self.points[name] @ T[name][:3, :3].T + T[name][:3, 3]
            for axis in range(3):
                if points[:, axis].min() < low[axis]:
                    low[axis], low_link[axis] = points[:, axis].min(), name
                if points[:, axis].max() > high[axis]:
                    high[axis], high_link[axis] = points[:, axis].max(), name
        return low, high, low_link, high_link
