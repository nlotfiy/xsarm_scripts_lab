"""
Work envelope of the arm: a box in the base frame that every moving link has to stay inside.

The envelope is a setting of the arm's computer, not of a script. It is kept in
`~/.config/xsarm_scripts/work_envelope.yaml` (or the file named by `XSARM_WORK_ENVELOPE`) and
written by `ros2 run xsarm_scripts work_envelope`. `arm.launch.py` runs `envelope_server` next
to the driver, which publishes that file (`station_message`), and every `SupervisedManipulator`
uses what it publishes (`read_station_message`), on whichever computer the script runs. See
docs/work_envelope.md for the procedure.
"""

import os
import socket

import numpy as np
import yaml

AXES = 'xyz'
# number of points checked along the move between the home and sleep poses
REST_SAMPLES = 20


def envelope_path():
    """Return the file the envelope of this machine is kept in."""
    return os.environ.get('XSARM_WORK_ENVELOPE') or os.path.expanduser(
        '~/.config/xsarm_scripts/work_envelope.yaml')


def load_envelope(path=None):
    """
    Read the work envelope.

    :param path: (optional) file to read instead of this machine's envelope
    :return: `{'min': [x, y, z], 'max': [x, y, z]}` in metres, or `None` if no envelope is set
    :raises ValueError: if the file does not describe a box
    """
    path = path or envelope_path()
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return checked_envelope(data, path)


def station_message(path=None):
    """
    Describe the envelope file of this computer, as `envelope_server` publishes it.

    :param path: (optional) file to describe instead of this machine's envelope
    :return: YAML text with `host`, `file`, and `envelope` (a box, or null when none is set) or
        `error` (why the file is not a box)
    """
    path = path or envelope_path()
    message = {'host': socket.gethostname(), 'file': path}
    try:
        message['envelope'] = load_envelope(path)
    except (OSError, ValueError, yaml.YAMLError) as error:
        message['error'] = str(error)
    return yaml.safe_dump(message, default_flow_style=None, sort_keys=False)


def read_station_message(text):
    """
    Read a message of `envelope_server`.

    :return: `(box or None, host, file)`
    :raises ValueError: if the station's file is not a box, or the message is not one of
        `station_message`
    """
    try:
        message = yaml.safe_load(text)
        host, path = message['host'], message['file']
    except (yaml.YAMLError, TypeError, KeyError):
        raise ValueError(f'not a work envelope message: {text!r}') from None
    if 'error' in message:
        raise ValueError(f"work envelope of {host} ({path}): {message['error']}")
    box = message.get('envelope')
    return (None if box is None else checked_envelope(box, f'{host}:{path}')), host, path


def checked_envelope(data, source='work envelope'):
    """Return `{'min', 'max'}` as lists of three floats, or raise `ValueError`."""
    try:
        box = {key: [float(v) for v in data[key]] for key in ('min', 'max')}
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"{source}: needs 'min' and 'max', each a list of x, y, z") from None
    if any(len(box[key]) != 3 for key in box) or not np.all(np.isfinite(box['min'] + box['max'])):
        raise ValueError(f"{source}: 'min' and 'max' must each be three finite numbers")
    if any(low >= high for low, high in zip(box['min'], box['max'])):
        raise ValueError(f"{source}: every 'min' must be below its 'max'")
    return box


def save_envelope(box, path=None, taught_points=()):
    """Write the work envelope, with the points it was taught from as a record."""
    path = path or envelope_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write('# Work envelope of the arm, written by `ros2 run xsarm_scripts work_envelope`.\n'
                '# A box in the base frame [m]; the supervisor refuses any move that takes a\n'
                '# moving link outside it (SUP_WORK_ENVELOPE). Delete this file, or run\n'
                '# `work_envelope clear`, to remove the envelope.\n')
        yaml.safe_dump({
            'min': [round(float(v), 4) for v in box['min']],
            'max': [round(float(v), 4) for v in box['max']],
            'taught_points': [[round(float(v), 4) for v in point] for point in taught_points],
        }, f, sort_keys=False, default_flow_style=None)
    return path


def excess(geometry, box, q, held=None):
    """
    Measure how far the arm reaches outside the envelope.

    :param q: dict of joint name -> position
    :param held: (optional) object in the gripper, as for `RobotGeometry.held_lowest`
    :return: (distance [m] outside the box on the worst side, negative when the arm is
        inside by that much; axis 'x', 'y' or 'z'; side 'min' or 'max'; the link there)
    """
    low, high, low_link, high_link = (
        geometry.extent(q) if held is None else geometry.extent(q, held))
    sides = [(box['min'][i] - low[i], AXES[i], 'min', low_link[i]) for i in range(3)]
    sides += [(high[i] - box['max'][i], AXES[i], 'max', high_link[i]) for i in range(3)]
    distance, axis, side, link = max(sides, key=lambda entry: entry[0])
    return float(distance), axis, side, link


def rest_path(joint_names, sleep_positions):
    """Return joint dicts along the straight joint-space move from home to the sleep pose."""
    return [dict(zip(joint_names, [q * k / REST_SAMPLES for q in sleep_positions]))
            for k in range(REST_SAMPLES + 1)]


def grow_to_contain(geometry, box, poses, clearance=0.01):
    """
    Enlarge a box until the arm is inside it, with some clearance, at every one of `poses`.

    :param poses: joint dicts
    :return: the enlarged `{'min', 'max'}` and the names of the sides that moved (e.g. 'x min')
    """
    low, high = np.array(box['min'], float), np.array(box['max'], float)
    for q in poses:
        pose_low, pose_high, _, _ = geometry.extent(q)
        low, high = np.minimum(low, pose_low - clearance), np.maximum(high, pose_high + clearance)
    moved = [f'{AXES[i]} min' for i in range(3) if low[i] < box['min'][i]]
    moved += [f'{AXES[i]} max' for i in range(3) if high[i] > box['max'][i]]
    return {'min': low.tolist(), 'max': high.tolist()}, moved
