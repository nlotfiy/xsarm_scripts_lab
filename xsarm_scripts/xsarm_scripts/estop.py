#!/usr/bin/env python3

import argparse
import sys
import termios
import threading
import tty

from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RobotInfo
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from xsarm_scripts.global_key import GlobalKey
from xsarm_scripts.supervisor.supervisor import ESTOP_QOS, estop_topic

"""
Software e-stop for the arm (task 2.8 in docs/supervisor_steps.md): a key that makes the arm
hold where it is, with torque on, until the stop is reset.

Run it in a terminal of its own, next to whatever moves the arm:

    ros2 run xsarm_scripts estop                 # F5 stops, from any window
    ros2 run xsarm_scripts estop --key Escape    # another key

    F5      stop, whichever window has the focus: the arm holds where it is, and every
            supervised script refuses to move it
    SPACE   stop, in this terminal
    r       reset, in this terminal only: supervised scripts may move the arm again (the
            move that was stopped is not resumed)
    q       quit (a stop stays in force in the scripts that are running)

While it runs, the stop key belongs to the e-stop on the whole desktop (an X11 key grab); it
goes back to other programs when the e-stop quits: meanwhile F5 does nothing in them (no
browser refresh, no run in VS Code), and an F5 meant for one of them stops the arm. Reset is
deliberately in this terminal only, so that a stray key press elsewhere cannot release the
arm.

On a stop this program commands the hold itself, from the arm's measured position, so the arm
stops even if the script moving it is not supervised or has hung. The stop latches: it stays
in force until r is pressed, also in supervised scripts started after it was pressed.
It is not a hardware e-stop: it does nothing if the computer, the driver or the USB link to
the arm has failed. Keep the power switch within reach.
"""

ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
KEYS = '''
    {key:<7s} stop, from any window: the arm holds, and supervised scripts refuse to move it
    SPACE   stop, in this terminal
    r       reset, in this terminal only: supervised scripts may move the arm again
    q       quit (a stop stays in force in the scripts that are running)

Ready.'''


class EStop:
    """The e-stop's state, the hold it commands, and the keys that change them."""

    def __init__(self, node, robot_name, joints=ARM_JOINTS, say=print):
        self.joints, self.say = list(joints), say
        self.pressed = None   # not known until this program presses or resets it
        self.positions = None
        self.lock = threading.Lock()
        self.state = node.create_publisher(Bool, estop_topic(robot_name), ESTOP_QOS)
        self.commands = node.create_publisher(
            JointGroupCommand, f'/{robot_name}/commands/joint_group', 10)
        node.create_subscription(
            JointState, f'/{robot_name}/joint_states', self._joint_states, 10)

    def _joint_states(self, message):
        index = {name: i for i, name in enumerate(message.name)}
        if all(joint in index for joint in self.joints):
            with self.lock:
                self.positions = [message.position[index[j]] for j in self.joints]

    def press(self):
        """Stop: command a hold where the arm is, and tell the supervised scripts."""
        self.state.publish(Bool(data=True))
        with self.lock:
            positions = self.positions
        if positions is None:
            self.say('STOPPED, but no joint states have arrived, so no hold was commanded. '
                     'Supervised scripts stop their moves.')
        else:
            self.commands.publish(JointGroupCommand(name='arm', cmd=positions))
            self.say('STOPPED: the arm is holding. Press r to reset.')
        self.pressed = True

    def reset(self):
        """Reset: supervised scripts may move the arm again."""
        self.state.publish(Bool(data=False))
        self.pressed = False
        self.say('Reset: the arm may be moved again. The stop key stops it.')

    def key(self, key):
        """
        Act on one key press.

        :return: False when the program should end
        """
        if key == ' ':
            self.press()
        elif key == 'r':
            self.reset()
        elif key in ('q', '\x03', '\x04'):   # q, Ctrl+C, Ctrl+D
            if self.pressed:
                self.say('Quitting with the stop in force: running scripts stay stopped until '
                         'an estop is started again and r is pressed.')
            return False
        return True


def read_keys(handle):
    """Feed single key presses from the terminal to `handle` until it returns False."""
    descriptor = sys.stdin.fileno()
    saved = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)   # keys arrive at once, without Enter; Ctrl+C is a key here
        attributes = termios.tcgetattr(descriptor)
        attributes[3] &= ~termios.ISIG
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        while handle(sys.stdin.read(1)):
            pass
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, saved)


def main():
    parser = argparse.ArgumentParser(description='Software e-stop for the arm.')
    parser.add_argument('--robot-name', default='vx300s')
    parser.add_argument('--key', default='F5',
                        help='key that stops the arm from any window (an X11 key name, such '
                             'as F5, Escape or Pause)')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        sys.exit('estop needs a terminal of its own to read the keys from.')
    rclpy.init()
    node = rclpy.create_node('estop', namespace=args.robot_name)
    joints = ARM_JOINTS
    info = node.create_client(RobotInfo, f'/{args.robot_name}/get_robot_info')
    if info.wait_for_service(timeout_sec=3.0):
        future = info.call_async(RobotInfo.Request(cmd_type='group', name='arm'))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if future.result() is not None:
            joints = list(future.result().joint_names)
    else:
        print('The driver does not answer yet; using the vx300s joint names.')
    estop = EStop(node, args.robot_name, joints)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        grab = GlobalKey(args.key, lambda: estop.key(' '))
        print(KEYS.format(key=args.key))
    except (OSError, RuntimeError) as problem:
        grab = None
        print(f'*** {args.key} cannot stop the arm from other windows: {problem}.\n'
              '*** Only SPACE in this terminal stops it.')
        print(KEYS.format(key='-'))
    try:
        read_keys(estop.key)
    finally:
        if grab is not None:
            grab.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
