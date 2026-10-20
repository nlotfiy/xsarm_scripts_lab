#!/usr/bin/env python3

import argparse
import sys

import rclpy
from std_msgs.msg import String
from xsarm_scripts.supervisor.envelope import envelope_path, station_message
from xsarm_scripts.supervisor.supervisor import ENVELOPE_QOS, envelope_topic

"""
Publish the work envelope of this computer for the supervised scripts that drive its arm.

    ros2 run xsarm_scripts envelope_server --robot-name vx300s

`arm.launch.py` starts it next to the driver, so the envelope that holds for an arm is the one
set on the computer the arm is plugged into, whichever computer a script runs on. The file
(`XSARM_WORK_ENVELOPE`, else ~/.config/xsarm_scripts/work_envelope.yaml) is read again every
second and published again when it changes; the topic keeps the last message for scripts that
start later.
"""


class EnvelopeServer:
    """Publish `station_message()` on the robot's envelope topic whenever it changes."""

    def __init__(self, node, robot_name, path=None, period=1.0):
        self._node = node
        self._path = path or envelope_path()
        self._publisher = node.create_publisher(String, envelope_topic(robot_name), ENVELOPE_QOS)
        self._last = None
        self.check()
        node.create_timer(period, self.check)

    def check(self):
        text = station_message(self._path)
        if text == self._last:
            return
        self._last = text
        self._publisher.publish(String(data=text))
        self._node.get_logger().info('Work envelope published:\n' + text.strip())


def main():
    parser = argparse.ArgumentParser(description='Publish the work envelope of this computer.')
    parser.add_argument('--robot-name', default='vx300s')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = rclpy.create_node('envelope_server', namespace=args.robot_name)
    EnvelopeServer(node, args.robot_name)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
