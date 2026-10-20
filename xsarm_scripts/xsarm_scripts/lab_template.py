#!/usr/bin/env python3

import sys

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator

"""
Starting point for lab scripts: copy this file and replace the motion in `run`.

All arm motion goes through the supervisor (`bot.arm`). A command it refuses raises
`MotionRejected` and the script stops with the arm holding its position.

Try it in simulation first:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s use_sim:=true
    ros2 run xsarm_scripts lab_template
"""


def run(bot):
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_components(x=0.3, z=0.3)
    bot.gripper.release()
    bot.gripper.grasp()
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()


def main():
    bot = SupervisedManipulator(robot_model='vx300s')
    robot_startup()
    try:
        run(bot)
    except MotionRejected as rejection:
        bot.get_node().get_logger().error(f'Stopped: {rejection}')
        sys.exit(1)
    finally:
        robot_shutdown()


if __name__ == '__main__':
    main()
