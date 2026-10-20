#!/usr/bin/env python3
"""Slider-based GUI for controlling VX300S joint angles."""

import threading
import tkinter as tk
from tkinter import ttk

import numpy as np

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator


JOINTS = [
    ('waist',        -180.0, 180.0),
    ('shoulder',     -101.0, 101.0),
    ('elbow',        -101.0,  92.0),
    ('forearm_roll', -180.0, 180.0),
    ('wrist_angle',  -107.0, 130.0),
    ('wrist_rotate', -180.0, 180.0),
]


class JointControlGUI:
    def __init__(self, bot: SupervisedManipulator):
        self.bot = bot

        self.root = tk.Tk()
        self.root.title('VX300S Joint Control')
        self.root.resizable(False, False)

        self._sliders: dict = {}
        self._labels: dict = {}
        self._tsb_labels: list = []
        self._build_ui()
        self._sync_T_sb()  # populate matrix with initial home-pose values

    def _build_ui(self):
        # ── Joint sliders (left) ──────────────────────────────────────────────
        slider_frame = ttk.LabelFrame(self.root, text='Joint Angles (degrees)', padding=10)
        slider_frame.grid(row=0, column=0, padx=(15, 5), pady=(15, 5), sticky='nsew')

        for row, (name, lo, hi) in enumerate(JOINTS):
            ttk.Label(slider_frame, text=name, width=14, anchor='e').grid(
                row=row, column=0, padx=(0, 8))

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(slider_frame, from_=lo, to=hi, variable=var,
                               orient='horizontal', length=360)
            slider.grid(row=row, column=1, pady=3)

            lbl = ttk.Label(slider_frame, text='  0.0°', width=8)
            lbl.grid(row=row, column=2, padx=(8, 0))

            var.trace_add('write', lambda *_, v=var, l=lbl: l.config(text=f'{v.get():+.1f}°'))
            slider.bind('<ButtonRelease-1>', lambda e, n=name, v=var: self._send_joint(n, v))

            self._sliders[name] = var
            self._labels[name] = lbl

        # ── T_sb matrix (right) ──────────────────────────────────────────────
        tsb_frame = ttk.LabelFrame(self.root, text='T_sb  —  End-Effector Pose', padding=10)
        tsb_frame.grid(row=0, column=1, padx=(5, 15), pady=(15, 5), sticky='nsew')

        # Row labels: [ R | p ]
        #             [ 0 | 1 ]
        MATRIX_FONT = ('Courier', 10)
        for i in range(4):
            row_labels = []
            for j in range(4):
                lbl = ttk.Label(tsb_frame, text=' 0.0000', width=9,
                                anchor='e', font=MATRIX_FONT)
                lbl.grid(row=i, column=j, padx=4, pady=3)
                row_labels.append(lbl)

            # Separator between rotation (cols 0-2) and translation (col 3)
            ttk.Separator(tsb_frame, orient='vertical').grid(
                row=i, column=3, sticky='ns', padx=(2, 0))

            self._tsb_labels.append(row_labels)

        # Horizontal separator between rotation rows (0-2) and bottom row (3)
        ttk.Separator(tsb_frame, orient='horizontal').grid(
            row=3, column=0, columnspan=4, sticky='ew', pady=(2, 0))

        # Column headers
        for j, h in enumerate(['x', 'y', 'z', 'p']):
            ttk.Label(tsb_frame, text=h, anchor='center',
                      font=('Courier', 9, 'italic')).grid(row=4, column=j)

        # ── Buttons (bottom, spanning both columns) ───────────────────────────
        btn_frame = ttk.Frame(self.root, padding=(15, 5, 15, 15))
        btn_frame.grid(row=1, column=0, columnspan=2, sticky='ew')
        btn_frame.columnconfigure((0, 1), weight=1)

        ttk.Button(btn_frame, text='Home Pose',
                   command=self._go_home).grid(row=0, column=0, padx=5, sticky='ew')
        ttk.Button(btn_frame, text='Sleep Pose',
                   command=self._go_sleep).grid(row=0, column=1, padx=5, sticky='ew')

    # ── Sync helpers ──────────────────────────────────────────────────────────

    def _sync_sliders(self):
        """Read joint states from the robot and push values back to the sliders."""
        by_name = dict(zip(
            self.bot.arm.group_info.joint_names, self.bot.arm.get_joint_positions()))
        for name, var in self._sliders.items():
            if name in by_name:
                self.root.after(0, var.set, round(np.rad2deg(by_name[name]), 2))

    def _sync_T_sb(self):
        """Read T_sb from the arm and update the matrix display."""
        T = self.bot.arm.T_sb
        for i in range(4):
            for j in range(4):
                text = f'{T[i, j]:+.4f}'
                self.root.after(0, self._tsb_labels[i][j].config, {'text': text})

    # ── Commands ──────────────────────────────────────────────────────────────

    def _background(self, command):
        """Run a motion command off the GUI thread; a refusal shows in the title bar."""
        def _run():
            try:
                command()
                self.root.after(0, self.root.title, 'VX300S Joint Control')
            except MotionRejected as rejection:
                self.root.after(0, self.root.title, f'Refused: {rejection.code}')
            self._sync_sliders()
            self._sync_T_sb()
        threading.Thread(target=_run, daemon=True).start()

    def _send_joint(self, joint_name: str, var: tk.DoubleVar):
        angle_rad = np.deg2rad(var.get())
        self._background(lambda: self.bot.arm.set_single_joint_position(joint_name, angle_rad))

    def _go_home(self):
        self._background(self.bot.arm.go_to_home_pose)

    def _park(self):
        # the supervisor only folds the arm into its sleep pose from the home pose
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()

    def _go_sleep(self):
        self._background(self._park)

    def run(self):
        self.root.mainloop()


def main():
    bot = SupervisedManipulator(robot_model='vx300s')
    robot_startup()
    try:
        bot.arm.go_to_home_pose()
        JointControlGUI(bot).run()
    except KeyboardInterrupt:
        pass
    except MotionRejected as rejection:
        bot.get_node().get_logger().error(f'Stopped: {rejection}')
    finally:
        try:
            bot.arm.go_to_home_pose()
            bot.arm.go_to_sleep_pose()
        except MotionRejected as rejection:
            bot.get_node().get_logger().error(f'Not parked: {rejection}')
        robot_shutdown()


if __name__ == '__main__':
    main()
