# xsarm_scripts_lab

Lab-computer copy of `xsarm_scripts` for the Interbotix vx300s: the supervisor, the work
envelope, the software e-stop, the driver launch (`arm.launch.py` with `driver_guard`) and the
example scripts. No documentation, tests, GUIs or development tools; those stay in the
development repo `nlotfiy/xsarm_scripts`. `xsarm_scripts/LAB_EXPORT_COMMIT` names the
development commit this copy was made from.

## Install on a lab computer (Ubuntu 22.04, for every user)

    git clone https://github.com/nlotfiy/xsarm_scripts_lab.git
    cd xsarm_scripts_lab
    sudo ./install_lab_computer.sh --domain-id 11      # a different number on every computer

Everything goes in system locations (`/opt/ros/humble`, `/opt/interbotix_ws`,
`/etc/ros2_lab.bash`), so every account gets ROS 2 and the arm in each new terminal.
Re-run the same command after `git pull` to update.

**Work envelope:** one per lab computer, in `/etc/xsarm_scripts/work_envelope.yaml`. Every
user's scripts read it; only members of the group `xsarm-admin` may change it. Add instructors
with `--admin` (repeatable; they must log out and in again):

    sudo ./install_lab_computer.sh --domain-id 2 --admin nima

Then, as such an account, on the lab computer:

    ros2 run xsarm_scripts work_envelope set --min -0.30 -0.45 0.0 --max 0.60 0.45 0.70
    ros2 run xsarm_scripts work_envelope teach      # or by hand; needs the driver running
    ros2 run xsarm_scripts work_envelope show

## Run the arm

On the lab computer (the arm's USB cable is plugged in there):

    ros2 launch xsarm_scripts arm.launch.py robot_model:=vx300s
    ros2 run xsarm_scripts lab_template          # second terminal
    ros2 run xsarm_scripts estop                 # F5 stops the arm from any window

From another computer on the same network, a script reaches station 11 with:

    export ROS_DOMAIN_ID=11
    ros2 run xsarm_scripts lab_template

The work envelope that holds is always the station's: `arm.launch.py` publishes the lab
computer's envelope file, and the supervisor uses it wherever the script runs (its start-up log
says `Work envelope (from <lab computer>, /etc/xsarm_scripts/work_envelope.yaml): ...`). If the
driver was started some other way, the script warns and falls back to its own computer's file.

Both clocks must be NTP-synchronised (`timedatectl`): the supervisor stops on lost feedback
by the age of the joint states' time stamps.

## Uninstall (to test a clean install)

    sudo ./uninstall_ros.sh --user <account with the old ~/interbotix_ws install>

Removes ROS 2 Humble and everything `install_lab_computer.sh` put in place. With `--user`, also
the per-user install of that account: its `~/interbotix_ws` and `~/apriltag_ws` are renamed to
`*.removed-<date>` (delete them once the new install works), the ROS lines in its `~/.bashrc`
are commented out (backup `~/.bashrc.before-ros-uninstall`), and modern_robotics/numpy are
removed from its `~/.local`. `--keep-ros` keeps the ROS 2 apt packages.

## Update this repo from the development repo

    ./export_lab_package.sh            # from git HEAD of ~/interbotix_ws/src/xsarm_scripts
    git add -A && git commit -m "Export xsarm_scripts $(cat xsarm_scripts/LAB_EXPORT_COMMIT)"
    git push
