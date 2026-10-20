#!/usr/bin/env bash
# Remove ROS 2 Humble and the Interbotix packages from a lab computer, so that
# install_lab_computer.sh can be tested on a clean machine.
#
#   sudo ./uninstall_ros.sh [--user NAME]... [--keep-ros]
#
#   --user NAME  also undo the per-user Interbotix install (xsarm_amd64_install.sh) of account
#                NAME: its workspaces are renamed to <name>.removed-<date> (not deleted), the ROS
#                lines in its ~/.bashrc are commented out (backup: ~/.bashrc.before-ros-uninstall)
#                and modern_robotics/numpy are removed from its ~/.local. Repeat for each account.
#   --keep-ros   leave the ROS 2 apt packages installed; remove only the Interbotix/lab parts.
#
# Removed system-wide: /opt/interbotix_ws, /etc/ros2_lab.bash (and its line in
# /etc/bash.bashrc), the Interbotix udev rules, modern_robotics in /usr/local, and (unless
# --keep-ros) every ros-humble-* package, the ROS apt source and the rosdep configuration.
# The work envelope in /etc/xsarm_scripts is kept.
set -euo pipefail

USERS=()
KEEP_ROS=false
while [ $# -gt 0 ]; do
  case "$1" in
    --user) USERS+=("$2"); shift 2 ;;
    --keep-ros) KEEP_ROS=true; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
if pgrep -x xs_sdk >/dev/null; then
  echo "the arm driver (xs_sdk) is running; stop it first" >&2; exit 1
fi
STAMP=$(date +%Y%m%d-%H%M%S)

for user in "${USERS[@]}"; do
  home=$(getent passwd "$user" | cut -d: -f6)
  [ -n "$home" ] && [ -d "$home" ] || { echo "no account $user" >&2; exit 1; }
  echo "== per-user install of $user ($home)"
  for ws in interbotix_ws apriltag_ws; do
    if [ -d "$home/$ws" ]; then
      mv "$home/$ws" "$home/$ws.removed-$STAMP"
      echo "moved $home/$ws -> $home/$ws.removed-$STAMP"
    fi
  done
  if [ -f "$home/.bashrc" ]; then
    cp -p "$home/.bashrc" "$home/.bashrc.before-ros-uninstall"
    sed -i -E 's@^(\s*(source|\.) \S*(/opt/ros/|interbotix_ws/|apriltag_ws/)\S*setup\.bash.*)$@# \1@;
               s@^(\s*export ROS_(DOMAIN_ID|LOCALHOST_ONLY)=.*)$@# \1@' "$home/.bashrc"
    echo "commented out in $home/.bashrc:"
    diff "$home/.bashrc.before-ros-uninstall" "$home/.bashrc" | grep '^>' || true
  fi
  for pkg in modern_robotics numpy; do
    sudo -u "$user" -H python3 -m pip uninstall -y --quiet "$pkg" 2>/dev/null \
      && echo "removed $pkg from $home/.local" || true
  done
done

echo "== system-wide Interbotix and lab install"
rm -rf /opt/interbotix_ws
rm -f /etc/ros2_lab.bash /etc/udev/rules.d/99-interbotix-udev.rules
sed -i '\@/etc/ros2_lab.bash@d' /etc/bash.bashrc
udevadm control --reload-rules
pip3 uninstall -y --quiet modern_robotics 2>/dev/null || true

if ! $KEEP_ROS; then
  echo "== ROS 2 apt packages"
  export DEBIAN_FRONTEND=noninteractive
  apt-get purge -y 'ros-humble-*' ros-dev-tools ros2-apt-source || \
    apt-get purge -y 'ros-humble-*'
  apt-get autoremove -y
  rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list \
    /usr/share/keyrings/ros-archive-keyring.gpg
  rm -rf /etc/ros/rosdep
  apt-get update
fi

echo
echo "Done. Open a new terminal; 'ros2' should now be 'command not found'."
