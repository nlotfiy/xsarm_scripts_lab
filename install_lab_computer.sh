#!/usr/bin/env bash
# Install ROS 2 Humble, the Interbotix arm packages and the lab copy of xsarm_scripts on an
# Ubuntu 22.04 lab computer, for every user account on it.
#
#   sudo ./install_lab_computer.sh --domain-id N [--pkg DIR]
#
#   --domain-id N  ROS_DOMAIN_ID of this computer (1-101), different on every lab computer: an
#                  arm answers only to computers on its domain, so students cannot move each
#                  other's arms, and the instructor reaches station N with `export ROS_DOMAIN_ID=N`.
#   --pkg DIR      the exported xsarm_scripts package (default: ./xsarm_scripts next to this
#                  script, made by export_lab_package.sh)
#
# What goes where (all readable by every user, writable only by root):
#   /opt/ros/humble            ROS 2 (apt)
#   /opt/interbotix_ws         Interbotix sources + xsarm_scripts, built into install/
#   /usr/local/lib/python3.10  modern_robotics (pip, system-wide)
#   /etc/udev/rules.d          Interbotix udev rules (/dev/ttyDXL, serial ports readable by all)
#   /etc/ros2_lab.bash         environment, sourced for every user from /etc/bash.bashrc
#   /etc/xsarm_scripts/work_envelope.yaml   the arm's work envelope (shared by all users)
#
# Re-running the script is safe: it updates xsarm_scripts and rebuilds.
set -euo pipefail

WS=/opt/interbotix_ws
ENV_FILE=/etc/ros2_lab.bash
ENVELOPE=/etc/xsarm_scripts/work_envelope.yaml
HERE="$(cd "$(dirname "$0")" && pwd)"
PKG="$HERE/xsarm_scripts"
DOMAIN_ID=""

# the Interbotix commits the lab package was tested with
declare -A REPOS=(
  [interbotix_ros_core]="https://github.com/Interbotix/interbotix_ros_core.git af18d4fe24ba08e09a0f1e92afaca1863e3205de"
  [interbotix_ros_manipulators]="https://github.com/Interbotix/interbotix_ros_manipulators.git b66d5b905725351dd71d3251a06cd3f4c777940f"
  [interbotix_ros_toolboxes]="https://github.com/Interbotix/interbotix_ros_toolboxes.git 28bd7aa4d057ff6354ffd69f2cbae0defeb6128d"
  [moveit_visual_tools]="https://github.com/ros-planning/moveit_visual_tools.git 89aa0b65d842bd55f12f77760cfe97b8407745c0"
)

while [ $# -gt 0 ]; do
  case "$1" in
    --domain-id) DOMAIN_ID="$2"; shift 2 ;;
    --pkg) PKG="$(cd "$2" && pwd)"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
. /etc/os-release
[ "$VERSION_CODENAME" = jammy ] || { echo "ROS 2 Humble needs Ubuntu 22.04 (jammy)" >&2; exit 1; }
[[ "$DOMAIN_ID" =~ ^[0-9]+$ ]] && [ "$DOMAIN_ID" -ge 1 ] && [ "$DOMAIN_ID" -le 101 ] || \
  { echo "give this computer's ROS_DOMAIN_ID: --domain-id N (1-101)" >&2; exit 2; }
[ -f "$PKG/package.xml" ] || { echo "no package at $PKG (run export_lab_package.sh first)" >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive

echo "== 1. ROS 2 Humble (apt)"
apt-get update
apt-get install -y software-properties-common curl git locales
add-apt-repository -y universe
locale-gen en_US en_US.UTF-8
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  # the apt source package replaces the older ros2.list + keyring (whose key expired in 2025)
  rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F tag_name | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$V/ros2-apt-source_$V.${VERSION_CODENAME}_all.deb"
  apt-get install -y /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi
apt-get update
apt-get install -y ros-humble-desktop ros-dev-tools python3-rosdep python3-pip \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-yaml python3-transforms3d

echo "== 2. modern_robotics, system-wide (the Interbotix installer puts it in ~/.local)"
pip3 install --no-deps modern_robotics

echo "== 3. Interbotix sources in $WS"
mkdir -p "$WS/src"
for name in "${!REPOS[@]}"; do
  read -r url commit <<< "${REPOS[$name]}"
  [ -d "$WS/src/$name/.git" ] || git clone "$url" "$WS/src/$name"
  git -C "$WS/src/$name" fetch --quiet origin
  git -C "$WS/src/$name" checkout --quiet --force "$commit"
done
git -C "$WS/src/interbotix_ros_core" submodule update --init \
  interbotix_ros_xseries/dynamixel_workbench_toolbox interbotix_ros_xseries/interbotix_xs_driver
# same packages as the Interbotix installer builds (without perception)
rm -f "$WS/src/interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE" \
  "$WS/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE" \
  "$WS/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE"

echo "== 4. xsarm_scripts (lab copy) from $PKG"
rm -rf "$WS/src/xsarm_scripts"
cp -r "$PKG" "$WS/src/xsarm_scripts"

echo "== 5. udev rules"
cp "$WS/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules" \
  /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger

echo "== 6. dependencies (rosdep) and build"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update --rosdistro humble
set +u; . /opt/ros/humble/setup.bash; set -u
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# a plain (not --symlink-install) build: the installed files do not point back into src
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
chmod -R a+rX "$WS"

echo "== 7. environment for every user ($ENV_FILE)"
{
  echo "# ROS 2 and the Interbotix arm for every user; written by install_lab_computer.sh"
  echo "source /opt/ros/humble/setup.bash"
  echo "source $WS/install/setup.bash"
  echo "export ROS_DOMAIN_ID=$DOMAIN_ID"
  echo "export XSARM_WORK_ENVELOPE=$ENVELOPE"
} > "$ENV_FILE"
chmod 644 "$ENV_FILE"
# /etc/bash.bashrc is read by every interactive bash (every terminal window), for every user
grep -qF "$ENV_FILE" /etc/bash.bashrc || \
  printf '\n[ -f %s ] && . %s\n' "$ENV_FILE" "$ENV_FILE" >> /etc/bash.bashrc
mkdir -p "$(dirname "$ENVELOPE")"
chmod 755 "$(dirname "$ENVELOPE")"

echo "== 8. clock synchronisation"
# the supervisor judges lost feedback by the age of the joint states' time stamps; when a script
# runs on another computer the two clocks must agree, so keep NTP on
timedatectl set-ntp true
timedatectl | grep -E 'synchronized|NTP'
if command -v ufw >/dev/null && ufw status | grep -q 'Status: active'; then
  echo "NOTE: the ufw firewall is active; ROS 2 from other computers needs UDP $((7400 + 250 * DOMAIN_ID))-$((7400 + 250 * DOMAIN_ID + 249)) open"
fi

echo
echo "Done (ROS_DOMAIN_ID=$DOMAIN_ID). Open a new terminal (any user) and check:  ros2 pkg prefix xsarm_scripts"
[ -f "$ENVELOPE" ] || echo "No work envelope yet: install one at $ENVELOPE (see the instructions)."
