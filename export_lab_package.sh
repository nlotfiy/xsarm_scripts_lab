#!/usr/bin/env bash
# Export the lab copy of xsarm_scripts: the committed state (git HEAD) of the development repo,
# without the documentation, the tests and the development/hardware-check tools.
#
#   ./export_lab_package.sh [SOURCE_REPO] [OUTPUT_DIR]
#
# SOURCE_REPO defaults to ~/interbotix_ws/src/xsarm_scripts, OUTPUT_DIR to ./xsarm_scripts next
# to this script. OUTPUT_DIR is replaced. The package keeps the name xsarm_scripts, so every
# `ros2 run xsarm_scripts ...` / `ros2 launch xsarm_scripts ...` command stays the same.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="${1:-$HOME/interbotix_ws/src/xsarm_scripts}"
OUT="${2:-$HERE/xsarm_scripts}"

EXCLUDE=(
  # documentation
  docs
  xsarm_scripts/sim_harness/README.md
  # tests and test tools
  test
  xsarm_scripts/sim_harness
  xsarm_scripts/acceptance.py
  xsarm_scripts/collision_prone_moves.py
  # hardware checks used to set the supervisor limits
  xsarm_scripts/traj_timing_check.py
  xsarm_scripts/limit_probe.py
  xsarm_scripts/stop_distance_check.py
  xsarm_scripts/current_profile.py
  xsarm_scripts/current_stop_check.py
  xsarm_scripts/feedback_loss_check.py
  # offline tools that produced config/vx300s_capsules.yaml and config/vx300s_acm.yaml
  # (they need Pinocchio/Coal, which the lab computers do not have)
  xsarm_scripts/supervisor/build_acm.py
  xsarm_scripts/supervisor/fit_capsules.py
  # GUIs and teleoperation (the lab computers run the supervisor, scripts and work envelope)
  xsarm_scripts/joint_gui.py
  xsarm_scripts/analytical_ik_gui.py
  xsarm_scripts/analytical_ik.py
  xsarm_scripts/ik_gui.py
  xsarm_scripts/moveit_ik_gui.py
  xsarm_scripts/xsarm_jog.py
  launch/joint_gui.launch.py
  launch/analytical_ik_gui.launch.py
  launch/analytical_ik_gui_sim_unsafe.launch.py
  launch/xsarm_jog.launch.py
  config/modes_jog.yaml
  # launch files moved to launch/archive, and the Gazebo/perception bin-picking demo
  launch/fk.launch.py
  launch/ik_gui.launch.py
  launch/moveit_ik_gui.launch.py
  launch/bin_pick_place_sim.launch.py
  xsarm_scripts/bin_pick_place_demo.py
  config/bin_pick_place_filter_params.yaml
  config/bin_pick_place_static_transforms.yaml
  worlds
  rviz
)

# entry points of the excluded scripts, removed from setup.py
EXCLUDED_ENTRY_POINTS=(
  sim_harness acceptance_run collision_prone_moves traj_timing_check limit_probe
  stop_distance_check current_profile current_stop_check feedback_loss_check bin_pick_place_demo
  joint_gui analytical_ik_gui ik_gui moveit_ik_gui xsarm_jog
)

COMMIT="$(git -C "$SRC" rev-parse --short HEAD)"
rm -rf "$OUT"
mkdir -p "$OUT"
git -C "$SRC" archive HEAD | tar -x -C "$OUT"
for path in "${EXCLUDE[@]}"; do
  rm -rf "${OUT:?}/$path"
done

for ep in "${EXCLUDED_ENTRY_POINTS[@]}"; do
  sed -i "/^ *'$ep = /d" "$OUT/setup.py"
done
sed -i "/worlds\/\*\.world/d; /rviz\/\*\.rviz/d; /tests_require=/d" "$OUT/setup.py"

# dependencies of what is left (no tests, GUIs, MoveIt or joystick); rosdep installs them
python3 - "$OUT/package.xml" <<'EOF2'
import re, sys
path = sys.argv[1]
deps = ['rclpy', 'launch', 'launch_ros', 'ament_index_python', 'std_msgs', 'std_srvs',
        'sensor_msgs', 'geometry_msgs', 'interbotix_xs_msgs', 'interbotix_xs_modules',
        'interbotix_common_modules', 'interbotix_xs_sdk', 'interbotix_xsarm_control',
        'interbotix_xsarm_descriptions', 'urdfdom_py', 'xacro', 'python3-numpy',
        'python3-scipy', 'python3-yaml']
text = open(path).read()
text = re.sub(r'\n(\s*<(exec|test)_depend>[^<]*</(exec|test)_depend>\n?)+\s*\n(?=\s*<export>)',
              '\n\n' + ''.join(f'  <exec_depend>{d}</exec_depend>\n' for d in deps) + '\n',
              text)
assert '<exec_depend>rclpy</exec_depend>' in text and '<test_depend>' not in text, 'package.xml'
open(path, 'w').write(text)
EOF2

# every module that stays must not import one that was removed, or a dependency dropped above
REMOVED='joint_gui|analytical_ik|ik_gui|moveit_ik_gui|xsarm_jog|sim_harness|build_acm|fit_capsules|bin_pick_place_demo|acceptance|collision_prone_moves|traj_timing_check|limit_probe|stop_distance_check|current_profile|current_stop_check|feedback_loss_check|tkinter|tf_transformations|interbotix_moveit_interface'
if grep -rnE "^\s*(from|import) \S*($REMOVED)|executable='($REMOVED)'" --include=*.py "$OUT"; then
  echo "ERROR: the lines above use something that was removed" >&2
  exit 1
fi

echo "$COMMIT" > "$OUT/LAB_EXPORT_COMMIT"
echo "Exported xsarm_scripts at commit $COMMIT to $OUT"
