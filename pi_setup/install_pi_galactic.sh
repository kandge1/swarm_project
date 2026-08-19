#!/usr/bin/env bash
# =============================================================================
# mycobot 280 Pi -- ROS2 Galactic robotics stack installer
#
# Target: Ubuntu 20.04 (Focal), ROS2 Galactic -- the vendor (elephantrobotics)
# OS image already installed on this Pi, alongside the factory ROS1 Noetic
# install at /opt/ros/noetic. This script only touches the Galactic side and
# does not modify the Noetic install. Not switching OS/distro on this robot --
# it stays on the vendor image; see WORKFLOW.md for why.
#
# Installs what pick_place.py / annulus_test.py / WORKFLOW.md need beyond the
# mycobot OS image: MoveIt2, RViz2, ros2_control, and the wrist camera
# driver. Does NOT install moveit_py -- pick_place.py and friends were
# refactored to talk to move_group purely over plain ROS2 services/actions
# (/plan_kinematic_path, /check_state_validity, /compute_ik, etc.), so no
# moveit_py bindings are needed at all, on this distro or any other.
#
# Does NOT install Gazebo or Isaac Sim -- those are dev-workstation-only
# (see mars's robotics_stack_install.sh), and this Pi has no GPU for them
# anyway.
#
# Run as your regular user (sudo will be called where needed).
# =============================================================================

set -euo pipefail
LOGFILE="$HOME/mycobot_ros2_stack_install.log"
exec > >(tee -a "$LOGFILE") 2>&1

echo "=============================================="
echo " mycobot Pi ROS2 Galactic stack installer -- $(date)"
echo "=============================================="

step() { echo; echo "──────────────────────────────────────────────"; echo "  STEP: $*"; echo "──────────────────────────────────────────────"; }

# ─── 0. SANITY CHECKS ─────────────────────────────────────────────────────────
step "0. Checking system prerequisites"

version=$(lsb_release -rs)
if [[ "$version" != "20.04" ]]; then
    echo "ERROR: expected Ubuntu 20.04 (Focal). Detected: Ubuntu $version"
    exit 1
fi
echo "[OK] Ubuntu $version detected."

if [[ ! -d /opt/ros/galactic ]]; then
    echo "ERROR: /opt/ros/galactic not found. This script assumes ROS2 Galactic"
    echo "is already installed (part of the vendor mycobot OS image). Aborting."
    exit 1
fi
echo "[OK] ROS2 Galactic found at /opt/ros/galactic."

# `set -u` above and ROS's setup.bash do not get along: its line 8 tests
# $AMENT_TRACE_SETUP_FILES with no default, which nounset treats as fatal, so
# the script aborts here having installed nothing --
#   /opt/ros/galactic/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# Turn nounset off across the source and straight back on. Do not "tidy" this.
set +u
source /opt/ros/galactic/setup.bash
set -u

sudo apt update


# ─── 1. MOVEIT2 + RVIZ2 ────────────────────────────────────────────────────────
step "1. Installing MoveIt2 + RViz2"

sudo apt install -y \
    ros-galactic-moveit \
    ros-galactic-moveit-planners \
    ros-galactic-moveit-ros-planning \
    ros-galactic-moveit-ros-move-group \
    ros-galactic-backward-ros \
    ros-galactic-moveit-ros-visualization \
    ros-galactic-moveit-kinematics \
    ros-galactic-moveit-configs-utils \
    ros-galactic-moveit-setup-assistant \
    ros-galactic-warehouse-ros-mongo \
    ros-galactic-rviz2 \
    ros-galactic-rviz-common \
    ros-galactic-rviz-default-plugins \
    ros-galactic-xacro \
    ros-galactic-robot-state-publisher \
    ros-galactic-joint-state-publisher \
    ros-galactic-joint-state-publisher-gui \
    ros-galactic-tf2-ros

echo "[OK] MoveIt2 + RViz2 installed. (No moveit_py -- not needed; see header.)"


# ─── 2. ROS2_CONTROL ───────────────────────────────────────────────────────────
step "2. Installing ros2_control"

sudo apt install -y \
    ros-galactic-ros2-control \
    ros-galactic-ros2-controllers \
    ros-galactic-controller-manager \
    ros-galactic-joint-state-broadcaster \
    ros-galactic-joint-trajectory-controller

echo "[OK] ros2_control installed."


# ─── 3. WRIST CAMERA ───────────────────────────────────────────────────────────
step "3. Installing wrist camera driver"

sudo apt install -y \
    ros-galactic-v4l2-camera \
    ros-galactic-cv-bridge \
    ros-galactic-vision-opencv \
    python3-opencv

echo "[OK] Camera stack installed."


# ─── 4. pymycobot VERSION CHECK ────────────────────────────────────────────────
step "4. Checking pymycobot version"

python3 -m pip install --user -r "$(dirname "$0")/requirements.txt"

python3 -c "
import pymycobot
from packaging import version
v = pymycobot.__version__
print('pymycobot version:', v)
if version.parse(v) < version.parse('3.6.1'):
    print('[WARN] pymycobot is older than 3.6.1 -- upgrade with:')
    print('       pip3 install -U pymycobot')
else:
    print('[OK] pymycobot version meets the >=3.6.1 requirement.')
"


# ─── 5. COLCON WORKSPACE TOOLS ─────────────────────────────────────────────────
step "5. colcon build tools"

sudo apt install -y python3-colcon-common-extensions python3-rosdep

echo "[OK] colcon build tools installed."


# ─── 6. VERIFY ─────────────────────────────────────────────────────────────────
step "6. Verifying the install"

# Check what actually landed rather than assuming apt did what was asked. This
# is the same read-only check you would run by hand before colcon build, and it
# names any package that is still missing.
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -x "$WS_ROOT/pi_setup/preflight_check.sh" ]]; then
    "$WS_ROOT/pi_setup/preflight_check.sh" || echo "[WARN] Preflight reported problems -- see above."
else
    echo "[WARN] pi_setup/preflight_check.sh not found or not executable; skipping."
fi

echo
echo "=============================================="
echo " INSTALL COMPLETE"
echo "=============================================="
cat <<SUMMARY

  IMPORTANT -- read before running pick_place.py on this robot:

  1. This installed onto ROS2 Galactic at /opt/ros/galactic. Your existing
     ROS1 Noetic install at /opt/ros/noetic is untouched -- don't source both
     in the same terminal (they conflict). Use:
       source /opt/ros/galactic/setup.bash
     in the terminal(s) you use for this project.

  2. No moveit_py needed or installed -- pick_place.py, reset_arm.py,
     collision_contacts.py, and annulus_test.py all talk to move_group over
     plain services/actions (/plan_kinematic_path, /check_state_validity,
     /compute_ik, /compute_cartesian_path), which work on any MoveIt2 distro.

  3. Real hardware IS wired up now, via the mycobot_hardware package
     (src/mycobot_hardware/): a ros2_control SystemInterface plugin
     (mycobot_hardware/MyCobotSystem) bridging to a pymycobot-based Python
     process (mycobot_bridge.py) over a Unix domain socket. Known gaps,
     read before trusting it against the physical arm:
       - Gripper contact detection (pick_place.py's
         gripper_close_until_contact) CANNOT work as-is: pymycobot's
         gripper API has no effort/force reading at all.
       - Joint velocity is always reported as 0.0 -- pymycobot exposes no
         velocity reading. A placeholder, not a bug; controllers here only
         rely on position tracking.
     Since confirmed on real hardware: mycobot_hardware builds clean on
     Galactic and mycobot_bridge.py talks to the arm at /dev/ttyAMA0 @
     1000000 baud. Note that is ttyAMA0, NOT /dev/serial0 -- on a Pi 4
     /dev/serial0 points at the mini UART unless Bluetooth is disabled, so
     a script hardcoding serial0 may open a port the arm is not on and
     silently read back stale angles.
     See WORKFLOW.md's "Real Hardware Workflow" section for the full story.

  4. Next steps:
       cd ~/swarm_project
       source /opt/ros/galactic/setup.bash
       ./pi_setup/preflight_check.sh     # confirms this install actually took
       colcon build
       source install/setup.bash
       ros2 launch mycobot_280pi_camera_moveit2 real_robot.launch.py

     Run preflight_check.sh before colcon build on a robot you have not built
     on before. It reports a missing ROS source or a missing apt package as a
     one-line fix; colcon reports the same two things as CMake stack traces
     that name neither cause (GitHub issue #22).

  5. Gazebo/Isaac Sim were intentionally skipped -- dev-workstation-only,
     and this Pi needs to run everything standalone without them.

SUMMARY
