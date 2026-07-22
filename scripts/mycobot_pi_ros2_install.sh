#!/usr/bin/env bash
# =============================================================================
# mycobot 280 Pi -- ROS2 Galactic robotics stack installer
#
# Target: Ubuntu 20.04 (Focal), ROS2 Galactic -- already present at
# /opt/ros/galactic on this robot, alongside the factory ROS1 Noetic install
# at /opt/ros/noetic. This script only touches the Galactic side and does not
# modify the Noetic install.
#
# Installs what pick_place.py / myscript.txt need beyond the mycobot OS image:
# MoveIt2 (+ moveit_py bindings), RViz2, ros2_control, and the wrist camera
# driver. Does NOT install Gazebo or Isaac Sim -- those are dev-workstation-
# only and this Pi has no GPU for them anyway.
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
    echo "is already installed. Aborting."
    exit 1
fi
echo "[OK] ROS2 Galactic found at /opt/ros/galactic."

source /opt/ros/galactic/setup.bash
sudo apt update


# ─── 1. moveit_py AVAILABILITY CHECK (before installing anything else) ────────
step "1. Checking moveit_py availability for Galactic"

# pick_place.py does `from moveit.planning import MoveItPy`. moveit_py landed
# late in MoveIt2's life; Galactic (EOL Nov 2022) may never have gotten a
# prebuilt apt package for it. Check now, before sinking time into the rest
# of the stack.
if apt-cache show ros-galactic-moveit-py >/dev/null 2>&1; then
    echo "[OK] ros-galactic-moveit-py is available via apt."
    MOVEIT_PY_AVAILABLE=1
else
    echo "[WARN] ros-galactic-moveit-py NOT found in apt."
    echo "       pick_place.py's 'from moveit.planning import MoveItPy' import"
    echo "       will fail even after this script finishes. Options:"
    echo "         a) build moveit_py from source against Galactic (heavy,"
    echo "            possibly unsupported for this EOL distro)"
    echo "         b) port pick_place.py off moveit_py onto the older"
    echo "            moveit_commander/MoveGroupCommander API, available on"
    echo "            Galactic"
    echo "         c) move this robot to a newer ROS2 distro (Humble+) via an"
    echo "            OS reflash -- bigger and riskier, don't do this without"
    echo "            deciding it deliberately"
    echo "       Continuing with the rest of the stack regardless -- RViz/"
    echo "       ros2_control/camera are useful on their own."
    MOVEIT_PY_AVAILABLE=0
fi


# ─── 2. MOVEIT2 + RVIZ2 ────────────────────────────────────────────────────────
step "2. Installing MoveIt2 + RViz2"

sudo apt install -y \
    ros-galactic-moveit \
    ros-galactic-moveit-planners \
    ros-galactic-moveit-ros-planning \
    ros-galactic-moveit-ros-move-group \
    ros-galactic-moveit-ros-visualization \
    ros-galactic-moveit-kinematics \
    ros-galactic-moveit-configs-utils \
    ros-galactic-moveit-setup-assistant \
    ros-galactic-warehouse-ros-mongo \
    ros-galactic-rviz2 \
    ros-galactic-rviz-common \
    ros-galactic-rviz-default-plugins

if [[ "$MOVEIT_PY_AVAILABLE" -eq 1 ]]; then
    sudo apt install -y ros-galactic-moveit-py
fi

echo "[OK] MoveIt2 + RViz2 installed."


# ─── 3. ROS2_CONTROL ───────────────────────────────────────────────────────────
step "3. Installing ros2_control"

sudo apt install -y \
    ros-galactic-ros2-control \
    ros-galactic-ros2-controllers \
    ros-galactic-controller-manager \
    ros-galactic-joint-state-broadcaster \
    ros-galactic-joint-trajectory-controller

echo "[OK] ros2_control installed."


# ─── 4. WRIST CAMERA ───────────────────────────────────────────────────────────
step "4. Installing wrist camera driver"

sudo apt install -y \
    ros-galactic-v4l2-camera \
    ros-galactic-cv-bridge \
    ros-galactic-vision-opencv \
    python3-opencv

echo "[OK] Camera stack installed."


# ─── 5. pymycobot VERSION CHECK ────────────────────────────────────────────────
step "5. Checking pymycobot version"

python3 -c "
import pymycobot
from packaging import version
v = pymycobot.__version__
print('pymycobot version:', v)
if version.parse(v) < version.parse('3.6.1'):
    print('[WARN] pymycobot is older than 3.6.1 -- listen_real.py/sync_plan.py')
    print('       will refuse to run. Upgrade with: pip3 install -U pymycobot')
else:
    print('[OK] pymycobot version meets the >=3.6.1 requirement.')
" || echo "[WARN] pymycobot not importable -- pip3 install --user pymycobot"


# ─── 6. COLCON WORKSPACE TOOLS ─────────────────────────────────────────────────
step "6. colcon build tools"

sudo apt install -y python3-colcon-common-extensions python3-rosdep

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

  2. moveit_py availability: $( [[ "$MOVEIT_PY_AVAILABLE" -eq 1 ]] && echo "FOUND -- ros-galactic-moveit-py installed." || echo "NOT FOUND -- see the WARN in step 1. pick_place.py will not import as-is." )

  3. This does NOT wire the real motors up to ros2_control. The
     firefighter.ros2_control.xacro hardware section only knows the
     Gazebo-sim and RViz-mock plugins -- installing this stack alone will
     not move the physical arm. That needs a real hardware_interface plugin
     (or a topic-bridge script following listen_real.py's pymycobot
     pattern), which is a separate piece of work.

  4. Gazebo/Isaac Sim were intentionally skipped -- dev-workstation-only,
     and this Pi has no GPU for them anyway.

SUMMARY
