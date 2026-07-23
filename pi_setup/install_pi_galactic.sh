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

source /opt/ros/galactic/setup.bash
sudo apt update


# ─── 1. MOVEIT2 + RVIZ2 ────────────────────────────────────────────────────────
step "1. Installing MoveIt2 + RViz2"

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

  3. This does NOT wire the real motors up to ros2_control. The
     firefighter.ros2_control.xacro hardware section only knows the
     Gazebo-sim and RViz-mock plugins -- installing this stack alone will
     not move the physical arm. That needs a real hardware_interface plugin
     (or a topic-bridge script following elephantrobotics' pymycobot
     pattern), which is a separate piece of work.

  4. Next steps:
       cd ~/swarm/swarm_project
       source /opt/ros/galactic/setup.bash
       colcon build
       source install/setup.bash

  5. Gazebo/Isaac Sim were intentionally skipped -- dev-workstation-only,
     and this Pi needs to run everything standalone without them.

SUMMARY
