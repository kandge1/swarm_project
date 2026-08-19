#!/usr/bin/env bash
# =============================================================================
# swarm_project -- WORKSTATION installer (ROS2 Jazzy on Ubuntu 24.04)
#
# The workstation is the machine that does the PLANNING: move_group (IK, OMPL)
# and RViz run here, and the scripts you type (stack_blocks.py, tag_pick_place.py)
# run here. Trajectory execution stays on the robot -- see README.md section 2.
#
# This is the counterpart to pi_setup/install_pi_galactic.sh, which installs the
# robot side. Run one on each machine. They are not interchangeable: the robot is
# Ubuntu 20.04/Galactic (vendor image, ROS2 already present), while this script
# targets a FRESH Ubuntu 24.04 with no ROS at all and installs Jazzy itself.
#
# Installs: ROS2 Jazzy, MoveIt2, ros2_control, RViz2, Cyclone DDS, OpenCV and
# the colcon/rosdep tooling. Does NOT install Gazebo, Isaac Sim, Docker or the
# NVIDIA stack -- none of them are needed to fly the real arm, and a capstone
# group should not have to debug a GPU container to stack two blocks.
#
# Does NOT install moveit_py: nothing in this project imports it. Every script
# talks to move_group over plain services/actions (/plan_kinematic_path,
# /compute_ik, /check_state_validity, /compute_cartesian_path).
#
# Safe to re-run: every step is idempotent.
# Run as your regular user (sudo will be called where needed).
# =============================================================================

set -euo pipefail
LOGFILE="$HOME/swarm_workstation_install.log"
exec > >(tee -a "$LOGFILE") 2>&1

ROS_DISTRO_TARGET="jazzy"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=============================================="
echo " swarm_project workstation installer -- $(date)"
echo " Log: $LOGFILE"
echo "=============================================="

step() { echo; echo "──────────────────────────────────────────────"; echo "  STEP: $*"; echo "──────────────────────────────────────────────"; }

# ─── 0. SANITY CHECKS ─────────────────────────────────────────────────────────
step "0. Checking system prerequisites"

if [[ $EUID -eq 0 ]]; then
    echo "ERROR: run this as your regular user, not with sudo. The script calls"
    echo "sudo itself where it needs to; running the whole thing as root leaves"
    echo "pip files and ~/.bashrc owned by root."
    exit 1
fi

version=$(lsb_release -rs)
if [[ "$version" != "24.04" ]]; then
    echo "ERROR: expected Ubuntu 24.04 (Noble), which is what ROS2 Jazzy targets."
    echo "Detected: Ubuntu $version"
    echo
    echo "If you are on the ROBOT (Ubuntu 20.04), you want the other script:"
    echo "    ./pi_setup/install_pi_galactic.sh"
    exit 1
fi
echo "[OK] Ubuntu $version detected."

# A ROS1 install sourced in this shell poisons the ROS2 build environment.
if [[ -n "${ROS_VERSION:-}" && "${ROS_VERSION}" != "2" ]]; then
    echo "ERROR: a ROS1 environment is sourced in this shell (ROS_VERSION=$ROS_VERSION)."
    echo "Open a clean terminal and re-run. Do not source ROS1 and ROS2 together."
    exit 1
fi
echo "[OK] No conflicting ROS1 environment in this shell."


# ─── 1. BASE TOOLING + LOCALE ─────────────────────────────────────────────────
step "1. Base tooling, locale and the universe repo"

sudo apt update
sudo apt install -y curl wget gnupg2 lsb-release software-properties-common locales git

# ROS2 requires a UTF-8 locale. On a minimal image this is often not set.
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Many ros-jazzy-* packages depend on packages that live in universe.
sudo add-apt-repository universe -y

echo "[OK] Base tooling installed."


# ─── 2. ROS2 JAZZY ────────────────────────────────────────────────────────────
step "2. Installing ROS2 Jazzy"

if [[ -d "/opt/ros/${ROS_DISTRO_TARGET}" ]]; then
    echo "[SKIP] /opt/ros/${ROS_DISTRO_TARGET} already exists."
else
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi

# ros-jazzy-desktop pulls in ROS2 core, RViz2, rqt and the demo nodes.
sudo apt install -y \
    ros-${ROS_DISTRO_TARGET}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

echo "[OK] ROS2 ${ROS_DISTRO_TARGET} installed."


# ─── 3. MOVEIT2 + ROS2_CONTROL ────────────────────────────────────────────────
step "3. Installing MoveIt2 and ros2_control"

# ros2_control is installed here even though the hardware plugin runs on the
# robot: `ros2 control list_controllers` is how you verify the robot's
# controllers from this machine, and the MoveIt config's controller manager
# plugin is resolved locally by move_group.
sudo apt install -y \
    ros-${ROS_DISTRO_TARGET}-moveit \
    ros-${ROS_DISTRO_TARGET}-moveit-planners \
    ros-${ROS_DISTRO_TARGET}-moveit-ros-planning \
    ros-${ROS_DISTRO_TARGET}-moveit-ros-move-group \
    ros-${ROS_DISTRO_TARGET}-moveit-ros-visualization \
    ros-${ROS_DISTRO_TARGET}-moveit-kinematics \
    ros-${ROS_DISTRO_TARGET}-moveit-configs-utils \
    ros-${ROS_DISTRO_TARGET}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO_TARGET}-moveit-setup-assistant \
    ros-${ROS_DISTRO_TARGET}-backward-ros \
    ros-${ROS_DISTRO_TARGET}-ros2-control \
    ros-${ROS_DISTRO_TARGET}-ros2-controllers \
    ros-${ROS_DISTRO_TARGET}-controller-manager \
    ros-${ROS_DISTRO_TARGET}-joint-state-broadcaster \
    ros-${ROS_DISTRO_TARGET}-joint-trajectory-controller \
    ros-${ROS_DISTRO_TARGET}-rviz2 \
    ros-${ROS_DISTRO_TARGET}-xacro \
    ros-${ROS_DISTRO_TARGET}-robot-state-publisher \
    ros-${ROS_DISTRO_TARGET}-joint-state-publisher \
    ros-${ROS_DISTRO_TARGET}-joint-state-publisher-gui \
    ros-${ROS_DISTRO_TARGET}-tf2-ros

# backward-ros above is not decorative: without it move_group can die at startup
# with "libbackward.so: cannot open shared object file". See WORKFLOW.md.

echo "[OK] MoveIt2 + ros2_control installed."


# ─── 4. CYCLONE DDS ───────────────────────────────────────────────────────────
step "4. Installing Cyclone DDS"

# The default RMW cannot do static unicast discovery, and campus Wi-Fi blocks
# the multicast it relies on. swarm_network ships the Cyclone config.
sudo apt install -y ros-${ROS_DISTRO_TARGET}-rmw-cyclonedds-cpp

echo "[OK] Cyclone DDS installed."
echo "     NOTE: the peer IPs in swarm_network/config/*.xml are hardcoded and"
echo "     WILL be wrong for your machines. This script does not guess them."
echo "     See README.md section 6 -- wrong IPs fail silently."


# ─── 5. VISION ────────────────────────────────────────────────────────────────
step "5. Installing OpenCV and vision support"

# zone_vision.py needs cv2.aruco (AprilTag 36h11 decode). Ubuntu's python3-opencv
# ships the contrib modules that provide it. cv_bridge is used by the detector
# tooling; the detector itself runs on the robot.
sudo apt install -y \
    ros-${ROS_DISTRO_TARGET}-cv-bridge \
    ros-${ROS_DISTRO_TARGET}-vision-opencv \
    python3-opencv

echo "[OK] Vision stack installed."


# ─── 6. PIP-ONLY DEPENDENCIES ─────────────────────────────────────────────────
step "6. Installing pip-only dependencies"

# pillow is workstation-only: print_zone_tags.py / print_block_tags.py generate
# the printable mats, and printing happens here rather than on the robot. This
# is why it is NOT in pi_setup/requirements.txt.
#
# Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so a
# plain `pip install --user` is refused. Prefer the apt build; fall back to pip
# with the override only if apt has no candidate.
if sudo apt install -y python3-pil; then
    echo "[OK] pillow installed via apt (python3-pil)."
else
    python3 -m pip install --user --break-system-packages pillow
    echo "[OK] pillow installed via pip."
fi

# pymycobot is deliberately NOT installed here. It drives the arm's serial port,
# which only exists on the robot.

echo "[OK] pip-only dependencies handled."


# ─── 7. ROSDEP ────────────────────────────────────────────────────────────────
step "7. Initializing rosdep"

sudo rosdep init 2>/dev/null || echo "[INFO] rosdep already initialized."
rosdep update || echo "[WARN] rosdep update failed (usually transient network). Re-run later."

echo "[OK] rosdep ready."


# ─── 8. SHELL SETUP ───────────────────────────────────────────────────────────
step "8. Sourcing ROS2 from your shell"

# Sourcing is per-terminal and does not persist. Adding it to .bashrc is the
# difference between "it works" and the Findament_cmake.cmake error that opened
# GitHub issue #22 on the robot side.
BASHRC="$HOME/.bashrc"
SOURCE_LINE="source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
if grep -qxF "$SOURCE_LINE" "$BASHRC" 2>/dev/null; then
    echo "[SKIP] '$SOURCE_LINE' already in $BASHRC."
else
    echo "$SOURCE_LINE" >> "$BASHRC"
    echo "[OK] Appended to $BASHRC: $SOURCE_LINE"
fi

echo
echo "The DDS variables are NOT added automatically -- ROS_DOMAIN_ID has to match"
echo "the robot's, and both machines need the config filename for THEIR distro."
echo "Once the peer IPs are set (README section 6), add this block yourself:"
cat <<'DDSBLOCK'

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=42
    export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jazzy.xml

DDSBLOCK


# ─── 9. VERIFY ────────────────────────────────────────────────────────────────
step "9. Verifying the install"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"

if [[ -x "$WS_ROOT/pi_setup/preflight_check.sh" ]]; then
    "$WS_ROOT/pi_setup/preflight_check.sh" || echo "[WARN] Preflight reported problems -- see above."
else
    echo "[WARN] pi_setup/preflight_check.sh not found or not executable; skipping."
fi


# ─── SUMMARY ──────────────────────────────────────────────────────────────────
echo
echo "=============================================="
echo " WORKSTATION INSTALL COMPLETE"
echo "=============================================="
cat <<SUMMARY

  1. Open a NEW terminal (or 'source ~/.bashrc') so ROS2 is on your PATH.

  2. Build the workspace. ALWAYS skip mycobot_hardware here -- it is a
     Galactic-only ros2_control plugin and cannot compile on Jazzy. It
     failing on this machine is expected, not a regression:

       cd $WS_ROOT
       colcon build --packages-skip mycobot_hardware
       source install/setup.bash

  3. Set the DDS peer IPs before expecting the two machines to see each
     other. Both config files, then REBUILD swarm_network -- CYCLONEDDS_URI
     points into install/, so editing the source alone changes nothing:

       hostname -I                      # here, and on the robot
       # edit src/swarm_network/config/cyclonedds_jazzy.xml
       # edit src/swarm_network/config/cyclonedds_galactic.xml
       colcon build --packages-select swarm_network

     Wrong IPs fail SILENTLY: both machines look healthy alone and simply
     never see each other.

  4. Install the robot side too, on the arm itself:

       ./pi_setup/install_pi_galactic.sh

  5. Then follow README.md section 8 to run the colour stack.

  Not installed, on purpose: Gazebo, Isaac Sim, Docker, the NVIDIA stack,
  moveit_py, and pymycobot (robot-only). See the header for why.

SUMMARY
