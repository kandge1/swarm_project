#!/usr/bin/env bash
# =============================================================================
# Robotics Stack Installer — Ubuntu 24.04 LTS (Noble Numbat)
# Target stack:
#   - NVIDIA Driver 580-open (RTX 40-series / Ada Lovelace)
#   - Docker + NVIDIA Container Toolkit
#   - Isaac Sim 5.x via NVIDIA container (NGC)
#   - ROS2 Jazzy Jalisco (LTS, EOL May 2029)
#   - MoveIt2 + ros2_control
#   - RViz2, rqt, rosbag2
#
# NOTE: Run this on a FRESH Ubuntu 24.04 LTS install.
#       DO NOT run on Ubuntu 26.04 — Isaac Sim container does not support it yet.
#       Run as your regular user (sudo will be called where needed).
# =============================================================================

set -euo pipefail
LOGFILE="$HOME/robotics_install.log"
exec > >(tee -a "$LOGFILE") 2>&1

echo "=============================================="
echo " Robotics Stack Installer — $(date)"
echo "=============================================="

# ─── HELPERS ──────────────────────────────────────────────────────────────────
check_ubuntu_version() {
    local version
    version=$(lsb_release -rs)
    if [[ "$version" != "24.04" ]]; then
        echo "ERROR: This script targets Ubuntu 24.04. Detected: Ubuntu $version"
        echo "Isaac Sim container does NOT officially support 26.04 yet."
        echo "Please install Ubuntu 24.04 LTS and re-run."
        exit 1
    fi
    echo "[OK] Ubuntu $version detected."
}

step() { echo; echo "──────────────────────────────────────────────"; echo "  STEP: $*"; echo "──────────────────────────────────────────────"; }


# ─── 0. SANITY CHECKS ─────────────────────────────────────────────────────────
step "0. Checking system prerequisites"
check_ubuntu_version

# Confirm NVIDIA GPU is visible (basic check)
if ! lspci | grep -qi nvidia; then
    echo "WARNING: No NVIDIA GPU found via lspci. Proceeding anyway — check BIOS settings."
fi


# ─── 1. SYSTEM BASE UPDATE ────────────────────────────────────────────────────
step "1. Full system update"
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    curl wget gnupg2 software-properties-common \
    apt-transport-https ca-certificates lsb-release \
    build-essential git dkms \
    python3-pip python3-venv \
    x11-xserver-utils libvulkan1 mesa-vulkan-drivers \
    vulkan-tools


# ─── 2. NVIDIA DRIVER 580-open ────────────────────────────────────────────────

echo "[OK] nvidia-smi output:"
nvidia-smi


# ─── 3. DOCKER ────────────────────────────────────────────────────────────────
step "3. Installing Docker CE"

# Remove old docker if present
sudo apt remove -y docker docker-engine docker.io containerd runc || true

# Add Docker's official GPG key and repo
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add current user to docker group (avoids needing sudo for every docker cmd)
sudo usermod -aG docker "$USER"
sudo systemctl enable --now docker

echo "[OK] Docker installed. Version: $(docker --version)"


# ─── 4. NVIDIA CONTAINER TOOLKIT ──────────────────────────────────────────────
step "4. Installing NVIDIA Container Toolkit"

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure Docker to use NVIDIA runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Quick GPU sanity check inside container
echo "[TEST] Running GPU test inside container..."
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi && \
    echo "[OK] NVIDIA Container Toolkit working." || \
    echo "[WARN] Container GPU test failed. Check docker group membership (may need re-login)."


# ─── 5. ISAAC SIM CONTAINER SETUP ────────────────────────────────────────────
step "5. Setting up Isaac Sim container (NGC)"

# Allow X11 forwarding for Isaac Sim GUI from container
xhost +local:docker 2>/dev/null || echo "[INFO] xhost not available yet — run 'xhost +local:docker' in a desktop session before launching Isaac Sim GUI."

# ─── NGC CREDENTIALS ──────────────────────────────────────────────
cat <<'INSTRUCTIONS'

  ┌─────────────────────────────────────────────────────────────────┐
  │  NGC API KEY REQUIRED                                           │
  │                                                                 │
  │  1. Sign up / log in at: https://ngc.nvidia.com                │
  │  2. Go to: top-right menu → Setup → Generate API Key           │
  │  3. Copy the key, then run:                                     │
  │                                                                 │
  │     docker login nvcr.io                                        │
  │       Username: $oauthtoken                                     │
  │       Password: <your-ngc-api-key>                              │
  │                                                                 │
  │  4. Then pull Isaac Sim (this is ~20GB, grab a coffee):        │
  │                                                                 │
  │     docker pull nvcr.io/nvidia/isaac-sim:5.1.0                  │
  │                                                                 │
  └─────────────────────────────────────────────────────────────────┘

INSTRUCTIONS

# Write a launcher script for Isaac Sim
mkdir -p "$HOME/isaac_sim"
cat > "$HOME/isaac_sim/launch_isaac_sim.sh" <<'LAUNCHER'
#!/usr/bin/env bash
# Isaac Sim Docker Launcher
# Usage:
#   ./launch_isaac_sim.sh          — GUI mode (requires desktop session)
#   ./launch_isaac_sim.sh headless — headless / WebRTC streaming mode

ISAAC_VERSION="${ISAAC_SIM_VERSION:-5.1.0}"
ISAAC_IMAGE="nvcr.io/nvidia/isaac-sim:${ISAAC_VERSION}"

# Allow X11 from docker
xhost +local:docker 2>/dev/null || true

COMMON_ARGS=(
    --name isaac-sim
    --entrypoint bash
    --runtime=nvidia
    --gpus all
    -e "ACCEPT_EULA=Y"
    -e "PRIVACY_CONSENT=Y"
    -e DISPLAY="$DISPLAY"
    -e "OMNI_KIT_ALLOW_ROOT=1"
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v "$HOME/isaac_sim/cache/kit:/isaac-sim/kit/cache:rw"
    -v "$HOME/isaac_sim/cache/ov:/root/.cache/ov:rw"
    -v "$HOME/isaac_sim/cache/pip:/root/.cache/pip:rw"
    -v "$HOME/isaac_sim/cache/glcache:/root/.cache/nvidia/GLCache:rw"
    -v "$HOME/isaac_sim/cache/computecache:/root/.nv/ComputeCache:rw"
    -v "$HOME/isaac_sim/logs:/root/.nvidia-omniverse/logs:rw"
    -v "$HOME/isaac_sim/data:/root/.local/share/ov/data:rw"
    -v "$HOME/isaac_sim/documents:/root/Documents:rw"
    --network=host
    --rm
)

mkdir -p \
    "$HOME/isaac_sim/cache/kit" \
    "$HOME/isaac_sim/cache/ov" \
    "$HOME/isaac_sim/cache/pip" \
    "$HOME/isaac_sim/cache/glcache" \
    "$HOME/isaac_sim/cache/computecache" \
    "$HOME/isaac_sim/logs" \
    "$HOME/isaac_sim/data" \
    "$HOME/isaac_sim/documents"

if [[ "${1:-}" == "headless" ]]; then
    echo "Launching Isaac Sim in headless/WebRTC mode..."
    echo "Connect browser to: http://localhost:8211/streaming/webrtc-demo/?server=localhost"
    docker run "${COMMON_ARGS[@]}" \
        -p 8211:8211 \
        -it "$ISAAC_IMAGE" \
        -c "./runheadless.webrtc.sh"
else
    echo "Launching Isaac Sim GUI..."
    docker run "${COMMON_ARGS[@]}" \
        -it "$ISAAC_IMAGE" \
        -c "./isaac-sim.sh"
fi
LAUNCHER

chmod +x "$HOME/isaac_sim/launch_isaac_sim.sh"
echo "[OK] Isaac Sim launcher written to ~/isaac_sim/launch_isaac_sim.sh"


# ─── 6. ROS2 JAZZY JALISCO ───────────────────────────────────────────────────
step "6. Installing ROS2 Jazzy Jalisco (LTS)"

# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable Universe repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Add ROS2 apt repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
# ros-jazzy-desktop: ROS2 core + RViz2 + demos + rqt
sudo apt install -y \
    ros-jazzy-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete

# Initialize rosdep
sudo rosdep init 2>/dev/null || echo "[INFO] rosdep already initialized."
rosdep update

# Source ROS2 in bashrc
grep -qxF 'source /opt/ros/jazzy/setup.bash' "$HOME/.bashrc" || \
    echo 'source /opt/ros/jazzy/setup.bash' >> "$HOME/.bashrc"

echo "[OK] ROS2 Jazzy installed."


# ─── 7. MoveIt2 + ros2_control + tools ───────────────────────────────────────
step "7. Installing MoveIt2, ros2_control, RViz2, rqt, rosbag2"

sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-servo \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-robot-steering \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-compression \
    ros-jazzy-rosbag2-storage-default-plugins

echo "[OK] MoveIt2, ros2_control, RViz2, rqt, rosbag2 installed."


# ─── 8. ROS2 <-> Isaac Sim bridge note ───────────────────────────────────────
step "8. ROS2 Bridge for Isaac Sim"
cat <<'BRIDGE_NOTE'

  ┌─────────────────────────────────────────────────────────────────────┐
  │  ROS2 BRIDGE — IMPORTANT NOTE                                       │
  │                                                                     │
  │  Isaac Sim 5.x communicates with ROS2 via the IsaacSim ROS2 bridge  │
  │  running INSIDE the container. The bridge uses the host network     │
  │  (--network=host in the launcher) so ROS2 topics are visible        │
  │  directly on your host.                                             │
  │                                                                     │
  │  In Isaac Sim (inside container), enable the bridge via:            │
  │    Extensions → search "ROS2 Bridge" → Enable                       │
  │                                                                     │
  │  Then on HOST, verify topics appear:                                │
  │    ros2 topic list                                                  │
  │                                                                     │
  │  If RMW mismatch occurs, set on both host and container:            │
  │    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp                       │
  │                                                                     │
  └─────────────────────────────────────────────────────────────────────┘

BRIDGE_NOTE


# ─── 9. WORKSPACE SETUP ───────────────────────────────────────────────────────
step "9. Creating ROS2 colcon workspace"

mkdir -p "$HOME/ros2_ws/src"

cat > "$HOME/ros2_ws/build_ws.sh" <<'BUILD'
#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/ros2_ws"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "Workspace built. Source with: source ~/ros2_ws/install/setup.bash"
BUILD
chmod +x "$HOME/ros2_ws/build_ws.sh"

# Add workspace overlay to bashrc (after initial build)
grep -qxF 'source $HOME/ros2_ws/install/setup.bash 2>/dev/null' "$HOME/.bashrc" || \
    echo 'source $HOME/ros2_ws/install/setup.bash 2>/dev/null' >> "$HOME/.bashrc"

echo "[OK] Workspace at ~/ros2_ws"


# ─── 10. FINAL VERIFICATION ───────────────────────────────────────────────────
step "10. Final verification"

source /opt/ros/jazzy/setup.bash

echo "── nvidia-smi ──────────────────────────────"
nvidia-smi | head -20

echo "── Docker version ──────────────────────────"
docker --version

echo "── ROS2 version ────────────────────────────"
ros2 --version 2>/dev/null || echo "ROS2 not in PATH yet (source ~/.bashrc or re-login)"

echo "── MoveIt2 check ───────────────────────────"
dpkg -l | grep ros-jazzy-moveit | head -5

echo
echo "=============================================="
echo " INSTALL COMPLETE — Summary"
echo "=============================================="
cat <<'SUMMARY'

  NEXT STEPS:
  ──────────────────────────────────────────────────────────────────

  1. LOG OUT AND BACK IN (or run: newgrp docker)
     Needed for docker group membership to take effect.

  2. GET YOUR NGC API KEY:
     https://ngc.nvidia.com → Setup → Generate API Key
     Then: docker login nvcr.io

  3. PULL ISAAC SIM (~20GB):
     docker pull nvcr.io/nvidia/isaac-sim:5.1.0

  4. RUN COMPATIBILITY CHECKER:
     docker run --rm --runtime=nvidia --gpus all \
       -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
       nvcr.io/nvidia/isaac-sim-comp-check:5.0.0

  5. LAUNCH ISAAC SIM:
     ~/isaac_sim/launch_isaac_sim.sh            # GUI
     ~/isaac_sim/launch_isaac_sim.sh headless   # WebRTC (browser)

  6. SOURCE ROS2 + WORKSPACE:
     source ~/.bashrc
     # or manually:
     source /opt/ros/jazzy/setup.bash

  7. VERIFY ROS2 TOOLS:
     rviz2 &
     rqt &
     ros2 bag record /topic

  DRIVER NOTES:
  ──────────────────────────────────────────────────────────────────
  • GPU:     NVIDIA RTX 4060 Laptop (Ada Lovelace / AD107M)
  • Driver:  nvidia-driver-580-open (open kernel module)
  • Isaac:   Requires driver >= 580.65.06
  • If Isaac detects wrong driver: wipe ~/.cache/ov ~/.local/share/ov
    and relaunch. Common laptop GPU bug with stale cached driver info.

  ROS2 DISTRIBUTION NOTES:
  ──────────────────────────────────────────────────────────────────
  • Jazzy Jalisco: LTS release, EOL May 2029, runs on Ubuntu 24.04
  • MoveIt2:       Fully supported on Jazzy
  • Kilted Kaiju:  Latest non-LTS, also on 24.04 (EOL Nov 2026)
  • DO NOT use Rolling Ridley for production work

  WHY NOT UBUNTU 26.04:
  ──────────────────────────────────────────────────────────────────
  • Isaac Sim container officially supports only 22.04 / 24.04
  • Jazzy and Kilted only have debs for 24.04
  • 26.04 is brand new (April 2026) — ecosystem not ready

SUMMARY
