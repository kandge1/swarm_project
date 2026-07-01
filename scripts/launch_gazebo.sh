#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_WS="${ROS2_WS:-$PROJECT_ROOT/source/ros2_ws}"

if [ ! -d "$ROS2_WS" ]; then
  FALLBACK_WS="$HOME/ros2_ws"
  if [ -d "$FALLBACK_WS" ]; then
    ROS2_WS="$FALLBACK_WS"
  else
    echo "ROS 2 workspace not found at $ROS2_WS or $FALLBACK_WS" >&2
    exit 1
  fi
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if [ -f "$ROS2_WS/install/setup.bash" ]; then
  set +u
  source "$ROS2_WS/install/setup.bash"
  set -u
fi

cd "$ROS2_WS"
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json gz sim -v 4 empty.sdf 2>&1 | tee /tmp/gz_log3.txt

