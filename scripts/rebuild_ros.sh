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

cd "$ROS2_WS"
set +u
colcon build --packages-select mycobot_description
source install/setup.bash
set -u
