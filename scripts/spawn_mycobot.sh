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

set +u

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
fi

set -u

URDF_FILE="$ROS2_WS/src/mycobot_ros2/mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi_with_gripper_gazebo.urdf"
if [ ! -f "$URDF_FILE" ]; then
  echo "URDF not found at $URDF_FILE" >&2
  exit 1
fi

cd "$ROS2_WS"
set +u
colcon build --packages-select mycobot_description && source install/setup.bash
set -u

ros2 run ros_gz_sim create -world empty \
  -file "$URDF_FILE" \
  -name mycobot
