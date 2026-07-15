"""
Manual Script for running the pick and place demo with myCobot 280 MoveIt2.

cd ~/swarm/swarm_project/source/ros2_ws
python3 ~/swarm/swarm_project/scripts/gen_disable_collisions.py
colcon build --packages-select mycobot_280_moveit2

source install/setup.bash
ros2 launch mycobot_280_moveit2 demo.launch.py

in a new terminal

source install/setup.bash
ros2 run controller_manager spawner gripper_group_controller

in a new terminal
python3 ~/swarm/swarm_project/scripts/pick_place.py
"""

#!/usr/bin/env bash
set -euo pipefail

WS=~/swarm/swarm_project/source/ros2_ws
SCRIPTS=~/swarm/swarm_project/scripts
TMP=$(mktemp -d)

# --- One-time prep: fix collisions + build, before opening any terminals ---
cd "$WS"
python3 "$SCRIPTS/gen_disable_collisions.py"
colcon build --packages-select mycobot_280_moveit2

# --- Write each tab's commands to its own script file ---

cat > "$TMP/tab1.sh" <<EOF
cd "$WS"
source install/setup.bash
ros2 launch mycobot_280_moveit2 demo.launch.py
exec bash
EOF

cat > "$TMP/tab2.sh" <<EOF
cd "$WS"
source install/setup.bash
echo "Waiting for /controller_manager/list_controllers..."
until ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; do
  sleep 1
done
sleep 2
ros2 run controller_manager spawner gripper_group_controller
exec bash
EOF

cat > "$TMP/tab3.sh" <<EOF
cd "$WS"
source install/setup.bash
echo "Waiting for gripper_group_controller to be active..."
until ros2 control list_controllers 2>/dev/null | grep -q "gripper_group_controller.*active"; do
  sleep 1
done
sleep 1
python3 "$SCRIPTS/pick_place.py"
exec bash
EOF

chmod +x "$TMP/tab1.sh" "$TMP/tab2.sh" "$TMP/tab3.sh"

# --- Open each in its own gnome-terminal tab ---
gnome-terminal --tab --title="demo.launch.py" -- bash "$TMP/tab1.sh"
gnome-terminal --tab --title="gripper spawner" -- bash "$TMP/tab2.sh"
gnome-terminal --tab --title="pick_place" -- bash "$TMP/tab3.sh"