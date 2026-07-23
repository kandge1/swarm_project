# Swarm Project Workflow

## Setup (Do Once Per Terminal Session)

```bash
cd ~/swarm/swarm_project
source /opt/ros/jazzy/setup.bash
colcon build  # Only if code changed
source install/setup.bash
```

---

## RViz Workflow

### Terminal 1: Launch RViz and MoveIt
```bash
cd ~/swarm/swarm_project
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 demo.launch.py
```

### Terminal 2: Spawn Controllers
```bash
source ~/swarm/swarm_project/install/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_group_controller
ros2 run controller_manager spawner gripper_group_controller
ros2 control list_controllers
```

### Terminal 3: Run pick_place.py
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/pick_place.py
```

### Terminal 3 (Alt): Run annulus_show.py (Visualization)
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/annulus_show.py
```

### Terminal 4: Run annulus_test.py (Execution)
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/annulus_test.py --execute --yaw-min -130 --yaw-max +130
```

### Terminal 5: Launch Wrist Camera
(Install first: `sudo apt install ros-jazzy-v4l2-camera`)

```bash
source ~/swarm/swarm_project/install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 camera.launch.py
```

---

## Gazebo Workflow

### Pre-Launch Cleanup
```bash
pkill -9 -f "gz sim"
pkill -9 -f "parameter_bridge"
pkill -9 -f "move_group"
pkill -9 -f "rviz2"
pkill -9 -f "robot_state_publisher"
rm -rf /dev/shm/fastrtps_* /dev/shm/ros_*

# Verify clean:
ps aux | grep -iE "gz sim|parameter_bridge|move_group|robot_state_publisher" | grep -v grep
```

### Terminal 1: Launch Gazebo + MoveIt + Controllers
```bash
cd ~/swarm/swarm_project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mycobot_description mycobot_280pi_camera_moveit2
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 gazebo.launch.py
```

Wait ~15 seconds for controllers to spawn, then verify:
```bash
ros2 control list_controllers
```

### Terminal 2: Run annulus_show.py (Visualization)
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/annulus_show.py
```

### Terminal 3: Run annulus_test.py (Execution)
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/annulus_test.py --execute
```

### Terminal 4: Camera Processing
```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/camera_view.py
```

### Terminal 5: Spawn World Objects
(One-shot: AprilTag marker + ball into Gazebo)

```bash
source ~/swarm/swarm_project/install/setup.bash
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/spawn_world.py
```

---

## Project Structure

```
~/swarm/swarm_project/
├── src/
│   ├── swarm_pkg/               # Main package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   └── src/
│   │       └── scripts/         # All Python scripts
│   │           ├── pick_place.py
│   │           ├── annulus_test.py
│   │           ├── annulus_show.py
│   │           ├── ik_probe.py
│   │           ├── reach_probe.py
│   │           ├── check_state_validity.py
│   │           ├── gen_disable_collisions.py
│   │           ├── camera_view.py
│   │           ├── camera_test.py
│   │           ├── gripper_test.py
│   │           ├── reset_arm.py
│   │           ├── collision_contacts.py
│   │           ├── gripper_offset_probe.py
│   │           └── spawn_world.py
│   │
│   ├── mycobot_description/     # Robot meshes & URDFs
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── urdf/
│   │       ├── adaptive_gripper/    (7 .dae mesh files)
│   │       └── mycobot_280_pi/      (11 .dae mesh files)
│   │
│   └── mycobot_280pi_camera_moveit2/  # MoveIt config
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── config/
│       │   ├── firefighter.urdf.xacro
│       │   ├── firefighter.srdf
│       │   ├── firefighter.ros2_control.xacro
│       │   ├── joint_limits.yaml
│       │   ├── kinematics.yaml
│       │   ├── initial_positions.yaml
│       │   ├── moveit_controllers.yaml
│       │   ├── ros2_controllers.yaml
│       │   ├── pilz_cartesian_limits.yaml
│       │   └── moveit.rviz
│       ├── launch/
│       └── worlds/
│
├── legacy/                      # Old code (keep for reference)
├── build/                       # Build artifacts (auto-generated)
├── install/                     # Installed packages (source this)
├── log/                         # Build logs (auto-generated)
├── WORKFLOW.md                  # This file
└── .git/
```

---

## Quick Commands

```bash
# Build everything
cd ~/swarm/swarm_project && colcon build

# Build only robot packages
colcon build --packages-select mycobot_description mycobot_280pi_camera_moveit2

# Build only swarm_pkg
colcon build --packages-select swarm_pkg

# Source setup
source ~/swarm/swarm_project/install/setup.bash

# List all packages
ros2 pkg list | grep -E "swarm|mycobot"

# Find a package
ros2 pkg prefix mycobot_280pi_camera_moveit2

# Run a script
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/pick_place.py
```

---

## Troubleshooting

### Packages not found
```bash
# Verify setup.bash was sourced
echo $ROS_PACKAGE_PATH

# Re-source if needed
source ~/swarm/swarm_project/install/setup.bash
```

### Script fails to find MoveIt config
```bash
# Make sure ROS_PACKAGE_PATH includes install/
ros2 pkg list | grep mycobot_280pi_camera_moveit2
```

### Gazebo clock issues
- Kill old processes: `pkill -9 -f "gz sim"`
- Clean shared memory: `rm -rf /dev/shm/fastrtps_* /dev/shm/ros_*`
- Launch fresh: `ros2 launch mycobot_280pi_camera_moveit2 gazebo.launch.py`

