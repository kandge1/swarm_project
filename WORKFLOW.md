# Swarm Project Workflow

## Setup (Do Once Per Terminal Session)

```bash
cd ~/swarm/swarm_project
source /opt/ros/jazzy/setup.bash
colcon build --packages-skip mycobot_hardware  # Only if code changed
source install/setup.bash
```

**On mars (Jazzy), always skip `mycobot_hardware`.** It's a `ros2_control`
plugin for the physical arm, written against ROS2 Galactic's
`hardware_interface` API (which the actual robot runs) -- Galactic and
Jazzy disagree on the `read()`/`write()` method signature, so this package
can only build on Galactic. Plain `colcon build` with no flags will now
fail on this one package; that's expected on mars, not a regression. On the
robot itself (Galactic), build it normally -- see "Real Hardware Workflow"
below.

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
colcon build
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

## Real Hardware Workflow

Runs against the physical mycobot 280 Pi instead of Gazebo or RViz-mock, via
the `mycobot_hardware/MyCobotSystem` `ros2_control` plugin
(`src/mycobot_hardware/`), which bridges to `mycobot_bridge.py` (a
persistent Python process using `pymycobot`, elephantrobotics' vendor
driver) over a Unix domain socket. **This only builds/runs on the robot
itself (ROS2 Galactic)** -- see the note under Setup above.

**Path note:** the robot's clone of this repo lives at `~/swarm_project`
(no `swarm/` parent directory) -- different from mars, where it's
`~/swarm/swarm_project`. The commands below use the robot's actual path;
adjust if yours differs.

### Known gaps -- read before running against real hardware

- **Gripper contact detection will not work as-is.** `pick_place.py`'s
  `gripper_close_until_contact()` relies on reading `gripper_controller`'s
  effort off `/joint_states`, but pymycobot's gripper API
  (`set_gripper_value`/`get_gripper_value`, a 0-100 scale) exposes no
  force/effort reading at all. `mycobot_bridge.py` reports a constant `0.0`
  placeholder for gripper effort, so the contact-detection loop will always
  behave as if nothing is ever touched. A real fix needs a different signal
  (e.g. `is_gripper_moving()` going to 0 mid-close as a stall/contact proxy)
  and hasn't been built yet.
- **Joint velocity is always reported as `0.0`** -- pymycobot exposes no
  velocity reading. Controllers here only rely on position tracking, so
  this is a placeholder, not a bug, but worth knowing.

Confirmed working on the real robot: `mycobot_hardware` builds clean on
Galactic, `mycobot_bridge.py` connects to the arm at `/dev/ttyAMA0 @
1000000` baud, and `move_group`/RViz/controllers all come up (see
Troubleshooting below for the `libbackward.so` fix that was needed first).

### Terminal 1: Build and launch (on the robot, Galactic)
```bash
cd ~/swarm_project
source /opt/ros/galactic/setup.bash
colcon build --packages-select mycobot_description mycobot_280pi_camera_moveit2 mycobot_hardware
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot.launch.py
```

This starts, in order: `mycobot_bridge.py` (opens the serial connection),
`robot_state_publisher`, `ros2_control_node` (loads `MyCobotSystem`, which
connects to the bridge's socket), `move_group`, `rviz2`, then -- after a
3s delay for the above to come up -- the controller spawners.

### Terminal 2: Verify, then run pick_place.py / annulus_test.py
```bash
source ~/swarm_project/install/setup.bash
ros2 node list                    # /move_group must be present
ros2 control list_controllers     # all three should show "active"
python3 ~/swarm_project/src/swarm_pkg/src/scripts/pick_place.py
```

Same scripts as the RViz/Gazebo workflows -- they talk to `move_group` over
the same services/actions regardless of which `hardware_mode` is actually
moving the arm underneath.

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
│   ├── mycobot_280pi_camera_moveit2/  # MoveIt config
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── config/
│   │   │   ├── firefighter.urdf.xacro       # hardware_mode: mock|gazebo|real
│   │   │   ├── firefighter.srdf
│   │   │   ├── firefighter.ros2_control.xacro
│   │   │   ├── joint_limits.yaml
│   │   │   ├── kinematics.yaml
│   │   │   ├── initial_positions.yaml
│   │   │   ├── moveit_controllers.yaml
│   │   │   ├── ros2_controllers.yaml
│   │   │   ├── pilz_cartesian_limits.yaml
│   │   │   └── moveit.rviz
│   │   ├── launch/
│   │   │   ├── demo.launch.py         # hardware_mode=mock (default)
│   │   │   ├── gazebo.launch.py       # hardware_mode=gazebo
│   │   │   └── real_robot.launch.py   # hardware_mode=real
│   │   └── worlds/
│   │
│   └── mycobot_hardware/        # ros2_control plugin for REAL hardware
│       ├── package.xml          # Galactic-only -- see Setup note above
│       ├── CMakeLists.txt
│       ├── mycobot_hardware.xml # pluginlib description
│       ├── include/mycobot_hardware/mycobot_system.hpp
│       ├── src/mycobot_system.cpp
│       └── scripts/mycobot_bridge.py  # pymycobot bridge daemon
│
├── pi_setup/                    # Robot-side install (Ubuntu 20.04/Galactic)
│   ├── install_pi_galactic.sh
│   └── requirements.txt
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
# Build everything (on mars/Jazzy: mycobot_hardware will NOT build -- see below)
cd ~/swarm/swarm_project && colcon build --packages-skip mycobot_hardware

# On the robot (Galactic), mycobot_hardware builds normally -- include it:
colcon build

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

### `mycobot_hardware` fails to build
- **On mars:** expected. It targets Galactic's `hardware_interface` API,
  which differs from Jazzy's `read()`/`write()` signature. Always build
  with `--packages-skip mycobot_hardware` on mars.
- **On the robot (Galactic):** confirmed working -- builds cleanly.

### `move_group` dies instantly: `libbackward.so: cannot open shared object file`
- Known Galactic packaging gap: `ros-galactic-moveit-ros-move-group`
  should pull in `ros-galactic-backward-ros` but doesn't always.
  `pi_setup/install_pi_galactic.sh` now installs it explicitly; if you
  set up the robot before this was added:
  ```bash
  sudo apt install ros-galactic-backward-ros
  ```
- **Symptom to watch for:** RViz's Motion Planning panel can still load
  and show the robot model even with `move_group` dead -- that's RViz's
  own internal preview, not proof move_group is running. The real
  tell is `Failed to call service get_planning_scene, have you launched
  move_group...?` in the RViz log, or `ros2 node list` not showing
  `/move_group`.

### Real hardware: `mycobot_bridge.py` can't open the serial port
- Confirm `DEFAULT_SERIAL_PORT`/`DEFAULT_BAUD_RATE` in
  `src/mycobot_hardware/scripts/mycobot_bridge.py` actually match this
  Pi's onboard UART -- both are unverified placeholders.
- Check permissions on the serial device (may need the user in the
  `dialout` group, or `sudo chmod`).

### Real hardware: gripper never reports contact
- Expected for now -- pymycobot's gripper API has no effort/force
  reading, so `gripper_close_until_contact()`'s contact detection can't
  work as written against real hardware. See "Known gaps" under Real
  Hardware Workflow above.

### `arm_group_controller`: "Time between points 0 and 1 is not strictly increasing"
- Fixed (Fix 6) -- was caused by dropping `ompl_planning.yaml`'s
  `response_adapters` (Fix 5, for the Galactic/Jazzy type conflict), which
  also dropped time parameterization on Galactic. `pick_place.py`'s
  `plan_motion()` now has a `_ensure_monotonic_timing()` safety net that
  recomputes valid waypoint timing when the planner returns none -- a
  no-op on Jazzy/Gazebo, where trajectories already come back properly
  timed. If this resurfaces, check whether `/plan_kinematic_path`'s
  response has all-zero `time_from_start` again.

