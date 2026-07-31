# Swarm Project Workflow

## Setup (Do Once Per Terminal Session)

### Build and source (on mars)
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

### DDS Unicast Discovery (campus network setup)
The campus network blocks UDP multicast, breaking ROS2's default discovery.
Configure Cyclone DDS with static unicast peers instead.

**On both mars and the robot (one-time setup):**
```bash
# Install Cyclone DDS RMW plugin
# On mars (Jazzy):
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# On the robot (Galactic):
sudo apt install ros-galactic-rmw-cyclonedds-cpp
```

**Add to your terminal session, each time you open a new terminal. The config
FILENAME differs per machine -- Cyclone DDS behaves differently enough between
the robot's Galactic build and mars's Jazzy build that the settings are split
into two files (see `cyclone_dds_integration_log.md`). Use `cyclonedds.xml`
with no suffix and it will not exist as valid config for either machine.**

On the robot (Galactic):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml
```

On mars (Jazzy):
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jazzy.xml
```

Or add the matching block to your shell's `.bashrc` / `.zshrc` (per machine) to
persist across sessions -- e.g. on the robot:
```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml' >> ~/.bashrc
```

Then reload: `source ~/.bashrc`

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

### Terminal 1: Setup DDS, build and launch (on the robot, Galactic)
```bash
cd ~/swarm_project
source /opt/ros/galactic/setup.bash

# Set DDS environment (must be done before ros2_control starts)
# NOTE: cyclonedds_galactic.xml, not cyclonedds.xml -- the config was split
# per-distro (see cyclone_dds_integration_log.md); cyclonedds.xml is a stale
# pre-split file that stays orphaned in install/ once colcon has ever built it,
# because colcon does not clean install/ artifacts whose source was deleted.
# It is also not valid XML (a "--" inside an XML comment body, illegal), so
# pointing at it makes ros2_control_node and robot_state_publisher crash on
# startup with "can't open configuration file" -- rmw_create_node then fails
# and every controller spawner retries against a domain that never formed.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml

colcon build --packages-select mycobot_description mycobot_280pi_camera_moveit2 mycobot_hardware
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot.launch.py
```

**Note:** If you added the DDS exports to your shell's `.bashrc` / `.zshrc`, you
don't need to run those `export` commands again -- just `source ~/.bashrc`
at the start of your terminal session.

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

## Split-Compute Real Hardware Workflow (planning on mars, execution on the robot)

Same physical setup as the single-machine Real Hardware Workflow above, but
`move_group` (IK, OMPL planning) and `rviz2` run on mars instead of the Pi --
the Pi is CPU-constrained for planning, especially on failed/infeasible
queries that burn the full planning timeout. Trajectory *execution* still
happens entirely on the Pi via `ros2_control`: mars sends one complete
planned trajectory per motion over a `FollowJointTrajectory` action goal,
and the Pi's local controller interpolates and executes it in real time
without needing the network mid-motion. `/joint_states` and action feedback
stream back from the Pi to mars at ~50-100Hz for visualization/monitoring.

This requires DDS unicast discovery working between mars and the robot --
see "DDS Unicast Discovery" under Setup above and the `swarm_network`
package. Both machines must have the DDS environment variables set
(`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID`, `CYCLONEDDS_URI`) before launching
either half below -- if they're in `.bashrc` already, a fresh terminal on
each machine is enough.

Two new launch files replace the monolithic `real_robot.launch.py` for this
mode: `real_robot_hardware.launch.py` (robot side: bridge,
`robot_state_publisher`, `ros2_control_node`, controller spawners -- no
`move_group`, no `rviz`) and `real_robot_planning.launch.py` (workstation
side: `move_group` and `rviz` only, built with `hardware_mode=real` so
`robot_description` matches the robot even though mars can't build the
Galactic-only `mycobot_hardware` plugin itself -- `move_group` never loads
that plugin, only `ros2_control_node` does, and that stays on the robot).

### Terminal 1: Hardware (on the robot, Galactic)
```bash
cd ~/swarm_project
source /opt/ros/galactic/setup.bash
# DDS env already in .bashrc -- skip these exports if so. If .bashrc still
# says cyclonedds.xml (no _galactic suffix), fix it there too -- see the note
# in Terminal 1 of the split-terminal walkthrough above.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml

# swarm_network MUST be in this list: CYCLONEDDS_URI points at the install
# tree, so a `git pull` that changes a peer IP has no effect until it is
# rebuilt -- the robot keeps announcing to mars's old address and mars sees
# zero publishers while everything looks healthy locally.
colcon build --packages-select swarm_network mycobot_description mycobot_280pi_camera_moveit2 mycobot_hardware
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot_hardware.launch.py
```

### Terminal 2: Planning + RViz (on mars, Jazzy)
```bash
cd ~/swarm/swarm_project
source /opt/ros/jazzy/setup.bash
# DDS env already in .bashrc -- skip these exports if so:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jazzy.xml

colcon build --packages-skip mycobot_hardware
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot_planning.launch.py
```

### Terminal 3: Verify, then run pick_place.py / annulus_test.py (on mars)
```bash
source ~/swarm/swarm_project/install/setup.bash
ros2 node list                    # /move_group must be present, plus the robot's nodes
ros2 control list_controllers     # all three should show "active" (queried over DDS from the robot)
ros2 topic hz /joint_states       # should show ~50-100Hz streaming from the Pi
python3 ~/swarm/swarm_project/src/swarm_pkg/src/scripts/pick_place.py
```

If `/move_group` doesn't see the robot's controllers, or `ros2 node list`
on mars is missing the robot's nodes (`/controller_manager`,
`/robot_state_publisher`, etc.), re-run the DDS verification test under
Troubleshooting before debugging further -- this almost always means
discovery isn't working, not a MoveIt/controller problem.

---

## AprilTag Workflow (vision-guided pick and place)

Full design, staging and measured numbers: **`APRIL_TAGS.md`**.

Same split-compute layout as above, plus two things on the robot. All vision
runs on the Pi -- no image ever crosses the DDS link, which silently drops
anything over ~1400 bytes.

### Terminal 4: Detector (on the robot, Galactic)
```bash
source ~/swarm_project/install/setup.bash
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_detector_node.py
```

**Do NOT also launch `camera.launch.py`.** Changed 2026-07-31:
`block_detector_node.py` now reads `/dev/video0` directly (`cv2.VideoCapture`)
instead of subscribing to a topic published by `camera.launch.py`'s
`v4l2_camera_node`. That used to route every frame through Cyclone DDS even
though both processes were on the same Pi -- a 640x480 frame is 921,600 bytes
against this link's deliberately small `MaxMessageSize=1400B` (tuned for the
mars<->robot Wi-Fi hop, irrelevant to a purely local topic), so it fragmented
into ~700 RTPS pieces per frame and occasionally stalled for the better part of
a minute. V4L2 only allows one reader; running `camera.launch.py` alongside this
node now means one of them fails to open the device, not that they cooperate.
If you need the raw topic for something else (RViz, `live_tag_view.py`), stop
this node first.

### Terminal 5: Detection and picking (on mars, Jazzy)
```bash
source ~/swarm/swarm_project/install/setup.bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts

# 1. does the service answer at all? (arm parked at a hover, zone in view)
#    NOTE: `ros2 node list` does NOT show the robot's nodes from mars even when
#    they are up -- use the service list, not the node list.
ros2 service list | grep detect_block
ros2 service call /detect_block swarm_interfaces/srv/DetectBlock \
  "{zone: 'pickup', zone_x: 0.0, zone_y: 0.25, zone_z: 0.0, zone_yaw: 0.0}"

# 2. detect and print the grasp pose, execute no descent
python3 tag_pick_place.py --zone-origin 0.0 0.25 0.0 --dry-run \
    --debug-image /tmp/zone.png --log /tmp/corrections.csv

# 3. the real thing
python3 tag_pick_place.py --zone-origin 0.0 0.25 0.0 --log /tmp/corrections.csv
```

`--zone-origin` is the **surveyed** world pose of the zone centre. Nothing
measures it, and every world coordinate reported is only as good as that
number -- the vision measures the block RELATIVE to the zone.

### No robot needed
```bash
# geometry regression test: catches corner-order, homography and
# classification bugs against synthetic ground truth, in about a second
python3 zone_vision_selftest.py

# look at what the detector sees in saved stills, and tune thresholds
python3 zone_view.py frames/*.png --show
python3 zone_view.py /tmp/zone.png --method otsu --write /tmp/annotated.png
```

### Gotchas
- `swarm_interfaces` must be built on **both** machines from identical `.srv`
  source, or the service type will not match across the link.
- Call `/detect_block` only while the arm is **stationary**. The serial link is
  half-duplex, and the Pi is also running the 100 Hz control loop.
- A block can only sit within ~±23 mm of the zone centre before it starts
  covering a tag -- see "Usable area" in `APRIL_TAGS.md`.

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
│   │           ├── tool_frame_check.py
│   │           ├── spawn_world.py
│   │           │
│   │           │   # AprilTag feature -- see APRIL_TAGS.md
│   │           ├── zone_vision.py           # pure OpenCV, no ROS: tags -> block pose
│   │           ├── zone_vision_selftest.py  # synthetic geometry test, no hardware
│   │           ├── zone_view.py             # overlay viewer / threshold tuning
│   │           ├── block_detector_node.py   # ON THE PI: /detect_block service
│   │           └── tag_pick_place.py        # ON MARS: Stage 1 orchestrator
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
│   ├── mycobot_hardware/        # ros2_control plugin for REAL hardware
│   │   ├── package.xml          # Galactic-only -- see Setup note above
│   │   ├── CMakeLists.txt
│   │   ├── mycobot_hardware.xml # pluginlib description
│   │   ├── include/mycobot_hardware/mycobot_system.hpp
│   │   ├── src/mycobot_system.cpp
│   │   └── scripts/mycobot_bridge.py  # pymycobot bridge daemon
│   │
│   ├── swarm_network/           # DDS unicast discovery config
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── config/
│   │       ├── cyclonedds_galactic.xml   # DDS config for the robot (Galactic)
│   │       └── cyclonedds_jazzy.xml      # DDS config for mars (Jazzy) -- the
│   │                                     #   two differ; see cyclone_dds_
│   │                                     #   integration_log.md for why
│   │
│   └── swarm_interfaces/        # Service defs shared Pi <-> mars
│       ├── package.xml          # MUST be built on BOTH machines
│       ├── CMakeLists.txt
│       ├── msg/BlockDetection.msg
│       └── srv/DetectBlock.srv
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
├── PROJECT_CONTEXT.md           # What the system is, and why
├── TESTS.md                     # Hardware characterization
├── APRIL_TAGS.md                # Vision-guided pick and place
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

### Real hardware: "Goal reached, success!" in the logs but the arm never moved
- Fixed (Fix 7) -- `real_robot.launch.py` starts `mycobot_bridge.py` and
  `ros2_control_node` at the same time. The bridge needs real wall-clock
  time to import `pymycobot` and open the serial connection before its
  socket exists; `MyCobotSystem::on_activate()` was trying to connect
  immediately and only once, reliably losing that race. `connect_bridge()`
  now retries for up to ~10s. **Important, independent of this fix:**
  Galactic's `controller_manager` does not appear to block controller
  activation even when a hardware component's `on_activate()` returns an
  error -- `arm_group_controller` spawned and reported "Goal reached,
  success!" even while `mycobot_hardware` was logging "Could not connect to
  mycobot_bridge.py" every single run. **The ROS logs alone are not
  sufficient proof of real motion on this setup -- always visually confirm
  the arm actually moved.**

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

### DDS discovery verification (campus network)
**Test that the workstation and robot can see each other over DDS:**

On mars (workstation), in one terminal:
```bash
# Source setup and DDS environment
source ~/swarm/swarm_project/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jazzy.xml

# Run a simple talker
ros2 run demo_nodes_cpp talker
```

On the robot, in another terminal:
```bash
# Source setup and DDS environment (if not already in .bashrc)
source ~/swarm_project/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml

# Echo the topic
ros2 topic echo /chatter
```

**Expected:** The robot's terminal will show messages from the workstation's
talker, like:
```
data: 'Hello World: 1'
---
data: 'Hello World: 2'
---
```

If it doesn't work:
- Verify both machines have sourced the DDS environment variables
- Check that both machines can ping each other (not multicast, regular ICMP)
- Confirm `ros-jazzy-rmw-cyclonedds-cpp` is installed on mars
- Confirm `ros-galactic-rmw-cyclonedds-cpp` is installed on the robot
- Check you are pointing at the right FILE for this machine:
  `cyclonedds_galactic.xml` on the robot, `cyclonedds_jazzy.xml` on mars.
  Plain `cyclonedds.xml` is a stale pre-split artifact -- if `ros2 pkg prefix
  swarm_network`'s install dir still has one, it is orphaned build output, not
  live config, and it is not even valid XML. Safe to `rm` it.
- Check the IPs in that file are correct. BOTH addresses change when the campus
  DHCP lease renews -- mars has moved (172.27.89.157 -> 172.27.80.139) and so
  has the robot (172.30.6.165 -> 172.30.11.51, 2026-07-31). Run `hostname -I`
  on each machine and compare against the `<Peer>` entries. Symptom of a stale
  entry: mars's `ros2 node list` shows only its own nodes and
  `ros2 topic info /joint_states` reports 0 publishers, while the robot side
  looks perfectly healthy locally. Confirm with a plain `ping` between the two
  before touching anything in ROS.
- Check the file you edited is the one Cyclone actually loads. `CYCLONEDDS_URI`
  points into the INSTALL tree, so a `git pull` alone does not take effect --
  `colcon build --packages-select swarm_network` and relaunch. Cyclone reads
  the XML once at process start, so a running launch keeps the old peers.

