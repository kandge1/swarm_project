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

# On a robot you have not built on before, confirm the prerequisites first --
# this is the difference between one actionable message and the CMake stack
# traces in issue #22. Read-only; installs and builds nothing.
./pi_setup/preflight_check.sh

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

# ... or, for the AprilTag-free colour path (new 2026-08-12):
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_detector_node.py \
    --ros-args -p method:=colour
```

`method` is `canny` (default) | `otsu` | `colour`. **A typo is refused at
startup**, not ignored: `zone_vision`'s dispatch falls through to canny, so
`method:=color` would run a whole session reporting no colours with nothing
saying why.

`method:=colour` segments by **saturation** — the mat is white paper, the blocks
are painted — which yields whole regions instead of a Canny outline, so it has
none of the dilate-driven size oversize. Colour travels on the
`block_detections` topic (`detection_wire.py` **schema 3**), *not* in the service
response, so **copy `detection_wire.py` and `zone_vision.py` to the Pi and
restart the node — there is no interface rebuild.** Both machines must be on
schema 3 or `decode()` refuses outright rather than misreading.

**A white block on a white mat is not found by this method** and is reported as an
absence. There is no saturation step to threshold. Natural wood *is* found, by
value.

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
  "{zone: 'pickup', zone_x: 0.0, zone_y: 0.2286, zone_z: 0.0, zone_yaw: 0.0}"

# 2. detect and print the grasp pose, execute no descent
python3 tag_pick_place.py --zone-origin 0.0 0.2286 0.050 --dry-run \
    --debug-image /tmp/zone.png --log /tmp/corrections.csv

# 3. the real thing
python3 tag_pick_place.py --zone-origin 0.0 0.2286 0.050 --log /tmp/corrections.csv
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
│   ├── install_pi_galactic.sh   # One-shot installer for a fresh arm
│   ├── preflight_check.sh       # Run before colcon build; explains failures
│   └── requirements.txt
│
├── workstation_setup/           # Workstation install (Ubuntu 24.04/Jazzy)
│   └── install_workstation_jazzy.sh
│
├── legacy/                      # Old code (keep for reference)
│   └── COLCON_IGNORE            # Keeps the dead 'control' pkg out of builds
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

### Fresh arm: `colcon build` fails (GitHub issue #22)

Run this first on any new robot. It checks every prerequisite in one pass and
prints the exact fix for each, instead of letting CMake report the same
problems as stack traces:

```bash
cd ~/swarm_project
./pi_setup/preflight_check.sh
```

Issue #22 was three separate problems stacked on top of each other, all of
which this script now catches up front:

**1. `Could not find ... "ament_cmake"` / `ros2: command not found`**

ROS was never sourced in that shell. Sourcing is per-terminal and does not
persist across new terminals or reboots:

```bash
source /opt/ros/galactic/setup.bash   # on the robot
source /opt/ros/jazzy/setup.bash      # on mars
```

Never source `/opt/ros/noetic` (the vendor image's ROS1) in the same shell.

**2. `Could not find ... "hardware_interface"`, `Package 'controller_manager' not found`**

ros2_control is not part of the stock Elephant Robotics image, and
`mycobot_hardware` cannot build without it. The fresh arm had never had the
installer run on it:

```bash
./pi_setup/install_pi_galactic.sh
```

That installs MoveIt2, ros2_control, the camera stack, and pymycobot. Expect
it to take a while on a Pi 4.

**3. `Starting >>> control` for a package that isn't in `src/`**

`legacy/ws/src/control/` is a dead AGV package kept for reference. colcon used
to discover it as a workspace package, so every plain `colcon build` tried to
build it and its failures were interleaved with real ones. `legacy/COLCON_IGNORE`
now stops that. If an older checkout already built it, clear the leftovers
(they are not rebuilt, but stay visible to `ros2 pkg list`):

```bash
rm -rf build/control install/control
```

Note that `colcon build` aborts *all* in-flight packages when any one of them
fails, so a single missing dependency reads like the whole workspace is broken.
`3 packages aborted` means "did not finish", not "also failed".

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


---

## Calibration workflow (as of 2026-08-11)

Full record and the reasoning behind every constant:
**`CALIBRATION_2026-08-11.md`**. Current open-loop grasp error is 0.58 mm RMS.

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
```

### Picking a real block — no calibration flags

```bash
python3 explore_pick_place.py --any-block --note real_pick
```

**Never pass `--force-grasp-yaw` here.** It pins the wrist and the arm will
hover dead over the block without orienting to it. That is the flag working, not
a bug — it exists so a caliper reading has a known axis.

### Measuring the open-loop error at one position

```bash
python3 explore_pick_place.py --any-block --skip-pick --position N \
    --force-grasp-yaw 0 --note my_note
```

At the confirm prompt the arm is parked 8 mm above the block's top face:

| type | does |
|---|---|
| `m 0 4` | **records a caliper reading, MOVES NOTHING.** Do this first. |
| `0 4` | nudges +4 mm in world Y and re-parks |
| `0 4 90` | nudge plus 90° of wrist |
| ENTER | descend and grasp |
| `q` | abort |

- `m` is the **measurement**; the nudge is a **control action**. They are not the
  same number — the first nudge at any pose loses one J1 dead band (2.6 mm at
  r=126, 4.7 mm at r=229). Give `m` the same sign you would type as a nudge.
- **Nudge in ONE step**, never several small ones: each reversal donates up to a
  full backlash (1.88°).
- `--skip-pick` returns before any descent, so the block never moves and repeats
  are free.
- Reach must be **121–222 mm (4.8–8.7 in)**. Outside that a `[tool]` warning
  fires and the tangential term is extrapolated.

### Positions (`--position`, offsets in inches)

```
A (-3,-7)   B ( 0,-7)   C (+3,-7)   D (+5,-5)   E (+6,-3)
F (+7,-2)   G (+8,-1)   H (+9, 0)   I (+8,+1)   J (+7,+2)
K (+6,+2)   L ( 0,+7)   M (+3,+7)   N (+5, 0)   O (+7, 0)
```

`--position` sets the truth column from the **nominal** inch grid, i.e. where the
mat was *meant* to go. It is not a measurement — do not fit against
`truth_world` unless the mat was independently measured.

### Survey only, gripper never leaves home

```bash
python3 explore_pick_place.py --survey-only --position G --note my_note
```

### In-zone (block off-centre) against a fixed surveyed origin

Take the origin and yaw from the survey above, then substitute **real numbers**:

```bash
python3 tag_pick_place.py --zone-origin 0.2059 -0.0315 0.050 --zone-yaw 88.8 \
    --skip-pick --truth-block-zone 0 -20 --note q2
```

- `--truth-block-zone` is **MILLIMETRES**, `--truth-block-world` is **METRES**.
- `--no-truth-block-on-centre` belongs to **`explore_pick_place.py`** and will
  be rejected by `tag_pick_place.py`.
- Usable range is 23.1 mm, checked on `max(|zx|,|zy|)`, so ±20 mm corners are
  legal.

### Reading the results

```bash
python3 calibration.py --report
python3 calibration.py --fit --channel survey --zone pickup
python3 calibration.py --fit --channel nudge --max-clearance-mm 10
```

`--zone pickup` is the default and should stay that way: place rows are rank
deficient alone (the place zone never moves) and pooling them corrupts the fit.

### Offline, no robot — all must print `0 failure(s)`

```bash
python3 -m py_compile tag_pick_place.py pick_place.py zone_vision.py \
    zone_calibrate.py explore.py explore_pick_place.py calibration.py \
    stack_blocks.py
python3 zone_vision_selftest.py
python3 explore.py --selftest
python3 calibration.py --selftest
python3 stack_blocks.py --selftest
python3 block_tags_selftest.py
```

---

## Stacking workflow (`stack_blocks.py`, new 2026-08-12)

Surveys both zones, picks two blocks **by name**, and stacks them at the place
zone centre with the near face square to the robot.

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts

# check the whole plan without touching a block
python3 stack_blocks.py --survey-only

# every pose parked at, nothing grasped or released
python3 stack_blocks.py --dry-run

# the real thing, with the operator checkpoints ON
python3 stack_blocks.py --stack "orange block" "the green one"
```

Names are plain English: `orange`, `orange top`, `the orange block`,
`the block named green` all resolve. Anything ambiguous or unknown is
**refused**, not guessed. `python3 stack_blocks.py --selftest` lists the
phrasings that are covered.

### The two parks, and why the first runs must keep them

Same protocol as the pick side — `m dx dy` records and moves nothing, `dx dy`
nudges and re-parks, `dx dy dyaw` also turns the wrist, ENTER goes. **The place
park is where the one measurement this project has never taken gets taken.** The
place side has never been calibrated (stage 0 declared ±25 mm acceptable), so type
`m` at the place park on every early run. It writes a row with `kind: "place"` and
`place_open_loop_offset`.

**The two after-the-fact yes/no questions are gone** (removed on request,
2026-08-13): *"did the jaws actually close on the X block?"* and *"is the block
sitting squarely on level N?"*. The operator is watching the arm and will stop it,
so the prompts only added a keystroke between them and the Ctrl-C.

What that gave up, so it is not a surprise later:

- **Nothing else knows whether a block is in the jaws.** The gripper's own
  `CONTACT` detection is the closest thing — it stops the close when the jaw
  trails its command by ≥ 0.06 rad — and that fires on the fingertips touching
  *anything*, including each other on a missed block.
- **Nothing measures whether a level went down square.** The camera never looks at
  the stack, only at the pickup zone. A crooked level 0 is the one failure that
  makes level 1 land on a slope, and the run will not notice.

Both fields are now recorded as `null` — *nobody looked* — rather than `false`,
which would have claimed the operator saw a failure. **Watch the place, and stop
the run yourself if a level goes down crooked.**

### Repeatability -- the four things that limit it, in order (2026-08-13)

Measured off the first autonomous colour stack. **Nothing here needs new hardware
and only the last needs new code.**

1. **The place survey moves up to 8 mm between runs.** Decomposed: pickup zone
   radial spread **0.17 mm** / tangential 1.5 mm; place zone radial 1.8 mm /
   tangential **8.3 mm** = 2.0 deg of J1. The error is almost purely *tangential*
   at both zones, and tangential is J1. Within a run it does **not** tip the stack
   -- both levels get the same surveyed XY, so it is common-mode -- but between
   runs it moves where the stack lands. **Not yet attributable**: run
   `--survey-only` twice touching nothing. Agree to ~1 mm and it is the bench;
   disagree by 8 mm and it is `j1_unidirectional_approach`.
2. **Half the survey stills never reach the vote** -- 10 of 20 across these logs.
   One causal chain: the lens re-centring pushes the framing flange 13-15 mm
   further out, four of five stills then lose `DETECT_HOVER_Z`, drop to z 0.240,
   soften (tag modules ~39 px instead of ~57), and fail `MULTIVIEW_MIN_TAGS`.
   Test with **`--no-lens-recentre`** and compare `[multiview] N usable view(s)`.
3. **The jaw aperture and finger dimensions are still guesses.** The clearance gate
   passed at +8.5 mm on this run and refused at −0.2 mm on the one before, both
   computed from unmeasured numbers. Three caliper readings.
4. **The z landing error, ±5 mm** -- now *measured* rather than assumed, see below.
   Ten runs of the new fields and it becomes a correction.

### The release height is checked before the jaws open

The arm does not land where it is sent in z, and the error changes sign with the
level: level 0 finished **+1.6 mm high** and level 1 **−3.9 mm low** on 2026-08-13,
so the blue was pressed **0.9 mm into the red** despite the 3 mm `PLACE_DROP_M`.
The run before was the same shape (+2.8 then −1.4).

So `release_z_gap()` reads the flange FK the instant the descent finishes, works
out where the held block's base actually is against the surface, and **lifts by the
shortfall before releasing** if it is negative:

```
[stack] descent landed at flange 0.1700, asked for 0.1739 (-3.9 mm). The block's
        base is 0.0205 against a surface of 0.0214: -0.9 mm.
[stack] NEGATIVE -- the block is 0.9 mm INTO level 0. That is what tips a stack.
[stack] lifting 0.9 mm before releasing ...
```

`place_release_z_achieved`, `place_release_z_error_mm` and `place_release_gap_mm`
now go into `calibration_history.jsonl`. **A bigger drop is not the fix** -- it
trades digging in for a harder landing, and `PLACE_DROP_M` was carrying the whole
±5 mm alone.

### What limits a stack

| | |
|---|---|
| Absolute position of the stack | the place **survey**, a few mm — needs ±25 mm, so fine |
| Straightness of the stack | **not** the survey. A systematic place error is common to both blocks and displaces the whole stack instead of tipping it |
| Real floor | the per-block grasp residual (~1 mm) and the **unmeasured** level-0-vs-level-1 droop difference |

`DESCENT_BIAS_Z` and the far-corner compliance term were both measured at
level 0 and neither has been checked 30 mm higher.

**Confirmed on hardware 2026-08-12**: both blocks placed, level 0 dead centre
and level 1 square on top, **with zero nudges** — rows 217/218 of
`calibration_history.jsonl`. Fully open loop.

### Transit height — the one thing that went wrong, and it is fixed

Carrying block 1 across, the **carried block struck block 2 and moved it**. The
retreat after a grasp went to `grasp_z + APPROACH_HEIGHT = 0.1855`, which leaves
the carried block's bottom face 10 mm above a block resting on the mat, and the
94° sweep to the place zone passed directly over it. The sweep is an
unconstrained OMPL plan, so nothing holds z between the endpoints.

`APPROACH_HEIGHT` is sized for the **descent**, not for flying a payload over
another block. Every cross-zone move now lifts straight up first
(`traverse()` → `transit_flange_z()`), default **25 mm** under the load:

```bash
python3 stack_blocks.py --transit-clearance-mm 29   # the most MAX_HOVER_Z allows
```

The ceiling is tight and it is the same one that blocks level 2: **29.5 mm over a
one-block pile, nothing at all over a two-block one**. The whole vertical budget,
with `g` = grasp height above the block's own base and `c` = transit clearance:

```
g + c <= 44.5 mm          (0.205 - GRASP_OFFSET_Z - one block on the mat)
```

Today `g = 15`, `c = 25`. **Grip low** — every millimetre of grasp height is a
millimetre of clearance given up. See `APRIL_TAGS_DEV.md`, "STEP 2 DESIGN".

### Picking by COLOUR instead of by AprilTag (`--by-colour`, new 2026-08-12)

**The ZONE tags are still required.** Colour replaces the per-*block* top tags,
not the four tags on each mat — the whole geometry chain (homography, zone frame,
parallax, the surveyed origin) still comes from those, and nothing about it
changes.

Two things have to line up:

1. **On the Pi**: copy `zone_vision.py`, `detection_wire.py` and
   `block_detector_node.py` across, then launch with `method:=colour`
   (see Terminal 4 above). **No interface rebuild.** Both machines must be on
   `detection_wire` schema 3.
2. **On mars**: pass `--by-colour`, and give `--stack` colour names.

```bash
python3 stack_blocks.py --by-colour --survey-only     # what does it see and name?
python3 stack_blocks.py --by-colour --dry-run --stack red
python3 stack_blocks.py --by-colour --stack red                 # pick + place
python3 stack_blocks.py --by-colour --stack red "the green one" # ... and stack
```

A colour names a *set*, not one block, so `--stack red red` is legal — the two
red blocks are two different contours, and the second pick sees the first one
gone. The tag path keeps its "same block twice" refusal, where the name really
does mean one physical block.

**Three reasons a contour is left unidentified**, each printed, because each wants
a different fix: `unknown` (no hue prototype within range, or an achromatic blob),
**low score** (a hue between two prototypes — usually a blob that is part mat or
part shaded side wall), **low agreement** (the views disagreed — lighting, or one
contour spanning two differently-coloured blocks, which must not be grasped).

#### If the pickup survey rejects every sighting

```
[survey] pickup J1 -12.5 REJECTED: image centre is 93 mm from the zone centre;
         3 tags are trusted only to 72 mm out
[stack] no pickup zone, so there is nothing to pick.
```

**That is framing plus a missing zone tag, not a colour problem, and the two
stack.** One undetected tag drops the trust radius from 144 mm to 72 mm
(`MAX_CENTRE_OFFSET_HALF_DIAGONALS`); if the fine arc also fails to bring the
camera inside 72 mm of the zone centre, every sighting goes. On 2026-08-12 the
closest approach was **74 mm** — it missed by 2 mm, ten times over. `refine_pitch`
now medians the coarse radius over every sighting instead of trusting the anchor's,
which is what aimed it 36 mm inside the mat.

**Two upstream causes of this were fixed on 2026-08-13; if you are reading an
older log, that is why it failed.**

1. **The coarse origins were built with an assumed zone yaw of 0°** (`zone_yaw_for(...)
   or 0.0`), while the pickup mat sits at −91.4°. On run 10's six coarse views that
   turned radii of 0.2445–0.2497 (MAD 1.6 mm) into 0.0365–0.2837 (MAD 84 mm), so
   `refine_pitch` refused to refine, the arc ran 51 mm short of the mat, and the
   camera never came inside the trust radius. `reseat_coarse_origins` now solves the
   yaw from the coarse sweep — `camera_zx/zy` and `joints` are both yaw-free, so
   `fit_zone` can do it with no new measurement — and rewrites the origins with it.
   A given `--pickup-yaw` still wins.
2. **An isolated rejected sighting split the arc.** `split_runs` grouped the
   *survivors*, so a gated view left a hole indistinguishable from the tags going
   out of sight. Run 10's six good views became fragments of 1, 2 and 3 with 0, 7
   and 14 mm of baseline, all refused for needing 25 mm — pooled they span 52 mm and
   fit to 5.3 mm. Grouping now runs over **all** the sightings; a real 90° hole
   still splits.

If a survey still rejects everything, the messages to read first are
`coarse yaw solved at ... deg` (is it near ±90 for these mats?) and whether
`fine arc` says *refine* or *KEEPING pitch*. A `KEEPING pitch` means the arc is
aimed at a known-wrong radius and the rest of the run is downstream of that.

**`--zone-yaw` APPLIES TO BOTH ZONES AND THE TWO MATS ARE ~180 DEG APART** —
pickup surveys near −91, place near +88.6. A yaw a half turn out displaces every
origin by *twice* the camera offset, which is 49.5 mm of scatter and an outright
rejection. Use the per-zone flags:

```bash
python3 stack_blocks.py --by-colour --pickup-yaw -93     # place still solved
python3 stack_blocks.py --by-colour --place-yaw 88.6     # or the other way
```

`fit_zone` now cross-checks any fixed yaw against the one the views imply and says
so when they differ by more than 10°, naming a half turn when it sees one.

**Try a fixed PICKUP yaw first.** `fit_zone` skips the yaw baseline check when the yaw is
given, so a single four-tag sighting is enough to hand off a *measured* origin:

```bash
python3 stack_blocks.py --by-colour --pickup-yaw -93
```

Only if that still finds nothing, override the origin too — that one is a number
you typed, feeding a grasp:

```bash
python3 stack_blocks.py --by-colour --pickup-at 0.209 0.003 --zone-yaw -93
```

Either way: **a one-view fit reports `residual 0.0 mm` because there is nothing to
compare it with, not because it is exact.** The survey warns about it; believe the
warning.

`--zone-yaw` is **required** with `--pickup-at` and it refuses without it: the zone
frame has an origin *and* a rotation, and guessing the rotation swings every block
position about your origin. Unlike `--place-at` this feeds a **grasp**, so tape the
number, do not estimate it, and keep `--confirm` on.

#### `--confirm` is the default

It is accepted as a flag (so the spelling works) but changes nothing — the park
check is on unless you pass **`--yes`**, which turns it off and gives up the only
physical confirmation that a block is in the jaws.

#### Reading the colour log

`block_detector_node.py` logs the median H/S/V of each contour's own pixels, on
the Pi:

```
[0] zone (+20.1, -22.4) mm ... colour red (0.86) HSV(165, 220, 200)
```

That is the line to look at when a colour comes out wrong — every threshold in
`COLOUR_HUES` / `COLOUR_PINK_MAX_SAT` / `COLOUR_SAT_MIN` is reasoned rather than
measured, and this turns tuning them into one reading instead of a sequence of
guesses. `score` is a hue *distance* turned into a confidence, not a pixel
fraction: 1.0 at the prototype, 0 at `COLOUR_MAX_HUE_DIST`, and
`COLOUR_MIN_SCORE = 0.45` is the identity gate.

**Red is a hue BAND (168 → 6 through the wrap), and pink is red below saturation
140.** Pink is not a hue prototype: it is physically a tint, and having it as a
prototype meant it captured every hue from 160 to 174 and named a red prism "pink"
on the first hardware run.

#### Read this before running a non-cube block

- **A non-cube colour needs a row in `tag_pick_place.COLOUR_FOOTPRINT_M`, or it
  will be refused as two blocks touching.** Three separate checks ask "is this
  footprint plausible for one block", and until 2026-08-13 all three compared
  against `BLOCK_NOMINAL_M = 0.030` — right for every tagged block on this bench,
  wrong for a set whose whole point is different shapes. The green brick reads
  60.3 mm because it *is* 61.0 mm, and it was refused on two full hardware runs
  for being the size it is. The table is `(short side, long side)` in metres, in
  the block's one assumed rest pose:

  | colour | block | footprint | height |
  |---|---|---|---|
  | `red` | 1 in trapezoid, sitting | 30.5 × 35.6 mm | **25.4 mm** |
  | `blue` | 1.2 in frustum, standing | 30.5 × 35.6 mm | 30.5 mm |
  | `green` | 1.2 × 1.2 × 2.4 in brick, lying | 30.5 × 61.0 mm | 30.5 mm |
  | *unlisted* | falls back to the 30 mm cube | 30.0 × 30.0 mm | 30.0 mm |

  **These tables are BENCH STATE, not a block library.** They are keyed by colour
  and the set has two reds, two blues and two greens — a row is only correct while
  that block is the one on the mat. Update the rows when you change the blocks. A
  *stale* row is worse than a missing one: a missing one falls back to the cube and
  says so out loud.

  **Heights are per level**, and they differ: the 25.4 mm red under the 30.5 mm
  blue is the current default pair. `--block-thickness` overrides with one number
  for every level.

  A missing row fails **safe** — it refuses to grasp rather than grasping wrong —
  but it fails *late*, after the whole sweep and survey. Add the row before the
  run, not after. `stack_blocks.py --selftest` asserts every colour in the default
  `--stack` list has one and that its short side clears the jaw aperture.
- **The 4-fold symmetry gate is relaxed to a warning** in colour mode. This set is
  mostly 2-fold and there are no side tags yet, so "near side faces the robot" is
  not well defined — the block goes down on whichever of its two face pairs the
  yaw fold lands on. That was the agreed trade for the demo.
- **The wrist-yaw convention is UNVERIFIED for elongated blocks.**
  `GRASP_YAW_FROM_MAJOR_DEG = 0.0` reproduces today's behaviour exactly, but
  whether `block_yaw_deg` names the closing axis or the block's long axis has
  never been distinguishable — a cube folds the 90° difference away. `[grip]` prints
  the block's long-axis world angle against the commanded wrist yaw and says when
  the two are distinguishable. **Watch a `--dry-run --confirm` park with an
  elongated block and look at the fingers.** If they line up on the long side, set
  that constant to 90. Do not run an elongated block unattended first.
- **The jaws refuse anything whose short side is over `JAW_APERTURE_OPEN_M`
  (40 mm, an ESTIMATE).** On this set that means every grasp is across a 1.4 in
  (35.6 mm) face or smaller — 1.6 in is 40.6 mm and already over, and the pink
  disc is ungraspable lying flat (55.9 mm every way through its centre).
  `--ignore-grip-span` exists for the case where the *aperture figure* is what is
  wrong, not the block. Caliper the open jaws and set
  `JAW_GEOMETRY_MEASURED = True`.

### Level 2 is refused, and not for the reason `APRIL_TAGS_DEV.md` gives

A level-2 release wants flange z **0.2055** against `MAX_HOVER_Z` **0.205**, so
`hover_z_for` clamps the pre-place hover *below* the release point and the
descent inverts. The flange can physically reach 0.2055 at the zone radius —
this is a hover-ceiling limit, not a workspace one, and it bites **before**
the reach margin that document tabulates (which is about *picking* from
level 2). `--max-level 2` does not rescue it; the hover check catches it too.

### Blocks next to each other

Two checks now run before any descent, in `stack_blocks.py` and
`tag_pick_place.py` alike:

- **Merged contour.** Two touching 30 mm blocks read as one 30 × 60 mm blob whose
  centroid is in the seam — and `MAX_BLOCK_LENGTH_M` is exactly 60, so it used to
  be *accepted*. Refused now, definitively when two block classes' TOP tags claim
  one contour, otherwise when **either** footprint side is `MERGED_MARGIN_M`
  (20 mm) past that block's own nominal — 50 mm for a cube, 81/50.5 mm for the
  green brick. Both sides are tested because two bricks touching along their long
  sides read 61 × 61, which no long-side test can see. `--ignore-merged` overrides.

  Note what this check cannot do: **two touching 30.5 mm cubes and one
  61 × 30.5 mm brick are the same rectangle.** It has to be told which block it is
  looking at, which is why `COLOUR_FOOTPRINT_M` exists.
- **Jaw clearance.** The fingers need room along the **closing** axis and almost
  none across it. On a square face, base and base+90 are different axes and both
  are valid grasps, so a blocked axis is retried 90° round. Refused if neither has
  room. `--ignore-clearance` overrides.

For 30 mm blocks: **51.3 mm** of centre separation is needed *along* the closing
axis and **20.8 mm** *across* it, against a usable box only **46.2 mm** across.
So the 90° rotation is mandatory, not optional — the jaw axis must end up
perpendicular to the line joining the two blocks. A 2-fold block (most of
`block_database/`) has no second axis and no escape.

#### Where to put the second block — direction, not distance

**Put it off the END of the target, in line with the target's long axis. Not
beside it.** The fingers are 8 mm thick along the closing axis and 18 mm wide
across it, so the two directions cost nothing like the same. With the green brick
as the target:

| blue's position | distance | margin |
|---|---|---|
| 40 mm **along** the closing axis (beside the brick) | 40 mm | **−11.3 mm** blocked |
| 34 mm **across** it (off the brick's end) | 34 mm | **+8.7 mm** clear |

A neighbour 6 mm *nearer* is 20 mm *better*, because it is in the other direction.
That is why the 2026-08-13 run refused at 48.5 mm of separation while an earlier one
passed: distance is the wrong variable. The refusal now decomposes the blocker into
the jaw frame and names the direction to move it.

And the box is genuinely tight: a 4 in mat with 1 in corner tags leaves **46.6 mm**
for a block centre, while the green brick is **61 mm long**. On the diagonal there
is 66 mm, which is the only reason two of these fit at all. **This pair is at the
geometric limit of a 4 in mat.**

**The jaw aperture and finger dimensions are UNMEASURED** — every run prints so.
Three caliper readings at `GRIPPER_OPEN` turn the rule from conservative into
exact; see `JAW_GEOMETRY_MEASURED` in `tag_pick_place.py`. Full reasoning in
`APRIL_TAGS_DEV.md`, "NEIGHBOURING BLOCKS".

### The pose memory

```bash
python3 stack_blocks.py --memory ~/stack_memory.json     # persist between runs
python3 stack_blocks.py --no-memory                      # measure what it buys
```

Caches the **commanded** joint target of each move (`pick_place.LAST_ARM_GOAL`),
keyed on the world pose asked for, and replays it as a joint goal instead of
re-solving IK.

- **Commanded, never achieved.** Storing achieved angles would bake in J1's
  arrival backlash and re-command it, compounding the error
  `j1_unidirectional_approach` exists to cancel — and it would look perfectly
  repeatable while doing it.
- **Keyed on the target, not on a label**, so a mat that has moved simply
  *misses* the cache. That is what makes `--memory` safe across sessions.
- It saves IK and planning latency, **not arm motion**. The real speedup in this
  script is that one pickup survey serves both blocks.

### One survey, two picks

The default surveys the pickup zone **once**: `identify_blocks` returns a class
per contour, and lifting one block does not move another. Use `--resurvey` when
blocks start out touching, which is the case where the cached position of the
second block can go stale.

### Angled / off-axis zones — fixed 2026-08-12

Two independent bugs made a pickup mat away from the +Y axis fail. Both are
fixed; this is what to know when reading older logs.

**1. The fine arc inherited the coarse pitch.** `EXPLORE_PITCH_DEG = -71` aims
the optical axis at **7.72 in**, not the 9 in its old comment claimed. That is a
fine *coarse* compromise for a 5–10 in bench, and it was fatal for the fine pass:
the axis landed 33–41 mm short of the mat centre at every J1, the image centre
sat 80–89 mm off, the 3-tag trust radius is 72 mm, so **every 3-tag sighting was
rejected** → one surviving still → cannot solve zone yaw → `no pickup zone`, arm
never moves. The fine arc now gets its own pitch from the coarse pass's measured
radius (`explore.refine_pitch`), which brings framing error under 0.2 mm at any
bench radius.

**2. The 5-still survey's wrist yaws were absolute world angles.**
`survey_flange_for_yaw` places the flange at `zone centre − lens offset`, and the
offset direction comes from that absolute angle — so whether a still pulls the
flange *in* or pushes it *out past the mat* depended on the mat's bearing. The
yaws are now relative to the mat's bearing. Worst-of-5 flange radius:

| mat bearing | before | after |
|---|---|---|
| 0° (N, O, H) | — | **bit-identical** |
| +90° (standard pickup) | 0.2640 | 0.2115 (−52 mm) |
| −41° | 0.2478 | 0.2200 (−28 mm) |
| −135° | 0.2711 | 0.2152 (−56 mm) |

Framing is now bearing-invariant: any bearing frames as well as bearing 0, which
is the case with the track record.

**Also:** `[ik] All seeds exhausted … falling back to constraint sampling` was a
lie whenever the caller passed `allow_constraint_sampling=False` (which
`detect_multiview` always does). Skipped survey stills logged a fallback that
never happened. The message no longer claims what the caller will do.

### `--dump-sightings` — for when a survey finds nothing

```bash
python3 stack_blocks.py --survey-only --dump-sightings /tmp/sightings.json
```

Written **before** the gate runs, so a survey that rejects everything still
leaves its evidence. Re-fit it offline with `explore.load_sightings(path)`, which
rebuilds real `Sighting` objects so `choose()` / `fit_zone()` run on them
unchanged. Added because on 2026-08-12 the question "is there a good solution in
this data that the gate threw away?" was unanswerable — the only record was
printed text, and rebuilding sightings by parsing it recovered none of a
known-good run's.

### If a run says `no pickup zone, so there is nothing to pick`

Check the tag count per sighting. Trust radius is tag-count dependent —
`{4 tags: 144 mm, 3 tags: 72 mm}` from the zone centre. A mat with one
undetected tag drops to 72 mm, and if it sits adjacent to the other zone the
fine pass centres on the *other* mat and every sighting is rejected. Clean or
reprint the missing tag.
