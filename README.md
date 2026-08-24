# Swarm Project — myCobot 280 Pi pick, place and stack

Vision-guided pick-and-place on an Elephant Robotics myCobot 280 Pi. The arm
surveys two paper mats with a wrist camera, identifies blocks by **AprilTag** or
by **colour**, picks two of them, and stacks one on the other — fully open loop,
no human in the geometry.

This README takes you from **a fresh arm and a fresh computer** to a working
colour stack. Everything here has been run on real hardware; where something is
unverified or known-fragile, it says so.

> **Read this first if you are new:** the arm moves through poses computed from
> printed paper. A mis-scaled printout or a wrong IP fails *silently* — the run
> looks healthy and the arm reaches for the wrong place. The checks in this
> document exist because each one has already cost someone a day.

---

## 1. Get the code (git and GitHub)

New to git? Read **[git_user_guide.md](git_user_guide.md)** first — it covers
everything below plus the branch naming this project uses. This section is just
enough to get the code onto both machines.

You need a GitHub account, and **push access to this repo**: ask the repo owner
to add you as a collaborator. Do this before your first lab session; nothing
below works without it.

### 1a. Tell git who you are (once per machine)

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

### 1b. Set up authentication (once per machine)

GitHub does not accept your account password from the command line. Use an SSH
key — it works the same on your laptop and on the robot, and you never type it
again.

```bash
ssh-keygen -t ed25519 -C "you@example.com"     # press ENTER at every prompt
cat ~/.ssh/id_ed25519.pub                       # copy the whole line
```

Paste that into GitHub → **Settings → SSH and GPG keys → New SSH key**. Then
check it worked:

```bash
ssh -T git@github.com
# "Hi <username>! You've successfully authenticated" — the "does not provide
# shell access" part that follows is normal, not an error.
```

**Do this on both machines.** Each one needs its own key; a key is tied to the
computer it was made on, so do not copy the private key around.

### 1c. Clone, on each machine

**The two machines use different paths.** Every command in this project's docs
assumes these exact locations:

```bash
# On the ROBOT
git clone git@github.com:kandge1/swarm_project.git ~/swarm_project
cd ~/swarm_project

# On the WORKSTATION
mkdir -p ~/swarm
git clone git@github.com:kandge1/swarm_project.git ~/swarm/swarm_project
cd ~/swarm/swarm_project
```

### 1d. Check out the right branch — do not skip this

A fresh clone puts you on `main`, which is **a long way behind** the arm's real
work — dozens of commits, and the gap grows every time `mycobot_main` moves.
Nothing in this README will match what you see there. Switch immediately:

```bash
git checkout mycobot_main
git log --oneline -3        # newest commit should mention april_tags / stacking

# how far behind is main today? (0 on the left, a big number on the right)
git rev-list --left-right --count origin/main...origin/mycobot_main
```

| Branch | What it is |
|---|---|
| `mycobot_main` | **The arm. Start here.** |
| `myagv_main` | The AGV line — a different robot, not this guide |
| `main` | A stale snapshot. Do not build from it or open PRs against it |

Then make your own branch before changing anything — see
[git_user_guide.md](git_user_guide.md):

```bash
git checkout -b feature/your_thing
```

---

## 2. What you need

### Hardware
| Item | Notes |
|---|---|
| myCobot 280 Pi | Raspberry Pi 4 inside, vendor Ubuntu 20.04 image |
| Wrist camera + camera flange | Mounted on the flange; lens sits ~40 mm off the flange axis |
| Adaptive gripper | |
| Linux workstation | Ubuntu 24.04 for ROS2 Jazzy. Does the planning; the Pi is too slow for it |
| Wi-Fi both machines can reach | They must be able to ping each other |
| Printer + white paper | For the zone mats |
| 2+ painted blocks, 30 mm cube | Default demo is **red** then **blue** on top |

### Software layout — two machines, two ROS distros

| | Robot (the arm) | Workstation |
|---|---|---|
| Called | "the robot" | "mars" in older docs — feel free to use your own name |
| OS / ROS | Ubuntu 20.04 / **Galactic** | Ubuntu 24.04 / **Jazzy** |
| Runs | `ros2_control`, the serial bridge, the camera + detector | `move_group` (IK + planning), RViz, the scripts you type |

Planning runs on the workstation because the Pi burns its CPU on failed planning
queries. Execution stays on the Pi: one complete trajectory crosses the network
per motion, then the Pi's controller runs it in real time.

---

## 3. What is NOT on the stock Elephant Robotics image

The vendor image ships ROS2 Galactic and ROS1 Noetic and almost nothing this
project needs. Installing these is most of the bring-up:

| Missing | Why we need it |
|---|---|
| `ros2_control`, `ros2_controllers`, `controller_manager` | `mycobot_hardware` will not compile without it — this is the `Findhardware_interface.cmake` error |
| MoveIt2 (`moveit`, `moveit_kinematics`, `moveit_configs_utils`, …) | All planning and IK |
| `rmw_cyclonedds_cpp` | Default DDS cannot do unicast discovery; campus Wi-Fi blocks multicast |
| `python3-opencv` (with `aruco`) | AprilTag decode and colour segmentation |
| `v4l2-camera`, `cv_bridge` | Wrist camera |
| `python3-colcon-common-extensions` | To build at all |
| `pymycobot >= 3.6.1` (pip) | The serial driver. pip only — not an apt package |
| `pillow` (pip) | Only for generating printable tag sheets |

`pi_setup/install_pi_galactic.sh` installs all of the robot-side ones in one
pass. It does **not** touch the ROS1 Noetic install.

---

## 4. Install — the robot

```bash
git clone <this-repo> ~/swarm_project
cd ~/swarm_project

./pi_setup/install_pi_galactic.sh        # 20-40 min on a Pi 4

source /opt/ros/galactic/setup.bash
./pi_setup/preflight_check.sh            # read-only; must pass before building
colcon build
source install/setup.bash
```

`preflight_check.sh` is the difference between one actionable line and a CMake
stack trace. Run it on any machine you have not built on before. It catches the
three things that broke the last fresh arm (GitHub issue #22):

1. **ROS not sourced.** `source /opt/ros/galactic/setup.bash` is **per terminal**
   and does not persist. Unsourced, CMake reports `Findament_cmake.cmake` missing,
   which names neither the cause nor the fix.
2. **The installer never run.** Then `hardware_interface` is missing and
   `mycobot_hardware` cannot build.
3. Never source `/opt/ros/noetic` in the same shell — ROS1 and ROS2 conflict.

If `colcon build` prints `3 packages aborted`, that means "did not finish", not
"also failed". One missing dependency reads like a broken workspace.

### Serial port
The bridge uses **`/dev/ttyAMA0` @ 1000000 baud**, not `/dev/serial0`. On a Pi 4
`serial0` symlinks to the mini UART unless Bluetooth is disabled, and opening the
wrong port *succeeds* — writes go nowhere and reads return stale angles with no
error. If the arm reports the same joint angles no matter what you command, check
this first.

---

## 5. Install — the workstation

Start from a fresh **Ubuntu 24.04**. The script installs ROS2 Jazzy itself, so
you do not need ROS beforehand.

```bash
git clone <this-repo> ~/swarm/swarm_project
cd ~/swarm/swarm_project

./workstation_setup/install_workstation_jazzy.sh

# then, in a NEW terminal so ROS2 is on your PATH:
cd ~/swarm/swarm_project
colcon build --packages-skip mycobot_hardware
source install/setup.bash
```

It installs ROS2 Jazzy, MoveIt2, ros2_control, RViz2, Cyclone DDS, OpenCV and
the colcon/rosdep tooling, adds `source /opt/ros/jazzy/setup.bash` to your
`.bashrc`, and finishes by running `preflight_check.sh` against the result. It
is safe to re-run.

Deliberately **not** installed: Gazebo, Isaac Sim, Docker and the NVIDIA stack
(none are needed to fly the real arm), `moveit_py` (nothing imports it), and
`pymycobot` (drives the arm's serial port, which only exists on the robot).

**Always `--packages-skip mycobot_hardware` here.** It is a Galactic-only
ros2_control plugin; Galactic and Jazzy disagree on the `read()`/`write()`
signature. It failing on the workstation is expected, not a regression.

## 6. Network — the step that breaks silently

Campus Wi-Fi blocks UDP multicast, so DDS discovery uses a **hardcoded list of
peer IPs**. On new hardware those IPs are wrong, and nothing reports it: each
machine looks perfectly healthy alone and simply never sees the other.

### 6a. Put your two IPs in the config

```bash
hostname -I     # run on BOTH machines, note both addresses
```

Edit **both** files — the two ends must agree:

- [src/swarm_network/config/cyclonedds_galactic.xml](src/swarm_network/config/cyclonedds_galactic.xml)
- [src/swarm_network/config/cyclonedds_jazzy.xml](src/swarm_network/config/cyclonedds_jazzy.xml)

```xml
<Peers>
  <Peer address="192.168.1.50"/>   <!-- your robot -->
  <Peer address="192.168.1.60"/>   <!-- your workstation -->
</Peers>
```

Then **rebuild `swarm_network` on both machines.** `CYCLONEDDS_URI` points into
`install/`, so editing the source alone changes nothing:

```bash
colcon build --packages-select swarm_network && source install/setup.bash
```

### 6b. Environment, every terminal

```bash
source ~/swarm_project/swarm_env.sh          # on the robot
source ~/swarm/swarm_project/swarm_env.sh    # on the workstation
```

One command, every terminal, both machines. It sources ROS2, then the
workspace, then sets the DDS variables — **that order matters**, because
`CYCLONEDDS_URI` is built from `ros2 pkg prefix swarm_network` and resolves to
nothing until the workspace is sourced. It prints the peer list and this
machine's IP, so a wrong address shows up straight away.

**Every terminal needs it, including ones that run a plain `python3 script.py`.**
A ROS2 node started without it joins the default domain, works perfectly, logs
nothing wrong, and is simply invisible to the other machine.

<details>
<summary>What it does by hand, if you would rather set it yourself</summary>


The filename differs per machine — Galactic and Jazzy need different Cyclone
settings. A bare `cyclonedds.xml` is a stale pre-split file and is not valid
config for either.

```bash
# On the robot
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_galactic.xml

# On the workstation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jazzy.xml
```

Both machines need the same `ROS_DOMAIN_ID`.

</details>

> **Nothing over ~1400 bytes crosses this link.** `MaxMessageSize` is deliberately
> small to survive the Wi-Fi hop. That is why no camera image ever crosses the
> network — all vision runs on the Pi and only coordinates come back.

---

## 7. Print and place the mats

Each zone is a **101.6 mm** square of white paper with four **25.4 mm** AprilTags
(36h11) at its corners — ids 0-3 for pickup, 4-7 for place. Pre-generated sheets
are in [print_sheets/](print_sheets/), or regenerate:

```bash
cd src/swarm_pkg/src/scripts
python3 print_zone_tags.py            # both zones, 300 dpi
```

**Verify the printed scale with the ruler on the sheet.** Detection decodes a tag
at any size, but the homography converts pixels to millimetres using those two
dimensions as ground truth. A printer's "fit to page" rescales everything, and
every position the detector reports is then wrong by that factor — silently.

Lay one mat in the pickup area and one in the place area, both flat and
unwrinkled. **The zone tags are required even in colour mode** — colour replaces
the per-*block* tags, never the mats.

A block can only sit within about ±23 mm of a zone centre before it starts
covering a corner tag.

---

## 8. Run the colour stack

Four terminals. **Every one starts with `swarm_env.sh`** — including Terminal 3,
which runs a plain `python3` script. Skip it there and the detector runs fine on
the wrong DDS domain, invisible to the workstation.

**Terminal 1 — robot: hardware**
```bash
cd ~/swarm_project && source swarm_env.sh
ros2 launch mycobot_280pi_camera_moveit2 real_robot_hardware.launch.py
```
Wait till you see three yellow lines with the names of the controllers being configured and initalized like the following

[spawner_joint_state_broadcaster]: Configured and started joint_state_broadcaster
[spawner_arm_group_controller]: Configured and started arm_group_controller
[spawner_gripper_group_controller]: Configured and started gripper_group_controller
[INFO] [bash-6]: process has finished cleanly [pid 17329]

Note that It takes usually two or three tries to get each controller to configure and the script automatically tries each controller 5 times before giving up. Why this does not work on the first try is only known to god. 

DO NOT start Terminal 2 on a workstation before Terminal 1 states that all controllers are configured and ready and the last "process has finished cleanly" is published in terminal 1. Doing so forces method calls from controllers that aren't configured and induces import/construction/initialization failures into the controllers and thus not letting them initialize properly. 

**Terminal 2 — workstation: planning + RViz**
```bash
cd ~/swarm/swarm_project && source swarm_env.sh
ros2 launch mycobot_280pi_camera_moveit2 real_robot_planning.launch.py
```

**Terminal 3 — robot: the colour detector**
```bash
source ~/swarm_project/swarm_env.sh
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_detector_node.py \
    --ros-args -p method:=colour
```
`method` is `canny` | `otsu` | `colour`. A typo is refused at startup rather than
silently falling back. **Do not also launch `camera.launch.py`** — this node opens
`/dev/video0` directly and V4L2 allows only one reader.

**Terminal 4 — workstation: verify, then stack**
```bash
source ~/swarm/swarm_project/swarm_env.sh
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts

# Discovery actually working? Do this before blaming MoveIt.
ros2 control list_controllers    # all three "active", queried from the robot
ros2 topic hz /joint_states      # ~50-100 Hz streaming from the Pi
ros2 service list | grep detect_block

# 1. What does it see and what does it name? Nothing moves toward a block.
python3 stack_blocks.py --by-colour --survey-only

# 2. Every pose parked at, nothing grasped or released.
python3 stack_blocks.py --by-colour --dry-run --confirm

# 3. The real thing — red, then blue on top.
python3 stack_blocks.py --by-colour --confirm
```

`--confirm` keeps the operator checkpoints on. **Keep them for every early run.**
Nothing in the system knows whether a block is really in the jaws, and the camera
never looks at the stack — a crooked level 0 will not be noticed by the software.
Watch the place, and stop the run yourself if a level goes down crooked.

Other useful flags:

```bash
python3 stack_blocks.py --by-colour --stack red "the green one"   # name them
python3 stack_blocks.py --by-colour --resurvey                    # blocks that touch
python3 stack_blocks.py --by-colour --pickup-yaw -93              # see below
python3 stack_blocks.py --selftest                                # no robot needed
```

`--stack red red` is legal in colour mode: a colour names a *set*, and the second
pick sees the first block gone.

---

## 9. When it fails

| Symptom | Cause |
|---|---|
| `Findament_cmake.cmake` missing / `ros2: command not found` | ROS not sourced in this terminal |
| `Findhardware_interface.cmake` missing | `install_pi_galactic.sh` never run |
| `Starting >>> control` for a package not in `src/` | Stale checkout — `legacy/COLCON_IGNORE` fixes it |
| Workstation sees no robot nodes, everything looks fine locally | Peer IPs wrong, or `swarm_network` not rebuilt after editing them (§6) |
| `can't open configuration file file:///share/...`, every node dies | `CYCLONEDDS_URI` exported before `source install/setup.bash`; build and source first |
| RViz plans fine but Execute is rejected, `Time between points ... not strictly increasing` | Expected on Galactic. Drive the arm with the Python scripts, not RViz's button |
| Arm reports identical angles regardless of command | Wrong serial port — `/dev/ttyAMA0`, not `/dev/serial0` |
| `send_angles() got an unexpected keyword argument '_async'`, arm never moves | pymycobot 3.7.0 dropped `_async`. Fixed 2026-08-19 — `git pull` and relaunch |
| `Goal reached, success!` but the arm never moved | Multi-waypoint trajectory issue — see WORKFLOW.md |
| Survey rejects every sighting | Framing plus a missing zone tag. Try `--pickup-yaw -93`. One undetected tag halves the trust radius |
| Every position off by a constant factor | The mats were printed at the wrong scale |
| `/detect_block never appeared in 15s`, but `/joint_states` works | Detector started without the DDS env (it's a bare `python3`, easy to miss), or stale `swarm_interfaces` on the robot |
| A white block is never found | Colour mode segments by saturation; white on white has none. Use tags for it |

**`--zone-yaw` applies to both zones, and the two mats sit ~180° apart** (pickup
near −91°, place near +88.6°). Use `--pickup-yaw` / `--place-yaw` separately.

Full troubleshooting, including the DDS verification procedure, is in
[WORKFLOW.md](WORKFLOW.md).

---

## 10. Known gaps and future work

Read these before promising anyone a demo:

- **Peer IPs are hardcoded and must be edited by hand** on every new pair of
  machines. This is the most common bring-up failure.
- **Gripper contact detection does not work.** pymycobot exposes no force or
  effort reading, so the contact loop behaves as if nothing is ever touched.
- **Joint velocity is always reported `0.0`** — pymycobot has no velocity read.
  Controllers here use position only, so this is a placeholder, not a bug.
- **The place zone has never been calibrated.** ±25 mm was declared acceptable.
- **Stack repeatability is limited by the survey**, which moves up to 8 mm between
  runs — almost entirely tangential, i.e. J1. Within one run it is common-mode and
  does not tip the stack.
- **Jaw aperture and finger dimensions are unmeasured guesses**, and a clearance
  gate depends on them. Three caliper readings would close this.

### Future work

- **Two or more arms at once is NOT supported.** Everything here assumes exactly
  one robot on the DDS domain. Bringing up a second arm on the same
  `ROS_DOMAIN_ID` gives you two `/controller_manager`s, two `/joint_states`
  publishers and two `arm_group_controller`s under identical names — the
  scripts would be commanding an ambiguous pair of robots, and which one
  answers is a race. **Do not run two arms on one domain.**

  The stopgap today is to keep them apart: `swarm_env.sh` honours
  `SWARM_DOMAIN_ID`, so a second arm and the terminals that talk to it can run
  isolated:

  ```bash
  SWARM_DOMAIN_ID=43 source swarm_env.sh
  ```

  That isolates them; it does not let them cooperate. Real multi-arm work needs
  per-robot namespaces (`/arm1/joint_states`, `/arm1/controller_manager`, …)
  pushed through the launch files, the MoveIt config and every script that
  hardcodes a topic or action name, plus a story for which arm a given script
  is addressing. None of that exists. This is the single biggest piece of
  future work for a project named "swarm".

- **Peer IPs are hand-edited and DHCP moves them.** The address has already
  changed three times (`172.30.6.165` → `172.30.11.51` → `172.30.11.42`). A
  script that writes both configs from `hostname -I` would remove the most
  common bring-up failure.

- **RViz's Execute button cannot drive the real arm on Galactic** — planning
  works, execution is rejected for zero time parameterization. Restoring it
  means giving Galactic a working time-parameterization response adapter. The
  Python scripts work around it; RViz cannot.

- **Only red, green and blue have measured geometry.** Orange, yellow, purple
  and cyan are detected but fall back to a generic 30 mm cube for height and
  footprint, which feeds the grasp height and the clearance gate. Three caliper
  readings per colour closes this.

Confirmed working on hardware: a full autonomous colour stack, both blocks placed,
level 0 centred and level 1 square on top, zero nudges (2026-08-12).

---

## 11. Where to read next

| Document | What's in it |
|---|---|
| [swarm_env.sh](swarm_env.sh) | `source` in every terminal: ROS + workspace + DDS, in the right order |
| [git_user_guide.md](git_user_guide.md) | Git and GitHub from scratch, and our branch naming |
| [WORKFLOW.md](WORKFLOW.md) | Every workflow in detail, all troubleshooting, project structure |
| [dev_logs/APRIL_TAGS.md](dev_logs/APRIL_TAGS.md) | Vision design, measured numbers, usable area |
| [dev_logs/PROJECT_CONTEXT.md](dev_logs/PROJECT_CONTEXT.md) | What the system is, and why it is built this way |
| [dev_logs/TESTS.md](dev_logs/TESTS.md) | Hardware characterization |
| [dev_logs/STACKED_BLOCKS_GUIDE.md](dev_logs/STACKED_BLOCKS_GUIDE.md) | Stacking specifics |
| [dev_logs/cyclone_dds_integration_log.md](dev_logs/cyclone_dds_integration_log.md) | Why the DDS config is split per distro |

### Scripts worth knowing

| Script | Purpose |
|---|---|
| `stack_blocks.py` | Survey, pick two, stack. The demo |
| `tag_pick_place.py` | Single vision-guided pick and place |
| `pick_place.py` | The motion primitives everything else calls |
| `reset_arm.py` | Return to a known pose |
| `zone_view.py` | Inspect what the detector sees in a saved still |
| `block_detector_node.py` | Robot-side vision service |
| `print_zone_tags.py` | Generate printable mats |

### No robot needed
```bash
python3 zone_vision_selftest.py     # geometry regression, ~1 second
python3 stack_blocks.py --selftest  # name-resolution coverage
```
