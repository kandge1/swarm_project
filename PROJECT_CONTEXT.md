# Project Context — Read This Before Doing Anything Else

This file exists so a future session (e.g. working on a new branch cut from
`fix/project_structure`) doesn't have to rediscover everything below from
scratch. It's a snapshot of hard-won context from one long session, not
general documentation — see `WORKFLOW.md` for day-to-day operational
commands.

---

## The two-machine, two-distro constraint (read this first)

This project runs across **two machines that cannot be made to match**:

| | mars (dev machine) | the robot (mycobot 280 Pi) |
|---|---|---|
| OS | Ubuntu 24.04 | Ubuntu 20.04 (Focal) |
| ROS2 distro | Jazzy | Galactic |
| Repo path | `~/swarm/swarm_project` | `~/swarm_project` (no `swarm/` parent!) |
| Why fixed | User's personal machine, won't reinstall | Vendor (elephantrobotics) OS image, must not be touched -- has other useful vendor tooling on it. Also: this Pi was tried on Ubuntu 24.04 once and was too slow to even run `apt update` on an SD card (root cause: SD card I/O, not a hard CPU ceiling, but reflashing wasn't pursued) |

**Neither machine's OS/distro is going to change.** Every piece of code that
runs on both must be written to tolerate both distros' APIs, or must be
built distro-specifically (see `mycobot_hardware` below). Don't propose
"just upgrade Galactic" or "just use moveit_py" as a fix -- both were
explicitly ruled out by the user for the reasons above.

**Path gotcha:** commands for the robot use `~/swarm_project`; commands for
mars use `~/swarm/swarm_project`. Mixing these up produces `No such file or
directory` and has happened repeatedly in this session. Double check which
machine a command is for before giving path advice.

---

## Why moveit_py was removed entirely

`pick_place.py` and friends originally used `moveit_py` (MoveItPy,
MoveItConfigsBuilder, moveit.core.robot_state) for in-process planning.
`moveit_py` bindings are not reliably available on ROS2 Galactic (the
robot's distro) -- there was no guarantee `ros-galactic-moveit-py` would
even be installable, and there was no willingness to reflash the robot to a
newer distro to get it (see constraint above).

**Fix:** rewrote `pick_place.py`'s `RobotIOClient` to talk to an
externally-launched `move_group` purely over plain ROS2 services/actions:
- `/plan_kinematic_path` (`GetMotionPlan`) -- replaces
  `PlanningComponent.plan()`
- `/check_state_validity` (`GetStateValidity`) -- replaces
  `planning_scene_monitor`-based collision checks
- `/compute_ik` (`GetPositionIK`) -- already existed, extended with an
  "exact pose, no tolerance" variant (`compute_ik_exact`) for
  `annulus_test.py`
- `/compute_cartesian_path` -- already existed, unchanged

This works identically on any ROS2 distro with MoveIt2, since it never
imports the `moveit_py` Python package at all. `reset_arm.py` and
`collision_contacts.py` were simplified to import shared helpers
(`RobotIOClient`, `go_home`, `check_state_validity`) from `pick_place.py`
instead of duplicating their own MoveItPy setup.
`collision_contacts.py` in particular got much simpler: `/check_state_validity`'s
response already lists every colliding link pair directly
(`response.contacts`), so the old ACM-toggle-and-recheck workaround (built
to work around `moveit_py`'s bool-only collision check) was deleted
entirely, not just ported.

**Known, accepted behavior nuance:** collision checks now read the
gripper's *actual current* joint position for unspecified joints
(`is_diff=True`), where the old `moveit_py` code implicitly zeroed them.
Different from before, arguably more correct, flagged rather than hidden.

---

## Project restructure (before the moveit_py work)

Converted from an ad-hoc `scripts/` folder + separate `source/ros2_ws`
workspace into a single, proper ROS2 colcon workspace:

```
swarm_project/
├── src/
│   ├── swarm_pkg/                      # all Python scripts live here now
│   │   └── src/scripts/pick_place.py, annulus_test.py, reset_arm.py, ...
│   ├── mycobot_description/            # robot meshes/URDF (minimal subset
│   │                                       kept -- most of the vendor's
│   │                                       mycobot_ros2 repo was unused
│   │                                       clutter, deleted)
│   ├── mycobot_280pi_camera_moveit2/    # MoveIt config package
│   └── mycobot_hardware/                # NEW: real-hardware ros2_control plugin
├── pi_setup/                            # robot-side install script + requirements.txt
├── legacy/                              # old code, kept for reference
└── WORKFLOW.md                          # operational how-to (day to day commands)
```

`source/ros2_ws` and `source/mycobot_280_pi_OLD_unused` (~1000s of files for
robot models this project doesn't use) were deleted after confirming via
grep/dependency tracing that nothing referenced them.

---

## One xacro file, three hardware modes

`firefighter.urdf.xacro` + `firefighter.ros2_control.xacro` take a single
`hardware_mode` arg: `mock` (RViz-only, default) | `gazebo` (Gazebo Harmonic
sim, mars only) | `real` (physical robot, via `mycobot_hardware`). Same
URDF/joint list for all three; only the `<plugin>` in the `<hardware>` block
changes. Launch files select it via
`.robot_description(mappings={"hardware_mode": "..."})`.

---

## `mycobot_hardware`: the real-hardware ros2_control plugin

New C++ package (`src/mycobot_hardware/`), Galactic-only (see below).
`MyCobotSystem` implements `hardware_interface::SystemInterface`, but
instead of reimplementing the mycobot serial protocol in C++, it bridges to
`mycobot_bridge.py` -- a persistent Python process using `pymycobot`
(elephantrobotics' vendor driver) -- over a Unix domain socket
(`/tmp/mycobot_hardware_bridge.sock`, newline-delimited JSON). Rationale:
`pymycobot` is Python-only and already vendor-tested; reimplementing its
serial protocol in C++ would be redundant and risky.

**This package can only be built on Galactic.** Its `read()`/`write()`
signatures target Galactic's exact `hardware_interface` API (confirmed
against control.ros.org/galactic docs, not guessed), which differs from
Jazzy's (Jazzy's `read()`/`write()` take `(time, period)` args; Galactic's
take none). On mars, always build with:
```bash
colcon build --packages-skip mycobot_hardware
```
The user prefers plain `colcon build` normally, but this package is the one
standing exception -- it will not compile on Jazzy, and that's expected,
not a bug.

**Verification method used throughout this session:** since mars can't
compile Galactic's exact API, changes to this file were validated by
copying it to a scratch dir, patching *only* the `read()`/`write()`
signatures to Jazzy's shape, and compiling that scratch copy on mars to
catch ordinary C++ mistakes (typos, missing includes, logic errors)
separately from the one known, deliberate distro difference. Worth reusing
this technique for any further `mycobot_hardware` changes.

**Confirmed on real hardware:** serial port `/dev/ttyAMA0` @ baud `1000000`
is correct (bridge connects cleanly). `ros-galactic-backward-ros` had to be
added to `pi_setup/install_pi_galactic.sh` (missing transitive dependency
of `move_group` on Galactic -- without it, `move_group` dies instantly with
`libbackward.so: cannot open shared object file`).

---

## The Fix 1-7 saga (Galactic vs Jazzy API archaeology)

Each of these was discovered by actually running `real_robot.launch.py` on
the robot and reading the traceback -- not by guessing. All are committed
on `fix/project_structure` (`git log --oneline` shows them as "Fix N: ...").
Whoever branches off this should understand each of these before touching
adjacent code, since they represent real, confirmed API differences, not
style preferences:

1. **Package name convention** -- Galactic's `MoveItConfigsBuilder` has no
   `package_name=` override; it hardcodes `{robot_name}_moveit_config`.
   Fixed via an ament-index alias in `mycobot_280pi_camera_moveit2`'s
   `CMakeLists.txt` (installs the same `config/launch/worlds` content a
   second time under the name `firefighter_moveit_config`, plus a manual
   resource-index marker file). Purely additive, doesn't rename the real
   package.
2. Package names/syntax cleanup (minor).
3. **`.setup_assistant` schema differs**: Galactic's parser reads uppercase
   `URDF`/`SRDF` keys; the file (written by a newer Setup Assistant) only
   had lowercase `urdf`/`srdf`. Added uppercase mirrors alongside the
   originals (both needed, for different distros).
4. **`moveit_config.package_path` doesn't exist on Galactic** -- it's a
   newer `MoveItConfigs` attribute. Replaced with
   `get_package_share_directory("mycobot_280pi_camera_moveit2")` everywhere
   it was used (distro-independent).
5. **`xacro.load_yaml()` vs `load_yaml()`**: Galactic's xacro doesn't expose
   the `xacro` module inside `${}` expressions at all, only the bare
   function name. Newer xacro (Jazzy) still accepts the bare form too (with
   a deprecation warning) -- so `load_yaml()` (no prefix) is the one
   spelling that works on both. Fixed in `firefighter.ros2_control.xacro`.
6. **`request_adapters`/`response_adapters` type conflict**: Galactic's
   `PlanningPipeline` C++ constructor demands a single space-separated
   *string*; Jazzy's demands a string *array*. Neither format satisfies
   both (confirmed by triggering both exceptions in turn). Resolution:
   **dropped both keys from `ompl_planning.yaml` entirely** -- each distro
   falls back to its own built-in default adapter list when the parameter
   isn't set at all, which is exactly what Jazzy was already doing before
   this file existed.
7. **Trajectory timing safety net** -- direct side effect of Fix 6: dropping
   `response_adapters` also dropped `AddTimeOptimalParameterization`.
   Jazzy's fallback still times trajectories properly without it;
   Galactic's doesn't, so `arm_group_controller` rejected every trajectory
   ("Time between points 0 and 1 is not strictly increasing"). Fixed with
   `_ensure_monotonic_timing()` in `pick_place.py`'s `plan_motion()`: a
   no-op if the trajectory already has valid increasing times (Jazzy,
   Gazebo), otherwise assigns conservative constant-speed (`0.2 rad/s`,
   well under `joint_limits.yaml`'s `1.0 rad/s` per-joint max) timing as a
   safety net. Verified in isolation with a standalone unit test before
   touching real hardware.
8. **(Also fixed, not distro-related) `mycobot_hardware` startup race**:
   `real_robot.launch.py` starts `mycobot_bridge.py` and `ros2_control_node`
   simultaneously. The bridge needs real time to open the serial port
   before its socket exists; `on_activate()` was trying to connect once,
   immediately, and losing that race on *every* real-hardware run so far.
   Silent failure mode: `read()`/`write()` became permanent no-ops, but
   `joint_trajectory_controller` still reported "Goal reached, success!"
   from elapsed trajectory time alone -- **so the ROS logs looked like
   success while the arm never physically moved.** Fixed with a ~10s retry
   loop in `connect_bridge()`.

### Important standing caveat from item 8

**Galactic's `controller_manager` does not appear to block controller
activation even when a hardware component's `on_activate()` returns an
error.** `arm_group_controller` spawned and reported success even while
`mycobot_hardware` was failing to connect on every prior run. **ROS logs
saying "active" / "Goal reached" are not sufficient proof of real motion on
this setup -- always physically watch the arm.** This bit the assistant
once already in this session (claimed success from logs alone; the user
corrected it -- the arm hadn't moved at all).

---

## Known, accepted, NOT-yet-fixed gaps

- **Gripper contact detection cannot work as written on real hardware.**
  `pick_place.py`'s `gripper_close_until_contact()` needs effort/force
  feedback; `pymycobot`'s gripper API (`set_gripper_value`/
  `get_gripper_value`, 0-100 scale) exposes none at all.
  `mycobot_bridge.py` reports a constant `0.0` placeholder. A real fix
  needs a different signal (e.g. `is_gripper_moving()` going to 0 mid-close
  as a stall/contact proxy) -- not designed or built yet.
- **Joint velocity always reported as `0.0`** from `mycobot_bridge.py` --
  `pymycobot` exposes no velocity reading. Harmless placeholder; nothing in
  this project relies on real velocity feedback.
- **Pick/place coordinates need real-hardware recalibration.** `PICK_XYZ`,
  `PLACE_XYZ`, `APPROACH_HEIGHT`, `GRASP_OFFSET_Z` in `pick_place.py`
  (including the `SPAWN_HEIGHT_CORRECTION` constant) were tuned against
  **simulated** Gazebo mounting geometry. The real robot's physical
  mounting/reach almost certainly doesn't match. First real attempt at
  "Move to pre-grasp" failed OMPL entirely (`Unable to sample any valid
  states for goal tree`) -- expected calibration work, not a bug. User
  wants to do this themselves later using the existing `reach_probe.py` /
  `ik_probe.py` scripts.
- **Benign, unexplained warning** on every real/mock/gazebo launch:
  `Skipping virtual joint 'virtual_joints' because its child frame 'g_base'
  does not match the URDF frame 'world'` / `Joint 'virtual_joints' not
  found in model 'firefighter'`. Present across all hardware_modes and both
  distros, never blocked anything -- looked at but not investigated
  further this session.

---

## Working conventions established this session

- **Git**: user commits/pushes manually and prefers to `git pull` manually
  on the robot too (not automated). Remote:
  `https://github.com/kandge1/swarm_project.git`, branch
  `fix/project_structure`. When the assistant makes a fix, commit + push
  immediately so the user can pull on the robot -- changes made only in the
  assistant's local sandbox do NOT reach the robot on their own (this
  caused one full wasted debug round-trip earlier in the session, since
  edits sat uncommitted while the user rebuilt on the robot).
- **Build preference**: plain `colcon build` (no `--packages-select`) on
  both machines, except mars must add `--packages-skip mycobot_hardware`.
- **Verification discipline**: don't claim something works from a log
  snippet alone if it involves real hardware motion -- confirm what was
  physically observed. This was violated once (see standing caveat above)
  and corrected by the user.
