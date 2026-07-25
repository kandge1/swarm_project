# Cyclone DDS Integration — Session Log (2026-07-24)

Branch: `feature/cyclone_dds_inegration`

## Goal

Split MoveIt2 compute: run `move_group` (IK, OMPL planning) on the workstation
("mars", Jazzy) instead of on the myCobot 280 Pi robot ("er", Galactic),
because the Pi is CPU-constrained for planning. Trajectory *execution* stays
on the Pi via `ros2_control` / `FollowJointTrajectory`. This requires DDS
discovery to work between mars and the robot over the campus network, which
blocks UDP multicast (confirmed: ping/SSH work, but `ros2 topic echo` did
not see a talker across machines) -- the fix is Cyclone DDS with a static
unicast peer list instead of the default Fast DDS + multicast discovery.

**Bottom line after this session: the DDS transport layer works. Real
hardware motion is still unreliable, and that unreliability turned out to
be independent of Cyclone DDS / the split architecture -- see "Critical
finding" near the end.**

---

## What was built

### 1. `swarm_network` package (`src/swarm_network/`)
New package holding DDS config, shared by both machines.

- `package.xml`, `CMakeLists.txt` -- installs `config/` to
  `share/swarm_network/config/`.
- **Two Cyclone DDS XML configs, not one** (see "Per-distro config" below):
  - `config/cyclonedds_galactic.xml` -- for the robot.
  - `config/cyclonedds_jazzy.xml` -- for mars.

### 2. Split launch files (`src/mycobot_280pi_camera_moveit2/launch/`)
The original `real_robot.launch.py` (single-machine: bridge + rsp +
ros2_control_node + move_group + rviz + spawners, all on the Pi) is
**unchanged** and still works as a single-machine fallback/reference.

New files split it in two:
- `real_robot_hardware.launch.py` (robot side): `mycobot_bridge.py`,
  `robot_state_publisher`, `ros2_control_node`, controller spawners. No
  `move_group`, no `rviz`.
- `real_robot_planning.launch.py` (mars side): `move_group` and `rviz` only,
  built with `hardware_mode=real` so `robot_description` matches the robot
  even though mars can't build the Galactic-only `mycobot_hardware` plugin
  itself (`move_group` never loads that plugin -- only `ros2_control_node`
  does, and that stays on the robot).

Spawners in `real_robot_hardware.launch.py` are **staggered AND retried**:
each of the three (`joint_state_broadcaster`, `arm_group_controller`,
`gripper_group_controller`) runs in its own `TimerAction` (3s/6s/9s delay)
wrapping a 5-attempt shell retry loop (`for i in 1 2 3 4 5; do ros2 run
controller_manager spawner <name> && exit 0; ...; sleep 2; done`). This is
a mitigation for a known-unfixed upstream Cyclone DDS discovery race (see
below) -- not a full fix, since no full fix exists yet.

### 3. Environment (both machines' `~/.bashrc`, after ROS is sourced)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_<galactic|jazzy>.xml
```
**Must be placed AFTER `source /opt/ros/<distro>/setup.bash` and
`source .../install/setup.bash`** -- `ros2 pkg prefix` doesn't resolve
before ROS is sourced, and this was a repeated source of confusion this
session (stale/broken `CYCLONEDDS_URI` silently pointing at nothing, or at
an old removed filename, because a shell had it exported before a later
`.bashrc` edit or before a package existed/was renamed).

**Footgun, hit multiple times:** editing `.bashrc` does NOT change an
already-running shell's exported vars. Always open a **fresh terminal**
(or explicitly re-`export`) after editing DDS-related `.bashrc` lines,
and verify with `echo $CYCLONEDDS_URI` before trusting a test.

### 4. WORKFLOW.md updates
- New "DDS Unicast Discovery" subsection under Setup (install
  `ros-<distro>-rmw-cyclonedds-cpp` on both machines, env var exports).
- New "Split-Compute Real Hardware Workflow" section: how to launch the
  hardware half (robot) and planning half (mars), plus a troubleshooting
  entry for the DDS verification test (`demo_nodes_cpp talker` /
  `topic echo /chatter`).
- Project Structure updated to include `swarm_network`.

---

## Machine details

- mars (workstation): Jazzy, IP `172.27.89.157` (from `hostname -I` /
  `ip route`, on `wlp0s20f3`), repo at `~/swarm/swarm_project`.
- robot ("er"): Galactic, Ubuntu 20.04.4, aarch64 Raspberry Pi, IP
  `172.30.6.165`, repo at `~/swarm_project` (no `swarm/` parent dir --
  different from mars). SSH user `er`.
- `ros-galactic-rmw-cyclonedds-cpp` (0.22.6) and
  `ros-jazzy-rmw-cyclonedds-cpp` were pre-installed by the user before this
  session started.

---

## Problems hit, in the order they were debugged

### Problem 1: Unrelated Bluetooth service holding `/dev/ttyAMA0`
Found via `sudo lsof /dev/ttyAMA0` -> a root-owned
`/home/er/mycobot_pi_bluetooth/uart_peripheral_serial.py`, launched by
`bt_auto_start.sh` via `/etc/rc.local` on every boot (Elephant Robotics'
own Bluetooth-remote-control setup for the myCobot app), was contending
with `mycobot_bridge.py` for the same serial port.

**Fix applied:** killed the process for this session only (`sudo kill
<pid>`). **`/etc/rc.local` was deliberately left untouched** -- it will
respawn on next reboot; user chose not to permanently disable it (Bluetooth
control might still be wanted). If serial flakiness resurfaces, check for
this process again: `ps aux | grep uart_peripheral`.

This was a real bug but **did not fully explain** the `did not reply within
200 ms` warnings -- those persisted after killing it.

### Problem 2: `AllowMulticast` / same-host discovery
Initial `cyclonedds.xml` used `AllowMulticast=false` (correct for the
cross-machine link, since campus Wi-Fi blocks multicast) with a static
`<Peers>` list of both machines' real IPs. This broke **same-host**
discovery: `controller_manager` spawners on the robot couldn't discover
`ros2_control_node`'s own participant on the same machine, crashing with:

```
rclpy._rclpy_pybind11.RCLError: Failed to get node names: empty node name
returned by the RMW layer, at .../rcl/graph.c:360
```

Tried `AllowMulticast=spdp` (permit multicast for discovery only, never
data) -- helped mars, was inconsistent on the robot (see next).

### Problem 3: Per-distro config split (the two-file setup)
Tried adding `<Peer address="127.0.0.1"/>` to force same-host discovery
through the static unicast mechanism instead of multicast at all.

- **On mars (Jazzy):** fixed it completely and immediately -- `ros2 node
  list` went from seeing nothing (not even mars's own `move_group`) to
  seeing everything instantly.
- **On the robot (Galactic, Cyclone DDS 0.22.6):** made it **strictly
  worse** -- spawners went from "occasionally succeeds" to "5/5 retries
  fail, every time." Galactic's older Cyclone DDS evidently doesn't handle
  a self-referential unicast peer cleanly.

**Resolution:** split into two files (`cyclonedds_jazzy.xml` uses
`AllowMulticast=false` + `127.0.0.1` peer; `cyclonedds_galactic.xml` uses
`AllowMulticast=spdp`, no `127.0.0.1` peer). Each machine's `.bashrc` points
at its own file. This is *not* fully reliable on the robot either (see
"Critical finding" below) but is the best config found.

### Problem 4: `empty node name returned by the RMW layer` -- researched, is a KNOWN unfixed bug
Spawned a research subagent. Findings (sourced):
- [ros2/rclpy#1448](https://github.com/ros2/rclpy/issues/1448) -- exact
  same error/line, open, unresolved.
- [ros2/ros2#489](https://github.com/ros2/ros2/issues/489) -- root cause:
  `rmw_get_node_names()` can return an empty string for a DDS participant
  whose ROS graph metadata (name/namespace, carried in DDS `USER_DATA`)
  hasn't finished propagating yet. `rcl`'s `graph.c:360` hard-fails on that
  instead of filtering/retrying. This is a **discovery-timing race**, not a
  config bug -- no CycloneDDS XML setting fixes it.
- Static unicast peers (vs LAN multicast) **widen** this race window
  (higher/more variable discovery latency), which is why it surfaces more
  on this setup than on a typical LAN.
- The separate `Failed to parse type hash for topic ... from USER_DATA
  '(null)'` warnings seen throughout are a **different, cosmetic, confirmed
  benign** issue -- cross-distro (Galactic vs Jazzy) type-hashing scheme
  mismatch ([ros2/rmw_cyclonedds#567](https://github.com/ros2/rmw_cyclonedds/issues/567)).
  Not related to the empty-node-name crash. Safe to ignore.
- Community workaround (no upstream fix exists): retry at the call site.
  This is what the staggered+retrying spawner wrapper does.

### Problem 5: `serdata.cpp:354` "string data is not null-terminated" / "invalid data size" -- was a RED HERRING
Hit while calling `ros2 control list_controllers`. Spawned a second
research subagent; findings pointed at a CDR/ABI type-support mismatch
between client and server as the textbook cause of this exact error
pattern.

**However:** when checked, the actual cause in every case it occurred here
was simpler -- **the service server (`ros2_control_node`) had already
exited** (terminal closed/died) by the time the CLI call was made. Cyclone
DDS produces this confusing, misleading deserialization error when trying
to reach an absent/dead service rather than a clean "not found." Confirmed
by restarting the hardware launch cleanly and re-testing: `ros2 control
list_controllers` succeeded instantly, no `serdata.cpp` errors, once a live
server actually existed.

**Lesson:** if this error reappears, check `ps aux` for the target node
being alive FIRST, before assuming a real ABI/message mismatch.

### Problem 6: `ros2 control list_controllers` (a service call) times out cross-machine, but works fine locally
Confirmed: same command, same domain, same everything -- succeeds
instantly when run **on** the robot, but times out after 30s (3 retries)
when run from **mars** against the robot's controller_manager, even while
mars can clearly see all the robot's topics/actions
(`rt/arm_group_controller/...`, `rt/joint_states`, etc. all show up in the
type-hash warnings, proving pub/sub discovery works fine cross-machine).

Likely explanation (from the earlier research, not re-verified in depth):
Cyclone DDS service-call (request/reply RPC) reliability across a WAN-like
unicast link is a known weaker point than topic pub/sub --
[ros2/rmw_cyclonedds#74](https://github.com/ros2/rmw_cyclonedds/issues/74).

**This was deliberately NOT chased further** -- `move_group` talks to
controllers via the `FollowJointTrajectory` **action** interface (through
`moveit_simple_controller_manager`), not via this service, and actions were
separately confirmed to cross machines fine (goals `Received`/`Accepted`/
`Goal reached` all showed up correctly on the robot when sent from mars).
`ros2 control list_controllers` cross-machine is a nice-to-have diagnostic,
not a blocker -- can revisit if it matters later.

---

## Critical finding (end of session) -- READ THIS FIRST TOMORROW

After getting the full split pipeline nominally working (mars plans, robot
executes, action goals cross machines and report "Goal reached, success!"),
running `pick_place.py` from mars produced **only gripper motion -- the arm
itself did not physically move**, despite the controller logging success.

This matched a known caveat already documented in `WORKFLOW.md`
(Troubleshooting, "Fix 7"): **"Goal reached, success!" in the logs is not
proof of real motion on this Galactic setup** -- `controller_manager` does
not appear to block controller activation even when the hardware
component's writes are failing/no-op'ing, so the controller can report
success purely from elapsed trajectory time while the arm never actually
tracked it.

**Control test performed to isolate the cause:** ran `real_robot.launch.py`
-- the ORIGINAL, pre-existing, single-machine launch file. No DDS split, no
mars, no unicast peers, no cross-machine anything involved.

**Result: the exact same symptoms appeared, entirely on one machine:**
- Same `empty node name returned by the RMW layer` spawner race (Problem 4
  above) -- happens purely from `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  being set at all, regardless of unicast/multicast/split config.
- Same continuous `mycobot_bridge.py did not reply within 200 ms` warnings.
- **New, more serious symptom observed here for the first time:** running
  `pick_place.py`, killing/restarting the whole launch, then running
  `pick_place.py` again -- the arm physically moved **from the FIRST run's
  commands**, but only *after* the second run had already started, roughly
  20 seconds late. I.e., `mycobot_bridge.py`'s write/command path is
  queuing or otherwise executing commands asynchronously and out of sync
  with what the controller has already reported as a completed goal.

### Conclusion
**Moving to Cyclone DDS neither caused nor fixed the real-hardware motion
reliability problem.** The instability is reproducible in pure
single-machine mode with the stock, pre-existing launch file. The actual
blocker for the project's real goal (get IK/planning off the Pi's CPU) is
**not** the DDS transport -- it's the `mycobot_hardware` <->
`mycobot_bridge.py` <-> serial write path, which appears to have been
unreliable/laggy independent of anything done this session.

The DDS unicast work itself is **done and functional**: topics, actions,
and cross-machine discovery all confirmed working, once the per-distro
config split and spawner retry mitigation were in place. That part of the
original goal (make DDS discovery work over the campus network) is
solved. It should NOT be the focus of further debugging.

---

## What to do next session

**Do NOT keep tuning DDS config.** The DDS layer is as solid as it's going
to get without an upstream Cyclone DDS fix landing (the `empty node name`
race has no config workaround; it's mitigated, not eliminated, by the
retry wrapper).

**Instead, debug `mycobot_bridge.py`'s write/command path directly** --
this is the real, actionable next problem:
- Add explicit timestamps/logging around every `read_state()` and
  `write_command()` call in `src/mycobot_hardware/scripts/mycobot_bridge.py`
  to see actual serial round-trip latency and whether writes are queuing.
- Check whether `pymycobot`'s `send_angles()` call is blocking longer than
  expected, or whether something in the Unix-socket request/response loop
  (`handle_client`) is causing requests to pile up.
- Consider whether the `did not reply within 200 ms` timeout on the C++
  side (`mycobot_system.cpp`'s `send_request()`, currently hardcoded via
  `timeout_ms`) is simply too aggressive for what the bridge/serial link
  can actually sustain at 100Hz, independent of any DDS discovery noise.
- The isolated direct-socket test earlier in this session (bypassing
  ROS2/DDS entirely, talking to the bridge's Unix socket directly) showed
  **consistent ~20ms round trips with valid data** -- so the bridge itself,
  queried in isolation with no `ros2_control_node` in the loop, is fast and
  healthy. That makes the 20-second-lag symptom even more suspicious: it
  suggests the problem is specifically in how `ros2_control_node`'s 100Hz
  read/write loop interacts with the bridge under real load, not the raw
  serial link speed itself.
- Worth checking git history / WORKFLOW.md's "Known gaps" section again --
  some of this may already be partially flagged (e.g. Fix 7, Fix 8) as
  known-incomplete areas from before this session even started.

## Current branch state
All work is committed and pushed to `feature/cyclone_dds_inegration` on
both the mars-visible remote and pulled onto the robot. Commit history for
this session (chronological):
1. Add `swarm_network` DDS configuration package
2. Update WORKFLOW.md for DDS setup (before it existed as a real package)
3. Simplify cyclonedds.xml for Galactic Cyclone DDS compatibility (dropped
   newer `<Interfaces>` schema Galactic's older parser rejected)
4. Split real_robot launch into hardware (Pi) and planning (mars) halves
5. Fix AllowMulticast=false breaking same-host DDS discovery on the robot
   (-> `spdp`)
6. Stagger controller spawners to avoid Cyclone DDS discovery race
7. Retry controller spawners against rmw_cyclonedds_cpp discovery race
   (shell retry loop)
8. Fix same-host DDS discovery by adding 127.0.0.1 as an explicit peer
   (this turned out mars-only-beneficial)
9. Split DDS config per-distro: Galactic's Cyclone DDS handles peers
   differently (`cyclonedds_galactic.xml` / `cyclonedds_jazzy.xml`)

No uncommitted work outstanding as of end of session.
