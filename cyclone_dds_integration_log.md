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

## Current branch state (as of end of Problem-6 session)
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

No uncommitted work outstanding as of end of that session.

---

## Session 2 (2026-07-25 night -> 2026-07-26 morning): the "3-goal wall" and real motion debugging

Picked up per "What to do next session" above: DDS transport was declared
solid, focus moved to `mycobot_hardware` <-> `mycobot_bridge.py` <-> serial
write path and real-motion reliability, using `pick_place.py` (MoveIt2
IK/OMPL planning on mars, execution on the robot) as the end-to-end test.

### Fix A: `mycobot_bridge.py` serial write blocking the control loop
`ros2_control_node` runs its read/write loop at 100Hz. `mycobot_bridge.py`'s
Unix-socket request handler was doing serial I/O to the arm **synchronously
in the same thread** that services socket requests, so a slow serial
write/read stalled the whole 100Hz loop behind it.

**Fix:** decoupled the Unix socket handler from serial I/O using a
background thread. This is what got the arm physically moving for
simple/small trajectories at all (home position, gripper open/close) --
before this fix, motion was essentially nonexistent or wildly laggy
regardless of DDS.

### Fix B: `mycobot_system.cpp` request/reply desync
`mycobot_hardware/src/mycobot_system.cpp` (the C++ `ros2_control`
`SystemInterface`) talks to `mycobot_bridge.py` over the same Unix socket.
When a request timed out (`did not reply within 200 ms`, see Session 1
Problem 5's cousin), the *late* reply for the timed-out request could still
arrive and get read as the reply to a *subsequent* request, corrupting all
future reads until the desync self-corrected or errored out.

**Fix:** added request ID tagging so a stale/late reply for an
already-abandoned request is detected and discarded instead of consumed by
the next request.

### The "3-goal wall" (the main mystery of the night)
With Fixes A and B in place, a clear, extremely repeatable pattern emerged
across **many** test runs, with different scripts, different trajectory
sizes, `use_sim_time` on and off, and multiple DDS config variants:

- The first ~3 action goals (typically: 1 arm move + 2 gripper moves, or
  similar small mix) succeed completely normally -- goal received, accepted,
  robot moves, "Goal reached, success!" logged, all within normal time.
- Then, **simultaneously**, `robot_state_publisher` AND `ros2_control_node`
  on the robot both start emitting repeated `serdata.cpp:354` "invalid data
  size" / "string data is not null-terminated" deserialization errors.
- From that point on, **no further "Received new action goal" ever appears
  in the robot's log for the rest of that process's lifetime** -- the
  process is not crashed, `/joint_states` may keep publishing, but action
  goals sent from mars simply vanish before reaching `arm_group_controller`.

**Ruled out, with evidence:**
- **Not size-dependent.** Tiny (~500 byte) single-waypoint goals hit the
  wall at the same count as large (~9KB) 30-waypoint trajectories.
- **Not time-dependent.** The wall hit whether the 3 goals were sent
  seconds apart or minutes apart.
- **Not a permanent robot-wide/DDS-wide failure.** A *fresh* process (e.g.
  `joint_trajectory_test.py`, run moments after an "old" process's goals
  stopped being received) could immediately and successfully send a new
  goal and get it accepted. This was the key clue: the failure is scoped to
  something in the *old process's* state, not the robot or the DDS network
  as a whole.
- **`use_sim_time:=true` removed from `pick_place.py`'s `rclpy.init()`**
  (it was set for historical/Gazebo-testing reasons but this script only
  ever targets real hardware) -- theorized as a possible contributor since
  `self.get_clock().now()` never advances without a `/clock` publisher
  (causing a **separate, confirmed, silent-infinite-loop bug** in
  `current_joint_positions()`'s timeout logic, fixed by switching to
  `time.monotonic()`). Removing `use_sim_time` was correct to do regardless,
  but **did not fix the 3-goal wall by itself** -- same failure persisted
  after removing it.
- **QoS durability mismatch on the old status-topic subscriptions** (see
  "Fix D" below) was real but was a *symptom-detection* bug, not the cause
  of goals failing to arrive.

**Attempted DDS-config fix (inconclusive/likely not the real fix):** added
`<Internal><Watermarks><WhcHigh>500kB</WhcHigh></Internal>` to both
`cyclonedds_galactic.xml` and `cyclonedds_jazzy.xml` (commit `49ac33d`),
theorizing a default writer/reader history high-water-mark being hit after
a handful of samples. Explicitly committed as **UNVERIFIED**. Still in
place as of this writing, but Fix C below (a pure Python-level fix) is what
actually resolved the goal-delivery problem in confirmed testing, so this
watermark change's effectiveness is unconfirmed and it may be inert. A
separate attempt to raise `FragmentSize`/`MaxMessageSize` to 65500B was
tried and **reverted** (`git revert`) after it caused an unrelated
regression (`/plan_kinematic_path service not available` -- actually a red
herring, `move_group`/rviz just weren't running in that terminal) without
fixing the original wall.

### Fix C (the confirmed fix): recreate `ActionClient` fresh before every goal send
`RobotIOClient` in `pick_place.py` originally created its `ActionClient`
instances (`self._arm_client`, `self._gripper_client`) **once**, in
`__init__`, and reused them for every goal for the process's whole
lifetime. This is the standard/recommended `rclpy` pattern -- but on this
setup, the long-lived `ActionClient`'s internal DDS writer/reader state
appears to degrade after ~3 goals in a way that matches the "3-goal wall"
signature exactly.

**Fix:** destroy and recreate the `ActionClient` immediately before every
single goal send, for both arm and gripper:
```python
self._arm_client.destroy()
self._arm_client = ActionClient(self, FollowJointTrajectory, self._arm_action_name)
if not self._arm_client.wait_for_server(timeout_sec=20.0):
    self.get_logger().error("arm_group_controller action server not available")
    return False
goal = FollowJointTrajectory.Goal()
goal.trajectory = joint_trajectory
return self._send_goal_and_wait(self._arm_client, goal, "arm")
```
(same pattern for `self._gripper_client` in `gripper_move_to`). Action
topic names are stored as `self._arm_action_name` /
`self._gripper_action_name` strings so they can be reused across
recreations.

**Confirmed working, twice, on real hardware:**
- First confirmation (late night): goal #4 (which had failed in every prior
  run at that exact position in the sequence) successfully reached
  `arm_group_controller` -- robot log showed `Received new action goal` ->
  `Accepted` -> `Goal reached, success!` in `1785030238.253` ->
  `1785030238.378` (~125ms).
- Second confirmation (next morning, fresh relaunch of both machines):
  running `pick_place.py`'s pre-grasp step (goal #4 in that run: 2 home
  goals + 2 gripper goals preceded it) reached `arm_group_controller`
  successfully -- `Received new action goal` -> `Accepted` -> `Goal reached,
  success!` from `1785074747.180` -> `1785074747.292` (~112ms). **No
  recurrence of the 3-goal wall in either confirmation run, or in any
  subsequent testing that night/morning.**

This is the most significant confirmed fix of Session 2: **the DDS/
ActionClient goal-delivery problem is resolved.**

### Fix D: switched goal-completion detection from action status topic to `/joint_states` polling
The old `_send_goal_with_retry` waited for completion by subscribing to the
action's `GoalStatusArray` status topic. Two problems:
1. **QoS mismatch:** the subscription used a bare integer (`10`) as its QoS
   argument, which defaults to `VOLATILE` durability, while action servers
   publish status with `TRANSIENT_LOCAL` -- a genuine incompatibility that
   silently drops delivery. (Fixed at the time with an explicit
   `QoSProfile`, but this whole subscription-based mechanism was later
   deleted entirely, see below -- so this fix is now moot/dead code that no
   longer exists.)
2. Even with QoS fixed, status-topic delivery back to mars was empirically
   correlated with the `serdata.cpp:354` error bursts and was unreliable,
   even though the *goal* had actually been delivered and executed
   correctly on the robot side (confirmed via robot-side logs showing
   `Goal reached, success!` while mars's process was still waiting/timing
   out on the status topic).

**Fix:** replaced status-topic waiting entirely with polling
`self._joint_positions` (populated by a `/joint_states` subscription via
`_on_joint_state`, which was rock-solid at ~100Hz all night with zero
delivery issues) for convergence to the goal's target joint positions
within a tolerance. Implemented in `_send_goal_and_wait`:
```python
target = {name: pos for name, pos in
          zip(goal.trajectory.joint_names, goal.trajectory.points[-1].positions)}
last_point_sec = (goal.trajectory.points[-1].time_from_start.sec +
                  goal.trajectory.points[-1].time_from_start.nanosec / 1e9)
earliest_done = time.monotonic() + last_point_sec
deadline = time.monotonic() + max(timeout_sec, last_point_sec + 5.0)
while time.monotonic() < deadline:
    rclpy.spin_once(self, timeout_sec=0.1)
    reached = all(
        name in self._joint_positions and
        abs(self._joint_positions[name] - pos) <= settle_tolerance
        for name, pos in target.items()
    )
    if reached and time.monotonic() >= earliest_done:
        return True
```
Includes periodic diagnostic prints of target vs. current joint positions
every 5s while waiting. `settle_tolerance` was loosened from `0.02` to
`0.05` rad after diagnostics showed the arm consistently settles
~0.014-0.031 rad away from its exact commanded target -- this is real
hardware precision, not a bug, and 0.02 was spuriously failing correct
completions. The old `GoalStatusArray`-based subscriptions/handlers/imports
(`_arm_status_sub`, `_gripper_status_sub`, `_on_goal_status`,
`_goal_statuses`, plus `action_msgs.msg.GoalStatus(Array)`,
`unique_identifier_msgs.msg.UUID`, the `rclpy.qos` QoS imports, and `uuid`)
were all deleted since this mechanism fully replaced them.

### `joint_trajectory_test.py` as the control/reference script
This script imports and reuses `RobotIOClient` from `pick_place.py` but
calls plain `rclpy.init()` (no `use_sim_time`). It worked consistently all
night (confirmed moving the arm to zero/squat) even during periods when
`pick_place.py` was hitting the 3-goal wall. Comparing the two was the key
clue that led to Fix C -- it showed what a working process looked like and
narrowed the difference down to `pick_place.py`'s long-lived `ActionClient`
reuse pattern.

### Day-boundary false alarm
A ~13 hour gap in robot log timestamps (`1785030238` -> `1785074159`) was
briefly misdiagnosed as a new regression (total `/joint_states` silence,
action server unreachable). User clarified it was simply that the earlier
session was "last night" and testing resumed "the next day" -- terminals
had been closed overnight, not a technical regression. Resolved by
relaunching both machines fresh. Not a real bug; noted here only so it's
not re-investigated as one.

---

## CURRENT UNRESOLVED PROBLEM (start here next session)

With Fixes A-D all in place and reconfirmed this morning on a fresh
relaunch of both machines:

- `joint_trajectory_test.py --to zero` **works correctly** -- arm visibly
  moves to all-zero, `/joint_states` streams real values throughout.
- Running `pick_place.py` immediately after: **home succeeds, both gripper
  goals succeed**, arm visibly moves for these. Then the **pre-grasp step**
  (the first goal derived from real MoveIt2 IK -- a much larger joint-space
  move, roughly `[1.83, -0.72, -0.81, -0.04, 0, 1.83]` rad, vs. the small
  moves used by home/gripper) is sent:
  - **The goal DOES successfully reach the robot** -- robot log confirms
    `Received new action goal` -> `Accepted` -> `Goal reached, success!`,
    this time in only ~112ms (`1785074747.180` -> `.292`).
  - **But the arm does not physically move.** `pick_place.py`'s own
    `/joint_states`-based completion polling (Fix D) presumably timed out
    or matched a false target, and the user visually confirmed no motion
    occurred for this step ("in the same terminal the arm goes from all 0
    to squat, but never moves after that").

**This is almost certainly NOT the same bug as the 3-goal wall.** Goal
delivery is fast (~112ms) and clean -- this is a *different*, older,
already-documented issue: the "Critical finding" from Session 1 (see
above, and `WORKFLOW.md` Troubleshooting "Fix 7") that **Galactic's
`controller_manager` can report "Goal reached, success!" purely from
elapsed trajectory time, without the hardware component's writes actually
having taken effect** -- i.e. it does not block/verify on real hardware
tracking. A ~112ms round trip for what should be a multi-second, large
joint-space trajectory is itself suspicious and consistent with this: the
controller may be accepting and "completing" the goal almost instantly
rather than actually executing it.

**Suspected next debugging targets, in likely order of value:**
1. Add explicit before/after logging in `mycobot_hardware/src/mycobot_system.cpp`'s
   `write()` (the `ros2_control` hardware interface write, which forwards
   commands to `mycobot_bridge.py`) specifically for the pre-grasp goal, to
   see whether the large joint delta is actually being sent to the bridge
   at all, or whether something about the trajectory (size, joint order,
   velocity/acceleration limits, IK-derived precision) causes it to be
   silently dropped/no-op'd differently than the small home/gripper moves
   that do work.
2. Compare the pre-grasp trajectory's actual `JointTrajectory` message
   (waypoint count, joint order, velocities) against the known-working
   home/gripper trajectories -- is it a single large jump vs. multiple
   interpolated waypoints? `pymycobot`'s `send_angles()` may behave
   differently (e.g. silently clamp, reject, or require different args)
   for large joint-space moves than small ones.
3. Re-check `mycobot_bridge.py`'s background write thread (Fix A) under
   this specific large-trajectory case -- confirm the write actually gets
   enqueued and dequeued, with timestamps, not just that the socket request
   returns quickly (a quick return only proves the *request* was received,
   not that the arm write succeeded).
4. Consider whether `settle_tolerance=0.05` combined with the *actual*
   (non-)motion means `_send_goal_and_wait` might be misreporting failure
   even if the arm eventually crept partway there -- print the actual final
   `/joint_states` values reached for this specific step to rule out "it
   moved a little but not enough" vs. "it never moved at all."

### Current exact code state (as of end of Session 2)
- `pick_place.py`: Fix C (fresh `ActionClient` per goal) and Fix D
  (`/joint_states`-polling completion detection, 0.05 rad tolerance) both
  in place; `use_sim_time` removed from `rclpy.init()`; old status-topic
  subscription code fully deleted.
- `cyclonedds_galactic.xml` / `cyclonedds_jazzy.xml`: `FragmentSize`/
  `MaxMessageSize` change reverted; `WhcHigh` 500kB watermark still present
  but unverified and likely not the actual fix (Fix C is).
- `mycobot_bridge.py`: background-thread socket/serial decoupling (Fix A)
  in place.
- `mycobot_system.cpp`: request ID tagging (Fix B) in place.

No uncommitted work outstanding as of end of Session 2 except any
diagnostic print statements added for the pre-grasp investigation -- check
`git status` / `git diff` at the start of next session before assuming a
clean tree.

---

## Session 3 (2026-07-26 morning): root cause found and measured

Everything below was established by measurement, not inference. Two things
turned out to be wrong in the Session 2 write-up above, and correcting them
is what unblocked the diagnosis.

### Correction 1: Fix C was never actually confirmed. Goal #4 has failed 3/3 runs.

Both "confirmed working" observations in Session 2's Fix C were **misattributed
timestamps**. From mars's own `~/.ros/log` files, all three runs are identical
to the second:

| | run A (`move_group_9319`) | run B (`move_group_15073`) | run C (`move_group_10379`) |
|---|---|---|---|
| plan #1 (`go_home`) | 1785030158.699 | 1785074665.937 | 1785074236.107 |
| plan #2 (pre-grasp) | 1785030177.128 | 1785074686.076 | 1785074257.176 |
| gap | 18.4s | 20.1s | 21.1s |
| `never converged within 60.0s` | **1785030237.201** | **1785074746.151** | (killed) |
| plan #3 (final `go_home`) | 1785030237.202 | 1785074746.151 | — |

The `Received new action goal -> Goal reached, success!` bursts cited as proof
that Fix C worked (`1785030238.253 -> .378`, and `1785074747.180 -> .292`)
both land **1 second AFTER the pre-grasp goal had already timed out**, i.e.
they belong to plan #3, the **final `go_home`** — which "succeeds" trivially
and in ~112ms because the arm was already sitting at home and never left.

So the `~112ms` figure was never anomalous, and there is **no evidence the
pre-grasp goal was ever received at all**. Goal #4 has failed in every run.

### Correction 2: `/plan_kinematic_path` returns NO time parameterization

Dumped real responses from the live `move_group` on mars (robot not needed;
faked `/joint_states` at home). Every OMPL plan comes back with
`time_from_start = 0` on every point and **empty `velocities`/`accelerations`**.

Root cause: `ompl_planning.yaml` omits `response_adapters` to dodge the
Galactic/Jazzy type conflict, and its comment claims both distros then fall
back to built-in defaults. That is true for *request* adapters but **false for
response adapters on Jazzy**. move_group's own log says, for the ompl pipeline
specifically:

```
[WARN] ...planning_pipeline]: No planning response adapter names specified.
```

while the stomp / pilz / chomp pipelines in the same log each go on to
`Loaded adapter 'default_planning_response_adapters/AddTimeOptimalParameterization'`.
**OMPL alone ends up with zero response adapters.**

Consequence: `pick_place.py`'s `_ensure_monotonic_timing()` — documented as a
"pure safety net" — is in fact the **only time parameterization in the entire
system, on both machines, on 100% of real plans**, and its
`_FALLBACK_MAX_JOINT_SPEED` is the real arm's actual commanded speed.

### The measured difference between trajectories that move the arm and ones that don't

| | pre-grasp (FAILS) | `joint_trajectory_test --to zero` (WORKS) |
|---|---|---|
| waypoints | **21** | **1** |
| total duration | 9.14 s | 3.0 s |
| commanded joint speed | 0.20 rad/s | 0.52 rad/s |
| velocities present | no | no |
| `header.stamp` | 0 (starts now — not a clock-skew bug) | 0 |

### THE DECISIVE TEST — the pre-grasp pose is fine, and so is everything else

Sent the exact failing IK solution as a **hand-built single-waypoint, 3 s**
goal via the new `joint_trajectory_test.py --degrees`:

```
--degrees 104.74 -41.17 -46.49 -2.35 0 104.74
```

**The real arm swung 1.8146 rad (104°) from home to the pre-grasp pose and
settled within 0.0246 rad** (`/joint_states`: `103.97 -42.09 -47.90 -3.42
-0.35 103.97`). Goal accepted, delivery confirmed, motion confirmed.

So the pose is reachable, `send_angles` accepts it, the bridge write path
works, and the cross-machine DDS link delivers goal #N fine. **None of those
are the problem.** What fails is specifically the *multi-waypoint, slow,
streamed* trajectory.

(Aside: the robot's nodes never appear in `ros2 node list` from mars even when
it is fully up — that's the same cross-distro `USER_DATA` metadata gap already
documented, and it is also why `ros2 action info` reports `Action servers: 0`
for a server that is demonstrably working. Do not use either as an up/down
check; use `ros2 topic echo --once /joint_states`.)

### Root cause: a servo interface driving a point-to-point API

`joint_trajectory_controller` is a **servo** interface — it interpolates the
planned path into a fresh position setpoint every control cycle (100 Hz) and
expects the hardware to track it. `pymycobot`'s `send_angles(angles, speed)`
is the opposite: a **point-to-point move** the arm's firmware executes
asynchronously over hundreds of ms, which **aborts and restarts** whatever
move is already in progress.

`mycobot_bridge.py`'s serial loop forwards only the newest setpoint once per
iteration, and one iteration is a full serial round trip (500–1500 ms
measured). So the 9.14 s / **920-setpoint** pre-grasp trajectory reaches the
arm as roughly **5–20 `send_angles()` calls**. At a fixed `speed=50` (~1.0
rad/s, 5× faster than the 0.2 rad/s the trajectory asks for) each call darts
~0.1–0.2 rad ahead in ~0.1 s, then the arm sits still for ~0.9 s until the
next one lands and aborts it. Net motion is a stutter covering a fraction of
the distance — and since `ros2_controllers.yaml` declares **no `constraints:`
block** for `arm_group_controller`, the controller has no goal tolerance to
check and reports "Goal reached, success!" purely from elapsed time.

**Single-waypoint trajectories escape this entirely**: once the duration
elapses, JTC holds ONE constant target forever, so exactly one `send_angles()`
runs to completion uninterrupted and the firmware drives the whole way there.
**Every motion this project has ever confirmed on real hardware was that
post-trajectory hold, not trajectory tracking.** That also explains Session
1's "commands executed ~20 s late" symptom.

### Fixes applied this session

- `mycobot_bridge.py` `_match_speed()`: scale the pymycobot speed to the
  *actual setpoint rate* instead of always using `--speed`. Turns the
  dart-and-stall into continuous motion; the aborts stop mattering because
  each replacement command starts near where the arm already is. `--speed` is
  now an upper bound. **`SPEED_100_RAD_PER_SEC = 2.0` is an unverified
  calibration — measure it and correct it.**
- `mycobot_bridge.py` **lost-command latch fix**: `command_dirty` was cleared
  *before* the serial write, so a failed or skipped write silently dropped
  that command permanently — `write_command()` compares against
  `state.command`, which had already been updated to that value, so no later
  identical command could re-dirty it. Terminal at the end of a trajectory,
  where JTC holds a byte-identical target forever. Now cleared only after a
  successful write, and only if no newer command arrived meanwhile.
- `mycobot_bridge.py`: `set_gripper_value()` no longer re-sent on every *arm*
  command change (one wasted round trip per control cycle, competing with the
  arm on the same UART); `get_gripper_value()` now read once per 10 iterations
  instead of every one, roughly halving the loop period — the loop period is
  the hard ceiling on commands/sec reaching the arm.
- `mycobot_bridge.py`: `TIMING` logging — per-second `loop_rate` and per-write
  `send_angles` duration / speed / target. **This is the number that has been
  missing all along.** `--no-log-timing` to suppress.
- `pick_place.py` `_send_goal_and_wait()`: now **awaits the goal-acceptance
  handle** instead of firing `send_goal_async()` and discarding the future.
  Without it, one error message covered three unrelated faults — goal never
  arrived / arrived but hardware didn't move / moved but stopped short — which
  is why two sessions were spent guessing. On timeout it now reports per-joint
  error and the max movement of any joint during the wait, so "never moved"
  and "moved but short" are distinguishable at a glance.
- `pick_place.py` `_describe_trajectory()`: prints waypoints / duration /
  implied speed / velocities / `header.stamp` before every send.
- `pick_place.py`: `_FALLBACK_MAX_JOINT_SPEED` 0.2 -> 0.5 rad/s, matching the
  confirmed-working hand-built rate; shortens pre-grasp from 9.14 s to ~3.7 s.
- `joint_trajectory_test.py`: `--joints` / `--degrees` for an arbitrary
  target, which is what made the decisive test possible.

### What to do next

1. Rebuild on the robot (`mycobot_hardware` only — `mycobot_bridge.py` is
   installed from it) and rerun `pick_place.py`. Watch the bridge's
   `TIMING loop_rate=` line: that number vs. the trajectory's duration and
   waypoint count from `_describe_trajectory` tells you immediately whether
   streaming is now viable.
2. If the arm still stutters, the real fix is to stop streaming: add a
   `constraints:` block to `ros2_controllers.yaml` so the controller can
   actually fail, and/or have `plan_motion()` collapse the planned path to a
   small number of waypoints so JTC's hold does the work. Do **not** go back
   to DDS tuning — the transport has now been positively confirmed working
   for a 104° goal end-to-end.
3. Calibrate `SPEED_100_RAD_PER_SEC` (command a known delta at speed 100, time
   it against `/joint_states`).
4. Fix the OMPL response-adapter gap properly with a per-distro
   `response_adapters` config — the same split already used for
   `cyclonedds_galactic.xml` / `cyclonedds_jazzy.xml`. That restores real
   `AddTimeOptimalParameterization` output including the velocities JTC needs
   for cubic instead of linear interpolation.

### Note on repo state

The arm is currently parked at the pre-grasp pose (~104°, -42°, -48°, -3°,
0°, 104°) from the decisive test, not at home. `move_group`/`rviz` from the
10:03 launch and the robot-side launch were both still running at the end of
this session.

---

## Session 3b (2026-07-26 midday): the goal never left mars

Ran `pick_place.py` after the Session 3 fixes. New failure signature, caught
by the new instrumentation:

```
[arm] trajectory: 21 waypoints, 3.631s total, ~0.500 rad/s peak joint speed, velocities=NO
[ERROR] Timed out after 10.0s waiting for arm goal acceptance
[ERROR] arm goal: NO ACCEPTANCE RESPONSE within 10s -- the goal never reached the controller
```

Robot side logged **no `Received new action goal` at all**.

### Two Session 2/3 conclusions now positively disproven

**1. `serdata.cpp:354` is benign.** In a successful `--to zero` run the bursts
fire *before* the goal, the goal is then `Received -> Accepted -> Goal
reached, success!` normally, and they fire again *after* completion. They
bracket ROS process start/exit — cross-distro discovery metadata parsing,
exactly as Session 1 classified them. Session 2's "simultaneous with the
3-goal wall" was coincidence. **Stop treating these as a signal.**

**2. The bridge is not the bottleneck, and the loop is ~89Hz, not ~1Hz.**
Session 3 assumed 500-1500ms serial round trips throughout. Measured reality:
**89Hz idle** (10-22ms reads), collapsing to **1.7-3.4Hz only while the arm is
physically moving** (`last read 569ms` — the firmware starves the UART during
motion). The speed-matching fix works and tracked a full move correctly:

```
send_angles speed=10 target_deg=[103.88, ...]   <- start
send_angles speed=16 target_deg=[93.41, ...]
send_angles speed=37 target_deg=[69.31, ...]
send_angles speed=42 target_deg=[15.09, ...]
send_angles speed=10 target_deg=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   <- landed exactly
```

Only ~11 setpoints got through in 3s — coarse, but sufficient.

### Root cause of the delivery failure

`wait_for_server()` consults only the **local ROS graph cache**. In the failed
run it returned true ~250ms after process start (`1785077664.658` -> `~.9`),
long before the DDS request writer had **matched** the robot's request reader
across the unicast WAN link. ROS2 action/service requests are **RELIABLE +
VOLATILE**, and a volatile writer **silently discards** samples written while
no reader is matched — no error, no retry, no log line anywhere.

`joint_trajectory_test.py` survives on luck: it has no `/plan_kinematic_path`
round trip reshaping when its send lands relative to discovery.

**"Fix C" made this worse.** Destroying and recreating the ActionClient before
every goal forces teardown + rediscovery of 5 DDS entities per send, on the
link where discovery is the known weak point. Before Fix C, goals 1-3 always
worked; after it, goal **#1** fails. And Session 2 *deleted* the retry logic on
the reasoning that "goal delivery itself has never been the problem" — which
came from the same misattributed timestamps that made Fix C look confirmed.

### Fixes (commit `3b4320a`, mars-side only — no robot rebuild needed)

- Spin for `_DDS_MATCH_SETTLE_SEC` (1.5s) after `wait_for_server()` before
  writing the first request, so DDS matching can complete.
- `_deliver_goal()`: retry delivery up to 3 times. A genuine controller
  *rejection* is not retried, only a missing response.
- ActionClient is long-lived again; recreate only as the **recovery step**
  after a failed attempt — keeps the "3-goal wall" escape hatch without paying
  rediscovery on every send.

### Answering the question this kept raising

`pick_place.py` and `joint_trajectory_test.py` send the **same message type on
the same action** (`FollowJointTrajectory` -> `arm_group_controller`). The IK
was never the problem — the pre-grasp pose was already proven reachable in
Session 3. The difference is only trajectory *shape* (21 waypoints vs 1) and,
critically, **when the send lands relative to DDS discovery**.
