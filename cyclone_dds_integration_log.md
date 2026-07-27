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

---

## Session 3c (2026-07-26): ROOT CAUSE — fragmented UDP never crosses this link

The instrumentation finally produced an unambiguous discriminator. One
`pick_place.py` run, cross-referenced against the robot-side log:

| goal | waypoints | serialized | robot logged `Received`? |
|---|---|---|---|
| `go_home` | 2 | 358 B | **yes** |
| gripper close | 1 | 158 B | **yes** |
| gripper open | 1 | 158 B | **yes** |
| **pre-grasp** | **21** | **1802 B** | **NO — 3/3 attempts** |
| `go_home` (final) | 2 | 358 B | **yes** |

The final 358 B goal was delivered **seconds after** the three 1802 B
failures, from the same process, so the link was healthy the whole time and
nothing was unmatched, poisoned or degraded.

**The only variable that predicts delivery is whether the serialized message
fits in a single UDP datagram.** 1802 B exceeds the 1500 B Ethernet MTU
(1472 B of UDP payload), so it requires IP-level fragmentation — which this
campus Wi-Fi silently drops, for the same reason it blocks multicast. The
cutoff is 16 waypoints for a 6-joint arm goal.

### Fix (commit `a1e888a`)

```xml
<MaxMessageSize>1400B</MaxMessageSize>
<FragmentSize>1300B</FragmentSize>
```
Under `<General>` on Jazzy, `<Internal>` on Galactic 0.22.6 (verified by
loading each; Jazzy logs "setting moved to //CycloneDDS/Domain/General/..."
if they are left under Internal).

Cyclone already fragments at `FragmentSize` (default 1344 B), but
`MaxMessageSize` (default 14720 B) then lets it pack several fragments back
into ONE oversized datagram — which is what reintroduces IP fragmentation.
Capping `MaxMessageSize` below the MTU forces one sub-MTU datagram per RTPS
message.

### Everything this subsumes

- **The "3-goal wall" was never a count.** Goals 1-3 were always `go_home` +
  two gripper moves (all small); goal #4 was always the first *large* one.
  Every "after ~3 goals" observation across Sessions 2 and 3 is this.
  The `WhcHigh=500kB` watermark aimed at that phantom is removed.
- **`ros2 control list_controllers` timing out cross-machine** while working
  locally (Session 1, Problem 6) — its response carries every controller's
  name, type and interface list, comfortably over the MTU. Same bug.
- **Session 3b's "DDS matching race"** was wrong. Retrying with fresh
  ActionClients failed 3/3 because size, not timing, was the variable. The
  settle + retry logic is harmless and worth keeping as robustness, but it
  was not the fix.
- **`serdata.cpp:354`** remains benign background noise (Session 3b).

### The near miss

Commit `204f828` ("Raise Cyclone DDS FragmentSize/MaxMessageSize to avoid
fragmentation") identified the correct parameter on 2026-07-25 and was
reverted in `798955c`. It set both to **65500B** — the right knob turned the
**wrong way**: raising `MaxMessageSize` makes Cyclone build *bigger*
datagrams and fragment *harder* at the IP layer. It was then reverted partly
because of a misdiagnosed side effect (`/plan_kinematic_path service not
available`, which was just move_group not running in that terminal). A note
in `cyclonedds_galactic.xml` records this so the direction is not retried.

### Still open after this

- `_ensure_monotonic_timing` is still the only time parameterization (see
  Session 3) — the OMPL response-adapter gap is unfixed.
- `SPEED_100_RAD_PER_SEC = 2.0` in `mycobot_bridge.py` is still an unverified
  calibration.
- The bridge's serial loop runs ~85 Hz idle but collapses to 1-3 Hz *while
  the arm moves* (`get_angles()` taking 500-1500 ms mid-motion), so only
  ~10-20 setpoints land per trajectory. It tracked a move correctly at that
  rate, but it is coarse and worth revisiting if motion looks steppy.

---

## Session 3d (2026-07-26): first successful grasp; gripper contact detection rewritten

With the MTU fix (`a1e888a`) and least-travel IK (`e2e629e`) in place, the run
reached **home -> pre-grasp -> Cartesian descent -> grasp**, physically picking
up the block. The Cartesian descent (14 waypoints, velocities present, from
`/compute_cartesian_path`) executed correctly -- first confirmed Cartesian
motion on real hardware.

It then aborted on "Close gripper (grasp, stop on contact)". Two bugs:

### 1. Float boundary in the convergence check

```
per-joint error: {'gripper_controller': 0.05}   tolerance 0.05  -> FAILED
```
`-0.48 - (-0.53)` is `0.050000000000000044` in IEEE754, so `<= 0.05` is false
by one ULP. Not an edge case here: the gripper readback is quantized to
0.0075 rad (pymycobot's 0-100 scale over the 0.75 rad jaw span), so landing
exactly on the tolerance is routine. Fixed with a 1e-9 slack.

### 2. Contact detection could never fire, and cost 60s per step

`GRIPPER_EFFORT_THRESHOLD` has never been able to work on real hardware --
pymycobot exposes no gripper force reading, so `/joint_states` carries a
constant `0.0` placeholder and the threshold test never fires. The whole close
logged `effort=0.000` for all ~40 increments and ran to the hard stop. This was
already flagged in WORKFLOW.md "Known gaps"; this session confirmed it live.

Worse, each increment used the 60s default timeout, so once the jaw stalled
every remaining step waited the full budget.

**Fix -- stall-based contact detection.** A free jaw tracks the commanded value
down; a jaw against a block stops advancing while the command keeps
decreasing. `GRIPPER_STALL_STEPS=3` consecutive steps with less than
`GRIPPER_STALL_EPS=0.003` rad of progress declares contact. Both constants are
set against the 0.0075 rad readback quantum: EPS below one quantum so real
motion registers, STEPS above one so a single 0.005 rad fine step cannot alone
look like contact. `GRIPPER_STEP_TIMEOUT=3.0` replaces the 60s default per
increment, and `gripper_move_to(require_convergence=False)` makes a jaw that
cannot reach its commanded value a success rather than an abort -- which is
the correct semantics when closing onto an object.

Replaying this session's recorded 41-step trace through the new detector
declares contact at step 26, jaw holding `-0.4200` while commanded `-0.455`
-- the actual contact point.

Effort is still honoured if a reading ever becomes available; it is strictly
a better signal than stall.

### Status

Confirmed working end to end on real hardware: goal delivery (any size),
multi-waypoint joint-space execution, Cartesian execution, IK branch
selection, grasp. Not yet exercised: retreat, pre-place, place descent,
release, and the final return -- the run has never gotten past the grasp.

---

## Session 3e (2026-07-26): FULL PICK AND PLACE COMPLETED, then speed work

The complete sequence ran end to end on real hardware for the first time:
home -> pre-grasp -> Cartesian descend -> grasp -> Cartesian retreat ->
pre-place -> Cartesian descend -> release -> Cartesian retreat -> home.
Stall-based contact detection fired correctly and the block was picked and
placed.

Elapsed: **over 3 minutes**, against ~15-20s for the same sequence in
simulation. Most of it was the grasp.

### Where the time actually went

- **`_DDS_MATCH_SETTLE_SEC` on every goal.** Introduced in Session 3b on the
  (wrong) theory that a DDS matching race was dropping goals; the real cause
  turned out to be the MTU. At 1.5s x ~30 goals per run that was ~45s of pure
  sleeping, most of it inside the gripper loop. Now paid **once per action
  client** instead of once per goal, re-armed if a retry recreates the client.
  `_deliver_goal`'s retry covers the residual race.
- **Over-squeezing.** Contact was declared only after
  `GRIPPER_STALL_STEPS=3` consecutive stalled increments, so the jaw was
  commanded 0.09 rad further closed after it had already stopped. The **lag**
  column added last session turned out to be a much sharper signal:

  ```
  free:    +0.0075 +0.030 +0.030 +0.030 +0.0375 ... +0.045 +0.045
  blocked: +0.0675 +0.0975 +0.1275 +0.1575   <- stall count only fired here
  ```

  Free-running lag never exceeded +0.045; the first blocked reading was
  +0.0675. `GRIPPER_CONTACT_LAG = 0.06` sits cleanly between them. Replaying
  the recorded 18-step trace, lag fires at step 15 instead of 18: three fewer
  increments and 0.09 rad less squeeze.
- **Fine stepping.** `GRIPPER_FINE_ENABLED = False`. It was tuned in
  simulation for landing precisely on a 1 inch cube, but a 0.005 rad fine step
  is *smaller than the 0.0075 rad readback quantum*, so it cannot even be
  measured on this hardware -- it only added ~20 increments (~50s) per grasp.
- **Duplicate Cartesian solve.** `cartesian_move_to` computed the path twice
  (with and without the orientation constraint) as a leftover diagnostic. Every
  Cartesian move in the successful run returned fraction=1.00 both ways, so the
  extra solve proved nothing. Removed.
- **Log noise.** Each stalled gripper increment logged
  `[ERROR] the arm NEVER MOVED AT ALL`, which is expected and correct
  behaviour while closing on a block. `_send_goal_and_wait(log_failure=False)`
  now suppresses it when the caller expects non-convergence.

Estimated saving: **~115s per run.**

### Still open on speed (not done)

- `_FALLBACK_MAX_JOINT_SPEED = 0.5 rad/s` while `joint_limits.yaml` allows
  1.0. Every arm move is paced by this. Raising it is the largest remaining
  win but it directly changes real arm speed, so it wants a careful hardware
  test rather than a blind bump.
- The proper fix is still the OMPL response-adapter gap (Session 3): with
  `AddTimeOptimalParameterization` actually running, trajectories would get
  real accel/decel profiles instead of `_ensure_monotonic_timing`'s uniform
  constant-velocity pacing, and would both move faster and stop more cleanly.
- The bridge's serial loop drops from ~85Hz to 1-3Hz *while the arm moves*
  (`get_angles()` taking 500-1500ms mid-motion), so only ~10-20 setpoints land
  per trajectory. Coarse but functional; revisit if motion looks steppy.
- Nothing tells MoveIt the block is in the gripper, so collision checking for
  the place moves does not account for it.

---

## Session 3f (2026-07-26): intermittent "stops 5 degrees short" -- a regression I introduced

Repeat runs of the now-working pick and place failed intermittently, always
the same way: the arm travels almost the whole distance and settles a few
degrees short of target, failing pick_place.py's 0.05 rad convergence check.

```
target: {1.8278, -0.7181, -0.8115, -0.0404, 0.0010, 1.8280}
final:  {1.7380, -0.6718, -0.8697, -0.0244, -0.0045, 1.7380}
error:  {-0.0898, +0.0463, -0.0582, +0.0160, -0.0101, -0.0900}
max movement during the wait: 1.7228 rad
```

1.72 rad travelled, 0.09 rad (5.2 deg) short. Not a stall. One run in three
completed the full sequence, so it is marginal rather than deterministic.

### Cause: `_match_speed` bottoms out exactly when it matters

`_match_speed` (Session 3, added to stop the dart-and-stall stutter) scales
the pymycobot speed to the observed setpoint rate. At the END of a trajectory
the setpoints barely move, so it computes near zero and clamps to `MIN_SPEED`.
Visible directly in a logged send_angles trace:

```
speed=10 -> 16 -> 37 -> 43 -> 42 -> 32 -> 21 -> 44 -> 17 -> speed=10  <- final
```

The final command -- the one that has to actually seat the arm on target --
went out at the floor value, which is below what this arm needs to break
static friction.

### Compounding cause: nothing ever re-commands

`joint_trajectory_controller` holds its final target forever once a trajectory
elapses. A held target never changes, so `write_command()` stops marking it
dirty and **the bridge falls silent**. Whatever that last low-speed nudge
achieved is where the arm stays. There is no closed-loop correction anywhere
in the pipeline -- JTC streams open loop, the bridge forwards open loop, and
the only feedback is pick_place.py noticing the failure 60s later.

### Fix

- `MIN_SPEED` 10 -> 25.
- **`_serial_settle_if_needed()`**: while the held command is un-dirty and the
  measured arm position is more than `SETTLE_TOLERANCE_RAD` (0.02) away,
  re-send it at FULL speed, rate limited to every 0.5s, capped at 20 attempts
  (~10s) so a physically blocked joint is not driven indefinitely. 0.02 is
  deliberately tighter than pick_place.py's 0.05 check so settling actually
  clears that threshold.
- **Arm joints only.** The gripper is excluded on purpose: when holding a
  block it cannot reach its commanded value, and that is the success
  condition -- settling it would drive the jaw harder into the object forever.

This is the first closed-loop position correction in the pipeline.

### Note for future debugging

Both `_match_speed` and this settle are compensating for the same underlying
mismatch: a servo-style controller driving a point-to-point vendor API (see
Session 3). The principled fix remains real time parameterization
(the OMPL response-adapter gap) plus a `constraints:` block in
ros2_controllers.yaml so the controller can report tracking failure itself
instead of pick_place.py inferring it from /joint_states 60s later.

## Session 4 (2026-07-27): the jerky-motion campaign, then IK

Starting point: the full pick and place worked but the motion was visibly
jerky, the gripper was slow and stepped, IK converged about half the time,
and runs frequently printed `nan` for every joint and hung. By the end of the
session a full sequence ran clean with no step failures.

Commits, in order: `2f65c35` `0d45cab` `32086f2` `530ca00` `8b567bb`
`45215e8` `1e859c5` `2b1f135` `ce76bb8` `0cf0020` `6d4c0b6` `844efcb`
`7a69b2f`.

### 4a. The jerk was command starvation, not waypoint count

The user's hypothesis -- "the code taking those waypoints and going to them is
too slow" -- was correct. Evidence from a single return-to-home:

```
joint1 targets reaching the arm:
-74.53  -74.53  -74.83  -74.88  -75.19  -74.04  -58.57  -27.64  -27.06  -11.59  0.04
                                                     |16deg| |31deg|      |15deg|
```

JTC generated ~250 setpoints for that 2.6s goal. **Eleven arrived.** One asked
the base to jump 31 degrees in a single point-to-point move, which the next
command aborted partway through. One visible jerk per command.

`loop_rate` was 0.9-1.9Hz while moving and 82-89Hz idle.

### 4b. Root cause: pymycobot's hardcoded 0.5s read timeout

`send_angles` defaults to `has_reply=True`. In pymycobot 4.0.6:

```python
# common.py read()
if platform.system() == "Windows": wait_time = 0.15
else:                              wait_time = 0.5      # <-- Linux
while True and time.time() - t < wait_time: ...

# mycobot280.py _res() -- retries the whole exchange 3 times, then returns -1
```

Every number in the logs falls out of that: writes bimodal at 2-5ms (firmware
replied) or 510-590ms (one timeout plus a retry that worked), one read at
1506ms (three timeouts), and `get_angles() returned -1` in the same window.
None of it was flaky hardware. It is also self-inflicted -- the firmware only
defers its reply because it is busy with the move just sent.

Fixes, each measured before the next:

1. **`_async=True` writes** (`32086f2`). `_mesg`'s `_async` branch is a bare
   `_write()`: no read, no timeout, no retry, ~0.2ms. Writes went 552ms -> 0-2ms
   immediately. It did NOT fix the command rate.
2. **The bottleneck moved to `get_angles`** (`8b567bb`). With the write free,
   the read was the whole loop period; a 4.4s trajectory still reached the arm
   as 11 commands, one of them a 58-degree jump. Reads were already 535-1506ms
   during motion in the PRE-async logs, so async writes did not cause this.
   Throttled reads to one per 1.5s while the setpoint is moving -- nothing
   closes a loop on measured position mid-trajectory.
3. **A 30Hz command-rate cap**, since the loop would otherwise fire at ~85Hz
   and each command still aborts the move in progress.
4. **Capped the read timeout at 0.1s** (`2b1f135`). `read()` honours a
   `timeout` argument that overrides `wait_time`, but no path from
   `get_angles()` passes one, so `_install_read_timeout` wraps the bound
   `_read` to inject it -- checking the signature first so a future pymycobot
   without the parameter degrades to a warning.

The `dt=` column added in `45215e8` is what made step 4 diagnosable. Between
blackouts the stream was already perfect; the jerks were the blackouts:

```
dt=35ms   target_deg=[ 4.36, -2.63, ..., -37.89]
dt=551ms  target_deg=[15.32, -7.01, ..., -22.31]   <- 11 and 15.6 degrees in one command
loop_rate=171.5Hz (last read 541ms)
```

A 5s move at a 0.5s read interval gives ~5 blackouts, matching the reported
"4 or 5 jerks per move" exactly. Result: **"the arm motion is genuinely very
smooth now, and jerks very little, i think i noticed only one."**

### 4c. A wrong turn worth recording

Between steps 3 and 4 the jerks were attributed to velocity discontinuities at
OMPL path corners, and `_ensure_monotonic_timing` was rewritten as a proper
trapezoidal, corner-aware profile with velocities (`1e859c5`). The reasoning
was sound from the mars log alone -- joint-space plans reported
`velocities=NO` and a flat 0.500 rad/s while Cartesian plans reported
`velocities=yes` and looked smooth on the same hardware -- but the `dt` column
then showed the blackout dominated. **The `dt` data had been requested a round
earlier and the change was made without it.** Wait for the discriminating
measurement.

The trapezoidal work is kept and is not wasted: `velocities` present means JTC
interpolates with a cubic spline rather than linearly, the ramps are real, and
durations did not regress (18wp 4.82s vs 5.19s). Testing it against the
trajectory shapes from the log before running it on hardware caught two bugs,
one dangerous:

- The average-of-endpoint-speeds shortcut for segment time is only valid while
  speed changes monotonically. On a **two-waypoint** plan both ends are at
  rest, the average is zero, the minimum-segment floor took over, and it
  commanded **3.0 rad/s for a 0.3 rad move -- six times the limit**. "Final
  return to home pose" plans exactly two waypoints. Replaced with an exact
  accelerate/cruise/decelerate solve.
- Penalising corners by `cos(theta)` sends a right angle to a dead stop and
  made a wiggly path take 4x as long. Using `cos(theta/2)`.

### 4d. `nan` joint positions: never the robot

Runs printed `nan` for all six joints with `max movement so far 0.0000 rad`
and hung until killed. Two rounds were spent looking at the robot. The cause
was in pick_place.py:

```python
current = {n: round(self._joint_positions.get(n, float("nan")), 4) for n in target}
```

`float("nan")` is the **default for a joint absent from the dict**, and an
empty dict means no `/joint_states` message has ever arrived -- so all six hit
the default at once. Confirmed against the robot's own 3616-line log for the
same period: zero occurrences of `nan`, zero non-finite read warnings, banner
showing the expected build. The robot was publishing correctly throughout.

The message actively sent debugging to the wrong machine. It now reports the
truth plus `count_publishers`, distinguishes "no data at all" from "this joint
absent", and refuses to start without `/joint_states` (`0cf0020`).

That check then produced the decisive datum: **`1 publisher(s) visible`** with
zero messages, while `ros2 control list_controllers` showed all three
controllers active. The writer is discovered; the reader never matches. Same
failure shape `_deliver_goal` already recovers from for action goals, and the
same remedy -- `wait_for_joint_states` now destroys and recreates the
subscription every 6s (`844efcb`). Observed working first try:

```
[joint_states] no data after 6s (1 publisher(s) visible) -- recreating the
               subscription to force a fresh DDS match (attempt 1)
[joint_states] recovered after 1 subscription recreate(s)
```

Not yet understood: WHY the match fails. The abort path now dumps each
publisher's reliability/durability/depth so a QoS incompatibility (permanent,
retrying cannot help) can be told from a failed match (recoverable).

### 4e. IK: every seed had the base joint ~1.5 rad from the answer

IK converged for the pick pose about half the time and for the place pose
never. Not reachability -- seeding.

| | joint1 (base) |
|---|---|
| pick solution | **+1.828** |
| place solution | **-1.310** |
| every seed in `IK_SEEDS` | 0.324, 0.035, 0.0, -0.035, -0.324 |

All 13 seeds sit within +/-0.33 rad of zero. KDL is a **local** solver, so from
1.5 rad away it only landed when its internal random restarts happened to
wander across -- exactly the observed coin flip. The `-mirrored` seeds added in
an earlier session do not help: -0.324 is no closer to -1.310 than +0.324 is.

The base angle needs no numeric solve: to reach a point the base must face it.
`_bearing_seeds` derives it as `atan2(y, x)`, plus a 0.257 rad offset applied
both ways for the gripper's lateral fingertip offset (measured -- both real
solutions sit 0.257 and 0.261 rad from their bearings). `joint6output` is set
to match, since every converged solution counter-rotates the wrist by the base
angle to hold the fixed grasp yaw (`6d4c0b6`).

Best seed distance: pick 1.504 -> 0.000 rad, place 0.986 -> 0.004 rad. Seeds
are added, not replaced, and least-travel selection is unchanged, so nothing
that converged before can regress.

### 4f. The place hover was outside the workspace

With good seeds, the place pose still failed **all 19 seeds on every run** --
including bearing seeds within 0.004 rad of the answer. A solver handed a seed
on top of the answer that cannot converge is being asked for something that
does not exist.

`APPROACH_HEIGHT`'s own comment had already called it: reach_probe.py measured
`0.215 is already outside the workspace at ANY orientation`, and the comment
derives its value from a place flange target of **0.16**. That assumption
expired when the grasp/place z offsets were re-measured for the flat mat --
the place target is now **0.175**, putting the hover at exactly 0.215.

`hover_z()` clamps to `MAX_HOVER_Z = 0.205` (`7a69b2f`). Pick unchanged at
0.195; place 0.215 -> 0.205 with a 3cm descent.

This also explains two long-standing complaints. The OMPL fallback satisfied
its 4cm position sphere by parking the flange lower AND tilted, with
constraint sampling returning a different branch every run -- which is both
"the gripper points a little to the side" and "it never goes to the same place
twice". After the clamp, `Retreat after release` plans at `fraction=1.00`
where it had been stuck at 0.90/0.91.

### 4g. 0.05 rad was tighter than the servo deadband

A complete pick and place was aborted by `Retreat after release` missing
tolerance by **0.0004 rad (0.02 degrees)**, after the block was already placed.
The bridge's own log shows why:

```
settle re-send 1/20: still short by arm 0.0504 rad ... at speed 50
settle re-send 2/20: still short by arm 0.0504 rad
settle re-send 3/20: still short by arm 0.0504 rad
settle re-send 4/20: still short by arm 0.0504 rad
settle giving up: 0.0504 rad error stopped improving over 3 attempts
```

Four full-speed re-sends, the error never changing by a digit. Hard deadband.
`ARM_SETTLE_TOLERANCE = 0.07` clears the worst residual observed (0.027,
0.0315, 0.0330, 0.0334, 0.0341, 0.0504) with ~40% margin. Not raised further
on purpose: ~4 degrees is already up to ~1cm at the fingertips, and a
tolerance far past the deadband stops catching real tracking failures -- which
is the check that caught several genuine bugs this session.

### 4h. Settle rework, and a bug I introduced

`_serial_settle_if_needed` (Session 3f) was firing MID-trajectory. Its only
gate was "command is not dirty", which is equally true in the 500-580ms dead
time between two setpoints -- so it corrected an arm still in transit,
reported nonsense like `still 0.5129 rad short`, and re-commanded at **full
speed** while the trajectory paced at speed 25. Roughly 7 spurious full-speed
darts were injected into a single 2.6s homing move, making the jerk worse.

Fixed by gating on the commanded position being UNCHANGED for
`SETTLE_QUIET_PERIOD_SEC = 1.0` -- "time since the setpoint last moved" is the
real end-of-trajectory signal; "time since last write" is not, since JTC calls
write() every cycle regardless (`0d45cab`).

Also: `SETTLE_TOLERANCE_RAD` 0.02 -> 0.03 and give up after 3 no-progress
attempts. At 0.02 a run ended 0.0270 rad short and burned all 20 re-sends with
the error unchanged -- 10s and a burst of serial traffic at the end of every
move for nothing.

The gripper was later ADDED to settle (`ce76bb8`). It had been excluded because
a jaw holding a block cannot reach its command and settling would push
"forever" -- but the stall detector removes the "forever", which makes it
correct for holding a block and the fix for a jaw stuck part-open (observed:
commanded 0.15, stopped at -0.435, never corrected because JTC held its target
so the command stopped changing and the bridge stopped writing).

### 4i. Gripper: still the open problem

The gripper had neither of the arm's fixes. During a gripper goal JTC
interpolates its setpoint every cycle, so the bridge fired `set_gripper_value`
at the full 30Hz -- each a point-to-point move aborting the previous, at fixed
speed 50, synchronously (`gripper=524ms` in the logs, blocking the arm's stream
too).

Speed matching and settle were added, but the startup probe reports:

```
[mycobot_bridge] gripper writes=sync (blocking) -- this pymycobot's
                 set_gripper_value takes no _async argument
```

**pymycobot 4.0.6 has no `_async` for `set_gripper_value`.** So the gripper
cannot get the fix that fixed the arm, and it remains slow and stepped. The
probe reporting this rather than silently doing nothing is the useful part.

Compounding it, `gripper_close_until_contact()` still sends **17 separate
action goals** of 0.3s each -- a staircase by construction, independent of how
each one executes. And `GRIPPER_READ_EVERY = 10` means the gripper position is
stale, so the `lag=` value contact detection triggers on is partly read
staleness rather than real jaw lag. It works, but the signal is weaker than it
looks.

### End state

A full sequence, clean, no step failures. Pre-place IK: 12 of 19 seeds
converged and all agreed on `[-1.313, -0.801, -0.394, -0.376, 0, -1.313]` --
deterministic, no constraint sampling. Every Cartesian segment `fraction=1.00`.

### Next session

1. **Why does the `/joint_states` reader fail to match?** The recreate is a
   workaround. Capture the abort output's QoS dump on a run where recreating
   does not recover.
2. **Gripper.** Rewrite `gripper_close_until_contact` as one continuous motion
   cancelled on contact instead of 17 goals; lower `GRIPPER_READ_EVERY` so the
   contact signal is real lag. Check whether a newer pymycobot adds `_async`
   to `set_gripper_value`.
3. Still open from Session 3e: the OMPL response-adapter gap per distro, a
   `constraints:` block in ros2_controllers.yaml, calibrating
   `SPEED_100_RAD_PER_SEC`, and telling MoveIt the block is in the gripper so
   place-move collision checking accounts for it.
4. `_FALLBACK_MAX_JOINT_SPEED` is still 0.5 rad/s against joint_limits.yaml's
   1.0. Now that the command path is no longer starved, raising it is worth a
   careful hardware test.

## Session 5 (2026-07-27/28): four requested fixes, three self-inflicted regressions

Requested: restore feedback during motion, unstream the gripper, watchdog the
`/joint_states` reader, raise the speed limit. All four landed, but three of
them regressed first and the regressions are the useful part of this entry.

Commits: `8c62212` `9a68d7e` `f69182e` `4d8e187` `8247ea8`.

### 5a. The half-duplex tradeoff (the real finding)

`motion_read_interval` was cut 1.5s -> 0.05s on the grounds that
`DEFAULT_READ_TIMEOUT_SEC` had made reads cheap (10-25ms measured). **Sized
from the median and ignored the failure rate**, which was the wrong statistic.
Reads still fail during motion -- the firmware defers its reply while driving
-- and each failure blocks writes for 100ms (one timeout) or 300ms (three).
Going from 0.67 to 20 reads/sec multiplied the blackouts by 30x:

```
1150 command gaps: median 35ms, p90 182ms
  236 of 1150 (20%) over 100ms, 90 over 300ms
```

against a near-uniform 34-36ms before. The speed raise compounded it: a 180ms
gap advances the trajectory 8.2 degrees at 0.8 rad/s versus 5.2 at 0.5.

Settled at 0.5s, plus two supporting changes: read timeout 0.1 -> 0.06s
(bounding one read's worst blackout at 3 x timeout, 300ms -> 180ms), and
**write before read** in the serial loop, so a command that is already due no
longer waits behind a read.

**Keep this:** on this hardware a high feedback rate and smooth motion are
mutually exclusive. Reads and writes share one half-duplex link AND reads are
unreliable *exactly while the arm is moving*. Raising the rate buys blackouts,
not information. Anything needing fast feedback during motion must get it from
somewhere other than this serial link -- which is the strongest argument for
doing the correction loop on the Pi with the wrist camera.

### 5b. Gripper: a false analogy

The gripper was rate-limited to one command per 0.4s, reasoning that streaming
a point-to-point API is inherently bad. **The analogy with the arm was false.**
The arm's problem was too FEW commands (11 per trajectory, 31-degree jumps).
The gripper was already getting ~30 per goal, each moving the target ~3% of the
jaw's span, and the firmware roughly kept pace -- the aborts did not matter
because each new target was already close to where the jaw was.

Measured effect of the rate limit: exactly three commands per 1s goal, so the
jaw darted to 33% of span, ARRIVED AND STOPPED, waited, darted to 66%, stopped,
darted to 100%. Three discrete steps, worse than the thing it replaced.
Reverted in `f69182e`.

The quantised-value gate was kept and is independent: the gripper output step
is 0.0075 rad while the old threshold was 0.001, so comparing the integers
rejects commands that re-send a byte-identical value whose only effect is to
abort the move in progress. Verified it passes all 31 commands of a full-span
1s close, i.e. no change in the streaming case.

### 5c. Overgripping, then the modulo bug

Adding the gripper to `_serial_settle_if_needed` (Session 4) caused
overgripping exactly as flagged: after contact was detected at -0.2325 with
-0.300 still commanded, settle re-sent -0.300 at speed 50 four times, squeezing
harder each time. The stall detector was not enough -- three extra full-speed
squeezes on a gripped object is already too many.

Fixed by making settle **direction-aware**. Higher value is more open, so the
sign of the error separates the cases: commanded more open than measured means
the jaw failed to open (settle should fix it); commanded more closed means it
is against an object, which is the success condition for a grasp.

Then `GRIPPER_READ_EVERY` was cut 10 -> 1, because contact detection is
entirely `lag = commanded - measured` and a frozen `measured` makes lag grow
purely because the target is moving away from it -- a FALSE contact at whatever
value the reading is stuck on. Observed: contact declared at pos=0.0075 (about
80% OPEN, against -0.225 in runs that gripped), with the identical 0.0075
across four consecutive steps.

**That change broke the gripper completely**, because the gate was
`self._read_count % self.GRIPPER_READ_EVERY == 1` and `x % 1` is always 0 --
at N == 1 the condition never fires and the gripper is NEVER read. Asking for
"read it every time" did the exact opposite, pinning the gripper at the 0.0
placeholder from `SharedState.__init__` for the life of the process: `pos=0.0`
on every poll, `max movement 0.0000 rad`, error 0.6 on the first gripper goal.
Fixed in `8247ea8` with `(count - 1) % N == 0`, correct for every N >= 1.

### 5d. Speed: raising it alone does nothing

`_FALLBACK_MAX_JOINT_SPEED` 0.5 -> 0.8 **with** `_MAX_JOINT_ACCEL` 1.0 -> 2.0.
Reaching speed v under acceleration a costs `v^2/(2a)` of travel to ramp each
way, so at v=0.8, a=1.0 a segment needs 0.64 rad to reach speed while typical
segments are ~0.1 rad. Checked against real trajectory shapes: speed alone
peaks at 0.673 rad/s, both together at 0.756, and a 19-waypoint move goes
5.07s -> 3.37s. Left at 0.8 rather than joint_limits.yaml's 1.0 because faster
motion loads the joints harder, worsening the droop and dead zone that were
already failing goals.

### 5e. Two process lessons

**Stale logs cost two full analyses.** `src/swarm_pkg/src/logs/logs.txt` on
mars is only as fresh as the last manual copy from the robot; twice it was a
build from before the changes being debugged. Check the bridge's startup banner
(`read timeout capped at...`, `gripper writes=...`, `writes=async, max command
rate=...`) against the current commit BEFORE drawing any conclusion from a log.
The robot's clock is also wrong, so timestamps are not a reliable staleness
check -- banner content is.

**Reasoning by analogy between the arm and the gripper is unreliable.** They
share an API and a serial link but have opposite failure modes.

### End state

Full pick and place, clean, no step failures. Contact at -0.2325 with the jaw
reading tracking every step. Command gaps: median 35ms, 86% under 45ms, 39 in
the 101-300ms band (was 146). Reads: median 22ms, max 186ms, zero `-1`
returns. `/joint_states` matched first try with no recreate.
