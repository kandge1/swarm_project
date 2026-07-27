# Characterization tests

Purpose: before designing any outer-loop controller (PID, disturbance
observer, Kalman filter) on top of `mycobot_bridge.py`, find out what the
hardware and the serial link actually support. All of this feeds one
decision -- **is closed-loop correction on this arm worth building, and at
what rate** -- not incremental tuning of `pick_place.py`.

Everything below is measured with `serial_rate_probe.py`, which talks to the
arm directly over the same `pymycobot` API the bridge uses and takes MoveIt,
ROS, and the trajectory controller entirely out of the loop. Run it ON THE
ROBOT (the Pi), not on mars -- it opens the serial port directly.

Test 2 (dial-indicator ground truth for encoder counts) is skipped: no dial
indicator available. Everything else here relies on the arm's own encoder
reading itself, which is weaker evidence but still answers go/no-go.

## Status

| # | Test | What it answers | Status |
|---|------|------------------|--------|
| 1 | Dead-zone / correctability | Does biasing a command past the target fix a residual, or is it stuck? | **Ready -- run today** |
| 6 | Repeatability | Same commanded pose, N trials: how much does the settled reading scatter? | Not written -- needs a small wrapper loop around `settle()` |
| — | Read failure rate under motion | What fraction of reads fail/time out while the arm is moving vs. parked? | Not written -- `--probe` mode's mid-move numbers are a partial answer already (see below) |
| 4 | Servo register sweep | Is load current (or any register besides angle) available at all? | Blocked -- needs `get_servo_data`/`get_system_version`-style register access checked against the firmware first; unknown if it's exposed |
| 3 | Step response (τ, dead time) | Time constant and delay of the position loop, for phase-margin math | Partially covered by `--probe` and `--stream`; not yet reduced to a clean τ/T_d number |

Bottom line: **Test 1 is ready now.** The rest need either a short script
(6, read-failure-rate) or a firmware capability check (4) before they can
run. Recommend doing Test 1 today across postures/joints as planned, and
writing Test 6 next since it reuses `settle()` almost directly.

---

## Test 1 -- Dead-zone / correctability (`--deadzone`)

### The question

`mycobot_bridge.py`'s settle logic has re-sent the SAME target at full speed
many times and watched the error not move by a digit (residuals of 0.0315,
0.0331, 0.0348, 0.0387, 0.0504 rad, logged 2026-07-27/28). That proves
re-commanding the same value fails. It says nothing about commanding a
DIFFERENT value -- which is exactly what an outer-loop controller would do.

So: move a joint, measure the residual `e = target - settled`, then command
`target + k*e` for k = 1, 2, 3 and watch what the joint does.

### Why k=1 is the whole answer

Two candidate explanations for a residual predict different curves:

- **Compliance / gravity droop** (steady-state proportional to command):
  simulated `|err|/|e|` at k=1 was 0.10.
- **Dead band** (inert within a window of the target, lands short outside
  it -- reproduces the observed "re-sending the same value never moves it"
  behavior): simulated `|err|/|e|` at k=1 was 0.00.

Both are correctable by biasing the command, so both mean go. What is NOT
correctable is a joint that ignores the bias entirely (flat `|err|/|e|`
across all k), or one whose residual is unpredictable run to run (stiction,
backlash) -- that would mean dither, dead-zone inversion, or external
metrology instead of a plain outer-loop PID.

Rising `|err|/|e|` at k=2, k=3 is EXPECTED and is a GOOD sign: it means the
joint tracks a biased command proportionally and overshoots when
over-biased. A column that stays flat at every k is the bad outcome.

### Why several joints and postures

Gravity load varies hugely with arm posture. A result from one joint at one
pose is not a result -- run folded, extended, and mid-reach, on at least
joints 1, 2, and 3 (0-based: `--joint 0`, `--joint 1`, `--joint 2`).

### Commands

From the robot (Pi), arm powered and clear to move ~30 deg on the tested
joint without hitting anything:

```bash
cd ~/swarm_project
source install/setup.bash

# joint 1 (0-based index 0), default 30 deg amplitude, mid posture
python3 src/mycobot_hardware/scripts/serial_rate_probe.py --deadzone \
    --joint 0 --amplitude-deg 30 --out /tmp/deadzone_j0_mid.csv

# repeat varying posture: move the arm by hand or with joint_trajectory_test.py
# to folded / extended first, then rerun with the same --joint.
python3 src/mycobot_hardware/scripts/serial_rate_probe.py --deadzone \
    --joint 1 --amplitude-deg 30 --out /tmp/deadzone_j1_mid.csv

python3 src/mycobot_hardware/scripts/serial_rate_probe.py --deadzone \
    --joint 2 --amplitude-deg 30 --out /tmp/deadzone_j2_mid.csv
```

Useful flags:

- `--amplitude-deg` -- bigger moves load the joint harder and are easier to
  read a residual off of; if a run reports the residual is below
  `--deadzone-min-deg` (0.3 deg default) there's nothing to correct at that
  pose/amplitude and it's worth trying a larger move or a more loaded
  posture instead.
- `--deadzone-steps` -- how many k values the staircase tries (default 3).
- `--speed` -- command speed (default 30); shouldn't matter much here since
  the test cares about the SETTLED position, not the path.
- `--out` -- CSV of the commanded/settled trace for each k, for later
  plotting.

The script always returns the joint to its starting angle when done (even on
Ctrl-C), and prints a VERDICT line (CORRECTABLE / NOT CORRECTABLE / mixed)
at the end of each run. Read the k=1 row first; the verdict line explains
itself.

### What to record per run

Joint, posture (folded/mid/extended), amplitude, the printed residual `e`,
and the `|err|/|e|` value at each k. Six data points (3 joints x 2 postures
minimum) is enough to tell if the dead-zone behavior is consistent or
posture-dependent, which itself matters for the controller design.

---

## Test 6 -- Repeatability (not yet written)

Command the SAME pose N times (~20), from different approach directions if
possible, and measure the scatter in the settled reading. This is the
practical floor on "how good can closed-loop position control be," since it
bounds what feedback could ever converge to, independent of dead-zone or
gravity effects. Feeds directly into the sub-mm feasibility number already
worked out from the encoder resolution (1.533e-3 rad -> ~0.38mm/count at
250mm, ~0.62mm RSS floor) -- this test tells us whether the achieved
repeatability is close to that theoretical floor or dominated by something
else (backlash, thermal drift, servo tolerance).

Plan: a thin wrapper reusing `settle()` and `read_angles()` from
`serial_rate_probe.py`, looping a single `send_angles()` target N times with
a return-to-a-different-pose in between each trial (so it's not just
re-settling from rest), logging the settled reading each time. Will add as
`--repeatability` in the same script once Test 1 results are in, so
`settle()`'s tolerance/quiet-period constants can be sanity-checked against
Test 1 first.

## Read failure rate under motion (not yet written)

`--probe` mode already measures this indirectly: it times `get_angles()`
and `send_angles()` both idle and mid-move, and the finding so far is that
the bridge's effective loop rate collapses from ~89Hz idle to 1-3Hz while
the arm is moving (Session 4/5 findings -- half-duplex UART, and pymycobot's
hardcoded 0.5s Linux read timeout x 3 retries dominates whenever a read
actually times out). What's missing is the FAILURE rate specifically --
what fraction of reads mid-motion come back invalid/timed-out rather than
just slow -- since that's the number that determines the state estimator's
effective update rate, not just its worst-case latency. Plan: extend
`mode_probe` to count `None`/timeout returns from `read_angles()` separately
from successful-but-slow ones, over a longer mid-move window (30s+, several
different trajectory shapes) rather than the current 4s single ramp.

## Test 4 -- Servo register sweep (blocked)

Whether `get_servo_data()` (or equivalent) exposes anything beyond position
-- load current, temperature, voltage -- on this firmware. This determines
whether "load currents and whatever else is available in mycobot's arsenal"
(the explicit no-camera control approach) has a second sensing channel
beyond joint angle, or whether the controller is angle-only. Needs checking
against the actual pymycobot version's supported calls and the myCobot 280
Pi's firmware register map before a test script is worth writing --
possible the call exists but returns nothing meaningful on this hardware
revision.

## Test 3 -- Step response (partially covered)

`--stream` and `--point-to-point` already produce commanded-vs-measured
traces at controllable rates and are exactly the step/ramp inputs this test
needs; what's missing is reducing a captured trace to a clean time constant
and dead time (e.g. fit against a first-order-plus-dead-time model) rather
than reading the numbers off a CSV by eye. Worth doing after Test 1 and 6,
once there's a real per-joint dead-zone number to subtract out first --
otherwise the fitted tau will be contaminated by dead-zone behavior instead
of isolating the servo loop's actual dynamics.
