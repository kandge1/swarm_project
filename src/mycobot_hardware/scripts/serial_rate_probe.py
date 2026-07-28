#!/usr/bin/env python3
"""
serial_rate_probe.py -- measure what the myCobot's serial link and pymycobot
can ACTUALLY sustain, with ROS, DDS, MoveIt and joint_trajectory_controller
entirely out of the picture.

WHY THIS EXISTS: real-hardware motion on this arm is jerky, and the pipeline
has three candidate culprits that the normal logs cannot cleanly separate:

  1. the planned trajectory itself (pick_place.py's _apply_uniform_timing
     assigns constant velocity with no accel/decel ramps, because OMPL here
     has no time-parameterization response adapter),
  2. joint_trajectory_controller's 100Hz setpoint stream being decimated to
     whatever mycobot_bridge.py's serial loop can forward (1-3Hz measured
     while the arm is moving),
  3. pymycobot's send_angles() being a point-to-point move that ABORTS AND
     RESTARTS the move already in progress, so every forwarded setpoint is a
     fresh dart toward a stale target.

This script tests 2 and 3 in isolation. If a smooth ramp streamed directly
into send_angles() at a healthy rate still comes out jerky here, the problem
is the vendor API and no amount of controller tuning will fix it. If it comes
out smooth, the jerk lives upstream in the ROS pipeline and is ours to fix.

RUN THIS ON THE ROBOT, AND STOP THE BRIDGE FIRST. mycobot_bridge.py holds the
serial port exclusively; two processes talking to /dev/ttyAMA0 at once produce
garbage that looks like a hardware fault. Shut down real_robot_hardware.launch.py
(or at least the bridge) before running, and relaunch afterwards.

  # timing histogram of the two serial calls, idle and mid-move
  ./serial_rate_probe.py --probe

  # stream a smooth ramp as N setpoints/sec, log commanded vs measured
  ./serial_rate_probe.py --stream --rate 10 --speed 30 --out /tmp/stream10.csv

  # the same total motion as ONE point-to-point move, for comparison
  ./serial_rate_probe.py --point-to-point --speed 30 --out /tmp/p2p.csv

Compare the CSVs: a staircase in the measured column means each command is
being aborted and restarted (culprit 3); smooth measured motion that simply
lags means the rate is too low (culprit 2).

SAFETY: every mode moves exactly ONE joint (default joint 1, the base yaw) by
a small default amplitude, and returns it to its starting angle on exit --
including on Ctrl-C. Nothing here plans around obstacles, so give the arm
clear space. Pick the joint and amplitude with --joint / --amplitude-deg.
"""

from __future__ import print_function

import argparse
import inspect
import math
import statistics
import sys
import time

DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD_RATE = 1000000

# Same cap mycobot_bridge.py installs. pymycobot's common.py read() hardcodes
# wait_time = 0.5 on Linux and _res() retries three times, so one unanswered
# read can hold the serial link for 1.5s. Left uncapped, sampling during motion
# becomes irregular half-second gaps, which is useless for a step response and
# misleading for a settle measurement. 0.06 bounds a failed read at ~0.18s while
# a healthy one takes 10-25ms.
DEFAULT_READ_TIMEOUT_SEC = 0.06


def install_read_timeout(arm, timeout_sec):
    """Wrap pymycobot's _read so every read carries an explicit timeout.

    read() supports one (`if timeout is not None: wait_time = timeout`) but no
    call path from get_angles() supplies it, so the hardcoded Linux default
    always wins. Wrapping the bound method injects it without editing any
    vendor file; if a future pymycobot drops the parameter this detects that
    and leaves the default rather than raising."""
    if not timeout_sec or timeout_sec <= 0:
        print("[probe] read timeout: pymycobot default (0.5s x 3 attempts)")
        return
    original_read = arm._read
    try:
        parameters = inspect.signature(original_read).parameters
    except (TypeError, ValueError):
        parameters = {}
    if "timeout" not in parameters:
        print("[probe] WARNING: this pymycobot's _read takes no 'timeout' "
              "argument -- leaving the default. Reads may block up to 1.5s, "
              "which will distort these measurements.")
        return

    def read_with_timeout(genre, *args, **kwargs):
        kwargs.setdefault("timeout", timeout_sec)
        return original_read(genre, *args, **kwargs)

    arm._read = read_with_timeout
    print("[probe] read timeout capped at {}s per attempt".format(timeout_sec))


def connect(port, baud, read_timeout=DEFAULT_READ_TIMEOUT_SEC):
    from pymycobot import MyCobot280
    print("[probe] connecting to {} @ {}...".format(port, baud))
    arm = MyCobot280(port, baud)
    print("[probe] connected.")
    install_read_timeout(arm, read_timeout)
    return arm


def read_angles(arm):
    """get_angles() with the same shape validation mycobot_bridge.py uses --
    pymycobot returns a bare int (often -1) on a comms timeout rather than
    raising, which silently corrupts any measurement that trusts it."""
    angles = arm.get_angles()
    if not isinstance(angles, (list, tuple)) or len(angles) != 6:
        return None
    return list(angles)


def timed(fn, *args):
    t0 = time.time()
    result = fn(*args)
    return result, (time.time() - t0) * 1000.0


def summarize(label, samples):
    if not samples:
        print("[probe] {:<28} no samples".format(label))
        return
    ordered = sorted(samples)
    print("[probe] {:<28} n={:<4} min={:7.1f}ms  median={:7.1f}ms  "
          "p90={:7.1f}ms  max={:7.1f}ms".format(
              label, len(samples), ordered[0],
              statistics.median(ordered),
              ordered[int(0.9 * (len(ordered) - 1))],
              ordered[-1]))


# One encoder count is 1.533e-3 rad = 0.0879 deg on this arm (measured from
# the bridge's own logs). A stability threshold BELOW that is unsatisfiable:
# a joint sitting perfectly still but dithering by a single count shows a
# 0.0879 deg spread forever, so settle() can never converge and burns its
# whole timeout before returning a possibly mid-motion reading. 0.12 deg is
# ~1.4 counts -- above the quantisation floor, still well under the 0.3 deg
# --deadzone-min-deg residual the test needs to resolve.
ENCODER_COUNT_DEG = 0.0879

SETTLE_STABLE_DEG = 0.12

# How far a joint must move before a "stable" reading is believed to be the
# END of a move rather than the moment before it started. Also ~1.4 counts.
SETTLE_MIN_MOTION_DEG = 0.12


# A stability window must span at least this much WALL TIME, not just
# `stable_reads` readings. Four reads at ~20ms each is an 80ms window -- short
# enough to fall entirely inside an acceleration ramp or a momentary pause, so
# the arm looks stopped while it is still driving. The 2026-07-28 --repeats 3
# run finished 120 settling moves in 69s (0.58s each) when a 30 deg move at
# speed 25 physically takes 1-2s: settle was returning mid-move, and since
# send_angles ABORTS the move in progress, each early return cancelled the
# previous command. That is what produced both the 7 trials whose move never
# happened and the 5 that began from the wrong posture.
SETTLE_MIN_STABLE_SEC = 0.4


def settle(arm, args, timeout_sec=6.0, stable_reads=4,
           stable_deg=SETTLE_STABLE_DEG, reference=None,
           min_motion_deg=SETTLE_MIN_MOTION_DEG, motion_grace_sec=2.0,
           min_stable_sec=SETTLE_MIN_STABLE_SEC):
    """Poll until the arm stops moving, then return the settled angles.

    "Stopped" means `stable_reads` consecutive readings within `stable_deg` of
    each other, not a fixed sleep -- the point of this whole script is that
    move durations here are not predictable.

    PASS `reference` (the pose from before the command) WHENEVER ONE EXISTS.
    send_angles() returns before the servos begin driving, and a read takes
    ~20ms, so four consecutive reads span ~80ms -- comfortably inside the gap
    between the command landing and the arm actually starting. Those four
    reads are identical, the stability test passes, and settle() returns the
    STARTING pose as though the move had completed. With `reference` given,
    stability is ignored until the arm has actually moved `min_motion_deg`
    away from it.

    That failure is not hypothetical: the first --deadzone-sweep run on
    hardware (2026-07-28) produced residuals of exactly +30.0000 deg -- the
    commanded amplitude to four decimal places, i.e. settled == start -- on
    4 of 9 trials, and the dead-zone staircase then extrapolated target + k*e
    with e = 30 deg, walking joints 60 and 90 deg out until one hit its limit
    and pymycobot raised. Racy by nature, which is why the other 5 trials
    looked fine.

    A move that never starts must still terminate, so after `motion_grace_sec`
    a stable reading is accepted regardless. The caller is responsible for
    noticing that nothing moved -- see run_deadzone_trial's failed-move guard,
    which is the independent second defense against exactly this."""
    deadline = time.time() + timeout_sec
    grace_until = time.time() + motion_grace_sec
    last_good = None
    moved = reference is None
    window_ref = None
    window_start = None
    window_count = 0
    while time.time() < deadline:
        angles = read_angles(arm)
        if angles is None:
            continue
        last_good = angles
        if not moved and max(abs(angles[i] - reference[i])
                             for i in range(6)) >= min_motion_deg:
            moved = True
        now = time.time()
        if (window_ref is None
                or max(abs(angles[i] - window_ref[i]) for i in range(6))
                > stable_deg):
            # Moved: start a fresh stability window from here.
            window_ref = angles
            window_start = now
            window_count = 1
            continue
        window_count += 1
        if (window_count >= stable_reads
                and now - window_start >= min_stable_sec
                and (moved or now >= grace_until)):
            return last_good
    return last_good


# Postures for --deadzone-sweep, and the gravity moment arm each one puts on
# each joint. NOT hand-picked: computed from the URDF by searching every
# (J2, J3, J4) combination on a 15-degree grid for the ones that (a) keep every
# link distal to the elbow at least 60mm above the base plane, INCLUDING after
# the +/-30 degree test perturbation, and (b) span the widest possible range of
# gravity load. 391 postures passed the clearance test; these three are the
# extremes and the midpoint of that set.
#
# "arm" below is the moment arm about that joint's own axis for a vertical load
# at the tool: tau = axis . (r x -Z), in metres. It is the number the residual
# should be proportional to IF the cause is gravity droop.
#
# J1's moment arm is 0.0000 in EVERY posture -- its axis is vertical, so
# gravity cannot load it at all. That makes joint 0 the control: whatever
# residual it shows is dead zone, stiction or quantisation with the gravity
# term provably absent. J5 and J6 are likewise ~zero and are not worth
# sweeping, which is why the default joint set is 0, 1, 2.
#   name        angles (deg)                  gravity arm (m)      inertia lever (m)   clearance
#                                             J1      J2      J3     J1      J2      J3
POSTURES = [
    ("vertical", [0,   0,  30,   0, 0, -45], (0.0000, 0.0018, 0.0018), (0.0640, 0.3047, 0.1943), 0.249),
    ("tucked",   [0, -15,  30, -60, 0, -45], (0.0000, 0.1231, 0.0945), (0.1231, 0.2210, 0.1218), 0.245),
    ("folded",   [0, -90,  60,  90, 0, -45], (0.0000, 0.1428, 0.0324), (0.1428, 0.2478, 0.2051), 0.139),
    ("reach",    [0, -30,   0,  45, 0, -45], (0.0000, 0.1766, 0.1214), (0.1766, 0.3261, 0.2159), 0.234),
    ("compact",  [0, -60,  45, -60, 0, -45], (0.0000, 0.2159, 0.1203), (0.2159, 0.2284, 0.1218), 0.194),
    ("extended", [0,  45,  15,  75, 0, -45], (0.0000, 0.2805, 0.2025), (0.2805, 0.3144, 0.2123), 0.213),
]

# WHY SIX, AND WHY THESE SIX.
#
# The 2026-07-28 three-posture set could not answer its own question. Joint 0's
# gravity moment arm is 0.0000 in every posture -- gravity CANNOT load it --
# yet its residual varied systematically with posture, 0.847 / 0.767 / 0.550
# deg, tracking the tool's distance from the rotation axis monotonically while
# the within-posture repeat spread was only 0.00-0.08 deg. That is a real
# posture-dependent effect with no gravity in it, worth ~0.30 deg, about 14% of
# the gravity span being attributed to joints 1 and 2.
#
# It could not be subtracted out, because in those three postures the gravity
# moment arm (a HORIZONTAL offset) and the inertia lever (perpendicular
# distance to the axis) were nearly proportional -- correlation +0.6 to +1.0.
# Two collinear regressors cannot be separated by any amount of data.
#
# These six were chosen by searching all 1472 postures that keep every link
# distal to the elbow >=60mm above the base plane (including after the +/-30
# deg perturbation on joints 0, 1 and 2) for a subset that spans the gravity
# range AND decorrelates the two. An arm stretched vertically has a large
# inertia lever and near-zero gravity arm, which is what breaks it:
#
#     corr(gravity arm, inertia lever)   J1: +0.033    J2: +0.005
#
# At that correlation a two-variable fit can attribute residual to gravity and
# to inertia separately, which is the difference between "droop is 7.5 deg/m"
# and "droop is 7.5 deg/m give or take an unknown confound".


# pymycobot enforces these itself and RAISES on violation, which aborts the
# whole sweep mid-run. Checking first turns "the process died at trial 9 of 9"
# into "that one staircase step was skipped and reported". Values are the
# myCobot 280's own limits, confirmed against the exception pymycobot threw on
# 2026-07-28: "error on index 2. Received 150.58 but angle should be -150 ~ 150".
JOINT_LIMITS_DEG = [
    (-168.0, 168.0),   # J1 base yaw
    (-135.0, 135.0),   # J2 shoulder
    (-150.0, 150.0),   # J3 elbow
    (-145.0, 145.0),   # J4
    (-165.0, 165.0),   # J5
    (-175.0, 175.0),   # J6
]
JOINT_LIMIT_MARGIN_DEG = 2.0

# How close to the commanded posture counts as "arrived". Droop itself is
# ~1.3 deg at the most loaded posture and is the thing being measured, so this
# has to sit above that while still catching the ~29 deg misses seen when a
# posture move gets cancelled.
POSTURE_TOLERANCE_DEG = 3.0
POSTURE_ATTEMPTS = 3

# send_angles() silently does nothing often enough to matter -- 7 of 27 trial
# moves on 2026-07-28. Retrying turns a 26% per-trial loss into ~2%.
MOVE_ATTEMPTS = 3

# A residual this close to the commanded amplitude means the joint did not
# move at all -- settled == start -- so there is no "error" to correct and
# k*e is not a small bias but a second full-sized move. Extrapolating from it
# is what walked joints 60 and 90 deg out of position on the first hardware
# run. Half the amplitude is far above any plausible real residual (those run
# 0.2-1.5 deg against a 30 deg move, i.e. under 5%) and far below a no-op.
FAILED_MOVE_FRACTION = 0.5


def within_limits(angles):
    """(ok, index, value, lo, hi) for the first joint outside its safe range."""
    for i, value in enumerate(angles[:6]):
        lo, hi = JOINT_LIMITS_DEG[i]
        if not (lo + JOINT_LIMIT_MARGIN_DEG <= value <= hi - JOINT_LIMIT_MARGIN_DEG):
            return False, i, value, lo, hi
    return True, None, None, None, None


def move_to_posture(arm, args, angles, label):
    """Send the whole arm to a posture and wait for it to stop.

    Uses a longer settle budget than a single-joint step: this is a six-joint
    move and several of them are large."""
    print("[probe] moving to posture '{}': {}".format(label, angles))
    for attempt in range(1, POSTURE_ATTEMPTS + 1):
        before = read_angles(arm)
        arm.send_angles(list(angles), args.sweep_speed)
        settled = settle(arm, args, timeout_sec=args.posture_settle_sec,
                         reference=before)
        if settled is None:
            print("[probe]   attempt {}: no valid reading".format(attempt),
                  file=sys.stderr)
            continue
        worst = max(abs(settled[i] - angles[i]) for i in range(6))
        if worst <= POSTURE_TOLERANCE_DEG:
            return settled
        # VERIFY, don't assume. Silently accepting a posture the arm never
        # reached is worse than failing: the trial still runs, and gets
        # credited with a moment arm belonging to a configuration it was
        # never in. Five of 27 trials on 2026-07-28 did exactly that, four of
        # them ~29 deg out, and their residuals were averaged into the wrong
        # load level.
        off = max(range(6), key=lambda i: abs(settled[i] - angles[i]))
        print("[probe]   attempt {}: did not arrive -- joint {} is {:.2f} deg "
              "off ({:.2f} vs {:.2f}), retrying"
              .format(attempt, off, worst, settled[off], angles[off]))
    print("[probe] ERROR: could not reach posture '{}' in {} attempts"
          .format(label, POSTURE_ATTEMPTS), file=sys.stderr)
    return None


def deadzone_verdict(ratios, steps):
    """Classify one staircase. Returns (label, list_of_explanation_lines).

    Shared by --deadzone and --deadzone-sweep so a single run and a matrix
    cell are never judged by different rules."""
    if not ratios:
        return "NO DATA", ["no staircase steps completed"]
    if ratios[0] < 0.5:
        lines = ["Biasing the command by e cut the error to {:.0f}% of its "
                 "original size.".format(100 * ratios[0]),
                 "The joint responds to a biased command, so an outer-loop "
                 "integrator or disturbance observer CAN close this out."]
        if len(ratios) > 1 and ratios[-1] > ratios[0]:
            lines.append("(The rise at higher k is expected and confirms it -- "
                         "the joint tracks the bias proportionally, so it "
                         "overshoots when over-biased.)")
        return "CORRECTABLE", lines
    if all(r > 0.8 for r in ratios):
        return "NOT CORRECTABLE", [
            "The error did not move at any k up to {}x. That is stiction or a "
            "dead band wider than the residual itself -- an integrator would "
            "wind up and limit-cycle.".format(steps),
            "Needs dither, dead-zone inversion, or external metrology."]
    return "MIXED", [
        "The error only responds past some k. The dead band lies between the "
        "last k that did nothing and the first that moved."]


def run_backlash_trial(arm, args, joint_idx, quiet=False):
    """Measure lost motion directly: command ONE target, reached from each side.

    This is the classic hysteresis measurement and it exists because the
    dead-zone staircase only stumbled onto backlash by accident -- it showed up
    solely in the trials whose residual happened to come out negative, 4 of 27,
    and every one of those stalled completely (|err|/|e| exactly 1.00 at k=1).
    That is far too important to leave to chance: at 1-2 deg it is ~6.5mm at a
    250mm reach, which dwarfs droop, the dead zone, and quantisation combined,
    and unlike those it CANNOT be fixed by feedback -- the joint does not move,
    so an integrator winds up against nothing.

    Approach the same target from below and from above; the gap between where
    the joint stops is the lost motion. Both approaches end with a move of the
    same size, so servo dynamics cancel and what remains is the direction
    dependence."""
    start = read_angles(arm)
    if start is None:
        return None
    j = joint_idx
    swing = abs(args.amplitude_deg)
    target = start[j]

    settled = {}
    for label, sign in (("from_below", -1.0), ("from_above", +1.0)):
        away = list(start)
        away[j] = target + sign * swing
        approach = list(start)
        approach[j] = target
        for pose in (away, approach):
            ok, bad_i, bad_v, lo, hi = within_limits(pose)
            if not ok:
                if not quiet:
                    print("[probe] backlash SKIP: joint {} would reach {:.2f} deg, "
                          "outside {:.0f}..{:.0f}".format(bad_i, bad_v, lo, hi))
                return None
        before = read_angles(arm)
        arm.send_angles(away, args.speed)
        settle(arm, args, timeout_sec=args.settle_timeout_sec, reference=before)
        before = read_angles(arm)
        arm.send_angles(approach, args.speed)
        result = settle(arm, args, timeout_sec=args.settle_timeout_sec,
                        reference=before)
        if result is None:
            return None
        settled[label] = result[j]

    lost = settled["from_below"] - settled["from_above"]
    if not quiet:
        print("[probe] backlash: target {:.3f} -> {:.3f} from below, {:.3f} from "
              "above, lost motion {:+.3f} deg ({:.1f} counts)"
              .format(target, settled["from_below"], settled["from_above"],
                      lost, abs(lost) / ENCODER_COUNT_DEG))
    return {"joint": j, "target": target, "from_below": settled["from_below"],
            "from_above": settled["from_above"], "lost_motion": lost}


def run_deadzone_trial(arm, args, joint_idx, quiet=False, amplitude=None):
    """One dead-zone measurement on one joint, from wherever the arm is now.

    Moves the joint by --amplitude-deg, measures the residual e, then commands
    target + k*e for k = 1..--deadzone-steps. Returns a result dict, or None if
    the arm could not be read. Does NOT restore the pose -- the caller decides,
    because the sweep needs to return to a posture rather than to the previous
    joint angle."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return None

    j = joint_idx
    if amplitude is None:
        amplitude = args.amplitude_deg
    target = list(start)
    target[j] = start[j] + amplitude

    ok, bad_i, bad_v, lo, hi = within_limits(target)
    if not ok:
        print("[probe] SKIP: the initial move would put joint {} at {:.2f} deg, "
              "outside its {:.0f}..{:.0f} limit".format(bad_i, bad_v, lo, hi))
        return None

    # Retry a move that simply did not happen. This is NOT retrying a bad
    # measurement -- a move that never executed produces no measurement at
    # all, and discarding it outright threw away 26% of trials.
    settled = None
    for attempt in range(1, MOVE_ATTEMPTS + 1):
        arm.send_angles(target, args.speed)
        settled = settle(arm, args, timeout_sec=args.settle_timeout_sec,
                         reference=start)
        if settled is None:
            print("[probe] ERROR: no valid reading after the initial move",
                  file=sys.stderr)
            return None
        if abs(target[j] - settled[j]) < FAILED_MOVE_FRACTION * abs(amplitude):
            break
        if attempt < MOVE_ATTEMPTS:
            print("[probe]   move did not execute (joint {} moved {:.2f} deg), "
                  "retrying {}/{}".format(j, abs(settled[j] - start[j]),
                                          attempt + 1, MOVE_ATTEMPTS))
            start = read_angles(arm) or start
            target = list(start)
            target[j] = start[j] + amplitude

    residual = target[j] - settled[j]
    if not quiet:
        print("[probe] commanded {:.3f} deg, settled at {:.3f} deg"
              .format(target[j], settled[j]))
        print("[probe] residual e = {:+.4f} deg ({:+.5f} rad)"
              .format(residual, math.radians(residual)))

    result = {"joint": j, "start": start, "target": target[j],
              "settled": settled[j], "residual": residual,
              "rows": [], "ratios": [], "steps": [], "below_min": False,
              "move_failed": False, "amplitude": amplitude,
              # A correction REVERSES when its sign opposes the approach. That
              # single flag separated 17/18 responding from 0/4 stalling on
              # 2026-07-28 and is the most actionable result in the test.
              "reversal": (residual < 0) != (amplitude < 0)}

    # The joint never moved. e is not an error to correct, it is the whole
    # commanded motion, and target + k*e would be a fresh full-sized move --
    # so refuse to run the staircase rather than walk the joint out of range.
    if abs(residual) >= FAILED_MOVE_FRACTION * abs(amplitude):
        result["move_failed"] = True
        moved = abs(settled[j] - start[j])
        print("[probe] MOVE FAILED: joint {} was commanded {:+.1f} deg and moved "
              "{:.2f} deg.".format(j, amplitude, moved))
        print("[probe]   residual ({:.2f} deg) is >= {:.0f}% of the amplitude, so "
              "there is no residual to correct here -- the move itself did not "
              "happen.".format(abs(residual), 100 * FAILED_MOVE_FRACTION))
        print("[probe]   Staircase SKIPPED (extrapolating k*e from this would "
              "command a second full-sized move).")
        return result

    if abs(residual) < args.deadzone_min_deg:
        result["below_min"] = True
        return result

    if not quiet:
        print("[probe] --- overshoot staircase: commanding target + k*e ---")
        print("[probe] {:>3} {:>12} {:>12} {:>12} {:>10}"
              .format("k", "commanded", "settled", "error", "|err|/|e|"))
    previous = settled
    for k in range(1, args.deadzone_steps + 1):
        biased = list(target)
        biased[j] = target[j] + k * residual
        ok, bad_i, bad_v, lo, hi = within_limits(biased)
        if not ok:
            print("[probe] stopping staircase at k={}: would command joint {} to "
                  "{:.2f} deg, outside its {:.0f}..{:.0f} limit"
                  .format(k, bad_i, bad_v, lo, hi))
            break
        arm.send_angles(biased, args.speed)
        measured = settle(arm, args, timeout_sec=args.settle_timeout_sec,
                          reference=previous)
        if measured is None:
            print("[probe] ERROR: lost readings at k={}".format(k), file=sys.stderr)
            break
        previous = measured
        err = target[j] - measured[j]
        ratio = abs(err) / abs(residual) if residual else float("nan")
        if not quiet:
            print("[probe] {:>3} {:>12.3f} {:>12.3f} {:>+12.4f} {:>10.2f}"
                  .format(k, biased[j], measured[j], err, ratio))
        result["rows"].append((time.time(), biased[j], measured[j]))
        result["steps"].append({"k": k, "commanded": biased[j],
                                "settled": measured[j], "err": err,
                                "ratio": ratio})
        result["ratios"].append(ratio)
    return result


def mode_deadzone(arm, args):
    """TEST 1 -- does commanding PAST the target correct a residual error?

    This is the go/no-go for outer-loop control, and it is the half of the
    question the existing evidence does NOT answer. mycobot_bridge.py's settle
    logic has re-sent the SAME target at full speed many times and watched the
    error not move by a digit (residuals of 0.0315, 0.0331, 0.0348, 0.0387,
    0.0504 rad observed across 2026-07-27/28). That proves re-commanding the
    same value fails. It says nothing about commanding a DIFFERENT value.

    So: move the joint, measure the residual e, then command target + k*e for
    k = 1, 2, 3 and watch what the joint does.

    The question is CORRECTABLE vs NOT, which matters more than naming the
    mechanism -- simulating both candidate plants shows they are corrected the
    same way:

      compliance / gravity droop (steady state = gain * command)
          -> |err|/|e| at k=1 was 0.10 in simulation
      "stops short" dead band (inert within dz of the command, else lands
          dz short -- this is the model that reproduces the observed
          behaviour, where re-sending the SAME value never moves it)
          -> |err|/|e| at k=1 was 0.00 in simulation

    Both are fixed by biasing the command, so both mean the control project is
    viable. What would NOT be correctable is a joint that ignores the bias
    entirely, or whose residual changes unpredictably run to run (stiction,
    backlash).

    READ THE k=1 ROW. That is the whole answer. Rising |err|/|e| at k=2 and
    k=3 is EXPECTED and is good news -- it means the joint tracks a biased
    command proportionally, so it overshoots when over-biased. A flat column
    that never moves at any k is the bad outcome.

    ONE JOINT AT ONE POSE IS NOT A RESULT -- gravity load varies hugely with
    posture. Prefer --deadzone-sweep, which runs the whole matrix unattended
    and additionally checks whether the residual scales with the gravity
    moment arm, which single runs cannot show."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1

    print("[probe] start pose (deg): {}".format([round(a, 2) for a in start]))
    print("[probe] testing joint {} (0-based), moving {:+.1f} deg at speed {}"
          .format(args.joint, args.amplitude_deg, args.speed))
    print()

    result = run_deadzone_trial(arm, args, args.joint)
    if result is None:
        return 1
    print()

    if result.get("move_failed"):
        print("[probe] No usable measurement: the commanded move did not execute,")
        print("[probe] so there is no residual to characterise. Retry with a lower")
        print("[probe] --speed or a longer --settle-timeout-sec.")
        settle_and_write(arm, args, result["start"], [])
        return 1

    if result["below_min"]:
        print("[probe] residual is below --deadzone-min-deg ({} deg): this joint"
              .format(args.deadzone_min_deg))
        print("[probe] reached its target at this pose, so there is nothing to")
        print("[probe] correct here. Retry at a pose with more gravity load")
        print("[probe] (arm extended), or with a larger --amplitude-deg.")
        settle_and_write(arm, args, result["start"], [])
        return 0

    label, lines = deadzone_verdict(result["ratios"], args.deadzone_steps)
    print("[probe] VERDICT: {}".format(label))
    for line in lines:
        print("[probe]   {}".format(line))
    print("[probe] ONE JOINT AT ONE POSE IS NOT A RESULT. Use --deadzone-sweep")
    print("[probe] to run every joint at every posture and test whether the")
    print("[probe] residual tracks gravity load.")

    settle_and_write(arm, args, result["start"], result["rows"])
    return 0


def mode_deadzone_sweep(arm, args):
    """TEST 1, the whole matrix, unattended.

    Runs the dead-zone trial for every requested joint at every posture in
    POSTURES, moving the arm to each posture itself and restoring it between
    trials. Replaces roughly 20-25 minutes of manual repositioning, rerunning
    and transcribing with a single command.

    It also answers a question no individual run can. Each posture has a known
    gravity moment arm per joint (computed from the URDF, see POSTURES), and
    the three span 0.29 / 0.15 / 0.00 m on joint 2. So:

      residual scales with the moment arm  -> gravity droop dominates. A
          feedforward gravity term or an integrator fixes it, and the fix
          generalises across the workspace once identified.
      residual is flat across postures     -> a dead band / stiction floor
          that has nothing to do with load. Biasing still works if the
          staircase says CORRECTABLE, but no gravity model will help.
      joint 0 shows a residual at all      -> that is the gravity-free
          control (its moment arm is 0.0000 everywhere), so whatever it
          shows is the floor the other joints cannot beat.

    The staircase verdict says whether closed-loop correction works at all;
    this cross-posture comparison says what the controller has to model."""
    postures = [p for p in POSTURES if p[0] in args.sweep_postures]
    if not postures:
        print("[probe] ERROR: no postures matched {}".format(args.sweep_postures),
              file=sys.stderr)
        return 1
    joints = args.sweep_joints

    amplitudes = ([args.amplitude_deg, -args.amplitude_deg]
                  if args.directions == "both"
                  else [args.amplitude_deg] if args.directions == "positive"
                  else [-args.amplitude_deg])
    trials = len(postures) * len(joints) * args.repeats * len(amplitudes)
    backlash_trials = 0 if args.no_backlash else len(postures) * len(joints)
    moves = (trials * (2 + args.deadzone_steps)
             + backlash_trials * 5)
    print()
    print("[probe] === TEST 1 SWEEP PLAN ===")
    print("[probe] postures : {}".format(", ".join(p[0] for p in postures)))
    print("[probe] joints   : {} (0-based)".format(
        ", ".join(str(j) for j in joints)))
    print("[probe] amplitude: {} deg   staircase steps: {}   repeats: {}"
          .format("/".join("{:+.0f}".format(a) for a in amplitudes),
                  args.deadzone_steps, args.repeats))
    print("[probe] backlash : {}".format(
        "skipped" if args.no_backlash
        else "{} trials (same target approached from both sides)"
             .format(backlash_trials)))
    print("[probe] {} trials, ~{} settling moves, roughly {:.0f}-{:.0f} min"
          .format(trials, moves, moves * 1.0 / 60.0, moves * 2.5 / 60.0))
    print()
    for name, angles, grav, lever, clearance in postures:
        print("[probe]   {:<9} {}".format(name, angles))
        print("[probe]             gravity arm J1/J2/J3 = {}   inertia lever = {}"
              "   clearance {:.3f}m"
              .format("/".join("{:.3f}".format(a) for a in grav),
                      "/".join("{:.3f}".format(a) for a in lever), clearance))
    print()
    print("[probe] THE ARM WILL MOVE THROUGH ALL OF THESE UNATTENDED. Clear the")
    print("[probe] workspace, remove any block or fixture, and make sure the")
    print("[probe] gripper is empty. Clearances above are from the URDF and do")
    print("[probe] NOT model the table, cables, or anything you left nearby.")
    print()

    if args.dry_run:
        print("[probe] --dry-run: nothing was moved, no serial port opened.")
        return 0

    origin = read_angles(arm)
    if origin is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1

    if not args.yes:
        try:
            if input("[probe] press Enter to start, Ctrl-C to abort: ").strip():
                pass
        except (EOFError, KeyboardInterrupt):
            print("\n[probe] aborted.")
            return 1

    results = []
    backlash = []
    try:
        for name, angles, grav, lever, _clearance in postures:
            print()
            print("[probe] " + "=" * 62)
            print("[probe] POSTURE '{}'".format(name))
            print("[probe] " + "=" * 62)
            for j in joints:
                for rep in range(args.repeats):
                    for amp in amplitudes:
                        print()
                        # Establish the posture BEFORE each trial and verify it,
                        # rather than restoring it afterwards and hoping. A trial
                        # that starts somewhere else is not a measurement of this
                        # load level.
                        if move_to_posture(arm, args, angles, name) is None:
                            print("[probe] SKIPPING '{}' joint {} rep {} amp {:+.0f}: "
                                  "could not reach the posture, so this trial would "
                                  "be credited with a moment arm it was never at."
                                  .format(name, j, rep + 1, amp))
                            continue
                        print("[probe] --- '{}' joint {} (gravity {:.4f}m, inertia "
                              "{:.4f}m) amp {:+.0f} deg{} ---"
                              .format(name, j, grav[j], lever[j], amp,
                                      "" if args.repeats == 1
                                      else "  rep {}/{}".format(rep + 1, args.repeats)))
                        # One bad trial must not lose all the others. pymycobot
                        # raises on out-of-range angles, and a serial hiccup can
                        # throw from anywhere in the vendor stack.
                        try:
                            result = run_deadzone_trial(arm, args, j, amplitude=amp)
                        except Exception as exc:
                            print("[probe] TRIAL FAILED ('{}', joint {}, amp {:+.0f}): "
                                  "{!r}".format(name, j, amp, exc), file=sys.stderr)
                            result = None
                        if result is not None:
                            result["posture"] = name
                            result["repeat"] = rep
                            result["moment_arm"] = grav[j]
                            result["inertia_lever"] = lever[j]
                            results.append(result)

                    if not args.no_backlash:
                        print()
                        if move_to_posture(arm, args, angles, name) is None:
                            continue
                        print("[probe] --- '{}' joint {} BACKLASH (same target from "
                              "both sides) ---".format(name, j))
                        try:
                            b = run_backlash_trial(arm, args, j)
                        except Exception as exc:
                            print("[probe] BACKLASH FAILED ('{}', joint {}): {!r}"
                                  .format(name, j, exc), file=sys.stderr)
                            b = None
                        if b is not None:
                            b["posture"] = name
                            b["moment_arm"] = grav[j]
                            b["inertia_lever"] = lever[j]
                            backlash.append(b)
    except KeyboardInterrupt:
        print("\n[probe] interrupted -- returning the arm and reporting what "
              "was collected so far.")
    finally:
        print()
        print("[probe] returning to the pose the sweep started from...")
        try:
            arm.send_angles(list(origin), args.sweep_speed)
            settle(arm, args, timeout_sec=args.posture_settle_sec)
        except Exception as exc:
            print("[probe] WARNING: return failed: {!r}".format(exc),
                  file=sys.stderr)

    report_sweep(args, results, backlash)
    return 0


def linear_fit(xs, ys):
    """Least-squares (slope, intercept, R^2), or None if x has no spread."""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx < 1e-12:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sxx
    intercept = mean_y - slope * mean_x
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return slope, intercept, r2


def multi_fit(rows, ys):
    """Least squares for y = a + b*x1 + c*x2 via normal equations.

    Two regressors, so a closed-form 3x3 solve is simpler and dependency-free
    compared to pulling in numpy on the Pi. Returns (a, b, c, R^2) or None if
    the design is rank-deficient -- which is exactly what happens when the two
    regressors are collinear, the failure the six-posture set exists to avoid."""
    n = len(rows)
    if n < 4:
        return None
    x1 = [r[0] for r in rows]
    x2 = [r[1] for r in rows]
    cols = [[1.0] * n, x1, x2]
    ata = [[sum(cols[i][k] * cols[j][k] for k in range(n)) for j in range(3)]
           for i in range(3)]
    atb = [sum(cols[i][k] * ys[k] for k in range(n)) for i in range(3)]
    # Gaussian elimination with partial pivoting.
    m = [ata[i][:] + [atb[i]] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    coef = [m[i][3] / m[i][i] for i in range(3)]
    mean_y = sum(ys) / n
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((ys[k] - (coef[0] + coef[1] * x1[k] + coef[2] * x2[k])) ** 2
                 for k in range(n))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return coef[0], coef[1], coef[2], r2


def report_sweep(args, results, backlash=None):
    """Summary table, the direction split, the gravity/inertia fit, and the CSV."""
    backlash = backlash or []
    if not results:
        print("[probe] no trials completed.")
        return

    usable = [r for r in results if not r.get("move_failed")]
    failed = [r for r in results if r.get("move_failed")]

    print()
    print("[probe] " + "=" * 78)
    print("[probe] TEST 1 SUMMARY -- {} trials, {} usable, {} never moved"
          .format(len(results), len(usable), len(failed)))
    print("[probe] " + "=" * 78)

    # ---------------------------------------------------------------- 1. GO/NO-GO
    # Split by whether the CORRECTION reverses the approach direction. On
    # 2026-07-28 this split 17/18 responding against 0/4 stalling, and the old
    # per-trial verdict column hid it completely by calling the stalled ones
    # "MIXED" -- the single most actionable result in the test, reported as
    # noise. It leads the summary now.
    staircased = [r for r in usable if r["ratios"]]
    fwd = [r for r in staircased if not r["reversal"]]
    rev = [r for r in staircased if r["reversal"]]
    print()
    print("[probe] --- 1. GO/NO-GO: does a biased command move the joint? ---")
    for label, group in (("SAME direction as the move", fwd),
                         ("REVERSING the move", rev)):
        if not group:
            print("[probe]   {:<28} no trials".format(label))
            continue
        k1 = sorted(r["ratios"][0] for r in group)
        responded = sum(1 for r in k1 if r < 0.8)
        print("[probe]   {:<28} n={:<4} responded {}/{}  k=1 ratio "
              "min {:.2f} / median {:.2f} / max {:.2f}"
              .format(label, len(group), responded, len(group),
                      k1[0], k1[len(k1) // 2], k1[-1]))
    if fwd:
        k1f = sorted(r["ratios"][0] for r in fwd)
        med = k1f[len(k1f) // 2]
        rate = sum(1 for r in k1f if r < 0.8) / float(len(k1f))
        if rate >= 0.8 and med < 0.5:
            print("[probe]   VERDICT: GO. A same-direction corrective command "
                  "removes {:.0f}% of the error in one step, {:.0f}% of the time. "
                  "An outer-loop integrator or disturbance observer can close "
                  "this.".format(100 * (1 - med), 100 * rate))
        else:
            print("[probe]   VERDICT: NO-GO on plain outer-loop correction -- "
                  "even same-direction commands do not reliably move the joint.")
    if rev:
        k1r = [r["ratios"][0] for r in rev]
        stalled = sum(1 for r in k1r if r >= 0.8)
        print("[probe]   CONDITION: {}/{} reversing corrections STALLED. Approach "
              "every target from one side, or feed backlash forward -- an "
              "integrator cannot fix this, the joint does not move at all."
              .format(stalled, len(k1r)))

    # ------------------------------------------------------------- 2. BACKLASH
    if backlash:
        print()
        print("[probe] --- 2. BACKLASH (lost motion, same target from both sides) ---")
        by_joint = {}
        for b in backlash:
            by_joint.setdefault(b["joint"], []).append(abs(b["lost_motion"]))
        for j in sorted(by_joint):
            v = sorted(by_joint[j])
            med = v[len(v) // 2]
            print("[probe]   joint {}: n={:<3} median {:.3f} deg ({:.1f} counts) "
                  "= {:.2f} mm at 250mm reach   [range {:.3f}..{:.3f}]"
                  .format(j, len(v), med, med / ENCODER_COUNT_DEG,
                          math.radians(med) * 250.0, v[0], v[-1]))
        allv = sorted(abs(b["lost_motion"]) for b in backlash)
        med = allv[len(allv) // 2]
        print("[probe]   Overall median {:.3f} deg = {:.2f} mm. This is NOT "
              "correctable by feedback: on a reversal the joint does not move, "
              "so the error carries no information the loop can act on."
              .format(med, math.radians(med) * 250.0))

    # ------------------------------------- 3. WHAT THE RESIDUAL DEPENDS ON
    print()
    print("[probe] --- 3. Residual vs gravity AND inertia (they are separated "
          "here, not assumed) ---")
    groups = {}
    for r in usable:
        groups.setdefault((r["posture"], r["joint"], r["amplitude"] > 0), []).append(r)

    # FIT EACH DIRECTION SEPARATELY -- pooling them cancels the signal.
    # Gravity torque is fixed in joint coordinates, so its contribution to the
    # residual does NOT flip when the approach direction flips. Friction, dead
    # zone and inertial overshoot all act along or against travel, so they DO
    # flip. Pool the two and the direction-dependent terms cancel while the
    # gravity term survives at half weight -- a synthetic dataset with a planted
    # 7.50 deg/m droop fitted back as -0.04 deg/m, R^2 0.000, before this split
    # was added.
    #
    # Fitting separately also gives a free consistency check: the gravity slope
    # must agree between the two directions, while the intercept must flip sign.
    # If it does not, the model is wrong, and that is worth knowing.
    for j in sorted({key[1] for key in groups}):
        slopes = {}
        for positive in (True, False):
            rows, ys, detail = [], [], []
            for (posture, joint, is_pos), items in sorted(groups.items()):
                if joint != j or is_pos != positive:
                    continue
                res = [x["residual"] for x in items]
                rows.append((items[0]["moment_arm"], items[0]["inertia_lever"]))
                ys.append(sum(res) / len(res))
                detail.append("{}:{:+.2f}".format(posture, ys[-1]))
            if len(ys) < 2:
                continue
            label = "approach {}".format("+" if positive else "-")
            print()
            print("[probe] JOINT {}  {}   ({} postures)".format(j, label, len(ys)))
            print("[probe]   {}".format("  ".join(detail)))

            grav = [r[0] for r in rows]
            lever = [r[1] for r in rows]
            if max(grav) - min(grav) < 1e-4:
                mean = sum(ys) / len(ys)
                span = max(ys) - min(ys)
                print("[probe]   Gravity arm is 0.0000 in every posture -- gravity "
                      "CANNOT load this joint. Residual {:+.3f} deg mean, {:.3f} "
                      "deg range = {:.1f} counts.".format(
                          mean, span, span / ENCODER_COUNT_DEG))
                fit1 = linear_fit(lever, ys)
                if fit1 and abs(fit1[2]) > 0.5:
                    print("[probe]   ...but it tracks the INERTIA lever "
                          "({:+.2f} deg/m, R^2 {:.2f}). Posture moves the residual "
                          "through a NON-gravity channel, and this is its size -- "
                          "the confound the six-posture set exists to expose."
                          .format(fit1[0], fit1[2]))
                continue

            fit = multi_fit(rows, ys)
            if fit is None:
                print("[probe]   too few postures for a two-variable fit "
                      "(need 4+).")
                continue
            a, b, c, r2 = fit
            slopes[positive] = b
            print("[probe]   residual = {:+.3f} {:+.2f}*gravity_arm "
                  "{:+.2f}*inertia_lever    R^2 = {:.3f}".format(a, b, c, r2))
            span_g = abs(b) * (max(grav) - min(grav))
            span_i = abs(c) * (max(lever) - min(lever))
            total = span_g + span_i + 1e-9
            print("[probe]   across the tested range: {:.3f} deg from gravity, "
                  "{:.3f} deg from inertia ({:.0f}% / {:.0f}%)"
                  .format(span_g, span_i, 100 * span_g / total, 100 * span_i / total))
            print("[probe]   -> gravity feedforward {:+.2f} deg/m of moment arm; "
                  "dead-zone bias {:+.3f} deg".format(b, a))
            if r2 < 0.8:
                print("[probe]   WARNING: R^2 {:.2f} is weak -- these two terms do "
                      "not explain the residual well.".format(r2))

        if len(slopes) == 2:
            up, down = slopes[True], slopes[False]
            spread = abs(up - down)
            mean = 0.5 * (abs(up) + abs(down))
            print()
            print("[probe]   CONSISTENCY: gravity slope {:+.2f} from + approach, "
                  "{:+.2f} from -.".format(up, down))
            if mean > 1e-6 and spread / mean < 0.3:
                print("[probe]   They agree to {:.0f}%, which is what gravity must "
                      "do -- it is fixed in joint coordinates and cannot care "
                      "which way you drove in. The model holds."
                      .format(100 * spread / mean))
            else:
                print("[probe]   They DISAGREE. Gravity cannot depend on approach "
                      "direction, so something direction-dependent is being "
                      "absorbed into the gravity term. Do not use this slope as "
                      "a feedforward until that is understood.")

    if failed:
        print()
        print("[probe] {} of {} trials never moved and were excluded. {} is "
              "{:.0f}%, {}".format(
                  len(failed), len(results), len(failed),
                  100.0 * len(failed) / len(results),
                  "acceptable" if len(failed) <= 0.05 * len(results)
                  else "TOO HIGH -- lower --speed and rerun before trusting this"))

    if not args.out:
        print()
        print("[probe] (pass --out FILE.csv to keep the raw data)")
        return
    try:
        with open(args.out, "w") as handle:
            handle.write("kind,posture,joint,amplitude_deg,gravity_arm_m,"
                         "inertia_lever_m,reversal,residual_deg,k,commanded_deg,"
                         "settled_deg,err_deg,ratio,lost_motion_deg\n")
            for r in results:
                base = "deadzone,{},{},{:.1f},{:.4f},{:.4f},{},{:.4f}".format(
                    r["posture"], r["joint"], r["amplitude"], r["moment_arm"],
                    r.get("inertia_lever", float("nan")),
                    int(bool(r.get("reversal"))), r["residual"])
                if not r["steps"]:
                    handle.write(base + ",,,,,,\n")
                for st in r["steps"]:
                    handle.write(base + ",{},{:.4f},{:.4f},{:.4f},{:.4f},\n".format(
                        st["k"], st["commanded"], st["settled"], st["err"],
                        st["ratio"]))
            for b in backlash:
                handle.write("backlash,{},{},,{:.4f},{:.4f},,,,,,,,{:.4f}\n".format(
                    b["posture"], b["joint"], b["moment_arm"],
                    b.get("inertia_lever", float("nan")), b["lost_motion"]))
        print()
        print("[probe] wrote {} trials + {} backlash measurements to {}"
              .format(len(results), len(backlash), args.out))
    except IOError as exc:
        print("[probe] WARNING: could not write {}: {!r}".format(args.out, exc),
              file=sys.stderr)


def mode_probe(arm, args):
    """Time get_angles() and send_angles() both while the arm is parked and
    while it is mid-move. The mid-move numbers are the ones that matter: the
    bridge's loop rate collapses from ~89Hz to 1-3Hz exactly when the arm is
    moving, and this says which of the two calls is responsible."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1
    print("[probe] start pose (deg): {}".format([round(a, 2) for a in start]))

    print("\n[probe] --- IDLE (arm parked, nothing commanded) ---")
    idle_reads = []
    for _ in range(args.samples):
        _, ms = timed(read_angles, arm)
        idle_reads.append(ms)
    summarize("get_angles idle", idle_reads)

    # A send_angles to where the arm already is: measures the CALL cost with
    # no physical motion, separating serial/protocol overhead from the time
    # the firmware spends actually driving the joint.
    idle_writes = []
    for _ in range(args.samples):
        _, ms = timed(arm.send_angles, list(start), args.speed)
        idle_writes.append(ms)
    summarize("send_angles idle (no motion)", idle_writes)

    print("\n[probe] --- MID-MOVE (same calls while the arm is driving) ---")
    target = list(start)
    target[args.joint] = start[args.joint] + args.amplitude_deg
    arm.send_angles(target, args.speed)

    moving_reads = []
    moving_writes = []
    t_end = time.time() + args.move_window_sec
    while time.time() < t_end:
        _, ms = timed(read_angles, arm)
        moving_reads.append(ms)
        _, ms = timed(arm.send_angles, list(target), args.speed)
        moving_writes.append(ms)
    summarize("get_angles mid-move", moving_reads)
    summarize("send_angles mid-move", moving_writes)

    if moving_reads and moving_writes:
        period = statistics.median(moving_reads) + statistics.median(moving_writes)
        print("\n[probe] implied serial-loop ceiling while moving: "
              "{:.1f}Hz ({:.0f}ms/iteration)".format(1000.0 / period, period))
        print("[probe] that is the hard upper bound on distinct position "
              "commands/sec reaching the arm during a trajectory")
    return 0


def mode_stream(arm, args):
    """Stream a smooth constant-velocity ramp on one joint as discrete
    send_angles() calls at --rate Hz, recording commanded vs measured.

    This is precisely what mycobot_bridge.py does with JTC's setpoints, minus
    every other moving part. Sweep --rate (try 2, 5, 10, 20) and see where
    the measured trace stops being a staircase."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1

    rows = []
    period = 1.0 / args.rate
    t_start = time.time()
    commanded = list(start)
    sent = 0

    try:
        while True:
            t = time.time() - t_start
            if t >= args.duration_sec:
                break
            # Constant-velocity ramp -- same profile _apply_uniform_timing
            # produces, so this reproduces the real command stream's shape.
            frac = t / args.duration_sec
            commanded[args.joint] = start[args.joint] + frac * args.amplitude_deg
            arm.send_angles(list(commanded), args.speed)
            sent += 1
            measured = read_angles(arm)
            rows.append((time.time() - t_start,
                         commanded[args.joint],
                         measured[args.joint] if measured else float("nan")))
            # Whatever is left of the period after the two serial round trips.
            # If this goes negative the requested --rate is above what the
            # link can sustain, which is itself the finding.
            slack = period - (time.time() - t_start - t)
            if slack > 0:
                time.sleep(slack)
    finally:
        elapsed = time.time() - t_start
        print("[probe] streamed {} commands in {:.2f}s = {:.1f}Hz achieved "
              "(requested {:.1f}Hz)".format(sent, elapsed,
                                            sent / elapsed if elapsed else 0.0,
                                            args.rate))
        settle_and_write(arm, args, start, rows)
    return 0


def mode_point_to_point(arm, args):
    """The control case: ONE send_angles() for the whole motion, sampled at
    the same cadence. Every real-hardware motion this project has confirmed
    working was effectively this -- a single uninterrupted firmware move --
    which is why single-waypoint goals look healthy and streamed trajectories
    do not."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1

    target = list(start)
    target[args.joint] = start[args.joint] + args.amplitude_deg

    rows = []
    t_start = time.time()
    arm.send_angles(target, args.speed)
    try:
        while time.time() - t_start < args.duration_sec:
            measured = read_angles(arm)
            rows.append((time.time() - t_start,
                         target[args.joint],
                         measured[args.joint] if measured else float("nan")))
    finally:
        settle_and_write(arm, args, start, rows)
    return 0


def settle_and_write(arm, args, start, rows):
    """Always return the joint to where it started, then persist the trace.
    Runs from a finally: block so Ctrl-C still puts the arm back."""
    print("[probe] returning joint {} to its starting angle...".format(args.joint))
    try:
        arm.send_angles(list(start), args.speed)
        time.sleep(args.return_settle_sec)
    except Exception as exc:
        print("[probe] WARNING: return-to-start failed: {!r}".format(exc),
              file=sys.stderr)

    if not args.out:
        return
    try:
        with open(args.out, "w") as handle:
            handle.write("t_sec,commanded_deg,measured_deg\n")
            for t, commanded, measured in rows:
                handle.write("{:.4f},{:.4f},{:.4f}\n".format(t, commanded, measured))
        print("[probe] wrote {} samples to {}".format(len(rows), args.out))
    except IOError as exc:
        print("[probe] WARNING: could not write {}: {!r}".format(args.out, exc),
              file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--probe", action="store_true",
                      help="time get_angles/send_angles idle and mid-move")
    mode.add_argument("--stream", action="store_true",
                      help="stream a ramp as discrete setpoints at --rate Hz")
    mode.add_argument("--point-to-point", action="store_true",
                      help="the same motion as one uninterrupted send_angles")
    mode.add_argument("--deadzone", action="store_true",
                      help="TEST 1, one joint at one pose: move the joint, measure "
                           "the residual error, then command target + k*e to see "
                           "whether biasing the command corrects it. Discriminates "
                           "gravity droop (outer-loop control works) from a servo "
                           "dead zone (it does not). This is the go/no-go.")
    mode.add_argument("--deadzone-sweep", action="store_true",
                      help="TEST 1, the whole matrix, unattended: every --sweep-joints "
                           "at every --sweep-postures, moving the arm to each posture "
                           "itself. Also reports whether the residual scales with the "
                           "gravity moment arm, which a single run cannot show. "
                           "Preferred over --deadzone.")

    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baud-rate", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--joint", type=int, default=0,
                        help="0-based joint index to move (default %(default)s, "
                             "the base yaw)")
    parser.add_argument("--amplitude-deg", type=float, default=30.0,
                        help="how far to move that joint (default %(default)s)")
    parser.add_argument("--speed", type=int, default=30,
                        help="pymycobot speed 0-100 (default %(default)s)")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="--stream setpoints per second (default %(default)s)")
    parser.add_argument("--duration-sec", type=float, default=5.0,
                        help="length of the streamed/observed motion "
                             "(default %(default)s)")
    parser.add_argument("--samples", type=int, default=30,
                        help="--probe samples per idle measurement "
                             "(default %(default)s)")
    parser.add_argument("--move-window-sec", type=float, default=4.0,
                        help="--probe seconds of mid-move sampling "
                             "(default %(default)s)")
    parser.add_argument("--return-settle-sec", type=float, default=3.0,
                        help="seconds to wait for the return-to-start move "
                             "(default %(default)s)")
    parser.add_argument("--read-timeout", type=float,
                        default=DEFAULT_READ_TIMEOUT_SEC,
                        help="cap on one pymycobot read, seconds (default %(default)s, "
                             "0 keeps its hardcoded 0.5s x 3). Uncapped, sampling "
                             "during motion becomes irregular half-second gaps.")
    parser.add_argument("--settle-timeout-sec", type=float, default=6.0,
                        help="--deadzone: max wait for the arm to stop moving "
                             "(default %(default)s)")
    parser.add_argument("--deadzone-steps", type=int, default=3,
                        help="--deadzone: how many k values to try in the "
                             "overshoot staircase (default %(default)s)")
    parser.add_argument("--deadzone-min-deg", type=float, default=0.3,
                        help="--deadzone: below this residual there is nothing to "
                             "correct, so the test reports that and stops "
                             "(default %(default)s)")
    parser.add_argument("--sweep-joints", default="0,1,2",
                        help="--deadzone-sweep: comma-separated 0-based joints "
                             "(default %(default)s -- joint 0 is the gravity-free "
                             "control, 1 and 2 are where the load actually varies; "
                             "joints 4 and 5 have ~zero moment arm in every posture "
                             "and are not worth sweeping)")
    parser.add_argument("--sweep-postures",
                        default=",".join(p[0] for p in POSTURES),
                        help="--deadzone-sweep: comma-separated posture names from "
                             "POSTURES (default %(default)s)")
    parser.add_argument("--sweep-speed", type=int, default=25,
                        help="--deadzone-sweep: speed for the posture-to-posture "
                             "moves, which are large six-joint motions "
                             "(default %(default)s)")
    parser.add_argument("--posture-settle-sec", type=float, default=12.0,
                        help="--deadzone-sweep: settle budget for a whole-arm "
                             "posture move (default %(default)s)")
    parser.add_argument("--repeats", type=int, default=1,
                        help="--deadzone-sweep: trials per (posture, joint). The "
                             "gravity-free joint's own scatter across postures was "
                             "0.34 deg on the first good run, against residuals of "
                             "0.5-1.2 deg -- so a single trial per cell barely "
                             "clears its own noise. 3 gives an error bar "
                             "(default %(default)s)")
    parser.add_argument("--directions", choices=("both", "positive", "negative"),
                        default="both",
                        help="--deadzone-sweep: run each trial with +amplitude, "
                             "-amplitude, or both. 'both' is strongly preferred: "
                             "whether a correction REVERSES the approach is what "
                             "decides if it works at all, and a single-direction "
                             "sweep only samples reversals by accident "
                             "(default %(default)s)")
    parser.add_argument("--no-backlash", action="store_true",
                        help="--deadzone-sweep: skip the backlash measurement "
                             "(same target approached from both sides). It is the "
                             "dominant error term, so only skip it for a quick "
                             "partial run")
    parser.add_argument("--dry-run", action="store_true",
                        help="--deadzone-sweep: print the plan and exit without "
                             "moving the arm")
    parser.add_argument("--yes", action="store_true",
                        help="--deadzone-sweep: skip the confirmation prompt")
    parser.add_argument("--out", default=None,
                        help="CSV path for the commanded-vs-measured trace")
    args = parser.parse_args()

    if not 0 <= args.joint <= 5:
        parser.error("--joint must be 0-5 (arm joints only; this script "
                     "deliberately never touches the gripper)")

    args.sweep_postures = [s.strip() for s in args.sweep_postures.split(",")
                           if s.strip()]
    known = {p[0] for p in POSTURES}
    unknown = [s for s in args.sweep_postures if s not in known]
    if unknown:
        parser.error("unknown posture(s) {}; known: {}".format(
            ", ".join(unknown), ", ".join(sorted(known))))
    try:
        args.sweep_joints = [int(s) for s in args.sweep_joints.split(",") if s.strip()]
    except ValueError:
        parser.error("--sweep-joints must be comma-separated integers")
    if any(not 0 <= j <= 5 for j in args.sweep_joints):
        parser.error("--sweep-joints entries must be 0-5")

    # --dry-run must not open the serial port: the whole point is that it can
    # be run on mars, away from the robot, to check the plan.
    if args.dry_run and args.deadzone_sweep:
        return mode_deadzone_sweep(None, args)

    arm = connect(args.serial_port, args.baud_rate, args.read_timeout)

    if args.deadzone_sweep:
        return mode_deadzone_sweep(arm, args)
    if args.deadzone:
        return mode_deadzone(arm, args)
    if args.probe:
        return mode_probe(arm, args)
    if args.stream:
        return mode_stream(arm, args)
    return mode_point_to_point(arm, args)


if __name__ == "__main__":
    sys.exit(main())
