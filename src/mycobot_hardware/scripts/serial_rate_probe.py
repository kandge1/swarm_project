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


def settle(arm, args, timeout_sec=6.0, stable_reads=4,
           stable_deg=SETTLE_STABLE_DEG, reference=None,
           min_motion_deg=SETTLE_MIN_MOTION_DEG, motion_grace_sec=2.0):
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
    history = []
    last_good = None
    moved = reference is None
    while time.time() < deadline:
        angles = read_angles(arm)
        if angles is not None:
            last_good = angles
            if not moved and max(abs(angles[i] - reference[i])
                                 for i in range(6)) >= min_motion_deg:
                moved = True
            history.append(angles)
            if len(history) > stable_reads:
                history.pop(0)
            if len(history) == stable_reads:
                spread = max(
                    max(abs(h[j] - history[0][j]) for h in history)
                    for j in range(6))
                if spread <= stable_deg and (moved or time.time() >= grace_until):
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
POSTURES = [
    # name      angles (deg, 6 joints)        armJ1   armJ2   armJ3  min clearance
    ("loaded", [0, -45, -30, 45, 0, -45], (0.0000, 0.2902, 0.2121), 0.0777),
    ("mid",    [0, -15, -15, 45, 0, -45], (0.0000, 0.1500, 0.1214), 0.2166),
    ("light",  [0,   0,  30,  0, 0, -45], (0.0000, 0.0018, 0.0018), 0.2342),
]


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
    before = read_angles(arm)
    arm.send_angles(list(angles), args.sweep_speed)
    settled = settle(arm, args, timeout_sec=args.posture_settle_sec,
                     reference=before)
    if settled is None:
        print("[probe] ERROR: no valid reading after moving to '{}'".format(label),
              file=sys.stderr)
    return settled


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


def run_deadzone_trial(arm, args, joint_idx, quiet=False):
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
    target = list(start)
    target[j] = start[j] + args.amplitude_deg

    ok, bad_i, bad_v, lo, hi = within_limits(target)
    if not ok:
        print("[probe] SKIP: the initial move would put joint {} at {:.2f} deg, "
              "outside its {:.0f}..{:.0f} limit".format(bad_i, bad_v, lo, hi))
        return None

    arm.send_angles(target, args.speed)
    settled = settle(arm, args, timeout_sec=args.settle_timeout_sec,
                     reference=start)
    if settled is None:
        print("[probe] ERROR: no valid reading after the initial move", file=sys.stderr)
        return None

    residual = target[j] - settled[j]
    if not quiet:
        print("[probe] commanded {:.3f} deg, settled at {:.3f} deg"
              .format(target[j], settled[j]))
        print("[probe] residual e = {:+.4f} deg ({:+.5f} rad)"
              .format(residual, math.radians(residual)))

    result = {"joint": j, "start": start, "target": target[j],
              "settled": settled[j], "residual": residual,
              "rows": [], "ratios": [], "steps": [], "below_min": False,
              "move_failed": False}

    # The joint never moved. e is not an error to correct, it is the whole
    # commanded motion, and target + k*e would be a fresh full-sized move --
    # so refuse to run the staircase rather than walk the joint out of range.
    if abs(residual) >= FAILED_MOVE_FRACTION * abs(args.amplitude_deg):
        result["move_failed"] = True
        moved = abs(settled[j] - start[j])
        print("[probe] MOVE FAILED: joint {} was commanded {:+.1f} deg and moved "
              "{:.2f} deg.".format(j, args.amplitude_deg, moved))
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

    trials = len(postures) * len(joints) * args.repeats
    moves = trials * (1 + args.deadzone_steps) + len(postures) * (1 + len(joints))
    print()
    print("[probe] === TEST 1 SWEEP PLAN ===")
    print("[probe] postures : {}".format(", ".join(p[0] for p in postures)))
    print("[probe] joints   : {} (0-based)".format(
        ", ".join(str(j) for j in joints)))
    print("[probe] amplitude: {:+.1f} deg   staircase steps: {}"
          .format(args.amplitude_deg, args.deadzone_steps))
    print("[probe] {} trials, ~{} settling moves, roughly {:.0f}-{:.0f} min"
          .format(trials, moves, moves * 2.5 / 60.0, moves * 5.0 / 60.0))
    print()
    for name, angles, arms_m, clearance in postures:
        print("[probe]   {:<7} {}  moment arms J1/J2/J3 = {}  min clearance {:.3f}m"
              .format(name, angles,
                      "/".join("{:.3f}".format(a) for a in arms_m), clearance))
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
    try:
        for name, angles, arms_m, _clearance in postures:
            print()
            print("[probe] " + "=" * 62)
            print("[probe] POSTURE '{}'".format(name))
            print("[probe] " + "=" * 62)
            if move_to_posture(arm, args, angles, name) is None:
                print("[probe] skipping posture '{}': no readings".format(name))
                continue
            for j in joints:
                for rep in range(args.repeats):
                    print()
                    print("[probe] --- posture '{}', joint {} (moment arm "
                          "{:.4f} m){} ---"
                          .format(name, j,
                                  arms_m[j] if j < len(arms_m) else float("nan"),
                                  "" if args.repeats == 1
                                  else "  repeat {}/{}".format(rep + 1, args.repeats)))
                    # One bad trial must not lose the eight good ones. pymycobot
                    # raises on out-of-range angles, and a serial hiccup can throw
                    # from anywhere in the vendor stack.
                    try:
                        result = run_deadzone_trial(arm, args, j)
                    except Exception as exc:
                        print("[probe] TRIAL FAILED (posture '{}', joint {}): {!r}"
                              .format(name, j, exc), file=sys.stderr)
                        result = None
                    if result is not None:
                        result["posture"] = name
                        result["repeat"] = rep
                        result["moment_arm"] = (arms_m[j] if j < len(arms_m)
                                                else float("nan"))
                        results.append(result)
                    # Back to the posture before the next trial, so every one
                    # starts from the same known configuration rather than from
                    # wherever the last staircase left the arm.
                    move_to_posture(arm, args, angles, name)
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

    report_sweep(args, results)
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


def report_sweep(args, results):
    """Summary table, the cross-posture gravity check, and the CSV."""
    if not results:
        print("[probe] no trials completed.")
        return

    print()
    print("[probe] " + "=" * 74)
    print("[probe] TEST 1 SUMMARY")
    print("[probe] " + "=" * 74)
    print("[probe] {:<8} {:>5} {:>10} {:>12} {:>9} {:>9} {:>9}  {}"
          .format("posture", "joint", "arm(m)", "residual(deg)",
                  "k=1", "k=2", "k=3", "verdict"))
    for r in results:
        ratios = r["ratios"]
        cells = ["{:>9.2f}".format(ratios[i]) if i < len(ratios) else "{:>9}".format("-")
                 for i in range(3)]
        if r.get("move_failed"):
            verdict = "MOVE DID NOT EXECUTE -- discard"
        elif r["below_min"]:
            verdict = "no residual to correct"
        else:
            verdict, _ = deadzone_verdict(ratios, args.deadzone_steps)
        print("[probe] {:<8} {:>5} {:>10.4f} {:>+12.4f} {} {} {}  {}"
              .format(r["posture"], r["joint"], r["moment_arm"], r["residual"],
                      cells[0], cells[1], cells[2], verdict))

    # Does the residual track gravity load? Compare each joint across postures.
    failed = [r for r in results if r.get("move_failed")]
    if failed:
        print()
        print("[probe] {} of {} trials NEVER MOVED and are excluded below."
              .format(len(failed), len(results)))
        print("[probe] A residual equal to the amplitude means settle() returned "
              "the start pose. If this is more than an occasional straggler, the "
              "readings are racing the motion -- raise --settle-timeout-sec or "
              "lower --speed and rerun; do not interpret the surviving rows as a "
              "result.")

    print()
    print("[probe] --- residual vs gravity moment arm, per joint ---")
    usable = [r for r in results if not r.get("move_failed")]

    # Group repeats of the same (posture, joint) so the fit sees one point per
    # posture with a measured scatter, rather than treating repeats as extra
    # independent load levels.
    groups = {}
    for r in usable:
        groups.setdefault((r["posture"], r["joint"]), []).append(r)

    for j in sorted({key[1] for key in groups}):
        points = []
        for (posture, joint), rows in groups.items():
            if joint != j:
                continue
            res = [x["residual"] for x in rows]
            points.append((rows[0]["moment_arm"], sum(res) / len(res),
                           max(res) - min(res), len(res), posture))
        if len(points) < 2:
            continue
        points.sort()

        detail = "  ".join(
            "{}:{:+.3f}{}@{:.3f}m".format(
                p[4], p[1], "" if p[3] == 1 else "+/-{:.2f}(n={})".format(p[2], p[3]),
                p[0])
            for p in points)
        print("[probe] joint {}: {}".format(j, detail))

        arms_m = [p[0] for p in points]
        # SIGNED, not absolute. Taking abs() here destroys the thing being
        # measured: droop and dead zone superpose as
        # residual = offset + slope * moment_arm, where offset is the
        # load-independent dead-zone/hysteresis term and can easily push the
        # lightly-loaded end NEGATIVE. abs() folds that negative end back up,
        # which both hides a genuinely linear relationship and manufactures a
        # fake "grows with load" out of a sign flip. The 2026-07-28 run hit
        # exactly this: -0.93 / +0.04 / +1.17 deg is a clean straight line in
        # load, and was reported as growth from 0.93 to 1.17.
        residuals = [p[1] for p in points]
        scatter = max((p[2] for p in points), default=0.0)
        spread_arm = max(arms_m) - min(arms_m)

        if spread_arm < 1e-4:
            mean = sum(residuals) / len(residuals)
            spread = max(residuals) - min(residuals)
            print("[probe]   gravity-free joint (moment arm ~0 at every posture). "
                  "Residual {:+.3f} deg mean, {:.3f} deg spread over {} postures "
                  "= {:.1f} encoder counts.".format(
                      mean, spread, len(residuals), abs(mean) / ENCODER_COUNT_DEG))
            print("[probe]   This is the dead-zone / stiction floor with gravity "
                  "provably absent, and the noise floor every other number here "
                  "has to clear.")
            continue

        if max(abs(r) for r in residuals) < args.deadzone_min_deg:
            print("[probe]   no meaningful residual at any posture.")
            continue

        fit = linear_fit(arms_m, residuals)
        if fit is None:
            print("[probe]   not enough spread in load to fit.")
            continue
        slope, intercept, r2 = fit
        print("[probe]   least-squares fit: residual = {:+.3f} + {:+.2f} * "
              "moment_arm   (deg, arm in m)   R^2 = {:.3f}, n = {}"
              .format(intercept, slope, r2, len(points)))

        model_range = abs(slope) * spread_arm
        if len(points) < 3:
            print("[probe]   ONLY {} LOAD LEVELS -- a line through {} points "
                  "always fits. Not evidence; rerun so the discarded posture "
                  "contributes.".format(len(points), len(points)))
        elif model_range < max(scatter, ENCODER_COUNT_DEG * 2):
            print("[probe]   The load term changes the residual by only {:.3f} deg "
                  "across the whole {:.3f}m range, under the {:.3f} deg "
                  "measurement scatter -> FLAT. Dead band / stiction floor; a "
                  "gravity feedforward term would not help."
                  .format(model_range, spread_arm,
                          max(scatter, ENCODER_COUNT_DEG * 2)))
        elif r2 >= 0.9:
            print("[probe]   Residual is LINEAR in gravity load (R^2 {:.3f}). The "
                  "{:+.3f} deg intercept is the load-independent dead-zone term; "
                  "the {:+.2f} deg/m slope is droop. Both are directly usable: "
                  "the slope as a gravity feedforward, the intercept as a "
                  "dead-zone bias.".format(r2, intercept, slope))
        else:
            print("[probe]   Load dependence present but a poor straight line "
                  "(R^2 {:.3f}). Real but not yet a model -- rerun with "
                  "--repeats to see whether the scatter or the shape is the "
                  "problem.".format(r2))

    print()
    print("[probe] Read the k=1 column for go/no-go, and the block above for "
          "what the controller has to model.")

    if not args.out:
        print("[probe] (pass --out FILE.csv to keep the raw staircase data)")
        return
    try:
        with open(args.out, "w") as handle:
            handle.write("posture,joint,moment_arm_m,residual_deg,k,"
                         "commanded_deg,settled_deg,err_deg,ratio\n")
            for r in results:
                if not r["steps"]:
                    handle.write("{},{},{:.4f},{:.4f},,,,,\n".format(
                        r["posture"], r["joint"], r["moment_arm"], r["residual"]))
                for s in r["steps"]:
                    handle.write("{},{},{:.4f},{:.4f},{},{:.4f},{:.4f},"
                                 "{:.4f},{:.4f}\n".format(
                                     r["posture"], r["joint"], r["moment_arm"],
                                     r["residual"], s["k"], s["commanded"],
                                     s["settled"], s["err"], s["ratio"]))
        print("[probe] wrote {} trials to {}".format(len(results), args.out))
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
