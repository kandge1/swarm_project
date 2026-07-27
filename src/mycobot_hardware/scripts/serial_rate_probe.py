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


def settle(arm, args, timeout_sec=6.0, stable_reads=4, stable_deg=0.05):
    """Poll until the arm stops moving, then return the settled angles.

    "Stopped" means `stable_reads` consecutive readings within `stable_deg` of
    each other, not a fixed sleep -- the point of this whole script is that
    move durations here are not predictable."""
    deadline = time.time() + timeout_sec
    history = []
    last_good = None
    while time.time() < deadline:
        angles = read_angles(arm)
        if angles is not None:
            last_good = angles
            history.append(angles)
            if len(history) > stable_reads:
                history.pop(0)
            if len(history) == stable_reads:
                spread = max(
                    max(abs(h[j] - history[0][j]) for h in history)
                    for j in range(6))
                if spread <= stable_deg:
                    return last_good
    return last_good


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

    Run this at several postures (arm folded, extended, mid) and on several
    joints -- gravity load varies hugely with pose, and a result that holds
    only in one posture is not a result."""
    start = read_angles(arm)
    if start is None:
        print("[probe] ERROR: could not read a valid starting pose", file=sys.stderr)
        return 1

    j = args.joint
    print("[probe] start pose (deg): {}".format([round(a, 2) for a in start]))
    print("[probe] testing joint {} (0-based), moving {:+.1f} deg at speed {}"
          .format(j, args.amplitude_deg, args.speed))
    print()

    target = list(start)
    target[j] = start[j] + args.amplitude_deg

    arm.send_angles(target, args.speed)
    settled = settle(arm, args, timeout_sec=args.settle_timeout_sec)
    if settled is None:
        print("[probe] ERROR: no valid reading after the initial move", file=sys.stderr)
        return 1

    residual = target[j] - settled[j]
    print("[probe] commanded {:.3f} deg, settled at {:.3f} deg"
          .format(target[j], settled[j]))
    print("[probe] residual e = {:+.4f} deg ({:+.5f} rad)"
          .format(residual, math.radians(residual)))
    print()

    if abs(residual) < args.deadzone_min_deg:
        print("[probe] residual is below --deadzone-min-deg ({} deg): this joint"
              .format(args.deadzone_min_deg))
        print("[probe] reached its target at this pose, so there is nothing to")
        print("[probe] correct here. Retry at a pose with more gravity load")
        print("[probe] (arm extended), or with a larger --amplitude-deg.")
        settle_and_write(arm, args, start, [])
        return 0

    print("[probe] --- overshoot staircase: commanding target + k*e ---")
    print("[probe] {:>3} {:>12} {:>12} {:>12} {:>10}"
          .format("k", "commanded", "settled", "error", "|err|/|e|"))
    rows = []
    for k in range(1, args.deadzone_steps + 1):
        biased = list(target)
        biased[j] = target[j] + k * residual
        arm.send_angles(biased, args.speed)
        result = settle(arm, args, timeout_sec=args.settle_timeout_sec)
        if result is None:
            print("[probe] ERROR: lost readings at k={}".format(k), file=sys.stderr)
            break
        err = target[j] - result[j]
        ratio = abs(err) / abs(residual) if residual else float("nan")
        print("[probe] {:>3} {:>12.3f} {:>12.3f} {:>+12.4f} {:>10.2f}"
              .format(k, biased[j], result[j], err, ratio))
        rows.append((time.time(), biased[j], result[j]))

    print()
    if rows:
        ratios = [abs(target[j] - r[2]) / abs(residual) for r in rows]
        if ratios[0] < 0.5:
            print("[probe] VERDICT: CORRECTABLE. Biasing the command by e cut the")
            print("[probe]   error to {:.0f}% of its original size. The joint responds"
                  .format(100 * ratios[0]))
            print("[probe]   to a biased command, so an outer-loop integrator or")
            print("[probe]   disturbance observer CAN close this out. GO for the")
            print("[probe]   control project.")
            if len(ratios) > 1 and ratios[-1] > ratios[0]:
                print("[probe]   (The rise at higher k is expected and confirms it --")
                print("[probe]   the joint tracks the bias proportionally, so it")
                print("[probe]   overshoots when over-biased.)")
        elif all(r > 0.8 for r in ratios):
            print("[probe] VERDICT: NOT CORRECTABLE by biasing. The error did not")
            print("[probe]   move at any k up to {}x. That is stiction or a dead"
                  .format(args.deadzone_steps))
            print("[probe]   band wider than the residual itself -- an integrator")
            print("[probe]   would wind up and limit-cycle. Needs dither, dead-zone")
            print("[probe]   inversion, or external metrology. NO-GO for a plain")
            print("[probe]   outer-loop PID on this joint.")
        else:
            print("[probe] VERDICT: mixed -- the error only responds past some k.")
            print("[probe]   The dead band lies between the last k that did nothing")
            print("[probe]   and the first that moved; read the |err|/|e| column.")
    print("[probe] ONE JOINT AT ONE POSE IS NOT A RESULT. Gravity load varies")
    print("[probe] hugely with posture -- repeat folded, extended and mid, on")
    print("[probe] joints 1, 2 and 3, before concluding anything.")

    settle_and_write(arm, args, start, rows)
    return 0


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
                      help="TEST 1: move the joint, measure the residual error, "
                           "then command target + k*e to see whether biasing the "
                           "command corrects it. Discriminates gravity droop "
                           "(outer-loop control works) from a servo dead zone "
                           "(it does not). This is the go/no-go.")

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
    parser.add_argument("--out", default=None,
                        help="CSV path for the commanded-vs-measured trace")
    args = parser.parse_args()

    if not 0 <= args.joint <= 5:
        parser.error("--joint must be 0-5 (arm joints only; this script "
                     "deliberately never touches the gripper)")

    arm = connect(args.serial_port, args.baud_rate, args.read_timeout)

    if args.deadzone:
        return mode_deadzone(arm, args)
    if args.probe:
        return mode_probe(arm, args)
    if args.stream:
        return mode_stream(arm, args)
    return mode_point_to_point(arm, args)


if __name__ == "__main__":
    sys.exit(main())
