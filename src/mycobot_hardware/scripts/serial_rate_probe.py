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
import math
import statistics
import sys
import time

DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD_RATE = 1000000


def connect(port, baud):
    from pymycobot import MyCobot280
    print("[probe] connecting to {} @ {}...".format(port, baud))
    arm = MyCobot280(port, baud)
    print("[probe] connected.")
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
    parser.add_argument("--out", default=None,
                        help="CSV path for the commanded-vs-measured trace")
    args = parser.parse_args()

    if not 0 <= args.joint <= 5:
        parser.error("--joint must be 0-5 (arm joints only; this script "
                     "deliberately never touches the gripper)")

    arm = connect(args.serial_port, args.baud_rate)

    if args.probe:
        return mode_probe(arm, args)
    if args.stream:
        return mode_stream(arm, args)
    return mode_point_to_point(arm, args)


if __name__ == "__main__":
    sys.exit(main())
