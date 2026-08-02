#!/usr/bin/env python3
"""
ff_verify.py -- measure whether mycobot_bridge.py's gravity feedforward
actually removes the droop it predicts.

RUN THIS ON MARS. It needs Terminal 1 (robot hardware) and Terminal 2 (mars
planning) up, and nothing else -- no vision, no move_group planning, no IK.
It sends ONE single-waypoint trajectory straight to arm_group_controller,
exactly like joint_trajectory_test.py, so the only thing between the commanded
angle and the servo is the bridge -- which is the thing under test.

WHY THIS EXISTS AS A SEPARATE SCRIPT. The feedforward's entire claim is "the
joint lands ON its target instead of short of it", and until now nothing in the
project printed commanded-vs-achieved per joint. The 4.19 -> 0.50 deg tilt
numbers in pick_place.py were derived offline from logged joint states. Testing
a correction by eye, or by whether a grasp happens to succeed, cannot separate
"the model is right" from "the model is wrong and something else moved".

DO NOT test the feedforward with serial_rate_probe.py. That script opens
/dev/ttyAMA0 directly and REQUIRES the bridge to be stopped, so it bypasses
mycobot_bridge.py completely and can never see the feedforward at all. It is
the instrument that MEASURED the droop, not the one that verifies the fix.

THE A/B, which is the whole point:

    # on the ROBOT: GRAVITY_FF_ENABLED = False in mycobot_bridge.py, relaunch
    python3 ff_verify.py --label ff-off

    # on the ROBOT: GRAVITY_FF_ENABLED = True, relaunch Terminal 1
    python3 ff_verify.py --label ff-on

The bridge reads its constants at import, so Terminal 1 MUST be restarted
between the two runs. Nothing warns you if you forget; the numbers just come
out identical, which is itself the tell.

WHAT SUCCESS LOOKS LIKE. The predicted droop column is what the model says each
joint gives up to gravity at this pose. With the feedforward off, the measured
error should roughly match it. With it on, the error on joints 1, 2 and 3
should collapse toward zero and the other three should not move. Anything else
-- errors unchanged, or every joint improving equally -- means something other
than the feedforward changed.

The residual will NOT be zero. The feedforward corrects the SYMMETRIC half of
the joint error (gravity, which does not care which way you drove in). The
ANTISYMMETRIC half -- friction, dead zone, lost motion -- is 0.4 to 0.9 deg per
pitch joint, reverses with approach direction, and is untouched by any
feedforward. Judge this by the CHANGE, not by the remainder.
"""

import argparse
import math
import os
import sys
import time

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pick_place import RobotIOClient, HOME_RADIANS  # noqa: E402

ARM_JOINT_NAMES = list(HOME_RADIANS.keys())

# The IK solution the grasp actually uses (pick_place.py records it as the
# chosen solution, identical before and after the IK-tolerance experiment).
# This is the pose the droop matters at, so it is the pose to test at.
GRASP_RADIANS = [1.828, -0.746, -0.605, -0.22, -0.0, 1.828]

# Must match mycobot_bridge.py's GRAVITY_FF_COEFFS. Duplicated rather than
# imported because that file lives in a different package and only exists on
# the robot's install tree; a stale copy here misreports the PREDICTION column
# only, never the measurement, so the A/B stays valid either way.
FF_COEFFS = {1: (-0.072, -5.47), 2: (+0.006, -4.59), 3: (-0.062, -13.16)}

# URDF chain for the moment-arm calculation, same constants as the bridge's
# _FK_CHAIN. (xyz, rpy, axis); axis None = fixed joint.
_FK_CHAIN = [
    ([0.0, 0.0, 0.13956], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 0.0, -0.001], [0.0, 1.5708, -1.5708], [0.0, 0.0, 1.0]),
    ([-0.1104, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([-0.096, 0.0, 0.06462], [0.0, 0.0, -1.5708], [0.0, 0.0, 1.0]),
    ([0.0, -0.07318, -0.001], [1.5708, -1.5708, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 0.0456, 0.0], [-1.5708, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 0.0, 0.01], [1.579, 0.0, 2.3562], None),
    ([0.0, 0.04, 0.0], [-0.0082, 1.5708, 0.0], None),
]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _origin_matrix(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
            [-sp, cp * sr, cp * cr, xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _axis_rotation(axis, theta):
    ax, ay, az = axis
    c, s, t = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
    return [[t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay, 0.0],
            [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax, 0.0],
            [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def forward(positions_rad):
    """(moment arms, flange z-axis, tool point) for one joint configuration."""
    transform = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    origins, axes = [], []
    # The six revolute joints, then the fixed tool chain ONCE. Splitting the
    # loop rather than skipping fixed entries inside it: the skip-and-replay
    # version applied the tool transforms twice and put the tool 43 mm out,
    # which reads as a plausible pose and silently corrupts every arm.
    for index in range(6):
        xyz, rpy, axis = _FK_CHAIN[index]
        transform = _mat_mul(transform, _origin_matrix(xyz, rpy))
        origins.append([transform[i][3] for i in range(3)])
        axes.append([sum(transform[i][k] * axis[k] for k in range(3))
                     for i in range(3)])
        transform = _mat_mul(transform, _axis_rotation(axis, positions_rad[index]))
    flange_z = [transform[i][2] for i in range(3)]
    for xyz, rpy, _axis in _FK_CHAIN[6:]:
        transform = _mat_mul(transform, _origin_matrix(xyz, rpy))
    tool = [transform[i][3] for i in range(3)]
    arms = []
    for origin, axis in zip(origins, axes):
        rx, ry, _rz = (tool[i] - origin[i] for i in range(3))
        arms.append(axis[0] * -ry + axis[1] * rx)
    return arms, flange_z, tool


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--degrees", nargs=6, type=float, default=None,
                        help="target pose, six joint angles in degrees "
                             "(default: the grasp IK solution)")
    parser.add_argument("--label", default="run",
                        help="tag for this run, e.g. ff-off / ff-on")
    parser.add_argument("--duration-sec", type=float, default=6.0,
                        help="trajectory duration (default %(default)s)")
    parser.add_argument("--settle-sec", type=float, default=3.0,
                        help="wait after the controller reports success before "
                             "reading /joint_states. The controller reports from "
                             "elapsed time alone, so it says 'done' before the "
                             "servos have stopped (default %(default)s)")
    parser.add_argument("--from-below", action="store_true",
                        help="approach the target from BELOW on every joint "
                             "(pre-move -8 deg first). Backlash is direction "
                             "dependent and un-correctable, so an A/B whose two "
                             "runs arrive from different sides is not comparing "
                             "the same thing. Use the same choice for both runs")
    args = parser.parse_args()

    target = ([math.radians(d) for d in args.degrees] if args.degrees
              else list(GRASP_RADIANS))

    rclpy.init()
    io_client = RobotIOClient()
    try:
        io_client.wait_for_joint_states()

        if args.from_below:
            pre = [t - math.radians(8.0) for t in target]
            print(f"[ff_verify] pre-move 8 deg below target, so the final "
                  f"approach is unidirectional...")
            io_client.arm_execute(_trajectory(pre, args.duration_sec))
            time.sleep(args.settle_sec)

        print(f"[ff_verify] [{args.label}] commanding target over "
              f"{args.duration_sec}s...")
        ok = io_client.arm_execute(_trajectory(target, args.duration_sec))
        if not ok:
            print("[ff_verify] controller REJECTED the goal -- nothing measured.")
            return 1
        print(f"[ff_verify] controller reported success; waiting "
              f"{args.settle_sec}s for the servos to actually stop.")
        time.sleep(args.settle_sec)
        # Spin so the post-settle /joint_states actually lands before reading.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(io_client, timeout_sec=0.05)

        achieved_map = io_client.current_joint_positions(ARM_JOINT_NAMES)
        achieved = [achieved_map[n] for n in ARM_JOINT_NAMES]
        report(args.label, target, achieved)
    finally:
        io_client.destroy_node()
        rclpy.shutdown()
    return 0


def _trajectory(target_radians, duration_sec):
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = list(target_radians)
    point.velocities = [0.0] * len(ARM_JOINT_NAMES)
    point.time_from_start.sec = int(duration_sec)
    point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
    traj.points = [point]
    return traj


def report(label, target, achieved):
    arms, _z, _t = forward(target)
    print()
    print("=" * 74)
    print(f"[ff_verify] RESULT  [{label}]")
    print("=" * 74)
    print("{:<24} {:>9} {:>9} {:>9} {:>11}".format(
        "joint", "target", "achieved", "error", "predicted"))
    for i, name in enumerate(ARM_JOINT_NAMES):
        tdeg = math.degrees(target[i])
        adeg = math.degrees(achieved[i])
        err = tdeg - adeg
        if i in FF_COEFFS:
            intercept, slope = FF_COEFFS[i]
            pred = "{:+.2f}".format(intercept + slope * arms[i])
        else:
            pred = "-"
        print("{:<24} {:>+9.2f} {:>+9.2f} {:>+9.2f} {:>11}".format(
            name, tdeg, adeg, err, pred))

    # Flange tilt and tool displacement: what the joint errors cost in the
    # place the grasp actually cares about.
    _a1, z_cmd, tool_cmd = forward(target)
    _a2, z_act, tool_act = forward(achieved)
    dot = max(-1.0, min(1.0, sum(z_cmd[i] * z_act[i] for i in range(3))))
    tilt = math.degrees(math.acos(dot))
    d = [tool_act[i] - tool_cmd[i] for i in range(3)]
    print()
    print("[ff_verify] flange tilt vs commanded : {:.2f} deg".format(tilt))
    print("[ff_verify] jaw displacement         : dx {:+.1f}  dy {:+.1f}  "
          "dz {:+.1f} mm".format(*[1000 * v for v in d]))
    print("[ff_verify] total jaw position error : {:.1f} mm".format(
        1000 * math.sqrt(sum(v * v for v in d))))
    print()
    print("[ff_verify] Run again with the feedforward toggled and compare. "
          "Joints 1-3 should improve; 0, 4, 5 should not move.")
    print("[ff_verify] ROS reporting success is not evidence of motion on this "
          "setup -- confirm the arm physically moved.")


if __name__ == "__main__":
    sys.exit(main())
