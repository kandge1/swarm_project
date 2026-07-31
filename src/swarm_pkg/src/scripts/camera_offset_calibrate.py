#!/usr/bin/env python3
"""MEASURE the flange->lens offset, instead of trusting the URDF for it.

RUNS ON MARS. Needs the Pi's camera + block_detector_node, and move_group here.

    python3 camera_offset_calibrate.py --zone-origin 0.0 0.254 0.0

WHY THIS EXISTS
---------------
The URDF's camera transform is not trustworthy, and it says so itself:

    <!-- lens is ~20mm along +X (lateral) and ~8.5mm along +Z ... -->
    <origin xyz="0.0400 0.0085 0 " rpy="-1.5708 0 1.5708"/>

The comment says 20 mm along X and 8.5 along Z; the values say 40 mm along X and
8.5 along Y. They disagree on the magnitude AND the axis, which means nobody ever
checked this against the hardware. Hardware on 2026-07-30 then showed the lens is
also on the OPPOSITE side of the flange from where that transform puts it (the
survey saw 1 of 4 tags; the tag angles only fit the flipped model).

So rather than guess a sign and a magnitude, measure both.

THE METHOD, AND WHY IT IS IMMUNE TO THE ARM'S OWN ERROR
-------------------------------------------------------
The obvious approach -- command the flange somewhere, see where the lens ended
up, subtract -- does not work on this robot. It measures

    lens_actual - flange_COMMANDED

which contains the arm's positioning error, and this arm's worn servo gearing
means the commanded and actual flange positions differ by millimetres in a way
the encoders cannot see (APRIL_TAGS.md, ROOT CAUSE). The offset and the arm
error would be inseparable.

The fix is to take TWO stills at the SAME commanded flange position, with the
wrist rotated 180 degrees between them. Writing f for wherever the flange really
went and d for the lateral offset in world:

    lens_A = f + d
    lens_B = f - d          (180 deg about the flange axis negates the lateral part)

    => d = (lens_A - lens_B) / 2      <-- f CANCELS. The arm's error drops out.
    => f = (lens_A + lens_B) / 2      <-- and this is a free bonus: where the
                                          flange ACTUALLY was, externally
                                          measured, owing nothing to encoders.

Both lens positions come from the tag homography (camera_zx/zy), which is an
external measurement independent of the arm entirely. So the offset falls out of
two numbers that the arm cannot corrupt.

CALIBRATION POSE
----------------
Default flange target is the zone centre itself. That is deliberate: with a
~40 mm offset the two stills then land the lens ~40 mm either side of the centre,
so each one is looking squarely at one edge's pair of tags rather than at the
zone corners. Both stills see their 2 tags comfortably inside the frame, which
is exactly the framing that the centre-aimed survey fails to get.
"""
import argparse
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tool_frame_check  # noqa: E402
from pick_place import (  # noqa: E402
    GRIPPER_OPEN,
    MAX_HOVER_Z,
    RobotIOClient,
    go_home,
    move_arm_to,
)
from tag_pick_place import (  # noqa: E402
    SETTLE_AFTER_MOVE_SEC,
    Detector,
    quat_to_matrix,
)

import time  # noqa: E402


def measure_lens(io_client, detector, x, y, z, yaw_deg, label, debug_path=None):
    """Move the flange to (x, y, z) at yaw_deg and return the lens position in
    world metres, measured from the tags. None if the still was unusable."""
    print("\n=== %s: flange (%.4f, %.4f, %.4f) at wrist yaw %+.1f deg ==="
          % (label, x, y, z, yaw_deg))
    if not move_arm_to(io_client, x, y, z, block_yaw_deg=yaw_deg):
        print("[%s] move failed" % label)
        return None
    time.sleep(SETTLE_AFTER_MOVE_SEC)

    response = detector.detect("pickup", debug_image=debug_path)
    if response is None:
        print("[%s] no usable homography -- the lens is probably not over the "
              "mat at this yaw. Try --flange-y nearer the zone." % label)
        return None

    wx, wy = detector.zone_to_world(response.camera_zx, response.camera_zy)
    print("[%s] lens measured at zone (%+.1f, %+.1f) mm -> world (%.4f, %.4f)"
          % (label, response.camera_zx * 1000, response.camera_zy * 1000, wx, wy))
    return (wx, wy)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone-origin", type=float, nargs=3,
                        metavar=("X", "Y", "Z"), required=True,
                        help="surveyed zone centre in world metres")
    parser.add_argument("--zone-yaw", type=float, default=0.0,
                        help="zone rotation about +Z, degrees")
    parser.add_argument("--zone-size", type=float, default=0.1524)
    parser.add_argument("--flange-x", type=float, default=None,
                        help="flange X for both stills (default: zone centre X)")
    parser.add_argument("--flange-y", type=float, default=None,
                        help="flange Y for both stills (default: zone centre Y)")
    parser.add_argument("--hover-z", type=float, default=MAX_HOVER_Z)
    parser.add_argument("--yaw", type=float, default=0.0,
                        help="first still's wrist yaw; the second is this +180")
    args = parser.parse_args()

    zone_x, zone_y, zone_z = args.zone_origin
    flange_x = args.flange_x if args.flange_x is not None else zone_x
    flange_y = args.flange_y if args.flange_y is not None else zone_y

    rclpy.init()
    io_client = RobotIOClient()
    detector = Detector(io_client, zone_x, zone_y, zone_z,
                        math.radians(args.zone_yaw), args.zone_size)

    try:
        # Preflight, because the failure mode otherwise is an rclpy traceback
        # that says nothing about the actual cause. Running this ON THE ROBOT is
        # the easy mistake: it needs move_group for IK (mars only), and this
        # Galactic Cyclone DDS build's same-host discovery is unreliable anyway
        # -- that is why real_robot_hardware.launch.py wraps its controller
        # spawners in a retry loop. Cross-host from mars is the reliable path.
        if not io_client._arm_client.wait_for_server(timeout_sec=10.0):
            print("\n" + "=" * 70)
            print("The arm action server never appeared. Almost always one of:")
            print("  1. This is running ON THE ROBOT. It must run on MARS --")
            print("     it needs move_group for IK, which only runs there.")
            print("  2. real_robot_hardware.launch.py is not up on the robot,")
            print("     or its controller spawners failed (check for")
            print("     'retrying_spawner' errors in that terminal).")
            print("  3. real_robot_planning.launch.py is not up on mars.")
            print("=" * 70)
            return 1

        if not go_home(io_client):
            return 1
        io_client.gripper_move_to(GRIPPER_OPEN)

        # Always save the frames. If the measurement fails, these are the only
        # thing that says why, and re-running to get them costs another cycle.
        a = measure_lens(io_client, detector, flange_x, flange_y, args.hover_z,
                         args.yaw, "still A", debug_path="/tmp/calib_A.png")
        b = measure_lens(io_client, detector, flange_x, flange_y, args.hover_z,
                         args.yaw + 180.0, "still B", debug_path="/tmp/calib_B.png")
        go_home(io_client)

        if a is None or b is None:
            print("\nCALIBRATION FAILED: need BOTH stills.")
            print("Frames were saved ON THE PI as /tmp/calib_A.png and")
            print("/tmp/calib_B.png -- the detector's diagnostics above say")
            print("whether the mat was out of frame, out of focus, or badly")
            print("exposed. Those need different fixes, so check before")
            print("moving --flange-y on a guess.")
            return 1

        # The arm's own error cancels here -- see the module docstring.
        dx = (a[0] - b[0]) / 2.0
        dy = (a[1] - b[1]) / 2.0
        fx = (a[0] + b[0]) / 2.0
        fy = (a[1] + b[1]) / 2.0

        lateral = math.hypot(dx, dy)
        print("\n" + "=" * 70)
        print("MEASURED flange->lens LATERAL offset, in WORLD at yaw %+.1f:" % args.yaw)
        print("   (%+.4f, %+.4f) m   magnitude %.1f mm" % (dx, dy, lateral * 1000))
        print()
        print("MEASURED actual flange position (free, and encoder-independent):")
        print("   (%.4f, %.4f) m   vs commanded (%.4f, %.4f)"
              % (fx, fy, flange_x, flange_y))
        print("   arm positioning error: (%+.1f, %+.1f) mm, %.1f mm total"
              % ((fx - flange_x) * 1000, (fy - flange_y) * 1000,
                 math.hypot(fx - flange_x, fy - flange_y) * 1000))

        # Compare against what the URDF currently claims.
        urdf_offset, _view = tool_frame_check.flange_to_camera()
        urdf_lateral = math.hypot(urdf_offset[0], urdf_offset[1])
        print()
        print("URDF currently claims %.1f mm lateral; measured %.1f mm."
              % (urdf_lateral * 1000, lateral * 1000))
        if urdf_lateral > 1e-6:
            print("   ratio measured/URDF = %.3f" % (lateral / urdf_lateral))

        # Rotate the measured world offset back into the flange frame so it can
        # be written into the URDF directly.
        from pick_place import grasp_quat_for
        R = quat_to_matrix(grasp_quat_for(args.yaw, flange_x, flange_y))
        # world = R @ flange  =>  flange = R^T @ world
        world_vec = (dx, dy, 0.0)
        flange_vec = [sum(R[k][i] * world_vec[k] for k in range(3))
                      for i in range(3)]
        print()
        print("Same offset expressed in the FLANGE frame (what the URDF needs):")
        print("   [%+.4f, %+.4f, %+.4f] m" % tuple(flange_vec))
        print()
        print("NEXT: put this into camera_flange_to_camera_link's origin in")
        print("      mycobot_280_pi_camera_flange_plus_gripper_unchanged_transforms.urdf,")
        print("      rebuild mycobot_description on BOTH machines, then set")
        print("      CAMERA_MOUNT_FLIPPED = False in tag_pick_place.py -- the")
        print("      code-side flip exists only until the model is right.")
        print("=" * 70)
        return 0
    finally:
        io_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
