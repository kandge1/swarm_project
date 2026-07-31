#!/usr/bin/env python3
"""Find the camera's usable focus range by sweeping hover height.

RUNS ON MARS. Needs camera.launch.py + block_detector_node.py on the Pi.

    python3 camera_focus_sweep.py --zone-origin 0.0 0.254 0.0

WHY
---
2026-07-30: the detector reported 0 AprilTags of ANY id while pointed squarely
at the mat, with exposure healthy (mean 121-133, range 16-236) but a Laplacian
focus metric of 84 and 44. Sharp 640x480 frames score in the high hundreds or
thousands. The tags were in frame; the picture was too blurred to decode them.

Two causes, needing opposite fixes:

  DEFOCUS      cheap fixed-focus USB webcams often have a minimum focus distance
               around 300 mm. The lens sits ~189 mm above the mat at
               MAX_HOVER_Z, which would put the entire mat inside the blur zone.
  MOTION BLUR  the arm still ringing when the frame is grabbed.

They are trivially separable, which is the entire point of this script: DEFOCUS
varies smoothly and monotonically with height, MOTION BLUR does not care about
height at all. Sweep the height, print the focus metric, read the shape.

It also settles a Stage 0b question that has been open since the beginning: the
camera's minimum working distance, which together with the FOV decides what
hover heights are usable at all. Both constraints happen to push the same way --
FOV wants height to see the whole mat, focus wants distance to be sharp -- so
the answer is likely "hover higher and further back", not "hover lower".

READING THE OUTPUT
------------------
  focus climbing steadily with height  -> defocus. The height where it plateaus
                                          is the minimum working distance.
  focus flat/noisy across all heights  -> not distance. Suspect motion blur;
                                          re-run with a larger --settle.
  focus high everywhere but no tags    -> neither. The tags are not where the
                                          camera is pointing after all.
"""
import argparse
import math
import os
import re
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pick_place import (  # noqa: E402
    GRIPPER_OPEN,
    RobotIOClient,
    go_home,
    move_arm_to,
)
from tag_pick_place import Detector, look_at_quat  # noqa: E402


def parse_metric(message, key):
    """Pull 'key <number>' out of the detector's diagnostic string."""
    m = re.search(r"%s (-?\d+(?:\.\d+)?)" % key, message or "")
    return float(m.group(1)) if m else None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone-origin", type=float, nargs=3,
                        metavar=("X", "Y", "Z"), required=True)
    parser.add_argument("--zone-yaw", type=float, default=0.0)
    parser.add_argument("--zone-size", type=float, default=0.1524)
    parser.add_argument("--heights", type=float, nargs="+",
                        default=[0.150, 0.180, 0.205, 0.240, 0.280, 0.320],
                        help="flange heights to try, metres")
    parser.add_argument("--flange-y", type=float, default=None,
                        help="flange Y (default: zone centre). Higher hovers may "
                             "need this pulled IN to stay reachable.")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="seconds to wait after each move before the still. "
                             "Raise to rule out motion blur.")
    parser.add_argument("--yaw", type=float, default=180.0,
                        help="wrist yaw; 180 puts the lens outboard")
    parser.add_argument("--free-orientation", action="store_true",
                        help="DANGEROUS, kept only for the record. Drops the "
                             "orientation constraint entirely rather than "
                             "aiming it. Tried 2026-07-30: OMPL is then free to "
                             "satisfy position alone, and picked a mirror-"
                             "configuration solve with the base swung 180 deg, "
                             "arm reaching back over itself. Sharp frames, "
                             "camera pointed at the wall. Use --look-at instead.")
    parser.add_argument("--look-at", action="store_true",
                        help="Aim the lens at the zone centre instead of "
                             "straight down (tag_pick_place.look_at_quat). This "
                             "is what actually solves the reach-vs-focus "
                             "conflict: it frees only the tilt detection never "
                             "needed, verified offline to 0.37mm aiming error "
                             "across the whole height sweep. Overrides --yaw "
                             "and --free-orientation.")
    args = parser.parse_args()

    zone_x, zone_y, zone_z = args.zone_origin
    flange_y = args.flange_y if args.flange_y is not None else zone_y

    rclpy.init()
    io_client = RobotIOClient()
    detector = Detector(io_client, zone_x, zone_y, zone_z,
                        math.radians(args.zone_yaw), args.zone_size)

    rows = []
    try:
        if not io_client._arm_client.wait_for_server(timeout_sec=10.0):
            print("arm action server not available -- is this running on MARS, "
                  "with both launch files up?")
            return 1
        if not go_home(io_client):
            return 1
        io_client.gripper_move_to(GRIPPER_OPEN)

        for z in args.heights:
            print("\n=== flange z = %.3f m ===" % z)
            move_kwargs = {}
            if args.look_at:
                target = (zone_x, zone_y, zone_z)
                q = look_at_quat((zone_x, flange_y, z), target)
                move_kwargs["orientation_override"] = q
                print("  [look-at] aiming lens at zone centre (%.3f, %.3f, %.3f)"
                      % target)
            else:
                move_kwargs["block_yaw_deg"] = args.yaw
                move_kwargs["lock_orientation"] = not args.free_orientation
            if not move_arm_to(io_client, zone_x, flange_y, z, **move_kwargs):
                print("  unreachable at this height -- skipping")
                rows.append((z, None, None, None))
                continue
            time.sleep(args.settle)

            response = detector.detect(
                "pickup", debug_image="/tmp/focus_%03d.png" % round(z * 1000))
            if response is None:
                # detect() returns None on failure, but the diagnostics live in
                # the message, which it already printed. Re-read via a raw call.
                rows.append((z, None, None, None))
                continue
            rows.append((z, None, None, response.tags_seen))

        go_home(io_client)
    finally:
        io_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("\n" + "=" * 62)
    print("Read the 'focus NNN' value printed in each [detect] line above.")
    print("Frames saved on the Pi as /tmp/focus_<height_mm>.png")
    print()
    print("  climbing with height   -> DEFOCUS; plateau = min working distance")
    print("  flat and low           -> MOTION BLUR; re-run with --settle 3.0")
    print("  high but still no tags -> neither; the aim is wrong, not the image")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())
