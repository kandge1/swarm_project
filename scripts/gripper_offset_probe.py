#!/usr/bin/env python3
"""
gripper_offset_probe.py -- print every gripper_* link's offset from
joint6_flange in one pass, so the real fingertip (TCP) depth can be read off
directly instead of guessing from link names or querying tf2_echo one link
at a time.

WHY THIS EXISTS: tf2_echo joint6_flange gripper_base and tf2_echo
joint6_flange gripper_left1 gave two different z-offsets (0.034 m, 0.055 m)
because gripper_base is the mounting plate and gripper_left1 is a proximal
finger segment, not the pad that actually contacts an object. The frame
graph (view_frames) lists gripper_left1/2/3 and gripper_right1/2/3 -- the
highest-numbered link in each chain is the most likely candidate for "the
part that actually touches the block," but that's a guess from naming, not
geometry. This prints all of them, sorted by how far along the flange's
local +z axis they sit (which, under the confirmed downward grasp
orientation, is world -z -- i.e. depth toward the floor), so the real
maximum reach is visible at a glance instead of inferred.

Usage:
    python3 gripper_offset_probe.py
    python3 gripper_offset_probe.py --root joint6_flange --prefix gripper_
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


CANDIDATE_LINKS = [
    "gripper_base",
    "gripper_left1", "gripper_left2", "gripper_left3",
    "gripper_right1", "gripper_right2", "gripper_right3",
]


class OffsetProbe(Node):
    def __init__(self, root, links, timeout_sec):
        super().__init__("gripper_offset_probe")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.root = root
        self.links = links
        self.timeout_sec = timeout_sec

    def run(self):
        results = []
        for link in self.links:
            try:
                # Give TF a moment to populate on first lookup.
                t = self.buffer.lookup_transform(
                    self.root, link, Time(),
                    timeout=rclpy.duration.Duration(seconds=self.timeout_sec))
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                print(f"  {link:16s}  UNAVAILABLE ({exc})")
                continue

            tr = t.transform.translation
            depth = tr.z  # along flange local +z == world -z under the grasp orientation
            lateral = math.hypot(tr.x, tr.y)
            results.append((link, tr.x, tr.y, tr.z, depth, lateral))

        if not results:
            print("\n  No gripper links resolved. Is the sim / robot_state_publisher"
                 " running?")
            return

        print(f"\n  {'link':16s} {'x':>8s} {'y':>8s} {'z (depth)':>10s} {'lateral':>8s}")
        for link, x, y, z, depth, lateral in sorted(results, key=lambda r: -r[4]):
            print(f"  {link:16s} {x:+8.4f} {y:+8.4f} {z:+10.4f} {lateral:8.4f}")

        deepest = max(results, key=lambda r: r[4])
        print(f"\n  Deepest link along flange +z: '{deepest[0]}' at z={deepest[3]:.4f} m")
        print("  Under the confirmed downward grasp orientation this is how far")
        print("  BELOW the commanded joint6_flange position the gripper hardware")
        print("  physically extends. Use it as the flange-to-TCP offset:")
        print(f"\n      flange_target_z = block_contact_z + {deepest[3]:.4f}")
        print("\n  Caveat: this is still a rigid link's origin, not necessarily the")
        print("  exact rubber-pad contact point if the URDF has extra geometry")
        print("  past the last named frame. Treat it as a lower bound on the true")
        print("  offset and sanity-check by eye once you can watch the arm move.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="joint6_flange")
    parser.add_argument("--links", nargs="+", default=CANDIDATE_LINKS)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = OffsetProbe(args.root, args.links, args.timeout)
    # Let TF buffer fill briefly before the first lookup.
    rclpy.spin_once(node, timeout_sec=1.0)
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()