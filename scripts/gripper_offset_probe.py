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
            t = self._lookup_with_retry(self.root, link, self.timeout_sec)
            if t is None:
                print(f"  {link:16s}  UNAVAILABLE (no transform within "
                     f"{self.timeout_sec:.1f}s)")
                continue

            tr = t.transform.translation
            depth = tr.z  # along flange local +z == world -z under the grasp orientation
            lateral = math.hypot(tr.x, tr.y)
            results.append((link, tr.x, tr.y, tr.z, depth, lateral))

        if not results:
            print("\n  No gripper links resolved. Is the sim / robot_state_publisher"
                 " running? (ros2 node list should show robot_state_publisher)")
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

    def _lookup_with_retry(self, root, link, timeout_sec, poll_sec=0.1):
        """Poll for a transform rather than doing a single blocking lookup.

        Buffer.lookup_transform's own `timeout=` argument only unblocks early
        if something ELSE is spinning this node concurrently to feed the
        TransformListener's subscription callbacks. Nothing was doing that
        here (no executor, no background thread), so the previous version's
        single spin_once() + one blocking lookup call raced /tf_static and
        lost, then threw 'does not exist' for every single link at once --
        the same startup race tf2_echo hit earlier, just with no retry to
        recover from it. This explicitly alternates spinning and lookup
        attempts instead.
        """
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        last_exc = None
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(self, timeout_sec=poll_sec)
            try:
                return self.buffer.lookup_transform(root, link, Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                last_exc = exc
                continue
        if last_exc is not None:
            self.get_logger().debug(f"{root}->{link}: {last_exc}")
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="joint6_flange")
    parser.add_argument("--links", nargs="+", default=CANDIDATE_LINKS)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = OffsetProbe(args.root, args.links, args.timeout)
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