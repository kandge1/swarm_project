#!/usr/bin/env python3
"""
Diagnostic: test which candidate downward-facing orientations are
reachable at the pick target, using CONSTRAINT-based IK (position sphere
+ orientation window) -- the same kind of search the real planner uses --
rather than exact-pose IK, which will almost always fail on a
non-redundant 6-DOF arm even for perfectly reasonable orientations.
Run with demo.launch.py already up. Does not move the arm.
"""

import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

GROUP_NAME = "arm_group"
POSE_LINK = "joint6_flange"
PLANNING_FRAME = "world"

TEST_X, TEST_Y, TEST_Z = 0.15, 0.04, 0.21
POSITION_TOLERANCE = 0.04       # 4cm sphere, matches pick_place.py
ORIENTATION_XY_TOLERANCE = 0.25  # ~14 deg on the "pointing down" axes
ORIENTATION_Z_TOLERANCE = 3.14   # yaw about approach axis: free


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


CANDIDATES = {
    "identity (no rotation)": (0.0, 0.0, 0.0, 1.0),
    "captured pose #1 (far pose)": (-0.500, 0.500, -0.500, 0.500),
    "captured pose #2 (near pose)": (-0.678, 0.685, -0.174, 0.200),
    "canonical: flange Z down, yaw=0": quat_from_rpy(math.pi, 0.0, 0.0),
    "canonical: flange Z down, yaw=90": quat_from_rpy(math.pi, 0.0, math.pi / 2),
    "canonical: flange X down (roll=90)": quat_from_rpy(math.pi / 2, 0.0, 0.0),
    "canonical: flange Y down (pitch=90)": quat_from_rpy(0.0, math.pi / 2, 0.0),
    "canonical: pitch=-90": quat_from_rpy(0.0, -math.pi / 2, 0.0),
}


def make_position_constraint(x, y, z, tolerance):
    c = PositionConstraint()
    c.header.frame_id = PLANNING_FRAME
    c.link_name = POSE_LINK
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [tolerance]
    c.constraint_region.primitives.append(primitive)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    c.constraint_region.primitive_poses.append(pose)
    c.weight = 1.0
    return c


def make_orientation_constraint(qx, qy, qz, qw):
    c = OrientationConstraint()
    c.header.frame_id = PLANNING_FRAME
    c.link_name = POSE_LINK
    c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = qx, qy, qz, qw
    c.absolute_x_axis_tolerance = ORIENTATION_XY_TOLERANCE
    c.absolute_y_axis_tolerance = ORIENTATION_XY_TOLERANCE
    c.absolute_z_axis_tolerance = ORIENTATION_Z_TOLERANCE
    c.weight = 1.0
    return c


class IKProbe(Node):
    def __init__(self):
        super().__init__("ik_probe")
        self.client = self.create_client(GetPositionIK, "/compute_ik")

    def try_ik(self, qx, qy, qz, qw):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik service not available")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2

        # A pose_stamped is still required by the message even when using
        # constraints -- give it the nominal center point, constraints do
        # the real work of defining the search region.
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = PLANNING_FRAME
        req.ik_request.pose_stamped.pose.position.x = TEST_X
        req.ik_request.pose_stamped.pose.position.y = TEST_Y
        req.ik_request.pose_stamped.pose.position.z = TEST_Z
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        req.ik_request.ik_link_name = POSE_LINK

        constraints = Constraints()
        constraints.position_constraints.append(
            make_position_constraint(TEST_X, TEST_Y, TEST_Z, POSITION_TOLERANCE)
        )
        constraints.orientation_constraints.append(
            make_orientation_constraint(qx, qy, qz, qw)
        )
        req.ik_request.constraints = constraints

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def try_ik_position_only(self):
        """IK with only a position constraint -- orientation completely free."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik service not available")
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2

        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = PLANNING_FRAME
        req.ik_request.pose_stamped.pose.position.x = TEST_X
        req.ik_request.pose_stamped.pose.position.y = TEST_Y
        req.ik_request.pose_stamped.pose.position.z = TEST_Z
        req.ik_request.pose_stamped.pose.orientation.w = 1.0
        req.ik_request.ik_link_name = POSE_LINK

        constraints = Constraints()
        constraints.position_constraints.append(
            make_position_constraint(TEST_X, TEST_Y, TEST_Z, POSITION_TOLERANCE)
        )
        # No orientation_constraints appended -- fully free
        req.ik_request.constraints = constraints

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    probe = IKProbe()

    print(f"\nTesting CONSTRAINT-based IK reachability near "
          f"({TEST_X}, {TEST_Y}, {TEST_Z}) for link '{POSE_LINK}'")
    print(f"Position tolerance: {POSITION_TOLERANCE*100:.0f}cm, "
          f"orientation x/y tolerance: {math.degrees(ORIENTATION_XY_TOLERANCE):.0f} deg, "
          f"yaw: free\n")

    results = []
    for name, (qx, qy, qz, qw) in CANDIDATES.items():
        response = probe.try_ik(qx, qy, qz, qw)
        if response is None:
            print(f"  {name:45s} -> SERVICE UNAVAILABLE")
            continue
        success = response.error_code.val == 1
        status = "REACHABLE" if success else f"unreachable (code {response.error_code.val})"
        print(f"  {name:45s} -> {status}   quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
        if success:
            js = response.solution.joint_state
            print(f"      solution joints: "
                  f"{dict(zip(js.name, [round(p, 3) for p in js.position]))}")
        results.append((name, success, (qx, qy, qz, qw)))

    reachable = [r for r in results if r[1]]
    print(f"\n{len(reachable)} of {len(results)} candidates reachable.")
    if reachable:
        print("Use one of the REACHABLE quaternions above as your GRASP_QX/QY/QZ/QW.")
    else:
        print("Still none reachable even with tolerance -- the target region "
              "itself is likely outside a downward-approach zone for this arm. "
              "Try a position closer to the base or at a different height.")

    # ---- Free-orientation sanity check ----
    print("\n--- Free-orientation IK (no orientation constraint at all) ---")
    print("This shows what orientation the arm naturally lands on here, and")
    print("whether the wrist joints are pinned at their limits.\n")
    free_req_result = probe.try_ik_position_only()
    if free_req_result is not None and free_req_result.error_code.val == 1:
        js = free_req_result.solution.joint_state
        print("Free-orientation solution FOUND. Joint values:")
        for name, pos in zip(js.name, js.position):
            print(f"    {name:30s} = {pos:.4f} rad ({math.degrees(pos):.1f} deg)")
        print("\nCompare the wrist joints (joint5_to_joint4, joint6_to_joint5,")
        print("joint6output_to_joint6) above against their limits from")
        print("joint_limits.yaml -- if any are at/near their min or max, that")
        print("joint's range is likely the reason no downward orientation works.")
    else:
        print("Free-orientation IK also failed -- position itself may be out of reach.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()