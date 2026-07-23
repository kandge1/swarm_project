#!/usr/bin/env python3
"""Read-only reachability probe (does NOT move the arm).

For the pick column (0, +0.25, z) and place column (0, -0.25, z), sweep
height z and, at each height, find the SMALLEST orientation tolerance at
which the fixed downward grasp orientation becomes reachable via /compute_ik.
Also does a fully-free-orientation position check to confirm the position
itself is reachable at all. Tells us whether straight-down is achievable at
the hover heights or only lower on the column.
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

# Exact downward grasp quaternion used by pick_place.py (GRIPPER_YAW_DEG=0).
QX, QY, QZ, QW = -0.7071, 0.7071, 0.0, 0.0

POS_TOL = 0.03  # position sphere radius (m)
ORI_TOLS = [0.05, 0.10, 0.15, 0.25, 0.40, 0.60, 0.90]  # rad, tried smallest first
COLUMNS = [("pick  y=+0.25", 0.0, 0.25), ("place y=-0.25", 0.0, -0.25)]
HEIGHTS = [0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28]


def pos_constraint(x, y, z, tol):
    c = PositionConstraint()
    c.header.frame_id = PLANNING_FRAME
    c.link_name = POSE_LINK
    p = SolidPrimitive()
    p.type = SolidPrimitive.SPHERE
    p.dimensions = [tol]
    c.constraint_region.primitives.append(p)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    c.constraint_region.primitive_poses.append(pose)
    c.weight = 1.0
    return c


def ori_constraint(tol_xy, tol_z):
    c = OrientationConstraint()
    c.header.frame_id = PLANNING_FRAME
    c.link_name = POSE_LINK
    c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = QX, QY, QZ, QW
    c.absolute_x_axis_tolerance = tol_xy
    c.absolute_y_axis_tolerance = tol_xy
    c.absolute_z_axis_tolerance = tol_z
    c.weight = 1.0
    return c


class Probe(Node):
    def __init__(self):
        super().__init__("reach_probe")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cli.wait_for_service(timeout_sec=10.0)

    def try_ik(self, x, y, z, ori_tol_xy=None, ori_tol_z=None, free=False):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.ik_link_name = POSE_LINK
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = PLANNING_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.x, ps.pose.orientation.y = QX, QY
        ps.pose.orientation.z, ps.pose.orientation.w = QZ, QW
        req.ik_request.pose_stamped = ps
        cons = Constraints()
        cons.position_constraints.append(pos_constraint(x, y, z, POS_TOL))
        if not free:
            cons.orientation_constraints.append(ori_constraint(ori_tol_xy, ori_tol_z))
        req.ik_request.constraints = cons
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().error_code.val == 1


def main():
    rclpy.init()
    probe = Probe()
    print(f"\nProbing downward-grasp reachability, quat=({QX},{QY},{QZ},{QW}), "
          f"pos sphere {POS_TOL*100:.0f}cm\n")
    for label, x, y in COLUMNS:
        print(f"--- column {label} ---")
        print(f"{'z(m)':>6} | {'pos-only':>8} | min downward-orientation tol (rad / deg)")
        for z in HEIGHTS:
            free_ok = probe.try_ik(x, y, z, free=True)
            min_tol = None
            for tol in ORI_TOLS:
                if probe.try_ik(x, y, z, ori_tol_xy=tol, ori_tol_z=tol):
                    min_tol = tol
                    break
            tol_str = (f"{min_tol:.2f} ({math.degrees(min_tol):.0f} deg)"
                       if min_tol is not None else f">{ORI_TOLS[-1]:.2f}  UNREACHABLE down")
            print(f"{z:6.2f} | {'OK' if free_ok else 'NO':>8} | {tol_str}")
        print()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
