#!/usr/bin/env python3
"""
Pick-and-place demo: known start/end block poses, no camera.
Arm motion via MoveItPy; gripper open/close via a direct
FollowJointTrajectory action client to gripper_group_controller
(bypasses MoveIt planning groups for the gripper).
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


# ---- Tunable poses (defaults; adjust to real measurements later) ----
# Kept close to the already-proven-reachable point (0.15, 0.0, 0.20) since
# fixed-orientation (identity) reachability is far more restrictive than
# position alone on this arm's small ~280mm reach.
PICK_XYZ = (0.15, 0.04, 0.18)
PLACE_XYZ = (0.15, -0.04, 0.18)
APPROACH_HEIGHT = 0.03  # how far above pick/place to pre-position, meters

GRIPPER_OPEN = 0.15   # matches URDF joint upper limit
GRIPPER_CLOSED = -0.60  # a bit short of full -0.74 limit, safe close

POSE_LINK = "joint6_flange"
PLANNING_FRAME = "world"


def _floatify_joint_limits(config_dict):
    try:
        joint_limits = config_dict["robot_description_planning"]["joint_limits"]
    except KeyError:
        return
    for limits in joint_limits.values():
        for key in ("max_velocity", "max_acceleration", "max_position", "min_position"):
            if key in limits and isinstance(limits[key], int):
                limits[key] = float(limits[key])


class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")
        self._client = ActionClient(
            self, FollowJointTrajectory, "/gripper_group_controller/follow_joint_trajectory"
        )

    def move_to(self, position, duration_sec=1.0):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("gripper_group_controller action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper_controller"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        goal.trajectory.points = [point]

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True


def build_moveit():
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2")
        .to_moveit_configs()
    )
    config_dict = moveit_config.to_dict()
    config_dict["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    config_dict["plan_request_params"] = {
        "planning_time": 10.0,
        "planning_pipeline": "ompl",
        "max_velocity_scaling_factor": 1.0,
        "max_acceleration_scaling_factor": 1.0,
    }
    _floatify_joint_limits(config_dict)
    return MoveItPy(node_name="pick_place", config_dict=config_dict)


def make_position_constraint(link_name, frame_id, x, y, z, tolerance=0.01):
    constraint = PositionConstraint()
    constraint.header.frame_id = frame_id
    constraint.link_name = link_name

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [tolerance]
    constraint.constraint_region.primitives.append(primitive)

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    constraint.constraint_region.primitive_poses.append(pose)

    constraint.weight = 1.0
    return constraint


def move_arm_to(mycobot, arm, x, y, z):
    arm.set_start_state_to_current_state()

    constraints = Constraints()
    constraints.position_constraints.append(
        make_position_constraint(POSE_LINK, PLANNING_FRAME, x, y, z)
    )
    arm.set_goal_state(motion_plan_constraints=[constraints])

    plan_result = arm.plan()
    if not plan_result:
        print(f"Planning FAILED for target ({x}, {y}, {z})")
        return False

    print(f"Executing move to ({x}, {y}, {z})...")
    mycobot.execute(plan_result.trajectory, controllers=[])
    return True


def main():
    rclpy.init()

    mycobot = build_moveit()
    arm = mycobot.get_planning_component("arm_group")
    gripper = GripperClient()

    px, py, pz = PICK_XYZ
    lx, ly, lz = PLACE_XYZ

    steps = [
        ("Open gripper (pre-start)", lambda: gripper.move_to(GRIPPER_OPEN)),
        ("Move to pre-grasp (above pick)", lambda: move_arm_to(mycobot, arm, px, py, pz + APPROACH_HEIGHT)),
        ("Move to grasp pose", lambda: move_arm_to(mycobot, arm, px, py, pz)),
        ("Close gripper (grasp)", lambda: gripper.move_to(GRIPPER_CLOSED)),
        ("Retreat after grasp", lambda: move_arm_to(mycobot, arm, px, py, pz + APPROACH_HEIGHT)),
        ("Move to pre-place (above place)", lambda: move_arm_to(mycobot, arm, lx, ly, lz + APPROACH_HEIGHT)),
        ("Move to place pose", lambda: move_arm_to(mycobot, arm, lx, ly, lz)),
        ("Open gripper (release)", lambda: gripper.move_to(GRIPPER_OPEN)),
        ("Retreat after release", lambda: move_arm_to(mycobot, arm, lx, ly, lz + APPROACH_HEIGHT)),
    ]

    for name, action in steps:
        print(f"\n=== {name} ===")
        ok = action()
        if ok is False:
            print(f"Step failed: {name}. Aborting sequence.")
            break
        time.sleep(0.5)

    print("\nPick-and-place sequence complete.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
