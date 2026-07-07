#!/usr/bin/env python3
"""
Pick-and-place demo: known start/end block poses, no camera.

- Lateral/approach moves (pre-grasp, pre-place) use joint-space OMPL
  planning with BOTH a position constraint AND an orientation constraint,
  so the gripper holds a fixed downward-facing pose instead of the
  "noodling" arbitrary-orientation behavior.
- Vertical pick/place/retreat moves use MoveIt's /compute_cartesian_path
  service directly (moveit_py's PlanningComponent doesn't expose Cartesian
  planning on this MoveIt version), so the end effector travels in a
  straight line along z instead of an arbitrary curved joint-space path.
- Gripper open/close via a direct FollowJointTrajectory action client to
  gripper_group_controller (bypasses MoveIt planning groups for the gripper).
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_configs_utils import MoveItConfigsBuilder


# ---- Tunable poses (defaults; adjust to real measurements later) ----
PICK_XYZ = (0.15, 0.04, 0.18)
PLACE_XYZ = (0.15, -0.04, 0.18)
APPROACH_HEIGHT = 0.03  # how far above pick/place to pre-position, meters

GRIPPER_OPEN = 0.15    # matches URDF joint upper limit
GRIPPER_CLOSED = -0.60  # a bit short of full -0.74 limit, safe close

POSE_LINK = "joint6_flange"
PLANNING_FRAME = "world"
GROUP_NAME = "arm_group"

# Downward-facing grasp orientation for joint6_flange -- confirmed
# REACHABLE via constraint-based IK probe (ik_probe.py) after fixing the
# camera_flange.dae mesh scale bug. This is roll=180deg, yaw=90deg: a
# genuine "point straight down" orientation, not an approximate one.
GRASP_QX = 0.707
GRASP_QY = 0.707
GRASP_QZ = 0.000
GRASP_QW = 0.000

CARTESIAN_MAX_STEP = 0.005       # 5mm interpolation resolution
CARTESIAN_JUMP_THRESHOLD = 0.0   # 0 disables jump-threshold filtering

# Designated home pose (matches reset_arm.py / config/initial_positions.yaml),
# originally specified in degrees and converted to radians here.
HOME_DEGREES = {
    "joint2_to_joint1": 2,
    "joint3_to_joint2": 41,
    "joint4_to_joint3": -89,
    "joint5_to_joint4": 48,
    "joint6_to_joint5": -2,
    "joint6output_to_joint6": 0,
}
HOME_RADIANS = {name: math.radians(deg) for name, deg in HOME_DEGREES.items()}


def _floatify_joint_limits(config_dict):
    try:
        joint_limits = config_dict["robot_description_planning"]["joint_limits"]
    except KeyError:
        return
    for limits in joint_limits.values():
        for key in ("max_velocity", "max_acceleration", "max_position", "min_position"):
            if key in limits and isinstance(limits[key], int):
                limits[key] = float(limits[key])


class RobotIOClient(Node):
    """Handles gripper open/close (action) and Cartesian path requests (service)."""

    def __init__(self):
        super().__init__("robot_io_client")
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_group_controller/follow_joint_trajectory"
        )
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")

    # ---- Gripper ----
    def gripper_move_to(self, position, duration_sec=1.0):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("gripper_group_controller action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper_controller"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        goal.trajectory.points = [point]

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # ---- Cartesian path ----
    def compute_cartesian_path(self, waypoints, avoid_collisions=True, path_constraints=None):
        """
        waypoints: list of geometry_msgs.msg.Pose for POSE_LINK, in PLANNING_FRAME.
        Returns (moveit_msgs/RobotTrajectory msg, fraction) or (None, 0.0) on failure.
        """
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_cartesian_path service not available")
            return None, 0.0

        request = GetCartesianPath.Request()
        request.header.frame_id = PLANNING_FRAME
        request.header.stamp = self.get_clock().now().to_msg()
        # Empty start_state + is_diff=True tells move_group to use the
        # robot's current state as the starting point.
        request.start_state.is_diff = True
        request.group_name = GROUP_NAME
        request.link_name = POSE_LINK
        request.waypoints = waypoints
        request.max_step = CARTESIAN_MAX_STEP
        request.jump_threshold = CARTESIAN_JUMP_THRESHOLD
        request.avoid_collisions = avoid_collisions
        if path_constraints is not None:
            request.path_constraints = path_constraints

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None:
            self.get_logger().error("Cartesian path service call failed (no response)")
            return None, 0.0

        return response.solution, response.fraction


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


def make_position_constraint(link_name, frame_id, x, y, z, tolerance=0.04):
    # Widened from 0.01 to 0.04 (4cm): on a non-redundant 6-DOF arm, only a
    # discrete set of orientations satisfy IK exactly at any single point.
    # A larger position sphere gives the constraint sampler room to land on
    # one of those solvable points near the target instead of being pinned
    # to one exact xyz where the desired orientation may not be achievable.
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


def make_orientation_constraint(link_name, frame_id, qx, qy, qz, qw,
                                 x_tolerance=0.15, y_tolerance=0.15, z_tolerance=3.14):
    # x/y tightened to ~8.6 deg now that GRASP_QX/QY/QZ/QW is a confirmed-
    # reachable orientation (via ik_probe.py), not a guess -- keeps the
    # gripper close to genuinely vertical. z (yaw) stays free.
    constraint = OrientationConstraint()
    constraint.header.frame_id = frame_id
    constraint.link_name = link_name
    constraint.orientation.x = qx
    constraint.orientation.y = qy
    constraint.orientation.z = qz
    constraint.orientation.w = qw
    constraint.absolute_x_axis_tolerance = x_tolerance
    constraint.absolute_y_axis_tolerance = y_tolerance
    constraint.absolute_z_axis_tolerance = z_tolerance
    constraint.weight = 1.0
    return constraint


def make_grasp_pose(x, y, z):
    """Pose for Cartesian waypoints: position + the fixed downward grasp orientation."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = GRASP_QX
    pose.orientation.y = GRASP_QY
    pose.orientation.z = GRASP_QZ
    pose.orientation.w = GRASP_QW
    return pose


def go_home(mycobot, arm):
    """Return the arm to its designated home pose (HOME_RADIANS) before planning
    anything else. Without this, a leftover pose from a previous run (e.g. gripper
    folded toward g_base) can leave the arm in self-collision, causing MoveIt's
    CheckStartStateCollision to reject all subsequent planning."""
    robot_model = mycobot.get_robot_model()
    goal_state = RobotState(robot_model)
    goal_state.set_joint_group_positions(GROUP_NAME, list(HOME_RADIANS.values()))
    goal_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)

    plan_result = arm.plan()
    if not plan_result:
        print("Planning to init_pose (home) FAILED.")
        return False

    print("Executing joint-space move to home pose...")
    mycobot.execute(plan_result.trajectory, controllers=[])
    return True


def move_arm_to(mycobot, arm, x, y, z, lock_orientation=True):
    """Joint-space plan to a target position, with the gripper orientation
    locked downward so the arm doesn't twist arbitrarily between waypoints."""
    arm.set_start_state_to_current_state()

    constraints = Constraints()
    constraints.position_constraints.append(
        make_position_constraint(POSE_LINK, PLANNING_FRAME, x, y, z)
    )
    if lock_orientation:
        constraints.orientation_constraints.append(
            make_orientation_constraint(POSE_LINK, PLANNING_FRAME, GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW)
        )
    arm.set_goal_state(motion_plan_constraints=[constraints])

    plan_result = arm.plan()
    if not plan_result:
        print(f"Planning FAILED for target ({x}, {y}, {z})")
        return False

    print(f"Executing joint-space move to ({x}, {y}, {z})...")
    mycobot.execute(plan_result.trajectory, controllers=[])
    return True


def cartesian_move_to(mycobot, io_client, x, y, z, min_fraction=0.95):
    """Straight-line Cartesian move from the current pose to (x, y, z),
    holding the fixed downward grasp orientation throughout."""
    target = make_grasp_pose(x, y, z)

    path_constraints = Constraints()
    path_constraints.orientation_constraints.append(
        make_orientation_constraint(POSE_LINK, PLANNING_FRAME, GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW)
    )

    solution_msg, fraction = io_client.compute_cartesian_path(
        waypoints=[target],
        avoid_collisions=True,
        path_constraints=path_constraints,
    )

    if solution_msg is None or fraction < min_fraction:
        print(f"Cartesian planning FAILED or incomplete for target ({x}, {y}, {z}) "
              f"(fraction={fraction:.2f})")
        return False

    print(f"Executing Cartesian move to ({x}, {y}, {z}) (fraction={fraction:.2f})...")

    robot_model = mycobot.get_robot_model()
    trajectory = RobotTrajectory(robot_model)

    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        current_state = scene.current_state
        trajectory.set_robot_trajectory_msg(current_state, solution_msg)

    mycobot.execute(trajectory, controllers=[])
    return True


def main():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])

    mycobot = build_moveit()
    arm = mycobot.get_planning_component(GROUP_NAME)
    io_client = RobotIOClient()

    px, py, pz = PICK_XYZ
    lx, ly, lz = PLACE_XYZ

    steps = [
        ("Return to home pose", lambda: go_home(mycobot, arm)),
        ("Open gripper (pre-start)", lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        # Lateral/approach moves: joint-space, orientation-locked
        ("Move to pre-grasp (above pick)",
         lambda: move_arm_to(mycobot, arm, px, py, pz + APPROACH_HEIGHT)),
        # Vertical descent: straight-line Cartesian
        ("Descend to grasp pose (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, px, py, pz)),
        ("Close gripper (grasp)", lambda: io_client.gripper_move_to(GRIPPER_CLOSED)),
        # Vertical retreat: straight-line Cartesian
        ("Retreat after grasp (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, px, py, pz + APPROACH_HEIGHT)),
        # Lateral transfer: joint-space, orientation-locked
        ("Move to pre-place (above place)",
         lambda: move_arm_to(mycobot, arm, lx, ly, lz + APPROACH_HEIGHT)),
        # Vertical descent: straight-line Cartesian
        ("Descend to place pose (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, lx, ly, lz)),
        ("Open gripper (release)", lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        # Vertical retreat: straight-line Cartesian
        ("Retreat after release (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, lx, ly, lz + APPROACH_HEIGHT)),
    ]

    for name, action in steps:
        print(f"\n=== {name} ===")
        ok = action()
        if ok is False:
            print(f"Step failed: {name}. Aborting sequence.")
            break
        time.sleep(0.5)

    print("\nPick-and-place sequence complete.")
    io_client.destroy_node()
    del mycobot
    rclpy.shutdown()


if __name__ == "__main__":
    main()