#!/usr/bin/env python3
"""
Pick-and-place demo: known start/end block poses, no camera.

- Lateral/approach moves (pre-grasp, pre-place) use joint-space OMPL
  planning with a deterministic seeded-IK goal state. KDL is seeded from
  multiple candidate joint configs; the first that is both IK-valid and
  collision-free is used. This replaces OMPL's randomized constraint
  sampling, which was landing the arm in a different (often near-singular
  or self-colliding) configuration each run.
- Vertical pick/place/retreat moves use MoveIt's /compute_cartesian_path
  service directly (moveit_py's PlanningComponent doesn't expose Cartesian
  planning on this MoveIt version), so the end effector travels in a
  straight line along z instead of an arbitrary curved joint-space path.
- Gripper open/close via a direct FollowJointTrajectory action client to
  gripper_group_controller (bypasses MoveIt planning groups for the gripper).
- Gripper closing watches gripper_controller's effort on /joint_states and
  stops as soon as it detects contact, instead of always driving to a fixed
  closed position regardless of what (if anything) is between the fingers.
  Requires the "effort" state_interface on gripper_controller -- see
  firefighter.ros2_control.xacro / ros2_controllers.yaml.
"""

import argparse
import math
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder


# ---- Tunable poses (defaults; adjust to real measurements later) ----
# PICK_XYZ/PLACE_XYZ's z is NOT a flange target -- see the pick/place-height
# convention comment above parse_args(). PICK_XYZ.z is a block CENTER
# (matches what spawn_world.py prints); PLACE_XYZ.z is a resting SURFACE
# (table top, or the top of the block underneath when stacking). Both are
# converted to actual flange targets in main() via GRASP_OFFSET_Z /
# DEFAULT_BLOCK_SIZE. PICK_XYZ.z below matches spawn_world.py's cube center
# (TABLE_TOP_Z + DEFAULT_BLOCK_SIZE/2 = 0.02 + 0.01 = 0.03) for the current
# cube size; re-derive it the same way if DEFAULT_BLOCK_SIZE changes again.
PICK_XYZ = (+0.000, +0.250, 0.030)
PLACE_XYZ = (+0.000, -0.250, 0.080)
APPROACH_HEIGHT = 0.08  # how far above pick/place to pre-position, meters

# Vertical distance from the commanded joint6_flange position down to where
# the gripper actually grips a block, i.e. flange_target_z = block_center_z +
# GRASP_OFFSET_Z when descending from directly above with the fixed downward
# grasp orientation. Consistent with the fingertip offset annulus_test.py
# measured via gripper_offset_probe.py. This does NOT depend on block size
# (it's purely gripper/flange geometry) -- re-measure with
# gripper_offset_probe.py and update this if the gripper or camera-flange
# geometry changes, not if the block size changes.
GRASP_OFFSET_Z = 0.09

# Cube side length, meters -- matches CUBE_SIZE_1/CUBE_SIZE_2 in
# spawn_world.py. Used to convert a place SURFACE height into the block-
# center height the flange must descend to when releasing.
DEFAULT_BLOCK_SIZE = 0.02

GRIPPER_OPEN = 0.15    # matches URDF joint upper limit
GRIPPER_CLOSED = -0.60  # a bit short of full -0.74 limit, safe close

# gripper_controller's URDF effort limit is 1000 (an unset-default value, not
# a real spec), so nothing in sim stops the gripper from driving straight
# through GRIPPER_CLOSED regardless of what's between the fingers -- it'll
# either crush/launch the block or grind against it at full commanded
# position error. GRIPPER_STEP closes in small increments instead, reading
# gripper_controller's effort off /joint_states after each one and stopping
# as soon as it spikes (contact), rather than always finishing at
# GRIPPER_CLOSED. GRIPPER_EFFORT_THRESHOLD is a placeholder -- has NOT been
# empirically tuned. Run once, watch the printed "[gripper] target=... "
# effort=..." trace while closing on a block, and set this comfortably above
# the free-swing noise floor and below whatever visibly deforms/launches the
# block.
GRIPPER_STEP = 0.03            # rad, per increment
GRIPPER_STEP_DURATION = 0.3    # sec, trajectory duration per increment
GRIPPER_SETTLE_SEC = 0.15      # sec to spin after each step before reading effort
GRIPPER_EFFORT_THRESHOLD = 5.0  # N*m -- UNTUNED, watch the trace and adjust

POSE_LINK = "joint6_flange"
PLANNING_FRAME = "world"
GROUP_NAME = "arm_group"

# Downward-facing grasp orientation for joint6_flange -- confirmed
# REACHABLE via constraint-based IK probe (ik_probe.py) after fixing the
# camera_flange.dae mesh scale bug. This is roll=180deg, yaw=90deg: a
# genuine "point straight down" orientation, not an approximate one. This is
# the yaw=0 REFERENCE quaternion for gripper_yaw_quat() below -- other
# scripts (annulus_test.py) import these raw components directly, so leave
# them as-is and do yaw adjustments via gripper_yaw_quat() instead.
GRASP_QX = -0.7071
GRASP_QY = 0.7071
GRASP_QZ = 0.0
GRASP_QW = 0.0

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

# joint6output_to_joint6 is the last joint before joint6_flange (POSE_LINK)
# and rotates the flange (and gripper) about its own approach axis without
# moving its position -- under the downward grasp orientation that axis is
# world-vertical, so this joint is exactly "gripper yaw about the vertical."
# Earlier this was pinned to a constant RAW JOINT VALUE, which does NOT keep
# the gripper facing a constant WORLD direction -- joint6output's zero
# position is measured relative to the base's own rotation (joint1), which
# differs between pick and place, so a fixed joint value let the world-frame
# facing drift between targets. Locking the world orientation instead means
# targeting a fixed absolute quaternion via full 6D IK (below) and letting
# joint6output solve to whatever value that requires -- it will vary, and
# that's the point: it's actively holding the gripper's facing constant in
# the world frame as the arm reaches to different (x, y).
#
# GRIPPER_YAW_DEG is the fixed world yaw (about Z) applied on top of the
# GRASP_Q* reference orientation above. 0.0 reproduces GRASP_Q* unchanged.
# Not yet empirically confirmed to be parallel to world +X -- watch the
# gripper in sim and adjust in +/-90deg steps until the jaws line up with X.
GRIPPER_YAW_DEG = 0.0


def quat_multiply(q1, q2):
    """Hamilton product q1 * q2, both as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def gripper_yaw_quat(yaw_deg=GRIPPER_YAW_DEG):
    """q_yaw(yaw_deg) * GRASP_Q -- the fixed downward grasp quaternion
    rotated by yaw_deg around world Z, keeping the "point straight down"
    component intact while fixing the world-frame gripper facing."""
    yaw = math.radians(yaw_deg)
    q_yaw = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    q_grasp = (GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW)
    return quat_multiply(q_yaw, q_grasp)


# The actual grasp target used throughout this script -- gripper always
# facing the same fixed world direction, regardless of pick/place location.
GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW = gripper_yaw_quat()

# IK candidate seeds tried in order. KDL's numeric solver can jump to a
# different solution branch depending on the seed, and some branches self-
# collide or have joint6output pegged at its limit (-2.44 rad). We provide
# multiple seeds biasing different arm configurations; the first that passes
# both IK convergence and collision checks is used.
#
# Each entry: (label, joint_values_dict)
# joint4 is the elbow -- keeping it around -1.55 (home) puts the arm in the
# elbow-down configuration. Seeds with joint6output=0 prevent it from being
# pegged at its limit.
def _build_ik_seeds():
    seeds = []

    # Seed 1: straight home -- the most natural starting point
    # Confirmed-good seed for the downward grasp orientation, found via
    # empirical IK/Cartesian survey after the camera_flange URDF fix.
    downward_seed = {
        "joint2_to_joint1":       0.324,
        "joint3_to_joint2":      -0.334,
        "joint4_to_joint3":      -0.655,
        "joint5_to_joint4":      -0.582,
        "joint6_to_joint5":      -0.0,
        "joint6output_to_joint6": 0.324,
    }
    seeds.append(("downward-confirmed", downward_seed))
    seeds.append(("home", dict(HOME_RADIANS)))

    # Seed 2: home but with joint6output forced to 0 (prevents limit-pegging)
    s = dict(HOME_RADIANS)
    s["joint6output_to_joint6"] = 0.0
    s["joint6_to_joint5"] = 0.0
    seeds.append(("home-wrist-zeroed", s))

    # Seed 3: elbow slightly more bent, wrist joints zeroed
    s = dict(HOME_RADIANS)
    s["joint4_to_joint3"] = -1.8
    s["joint5_to_joint4"] = 0.5
    s["joint6_to_joint5"] = 0.0
    s["joint6output_to_joint6"] = 0.0
    seeds.append(("elbow-bent-wrist-zero", s))

    # Seed 4: all zeros -- catches cases where other seeds all fail
    s = {name: 0.0 for name in HOME_RADIANS}
    seeds.append(("all-zeros", s))

    # Seed 5: joint4 forced negative and deep, biases strongly toward elbow-down
    s = dict(HOME_RADIANS)
    s["joint4_to_joint3"] = -2.0
    s["joint5_to_joint4"] = 0.3
    s["joint6_to_joint5"] = 0.0
    s["joint6output_to_joint6"] = 0.0
    seeds.append(("deep-elbow-down", s))

    return seeds


IK_SEEDS = _build_ik_seeds()


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
    """Handles gripper open/close (action), arm trajectory execution (action),
    and Cartesian path requests (service)."""

    def __init__(self):
        super().__init__("robot_io_client")
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_group_controller/follow_joint_trajectory"
        )
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_group_controller/follow_joint_trajectory"
        )
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._joint_efforts = {}
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        for name, effort in zip(msg.name, msg.effort):
            self._joint_efforts[name] = effort

    def joint_effort(self, joint_name):
        """Latest effort reading for joint_name from /joint_states, or None
        if it hasn't been received yet (e.g. no "effort" state_interface
        configured for that joint)."""
        return self._joint_efforts.get(joint_name)

    # ---- Arm ----
    def arm_execute(self, joint_trajectory):
        """Send a trajectory_msgs/JointTrajectory straight to
        arm_group_controller's action server, bypassing MoveItPy's own
        execution manager. MoveItPy's built-in execute() validates the
        current joint state's timestamp against its own clock before
        running, and with use_sim_time it crashes on construction (a known
        unresolved bug: moveit/moveit2#2220, #2940) so it never runs with
        sim time at all -- it only ever sees wall-clock, which will never
        match Gazebo's sim-time-stamped /joint_states, so validation always
        times out. Going straight to the controller's action server (same
        pattern already used for the gripper below) skips that check
        entirely.
        """
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("arm_group_controller action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

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
        MoveItConfigsBuilder("firefighter", package_name="mycobot_280pi_camera_moveit2")
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

    # use_sim_time can't go through config_dict: MoveItPy's config_dict path
    # triggers an upstream bug (moveit2#2220/#2940) where enabling sim time
    # crashes with "qos_overrides./clock.subscription.durability could not
    # be set". Loading it through a real YAML file via launch_params_filepaths
    # goes through the normal rclcpp parameter-file path instead and avoids
    # that bug. Without this, MoveItPy's clock stays on wall-time while
    # Gazebo's joint_states are stamped with sim time, so every trajectory
    # validation fails with "couldn't receive full current joint state
    # within 1s" even though joint_states is publishing fine.
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False
    ) as sim_time_yaml:
        sim_time_yaml.write("/**:\n  ros__parameters:\n    use_sim_time: true\n")
        sim_time_yaml_path = sim_time_yaml.name

    return MoveItPy(
        node_name="pick_place",
        config_dict=config_dict,
        launch_params_filepaths=[sim_time_yaml_path],
    )


def make_position_constraint(link_name, frame_id, x, y, z, tolerance=0.04):
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
    """Pose for Cartesian waypoints: position + the fixed downward, fixed-yaw grasp orientation."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = GRIPPER_LOCK_QX
    pose.orientation.y = GRIPPER_LOCK_QY
    pose.orientation.z = GRIPPER_LOCK_QZ
    pose.orientation.w = GRIPPER_LOCK_QW
    return pose


def _is_state_colliding(mycobot, state):
    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        return scene.is_state_colliding(state, GROUP_NAME)


def _is_near_joint_limit(state, margin=0.15):
    """Reject IK solutions where joint6output_to_joint6 is near its limit.
    KDL pegs it at -2.4434 rad even when seeded elsewhere; OMPL can't plan
    to a state wedged at a joint limit (no room to sample nearby states)."""
    # joint6output_to_joint6 limits from URDF: lower=-2.4434, upper=3.14159
    positions = state.joint_positions  # dict: joint_name -> value
    val = positions.get("joint6output_to_joint6", None)
    if val is not None:
        if val < -2.4434 + margin or val > 3.14159 - margin:
            return True, "joint6output_to_joint6", val, -2.4434, 3.14159
    return False, None, None, None, None


def solve_ik_state(mycobot, x, y, z, qx, qy, qz, qw):
    """Try each IK_SEEDS entry in order. Return the first RobotState that
    both converges and is collision-free, or None if all seeds fail."""
    robot_model = mycobot.get_robot_model()
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    for label, seed in IK_SEEDS:
        state = RobotState(robot_model)
        state.set_joint_group_positions(GROUP_NAME, list(seed.values()))
        state.update()

        if not state.set_from_ik(GROUP_NAME, pose, POSE_LINK, timeout=0.5):
            print(f"[ik] '{label}' seed: IK did not converge")
            continue

        joints = [round(v, 3) for v in state.get_joint_group_positions(GROUP_NAME)]

        near_limit, lname, lval, llo, lhi = _is_near_joint_limit(state)
        if near_limit:
            print(f"[ik] '{label}' seed: converged to {joints} BUT '{lname}'={lval:.3f} "
                  f"near limit [{llo:.3f},{lhi:.3f}], skipping")
            continue

        if _is_state_colliding(mycobot, state):
            print(f"[ik] '{label}' seed: converged to {joints} BUT self-collides, skipping")
            continue

        print(f"[ik] '{label}' seed: OK -> {joints}")
        return state

    print(f"[ik] All seeds exhausted for ({x:.3f},{y:.3f},{z:.3f}) -- falling back to constraint sampling")
    return None


def toggle_gripper(io_client):
    """Close then open the gripper as a functional pre-start check."""
    if not io_client.gripper_move_to(GRIPPER_CLOSED):
        return False
    time.sleep(0.5)
    return io_client.gripper_move_to(GRIPPER_OPEN)


def gripper_close_until_contact(io_client, start=GRIPPER_OPEN, closed=GRIPPER_CLOSED,
                                 step=GRIPPER_STEP, step_duration=GRIPPER_STEP_DURATION,
                                 settle_sec=GRIPPER_SETTLE_SEC,
                                 effort_threshold=GRIPPER_EFFORT_THRESHOLD):
    """Close the gripper in small increments, stopping as soon as
    gripper_controller's measured effort exceeds effort_threshold (contact
    with the block) instead of always driving to `closed` regardless of
    what's in the way. Falls back to a single move straight to `closed` if
    no effort reading is available (e.g. the effort state_interface isn't
    configured), since that's strictly the old behavior, not a new failure
    mode. Returns True unless a gripper action call itself fails."""
    target = start
    got_any_reading = False

    while target > closed:
        target = max(closed, target - step)
        if not io_client.gripper_move_to(target, duration_sec=step_duration):
            return False

        rclpy.spin_once(io_client, timeout_sec=settle_sec)
        effort = io_client.joint_effort("gripper_controller")

        if effort is None:
            print(f"[gripper] target={target:.3f}  effort=<no reading -- "
                  f"is the effort state_interface configured?>")
            continue

        got_any_reading = True
        print(f"[gripper] target={target:.3f}  effort={effort:.3f}")
        if abs(effort) >= effort_threshold:
            print(f"[gripper] contact detected (|effort|={abs(effort):.3f} >= "
                  f"{effort_threshold:.3f}) at position {target:.3f}, stopping close")
            return True

    if not got_any_reading:
        print("[gripper] no effort readings received at all -- closed fully to "
              f"{closed:.3f} without contact detection (old fixed-close behavior)")
    else:
        print(f"[gripper] reached fully-closed position {closed:.3f} without "
              "detecting contact (nothing between the fingers, or "
              "GRIPPER_EFFORT_THRESHOLD is set too high)")
    return True


def go_home(mycobot, arm, io_client):
    """Return the arm to its designated home pose before planning anything else."""
    robot_model = mycobot.get_robot_model()
    goal_state = RobotState(robot_model)
    goal_state.set_joint_group_positions(GROUP_NAME, list(HOME_RADIANS.values()))
    goal_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)

    plan_result = arm.plan()
    if not plan_result:
        print("Planning to home pose FAILED.")
        return False

    print("Executing joint-space move to home pose...")
    joint_trajectory = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
    return io_client.arm_execute(joint_trajectory)


def move_arm_to(mycobot, arm, io_client, x, y, z, lock_orientation=True):
    """Joint-space plan to a target position. Uses deterministic seeded IK
    when possible; falls back to OMPL constraint sampling if all seeds fail."""
    arm.set_start_state_to_current_state()

    ik_state = None
    if lock_orientation:
        ik_state = solve_ik_state(mycobot, x, y, z,
                                   GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW)

    if ik_state is not None:
        arm.set_goal_state(robot_state=ik_state)
    else:
        print(f"[move_arm_to] No valid IK state found for ({x},{y},{z}), using constraint sampling")
        constraints = Constraints()
        constraints.position_constraints.append(
            make_position_constraint(POSE_LINK, PLANNING_FRAME, x, y, z)
        )
        if lock_orientation:
            # z_tolerance tightened to match x/y (default is ~free, 3.14) so
            # the fallback also holds the fixed world yaw instead of letting
            # OMPL pick any wrist twist -- see GRIPPER_YAW_DEG above.
            constraints.orientation_constraints.append(
                make_orientation_constraint(
                    POSE_LINK, PLANNING_FRAME,
                    GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW,
                    z_tolerance=0.15)
            )
        arm.set_goal_state(motion_plan_constraints=[constraints])

    plan_result = arm.plan()
    if not plan_result:
        print(f"Planning FAILED for target ({x}, {y}, {z})")
        return False

    print(f"Executing joint-space move to ({x}, {y}, {z})...")
    joint_trajectory = plan_result.trajectory.get_robot_trajectory_msg().joint_trajectory
    return io_client.arm_execute(joint_trajectory)


def cartesian_move_to(mycobot, io_client, x, y, z, min_fraction=0.90):
    """Straight-line Cartesian move from the current pose to (x, y, z),
    holding the fixed downward grasp orientation throughout."""
    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        joint_values = scene.current_state.get_joint_group_positions(GROUP_NAME)
        print(f"[cartesian] joints at start: {[round(v, 4) for v in joint_values]}")

    target = make_grasp_pose(x, y, z)

    path_constraints = Constraints()
    path_constraints.orientation_constraints.append(
        make_orientation_constraint(
            POSE_LINK, PLANNING_FRAME,
            GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW,
            z_tolerance=0.15)
    )

    # DIAGNOSTIC: try with AND without orientation constraint to isolate
    # whether the constraint or the pose itself is causing the failure
    solution_msg, fraction = io_client.compute_cartesian_path(
        waypoints=[target],
        avoid_collisions=True,
        path_constraints=None,  # TEMP: no orientation constraint
    )
    print(f"[diag] Cartesian fraction WITHOUT orientation constraint: {fraction:.2f}")
    solution_msg2, fraction2 = io_client.compute_cartesian_path(
        waypoints=[target],
        avoid_collisions=True,
        path_constraints=path_constraints,
    )
    print(f"[diag] Cartesian fraction WITH orientation constraint: {fraction2:.2f}")
    solution_msg, fraction = solution_msg2, fraction2

    if solution_msg is None or fraction < min_fraction:
        print(f"Cartesian planning FAILED for ({x}, {y}, {z}) (fraction={fraction:.2f})")
        return False

    print(f"Executing Cartesian move to ({x}, {y}, {z}) (fraction={fraction:.2f})...")

    return io_client.arm_execute(solution_msg.joint_trajectory)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pick-and-place demo with hardcoded/known block poses.")
    parser.add_argument("--pick-position", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        default=list(PICK_XYZ),
                        help="pick location, meters. X,Y and the picked-up block's "
                        "CENTER Z -- same convention spawn_world.py's coordinate "
                        f"summary prints, paste directly (default: {PICK_XYZ})")
    parser.add_argument("--place-position", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        default=list(PLACE_XYZ),
                        help="place location, meters. X,Y and the SURFACE Z the block "
                        "should come to rest on -- the table top for a fresh "
                        "placement, or the top of the block underneath when "
                        f"stacking (default: {PLACE_XYZ})")
    parser.add_argument("--block-size", type=float, default=DEFAULT_BLOCK_SIZE,
                        help="cube side length, meters, used to turn --place-position's "
                        "surface Z into the block-center Z the flange must descend to "
                        f"when releasing (default: {DEFAULT_BLOCK_SIZE})")
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])

    mycobot = build_moveit()
    arm = mycobot.get_planning_component(GROUP_NAME)
    io_client = RobotIOClient()

    px, py, pick_center_z = args.pick_position
    lx, ly, place_surface_z = args.place_position

    # Convert block-center (pick) / resting-surface (place) heights into
    # actual joint6_flange targets -- see GRASP_OFFSET_Z above.
    pz = pick_center_z + GRASP_OFFSET_Z
    lz = place_surface_z + args.block_size / 2.0 + GRASP_OFFSET_Z

    print(f"[pick_place] pick block-center z={pick_center_z:.3f} -> flange target z={pz:.3f}")
    print(f"[pick_place] place surface z={place_surface_z:.3f} "
          f"(block size {args.block_size:.3f}) -> flange target z={lz:.3f}")

    steps = [
        ("Return to home pose", lambda: go_home(mycobot, arm, io_client)),
        ("Toggle gripper (pre-start)", lambda: toggle_gripper(io_client)),
        ("Move to pre-grasp (above pick)",
         lambda: move_arm_to(mycobot, arm, io_client, px, py, pz + APPROACH_HEIGHT)),
        ("Descend to grasp pose (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, px, py, pz)),
        ("Close gripper (grasp, stop on contact)", lambda: gripper_close_until_contact(io_client)),
        ("Retreat after grasp (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, px, py, pz + APPROACH_HEIGHT)),
        ("Move to pre-place (above place)",
         lambda: move_arm_to(mycobot, arm, io_client, lx, ly, lz + APPROACH_HEIGHT)),
        ("Descend to place pose (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, lx, ly, lz)),
        ("Open gripper (release)", lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        ("Retreat after release (Cartesian)",
         lambda: cartesian_move_to(mycobot, io_client, lx, ly, lz + APPROACH_HEIGHT)),
        ("Return to home pose (final)", lambda: go_home(mycobot, arm, io_client)),
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