#!/usr/bin/env python3
"""
Pick-and-place demo: known start/end block poses, no camera.

- Lateral/approach moves (pre-grasp, pre-place) use joint-space OMPL
  planning with a deterministic IK goal state. The goal is solved via
  MoveIt's /compute_ik service using a small position sphere + orientation
  window (constraint-based IK), seeded from multiple candidate joint
  configs; the first collision-free, non-limit-pegged solution is used.
  This replaces two earlier approaches: exact-pose RobotState.set_from_ik,
  which effectively never converged for the downward grasp on this
  non-redundant 6-DOF arm (KDL couldn't land on the exact orientation near
  the wrist singularity), and raw OMPL constraint sampling, which landed the
  arm in a different (often near-singular) configuration each run.
- Vertical pick/place/retreat moves use MoveIt's /compute_cartesian_path
  service directly, so the end effector travels in a straight line along z
  instead of an arbitrary curved joint-space path.
- Joint-space planning (pre-grasp/pre-place, home) uses MoveIt's
  /plan_kinematic_path service against an externally-launched move_group --
  no moveit_py. This works against any ROS2 distro with MoveIt2, regardless
  of whether prebuilt moveit_py bindings exist for it (they don't for every
  distro/platform this project targets).
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
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetPositionIK, GetMotionPlan, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# ---- Tunable poses (defaults; adjust to real measurements later) ----
# PICK_XYZ/PLACE_XYZ's z is NOT a flange target -- see the pick/place-height
# convention comment above parse_args(). PICK_XYZ.z is a block CENTER
# (matches what spawn_world.py prints); PLACE_XYZ.z is a resting SURFACE
# (table top, or the top of the block underneath when stacking). Both are
# converted to actual flange targets in main() via GRASP_OFFSET_Z /
# DEFAULT_BLOCK_SIZE.
#
# MoveIt plans purely in joint-angle space against the URDF's own kinematic
# tree, which has no knowledge of where gazebo.launch.py's `ros_gz_sim
# create -z ...` physically places the robot in Gazebo's absolute frame.
# But since those SAME joint angles are what actually gets executed by
# Gazebo (which IS anchored at the spawn pose), any MoveIt Cartesian target
# lands, in Gazebo-absolute space, at target_z + spawn_z. spawn_z was
# lowered from 0.055 to 0.02 (g_base embedded in the table -- see
# gazebo.launch.py) to make the real robot flush with the table, a -0.035m
# shift. Every target below that's meant to hit an ABSOLUTE table/block
# height -- i.e. numbers taken directly from spawn_world.py's printed
# coordinates -- needs +0.035m to compensate, or it'll now aim 0.035m too
# low (into the table). GRASP_OFFSET_Z below is NOT one of these: it's a
# difference between two points on the same rigid gripper (flange to
# fingertip), so the spawn shift cancels out of it and it's untouched.
#
# SPAWN_HEIGHT_CORRECTION documents that delta; re-derive it (old_spawn_z -
# new_spawn_z) and reapply below if the spawn height in gazebo.launch.py
# ever changes again. PICK_XYZ.z, uncorrected, would match spawn_world.py's
# cube center (TABLE_TOP_Z + DEFAULT_BLOCK_SIZE/2 = 0.02 + 0.01 = 0.03); add
# the correction the same way for any new pick/place position derived from
# spawn_world.py's output. NOT re-verified by an actual run yet -- watch the
# first grasp closely.
SPAWN_HEIGHT_CORRECTION = 0.035  # m, = old spawn_z (0.055) - new spawn_z (0.02)
PICK_XYZ = (+0.000, +0.250, 0.030 + SPAWN_HEIGHT_CORRECTION)
PLACE_XYZ = (+0.000, -0.250, 0.040 + SPAWN_HEIGHT_CORRECTION)
# APPROACH_HEIGHT is how far above the grasp/place flange target to
# pre-position for the straight-down descent. It is HARD-CAPPED by the arm's
# reach, NOT a free choice: at the pick/place radius (0.25m) the flange can
# only reach at all up to z~=0.21 (measured with scripts/reach_probe.py --
# 0.21 works, 0.215 is already outside the workspace at ANY orientation).
# The place flange target (0.16) is the higher of the two, so keep
# 0.16+APPROACH_HEIGHT <= ~0.205. Anything taller puts the hover outside the
# reachable workspace, where every IK seed legitimately fails to converge
# (there is no solution) and the OMPL fallback can only satisfy its 4cm
# position sphere by parking the flange lower AND tilted -- that was the
# "all seeds exhausted" + visibly-tilted-hover symptom. 0.04 keeps both
# hovers (0.18 pick, 0.20 place) comfortably reachable and pointing straight
# down to within ~3 deg. Re-measure the ceiling with reach_probe.py if the
# targets, radius, or robot mount height change.
APPROACH_HEIGHT = 0.04  # meters above the flange target; capped by reach (see above)

# Vertical distance from the commanded joint6_flange position down to where
# the gripper actually grips a block, i.e. flange_target_z = block_center_z +
# GRASP_OFFSET_Z when descending from directly above with the fixed downward
# grasp orientation. Consistent with the fingertip offset annulus_test.py
# measured via gripper_offset_probe.py. This does NOT depend on block size
# (it's purely gripper/flange geometry) -- re-measure with
# gripper_offset_probe.py and update this if the gripper or camera-flange
# geometry changes, not if the block size changes.
GRASP_OFFSET_Z = 0.075

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
# GRIPPER_CLOSED.
#
# An observed trace closing on a 2cm cube: free-swing noise floor sits at
# ~0.001 all the way through -0.390, then the SINGLE NEXT 0.03 rad step (to
# -0.420) already spiked to -0.78, and the step after that (-0.450) to
# -2.28. 0.03 rad is too coarse to land ON the "just touching" point --
# it jumps straight past it into the squeeze/glitch regime in one step, so
# no single GRIPPER_EFFORT_THRESHOLD value can distinguish "holding it well"
# from "squeezing it out of the gripper": both can happen within the same
# increment. No threshold fixes a resolution problem -- what actually helps
# is taking smaller, slower steps once past the point contact was last
# observed, so the effort reading has a chance to land in between.
# GRIPPER_FINE_ZONE is that cutover position; below GRIPPER_STEP/
# GRIPPER_STEP_DURATION are used, at-or-past it GRIPPER_FINE_STEP/
# GRIPPER_FINE_STEP_DURATION take over. Re-tune all of this the same way
# (watch the "[gripper] target=... effort=..." trace) if the block
# size/mass or gripper geometry changes enough to shift these numbers.
GRIPPER_STEP = 0.03              # rad, per increment before the fine zone
GRIPPER_STEP_DURATION = 0.3      # sec, trajectory duration per coarse increment
GRIPPER_FINE_ZONE = -0.40        # rad -- switch to fine stepping at/past this position
GRIPPER_FINE_STEP = 0.005        # rad, per increment once inside the fine zone
GRIPPER_FINE_STEP_DURATION = 0.5  # sec, per fine increment -- slower, gives the
                                   # physics engine/effort reading more time to
                                   # settle between smaller nudges
GRIPPER_SETTLE_SEC = 0.15        # sec to spin after each step before reading effort
GRIPPER_EFFORT_THRESHOLD = 0.2   # N*m

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

# Tolerances for the constraint-based IK goal search (solve_ik_state).
# Exact-pose IK (RobotState.set_from_ik) demands the flange hit the target
# position AND orientation to KDL's tight numeric tolerance; near this arm's
# downward-grasp wrist configuration that Newton solve fails to converge from
# every seed -- even the current state sitting 8cm directly below a target on
# a column the Cartesian planner reaches at fraction=1.00 (see solve_ik_state
# for the full explanation). Allowing a small position sphere + orientation
# window -- the same trick ik_probe.py used to confirm reachability -- makes
# the same targets converge reliably and deterministically. The hover this
# feeds only needs to be roughly downward; the straight-down orientation is
# re-imposed exactly by the Cartesian descent that follows.
IK_POS_TOLERANCE = 0.02          # m, radius of the goal position sphere
# 0.10 rad (~5.7 deg): at the reachable hover heights straight-down solves to
# within ~3 deg (measured, reach_probe.py), so this window is comfortably
# satisfiable while still keeping the hover visibly straight rather than
# tilted. Widen it only if a target near the reach edge stops converging.
IK_ORI_XY_TOLERANCE = 0.10       # rad, tilt allowed off straight-down
IK_ORI_Z_TOLERANCE = 0.15        # rad, yaw window about the approach axis
IK_SERVICE_TIMEOUT = 0.3         # sec, per-seed /compute_ik solve budget

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

    # Mirrored copies: every seed above biases joint2_to_joint1 (the base
    # rotation) toward the SAME positive direction (0.324, HOME_RADIANS's
    # +0.035, or 0.0) -- none bias negative. KDL's IK is a local numeric
    # solver that converges to whichever solution branch is nearest its
    # seed, not a global search, so targets needing the base rotated the
    # OTHER way (e.g. PLACE_XYZ's negative Y) had no seed anywhere near
    # their true solution and consistently failed to converge on every one
    # of the seeds above, forcing the slow/unreliable OMPL constraint-
    # sampling fallback. Appended after the originals (not interleaved) so
    # positive-Y targets keep converging on the same seed as before with no
    # behavior change; negative-Y targets now fall through to these instead
    # of exhausting all 6 originals and failing.
    seeds += [(f"{label}-mirrored", _mirror_seed(seed)) for label, seed in seeds]

    return seeds


def _mirror_seed(seed):
    """Negate the base rotation and wrist-twist as a starting guess for a
    target on the opposite side of the robot from what `seed` was tuned
    for. Elbow/wrist bend (joint3/4/5) don't need mirroring -- those only
    depend on radial distance and height, not which side of the base the
    target is on."""
    mirrored = dict(seed)
    mirrored["joint2_to_joint1"] = -seed["joint2_to_joint1"]
    mirrored["joint6output_to_joint6"] = -seed["joint6output_to_joint6"]
    return mirrored


IK_SEEDS = _build_ik_seeds()


class RobotIOClient(Node):
    """Handles gripper open/close (action), arm trajectory execution (action),
    Cartesian path / IK / motion planning / state validity (services). Talks
    only to an externally-launched move_group over plain ROS2 services and
    actions -- no moveit_py, so this works against any ROS2 distro that has
    MoveIt2, regardless of whether moveit_py bindings were ever packaged for
    it (see build_motion_plan_request / check_state_validity below)."""

    def __init__(self):
        super().__init__("robot_io_client")
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_group_controller/follow_joint_trajectory"
        )
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_group_controller/follow_joint_trajectory"
        )
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._joint_efforts = {}
        self._joint_positions = {}
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        for name, effort in zip(msg.name, msg.effort):
            self._joint_efforts[name] = effort
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position

    def joint_effort(self, joint_name):
        """Latest effort reading for joint_name from /joint_states, or None
        if it hasn't been received yet (e.g. no "effort" state_interface
        configured for that joint)."""
        return self._joint_efforts.get(joint_name)

    def current_joint_positions(self, joint_names, timeout_sec=5.0):
        """Latest /joint_states positions for joint_names, waiting for the
        first message to arrive if none has been received yet. Replaces the
        old planning_scene_monitor-based current-state read (moveit_py) --
        the live /joint_states topic already carries the same values."""
        end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while not self._joint_positions and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        return {n: self._joint_positions.get(n, 0.0) for n in joint_names}

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

    # ---- Inverse kinematics ----
    def compute_ik(self, x, y, z, qx, qy, qz, qw, seed_joint_names, seed_positions,
                   pos_tolerance=IK_POS_TOLERANCE, xy_tolerance=IK_ORI_XY_TOLERANCE,
                   z_tolerance=IK_ORI_Z_TOLERANCE, timeout_sec=IK_SERVICE_TIMEOUT):
        """Constraint-based IK via MoveIt's /compute_ik service: find a
        collision-free joint solution that puts POSE_LINK within a small
        position sphere + orientation window of the target, seeded from a
        specific configuration so the numeric solver stays on one solution
        branch instead of jumping between them per call.

        Returns {joint_name: value} for the whole returned joint_state (arm
        joints plus whatever else move_group echoes back), or None if the
        solver found nothing inside the tolerated region. This succeeds on
        targets that RobotState.set_from_ik (exact pose) cannot -- see
        solve_ik_state / IK_POS_TOLERANCE above for why.
        """
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik service not available")
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.ik_link_name = POSE_LINK
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int((timeout_sec % 1) * 1e9)

        # Seed the numeric solver. is_diff=True applies these joint values as
        # an override on top of the robot's current state, so unspecified
        # joints (e.g. the gripper) come from the live state rather than 0.
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = list(seed_joint_names)
        request.ik_request.robot_state.joint_state.position = list(seed_positions)

        # pose_stamped is required by the message even alongside constraints;
        # it is the nominal center, the constraints define the tolerated region.
        request.ik_request.pose_stamped.header.frame_id = PLANNING_FRAME
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = qx
        request.ik_request.pose_stamped.pose.orientation.y = qy
        request.ik_request.pose_stamped.pose.orientation.z = qz
        request.ik_request.pose_stamped.pose.orientation.w = qw

        constraints = Constraints()
        constraints.position_constraints.append(
            make_position_constraint(POSE_LINK, PLANNING_FRAME, x, y, z, tolerance=pos_tolerance)
        )
        constraints.orientation_constraints.append(
            make_orientation_constraint(
                POSE_LINK, PLANNING_FRAME, qx, qy, qz, qw,
                x_tolerance=xy_tolerance, y_tolerance=xy_tolerance, z_tolerance=z_tolerance)
        )
        request.ik_request.constraints = constraints

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or response.error_code.val != 1:
            return None

        js = response.solution.joint_state
        return dict(zip(js.name, js.position))

    def compute_ik_exact(self, x, y, z, qx, qy, qz, qw, seed_joint_names, seed_positions,
                        timeout_sec=0.5):
        """Exact-pose IK via /compute_ik with no tolerance constraints -- the
        service-based equivalent of moveit_py's RobotState.set_from_ik used
        by annulus_test.py's reachability screening, where exact convergence
        (not a tolerant window) is what's being measured. avoid_collisions is
        deliberately False here: callers do their own explicit ground-truth
        collision check afterward via check_state_validity(), kept separate
        from IK convergence on purpose (see solve_ik_filtered)."""
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik service not available")
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.ik_link_name = POSE_LINK
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = int(timeout_sec)
        request.ik_request.timeout.nanosec = int((timeout_sec % 1) * 1e9)

        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = list(seed_joint_names)
        request.ik_request.robot_state.joint_state.position = list(seed_positions)

        request.ik_request.pose_stamped.header.frame_id = PLANNING_FRAME
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = qx
        request.ik_request.pose_stamped.pose.orientation.y = qy
        request.ik_request.pose_stamped.pose.orientation.z = qz
        request.ik_request.pose_stamped.pose.orientation.w = qw
        # No `constraints` set -- exact pose match, not a tolerant region.

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or response.error_code.val != 1:
            return None

        js = response.solution.joint_state
        return dict(zip(js.name, js.position))

    # ---- Motion planning (replaces moveit_py's PlanningComponent) ----
    def plan_motion(self, goal_constraints, group_name=GROUP_NAME,
                    planning_time=10.0, planning_attempts=10,
                    velocity_scaling=1.0, acceleration_scaling=1.0):
        """moveit_msgs/GetMotionPlan (/plan_kinematic_path): plan -- but do
        NOT execute -- a joint-space trajectory from the robot's current
        state to goal_constraints (a list of moveit_msgs/Constraints, e.g.
        from make_joint_goal_constraints() or raw position/orientation
        constraints). Returns a trajectory_msgs/JointTrajectory, or None on
        failure. This is the plan-only equivalent of moveit_py's
        PlanningComponent.plan() -- callers still execute the returned
        trajectory themselves via arm_execute(), exactly as before."""
        if not self._motion_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/plan_kinematic_path service not available")
            return None

        request = GetMotionPlan.Request()
        mpr = request.motion_plan_request
        mpr.group_name = group_name
        # is_diff=True + empty joint_state: plan from the robot's actual
        # current state, same as moveit_py's set_start_state_to_current_state().
        mpr.start_state.is_diff = True
        mpr.goal_constraints = goal_constraints
        mpr.pipeline_id = "ompl"
        mpr.num_planning_attempts = planning_attempts
        mpr.allowed_planning_time = planning_time
        mpr.max_velocity_scaling_factor = velocity_scaling
        mpr.max_acceleration_scaling_factor = acceleration_scaling

        future = self._motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or response.motion_plan_response.error_code.val != 1:
            return None

        joint_trajectory = response.motion_plan_response.trajectory.joint_trajectory
        _ensure_monotonic_timing(joint_trajectory)
        return joint_trajectory

    # ---- State validity (replaces moveit_py's planning_scene_monitor) ----
    def check_state_validity(self, joint_dict, group_name=GROUP_NAME):
        """moveit_msgs/GetStateValidity (/check_state_validity): is this
        joint configuration self-collision-free? Unspecified joints (outside
        joint_dict, e.g. the gripper) are left at their live current value
        (is_diff=True) rather than an arbitrary default.

        Returns (valid: bool, contacts: list[ContactInformation]), or
        (None, None) if the service call itself failed."""
        if not self._state_validity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/check_state_validity service not available")
            return None, None

        request = GetStateValidity.Request()
        request.group_name = group_name
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(joint_dict.keys())
        request.robot_state.joint_state.position = list(joint_dict.values())

        future = self._state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            return None, None

        return response.valid, response.contacts


def make_joint_goal_constraints(joint_dict, tolerance=0.001):
    """moveit_msgs/Constraints built from {joint_name: value}, for use as a
    plan_motion() goal -- the service-call equivalent of moveit_py's
    arm.set_goal_state(robot_state=...)."""
    constraints = Constraints()
    for name, value in joint_dict.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = value
        jc.tolerance_above = tolerance
        jc.tolerance_below = tolerance
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    return constraints


# Fallback time parameterization, in rad/s -- deliberately conservative,
# well under joint_limits.yaml's 1.0 rad/s per-joint max (which itself
# defaults to a further 0.1 scaling factor project-wide). Only ever used as
# a safety net; see _ensure_monotonic_timing's docstring for when it kicks in.
_FALLBACK_MAX_JOINT_SPEED = 0.2  # rad/s
_FALLBACK_MIN_SEGMENT_SEC = 0.1  # floor per waypoint, avoids zero-length segments


def _duration_to_sec(duration):
    return duration.sec + duration.nanosec * 1e-9


def _sec_to_duration(seconds, duration):
    duration.sec = int(seconds)
    duration.nanosec = int((seconds % 1) * 1e9)


def _ensure_monotonic_timing(joint_trajectory):
    """Recompute strictly-increasing time_from_start for every waypoint if
    the planner returned a raw geometric path with no time parameterization
    applied at all (every point at time_from_start=0). Observed on ROS2
    Galactic's /plan_kinematic_path: the response_adapters config key that
    normally adds time parameterization (AddTimeOptimalParameterization) had
    to be dropped from ompl_planning.yaml entirely, because Galactic and
    Jazzy require opposite, mutually incompatible types for that parameter
    (a plain string vs. a string array) -- see ompl_planning.yaml's comment.
    Jazzy's own fallback still applies proper timing without it; Galactic's
    doesn't, and joint_trajectory_controller then rejects the trajectory
    outright ("Time between points 0 and 1 is not strictly increasing").

    This is a pure safety net: if the trajectory already has strictly
    increasing times (Jazzy, Gazebo, or once a real per-distro fix exists),
    this is a no-op. When it does need to act, it assigns each waypoint a
    time delta from the previous one based on the largest single-joint
    angular step and a conservative constant speed -- not true time-optimal
    parameterization, just enough to produce a valid, safely-paced
    trajectory for the controller to execute."""
    points = joint_trajectory.points
    if len(points) < 2:
        return

    already_monotonic = all(
        _duration_to_sec(points[i + 1].time_from_start) > _duration_to_sec(points[i].time_from_start)
        for i in range(len(points) - 1)
    )
    if already_monotonic:
        return

    t = 0.0
    _sec_to_duration(t, points[0].time_from_start)
    prev_positions = list(points[0].positions)
    for point in points[1:]:
        max_delta = max(
            (abs(a - b) for a, b in zip(point.positions, prev_positions)),
            default=0.0,
        )
        dt = max(_FALLBACK_MIN_SEGMENT_SEC, max_delta / _FALLBACK_MAX_JOINT_SPEED)
        t += dt
        _sec_to_duration(t, point.time_from_start)
        prev_positions = list(point.positions)


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


def _is_state_colliding(io_client, joint_dict, group_name=GROUP_NAME):
    """True if joint_dict (a {joint_name: value} arm configuration) is
    self-colliding, via MoveIt's /check_state_validity service -- the
    service-call equivalent of moveit_py's planning_scene_monitor-based
    scene.is_state_colliding(). Returns None if the service call failed."""
    valid, _contacts = io_client.check_state_validity(joint_dict, group_name=group_name)
    if valid is None:
        return None
    return not valid


def _is_near_joint_limit(state, margin=0.15):
    """Reject IK solutions where joint6output_to_joint6 is near its limit.
    KDL pegs it at -2.4434 rad even when seeded elsewhere; OMPL can't plan
    to a state wedged at a joint limit (no room to sample nearby states)."""
    # joint6output_to_joint6 limits from URDF: lower=-2.4434, upper=3.14159
    val = state.get("joint6output_to_joint6", None)  # state: dict, joint_name -> value
    if val is not None:
        if val < -2.4434 + margin or val > 3.14159 - margin:
            return True, "joint6output_to_joint6", val, -2.4434, 3.14159
    return False, None, None, None, None


def solve_ik_state(io_client, x, y, z, qx, qy, qz, qw):
    """Deterministic, downward-orientation IK for an OMPL goal state, using
    constraint-based IK (a small position sphere + orientation window) seeded
    from the robot's current state and then each IK_SEEDS entry in order.
    Returns the first {joint_name: value} goal state that converges and
    isn't pegged at joint6output's limit, or None if every seed fails.

    This used to call RobotState.set_from_ik, which solves for an EXACT
    position + orientation. On this non-redundant 6-DOF arm that effectively
    never converged for the downward grasp near (0, +/-0.25, ~0.2): every
    seed -- including 'current-state' sitting 8cm directly below a target the
    Cartesian planner then reaches at fraction=1.00 -- failed, because the
    wrist there is close enough to a singularity that KDL's Newton solve
    can't land on the exact orientation even though a solution plainly
    exists. The same targets converge immediately once the orientation is
    given a small window (IK_ORI_XY_TOLERANCE), which is exactly what the
    /compute_ik service does and what ik_probe.py used to confirm
    reachability in the first place. The straight-down orientation is not
    lost -- it's re-imposed exactly by the Cartesian descent that follows
    this hover. avoid_collisions=True in compute_ik already rejects
    self-colliding solutions, so no separate collision check is needed here.
    """
    joint_names = list(HOME_RADIANS.keys())

    current_state = io_client.current_joint_positions(joint_names)
    seeds = [("current-state", current_state)] + IK_SEEDS
    for label, seed in seeds:
        solution = io_client.compute_ik(
            x, y, z, qx, qy, qz, qw,
            seed_joint_names=joint_names,
            seed_positions=[seed[n] for n in joint_names],
        )
        if solution is None:
            print(f"[ik] '{label}' seed: IK did not converge")
            continue

        # The service echoes back every joint (arm + gripper); pull the arm
        # joints in group order to rebuild a goal state for the motion planner.
        try:
            joint_values = {n: solution[n] for n in joint_names}
        except KeyError as missing:
            print(f"[ik] '{label}' seed: solution missing joint {missing}, skipping")
            continue

        joints = [round(v, 3) for v in joint_values.values()]
        near_limit, lname, lval, llo, lhi = _is_near_joint_limit(joint_values)
        if near_limit:
            print(f"[ik] '{label}' seed: converged to {joints} BUT '{lname}'={lval:.3f} "
                  f"near limit [{llo:.3f},{lhi:.3f}], skipping")
            continue

        print(f"[ik] '{label}' seed: OK -> {joints}")
        return joint_values

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
                                 fine_zone=GRIPPER_FINE_ZONE, fine_step=GRIPPER_FINE_STEP,
                                 fine_step_duration=GRIPPER_FINE_STEP_DURATION,
                                 settle_sec=GRIPPER_SETTLE_SEC,
                                 effort_threshold=GRIPPER_EFFORT_THRESHOLD):
    """Close the gripper in small increments, stopping as soon as
    gripper_controller's measured effort exceeds effort_threshold (contact
    with the block) instead of always driving to `closed` regardless of
    what's in the way. Steps are coarse (`step`) until `target` reaches
    `fine_zone`, then switch to smaller, slower (`fine_step`,
    `fine_step_duration`) increments -- see GRIPPER_FINE_ZONE comment above
    for why a single step size can't both close quickly through open air and
    land precisely on first contact. Falls back to a single move straight to
    `closed` if no effort reading is available (e.g. the effort
    state_interface isn't configured), since that's strictly the old
    behavior, not a new failure mode. Returns True unless a gripper action
    call itself fails."""
    target = start
    got_any_reading = False

    while target > closed:
        in_fine_zone = target <= fine_zone
        this_step = fine_step if in_fine_zone else step
        this_duration = fine_step_duration if in_fine_zone else step_duration

        target = max(closed, target - this_step)
        if not io_client.gripper_move_to(target, duration_sec=this_duration):
            return False

        rclpy.spin_once(io_client, timeout_sec=settle_sec)
        effort = io_client.joint_effort("gripper_controller")

        zone = "fine" if in_fine_zone else "coarse"
        if effort is None:
            print(f"[gripper] target={target:.3f} ({zone})  effort=<no reading -- "
                  f"is the effort state_interface configured?>")
            continue

        got_any_reading = True
        print(f"[gripper] target={target:.3f} ({zone})  effort={effort:.3f}")
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


def go_home(io_client):
    """Return the arm to its designated home pose before planning anything else."""
    joint_trajectory = io_client.plan_motion([make_joint_goal_constraints(HOME_RADIANS)])
    if joint_trajectory is None:
        print("Planning to home pose FAILED.")
        return False

    print("Executing joint-space move to home pose...")
    return io_client.arm_execute(joint_trajectory)


def move_arm_to(io_client, x, y, z, lock_orientation=True):
    """Joint-space plan to a target position. Uses deterministic seeded IK
    when possible; falls back to OMPL constraint sampling if all seeds fail."""
    ik_state = None
    if lock_orientation:
        ik_state = solve_ik_state(io_client, x, y, z,
                                   GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW)

    if ik_state is not None:
        goal_constraints = [make_joint_goal_constraints(ik_state)]
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
        goal_constraints = [constraints]

    joint_trajectory = io_client.plan_motion(goal_constraints)
    if joint_trajectory is None:
        print(f"Planning FAILED for target ({x}, {y}, {z})")
        return False

    print(f"Executing joint-space move to ({x}, {y}, {z})...")
    return io_client.arm_execute(joint_trajectory)


def cartesian_move_to(io_client, x, y, z, min_fraction=0.90, allow_fallback=False):
    """Straight-line Cartesian move from the current pose to (x, y, z),
    holding the fixed downward grasp orientation throughout.

    If allow_fallback is True and the straight-line path falls short of
    min_fraction, falls back to a joint-space move_arm_to() instead of
    failing outright. Retreats near the edge of the validated reach
    envelope routinely land at ~0.85-0.89 -- just under the threshold -- and
    don't need a dead-straight path the way the delicate grasp/place
    descent does (a curved joint-space retreat can't knock a held block
    sideways the way a curved DESCENT could). Only pass allow_fallback=True
    for moves where that's true."""
    joint_values = io_client.current_joint_positions(list(HOME_RADIANS.keys()))
    print(f"[cartesian] joints at start: {[round(v, 4) for v in joint_values.values()]}")

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
        if not allow_fallback:
            return False
        print(f"[cartesian] falling back to joint-space move_arm_to for ({x}, {y}, {z})")
        return move_arm_to(io_client, x, y, z)

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
        ("Return to home pose", lambda: go_home(io_client)),
        ("Toggle gripper (pre-start)", lambda: toggle_gripper(io_client)),
        ("Move to pre-grasp (above pick)",
         lambda: move_arm_to(io_client, px, py, pz + APPROACH_HEIGHT)),
        ("Descend to grasp pose (Cartesian)",
         lambda: cartesian_move_to(io_client, px, py, pz)),
        ("Close gripper (grasp, stop on contact)", lambda: gripper_close_until_contact(io_client)),
        ("Retreat after grasp (Cartesian)",
         lambda: cartesian_move_to(io_client, px, py, pz + APPROACH_HEIGHT, allow_fallback=True)),
        ("Move to pre-place (above place)",
         lambda: move_arm_to(io_client, lx, ly, lz + APPROACH_HEIGHT)),
        ("Descend to place pose (Cartesian)",
         lambda: cartesian_move_to(io_client, lx, ly, lz)),
        ("Open gripper (release)", lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        ("Retreat after release (Cartesian)",
         lambda: cartesian_move_to(io_client, lx, ly, lz + APPROACH_HEIGHT, allow_fallback=True)),
        ("Return to home pose (final)", lambda: go_home(io_client)),
    ]

    for name, action in steps:
        print(f"\n=== {name} ===")
        ok = action()
        if ok is False:
            print(f"Step failed: {name}. Aborting sequence.")
            break
        time.sleep(0.5)

    print("\n=== Final return to home pose ===")
    go_home(io_client)

    print("\nPick-and-place sequence complete.")
    io_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()