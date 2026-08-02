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
#
# ZONE RADIUS: 9 in = 0.2286 m, moved in from 0.250 m on 2026-08-02.
#
# This is a PHYSICAL change -- the mats were re-taped closer to the base -- and
# the reason is reach, not neatness. Everything in this file and in
# tag_pick_place.py that failed on hardware failed by asking for a flange
# position a centimetre or two outside the workspace: the detection hover had to
# drop 0.280 -> 0.240 to reach the zone at all, and the multiview survey has to
# pull the flange IN from the zone centre because reaching outward is simply not
# available. Pulling the zone itself in buys that margin back everywhere at once,
# for free, instead of spending code on working around it.
#
# 0.2286 is exact (9 * 0.0254), not rounded, because the tag square, the block
# database and the print sheets are all dimensioned in inches -- keep the whole
# chain in one unit system so a 0.4 mm rounding error never has to be chased.
ZONE_RADIUS_M = 9 * 0.0254   # 0.2286
PICK_XYZ = (+0.000, +ZONE_RADIUS_M, 0.030 + SPAWN_HEIGHT_CORRECTION)
PLACE_XYZ = (+0.000, -ZONE_RADIUS_M, 0.040 + SPAWN_HEIGHT_CORRECTION)
# APPROACH_HEIGHT is how far above the grasp/place flange target to
# pre-position for the straight-down descent. It is HARD-CAPPED by the arm's
# reach, NOT a free choice: at a pick/place radius of 0.25m the flange could
# only reach at all up to z~=0.21 (measured with scripts/reach_probe.py --
# 0.21 works, 0.215 is already outside the workspace at ANY orientation).
# The zone has since moved in to 0.2286, so that ceiling is now the CONSERVATIVE
# side of the truth rather than the tight one; it has NOT been re-measured, so
# nothing below has been raised to spend the slack. Re-run reach_probe.py at
# 0.2286 before treating any of these numbers as headroom.
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

# Hard ceiling on any hover height, enforced rather than assumed.
#
# The comment above works out APPROACH_HEIGHT from a place flange target of
# 0.16. That assumption silently expired: the place target is now 0.175 (the
# grasp/place z offsets were re-measured when the blocks moved off a box onto
# a flat mat), which puts the place hover at 0.175 + 0.04 = 0.215 -- exactly
# the value reach_probe.py measured as outside the workspace at ANY
# orientation.
#
# The symptom was total and consistent: on 2026-07-27 every one of the 19 IK
# seeds failed for (0, -0.25, 0.215) on every run, INCLUDING bearing seeds
# sitting within 0.004 rad of the true base angle. A solver handed a seed on
# top of the answer that still cannot converge is being asked for a solution
# that does not exist. The OMPL fallback then satisfied its 4cm position
# sphere by parking the flange lower and tilted, which is also where the
# "gripper points a little to the side" and "never the same spot twice"
# behaviour came from -- constraint sampling returns a different branch each
# run.
#
# Clamping here rather than shrinking APPROACH_HEIGHT keeps the pick hover
# (0.195) unchanged and only pulls in the one that was over the line.
MAX_HOVER_Z = 0.205

# Convergence tolerance for an arm goal, radians. Raised 0.05 -> 0.07 on
# 2026-07-27 because 0.05 is tighter than this hardware's servo deadband.
#
# Evidence, from the bridge's own log during a failed "Retreat after release":
# it re-sent the held command at FULL SPEED four times and the reported error
# read 0.0504 rad on every single one, never changing by a digit, before the
# stall detector gave up. The joint physically cannot close that gap -- and
# the goal was then failed for missing 0.05 by 0.0004 rad, i.e. 0.02 degrees,
# which aborted an otherwise complete pick and place after the block had
# already been placed successfully.
#
# Deadband residuals seen across runs: 0.027, 0.0315, 0.0330, 0.0334, 0.0341,
# 0.0504. 0.07 clears the worst with ~40% margin.
#
# The cost is real: 0.07 rad is ~4 degrees, which at this arm's link lengths
# is up to ~1cm at the fingertips. That is absorbed by the gripper closing on
# contact rather than to a fixed position, but it is the reason not to raise
# this further -- a tolerance the arm cannot meet fails honest runs, and one
# far past the deadband stops catching real tracking failures.
ARM_SETTLE_TOLERANCE = 0.07

# PER-JOINT override, for joints where the global tolerance is the wrong question.
#
# 2026-07-29: a run aborted at "Move to pre-grasp" with joint6output_to_joint6 at
# -0.0712 rad against the 0.07 tolerance -- failing by 0.0012 rad, i.e. 0.07 deg.
# Every other joint was inside 0.022. Re-running worked. That is not a fluke, it
# is a joint sitting on the threshold: joint6output carries the largest dead-band
# residual of the six (+3.06 deg = 0.053 rad in earlier logs, 4.08 deg here), so
# it will keep landing either side of 0.07 and aborting good runs at random. Test
# 1's 'extended' posture failed the same way, by 0.07 deg against a 3.0 deg gate.
#
# Raising the GLOBAL tolerance to cover it is the wrong fix, because 0.07 rad is
# already ~1 cm at the fingertips and the pitch joints are exactly where that
# error becomes the grasp tilt this file spends so much effort on.
#
# joint6output_to_joint6 is different in kind, though: it is one of the two
# VERTICAL-axis joints, so it contributes exactly 0.000 deg of tilt (see the FK
# decomposition in APRIL_TAGS.md). Its error shows up as gripper YAW, not lean --
# on a square block that is nearly free, and Stage 3's rectangular blocks care
# about it only to the extent the jaws must span the short face. So it can be held
# to a looser standard than the joints that tilt the tool, and this is a real
# distinction rather than a convenient one.
#
# 0.09 rad = 5.2 deg: clears the worst residual seen with margin, still far short
# of a genuine tracking failure (the aborted run had moved 3.30 rad before
# stalling, so real failures are not subtle).
ARM_SETTLE_TOLERANCE_PER_JOINT = {
    "joint6output_to_joint6": 0.09,
}

# Mirrors mycobot_bridge.py's SETTLE_QUIET_PERIOD_SEC. Kept in sync BY HAND, the
# same way GRIPPER_OPEN/CLOSED_RAD already are -- the two files live in different
# packages and neither currently depends on the other. If you change it there,
# change it here.
#
# This exists because the two halves disagree about when a move is finished:
# _send_goal_and_wait is satisfied at ARM_SETTLE_TOLERANCE (0.07 rad), while the
# bridge will not attempt any correction until the command has been UNCHANGED
# for this long. See settle_pause().
SETTLE_QUIET_PERIOD_SEC = 1.0
# Extra time on top, for the settle to actually issue its moves and for the arm
# to execute them.
#
# Raised 1.0 -> 3.0s on 2026-07-29 to let the bridge's escalating settle bias run
# more than one attempt, then put BACK to 1.0s the same day. The bias it existed
# to serve is disabled (see mycobot_bridge.py's SETTLE_BIAS_ENABLED: gain 1.0 sits
# inside the servo dead band, gain 2.0 is the marginal-stability boundary, and
# nothing in between both moves the joint and converges).
#
# Widening the window is not neutral now that there is nothing to escalate. It let
# the divergent run reach "settle re-send 14/20", and every one of those attempts
# was a real ~4 deg command to the arm. With the bias off the re-sends are
# identical values the arm has already ignored, so extra attempts buy nothing and
# just add dead time to every move. The bridge's own stall detector gives up after
# SETTLE_MAX_STALLED anyway.
SETTLE_ACT_MARGIN_SEC = 1.0


def hover_z(target_z):
    """Hover height above target_z, clamped to the reachable ceiling."""
    requested = target_z + APPROACH_HEIGHT
    if requested > MAX_HOVER_Z:
        print(f"[pick_place] hover {requested:.3f} exceeds the reachable "
              f"ceiling {MAX_HOVER_Z:.3f} -- clamping (descent shortens to "
              f"{MAX_HOVER_Z - target_z:.3f}m)")
        return MAX_HOVER_Z
    return requested

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

# ---------------------------------------------------------------------------
# WHY A LEVEL FLANGE CAN STILL HOLD THE BLOCK AT AN ANGLE
# ---------------------------------------------------------------------------
# Recorded 2026-07-29 after the sag fix landed. /joint_states-derived flange tilt
# was 0.50 deg at the grasp and 0.19 deg at the place -- and the held block was
# still visibly tilted, more so at the place. Both observations are correct. They
# measure different things, and the difference is the point:
#
#   sag   -> the FLANGE is not vertical. Pose-dependent, visible in
#            /joint_states, fixed by SAG_PRECOMP_* (4.19 -> 0.50 deg, verified).
#   this  -> the flange IS vertical and the BLOCK is not. Everything past the
#            last encoder. Completely invisible to /joint_states, so no amount of
#            joint-space work can see it, let alone fix it.
#
# A theory that was WRONG, written down so it is not re-derived: the gripper is
# angular, every finger joint being revolute about one axis, and that axis is
# HORIZONTAL in world at both poses ([-0.025, +0.9997, -0.001] at the grasp),
# so pad swing would tilt the block degree for degree. It does not. The mimic
# tags make it a parallelogram linkage --
#     gripper_left3_to_gripper_left1  mimic gripper_controller x-1.0
# -- so gripper_left3 turns +theta and the pad turns -theta, netting exactly
# zero. Confirmed by FK: pad rotation 0.000 deg at every point in the travel
# (0.135, 0.0, -0.2325, -0.60). The pads translate. They never rotate.
#
# What the same check DID turn up, and what to investigate first: at the logged
# contact point (gripper_controller = -0.2325) the two pad links sit ~69 mm
# apart, against a 30 mm block. And `effort` reads 0.000 on every line of every
# log, so gripper_close_until_contact has only jaw POSITION LAG to work with --
# "CONTACT: jaw trailing its command by 0.0675 rad" is as consistent with the
# linkage binding as with the block. If contact is firing early, the block is
# held SLACK, which would explain a tilt that is worse at the place pose than the
# pick pose: a loose block swings and re-settles during the transit between them.
# A rigid mount error, by contrast, would be identical at both.
#
# THE OTHER CANDIDATE is a real mount offset: the URDF reaches the gripper via
# two hand-authored right angles,
#     joint6output_to_camera_flange   rpy = "1.5708 1.5708 0"
#     camera_flange_to_gripper_base   rpy = "0 1.5708 1.5708"
# and if the physical mount does not match them, "flange vertical" and "jaws
# vertical" differ by a constant -- which is exactly the original complaint that
# the tilt "is always the same, always very consistently that angle, and never
# goes away".
#
# The two are distinguished by one 5-second test: grip a block, then try to move
# it by hand. If it shifts, the grip is slack (fix the contact detection, not the
# geometry). If it is rock solid and still tilted, it is the mount, and the
# constants below correct it.
#
# These are a TOOL-FRAME correction, and that frame matters. The error is fixed
# relative to the gripper, so it must rotate with the gripper; SAG_PRECOMP_* is a
# world-frame effect and pre-multiplies instead. Applying a tool-frame error in
# the radial frame would cancel it at one pose and double it at the pose 180 deg
# opposite -- which is worth knowing, because grasp and place ARE ~180 deg apart
# here, and that is one candidate explanation for the pick/place asymmetry.
#
# MEASURING THEM: put the arm at the grasp pose, then sight the jaw faces against
# vertical from the front and from the side (phone level app against a jaw face
# is enough). Adjust the axis that matches the direction of lean; if the tilt
# doubles instead of vanishing, flip the sign.
#
# Both 0.0 = disabled, exactly the behaviour before this was added.
GRIPPER_MOUNT_TILT_X_DEG = 0.0   # about the flange's local X
GRIPPER_MOUNT_TILT_Y_DEG = 0.0   # about the flange's local Y

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
GRIPPER_EFFORT_THRESHOLD = 0.2   # N*m -- see below: never fires on real hardware

# Stall-based contact detection (2026-07-26). GRIPPER_EFFORT_THRESHOLD above
# cannot work against the real arm: pymycobot exposes no gripper force reading,
# so /joint_states carries a constant 0.0 placeholder for gripper_controller's
# effort and the threshold test never fires. Position readback does carry the
# signal -- a free jaw tracks the commanded value down, a jaw against a block
# stops advancing while the command keeps decreasing.
#
# GRIPPER_STALL_EPS must sit below one readback quantum (0.0075 rad = the
# 0-100 pymycobot gripper scale over the 0.75 rad jaw span) so real motion
# still registers, and GRIPPER_STALL_STEPS must be >1 so a single fine step
# (0.005 rad, i.e. under one quantum) can't be mistaken for contact on its own.
GRIPPER_STALL_EPS = 0.003        # rad of closing progress that counts as "moved"
GRIPPER_STALL_STEPS = 3          # consecutive stalled steps before declaring contact

# Primary contact signal: how far the jaw is allowed to trail its commanded
# position before we call it contact. Faster and gentler than stall counting,
# which needs GRIPPER_STALL_STEPS increments to be sure and squeezes that much
# harder in the meantime. From the first full grasp (2026-07-26), lag while
# closing through open air never exceeded +0.045 rad, then jumped the instant
# the block stopped the jaw:
#     free:    +0.0075 +0.030 +0.030 +0.030 +0.0375 ... +0.045 +0.045
#     blocked: +0.0675 +0.0975 +0.1275 +0.1575   <- stall count only fired here
# 0.06 sits above the free-running maximum and below the first blocked
# reading, so it fires 3 increments (0.09 rad of squeeze) earlier than stall
# detection did. Stall counting is kept as a backup for a jaw that creeps one
# quantum at a time instead of stopping cleanly.
GRIPPER_CONTACT_LAG = 0.06       # rad the jaw may trail its command

# Fine (slow, 0.005 rad) stepping near the closed end. DISABLED (2026-07-26):
# it was tuned in simulation for landing precisely on a 1 inch cube, and on
# real hardware a fine step is smaller than the 0.0075 rad readback quantum,
# so it cannot even be measured -- it just adds ~20 extra increments and
# roughly a minute per grasp. Re-enable once a real force/contact signal
# exists and precise closing actually buys something.
GRIPPER_FINE_ENABLED = False
# Per-increment timeout. The 60s default is right for an arm move but wrong
# here: a stalled increment waits the whole budget, ~40 times over, which is
# what made a single grasp hang for minutes before aborting.
GRIPPER_STEP_TIMEOUT = 3.0       # sec

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
#
# EXACT, not 4-decimal (fixed 2026-07-29). 0.7071 is not 1/sqrt(2); the
# resulting quaternion is not quite a unit quaternion and does not describe
# quite a straight-down rotation. Measured: the rounded constants asked the
# flange for a pose 0.5019 deg off vertical, so half a degree of the
# long-standing grasp tilt was baked into the TARGET before any solver or servo
# was involved. Costs nothing to make exact.
#
# This is NOT the main cause of that tilt -- the other ~3.2 deg is the pitch
# joints undershooting, see SAG_PRECOMP_RADIAL_DEG below -- but it is the one part
# that was pure arithmetic.
GRASP_QX = -math.sqrt(0.5)
GRASP_QY = math.sqrt(0.5)
GRASP_QZ = 0.0
GRASP_QW = 0.0

# GRAVITY SAG PRE-COMPENSATION (2026-07-29). Aim the grasp orientation slightly
# off vertical so that the arm's own sag brings it back TO vertical.
#
# Replaces the corrective-nudge approach in mycobot_bridge.py, which is disabled
# -- see SETTLE_BIAS_ENABLED there for the algebra. In short: a stationary joint
# will not move for a command delta smaller than its dead band, and the dead band
# here is LARGER than the error being corrected, so no after-the-fact nudge can
# work at any gain. Pre-compensation sidesteps that entirely, because the arm
# reaches this target as part of a full trajectory -- the joints are already in
# motion, so the dead band never arms.
#
# TWO components, in the arm's own frame at the target, not one:
#   radial     -- in the vertical plane through the base axis and the target.
#                 Positive = lean OUTWARD, away from the base.
#   tangential -- perpendicular to that, about the radial direction.
#                 Positive = lean along (z_hat x r_hat).
#
# The first version of this corrected radial only, and hardware showed why that
# was not enough. Measured tilt decomposed into the two components (deg):
#
#   pose             tilt    radial   tangential
#   grasp  before    4.189    -3.809     -1.743
#   place  before    5.049    -4.651     -1.964
#   grasp  radial-only run 1  2.019    +1.543     -1.301
#   grasp  radial-only run 3  2.246    +1.615     -1.561
#   place  radial-only run 1  2.319    -0.572     -2.247
#   place  radial-only run 3  2.197    -0.510     -2.137
#
# Radial went from -3.8/-4.7 to about zero: that part worked. TANGENTIAL was never
# touched (-1.7 -> -1.3, -2.0 -> -2.2) and is now the dominant residual. It is
# negative in all eight measurements, so it is exactly as systematic as radial
# was; correcting one axis and not the other just left the other one behind.
#
# This is also why the place pose looks visibly worse than the pick pose, which
# reads as the tilt "amplifying" through the sequence. It does not amplify: place
# simply has the larger tangential term (-2.2 vs -1.3) and always did.
#
# PAYLOAD. The radial residuals disagree in a way that is physically meaningful,
# not noise: grasp OVERSHOT (+1.54, wants less pre-comp) while place UNDERSHOT
# (-0.57, wants more), and the place descent is the one where the gripper is
# HOLDING A BLOCK. More mass on the end -> more sag -> more pre-compensation
# needed. So the correction carries a payload term rather than two unrelated
# per-pose constants, which is both better justified and generalises to any
# loaded/unloaded move instead of just these two hardcoded poses.
#
# Values below solve the four measurements for zero residual, using the measured
# radial response (1.16x at the grasp pose, 0.89x at the place pose) rather than
# assuming the commanded degree lands as a degree.
#
# Set both EMPTY values to 0.0 to disable entirely and recover prior behaviour.
#
# ZEROED 2026-08-02, BECAUSE THE DROOP IS NOW CORRECTED AT SOURCE.
# The previous values are kept immediately below; restoring them is a
# copy-paste and nothing else depends on them being zero.
#
#   SAG_PRECOMP_RADIAL_DEG = 3.27
#   SAG_PRECOMP_TANGENTIAL_DEG = 1.30
#   SAG_PRECOMP_PAYLOAD_RADIAL_DEG = 1.97
#   SAG_PRECOMP_PAYLOAD_TANGENTIAL_DEG = 0.95
#
# mycobot_bridge.py now applies a measured gravity feedforward in JOINT space
# on every streamed setpoint (see GRAVITY_FF_ENABLED there). At this pose that
# cancels 2.92 deg of the 4.19 deg tilt these constants were fitted to cancel
# empirically. Leaving both corrections live would over-correct by roughly the
# amount the feedforward removes, i.e. tilt the jaws ~3 deg the OTHER way.
#
# ZEROED RATHER THAN SCALED TO 30%, DELIBERATELY. Two corrections for one
# effect cannot be tuned at the same time -- any residual could belong to
# either.
#
# BE HONEST ABOUT WHAT THIS TRADES AWAY. These constants were VERIFIED to take
# the tilt from 4.19 to 0.50 deg at the grasp (see the note above, 2026-07-29).
# The feedforward is predicted to remove 2.92 of that 4.19 deg, so on tilt
# ALONE this is very likely a step backwards, to something like 1.3 deg.
#
# It is still the right trade, because the two corrections do not fix the same
# thing:
#
#   SAG_PRECOMP_* rotates the TARGET ORIENTATION only. It aims the flange off
#       vertical so sag brings it back. It never moves the target position, so
#       the ~8.6 mm the arm sags DOWNWARD at the grasp is completely
#       uncorrected by it -- and that is the error that makes the jaws close
#       on nothing. Tilt was never the thing breaking the grasp.
#   The feedforward corrects the JOINT ANGLES, so it fixes position and
#       orientation together, at every pose, not at two fitted ones.
#
# 1 deg of residual tilt is 1.0 mm of jaw offset over the 0.056 m flange-to-jaw
# lever. Giving up ~0.8 deg of tilt (0.8 mm) to recover 8.6 mm of height is
# worth it by an order of magnitude.
#
# WHAT THE LEFTOVER PROBABLY IS -- and this is a guess, not a measurement. The
# feedforward corrects only the SYMMETRIC half of the joint error, the half
# that does not reverse with travel direction. The ANTISYMMETRIC half
# (friction, dead zone, lost motion) runs 0.4-0.9 deg per pitch joint and is
# untouched by any feedforward. That is the right order of magnitude for the
# missing ~1.3 deg, and unlike a mount offset it depends on which way the arm
# drove in.
#
# NEXT STEP, and it needs the arm: ff_verify.py, run twice with the feedforward
# toggled. If the remainder is a pose-independent constant, add it as a fixed
# offset here. If it flips sign with approach direction, it is the
# antisymmetric term and belongs in the unidirectional-approach work instead.
# If it still varies with reach, the feedforward coefficients are wrong rather
# than incomplete.
SAG_PRECOMP_RADIAL_DEG = 0.0           # empty gripper
SAG_PRECOMP_TANGENTIAL_DEG = 0.0       # empty gripper
# Added ON TOP of the above while a block is held.
SAG_PRECOMP_PAYLOAD_RADIAL_DEG = 0.0
SAG_PRECOMP_PAYLOAD_TANGENTIAL_DEG = 0.0

# HONEST LIMITS OF THIS MODEL, because the goal is ~1 mm and this will not get
# there on its own:
#   - It is fitted to TWO positions, both at 0.249 m reach. Reach and height
#     dependence are entirely unmeasured, so it is not known to hold anywhere
#     else in the workspace -- and Stage 1 onward grasps at arbitrary positions
#     inside the zone.
#     THIS BIT HAS NOW ACTUALLY EXPIRED, 2026-08-02: the zones moved in to
#     ZONE_RADIUS_M = 0.2286, so no grasp happens at the fitted reach any more.
#     Direction of the error is known even though the size is not -- less reach
#     means a shorter gravity lever, so the true sag at 0.2286 is SMALLER than at
#     0.249 and these constants now OVER-correct. Left unchanged deliberately
#     rather than scaled by a guessed cos/lever ratio: 2.5 cm is ~10% of reach,
#     the residuals being corrected are 1-3 deg, so the error introduced is a
#     fraction of a degree -- smaller than the scatter the fit was made against.
#     Re-fit from a real run before trusting sub-millimetre placement.
#   - The payload term is fitted to ONE block mass.
#   - The flange-to-jaw lever is 0.056 m, so 1 deg of residual tilt is 1.0 mm of
#     jaw offset, and a 30 mm block tilted 1 deg has its top face 0.5 mm out of
#     level. Stacking compounds that per course.
# The scalable answer is to stop modelling and start measuring: the wrist camera
# is rigid to the flange, so the AprilTag homography taken at hover can report
# the actual flange tilt in situ, per pose, every time. That needs the camera
# intrinsics that camera.launch.py's unused camera_info_url hook is already
# there for. See "Stage 0b.4" in APRIL_TAGS.md.

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
# 0.10 rad (~5.7 deg). Tightened to 0.04 on 2026-07-28 to try to fix the
# visible gripper tilt, and REVERTED the same day because it fixed nothing and
# cost convergence. Do not try this again without reading the next paragraph.
#
# THIS CONSTANT DOES NOT CONTROL THE GRASP TILT. It only shapes the IK search
# for the HOVER joint state. The orientation the gripper actually holds is set
# by make_orientation_constraint's x_tolerance/y_tolerance defaults (0.15 rad,
# ~8.6 deg), which is what both move_arm_to and cartesian_move_to pass to the
# planner. Tightening this one narrowed the seed search and left the tilt
# untouched -- confirmed on hardware: identical chosen solution
# [1.828, -0.746, -0.605, -0.22, -0.0, 1.828] before and after, with more seeds
# failing to converge and no visible change to the arm.
#
# The tilt is also visible with ALL JOINTS AT ZERO, where no IK or planner
# tolerance is involved at all -- so its cause is mechanical (sag under the
# camera+gripper mass), servo deadband, or the URDF/mount mismatch, and no
# tolerance anywhere will remove it.
IK_ORI_XY_TOLERANCE = 0.10       # rad, tilt allowed off straight-down
IK_ORI_Z_TOLERANCE = 0.15        # rad, yaw window about the approach axis
IK_SERVICE_TIMEOUT = 0.3         # sec, per-seed /compute_ik solve budget

# Designated home pose (matches reset_arm.py / config/initial_positions.yaml),
# originally specified in degrees and converted to radians here.
HOME_DEGREES = {
    "joint2_to_joint1": 0,
    "joint3_to_joint2": 0,
    "joint4_to_joint3": 0,
    "joint5_to_joint4": 0,
    "joint6_to_joint5": 0,
    "joint6output_to_joint6": -45,
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
#
# -45 deg, set 2026-07-28. This is now MEASURED, not guessed. Composing the
# two fixed transforms below the flange (joint6output_to_camera_flange then
# camera_flange_to_gripper_base) against GRASP_Q* puts gripper_base's X axis
# -- the finger-opening axis -- at exactly +45.00 deg from world +X, dead
# horizontal, with the approach axis at -1.000 z (straight down). That 45 deg
# is why the target cube had to be sat on the floor rotated 45 deg to be
# grasped square. -45 here brings the jaw axis to 0.00 deg from world +X and
# leaves the straight-down component untouched.
#
# NOTE this is NOT the same 45 deg as HOME_DEGREES' joint6output = -45. That
# one is the physical tool mount sitting askew at the servo's zero, and it
# lives in joint space. This one is the fixed rotation baked into the mount's
# geometry, and it lives in the world frame. They are numerically equal
# because they are two views of the same piece of hardware, but changing one
# does nothing to the other -- the grasp yaw is an absolute world quaternion
# and never consults the home pose.
GRIPPER_YAW_DEG = -45.0


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


# How far the base must rotate PAST the straight-line bearing to the target,
# in radians. The gripper's fingertips sit laterally offset from the flange
# axis, so pointing the base exactly at the target puts the FLANGE on the
# bearing and the jaw beside it; the base overshoots to compensate.
#
# Measured from real converged solutions on 2026-07-27: target (0, +0.25)
# has bearing atan2(0.25, 0) = +1.5708 and solves at joint1 = +1.828, and
# target (0, -0.25) has bearing -1.5708 and solves at joint1 = -1.31. Both
# are 0.257 rad from their bearing, in the direction that increases |joint1|
# for +y and decreases it for -y -- i.e. a consistent +0.257 offset.
_IK_BEARING_OFFSET = 0.257

# joint6output shift caused by GRIPPER_YAW_DEG, in radians. Under the
# downward grasp the flange's own Z axis points at world -Z, so turning
# joint6output by +d yaws the gripper by -d about world +Z: the joint offset
# is the NEGATIVE of the world yaw. Verified against the real converged
# solution [1.828, -0.746, -0.605, -0.22, -0.0, 1.828] logged on hardware
# 2026-07-27 -- at j6out 1.828 the jaw sits at +45.00 deg from world +X, at
# 1.828 + 0.7854 it sits at 0.00 deg, and at 1.828 - 0.7854 at +90.00 deg.
# Derived from GRIPPER_YAW_DEG rather than written as a literal so that
# changing the grasp yaw moves the seeds with it; seeding joint6output a full
# 45 deg from the answer is precisely the local-solver miss described below.
_GRASP_YAW_JOINT_OFFSET = -math.radians(GRIPPER_YAW_DEG)


# ---------------------------------------------------------------------------
# PER-BLOCK GRASP YAW (added 2026-07-28 for the AprilTag feature)
# ---------------------------------------------------------------------------
# Everything above holds the gripper at ONE fixed world yaw for every grasp,
# which is right when the block is placed by hand to match the jaws. Once a
# camera reports where the block is AND how it is rotated, the jaws have to
# turn to meet it, and the yaw becomes per-move rather than per-run.
#
# block_yaw_deg is that per-move rotation, in the WORLD frame, and it composes
# ON TOP of GRIPPER_YAW_DEG. The two are different things and must not be
# collapsed into one constant:
#
#   GRIPPER_YAW_DEG   the tool sits 45 deg askew on its mount. A property of
#                     the hardware. Fixed forever, same for every grasp.
#   block_yaw_deg     how far this particular block is rotated on the table.
#                     Different for every grasp, and 0.0 for every caller that
#                     predates this feature.
#
# Every function below takes block_yaw_deg=0.0 as a default, which reproduces
# the previous fixed-yaw behaviour EXACTLY -- gripper_yaw_quat(GRIPPER_YAW_DEG
# + 0.0) is the same quaternion as the GRIPPER_LOCK_Q* globals. That matters:
# reset_arm.py, annulus_test.py, collision_contacts.py and this script's own
# main() all keep working unchanged, and the TESTS.md characterization baseline
# does not move.
def sag_precomp_angles(holding_block=False):
    """(radial_deg, tangential_deg) of pre-compensation to apply."""
    radial = SAG_PRECOMP_RADIAL_DEG
    tangential = SAG_PRECOMP_TANGENTIAL_DEG
    if holding_block:
        radial += SAG_PRECOMP_PAYLOAD_RADIAL_DEG
        tangential += SAG_PRECOMP_PAYLOAD_TANGENTIAL_DEG
    return radial, tangential


def sag_precomp_quat(x, y, holding_block=False, angles=None):
    """World-frame rotation that tips the downward approach axis against the
    arm's gravity sag at target (x, y) -- see SAG_PRECOMP_RADIAL_DEG.

    Pre-multiplied onto the grasp quaternion, so it composes with the block yaw
    rather than being applied in the already-rotated tool frame.

    Both rotation axes are horizontal and derived from r_hat, the direction from
    the base axis out to the target:
      radial     about (r_hat x z_hat) -- swings the tool away from the base.
      tangential about  r_hat          -- swings it along (z_hat x r_hat).
    Rodrigues on a = -z_hat confirms both signs: rotating -z_hat about r_hat by
    +T gives -z_hat*cos(T) + (z_hat x r_hat)*sin(T), i.e. +T of tangential lean.

    Returns identity when there is nothing to do, including for a target on the
    base axis where r_hat -- and therefore any notion of "outward" -- is undefined.
    """
    if angles is None:
        angles = sag_precomp_angles(holding_block)
    radial_deg, tangential_deg = angles
    r = math.hypot(x, y)
    if r < 1e-6 or (not radial_deg and not tangential_deg):
        return (0.0, 0.0, 0.0, 1.0)
    rx, ry = x / r, y / r

    def about(axis, deg):
        half = math.radians(deg) / 2.0
        s = math.sin(half)
        return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))

    # r_hat x z_hat = (ry, -rx, 0) for unit r_hat in the XY plane.
    q_radial = about((ry, -rx, 0.0), radial_deg)
    q_tangential = about((rx, ry, 0.0), tangential_deg)
    return quat_multiply(q_tangential, q_radial)


def grasp_quat_for(block_yaw_deg=0.0, x=None, y=None, holding_block=False):
    """The grasp quaternion for a block rotated block_yaw_deg about world +Z.

    Reads GRIPPER_YAW_DEG at CALL time, not at import time, so --gripper-yaw-deg
    still takes effect (gripper_yaw_quat's own default argument is bound at def
    time and would not).

    x, y are the TARGET position. Supplying them enables the gravity sag
    pre-compensation, which needs to know which way "outward" is. Omitting them
    (the default) returns the uncompensated orientation, so any caller that does
    not care about sag -- or any pose where it does not apply -- is unchanged.

    holding_block adds the payload term: a loaded gripper sags measurably more
    than an empty one, which is the difference between the pick and place descents.
    """
    if not block_yaw_deg:
        base = (GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW)
    else:
        base = gripper_yaw_quat(GRIPPER_YAW_DEG + block_yaw_deg)
    # Mount tilt is constant in the TOOL frame, so it POST-multiplies -- unlike
    # the sag correction, which is a world-frame effect and pre-multiplies. See
    # GRIPPER_MOUNT_TILT_X_DEG for why the frame is the whole question here.
    for axis, deg in ((0, GRIPPER_MOUNT_TILT_X_DEG), (1, GRIPPER_MOUNT_TILT_Y_DEG)):
        if not deg:
            continue
        half = math.radians(deg) / 2.0
        v = [0.0, 0.0, 0.0]
        v[axis] = math.sin(half)
        base = quat_multiply(base, (v[0], v[1], v[2], math.cos(half)))
    if x is None or y is None:
        return base
    # World-frame correction: pre-multiply, so it composes with the yaw rather
    # than being applied in the (already rotated) tool frame.
    return quat_multiply(sag_precomp_quat(x, y, holding_block), base)


def _bearing_seeds(x, y, block_yaw_deg=0.0):
    """Seeds whose base rotation actually points at the target.

    THIS IS THE FIX for IK converging only about half the time (2026-07-27).
    Every entry in IK_SEEDS puts joint2_to_joint1 within +/-0.33 rad of zero
    -- 0.324, HOME's 0.035, 0.0, and the mirrored negatives -- while the real
    solutions for the pick and place poses need +1.828 and -1.31. KDL is a
    LOCAL solver: it converges to the basin nearest its seed, so from 1.5 rad
    away it only landed when its internal random restarts happened to wander
    across, which is exactly the coin-flip behaviour observed. The mirrored
    seeds did not help, because -0.324 is no closer to -1.31 than +0.324 is.

    The base angle is the one joint that needs no numeric solve at all: to
    reach a point the base must turn to face it. Seeding it analytically puts
    KDL 0.26 rad from the answer instead of 1.5.

    joint6output_to_joint6 is set to match joint2_to_joint1 because that is
    what every converged solution does here -- the wrist counter-rotates by
    the base angle to hold the fixed grasp yaw, e.g. [1.828, ..., 1.828] and
    [-1.31, ..., -1.31] -- plus _GRASP_YAW_JOINT_OFFSET, the constant shift
    that GRIPPER_YAW_DEG puts on every solution. Those logged pairs were
    measured at GRIPPER_YAW_DEG = 0; the offset carries them forward.

    The bearing itself, plus the offset applied both ways, so a target whose
    offset runs the other way (or a different gripper geometry later) is
    still covered rather than depending on _IK_BEARING_OFFSET being exact.
    """
    bearing = math.atan2(y, x)
    candidates = [
        ("bearing+offset", bearing + math.copysign(_IK_BEARING_OFFSET, bearing)),
        ("bearing", bearing),
        ("bearing-offset", bearing - math.copysign(_IK_BEARING_OFFSET, bearing)),
    ]

    # Elbow/wrist shapes worth trying at each base angle. Reuses the two
    # confirmed-good bends rather than inventing new ones -- only the base
    # rotation was ever wrong.
    shapes = [
        ("elbow-down", {"joint3_to_joint2": -0.746, "joint4_to_joint3": -0.605,
                        "joint5_to_joint4": -0.220, "joint6_to_joint5": 0.0}),
        ("elbow-up", {"joint3_to_joint2": -1.307, "joint4_to_joint3": 0.605,
                      "joint5_to_joint4": -0.869, "joint6_to_joint5": 0.0}),
    ]

    # The wrist offset must track the yaw the gripper is actually being asked to
    # hold, not just the mount correction. Seeding joint6output for a straight-
    # on grasp while solving for a block rotated 40 deg puts KDL 0.7 rad from
    # the answer -- the same local-solver miss described above, reintroduced by
    # the back door. Recomputed here rather than read from the module-level
    # _GRASP_YAW_JOINT_OFFSET so a per-move yaw moves the seeds with it.
    yaw_joint_offset = -math.radians(GRIPPER_YAW_DEG + block_yaw_deg)

    seeds = []
    for base_label, base in candidates:
        for shape_label, shape in shapes:
            seed = dict(shape)
            seed["joint2_to_joint1"] = base
            seed["joint6output_to_joint6"] = base + yaw_joint_offset
            seeds.append((f"{base_label}/{shape_label}", seed))
    return seeds


class RobotIOClient(Node):
    """Handles gripper open/close (action), arm trajectory execution (action),
    Cartesian path / IK / motion planning / state validity (services). Talks
    only to an externally-launched move_group over plain ROS2 services and
    actions -- no moveit_py, so this works against any ROS2 distro that has
    MoveIt2, regardless of whether moveit_py bindings were ever packaged for
    it (see build_motion_plan_request / check_state_validity below)."""

    def __init__(self):
        super().__init__("robot_io_client")
        self._gripper_action_name = "/gripper_group_controller/follow_joint_trajectory"
        self._arm_action_name = "/arm_group_controller/follow_joint_trajectory"
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, self._gripper_action_name
        )
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, self._arm_action_name
        )
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._motion_plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._state_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._joint_efforts = {}
        self._joint_positions = {}
        # Action clients whose DDS writer has already been given time to match
        # -- see _DDS_MATCH_SETTLE_SEC / _deliver_goal.
        self._warmed_clients = set()
        self._last_joint_state_monotonic = 0.0
        self._joint_state_recreates = 0
        self._create_joint_state_sub()

    def _on_joint_state(self, msg):
        self._last_joint_state_monotonic = time.monotonic()
        for name, effort in zip(msg.name, msg.effort):
            self._joint_efforts[name] = effort
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position

    def joint_effort(self, joint_name):
        """Latest effort reading for joint_name from /joint_states, or None
        if it hasn't been received yet (e.g. no "effort" state_interface
        configured for that joint)."""
        return self._joint_efforts.get(joint_name)

    def _create_joint_state_sub(self):
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def recreate_joint_state_sub_if_stale(self, stale_after_sec=3.0):
        """Rebuild the /joint_states subscription if it has gone quiet.

        The startup recreate in wait_for_joint_states only covers a reader that
        never matched. The same one-sided match failure can happen at ANY time
        -- and mid-run it is worse, because every convergence check silently
        stops updating while the goal quietly burns its full 60s timeout and is
        then reported as a tracking failure the arm never had.

        /joint_states publishes at 50Hz, so 3s of silence is ~150 missed
        messages: unambiguous, and far longer than any read stall the bridge
        can produce (its own worst case is ~0.3s)."""
        if self._last_joint_state_monotonic <= 0.0:
            return False  # nothing received yet -- wait_for_joint_states owns that
        if time.monotonic() - self._last_joint_state_monotonic < stale_after_sec:
            return False

        self._joint_state_recreates += 1
        print(f"[joint_states] STALE: no message for "
              f"{time.monotonic() - self._last_joint_state_monotonic:.1f}s "
              f"({self.count_publishers('/joint_states')} publisher(s) visible) "
              f"-- recreating the subscription (recreate "
              f"#{self._joint_state_recreates} this run)")
        self.destroy_subscription(self._joint_state_sub)
        self._create_joint_state_sub()
        # Reset the clock so the next check gives the new reader time to match
        # instead of firing again immediately.
        self._last_joint_state_monotonic = time.monotonic()
        return True

    def describe_joint_state_publishers(self):
        """QoS and identity of every discovered /joint_states publisher.

        count_publishers() counts publishers on the TOPIC regardless of
        whether their QoS is compatible with ours, so "1 publisher visible,
        0 messages" is exactly what an unmatched endpoint looks like. This
        prints enough to tell an incompatibility (a durability/reliability
        mismatch, which would be permanent) apart from a failed match (which
        recreating the subscription can recover)."""
        try:
            infos = self.get_publishers_info_by_topic("/joint_states")
        except Exception as exc:
            return f"    <could not query publisher info: {exc!r}>"
        if not infos:
            return "    <no publishers discovered at all>"
        lines = []
        for info in infos:
            qos = info.qos_profile
            lines.append(
                f"    node={info.node_name} reliability={qos.reliability.name} "
                f"durability={qos.durability.name} depth={qos.depth}")
        return "\n".join(lines)

    def wait_for_joint_states(self, timeout_sec=30.0, recreate_after_sec=6.0):
        """Block until the first /joint_states message arrives. True if one
        did.

        Separate from current_joint_positions so callers can distinguish "no
        data yet" from "data, but this joint is absent" --
        current_joint_positions returns 0.0 for both, which is
        indistinguishable from a joint genuinely at zero.

        RECREATES THE SUBSCRIPTION while waiting. Observed 2026-07-27: runs
        fail with `1 publisher(s) visible` and zero messages, while the robot
        has all three controllers active and is publishing normally. So the
        writer is discovered but the reader never matches it -- a one-sided
        endpoint match on this unicast link. This is the same failure mode
        _deliver_goal already recovers from for action goals, and by the same
        means: tearing the endpoint down and rebuilding it forces a fresh
        announcement instead of waiting on a handshake that has already been
        missed. Waiting longer alone does not help, because nothing retries."""
        end_time = time.monotonic() + timeout_sec
        next_recreate = time.monotonic() + recreate_after_sec
        attempts = 0
        while not self._joint_positions and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self._joint_positions and time.monotonic() >= next_recreate:
                attempts += 1
                print(f"[joint_states] no data after "
                      f"{recreate_after_sec * attempts:.0f}s "
                      f"({self.count_publishers('/joint_states')} publisher(s) "
                      f"visible) -- recreating the subscription to force a "
                      f"fresh DDS match (attempt {attempts})")
                self.destroy_subscription(self._joint_state_sub)
                self._create_joint_state_sub()
                next_recreate = time.monotonic() + recreate_after_sec
        if self._joint_positions and attempts:
            print(f"[joint_states] recovered after {attempts} "
                  f"subscription recreate(s)")
        return bool(self._joint_positions)

    def current_joint_positions(self, joint_names, timeout_sec=5.0):
        """Latest /joint_states positions for joint_names, waiting for the
        first message to arrive if none has been received yet. Replaces the
        old planning_scene_monitor-based current-state read (moveit_py) --
        the live /joint_states topic already carries the same values."""
        # time.monotonic(), not self.get_clock() -- this node runs with
        # use_sim_time:=true (needed elsewhere for Gazebo), and with no
        # /clock publisher in the real-hardware split-compute setup,
        # self.get_clock().now() never advances at all, which silently
        # turns this into an infinite loop instead of a 5s wait. See
        # _send_goal_and_wait's comment for the same bug found there.
        end_time = time.monotonic() + timeout_sec
        while not self._joint_positions and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        return {n: self._joint_positions.get(n, 0.0) for n in joint_names}

    def _spin_until_complete(self, future, timeout_sec=30.0, what=""):
        """rclpy.spin_until_future_complete() with a bounded timeout.
        Confirmed on real split-compute hardware (mars planning, robot
        executing over the Cyclone DDS unicast link): an action's goal
        acceptance can arrive fine while its RESULT message never does, even
        though the robot-side controller genuinely finished ("Goal reached,
        success!" in ros2_control_node's log) -- some message classes are
        just less reliable cross-machine on this DDS setup than others
        (matches the already-documented flakiness of the
        `ros2 control list_controllers` service). Without a timeout here,
        that silently hangs the whole script forever with no way to tell
        what's stuck. Returns future.result(), or None on timeout (logged)."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error(
                f"Timed out after {timeout_sec}s waiting for {what or 'a response'}")
            return None
        return future.result()

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
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        return self._send_goal_and_wait("_arm_client", self._arm_action_name,
                                        goal, "arm")

    # How long to keep spinning after wait_for_server() reports the action
    # server present, before writing the first request to it.
    #
    # wait_for_server() only consults the local ROS GRAPH CACHE. It returns
    # true as soon as discovery has told this node that a server exists --
    # which on this link happens within milliseconds of process start, long
    # before the underlying DDS request writer here has finished MATCHING the
    # robot's request reader across the unicast WAN link. ROS2 action/service
    # requests use RELIABLE + VOLATILE QoS, and a volatile writer silently
    # DISCARDS any sample written while no reader is matched. There is no
    # error, no retry, and no log line anywhere -- the request simply
    # evaporates.
    #
    # PAID ONCE PER ACTION CLIENT, NOT PER GOAL (2026-07-26). This was
    # originally applied to every send, on the theory that a matching race was
    # what made goals vanish. That theory was wrong -- the real cause was
    # RTPS messages exceeding the MTU (see cyclonedds_galactic.xml) -- and at
    # ~30 goals per pick-and-place run a per-goal settle was costing ~45s of
    # pure sleeping, most of it inside the gripper closing loop.
    #
    # A brief settle on FIRST use of a client is still worth keeping: that is
    # the only moment the underlying DDS writer genuinely has not matched yet,
    # and _deliver_goal's retry covers the rare case where it still races.
    _DDS_MATCH_SETTLE_SEC = 1.5

    def _deliver_goal(self, client_attr, action_name, goal, label, attempts=3):
        """Get a FollowJointTrajectory goal ACCEPTED by the controller,
        retrying across the known-lossy cross-machine DDS link.

        Returns the accepted ClientGoalHandle, or None if every attempt
        failed. Retrying (rather than one-shot) is the whole point: the
        dominant failure mode here is a request silently dropped by an
        unmatched volatile writer, and by the time a second request goes out
        the matching has completed, so attempt 2 succeeds where attempt 1
        vanished.

        Session 2 deleted the previous retry logic on the reasoning that
        "goal delivery itself has never been the problem, only completion
        detection". That conclusion came from misattributed log timestamps
        (see the session log): the goals believed to prove delivery worked
        were actually a LATER, trivial go_home goal. Goal delivery is the
        problem."""
        for attempt in range(attempts):
            client = getattr(self, client_attr)

            # 20s, not 5s: cross-machine action-server discovery over the
            # split-compute Cyclone DDS unicast link has repeatedly needed
            # more than 5s in practice (confirmed: the action was genuinely
            # reachable via `ros2 action list` moments after a 5s
            # wait_for_server() timed out) -- not a real unavailability, just
            # slow first-discovery.
            if not client.wait_for_server(timeout_sec=20.0):
                self.get_logger().error(
                    f"{label} action server {action_name} not available")
                return None

            # Let DDS matching complete, but only the first time this client is
            # used (or the first time after a retry recreated it) -- see
            # _DDS_MATCH_SETTLE_SEC.
            if client_attr not in self._warmed_clients:
                settle_until = time.monotonic() + self._DDS_MATCH_SETTLE_SEC
                while time.monotonic() < settle_until:
                    rclpy.spin_once(self, timeout_sec=0.05)
                self._warmed_clients.add(client_attr)

            goal_future = client.send_goal_async(goal)
            handle = self._spin_until_complete(
                goal_future, timeout_sec=10.0,
                what=f"{label} goal acceptance (attempt {attempt + 1}/{attempts})")

            if handle is not None and handle.accepted:
                print(f"[{label}] goal ACCEPTED by the controller "
                      f"(attempt {attempt + 1}) -- delivery confirmed, now "
                      f"watching /joint_states for real motion")
                return handle

            if handle is not None and not handle.accepted:
                # A real rejection is a decision, not a dropped packet --
                # resending an identical goal will just be rejected again.
                self.get_logger().error(
                    f"{label} goal: controller REJECTED the goal (it arrived "
                    f"fine; the controller refused it -- check joint names, "
                    f"waypoint timing, and the robot-side log)")
                return None

            self.get_logger().warn(
                f"{label} goal: no acceptance response on attempt "
                f"{attempt + 1}/{attempts} -- the request was almost certainly "
                f"dropped before reaching the robot (no 'Received new action "
                f"goal' will appear in its log). Recreating the ActionClient "
                f"and retrying.")

            # Recreate the client as the recovery step ONLY, not before every
            # goal. Recreating tears down and rediscovers 5 DDS entities,
            # which on this link is precisely the expensive, race-prone
            # operation -- doing it unconditionally per goal (the 2026-07-26
            # "Fix C") is what pushed the failure all the way forward onto
            # goal #1. Keeping the client long-lived and only recreating
            # after a failure keeps the escape hatch for the "3-goal wall"
            # without paying rediscovery on every single send.
            client.destroy()
            setattr(self, client_attr,
                    ActionClient(self, FollowJointTrajectory, action_name))
            # Fresh client: it has to re-match, so re-arm the one-off settle.
            self._warmed_clients.discard(client_attr)

        self.get_logger().error(
            f"{label} goal: NOT ACCEPTED after {attempts} attempts -- goal "
            f"delivery to the robot is failing, NOT the hardware failing to "
            f"move. Check the robot's log for 'Received new action goal'.")
        return None

    def _send_goal_and_wait(self, client_attr, action_name, goal, label,
                            settle_tolerance=ARM_SETTLE_TOLERANCE, timeout_sec=60.0,
                            log_failure=True):
        """settle_tolerance default 0.02 -> 0.05 rad (2026-07-26): confirmed
        on real hardware that the arm consistently settles ~0.014-0.031 rad
        away from the commanded target even when the controller reports
        "Goal reached, success!" -- real mechanical precision on this arm
        (backlash, calibration) rather than a detection bug (target values
        and /joint_states updates both looked correct; the gap was simply
        larger than 0.02 rad allowed for). 0.05 rad (~2.9 deg) comfortably
        covers that gap while still catching a genuine non-move.

        Send a FollowJointTrajectory goal, then detect completion by
        polling /joint_states for convergence to the goal's final target --
        NOT via the action's /follow_joint_trajectory/_action/status topic
        or send_goal_async()/get_result_async() futures.

        Confirmed on real split-compute hardware (2026-07-26), with a
        controlled back-to-back test (10 identical small goals in a row):
        goals consistently reach the robot, get accepted, and execute
        successfully WITHIN SECONDS every single time (robot-side log
        always shows Received -> Accepted -> "Goal reached, success!"
        promptly) -- the goal delivery itself is not the flaky part. What's
        unreliable is specifically the /action/status update finding its
        way back to mars afterward: it would arrive anywhere from instantly
        to 20-40+ seconds late, correlated with bursts of serdata.cpp:354
        deserialization errors on the robot's OTHER processes
        (robot_state_publisher) at the same moments -- something about this
        DDS link's handling of that one topic/direction degrades
        periodically, independent of message size (even single-point,
        ~500-byte gripper/arm goals hit it).

        /joint_states, by contrast, has streamed reliably all night at a
        steady ~100Hz cross-machine (confirmed repeatedly with `ros2 topic
        hz`), so using it to detect "did the arm actually get where it was
        told to go" sidesteps whatever is specifically wrong with the
        status/action machinery entirely, using a channel already proven
        solid instead of trying to fix the flaky one further.

        Does not resend the goal -- since goal delivery itself has never
        been the problem, only completion detection, resending here would
        just risk commanding a second, overlapping trajectory."""
        target = {name: pos for name, pos in
                  zip(goal.trajectory.joint_names, goal.trajectory.points[-1].positions)}
        print(f"[{label}] target joint positions: "
              f"{ {n: round(p, 4) for n, p in target.items()} }")
        # Full time_from_start of the last point, in seconds -- the earliest
        # the trajectory could possibly finish. Guards against declaring
        # success instantly just because the arm happened to already be
        # near the target before this goal was even sent (e.g. re-sending
        # the same pose, or a short final approach segment).
        last_point_sec = (goal.trajectory.points[-1].time_from_start.sec +
                          goal.trajectory.points[-1].time_from_start.nanosec / 1e9)
        earliest_done = time.monotonic() + last_point_sec

        _describe_trajectory(label, goal.trajectory)

        # ------------------------------------------------------------------
        # Confirm the goal was actually ACCEPTED before waiting on motion,
        # instead of firing send_goal_async() and discarding its future
        # (2026-07-26).
        #
        # WHY THIS MATTERS MORE THAN IT LOOKS: without it, a failure here is
        # undiagnosable, because /joint_states-convergence polling alone
        # cannot distinguish three totally different faults, all of which
        # present as the identical "never converged within 60s" error:
        #   1. the goal never reached the robot at all (a dropped request --
        #      see _deliver_goal),
        #   2. the goal was received and accepted but the hardware never
        #      actually moved (the Galactic controller_manager reports
        #      "Goal reached, success!" from elapsed trajectory time alone --
        #      there is no `constraints:` block in ros2_controllers.yaml, so
        #      arm_group_controller has NO goal tolerance to check and
        #      literally cannot report failure), or
        #   3. the arm moved but stopped short of settle_tolerance.
        # Two full debugging sessions were spent guessing between these.
        if self._deliver_goal(client_attr, action_name, goal, label) is None:
            return False

        # Snapshot the starting position so a timeout can report whether the
        # arm moved at all vs. moved but fell short -- see the error below.
        start_positions = {n: self._joint_positions.get(n) for n in target}
        max_excursion = 0.0

        deadline = time.monotonic() + max(timeout_sec, last_point_sec + 5.0)
        last_print = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            # A reader that unmatches mid-goal freezes every check below, so
            # the goal would otherwise burn its full timeout and be reported as
            # a tracking failure that never happened.
            self.recreate_joint_state_sub_if_stale()
            # The 1e-9 slack is not cosmetic. The gripper's position readback
            # is quantized to 0.0075 rad (pymycobot's 0-100 scale over the
            # 0.75 rad jaw span), so a commanded target and a readback landing
            # EXACTLY settle_tolerance apart is a routine outcome, not an edge
            # case -- and in IEEE754 that comparison goes the wrong way:
            # -0.48 - (-0.53) evaluates to 0.050000000000000044, which is not
            # <= 0.05. Observed on real hardware 2026-07-26: a grasp reported
            # "per-joint error 0.05, tolerance 0.05" and failed.
            # Per-joint tolerance where the global one asks the wrong question --
            # see ARM_SETTLE_TOLERANCE_PER_JOINT. Only applied when it LOOSENS the
            # gate, so passing a tighter settle_tolerance explicitly (the gripper
            # calls this with 0.05) is never silently widened.
            reached = all(
                name in self._joint_positions and
                abs(self._joint_positions[name] - pos) <= max(
                    settle_tolerance,
                    ARM_SETTLE_TOLERANCE_PER_JOINT.get(name, 0.0)) + 1e-9
                for name, pos in target.items()
            )
            for n, p0 in start_positions.items():
                p = self._joint_positions.get(n)
                if p0 is not None and p is not None:
                    max_excursion = max(max_excursion, abs(p - p0))
            if time.monotonic() - last_print > 5.0:
                last_print = time.monotonic()
                if not self._joint_positions:
                    # NOT a sensor fault. An empty dict means no /joint_states
                    # message has EVER arrived, so every joint falls back to its
                    # default. Printing float("nan") for that default made the
                    # failure look like corrupted readings from the arm, and
                    # sent a debugging session to the robot -- whose own log was
                    # completely clean, because the robot was publishing fine
                    # and the subscription simply never matched over DDS.
                    print(f"[{label}] waiting... NO /joint_states MESSAGE HAS "
                          f"EVER ARRIVED ({self.count_publishers('/joint_states')} "
                          f"publisher(s) currently visible). This is a DDS "
                          f"discovery failure on this end, not an arm fault -- "
                          f"the arm may well be moving. Nothing can converge "
                          f"until it is fixed.")
                else:
                    current = {n: round(self._joint_positions[n], 4)
                               for n in target if n in self._joint_positions}
                    missing = [n for n in target if n not in self._joint_positions]
                    missing_note = f", NOT REPORTED: {missing}" if missing else ""
                    print(f"[{label}] waiting... current joint positions: {current}"
                          f"{missing_note}  "
                          f"(max movement so far {max_excursion:.4f} rad)")
            if reached and time.monotonic() >= earliest_done:
                return True

        # Timed out. The goal WAS accepted (checked above), so this is a
        # hardware-side tracking failure, not a delivery failure -- report
        # enough to tell "never moved" apart from "moved but stopped short",
        # which is the difference between a dead write path
        # (mycobot_bridge.py never called send_angles) and a slow/blocked arm.
        # log_failure=False: the caller expects non-convergence and handles it
        # (the gripper closing loop, where a jaw held by a block CANNOT reach
        # its commanded value). Logging an ERROR per increment there buried the
        # real output under ~20 alarming-but-meaningless messages per grasp.
        if not log_failure:
            return False

        if not self._joint_positions:
            print(f"[{label}] TIMED OUT, but NOT because of the arm: no "
                  f"/joint_states message arrived during the entire wait "
                  f"({self.count_publishers('/joint_states')} publisher(s) "
                  f"visible). The goal was accepted, so the robot very likely "
                  f"executed it -- this process just cannot see the result. "
                  f"Check DDS discovery on this machine before touching "
                  f"anything on the robot.")
            return False

        errors = {n: round(self._joint_positions.get(n, float("nan")) - p, 4)
                  for n, p in target.items()}
        if max_excursion < 0.01:
            verdict = ("the arm NEVER MOVED AT ALL -- the command never reached "
                       "the servos. Check mycobot_bridge.py's stdout for TIMING / "
                       "send_angles lines and for 'ERROR during serial write'.")
        else:
            verdict = ("the arm DID move but stopped short -- a tracking or joint "
                       "limit problem, not a dead write path.")
        # Name the per-joint overrides explicitly. Reporting a flat "tolerance
        # 0.07" while a joint is actually gated at 0.09 sends the next person
        # chasing the wrong number, which is how the 0.0712-vs-0.07 abort read as
        # a random fluke rather than a threshold sitting in the wrong place.
        applied = {n: max(settle_tolerance, ARM_SETTLE_TOLERANCE_PER_JOINT[n])
                   for n in target
                   if ARM_SETTLE_TOLERANCE_PER_JOINT.get(n, 0.0) > settle_tolerance}
        tol_note = (f"{settle_tolerance} rad, overridden {applied}"
                    if applied else f"{settle_tolerance} rad")
        self.get_logger().error(
            f"{label} goal: ACCEPTED by the controller but /joint_states never "
            f"converged within {timeout_sec}s (tolerance {tol_note}).\n"
            f"  per-joint error (current - target): {errors}\n"
            f"  max movement of ANY joint during the whole wait: "
            f"{max_excursion:.4f} rad\n"
            f"  -> {verdict}")
        return False

    # ---- Gripper ----
    def joint_position(self, joint_name):
        """Latest /joint_states position for joint_name, or None if it hasn't
        been received yet."""
        return self._joint_positions.get(joint_name)

    def gripper_move_to(self, position, duration_sec=1.0,
                        timeout_sec=60.0, require_convergence=True):
        """require_convergence=False: send the goal and wait, but treat a
        failure to reach `position` as success rather than an error.

        That is the correct behaviour while closing onto an object, where the
        jaw physically CANNOT reach the commanded value -- see
        gripper_close_until_contact(). Measured on real hardware 2026-07-26:
        pymycobot's gripper readback trails the commanded value by roughly
        0.05-0.0675 rad all the way through a close, so demanding convergence
        on an incremental closing step fails semi-randomly depending on where
        the 0.0075 rad quantization happens to land.

        timeout_sec also matters here: the default 60s is right for a real
        arm move, but gripper_close_until_contact() issues ~40 sub-second
        increments, and at 60s per stalled increment a single grasp burned
        many minutes before aborting."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper_controller"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        goal.trajectory.points = [point]

        # Gripper has a wider tolerance than the arm (0.05 vs 0.02 rad) --
        # pymycobot's gripper position readback is coarser (0-100 scale
        # mapped to radians) than the arm's joint encoders.
        ok = self._send_goal_and_wait("_gripper_client",
                                      self._gripper_action_name,
                                      goal, "gripper", settle_tolerance=0.05,
                                      timeout_sec=timeout_sec,
                                      log_failure=require_convergence)
        return True if not require_convergence else ok

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
        response = self._spin_until_complete(future, what="/compute_cartesian_path response")

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
        response = self._spin_until_complete(future, what="/compute_ik response")
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
        response = self._spin_until_complete(future, what="/compute_ik (exact) response")
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
        # Generous margin over planning_time itself (not just the default
        # 30s) -- OMPL's allowed_planning_time is move_group's internal
        # solve budget, separate from how long the response then takes to
        # actually arrive back over the cross-machine DDS link.
        response = self._spin_until_complete(
            future, timeout_sec=planning_time + 20.0, what="/plan_kinematic_path response")
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
        response = self._spin_until_complete(future, what="/check_state_validity response")
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


# Time parameterization speed, in rad/s, used by _ensure_monotonic_timing.
#
# NOT actually a "fallback" any more, despite the name and despite what
# _ensure_monotonic_timing's docstring used to claim. Measured against the
# live move_group on mars (Jazzy) on 2026-07-26: EVERY trajectory returned by
# /plan_kinematic_path comes back with time_from_start=0 on every point and
# empty velocities/accelerations arrays, so this function is the ONLY time
# parameterization anywhere in the system, on both machines. See
# _ensure_monotonic_timing's docstring for the root cause (ompl_planning.yaml
# omits response_adapters, and on Jazzy that means the OMPL pipeline loads NO
# response adapters at all -- move_group's own log says "No planning response
# adapter names specified" for ompl, while the stomp/pilz/chomp pipelines each
# log "Loaded adapter default_planning_response_adapters/
# AddTimeOptimalParameterization"). The comment in ompl_planning.yaml claiming
# "both distros fall back to their own built-in default adapter list" is
# therefore wrong for the response side.
#
# Raised 0.2 -> 0.5 rad/s (2026-07-26). 0.2 was what this had been silently
# running at: 5x slower than joint_limits.yaml's 1.0 rad/s per-joint max, and
# 2.6x slower than the hand-built trajectories in joint_trajectory_test.py
# (1.55 rad over 3.0s = 0.52 rad/s) which ARE confirmed to move the real arm.
# 0.5 rad/s matches that confirmed-working rate, and shortens the pre-grasp
# trajectory from 9.14s to ~3.7s -- fewer seconds during which
# mycobot_bridge.py's ~1Hz serial loop has to keep up with a 100Hz setpoint
# stream (see _match_speed in mycobot_bridge.py for why that mismatch is the
# real failure mode). Anything changed here changes the real arm's speed
# directly -- it is not a safety net.
# Raised 0.5 -> 0.8 rad/s on 2026-07-27, together with _MAX_JOINT_ACCEL.
#
# 0.5 was chosen while the command path was starved: the arm was receiving ~11
# point-to-point commands per trajectory, so a higher speed only meant bigger
# jumps between them. That constraint is gone -- commands now arrive at 30Hz
# with sub-degree steps -- so speed is once again limited by the hardware
# rather than by the pipeline. joint_limits.yaml declares 1.0 rad/s.
#
# REVERTED 0.8 -> 0.5 on 2026-07-28 at the user's request: the motion at 0.5
# was described as "genuinely very smooth", and speed was explicitly not worth
# trading for it. 0.8 does not itself create command gaps, but it makes every
# gap hurt 1.6x more -- a 180ms blackout advances the trajectory 8.2 degrees at
# 0.8 rad/s against 5.2 at 0.5. Raise this again only after the gaps themselves
# are gone, and re-raise _MAX_JOINT_ACCEL with it or it buys almost nothing.
_FALLBACK_MAX_JOINT_SPEED = 0.5  # rad/s
_FALLBACK_MIN_SEGMENT_SEC = 0.1  # floor per waypoint, avoids zero-length segments

# Acceleration limit for the ramps, rad/s^2. UNVERIFIED against this arm --
# joint_limits.yaml declares no acceleration limits, so this is chosen rather
# than derived.
#
# Raised 1.0 -> 2.0 on 2026-07-27, and it MUST be raised alongside
# _FALLBACK_MAX_JOINT_SPEED or raising the speed does nothing. Reaching speed v
# under acceleration a costs v^2/(2a) of travel to ramp up and the same to ramp
# down, so a segment shorter than v^2/a never reaches v at all:
#
#   v=0.5, a=1.0  ->  needs 0.25 rad to reach speed
#   v=0.8, a=1.0  ->  needs 0.64 rad   <- most segments are ~0.1 rad
#   v=0.8, a=2.0  ->  needs 0.32 rad
#
# The forward/backward passes carry speed across waypoints, so a whole 2 rad
# move can still reach the cap even when no single segment could -- but with
# a=1.0 the profile would spend nearly all of a typical move ramping, and the
# nominal speed increase would show up as almost no real speedup.
_MAX_JOINT_ACCEL = 1.0

# Floor on how much a sharp corner is allowed to slow the arm, as a fraction of
# _FALLBACK_MAX_JOINT_SPEED. Without a floor a 90-degree turn in joint space
# would demand a full stop at that waypoint, which is smooth but slow; without
# any corner slowdown at all the velocity changes discontinuously there, which
# is the jerk this is here to remove. 0.1 keeps the arm moving through even a
# reversal while still taking most of the speed out of it.
_MIN_CORNER_SPEED_SCALE = 0.1


def _duration_to_sec(duration):
    return duration.sec + duration.nanosec * 1e-9


def _sec_to_duration(seconds, duration):
    duration.sec = int(seconds)
    duration.nanosec = int((seconds % 1) * 1e9)


def _describe_trajectory(label, joint_trajectory):
    """Print the shape of a trajectory right before it's sent, so a failure
    can be attributed to the trajectory rather than guessed at afterwards.

    Added 2026-07-26 because two debugging sessions ran without ever seeing
    these numbers, and they turn out to be the whole story: the trajectories
    that move the real arm and the ones that don't differ in exactly these
    fields (waypoint count, total duration, implied joint speed, and whether
    velocities are present at all)."""
    points = joint_trajectory.points
    if not points:
        print(f"[{label}] EMPTY TRAJECTORY -- nothing to execute")
        return
    total = _duration_to_sec(points[-1].time_from_start)
    max_delta = 0.0
    for i in range(len(points) - 1):
        max_delta = max(max_delta, max(
            (abs(a - b) for a, b in zip(points[i].positions, points[i + 1].positions)),
            default=0.0))
    speed = (max_delta / (total / max(1, len(points) - 1))) if total > 0 else float("inf")
    stamp = joint_trajectory.header.stamp
    print(f"[{label}] trajectory: {len(points)} waypoints, {total:.3f}s total, "
          f"~{speed:.3f} rad/s peak joint speed, "
          f"velocities={'yes' if points[0].velocities else 'NO'}, "
          f"header.stamp={stamp.sec}.{stamp.nanosec:09d}")


def _ensure_monotonic_timing(joint_trajectory):
    """Assign strictly-increasing time_from_start to every waypoint when the
    planner returned a raw geometric path with no time parameterization at
    all (every point at time_from_start=0).

    THIS IS NOT A SAFETY NET -- IT IS THE ONLY TIME PARAMETERIZATION IN THE
    SYSTEM. Verified against the live move_group on mars (Jazzy) on
    2026-07-26 by dumping real /plan_kinematic_path responses: every OMPL
    plan comes back with all-zero time_from_start and empty velocities and
    accelerations, so this function fires on 100% of real plans, on both
    machines, and its _FALLBACK_MAX_JOINT_SPEED is the real arm's actual
    commanded speed.

    Root cause: ompl_planning.yaml deliberately omits `response_adapters`
    because Galactic and Jazzy require mutually incompatible types for it (a
    plain string vs. a string array) -- see that file's comment. Its claim
    that "both distros fall back to their own built-in default adapter list"
    holds for REQUEST adapters but NOT for response adapters on Jazzy:
    move_group logs

        [WARN] ...planning_pipeline]: No planning response adapter names specified.

    for the ompl pipeline specifically, while the stomp / pilz / chomp
    pipelines in the same log each go on to "Loaded adapter
    'default_planning_response_adapters/AddTimeOptimalParameterization'".
    OMPL alone ends up with zero response adapters, hence zero timing.

    A real fix is a per-distro response_adapters config (the same split
    already used for cyclonedds_galactic.xml / cyclonedds_jazzy.xml), which
    would restore proper AddTimeOptimalParameterization output -- including
    the velocities/accelerations that joint_trajectory_controller needs for
    cubic rather than linear interpolation. Until that exists, everything
    below is what actually paces the arm.

    WHAT IT ASSIGNS (rewritten 2026-07-27): a trapezoidal profile that ramps
    up from rest, slows through corners in the path, and ramps back down to
    rest, plus a velocities field.

    It used to assign a single constant speed from start to finish. That is
    what produced the "mostly smooth with 4 or 5 jerks per move" behaviour
    observed on real hardware: an OMPL path is piecewise linear through its
    waypoints, so holding speed constant across a corner means the joint
    velocity changes DISCONTINUOUSLY there, and a simplified RRTConnect path
    has a handful of genuinely sharp corners. The jerks were the corners. The
    same run showed the Cartesian moves (which DO come back time
    parameterized, from /compute_cartesian_path) looking smooth over the same
    hardware, which is what isolated this to the timing rather than to the
    command pipeline.

    Populating velocities also matters on its own: joint_trajectory_controller
    interpolates linearly between waypoints when they carry positions only,
    and with a cubic spline when velocities are present. Centered differences
    are used so the velocities stay consistent with the positions and times
    and the spline does not overshoot."""
    points = joint_trajectory.points
    if len(points) < 2:
        return

    already_monotonic = all(
        _duration_to_sec(points[i + 1].time_from_start) > _duration_to_sec(points[i].time_from_start)
        for i in range(len(points) - 1)
    )
    if already_monotonic:
        return

    _apply_trapezoidal_timing(points)


def _corner_speed_scale(prev_delta, next_delta):
    """How much of the maximum speed the arm may carry through a waypoint,
    from the angle between the incoming and outgoing path segments.

    1.0 where the path runs straight through, falling to
    _MIN_CORNER_SPEED_SCALE at a right angle or a reversal. The sqrt makes
    the reduction gentle for slight bends and aggressive only for real
    corners, which is where the velocity discontinuity actually hurts."""
    norm_prev = math.sqrt(sum(x * x for x in prev_delta))
    norm_next = math.sqrt(sum(x * x for x in next_delta))
    if norm_prev < 1e-9 or norm_next < 1e-9:
        return 1.0
    cosine = sum(a * b for a, b in zip(prev_delta, next_delta)) / (norm_prev * norm_next)
    cosine = max(-1.0, min(1.0, cosine))
    # cos(theta/2) via the half-angle identity: 1.0 straight through, 0.71 at a
    # right angle, 0 at a reversal. Deliberately gentler than penalising by
    # cos(theta) directly, which sends a right-angle corner to a dead stop and
    # made a wiggly path take four times as long in testing.
    return max(_MIN_CORNER_SPEED_SCALE, math.sqrt(0.5 * (1.0 + cosine)))


def _segment_duration(distance, speed_start, speed_end):
    """Exact time to cover `distance` going from speed_start to speed_end
    under _MAX_JOINT_ACCEL, without ever exceeding _FALLBACK_MAX_JOINT_SPEED.

    The obvious shortcut -- distance divided by the average of the two end
    speeds -- is only valid while the speed changes monotonically across the
    segment, and it fails exactly where it is most dangerous. On a
    two-waypoint plan BOTH ends are at rest, so the average is zero, the
    minimum-segment floor takes over, and the result commanded 3.0 rad/s for
    a 0.3 rad move: six times the speed limit, on a real arm. The
    'Final return to home pose' step plans exactly two waypoints, so that was
    not a hypothetical.

    Solving for the peak the segment can actually reach handles that case and
    every other one the same way: accelerate to the peak, optionally cruise,
    decelerate to the exit speed."""
    if distance <= 1e-9:
        return _FALLBACK_MIN_SEGMENT_SEC

    # Highest speed reachable within this segment while still braking to
    # speed_end by its far end, capped by the global limit.
    peak = min(
        _FALLBACK_MAX_JOINT_SPEED,
        math.sqrt(max(0.0, (2.0 * _MAX_JOINT_ACCEL * distance
                            + speed_start ** 2 + speed_end ** 2) / 2.0)),
    )
    if peak <= 1e-9:
        return _FALLBACK_MIN_SEGMENT_SEC

    accel_distance = max(0.0, (peak ** 2 - speed_start ** 2) / (2.0 * _MAX_JOINT_ACCEL))
    decel_distance = max(0.0, (peak ** 2 - speed_end ** 2) / (2.0 * _MAX_JOINT_ACCEL))
    cruise_distance = max(0.0, distance - accel_distance - decel_distance)

    duration = ((peak - speed_start) / _MAX_JOINT_ACCEL
                + (peak - speed_end) / _MAX_JOINT_ACCEL
                + cruise_distance / peak)
    return max(_FALLBACK_MIN_SEGMENT_SEC, duration)


def _apply_trapezoidal_timing(points):
    """Assign time_from_start and velocities for a trapezoidal, corner-aware
    velocity profile along an already-fixed geometric path."""
    n = len(points)

    # Segment "length" is the largest single-joint step, so that
    # _FALLBACK_MAX_JOINT_SPEED keeps its meaning as a PER-JOINT limit rather
    # than silently becoming a limit on the norm across all six.
    deltas = []
    dists = []
    for i in range(n - 1):
        delta = [b - a for a, b in zip(points[i].positions, points[i + 1].positions)]
        deltas.append(delta)
        dists.append(max((abs(x) for x in delta), default=0.0))

    # Speed ceiling at each waypoint: at rest at both ends, limited by the
    # corner angle in between.
    speeds = [_FALLBACK_MAX_JOINT_SPEED] * n
    speeds[0] = 0.0
    speeds[n - 1] = 0.0
    for i in range(1, n - 1):
        speeds[i] = min(
            speeds[i],
            _FALLBACK_MAX_JOINT_SPEED * _corner_speed_scale(deltas[i - 1], deltas[i]),
        )

    # Forward then backward pass so the profile is reachable under
    # _MAX_JOINT_ACCEL from both directions -- the standard way to turn a set
    # of per-waypoint speed caps into a profile that can actually be flown.
    for i in range(1, n):
        speeds[i] = min(
            speeds[i],
            math.sqrt(speeds[i - 1] ** 2 + 2.0 * _MAX_JOINT_ACCEL * dists[i - 1]),
        )
    for i in range(n - 2, -1, -1):
        speeds[i] = min(
            speeds[i],
            math.sqrt(speeds[i + 1] ** 2 + 2.0 * _MAX_JOINT_ACCEL * dists[i]),
        )

    times = [0.0]
    for i in range(n - 1):
        times.append(times[-1] + _segment_duration(dists[i], speeds[i], speeds[i + 1]))

    n_joints = len(points[0].positions)
    for i, point in enumerate(points):
        _sec_to_duration(times[i], point.time_from_start)
        if i == 0 or i == n - 1:
            point.velocities = [0.0] * n_joints
            continue
        span = times[i + 1] - times[i - 1]
        if span <= 1e-9:
            point.velocities = [0.0] * n_joints
            continue
        point.velocities = [
            (points[i + 1].positions[j] - points[i - 1].positions[j]) / span
            for j in range(n_joints)
        ]


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


def make_grasp_pose(x, y, z, block_yaw_deg=0.0, holding_block=False):
    """Pose for Cartesian waypoints: position + the downward grasp orientation,
    yawed to meet a block rotated block_yaw_deg (0.0 = the fixed grasp yaw), and
    tipped against the arm's gravity sag -- see SAG_PRECOMP_RADIAL_DEG."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = grasp_quat_for(
        block_yaw_deg, x, y, holding_block)
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


def solve_ik_state(io_client, x, y, z, qx, qy, qz, qw, block_yaw_deg=0.0,
                   ori_xy_tolerance=None):
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
    # Bearing seeds first: they are the only ones whose base rotation is
    # anywhere near the answer. See _bearing_seeds for the measurements.
    # block_yaw_deg only shapes the SEEDS here; the orientation actually solved
    # for is the qx..qw the caller passed, which already carries the yaw.
    seeds = ([("current-state", current_state)]
             + _bearing_seeds(x, y, block_yaw_deg)
             + IK_SEEDS)

    # Evaluate EVERY seed and keep the solution closest to where the arm is
    # standing, rather than returning the first one that converges
    # (2026-07-26).
    #
    # WHY: on this 6-DOF arm most reachable targets have several valid IK
    # branches (elbow up/down, wrist flipped), and KDL returns whichever one
    # is nearest its seed. Taking the first convergence made the branch a
    # function of seed ORDER, and since 'current-state' is tried first, the
    # answer silently changed depending on where the arm happened to be
    # sitting when the script ran. Two runs against the identical target
    # (0.000, 0.250, 0.180):
    #
    #   current-state converged -> [1.828, -0.718, -0.811, -0.041, 0, 1.828]
    #       the arm reached this pose and settled within 0.025 rad.
    #   current-state failed, fell through to 'downward-confirmed'
    #                       -> [1.828, -1.470, +0.811, -0.912, 0, 1.828]
    #       elbow flipped, far more extended: the arm tracked ~1.98 rad of it
    #       and then stopped dead ~0.29 rad (16 deg) short on four joints,
    #       with /joint_states byte-identical for 50+ seconds afterwards.
    #
    # Both are legitimate IK solutions for the same flange pose; only one is
    # something this arm can actually hold. Minimising joint-space travel
    # from the current state is the standard tie-break and it prefers the
    # branch that worked here: the failing solution sits 3.9 rad from home
    # versus 2.9 rad for the working one. It also means less time spent
    # sweeping through awkward intermediate configurations, and it makes the
    # choice deterministic run to run instead of depending on start pose.
    #
    # NOTE this picks the least-travel solution, not necessarily one the arm
    # can physically hold -- there is no torque/gravity model anywhere in
    # this project. If a chosen pose still stalls, that is a payload/reach
    # limit to be measured, not an IK bug.
    # ori_xy_tolerance: how far off the REQUESTED orientation a solution may
    # sit. Defaults to IK_ORI_XY_TOLERANCE (0.10 rad / 5.7 deg), which is the
    # right answer for a grasp -- tilt there is error, and the whole project
    # is trying to drive it out.
    #
    # It is the wrong answer for the camera-aiming poses, and this cost a
    # night to see (hardware 2026-07-31). look_at_quat asks for an orientation
    # that at the survey pose sits BETWEEN 5.7 and 8.6 deg from anything this
    # arm can hold. So no seed can converge -- there is no solution inside the
    # window -- and every still fell through to OMPL constraint sampling,
    # whose own window is the make_orientation_constraint default of 0.15 rad
    # (8.6 deg). Sampling then picked ARBITRARILY from that 3 deg band: two
    # runs of identical code chose poses 0.371 rad apart and got 3 usable
    # views against 1. Seeding cannot fix that, and adding seeds taken from
    # poses the arm had physically reached did not (they all still failed).
    #
    # Passing the wider window here does not let the arm do anything it was
    # not already doing -- the fallback was commanding poses in that band
    # regardless. It makes the choice DETERMINISTIC and least-travel instead
    # of random, which is what the framing needs to be reproducible.
    xy_tol = (IK_ORI_XY_TOLERANCE if ori_xy_tolerance is None
              else ori_xy_tolerance)

    best = None
    for label, seed in seeds:
        solution = io_client.compute_ik(
            x, y, z, qx, qy, qz, qw,
            seed_joint_names=joint_names,
            seed_positions=[seed[n] for n in joint_names],
            xy_tolerance=xy_tol,
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

        travel = math.sqrt(sum((joint_values[n] - current_state[n]) ** 2
                               for n in joint_names))
        print(f"[ik] '{label}' seed: OK -> {joints}  (travel {travel:.3f} rad)")
        if best is None or travel < best[0]:
            best = (travel, label, joint_values)

    if best is None:
        print(f"[ik] All seeds exhausted for ({x:.3f},{y:.3f},{z:.3f}) -- falling back to constraint sampling")
        return None

    travel, label, joint_values = best
    print(f"[ik] chose '{label}' ({travel:.3f} rad of joint travel): "
          f"{[round(v, 3) for v in joint_values.values()]}")
    return joint_values


def settle_pause(io_client, label):
    """Hold still long enough for mycobot_bridge's biased settle to act.

    Needed because the two mechanisms disagree about when a move is "done".
    _send_goal_and_wait declares convergence at ARM_SETTLE_TOLERANCE = 0.07 rad,
    while the bridge's settle needs SETTLE_QUIET_PERIOD_SEC = 1.0 s of
    UNCHANGED command before it will fire at all. With only the 0.5 s inter-step
    sleep, the descent was declared converged at 0.0426 rad, the gripper close
    started 0.5 s later, that changed the command, and the settle timer reset --
    so the arm's settle never ran even once before the grasp. Measured in the
    2026-07-29 grasp pose: 3.70 deg of tilt and 10.2 mm of droop still present
    at the moment the jaws closed, with a correction mechanism that was
    technically enabled and never got a turn.

    Only worth doing where the final pose accuracy is what matters -- the two
    descents. Hovers and retreats do not need it and should not pay for it.
    """
    wait = SETTLE_QUIET_PERIOD_SEC + SETTLE_ACT_MARGIN_SEC
    print(f"[settle] holding {wait:.1f}s before {label} so the bridge's biased "
          "settle can close out the residual")
    time.sleep(wait)
    joints = io_client.current_joint_positions(list(HOME_RADIANS.keys()))
    print("[settle] joints after pause: "
          f"{[round(math.degrees(v), 2) for v in joints.values()]}")
    return True


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
    got_any_effort = False
    stalled_steps = 0
    last_position = io_client.joint_position("gripper_controller")

    while target > closed:
        in_fine_zone = GRIPPER_FINE_ENABLED and target <= fine_zone
        this_step = fine_step if in_fine_zone else step
        this_duration = fine_step_duration if in_fine_zone else step_duration

        target = max(closed, target - this_step)
        # require_convergence=False: once the jaw touches the block it cannot
        # reach the commanded value, and that is the SUCCESS condition here,
        # not a failure. Short timeout because a stalled increment would
        # otherwise wait the full 60s, ~40 times over.
        if not io_client.gripper_move_to(target, duration_sec=this_duration,
                                         timeout_sec=GRIPPER_STEP_TIMEOUT,
                                         require_convergence=False):
            return False

        rclpy.spin_once(io_client, timeout_sec=settle_sec)
        position = io_client.joint_position("gripper_controller")
        effort = io_client.joint_effort("gripper_controller")
        zone = "fine" if in_fine_zone else "coarse"

        # --- contact detection by STALL, not by effort ---
        # pymycobot's gripper API exposes no force/effort reading at all, so
        # joint_effort() here is a hardcoded 0.0 placeholder and the old
        # effort_threshold test could never once fire on real hardware (it
        # worked only in Gazebo). Confirmed on hardware 2026-07-26: a full
        # close onto a block logged "effort=0.000" for all ~40 increments and
        # ran to the hard stop.
        #
        # What DOES carry the signal is the position readback: while the jaw
        # is free it tracks the commanded value down; once it meets the block
        # it stops advancing while the command keeps decreasing. Requiring
        # several consecutive stalled steps beats the 0.0075 rad readback
        # quantization, which alone can make one fine step (0.005 rad) look
        # like no motion.
        if position is None:
            print(f"[gripper] target={target:.3f} ({zone})  <no position reading yet>")
            continue

        progressed = last_position is None or (last_position - position) > GRIPPER_STALL_EPS
        stalled_steps = 0 if progressed else stalled_steps + 1
        lag = position - target
        last_position = position

        if effort is not None:
            got_any_effort = got_any_effort or abs(effort) > 0.0
        effort_text = "n/a" if effort is None else f"{effort:.3f}"
        print(f"[gripper] target={target:.3f} ({zone})  pos={position:.4f}  "
              f"lag={lag:+.4f}  stalled={stalled_steps}/{GRIPPER_STALL_STEPS}  "
              f"effort={effort_text}")

        # Primary signal: the jaw is trailing its command by more than it ever
        # does while moving freely, so something is stopping it.
        if lag >= GRIPPER_CONTACT_LAG:
            print(f"[gripper] CONTACT: jaw trailing its command by {lag:.4f} rad "
                  f"(>= {GRIPPER_CONTACT_LAG}), holding at {position:.4f} while "
                  f"commanded {target:.3f}. Stopping close.")
            return True

        # Backup signal, for a jaw that creeps one readback quantum at a time
        # rather than stopping cleanly enough to build up lag.
        if stalled_steps >= GRIPPER_STALL_STEPS:
            print(f"[gripper] CONTACT: jaw stopped advancing for "
                  f"{GRIPPER_STALL_STEPS} consecutive steps while still being "
                  f"commanded closed (holding at {position:.4f}, commanded "
                  f"{target:.3f}). Stopping close.")
            return True

        # Effort is still honoured if a future hardware/sim setup ever
        # provides a real reading -- it is strictly a better signal than stall.
        if effort is not None and abs(effort) >= effort_threshold:
            print(f"[gripper] CONTACT: |effort|={abs(effort):.3f} >= "
                  f"{effort_threshold:.3f} at position {target:.3f}. Stopping close.")
            return True

    if not got_any_effort:
        print(f"[gripper] reached fully-closed position {closed:.3f} without "
              "detecting contact -- nothing between the fingers. (Effort "
              "readings were all 0.0, as expected on this hardware: contact "
              "detection here is stall-based, see gripper_close_until_contact.)")
    else:
        print(f"[gripper] reached fully-closed position {closed:.3f} without "
              "detecting contact.")
    return True


def go_home(io_client):
    """Return the arm to its designated home pose before planning anything else."""
    joint_trajectory = io_client.plan_motion([make_joint_goal_constraints(HOME_RADIANS)])
    if joint_trajectory is None:
        print("Planning to home pose FAILED.")
        return False

    print("Executing joint-space move to home pose...")
    return io_client.arm_execute(joint_trajectory)


def move_arm_to(io_client, x, y, z, lock_orientation=True, block_yaw_deg=0.0,
                holding_block=False, orientation_override=None,
                ori_xy_tolerance=None):
    """Joint-space plan to a target position. Uses deterministic seeded IK
    when possible; falls back to OMPL constraint sampling if all seeds fail.

    block_yaw_deg rotates the jaws about world +Z to meet a block that is not
    square to the world. 0.0 is the fixed grasp yaw every caller used before
    the AprilTag feature -- see grasp_quat_for().

    x, y are passed to grasp_quat_for so the sag pre-compensation applies here
    too, not only to the Cartesian descent. It has to be on BOTH: hover uses this
    function and the descent uses make_grasp_pose, and if their orientations
    disagree the "straight down" Cartesian descent has to rotate the wrist while
    translating, which is exactly the sideways nudge that descent is careful to
    avoid.

    orientation_override: (qx, qy, qz, qw) to use INSTEAD of grasp_quat_for.
    For detection hovers, where the requirement is "aim the lens at the zone",
    not "point the jaws straight down" -- see tag_pick_place.look_at_quat. This
    is not the same problem as dropping lock_orientation: an unconstrained plan
    satisfies position alone and OMPL is then free to pick ANY orientation,
    which on 2026-07-30 produced a mirror-configuration solve with the base
    swung 180 deg from the target, arm reaching back over itself. A specific
    orientation, even one that is not straight down, keeps the plan a normal
    reach instead of an arbitrary one."""
    if orientation_override is not None:
        qx, qy, qz, qw = orientation_override
    else:
        qx, qy, qz, qw = grasp_quat_for(block_yaw_deg, x, y, holding_block)

    ik_state = None
    if lock_orientation:
        ik_state = solve_ik_state(io_client, x, y, z, qx, qy, qz, qw,
                                  block_yaw_deg=block_yaw_deg,
                                  ori_xy_tolerance=ori_xy_tolerance)

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
                    POSE_LINK, PLANNING_FRAME, qx, qy, qz, qw,
                    z_tolerance=0.15)
            )
        goal_constraints = [constraints]

    joint_trajectory = io_client.plan_motion(goal_constraints)
    if joint_trajectory is None:
        print(f"Planning FAILED for target ({x}, {y}, {z})")
        return False

    print(f"Executing joint-space move to ({x}, {y}, {z})...")
    return io_client.arm_execute(joint_trajectory)


def cartesian_move_to(io_client, x, y, z, min_fraction=0.90, allow_fallback=False,
                      block_yaw_deg=0.0, holding_block=False):
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

    target = make_grasp_pose(x, y, z, block_yaw_deg, holding_block)

    # The PATH constraint must carry the same yaw as the target pose. Leave it
    # at the fixed grasp yaw while descending onto a rotated block and the two
    # disagree by exactly block_yaw_deg, which shows up as a Cartesian solve
    # that falls short of min_fraction for no visible reason. Same argument for
    # the sag pre-compensation: constraint and target must describe the SAME
    # orientation, or the solve falls short of min_fraction with no visible cause.
    path_constraints = Constraints()
    path_constraints.orientation_constraints.append(
        make_orientation_constraint(
            POSE_LINK, PLANNING_FRAME,
            *grasp_quat_for(block_yaw_deg, x, y, holding_block),
            z_tolerance=0.15)
    )

    # The constrained and unconstrained solves were computed side by side here
    # as a diagnostic while chasing planning failures. Removed 2026-07-26:
    # every Cartesian move in the first full real-hardware pick and place
    # returned fraction=1.00 both with and without the orientation constraint,
    # so the unconstrained solve proved nothing and cost an extra
    # /compute_cartesian_path round trip on all four Cartesian moves.
    solution_msg, fraction = io_client.compute_cartesian_path(
        waypoints=[target],
        avoid_collisions=True,
        path_constraints=path_constraints,
    )
    print(f"[cartesian] planned fraction: {fraction:.2f}")

    if solution_msg is None or fraction < min_fraction:
        print(f"Cartesian planning FAILED for ({x}, {y}, {z}) (fraction={fraction:.2f})")
        if not allow_fallback:
            return False
        print(f"[cartesian] falling back to joint-space move_arm_to for ({x}, {y}, {z})")
        return move_arm_to(io_client, x, y, z, block_yaw_deg=block_yaw_deg,
                           holding_block=holding_block)

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
    parser.add_argument("--gripper-yaw-deg", type=float, default=GRIPPER_YAW_DEG,
                        help="world-frame yaw (deg, about +Z) the jaws hold during "
                        "every grasp, pick and place alike -- this is a MOUNT "
                        "correction (the tool sits 45 deg askew, see GRIPPER_YAW_DEG's "
                        "comment), not a per-block rotation. At the default the jaw "
                        "axis lines up with world +X; add +/-90*n deg to line up with "
                        "+Y instead, or anything in between for a mat that isn't "
                        f"axis-aligned with world (default: {GRIPPER_YAW_DEG})")
    return parser.parse_args()


def main():
    args = parse_args()

    # --gripper-yaw-deg overrides the module-level default. Recompute the
    # derived quaternion and seed offset from it now, before anything below
    # reads them -- gripper_yaw_quat()/_bearing_seeds() re-read these globals
    # on every call, so this is the only place a rebuild is needed.
    global GRIPPER_YAW_DEG, GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, \
        GRIPPER_LOCK_QW, _GRASP_YAW_JOINT_OFFSET
    if args.gripper_yaw_deg != GRIPPER_YAW_DEG:
        GRIPPER_YAW_DEG = args.gripper_yaw_deg
        GRIPPER_LOCK_QX, GRIPPER_LOCK_QY, GRIPPER_LOCK_QZ, GRIPPER_LOCK_QW = \
            gripper_yaw_quat(GRIPPER_YAW_DEG)
        _GRASP_YAW_JOINT_OFFSET = -math.radians(GRIPPER_YAW_DEG)
        print(f"[pick_place] gripper yaw overridden to {GRIPPER_YAW_DEG} deg "
              "(default corrects the 45 deg mount offset)")

    # use_sim_time:=true REMOVED (2026-07-26): this script targets real
    # hardware (split-compute, mars<->robot), never Gazebo -- there is no
    # /clock publisher anywhere in that setup, so use_sim_time here only
    # ever caused problems and never provided the sim-time-matching benefit
    # it exists for. Already confirmed to make self.get_clock().now() never
    # advance (a silent-infinite-loop bug, fixed by switching timeout math
    # to time.monotonic() in RobotIOClient). Suspected of causing a second,
    # not-yet-fully-understood failure mode too: after a handful of action
    # goals, this process alone (never joint_trajectory_test.py, which
    # calls plain rclpy.init() with no use_sim_time -- the only real
    # difference between the two scripts) stops receiving any further
    # goal completion via /joint_states, every single run tonight,
    # regardless of DDS-level tuning (QoS, watermarks, fragment size) that
    # had no effect. If real motion still fails after this change, the
    # theory is wrong and should be revisited; if it's fixed, use_sim_time
    # was likely corrupting this node's rclpy clock/QoS machinery in some
    # way not limited to the two explicit self.get_clock() call sites
    # already patched.
    rclpy.init()

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

    # Refuse to start blind. Without /joint_states nothing in this script can
    # verify that anything happened: every convergence check needs a measured
    # position, so each goal burns its full timeout and the run reports a
    # motion failure for what is actually a discovery failure on THIS machine.
    # Observed 2026-07-27 across three consecutive runs, each of which sent
    # real goals the robot really executed while its own log stayed clean.
    if not io_client.wait_for_joint_states(timeout_sec=30.0):
        print("\nABORTING: no /joint_states received in 30s, across several "
              "subscription recreates.\n"
              f"  {io_client.count_publishers('/joint_states')} publisher(s) "
              "discovered:\n"
              f"{io_client.describe_joint_state_publishers()}\n"
              "  If a publisher IS listed above, the robot is publishing and\n"
              "  this is an endpoint match failure on this machine -- compare\n"
              "  the reliability/durability shown against this subscription's\n"
              "  (RELIABLE/VOLATILE, depth 10); a mismatch there never\n"
              "  delivers and no amount of retrying will help.\n"
              "  If NO publisher is listed, it is plain discovery: check that\n"
              "  real_robot_hardware.launch.py is up, that\n"
              "  joint_state_broadcaster is active, and that CYCLONEDDS_URI is\n"
              "  set in THIS shell.")
        io_client.destroy_node()
        rclpy.shutdown()
        return

    steps = [
        ("Return to home pose", lambda: go_home(io_client)),
        ("Toggle gripper (pre-start)", lambda: toggle_gripper(io_client)),
        ("Move to pre-grasp (above pick)",
         lambda: move_arm_to(io_client, px, py, hover_z(pz))),
        ("Descend to grasp pose (Cartesian)",
         lambda: cartesian_move_to(io_client, px, py, pz)),
        ("Let the settle close out the grasp residual",
         lambda: settle_pause(io_client, "the grasp")),
        ("Close gripper (grasp, stop on contact)", lambda: gripper_close_until_contact(io_client)),
        # holding_block=True from here until the release: a loaded gripper sags
        # measurably more than an empty one, and the place descent is where that
        # showed up as ~2 deg of extra tilt versus the pick descent.
        ("Retreat after grasp (Cartesian)",
         lambda: cartesian_move_to(io_client, px, py, hover_z(pz), allow_fallback=True,
                                   holding_block=True)),
        ("Move to pre-place (above place)",
         lambda: move_arm_to(io_client, lx, ly, hover_z(lz), holding_block=True)),
        ("Descend to place pose (Cartesian)",
         lambda: cartesian_move_to(io_client, lx, ly, lz, holding_block=True)),
        ("Let the settle close out the place residual",
         lambda: settle_pause(io_client, "the release")),
        ("Open gripper (release)", lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        # Block released: back to the empty-gripper pre-compensation.
        ("Retreat after release (Cartesian)",
         lambda: cartesian_move_to(io_client, lx, ly, hover_z(lz), allow_fallback=True)),
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