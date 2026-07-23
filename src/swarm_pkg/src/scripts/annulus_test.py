#!/usr/bin/env python3
"""
annulus_test.py -- trace the boundary of an annular-sector work zone.

Work zone (the pivot decided at the end of the spiral sweep chat):
    r    = R_INNER .. R_OUTER   (default 0.15 .. 0.24 m)
    yaw  = YAW_MIN .. YAW_MAX   (default -80 .. +80 deg, base joint)
    z    = TRACE_Z              (default 0.14 m, the confirmed grasp plane)

The boundary is a closed loop of 4 edges:

        outer arc  (r=R_OUTER, yaw sweeps YAW_MIN -> YAW_MAX)
    +80 radial in  (yaw=YAW_MAX, r sweeps R_OUTER -> R_INNER)
        inner arc  (r=R_INNER, yaw sweeps YAW_MAX -> YAW_MIN)
    -80 radial out (yaw=YAW_MIN, r sweeps R_INNER -> R_OUTER)

MODES
-----
  --screen    (default) IK feasibility per vertex. No motion, no move_group
              execution. Writes a CSV + per-edge summary. Run this FIRST.
  --probe     Ask /compute_cartesian_path for each edge and report the
              achieved fraction. Plans but does not execute.
  --execute   Actually trace the boundary.

  --strategy cartesian   (default, execute mode) one compute_cartesian_path
                         call per edge with all that edge's waypoints. The
                         returned fraction tells you exactly how far along
                         each edge the arm can physically travel.
  --strategy joint       chained joint-space OMPL plans, vertex to vertex,
                         no go_home() in between. Slower and the EE path
                         between vertices is not straight, but it degrades
                         more gracefully than Cartesian on this MoveIt build.

Requires pick_place.py importable from the same directory.

    python3 annulus_test.py --screen --verbose
    python3 annulus_test.py --probe
    python3 annulus_test.py --execute --strategy cartesian
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pick_place import (  # noqa: E402
    GROUP_NAME,
    POSE_LINK,
    PLANNING_FRAME,
    GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW,
    GRIPPER_OPEN, GRIPPER_CLOSED,
    HOME_RADIANS,
    IK_SEEDS,
    RobotIOClient,
    make_joint_goal_constraints,
    go_home,
    _is_near_joint_limit,
    _is_state_colliding,
)


# ---------------------------------------------------------------------------
# Work zone geometry
# ---------------------------------------------------------------------------

# Empirically confirmed via `--sweep-rz --no-elbow-ceiling --no-wrist-ceiling`
# (real, unmasked collision checking -- see selftest_collision): at z=0.14,
# the true physical wall is r=0.13, identical across yaw 0/+80/-80 -- no
# yaw-dependence at this z. R_INNER carries a 2cm margin above that.
#
# The old default of 0.07 was never real. Two things were wrong at once:
#   1. solve_ik_filtered was missing state.update() after set_from_ik, so
#      the collision check evaluated the IK seed pose, not the solution --
#      it returned False for everything (fixed; see selftest_collision).
#   2. Even after that fix, the elbow ceiling (1.85) was intercepting
#      rejects before collision got a chance to clear them, which made
#      real collisions display as ceiling rejects on the map ('e'/'w'
#      instead of 'X') and skewed the boundary estimate outward to 0.21.
# Both of those were superseded once TRACE_Z was corrected to the real
# grasp height below (0.081 m, not the earlier IK-convenience guess of
# 0.14 m) and R_INNER was re-derived from a --sweep-rz run at that real
# height, confirmed across yaw = 0, +-80, +-120 deg. See chat log.
R_INNER = 0.170         # m -- re-confirmed by --sweep-rz at z=0.081 with the
                        # adaptive gripper + camera_flange attached (2026-07-16).
                        # r=0.15-0.16 are real self-collisions (the gripper body
                        # contacts the arm when folded that close); r=0.12-0.14
                        # hit the wrist ceiling AND collide. Holds uniformly
                        # across yaw = 0, +-80, +-120 deg (planar assumption
                        # confirmed). Previously 0.145, but that was calibrated
                        # on mycobot_280_m5.urdf which has no gripper. Don't
                        # lower without re-running --sweep-rz with the gripper.
R_OUTER = 0.24          # m
YAW_MIN = math.radians(-120.0)
YAW_MAX = math.radians(+120.0)
# NOTE: r=0.22 at yaw=-120 hits a joint2_to_joint1 LIMIT reject (not a
# collision) in the confirming sweep -- a real but minor mechanical
# constraint only at the extreme corner of (large r, large |yaw|).

# SPAWN_HEIGHT_CORRECTION: gazebo.launch.py's spawn -z was lowered from
# 0.055 to 0.02 (g_base embedded in the table so the real robot sits flush
# with the table top -- see gazebo.launch.py). MoveIt plans in joint-angle
# space against the URDF's own kinematic tree, oblivious to where Gazebo
# physically anchors the robot; since the SAME joint angles get executed by
# Gazebo's physics, any MoveIt Cartesian target lands, in Gazebo-absolute
# space, at target_z + spawn_z. Every constant below that targets an
# ABSOLUTE table/floor height (TRACE_Z; the sweep defaults and warning
# thresholds further down in main()) now needs +0.035m to keep meaning the
# same real-world height it did before. R_INNER/R_OUTER are XY-plane radii
# and HOVER_DZ is a relative offset ON TOP of TRACE_Z, so none of those are
# affected by a Z-only spawn shift.
#
# TRACE_Z below has the correction applied, but this file's whole point is
# empirically re-deriving exactly these numbers (see the file's own history
# of corrections above), and I can't run --sweep-rz myself -- treat this as
# an unverified starting estimate and re-run --sweep-rz /
# --selftest-collision to confirm it, same as after any other change to the
# robot's physical mounting. The --sweep-z default range and the z<=0.14
# warning threshold further down in main() were NOT shifted; they're stale
# against the old mounting until re-validated the same way.
SPAWN_HEIGHT_CORRECTION = 0.035  # m, = old spawn_z (0.055) - new spawn_z (0.02)

TRACE_Z = 0.105 + SPAWN_HEIGHT_CORRECTION  # m -- flange target for grasping
                        # a 4cm cube resting on the floor (block center at
                        # z=0.02) through the gripper's measured fingertip
                        # offset of 0.061 m (deepest link gripper_left2/
                        # right2, measured via gripper_offset_probe.py
                        # against joint6_flange) PLUS the 16mm camera flange
                        # that sits between joint6_flange and gripper_base:
                        #     flange_target_z = block_contact_z + offset + camera_flange
                        #                     = 0.02 + 0.094 + 0.016 = 0.130
                        # (pre-spawn-height-correction value; not re-verified)
                        # Physical hard floor: flange_z can't go below
                        # 0.077 + SPAWN_HEIGHT_CORRECTION m without driving
                        # the fingertips into the ground -- that's hardware
                        # geometry, not an IK/OMPL limit.
                        # NOTE: R_INNER was calibrated at z=0.081 (pre-
                        # correction); re-run --sweep-rz to re-confirm it.
HOVER_DZ = 0.06         # m -- relative hover offset above TRACE_Z

ARC_STEP = 0.02         # m -- tangential spacing along the arcs
RADIAL_STEP = 0.02      # m -- spacing along the radial segments

DWELL_SEC = 0.3
CARTESIAN_MIN_FRACTION = 0.90


# ---------------------------------------------------------------------------
# Joint indexing / filters (carried over from spiral_reach_test.py)
# ---------------------------------------------------------------------------

BASE_JOINT  = "joint2_to_joint1"
ELBOW_JOINT = "joint4_to_joint3"
WRIST_JOINT = "joint6output_to_joint6"

JOINT_ORDER       = list(HOME_RADIANS.keys())
BASE_JOINT_INDEX  = JOINT_ORDER.index(BASE_JOINT)
ELBOW_JOINT_INDEX = JOINT_ORDER.index(ELBOW_JOINT)
WRIST_JOINT_INDEX = JOINT_ORDER.index(WRIST_JOINT)

BASE_YAW_SIGN = +1.0
BASE_YAW_REF  = 0.0

BASE_JOINT_LIMITS = (math.radians(-168.0), math.radians(168.0))

# abs(joint4) ceiling. Originally 1.85, meant as a proxy for "reject the
# deep +/-2.5 elbow self-collision branches" from back when the collision
# checker had the state.update() bug and always returned False. Once the
# checker was fixed and verified (selftest_collision), an unmasked
# --sweep-rz run showed this ceiling was actively rejecting REAL, reachable
# configurations at r~0.13-0.24 -- e.g. elbow=-2.375 at r=0.15 was flagged
# "NOT colliding" by the true checker on most IK branches, but the ceiling
# discarded it anyway before collision was ever consulted. R_INNER=0.15 was
# picked from a run with this ceiling off; leaving it on by default here
# would silently contradict that boundary. Now that the real checker is
# trustworthy, it is the sole arbiter by default. Pass --elbow-ceiling to
# reinstate a numeric cap if you ever have reason to distrust the checker
# again (e.g. after a URDF/collision-mesh change you haven't re-validated
# with --selftest-collision).
ELBOW_CEILING = None

# abs(joint6output) ceiling -- rejects wrist-spin configs where joint5
# physically contacts gripper_base. Independently validated: an unmasked
# sweep with this ceiling active found 48/48 of its rejects were confirmed
# real self-collisions (0 false positives), so it's left on by default.
WRIST_CEILING = 0.90


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

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


def yaw_rotated_grasp_quat(yaw):
    """q_yaw(yaw) * q_grasp -- the fixed downward grasp quaternion rotated by
    'yaw' around world Z. Keeps joint6output near zero as the base sweeps."""
    q_yaw = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    q_grasp = (GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW)
    return quat_multiply(q_yaw, q_grasp)


def make_pose(x, y, z, quat):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# Boundary generation
# ---------------------------------------------------------------------------

def _linspace(a, b, n):
    if n <= 1:
        return [a]
    step = (b - a) / (n - 1)
    return [a + i * step for i in range(n)]


def _arc_points(r, yaw_a, yaw_b, z, arc_step, edge, start_index):
    """Vertices along an arc of radius r, arc-length spaced."""
    span = abs(yaw_b - yaw_a) * r
    n = max(2, int(math.ceil(span / arc_step)) + 1)
    pts = []
    for i, yaw in enumerate(_linspace(yaw_a, yaw_b, n)):
        pts.append({
            "index": start_index + i,
            "edge": edge,
            "x": r * math.cos(yaw),
            "y": r * math.sin(yaw),
            "z": z,
            "r": r,
            "yaw": yaw,
            "yaw_deg": math.degrees(yaw),
        })
    return pts


def _radial_points(yaw, r_a, r_b, z, radial_step, edge, start_index):
    """Vertices along a radial segment at fixed yaw."""
    span = abs(r_b - r_a)
    n = max(2, int(math.ceil(span / radial_step)) + 1)
    pts = []
    for i, r in enumerate(_linspace(r_a, r_b, n)):
        pts.append({
            "index": start_index + i,
            "edge": edge,
            "x": r * math.cos(yaw),
            "y": r * math.sin(yaw),
            "z": z,
            "r": r,
            "yaw": yaw,
            "yaw_deg": math.degrees(yaw),
        })
    return pts


def generate_boundary(r_inner=R_INNER, r_outer=R_OUTER,
                      yaw_min=YAW_MIN, yaw_max=YAW_MAX,
                      z=TRACE_Z, arc_step=ARC_STEP, radial_step=RADIAL_STEP):
    """Closed 4-edge boundary of the annular sector. Corner vertices are
    emitted once (the first point of each edge after the first is dropped,
    since it duplicates the previous edge's last point). The final vertex
    duplicates vertex 0 to close the loop."""
    edges = []
    idx = 0

    e1 = _arc_points(r_outer, yaw_min, yaw_max, z, arc_step, "outer_arc", idx)
    idx += len(e1)
    edges.append(e1)

    e2 = _radial_points(yaw_max, r_outer, r_inner, z, radial_step, "radial_in@+yaw", idx - 1)
    e2 = e2[1:]
    idx = e2[-1]["index"] + 1
    edges.append(e2)

    e3 = _arc_points(r_inner, yaw_max, yaw_min, z, arc_step, "inner_arc", idx - 1)
    e3 = e3[1:]
    idx = e3[-1]["index"] + 1
    edges.append(e3)

    e4 = _radial_points(yaw_min, r_inner, r_outer, z, radial_step, "radial_out@-yaw", idx - 1)
    e4 = e4[1:]
    edges.append(e4)

    # Re-index cleanly and close the loop back onto vertex 0.
    flat = []
    for edge in edges:
        flat.extend(edge)
    for i, p in enumerate(flat):
        p["index"] = i
    close = dict(flat[0])
    close["index"] = len(flat)
    close["edge"] = "close_loop"
    flat.append(close)

    return flat, edges


# ---------------------------------------------------------------------------
# RViz visualization -- extruded work-zone volume + traced-boundary outline
# ---------------------------------------------------------------------------

def _pt(x, y, z):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def build_workzone_markers(r_inner, r_outer, yaw_min, yaw_max,
                           z_bottom, z_top, frame_id=PLANNING_FRAME,
                           n_arc=48, ns="annulus_workzone",
                           boundary_points=None):
    """Build a MarkerArray showing the validated work zone:
      id 0: TRIANGLE_LIST -- translucent green solid, the extruded annular
            sector from z_bottom to z_top (an actual volume, not a flat patch).
      id 1: LINE_LIST -- solid green wireframe. If boundary_points is given
            (the same list generate_boundary() produced), the top-face edge
            of the wireframe is drawn from those exact points -- so what you
            see traces the path the arm actually walked, not a re-derived
            approximation of it.
      id 2: TEXT_VIEW_FACING -- a label with the zone parameters.
    """
    angles = [yaw_min + (yaw_max - yaw_min) * i / n_arc for i in range(n_arc + 1)]
    outer_top = [(r_outer * math.cos(a), r_outer * math.sin(a), z_top) for a in angles]
    outer_bot = [(r_outer * math.cos(a), r_outer * math.sin(a), z_bottom) for a in angles]
    inner_top = [(r_inner * math.cos(a), r_inner * math.sin(a), z_top) for a in angles]
    inner_bot = [(r_inner * math.cos(a), r_inner * math.sin(a), z_bottom) for a in angles]

    tris = []

    def quad(a, b, c, d):
        tris.append((a, b, c))
        tris.append((a, c, d))

    for i in range(n_arc):
        quad(outer_top[i], outer_top[i + 1], inner_top[i + 1], inner_top[i])       # top cap
        quad(outer_bot[i], inner_bot[i], inner_bot[i + 1], outer_bot[i + 1])       # bottom cap
        quad(outer_top[i], outer_bot[i], outer_bot[i + 1], outer_top[i + 1])       # outer wall
        quad(inner_top[i], inner_top[i + 1], inner_bot[i + 1], inner_bot[i])       # inner wall
    quad(outer_top[0], inner_top[0], inner_bot[0], outer_bot[0])                   # yaw_min end cap
    quad(outer_top[n_arc], outer_bot[n_arc], inner_bot[n_arc], inner_top[n_arc])   # yaw_max end cap

    fill = Marker()
    fill.header.frame_id = frame_id
    fill.ns = ns
    fill.id = 0
    fill.type = Marker.TRIANGLE_LIST
    fill.action = Marker.ADD
    fill.pose.orientation.w = 1.0
    fill.scale.x = fill.scale.y = fill.scale.z = 1.0
    fill.color = ColorRGBA(r=0.15, g=0.85, b=0.25, a=0.25)
    for a, b, c in tris:
        fill.points += [_pt(*a), _pt(*b), _pt(*c)]

    outline = Marker()
    outline.header.frame_id = frame_id
    outline.ns = ns
    outline.id = 1
    outline.type = Marker.LINE_LIST
    outline.action = Marker.ADD
    outline.pose.orientation.w = 1.0
    outline.scale.x = 0.0025
    outline.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)

    def line(p1, p2):
        outline.points += [_pt(*p1), _pt(*p2)]

    if boundary_points:
        # Draw the top-face outer/inner+radial edges from the ACTUAL traced
        # points rather than the freshly-sampled `angles` above -- this is
        # the literal path --execute walked, not a re-derivation of it.
        by_edge = {}
        for p in boundary_points:
            by_edge.setdefault(p["edge"], []).append(p)
        for edge_name, pts in by_edge.items():
            for a, b in zip(pts, pts[1:]):
                line((a["x"], a["y"], z_top), (b["x"], b["y"], z_top))
    else:
        for i in range(n_arc):
            line(outer_top[i], outer_top[i + 1])
            line(inner_top[i], inner_top[i + 1])

    for i in range(n_arc):
        line(outer_bot[i], outer_bot[i + 1])
        line(inner_bot[i], inner_bot[i + 1])
    line(outer_top[0], outer_bot[0])
    line(inner_top[0], inner_bot[0])
    line(outer_top[n_arc], outer_bot[n_arc])
    line(inner_top[n_arc], inner_bot[n_arc])
    line(inner_top[0], outer_top[0])
    line(inner_bot[0], outer_bot[0])
    line(inner_top[n_arc], outer_top[n_arc])
    line(inner_bot[n_arc], outer_bot[n_arc])

    label = Marker()
    label.header.frame_id = frame_id
    label.ns = ns
    label.id = 2
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    mid_angle = (yaw_min + yaw_max) / 2.0
    mid_r = (r_inner + r_outer) / 2.0
    label.pose.position.x = mid_r * math.cos(mid_angle)
    label.pose.position.y = mid_r * math.sin(mid_angle)
    label.pose.position.z = z_top + 0.04
    label.pose.orientation.w = 1.0
    label.scale.z = 0.02
    label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
    label.text = (f"validated work zone\nr={r_inner:.2f}-{r_outer:.2f} m  "
                  f"yaw={math.degrees(yaw_min):+.0f}..{math.degrees(yaw_max):+.0f} deg\n"
                  f"z={z_bottom:.2f}-{z_top:.2f} m")

    arr = MarkerArray()
    arr.markers = [fill, outline, label]
    return arr


def publish_workzone(topic, frame_id, r_inner, r_outer, yaw_min, yaw_max,
                     z_bottom, z_top, boundary_points, rate_hz, once,
                     verbose=False):
    """Publish the work-zone MarkerArray. Loops at rate_hz by default so late-
    joining RViz sessions still see it (Marker durability is tied to the
    publisher's lifetime, not a one-shot latch) -- pass once=True for a
    single publish if RViz is already open and subscribed."""
    node = rclpy.create_node("annulus_workzone_publisher")
    pub = node.create_publisher(MarkerArray, topic, 10)

    marker_array = build_workzone_markers(
        r_inner, r_outer, yaw_min, yaw_max, z_bottom, z_top,
        frame_id=frame_id, boundary_points=boundary_points)

    print(f"Publishing MarkerArray on '{topic}' (frame_id='{frame_id}')")
    print(f"  fill volume : r={r_inner:.3f}-{r_outer:.3f} m, "
          f"yaw={math.degrees(yaw_min):+.0f}..{math.degrees(yaw_max):+.0f} deg, "
          f"z={z_bottom:.3f}-{z_top:.3f} m")
    print(f"  In RViz: Add -> By topic -> {topic} -> MarkerArray. Set Fixed")
    print(f"  Frame to '{frame_id}' if the display doesn't appear.")

    if once:
        for stamp_pass in range(3):  # a few sends -- ROS2 pub/sub needs the
            for m in marker_array.markers:  # discovery handshake to land
                m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(marker_array)
            rclpy.spin_once(node, timeout_sec=0.3)
        print("Published once (sent a few times to survive discovery). "
              "Exiting -- the marker will vanish if the node it came from "
              "(this one) isn't the thing keeping RViz's late-join durability "
              "alive; if you need it to persist for RViz windows opened "
              "later, drop --once and leave this running.")
        node.destroy_node()
        return

    print("  Publishing every %.1fs. Ctrl-C to stop." % (1.0 / rate_hz))
    try:
        while rclpy.ok():
            for m in marker_array.markers:
                m.header.stamp = node.get_clock().now().to_msg()
            pub.publish(marker_array)
            rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# IK screening
# ---------------------------------------------------------------------------

_UNSET = object()


def solve_ik_filtered(io_client, x, y, z, quat, verbose=False,
                      elbow_ceiling=_UNSET, wrist_ceiling=_UNSET):
    """Seeded IK with the spiral-sweep filters applied.

    IMPORTANT ORDERING NOTE: the elbow/wrist ceilings are HEURISTICS chosen in
    the spiral sweep to reject the deep +/-2.5 self-collision branches. They
    are not ground truth. So this function ALWAYS runs the real collision
    check, even on states the ceilings reject, and reports both verdicts. That
    way a ceiling reject on a collision-free state shows up as
    'elbow ... [NOT colliding]' -- which is your signal that the ceiling is
    over-conservative for this work zone and can be relaxed.

    Pass elbow_ceiling=None / wrist_ceiling=None to disable a ceiling entirely
    and let collision be the only arbiter.

    Returns ({joint_name: value}|None, seed_label|None, reject_reason|None).
    """
    joint_names = JOINT_ORDER

    if elbow_ceiling is _UNSET:
        elbow_ceiling = ELBOW_CEILING
    if wrist_ceiling is _UNSET:
        wrist_ceiling = WRIST_CEILING

    last_reason = "no seed converged"

    for label, seed in IK_SEEDS:
        # compute_ik_exact is the service-call equivalent of moveit_py's
        # RobotState.set_from_ik: exact pose match (no tolerance window),
        # seeded from `seed` exactly like the old in-process solve.
        solution = io_client.compute_ik_exact(
            x, y, z, *quat,
            seed_joint_names=joint_names,
            seed_positions=[seed[n] for n in joint_names],
        )
        if solution is None:
            if verbose:
                print(f"    [ik] '{label}': no convergence")
            continue

        try:
            joint_values = {n: solution[n] for n in joint_names}
        except KeyError:
            if verbose:
                print(f"    [ik] '{label}': solution missing a joint, skipping")
            continue

        joints = [joint_values[n] for n in joint_names]

        near_limit, lname, lval, llo, lhi = _is_near_joint_limit(joint_values)
        if near_limit:
            last_reason = f"{lname}={lval:.3f} near limit"
            if verbose:
                print(f"    [ik] '{label}': {last_reason}")
            continue

        base = joints[BASE_JOINT_INDEX]
        if not (BASE_JOINT_LIMITS[0] <= base <= BASE_JOINT_LIMITS[1]):
            last_reason = f"base {base:.3f} out of joint limits"
            if verbose:
                print(f"    [ik] '{label}': {last_reason}")
            continue

        # Ground truth first, so we can tell whether a ceiling reject was real.
        colliding = _is_state_colliding(io_client, joint_values)

        elbow = joints[ELBOW_JOINT_INDEX]
        wrist = joints[WRIST_JOINT_INDEX]

        ceiling_hit = None
        if elbow_ceiling is not None and abs(elbow) > elbow_ceiling:
            ceiling_hit = f"elbow |{elbow:.3f}| > {elbow_ceiling}"
        elif wrist_ceiling is not None and abs(wrist) > wrist_ceiling:
            ceiling_hit = f"wrist |{wrist:.3f}| > {wrist_ceiling}"

        if ceiling_hit is not None:
            verdict = "colliding" if colliding else "NOT colliding"
            last_reason = f"{ceiling_hit} [{verdict}]"
            if verbose:
                print(f"    [ik] '{label}': {last_reason}")
            continue

        if colliding:
            last_reason = (f"self-collision (elbow={elbow:.3f} wrist={wrist:.3f})")
            if verbose:
                print(f"    [ik] '{label}': {last_reason}")
            continue

        if verbose:
            print(f"    [ik] '{label}': OK -> {[round(v, 3) for v in joints]}")
        return joint_values, label, None

    return None, None, last_reason


def screen_boundary(io_client, points, verbose=False):
    """IK-screen every vertex. Returns list of result dicts."""
    results = []
    for p in points:
        if p["edge"] == "close_loop":
            continue
        quat = yaw_rotated_grasp_quat(p["yaw"])
        state, label, reason = solve_ik_filtered(
            io_client, p["x"], p["y"], p["z"], quat, verbose=verbose)

        row = dict(p)
        row["ik_ok"] = state is not None
        row["seed"] = label or ""
        row["reject"] = reason or ""
        if state is not None:
            joints = [state[n] for n in JOINT_ORDER]
            row["joints"] = [round(v, 4) for v in joints]
            row["base_j1"] = round(joints[BASE_JOINT_INDEX], 4)
            row["elbow_j4"] = round(joints[ELBOW_JOINT_INDEX], 4)
            row["wrist_j6o"] = round(joints[WRIST_JOINT_INDEX], 4)
        else:
            row["joints"] = []
            row["base_j1"] = ""
            row["elbow_j4"] = ""
            row["wrist_j6o"] = ""

        results.append(row)

        mark = "ok " if row["ik_ok"] else "XX "
        print(f"  {mark}[{p['index']:3d}] {p['edge']:16s} "
              f"r={p['r']:.3f} yaw={p['yaw_deg']:+7.2f}deg "
              f"({p['x']:+.3f},{p['y']:+.3f},{p['z']:.3f})"
              + ("" if row["ik_ok"] else f"  <- {reason}"))

    return results


def report_screen(results, edges):
    print("\n" + "=" * 68)
    print("SCREEN SUMMARY")
    print("=" * 68)

    by_edge = {}
    for row in results:
        by_edge.setdefault(row["edge"], []).append(row)

    for edge_name in [e[0]["edge"] for e in edges]:
        rows = by_edge.get(edge_name, [])
        ok = sum(1 for r in rows if r["ik_ok"])
        print(f"  {edge_name:16s}  {ok:3d}/{len(rows):3d} reachable", end="")
        if ok and ok < len(rows):
            good_r = [r["r"] for r in rows if r["ik_ok"]]
            good_y = [r["yaw_deg"] for r in rows if r["ik_ok"]]
            print(f"   (r {min(good_r):.3f}..{max(good_r):.3f}, "
                  f"yaw {min(good_y):+.1f}..{max(good_y):+.1f})")
        else:
            print()

    total_ok = sum(1 for r in results if r["ik_ok"])
    print(f"\n  TOTAL: {total_ok}/{len(results)} vertices reachable")

    ok_rows = [r for r in results if r["ik_ok"]]
    if ok_rows:
        wrists = [abs(r["wrist_j6o"]) for r in ok_rows]
        print(f"  max |joint6output| across reachable vertices: {max(wrists):.4f} rad")
        print("    (near zero confirms the yaw-rotation is doing its job and")
        print("     the OMPL ceiling from the spiral sweep is being sidestepped)")

    # Reject reason histogram
    reasons = {}
    for r in results:
        if not r["ik_ok"]:
            key = r["reject"].split("=")[0].split("|")[0].strip()
            reasons[key] = reasons.get(key, 0) + 1
    if reasons:
        print("\n  Reject reasons:")
        for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"    {v:3d}  {k}")


def write_csv(results, path):
    fields = ["index", "edge", "r", "yaw_deg", "x", "y", "z",
              "ik_ok", "seed", "base_j1", "elbow_j4", "wrist_j6o",
              "joints", "reject"]
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in results:
            out = dict(row)
            out["joints"] = " ".join(str(v) for v in row["joints"])
            writer.writerow(out)
    print(f"\nWrote {path}")


# ---------------------------------------------------------------------------
# Collision checker self-test
# ---------------------------------------------------------------------------

def selftest_collision(io_client, n_random=400, verbose=False):
    """Prove the collision checker actually works before trusting any verdict.

    WHY THIS EXISTS: the first (r,z) sweep reported 181 ceiling rejects of
    which 181 were 'NOT actually self-colliding', and zero self-collisions
    anywhere across ~1200 evaluations -- including r=0.08 configs where the
    arm is folded back over its own base. A 6-DOF arm with a gripper does not
    have a collision-free configuration space. Zero X's means the checker is
    stuck on False, not that the robot is limber.

    Three probes:
      1. home pose            -> expect NOT colliding
      2. deliberately folded  -> expect colliding
      3. N random configs     -> expect a nonzero collision RATE

    Probe 3 is the real one. If 0/N random configurations collide, the checker
    is dead and every reachability conclusion so far is void.
    """
    print("\n" + "=" * 68)
    print("COLLISION CHECKER SELF-TEST")
    print("=" * 68)

    def check(joint_dict):
        return _is_state_colliding(io_client, joint_dict)

    # ---- Probe 1: home ----
    home_colliding = check(dict(HOME_RADIANS))
    print(f"  [1] home pose            -> colliding={home_colliding}   "
          f"(expected False)")

    # ---- Probe 2: deliberately folded configurations ----
    folded_cases = [
        ("elbow fully folded",
         {"joint2_to_joint1": 0.0, "joint3_to_joint2": -1.5,
          "joint4_to_joint3": -2.9, "joint5_to_joint4": -1.5,
          "joint6_to_joint5": 0.0, "joint6output_to_joint6": 0.0}),
        ("wrist spun into forearm",
         {"joint2_to_joint1": 0.0, "joint3_to_joint2": 0.0,
          "joint4_to_joint3": -2.9, "joint5_to_joint4": 2.9,
          "joint6_to_joint5": 1.5, "joint6output_to_joint6": 0.0}),
        ("all joints jammed positive",
         {"joint2_to_joint1": 0.0, "joint3_to_joint2": 2.0,
          "joint4_to_joint3": 2.9, "joint5_to_joint4": 2.5,
          "joint6_to_joint5": 1.5, "joint6output_to_joint6": 2.0}),
    ]
    any_folded_collide = False
    for label, cfg in folded_cases:
        c = check(cfg)
        any_folded_collide = any_folded_collide or c
        print(f"  [2] {label:24s} -> colliding={c}")

    # ---- Probe 3: random configuration collision rate ----
    import random

    n_collide = 0
    n_ok = 0
    failures = 0
    for i in range(n_random):
        vals = {n: random.uniform(-2.8, 2.8) for n in JOINT_ORDER}
        try:
            if check(vals):
                n_collide += 1
            else:
                n_ok += 1
        except Exception as exc:
            failures += 1
            if verbose and failures < 4:
                print(f"      random probe {i} raised: {exc}")

    rate = (100.0 * n_collide / n_random) if n_random else 0.0
    print(f"  [3] {n_random} random configs   -> {n_collide} colliding "
          f"({rate:.1f}%), {n_ok} free, {failures} errored")

    # ---- Verdict ----
    print("\n  VERDICT:", end=" ")
    if n_collide == 0 and not any_folded_collide:
        print("CHECKER IS DEAD.")
        print("  Nothing collides, not even deliberately folded poses. Every")
        print("  'NOT colliding' verdict in the sweeps is meaningless, and the")
        print("  0.13 m inner boundary from --no-elbow-ceiling is UNPROVEN.")
        print("  Do not relax ELBOW_CEILING on the strength of that number.")
        print("  Next: check _is_state_colliding in pick_place.py -- confirm")
        print("  /check_state_validity is being called correctly and the ACM")
        print("  isn't disabling every pair. Try passing verbose=True to it.")
        return False
    if home_colliding:
        print("SUSPECT.")
        print("  The home pose reports colliding, which it should not -- the")
        print("  robot sits there at rest. Checker is inverted or misconfigured.")
        return False
    if n_collide == 0:
        print("SUSPECT.")
        print("  Folded poses collide but 0/%d random configs do. Possible, but"
              % n_random)
        print("  unlikely on a 6-DOF arm with a gripper. Investigate.")
        return False
    print("CHECKER IS LIVE.")
    print(f"  {rate:.1f}% of random configs collide and home does not. Verdicts")
    print("  from the sweep can be trusted.")
    return True


# ---------------------------------------------------------------------------
# (r, z) sweep -- finds the real inner boundary
# ---------------------------------------------------------------------------

def sweep_rz(io_client, r_lo, r_hi, r_step, z_list, yaw_list,
             elbow_ceiling, wrist_ceiling, outdir, verbose=False):
    """Screen an (r, z) grid to locate the inner radius cliff.

    The spiral+annulus screens established that joints 2..6 depend only on
    (r, z) once the grasp orientation is yaw-rotated -- the elbow angle at
    r=0.183 was -2.032 at BOTH +80 and -80 deg, identical to 3 decimals. So
    the inner-boundary question is 2-D, not 3-D, and a grid this small
    answers it outright instead of guessing radii one at a time.

    yaw_list is sampled anyway as a falsification check: if the yaw columns
    ever disagree for the same (r, z), the planar assumption is wrong and
    everything downstream needs rethinking.
    """
    n_r = int(math.floor((r_hi - r_lo) / r_step)) + 1
    radii = [r_lo + i * r_step for i in range(n_r)]

    print("\n" + "=" * 68)
    print("(r, z) SWEEP -- locating the inner boundary")
    print("=" * 68)
    print(f"  r    : {r_lo:.3f} .. {radii[-1]:.3f} step {r_step:.3f}  ({len(radii)} cols)")
    print(f"  z    : {', '.join(f'{v:.3f}' for v in z_list)}")
    print(f"  yaw  : {', '.join(f'{math.degrees(v):+.0f}' for v in yaw_list)}")
    print(f"  elbow ceiling: {elbow_ceiling}   wrist ceiling: {wrist_ceiling}")
    print("  legend: # reachable   e elbow-ceiling reject   w wrist reject")
    print("          L joint-limit reject   X self-collision   . no IK")
    if elbow_ceiling is not None or wrist_ceiling is not None:
        print("  NOTE: with a ceiling active, a genuinely colliding state that")
        print("        also trips the ceiling shows as 'e'/'w' on the map, not")
        print("        'X' -- ceilings are checked first and their label wins.")
        print("        'X' only appears for collisions the ceilings didn't")
        print("        catch. For an unmasked ground-truth map, use")
        print("        --no-elbow-ceiling --no-wrist-ceiling.")

    rows = []
    grid = {}
    disagreements = []

    for z in z_list:
        for r in radii:
            per_yaw = {}
            for yaw in yaw_list:
                quat = yaw_rotated_grasp_quat(yaw)
                x, y = r * math.cos(yaw), r * math.sin(yaw)
                state, label, reason = solve_ik_filtered(
                    io_client, x, y, z, quat, verbose=verbose,
                    elbow_ceiling=elbow_ceiling, wrist_ceiling=wrist_ceiling)

                if state is not None:
                    joints = [state[n] for n in JOINT_ORDER]
                    elbow = round(joints[ELBOW_JOINT_INDEX], 3)
                    wrist = round(joints[WRIST_JOINT_INDEX], 3)
                    code = "#"
                    actually_colliding = False
                else:
                    elbow = wrist = ""
                    rl = reason or ""
                    # actually_colliding is parsed from the reason text
                    # DIRECTLY, independent of which filter's label wins the
                    # code below. This matters: when a ceiling is active its
                    # label ("elbow ...", "wrist ...") always appears first in
                    # the reason string, ahead of collision status -- so code
                    # alone can NEVER show 'X' for a ceiling-masked collision.
                    # A tally built from `code == "X"` is therefore blind
                    # whenever a ceiling is on, and that blindness is exactly
                    # what produced a false "checker is dead" verdict on a run
                    # where the checker was, in fact, live.
                    actually_colliding = (
                        "[colliding]" in rl or rl.startswith("self-collision"))
                    if rl.startswith("elbow"):
                        code = "e"
                    elif rl.startswith("wrist"):
                        code = "w"
                    elif "near limit" in rl or "joint limits" in rl:
                        code = "L"
                    elif "collision" in rl:
                        code = "X"
                    else:
                        code = "."

                per_yaw[yaw] = code
                rows.append({
                    "z": round(z, 4), "r": round(r, 4),
                    "yaw_deg": round(math.degrees(yaw), 2),
                    "ok": state is not None, "code": code,
                    "actually_colliding": actually_colliding,
                    "elbow_j4": elbow, "wrist_j6o": wrist,
                    "seed": label or "", "reject": reason or "",
                })

            # Compare REACHABILITY across yaw, not the failure code. Which
            # seed happens to win (and therefore which ceiling fires first) is
            # KDL branch-jumping noise in the already-unreachable region -- it
            # says nothing about whether the planar assumption holds. Only a
            # cell that is reachable at one yaw and not another falsifies it.
            reach = {y: (c == "#") for y, c in per_yaw.items()}
            if len(set(reach.values())) > 1:
                disagreements.append((z, r, dict(per_yaw)))
            grid[(z, r)] = per_yaw

    # ---- ASCII map, one block per yaw ----
    for yaw in yaw_list:
        print(f"\n  yaw = {math.degrees(yaw):+.0f} deg")
        header = "    z\\r  " + "".join(f"{int(round(r * 100)):>4d}" for r in radii)
        print(header)
        print("         " + "cm".rjust(4 * len(radii)))
        for z in z_list:
            line = f"   {z:.3f} " + "".join(
                f"{grid[(z, r)][yaw]:>4s}" for r in radii)
            print(line)

    # ---- first reachable radius per z ----
    print("\n  Inner boundary (first reachable r, scanning outward):")
    for yaw in yaw_list:
        for z in z_list:
            first = None
            for r in radii:
                if grid[(z, r)][yaw] == "#":
                    first = r
                    break
            label = f"z={z:.3f} yaw={math.degrees(yaw):+.0f}"
            if first is None:
                print(f"    {label:22s} -> nothing reachable in range")
            else:
                print(f"    {label:22s} -> r >= {first:.3f} m")

    # ---- planar-assumption falsification ----
    if disagreements:
        print(f"\n  !! {len(disagreements)} (r,z) cells are REACHABLE at one yaw")
        print("     and not another. The planar assumption is violated -- the")
        print("     work zone is not a function of (r,z) alone.")
        for z, r, pv in disagreements[:8]:
            detail = "  ".join(f"{math.degrees(k):+.0f}:{v}" for k, v in pv.items())
            print(f"       r={r:.3f} z={z:.3f}   {detail}")
    else:
        print("\n  Planar assumption HOLDS: every (r,z) cell has the same")
        print("  reachability at every sampled yaw. The work zone really is a")
        print("  function of (r,z). (Failure CODES differ between yaws in the")
        print("  unreachable region -- that is just KDL picking different")
        print("  branches per seed, and is not evidence against the assumption.)")

    # ---- how much of the elbow rejection is real ----
    ceiling_rejects = [r for r in rows if r["code"] in ("e", "w")]
    n_real_collisions = sum(1 for r in rows if r["actually_colliding"])
    if ceiling_rejects:
        not_colliding = [r for r in ceiling_rejects if not r["actually_colliding"]]
        print(f"\n  Ceiling rejects: {len(ceiling_rejects)}, of which "
              f"{len(not_colliding)} are NOT actually self-colliding "
              f"(confirmed via reason text, independent of the map glyph --")
        print(f"  a ceiling reject always shows as 'e'/'w' on the map above "
              f"even when the underlying state truly collides).")
        print(f"  Real collisions detected this run (any cause): {n_real_collisions}")
        if len(ceiling_rejects) and n_real_collisions == 0:
            print("  !! Genuinely zero real collisions across the whole sweep.")
            print("     That's suspicious on its own even with the checker fix")
            print("     confirmed by --selftest-collision -- rerun --selftest-collision")
            print("     again before trusting this particular map.")
        elif not_colliding:
            print("  => the ceiling is over-conservative in this region. Re-run")
            print("     with --no-elbow-ceiling / --no-wrist-ceiling to recover")
            print("     that reachable volume (only the collision check remains")
            print("     as the arbiter, so X's will appear where it's genuinely")
            print("     unsafe).")

    path = os.path.join(outdir, "annulus_sweep_rz.csv")
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=[
            "z", "r", "yaw_deg", "ok", "code", "actually_colliding",
            "elbow_j4", "wrist_j6o", "seed", "reject"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nWrote {path}")
    return rows


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------

def move_to_vertex(io_client, point, z_override=None, verbose=False):
    """Joint-space plan to a single boundary vertex using seeded IK."""
    z = point["z"] if z_override is None else z_override
    quat = yaw_rotated_grasp_quat(point["yaw"])

    state, label, reason = solve_ik_filtered(
        io_client, point["x"], point["y"], z, quat, verbose=verbose)
    if state is None:
        print(f"    IK failed ({reason})")
        return False

    joint_trajectory = io_client.plan_motion([make_joint_goal_constraints(state)])
    if joint_trajectory is None:
        print(f"    OMPL planning FAILED (IK seed '{label}' was valid)")
        return False

    return io_client.arm_execute(joint_trajectory)


def cartesian_edge(io_client, edge_points, z_override=None,
                   execute=True, min_fraction=CARTESIAN_MIN_FRACTION):
    """One compute_cartesian_path call for a whole edge.

    NOTE: path_constraints is deliberately None. The orientation is
    yaw-rotated per waypoint, so a single fixed OrientationConstraint would
    be violated everywhere except one point on the arc. The waypoints
    themselves carry the orientation and MoveIt slerps between them; at
    ARC_STEP=0.02 m the waypoints are dense enough that the interpolation
    error stays small.

    Returns (fraction, executed_bool).
    """
    waypoints = []
    for p in edge_points:
        z = p["z"] if z_override is None else z_override
        waypoints.append(make_pose(p["x"], p["y"], z, yaw_rotated_grasp_quat(p["yaw"])))

    solution_msg, fraction = io_client.compute_cartesian_path(
        waypoints=waypoints,
        avoid_collisions=True,
        path_constraints=None,
    )

    n = len(waypoints)
    reached = fraction * n
    print(f"    fraction={fraction:.3f}  (~{reached:.1f}/{n} waypoints)")

    if solution_msg is None:
        return 0.0, False

    if fraction < min_fraction:
        print(f"    below min_fraction={min_fraction:.2f}, not executing this edge")
        return fraction, False

    if not execute:
        return fraction, False

    io_client.arm_execute(solution_msg.joint_trajectory)
    return fraction, True


def joint_chain_edge(io_client, edge_points, z_override=None, verbose=False,
                     dwell_sec=DWELL_SEC):
    """Chained joint-space plans, vertex to vertex, no go_home() in between.
    Each step is a small increment, which is what kept planning reliable in
    the spiral sweep. Returns count of vertices reached."""
    reached = 0
    for p in edge_points:
        print(f"    -> [{p['index']:3d}] r={p['r']:.3f} yaw={p['yaw_deg']:+7.2f}")
        if not move_to_vertex(io_client, p, z_override=z_override, verbose=verbose):
            print(f"    STOPPED at vertex {p['index']}")
            break
        reached += 1
        time.sleep(dwell_sec)
    return reached


def probe_edges(io_client, edges, z_override=None):
    """Cartesian fraction per edge, no execution. Note the fractions are
    measured from whatever the current state is, so this is a rough signal --
    run --execute for the real answer."""
    print("\n" + "=" * 68)
    print("CARTESIAN PROBE (planning only, no motion)")
    print("=" * 68)
    for edge in edges:
        print(f"\n  {edge[0]['edge']} ({len(edge)} waypoints)")
        cartesian_edge(io_client, edge, z_override=z_override,
                       execute=False, min_fraction=2.0)  # min>1 => never executes


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global ELBOW_CEILING, WRIST_CEILING

    parser = argparse.ArgumentParser(
        description="Trace the boundary of an annular-sector work zone.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--screen", action="store_true",
                      help="IK feasibility only, no motion (default)")
    mode.add_argument("--probe", action="store_true",
                      help="Cartesian fraction per edge, no motion")
    mode.add_argument("--execute", action="store_true",
                      help="Actually trace the boundary")
    mode.add_argument("--sweep-rz", action="store_true",
                      help="screen an (r,z) grid to find the real inner boundary")
    mode.add_argument("--selftest-collision", action="store_true",
                      help="prove the collision checker works before trusting it")
    mode.add_argument("--rviz", action="store_true",
                      help="publish the work zone as a translucent green "
                      "MarkerArray for RViz; no MoveIt/motion involved")

    parser.add_argument("--rviz-topic", default="/annulus_workzone")
    parser.add_argument("--rviz-frame", default=None,
                        help=f"marker frame_id (default: PLANNING_FRAME = "
                        f"'{PLANNING_FRAME}')")
    parser.add_argument("--rviz-z-bottom", type=float, default=None,
                        help="default: --z (the trace/grasp plane)")
    parser.add_argument("--rviz-z-top", type=float, default=None,
                        help="default: --z + HOVER_DZ (the hover height)")
    parser.add_argument("--rviz-rate", type=float, default=1.0,
                        help="publish rate in Hz (default 1.0)")
    parser.add_argument("--rviz-once", action="store_true",
                        help="publish a few times then exit, instead of "
                        "looping until Ctrl-C")

    parser.add_argument("--selftest-n", type=int, default=400,
                        help="random configs to sample in --selftest-collision")

    parser.add_argument("--sweep-r-lo", type=float, default=0.08)
    parser.add_argument("--sweep-r-hi", type=float, default=0.26)
    parser.add_argument("--sweep-r-step", type=float, default=0.01)
    parser.add_argument("--sweep-z", type=float, nargs="+",
                        default=[0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18],
                        help="z planes to sweep (m)")
    parser.add_argument("--sweep-yaw", type=float, nargs="+",
                        default=[0.0, 80.0, -80.0],
                        help="yaw values to sample, degrees")

    parser.add_argument("--elbow-ceiling", type=float, default=ELBOW_CEILING,
                        help="abs(joint4) ceiling. Off by default -- the real "
                        "collision checker is the arbiter. Pass a value "
                        "(e.g. 2.6) to reinstate a numeric cap.")
    parser.add_argument("--wrist-ceiling", type=float, default=WRIST_CEILING,
                        help=f"abs(joint6output) ceiling (default {WRIST_CEILING}, "
                        "validated against real collisions)")
    parser.add_argument("--no-elbow-ceiling", action="store_true",
                        help="no-op now that the elbow ceiling defaults to off; "
                        "kept for symmetry with --no-wrist-ceiling")
    parser.add_argument("--no-wrist-ceiling", action="store_true",
                        help="disable the wrist ceiling; let collision decide")

    parser.add_argument("--strategy", choices=["cartesian", "joint"],
                        default="cartesian",
                        help="how to traverse each edge in --execute mode")
    parser.add_argument("--r-inner", type=float, default=R_INNER)
    parser.add_argument("--r-outer", type=float, default=R_OUTER)
    parser.add_argument("--yaw-min", type=float, default=math.degrees(YAW_MIN),
                        help="degrees")
    parser.add_argument("--yaw-max", type=float, default=math.degrees(YAW_MAX),
                        help="degrees")
    parser.add_argument("--z", type=float, default=TRACE_Z)
    parser.add_argument("--arc-step", type=float, default=ARC_STEP)
    parser.add_argument("--radial-step", type=float, default=RADIAL_STEP)
    parser.add_argument("--dwell", type=float, default=DWELL_SEC,
                        help=f"seconds to pause at each vertex in "
                        f"--strategy joint (default {DWELL_SEC})")
    parser.add_argument("--gripper", action="store_true",
                        help="toggle gripper closed/open as a pre-start check")
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--outdir", default="/tmp")
    args = parser.parse_args()

    if not (args.probe or args.execute or args.sweep_rz
            or args.selftest_collision or args.rviz):
        args.screen = True

    # Ceilings are heuristics; let the CLI override or disable them.
    ELBOW_CEILING = None if args.no_elbow_ceiling else args.elbow_ceiling
    WRIST_CEILING = None if args.no_wrist_ceiling else args.wrist_ceiling

    yaw_min = math.radians(args.yaw_min)
    yaw_max = math.radians(args.yaw_max)

    # --- --rviz needs the boundary geometry (for the traced-outline overlay)
    # but NOT MoveIt -- no RobotIOClient, no planning scene, so it stays
    # lightweight.
    if args.rviz:
        points, _edges = generate_boundary(
            r_inner=args.r_inner, r_outer=args.r_outer,
            yaw_min=yaw_min, yaw_max=yaw_max, z=args.z,
            arc_step=args.arc_step, radial_step=args.radial_step)
        points = [p for p in points if p["edge"] != "close_loop"]

        z_bottom = args.z if args.rviz_z_bottom is None else args.rviz_z_bottom
        z_top = (args.z + HOVER_DZ) if args.rviz_z_top is None else args.rviz_z_top
        frame_id = args.rviz_frame or PLANNING_FRAME

        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
        try:
            publish_workzone(
                args.rviz_topic, frame_id,
                args.r_inner, args.r_outer, yaw_min, yaw_max,
                z_bottom, z_top, points,
                rate_hz=args.rviz_rate, once=args.rviz_once,
                verbose=args.verbose)
        finally:
            rclpy.shutdown()
        return

    # --- modes that don't need the boundary geometry ---
    if args.sweep_rz or args.selftest_collision:
        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
        time.sleep(1.5)
        io_client = RobotIOClient()
        if args.selftest_collision:
            selftest_collision(io_client, n_random=args.selftest_n,
                               verbose=args.verbose)
        else:
            sweep_rz(io_client,
                     args.sweep_r_lo, args.sweep_r_hi, args.sweep_r_step,
                     list(args.sweep_z),
                     [math.radians(v) for v in args.sweep_yaw],
                     ELBOW_CEILING, WRIST_CEILING,
                     args.outdir, verbose=args.verbose)
        io_client.destroy_node()
        rclpy.shutdown()
        return


    points, edges = generate_boundary(
        r_inner=args.r_inner, r_outer=args.r_outer,
        yaw_min=yaw_min, yaw_max=yaw_max, z=args.z,
        arc_step=args.arc_step, radial_step=args.radial_step)

    if args.max_points:
        points = points[:args.max_points]
        keep = {p["index"] for p in points}
        edges = [[p for p in e if p["index"] in keep] for e in edges]
        edges = [e for e in edges if e]

    print("=" * 68)
    print("ANNULAR SECTOR BOUNDARY TRACE")
    print("=" * 68)
    print(f"  r      : {args.r_inner:.3f} .. {args.r_outer:.3f} m")
    print(f"  yaw    : {args.yaw_min:+.1f} .. {args.yaw_max:+.1f} deg")
    print(f"  z      : {args.z:.3f} m   (hover {args.z + HOVER_DZ:.3f} m)")
    print(f"  steps  : arc {args.arc_step:.3f} m, radial {args.radial_step:.3f} m")
    print(f"  points : {len(points)} across {len(edges)} edges")
    print(f"  filters: |j4|<{ELBOW_CEILING if ELBOW_CEILING else 'off'}  "
          f"|j6output|<{WRIST_CEILING if WRIST_CEILING else 'off'}")
    print(f"  mode   : {'SCREEN' if args.screen else 'PROBE' if args.probe else 'EXECUTE/' + args.strategy}")

    if args.r_inner < 0.14 and args.z <= 0.14:
        print()
        print("  !! WARNING: r_inner = %.3f m at z = %.3f m." % (args.r_inner, args.z))
        print("     --sweep-rz --no-elbow-ceiling --no-wrist-ceiling (real, unmasked")
        print("     collision checking) found the true physical wall at z=0.14 is")
        print("     r=0.13, consistent across yaw 0/+80/-80. Below that the arm")
        print("     genuinely self-collides, confirmed by X's on the sweep map, not")
        print("     a filter guess. Re-run --sweep-rz at your target z if you need")
        print("     the wall confirmed at a different height.")
    elif args.r_inner < 0.10 and args.z >= 0.16:
        print()
        print("  !! WARNING: r_inner = %.3f m at z = %.3f m." % (args.r_inner, args.z))
        print("     The sweep found r=0.09 collides at yaw +-80 deg (though not at")
        print("     yaw 0) for z>=0.16 -- a real, yaw-DEPENDENT hazard near the base")
        print("     at this height, unlike z=0.14 where the wall was yaw-independent.")

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    time.sleep(1.5)

    io_client = RobotIOClient()

    try:
        # ---- SCREEN ----
        if args.screen:
            print("\n=== IK screen ===")
            results = screen_boundary(io_client, points, verbose=args.verbose)
            report_screen(results, edges)
            write_csv(results, os.path.join(args.outdir, "annulus_screen.csv"))
            return

        # ---- Everything below moves or plans against move_group ----
        print("\n=== Return to home pose ===")
        if not go_home(io_client):
            print("Could not reach home. Aborting.")
            return

        if args.gripper:
            print("\n=== Gripper pre-start check ===")
            io_client.gripper_move_to(GRIPPER_CLOSED)
            time.sleep(0.5)
            io_client.gripper_move_to(GRIPPER_OPEN)

        # ---- PROBE ----
        if args.probe:
            probe_edges(io_client, edges, z_override=args.z)
            return

        # ---- EXECUTE ----
        first = edges[0][0]

        print("\n=== Move to hover above first vertex ===")
        if not move_to_vertex(io_client, first,
                              z_override=args.z + HOVER_DZ, verbose=args.verbose):
            print("Could not reach the start hover pose. Aborting.")
            return

        print("\n=== Descend to trace plane (Cartesian) ===")
        frac, ok = cartesian_edge(io_client, [first], z_override=args.z)
        if not ok:
            print("Could not descend to the trace plane. Aborting.")
            return

        summary = []
        for edge in edges:
            name = edge[0]["edge"]
            print(f"\n=== Edge: {name} ({len(edge)} waypoints) ===")
            if args.strategy == "cartesian":
                frac, ok = cartesian_edge(io_client, edge, z_override=args.z)
                summary.append((name, f"fraction={frac:.3f}", "executed" if ok else "skipped"))
                if not ok:
                    print("    edge not executed -- stopping trace here")
                    break
            else:
                reached = joint_chain_edge(io_client, edge,
                                           z_override=args.z, verbose=args.verbose,
                                           dwell_sec=args.dwell)
                summary.append((name, f"{reached}/{len(edge)} vertices", ""))
                if reached < len(edge):
                    print("    edge incomplete -- stopping trace here")
                    break

        print("\n=== Retreat to hover ===")
        last = edges[-1][-1] if summary else first
        cartesian_edge(io_client, [last], z_override=args.z + HOVER_DZ)

        print("\n=== Return to home pose (final) ===")
        go_home(io_client)

        print("\n" + "=" * 68)
        print("TRACE SUMMARY")
        print("=" * 68)
        for name, detail, note in summary:
            print(f"  {name:16s}  {detail:24s} {note}")

    finally:
        io_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()