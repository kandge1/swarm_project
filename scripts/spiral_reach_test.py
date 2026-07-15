#!/usr/bin/env python3
"""
Spiral reachability sweep -- proof of concept that the arm can hover/descend/
retreat at arbitrary (x, y) points in its workspace.

IK solution filters:
  1. Base drift   -- abs(base - seed) < MAX_BASE_DRIFT (60°) -- kills reach-backward branch
  2. Elbow        -- abs(joint4) < ELBOW_CEILING (1.85) -- kills deep ±2.5 elbow branches
  3. Wrist spin   -- abs(joint6output) < WRIST_CEILING (0.90) -- kills joint5/gripper_base
                     contact caused by large wrist-spin compensation for base rotation.
                     Working pts 0-2 had joint6output=0.31..0.78. Pt 3 (fails) had 1.007.
  4. Joint limits -- standard near-limit rejection

FIX (regression from 7/43): The grasp quaternion was a fixed world-frame constant.
Every degree of base rotation forced joint6output to unwind by the same amount to
keep the gripper's world-yaw fixed. Solution: rotate the grasp quaternion by
target_yaw around world Z before passing it to IK and the Cartesian path
constraint. In the arm's own rotated frame, every target is at (r, 0, z) with
the same relative orientation, so joint6output stays near zero across the whole
spiral.

Geometry (measured from j4 vs radius/height data):
  GRASP_Z=0.14m, HOVER_DZ=0.06m (hover at 0.20m), SPIRAL_R0=0.21m

Execute mode always returns home after every point (success or failure) to
guarantee a clean start state for OMPL.

Requires pick_place.py importable from the same directory.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory

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
    build_moveit,
    go_home,
    make_grasp_pose,
    make_orientation_constraint,
    solve_ik_state,
    _is_near_joint_limit,
)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

SPIRAL_R0              = 0.21
SPIRAL_R_MAX           = 0.24
SPIRAL_GROWTH_PER_TURN = 0.02   # tighter growth to get more points in the smaller band
SPIRAL_ARC_STEP        = 0.014  # ~152 points in the 0.21-0.24m band

GRASP_Z  = 0.14
HOVER_DZ = 0.06   # hover at 0.20 m

DWELL_SEC              = 0.5
CARTESIAN_MIN_FRACTION = 0.90

# ---------------------------------------------------------------------------
# Joint indexing
# ---------------------------------------------------------------------------

BASE_JOINT       = "joint2_to_joint1"
JOINT_ORDER      = list(HOME_RADIANS.keys())
BASE_JOINT_INDEX = JOINT_ORDER.index(BASE_JOINT)

ELBOW_JOINT       = "joint4_to_joint3"
ELBOW_JOINT_INDEX = JOINT_ORDER.index(ELBOW_JOINT)

WRIST_JOINT       = "joint6output_to_joint6"
WRIST_JOINT_INDEX = JOINT_ORDER.index(WRIST_JOINT)

BASE_YAW_SIGN = +1.0
BASE_YAW_REF  =  0.0

CALIB_R           = 0.23
CALIB_PROBE_DELTA = 0.30

BASE_JOINT_LIMITS = (math.radians(-168.0), math.radians(168.0))
MAX_BASE_DRIFT    = math.radians(120.0)  # widened from 60: KDL drifts more at large azimuths

# abs(joint4) ceiling -- rejects deep ±2.5 elbow self-collision branches
ELBOW_CEILING = 1.85

# abs(joint6output) ceiling -- rejects wrist-spin configurations where
# joint5 physically contacts gripper_base.
WRIST_CEILING = 0.90


# ---------------------------------------------------------------------------
# Yaw-rotated orientation helpers
# ---------------------------------------------------------------------------

def yaw_rotated_grasp_quat(yaw):
    """
    Return q_yaw(yaw) * q_grasp, i.e. the fixed grasp quaternion rotated by
    'yaw' around world Z.

    When yaw=0 this returns (GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW) unchanged.

    This is the key fix: instead of asking IK to hit a fixed world-frame
    orientation (which forces joint6output to grow with every degree of base
    rotation), we rotate the target orientation by the same amount as the base.
    In the arm's own rotated frame the problem is always identical, so
    joint6output stays near zero for the entire spiral.

    Hamilton product (x,y,z,w) convention:
        q_yaw = (0, 0, sin(yaw/2), cos(yaw/2))
        result = q_yaw * q_grasp
    """
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    gx, gy, gz, gw = GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW
    # q_yaw = (0, 0, sy, cy)
    # Hamilton product: (a1 b2 - a2 b1 + ...) standard formula
    qx =  cy * gx + sy * gy
    qy = -sy * gx + cy * gy
    qz =  cy * gz + sy * gw
    qw =  cy * gw - sy * gz
    return qx, qy, qz, qw


def yawed_grasp_pose(x, y, z, yaw):
    """PoseStamped with position (x,y,z) and grasp orientation rotated by yaw."""
    qx, qy, qz, qw = yaw_rotated_grasp_quat(yaw)
    ps = PoseStamped()
    ps.header.frame_id = PLANNING_FRAME
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def yawed_orientation_constraint(yaw):
    """Orientation path constraint with grasp quaternion rotated by yaw."""
    qx, qy, qz, qw = yaw_rotated_grasp_quat(yaw)
    return make_orientation_constraint(POSE_LINK, PLANNING_FRAME, qx, qy, qz, qw)


# ---------------------------------------------------------------------------
# Spiral generation
# ---------------------------------------------------------------------------

def generate_spiral(r0=SPIRAL_R0, r_max=SPIRAL_R_MAX,
                    growth_per_turn=SPIRAL_GROWTH_PER_TURN,
                    arc_step=SPIRAL_ARC_STEP, z=GRASP_Z):
    b = growth_per_turn / (2.0 * math.pi)
    points, theta, idx = [], 0.0, 0
    while True:
        r = r0 + b * theta
        if r > r_max:
            break
        points.append({
            "index": idx,
            "x": r * math.cos(theta),
            "y": r * math.sin(theta),
            "z": z,
            "r": r,
            "theta_deg": math.degrees(theta) % 360.0,
        })
        theta += arc_step / math.sqrt(r * r + b * b)
        idx += 1
    return points


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# RobotState helpers
# ---------------------------------------------------------------------------

def current_joint_values(mycobot):
    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        return list(scene.current_state.get_joint_group_positions(GROUP_NAME))


def make_state(mycobot, values):
    state = RobotState(mycobot.get_robot_model())
    state.set_joint_group_positions(GROUP_NAME, list(values))
    state.update()
    return state


def with_base_yaw(values, yaw):
    v = list(values)
    v[BASE_JOINT_INDEX] = yaw
    return v


def base_yaw_of(state):
    return float(state.get_joint_group_positions(GROUP_NAME)[BASE_JOINT_INDEX])


def elbow_of(state):
    return float(state.get_joint_group_positions(GROUP_NAME)[ELBOW_JOINT_INDEX])


def wrist_of(state):
    return float(state.get_joint_group_positions(GROUP_NAME)[WRIST_JOINT_INDEX])


def joint_vec(state):
    return [round(v, 3) for v in state.get_joint_group_positions(GROUP_NAME)]


def fk_xy(state):
    tf = state.get_global_link_transform(POSE_LINK)
    return float(tf[0, 3]), float(tf[1, 3])


def state_within_bounds(state):
    try:
        return bool(state.satisfies_bounds())
    except (AttributeError, TypeError):
        lo, hi = BASE_JOINT_LIMITS
        return lo <= base_yaw_of(state) <= hi


# ---------------------------------------------------------------------------
# Base yaw calibration (no motion)
# ---------------------------------------------------------------------------

def calibrate_base_yaw(mycobot):
    global BASE_YAW_SIGN, BASE_YAW_REF

    cur = current_joint_values(mycobot)
    x0, y0 = fk_xy(make_state(mycobot, with_base_yaw(cur, 0.0)))
    x1, y1 = fk_xy(make_state(mycobot, with_base_yaw(cur, CALIB_PROBE_DELTA)))
    dphi = wrap_to_pi(math.atan2(y1, x1) - math.atan2(y0, x0))
    BASE_YAW_SIGN = 1.0 if dphi > 0 else -1.0

    print("\n[calib] base yaw sign (differential FK)")
    print(f"  base  0.000 -> EE ({x0:+.4f},{y0:+.4f})")
    print(f"  base {CALIB_PROBE_DELTA:+.3f} -> EE ({x1:+.4f},{y1:+.4f})")
    print(f"  d(azimuth)={dphi:+.4f} rad -> BASE_YAW_SIGN={BASE_YAW_SIGN:+.1f}")

    z = GRASP_Z + HOVER_DZ
    print(f"\n[calib] base yaw reference (IK at ({CALIB_R:.3f}, 0.000, {z:.3f}))")
    # Calibration point is at yaw=0, so yawed_grasp_pose == original grasp pose
    ref_state = solve_ik_state(mycobot, CALIB_R, 0.0, z,
                               GRASP_QX, GRASP_QY, GRASP_QZ, GRASP_QW)
    if ref_state is None:
        BASE_YAW_REF = 0.0
        print("  IK failed -- BASE_YAW_REF=0.0")
    else:
        BASE_YAW_REF = base_yaw_of(ref_state)
        print(f"  -> BASE_YAW_REF={BASE_YAW_REF:+.4f} rad "
              f"({math.degrees(BASE_YAW_REF):+.1f} deg)")
    print()


def azimuth_to_base_yaw(x, y):
    return wrap_to_pi(BASE_YAW_REF + BASE_YAW_SIGN * math.atan2(y, x))


# ---------------------------------------------------------------------------
# IK seeding and solving
# ---------------------------------------------------------------------------

def in_plane_seeds(mycobot, target_yaw, hint_state=None):
    current = dict(zip(JOINT_ORDER, current_joint_values(mycobot)))
    seeds = []

    if hint_state is not None:
        hint_vals = list(hint_state.get_joint_group_positions(GROUP_NAME))
        hint_dict = dict(zip(JOINT_ORDER, hint_vals))
        hint_dict[BASE_JOINT] = target_yaw
        seeds.append(("hint", hint_dict))

    s = dict(current)
    s[BASE_JOINT] = target_yaw
    s[WRIST_JOINT] = 0.0  # seed neutral so KDL finds near-zero solution
    seeds.append(("current-yawed", s))

    # Explicit home-shape + zero wrist -- best bet for the ±pi dead zone
    s = dict(HOME_RADIANS)
    s[BASE_JOINT] = target_yaw
    s[WRIST_JOINT] = 0.0
    seeds.append(("home-zero-wrist", s))

    for label, j3, j4, j5 in (
        ("shallow-A",   0.50, -0.55, -0.45),
        ("shallow-B",   0.20, -0.90,  0.05),
        ("shallow-C",  -0.10, -1.25,  0.20),
        ("shallow-D",  -0.30, -1.60,  0.30),
    ):
        s = dict(current)
        s[BASE_JOINT] = target_yaw
        s["joint3_to_joint2"] = j3
        s["joint4_to_joint3"] = j4
        s["joint5_to_joint4"] = j5
        s["joint6_to_joint5"] = 0.0
        # joint6output is a mimic of the base joint -- seed it at the correct
        # value so KDL starts from a mechanically consistent state
        s[WRIST_JOINT] = 0.0  # seed neutral; KDL should find near-zero solution
        seeds.append((label, s))

    for label, seed in IK_SEEDS:
        s = dict(seed)
        s[BASE_JOINT] = target_yaw
        s[WRIST_JOINT] = 0.0  # seed neutral; KDL should find near-zero solution  # keep mimic consistent in pp seeds too
        seeds.append((f"pp:{label}", s))

    # Dead-zone recovery: near base=±pi the mimic joint hits its limit.
    # Try the equivalent angle from the opposite wrap direction.
    # target_yaw - 2pi brings a ~96deg target to ~-264deg which KDL may
    # reject on limits, but target_yaw approached from negative is valid
    # for base joints with symmetric ±168deg limits.
    for wrap_offset, wrap_label in ((-2*math.pi, "wrap-neg"), (+2*math.pi, "wrap-pos")):
        alt_yaw = target_yaw + wrap_offset
        lo, hi = BASE_JOINT_LIMITS
        if not (lo <= alt_yaw <= hi):
            continue
        for label, j3, j4, j5 in (
            ("shallow-A",  0.50, -0.55, -0.45),
            ("shallow-B",  0.20, -0.90,  0.05),
        ):
            s = dict(current)
            s[BASE_JOINT] = alt_yaw
            s["joint3_to_joint2"] = j3
            s["joint4_to_joint3"] = j4
            s["joint5_to_joint4"] = j5
            s["joint6_to_joint5"] = 0.0
            s[WRIST_JOINT] = 0.0  # seed neutral for wrap seeds too
            seeds.append((f"{wrap_label}-{label}", s))

    return seeds


def solve_ik_filtered(mycobot, x, y, z, desired_yaw,
                      verbose=True, hint_state=None):
    """
    Solve IK for (x, y, z) with the grasp orientation rotated by desired_yaw.

    Using a yaw-rotated pose means the IK problem in the arm's own frame is
    identical for every azimuth, so joint6output stays near zero across the
    whole spiral instead of winding up with each degree of base rotation.
    """
    robot_model = mycobot.get_robot_model()
    # Rotate by delta from BASE_YAW_REF, not absolute yaw.
    # The grasp quaternion was calibrated at base=BASE_YAW_REF; we only need
    # to rotate by how much the base moves beyond that reference point.
    yaw_delta = wrap_to_pi(desired_yaw - BASE_YAW_REF)
    pose = yawed_grasp_pose(x, y, z, yaw_delta)

    for label, seed in in_plane_seeds(mycobot, desired_yaw, hint_state):
        state = RobotState(robot_model)
        state.set_joint_group_positions(GROUP_NAME, list(seed.values()))
        state.update()

        if not state.set_from_ik(GROUP_NAME, pose.pose, POSE_LINK, timeout=0.5):
            if verbose:
                print(f"  [ik] '{label}': no convergence")
            continue

        # 1. Base drift filter
        drift = wrap_to_pi(base_yaw_of(state) - desired_yaw)
        if abs(drift) > MAX_BASE_DRIFT:
            if verbose:
                print(f"  [ik] '{label}': base drift {math.degrees(drift):+.0f}°, skip")
            continue

        # 2. Elbow filter (catches ±2.5 deep branches)
        j4 = elbow_of(state)
        if abs(j4) > ELBOW_CEILING:
            if verbose:
                print(f"  [ik] '{label}': |j4|={abs(j4):.2f} > {ELBOW_CEILING:.2f}, skip")
            continue

        # 3. Joint limit filter  (wrist ceiling removed: joint6output is a mimic
        #    of the base joint and legitimately grows with base angle)
        near_limit, lname, lval, llo, lhi = _is_near_joint_limit(state)
        if near_limit:
            if verbose:
                print(f"  [ik] '{label}': '{lname}'={lval:.3f} near limit, skip")
            continue

        if verbose:
            print(f"  [ik] '{label}': OK j4={j4:.2f} w={wrist_of(state):.2f} -> {joint_vec(state)}")
        return state, label

    return None, None


def diagnose_ik(mycobot, x, y, z, desired_yaw):
    robot_model = mycobot.get_robot_model()
    yaw_delta = wrap_to_pi(desired_yaw - BASE_YAW_REF)
    pose = yawed_grasp_pose(x, y, z, yaw_delta)  # delta from ref, same as solve_ik_filtered
    current = dict(zip(JOINT_ORDER, current_joint_values(mycobot)))

    for label, j3, j4_seed, j5 in (
        ("s-shallow",   0.50, -0.55, -0.45),
        ("s-mid",       0.00, -1.00,  0.05),
        ("s-neg-deep", -0.50, -1.80,  0.35),
        ("s-pos-deep",  0.50,  1.80, -0.35),
    ):
        s = dict(current)
        s[BASE_JOINT] = desired_yaw
        s["joint3_to_joint2"] = j3
        s["joint4_to_joint3"] = j4_seed
        s["joint5_to_joint4"] = j5
        s["joint6_to_joint5"] = 0.0
        s["joint6output_to_joint6"] = 0.0
        state = RobotState(robot_model)
        state.set_joint_group_positions(GROUP_NAME, list(s.values()))
        state.update()
        if state.set_from_ik(GROUP_NAME, pose.pose, POSE_LINK, timeout=0.5):
            j4 = elbow_of(state)
            w = wrist_of(state)
            passes = abs(j4) <= ELBOW_CEILING and abs(w) <= WRIST_CEILING
            flag = " *** PASSES" if passes else ""
            print(f"  [diag] '{label}': j4={j4:.3f} w={w:.3f}{flag}")
        else:
            print(f"  [diag] '{label}': no IK")


# ---------------------------------------------------------------------------
# Motion stages
# ---------------------------------------------------------------------------

def yaw_to(mycobot, arm, target_yaw):
    goal_state = make_state(mycobot,
                            with_base_yaw(current_joint_values(mycobot), target_yaw))
    if not state_within_bounds(goal_state):
        print(f"[yaw] {math.degrees(target_yaw):+.1f}° outside base limits")
        return False, "yaw_out_of_bounds"

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)
    plan_result = arm.plan()
    if not plan_result:
        print("[yaw] planning FAILED")
        return False, "yaw_plan_failed"

    print(f"[yaw] base -> {math.degrees(target_yaw):+.1f}°")
    mycobot.execute(plan_result.trajectory, controllers=["arm_group_controller"])
    return True, None


def plan_to_state(mycobot, arm, goal_state, tag="reach"):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)
    plan_result = arm.plan()
    if not plan_result:
        print(f"[{tag}] planning FAILED")
        return False, f"{tag}_plan_failed"

    j4 = elbow_of(goal_state)
    w = wrist_of(goal_state)
    print(f"[{tag}] -> j4={j4:.2f} w={w:.2f} {joint_vec(goal_state)}")
    mycobot.execute(plan_result.trajectory, controllers=["arm_group_controller"])
    return True, None


def cartesian_z(mycobot, io_client, x, y, z, yaw, tag="cartesian"):
    """
    Straight-line Cartesian move to (x, y, z).
    No orientation constraint -- the hover state already has the arm
    positioned correctly, and a purely vertical descent at fixed (x,y)
    doesn't need one. Constraints were causing fraction drops as joint6output
    needed to reconfigure mid-descent.
    """
    solution_msg, fraction = io_client.compute_cartesian_path(
        waypoints=[make_grasp_pose(x, y, z)],
        avoid_collisions=True,
        path_constraints=Constraints(),
    )
    if solution_msg is None or fraction < CARTESIAN_MIN_FRACTION:
        print(f"[{tag}] FAILED fraction={fraction:.2f}")
        return False, f"{tag}_failed"

    print(f"[{tag}] OK fraction={fraction:.2f}")
    trajectory = RobotTrajectory(mycobot.get_robot_model())
    trajectory.joint_model_group_name = GROUP_NAME
    psm = mycobot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        trajectory.set_robot_trajectory_msg(scene.current_state, solution_msg)
    mycobot.execute(trajectory, controllers=["arm_group_controller"])
    return True, None


# ---------------------------------------------------------------------------
# Per-point sequence
# ---------------------------------------------------------------------------

def visit_point_screen(mycobot, pt, verbose=False, run_diagnose=False):
    x, y, z = pt["x"], pt["y"], pt["z"]
    hover_z = z + HOVER_DZ
    seed = azimuth_to_base_yaw(x, y)

    hover_state, label = solve_ik_filtered(mycobot, x, y, hover_z, seed,
                                           verbose=verbose)
    if hover_state is None:
        print(f"  [resolve] no valid hover IK (seed {math.degrees(seed):+.1f}°)")
        if run_diagnose:
            diagnose_ik(mycobot, x, y, hover_z, seed)
        return False, "hover_ik_failed", math.degrees(seed)

    target_yaw = base_yaw_of(hover_state)
    j4 = elbow_of(hover_state)
    w = wrist_of(hover_state)
    drift = math.degrees(wrap_to_pi(target_yaw - seed))
    print(f"  [resolve] seed {math.degrees(seed):+.1f}° -> base "
          f"{math.degrees(target_yaw):+.1f}° (drift {drift:+.1f}) "
          f"j4={j4:.2f} w={w:.2f} '{label}'")

    grasp_state, glabel = solve_ik_filtered(
        mycobot, x, y, z, target_yaw, verbose=False, hint_state=hover_state)
    if grasp_state is None:
        print(f"  [grasp-ik] FAILED")
        if run_diagnose:
            diagnose_ik(mycobot, x, y, z, target_yaw)
        return False, "grasp_ik_failed", math.degrees(target_yaw)
    print(f"  [grasp-ik] OK j4={elbow_of(grasp_state):.2f} "
          f"w={wrist_of(grasp_state):.2f} '{glabel}'")
    return True, None, math.degrees(target_yaw)


def visit_point_execute(mycobot, arm, io_client, pt,
                        use_gripper=False, verbose=False, run_diagnose=False):
    x, y, z = pt["x"], pt["y"], pt["z"]
    hover_z = z + HOVER_DZ
    seed = azimuth_to_base_yaw(x, y)

    hover_state, label = solve_ik_filtered(mycobot, x, y, hover_z, seed,
                                           verbose=verbose)
    if hover_state is None:
        print(f"  [resolve] no valid hover IK (seed {math.degrees(seed):+.1f}°)")
        if run_diagnose:
            diagnose_ik(mycobot, x, y, hover_z, seed)
        go_home(mycobot, arm)
        return False, "hover_ik_failed", math.degrees(seed)

    target_yaw = base_yaw_of(hover_state)
    j4 = elbow_of(hover_state)
    w = wrist_of(hover_state)
    drift = math.degrees(wrap_to_pi(target_yaw - seed))
    print(f"  [resolve] seed {math.degrees(seed):+.1f}° -> base "
          f"{math.degrees(target_yaw):+.1f}° (drift {drift:+.1f}) "
          f"j4={j4:.2f} w={w:.2f} '{label}'")

    ok, stage = yaw_to(mycobot, arm, target_yaw)
    if not ok:
        go_home(mycobot, arm)
        return False, stage, math.degrees(target_yaw)

    ok, stage = plan_to_state(mycobot, arm, hover_state, tag="hover")
    if not ok:
        go_home(mycobot, arm)
        return False, stage, math.degrees(target_yaw)

    ok, stage = cartesian_z(mycobot, io_client, x, y, z, target_yaw, tag="descend")
    if not ok:
        go_home(mycobot, arm)
        return False, stage, math.degrees(target_yaw)

    if use_gripper:
        io_client.gripper_move_to(GRIPPER_CLOSED)
        time.sleep(DWELL_SEC)
        io_client.gripper_move_to(GRIPPER_OPEN)
    else:
        time.sleep(DWELL_SEC)

    ok, stage = cartesian_z(mycobot, io_client, x, y, hover_z, target_yaw, tag="ascend")
    if not ok:
        go_home(mycobot, arm)
        return False, stage, math.degrees(target_yaw)

    # Do NOT go home between points. Chaining hover->descend->ascend->next_hover
    # keeps joint6output changes incremental (avoids the singularity from
    # zeroing it, and avoids the OMPL failure when it has to plan from home
    # to a state with joint6output ~1.0+).
    return True, None, math.degrees(target_yaw)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def write_csv(results, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "index", "x", "y", "z", "r", "theta_deg",
            "base_yaw_deg", "success", "failed_stage"])
        writer.writeheader()
        writer.writerows(results)
    print(f"\nWrote {path}")


def summarize(results):
    total = len(results)
    if not total:
        return
    ok  = [r for r in results if r["success"]]
    bad = [r for r in results if not r["success"]]

    print("\n" + "=" * 62)
    print(f"SPIRAL SWEEP: {len(ok)}/{total} points succeeded "
          f"({100.0*len(ok)/total:.1f}%)")
    print("=" * 62)

    if ok:
        radii = [r["r"] for r in ok]
        print(f"  reachable radius: {min(radii):.3f} m .. {max(radii):.3f} m")
        safe = SPIRAL_R0
        for r in sorted(results, key=lambda d: d["r"]):
            if not r["success"]:
                break
            safe = r["r"]
        print(f"  contiguous safe envelope: r <= {safe:.3f} m")

    if bad:
        stages = {}
        for r in bad:
            stages[r["failed_stage"]] = stages.get(r["failed_stage"], 0) + 1
        print("\n  failures by stage:")
        for stage, count in sorted(stages.items(), key=lambda kv: -kv[1]):
            print(f"    {stage:<22} {count}")
        print("\n  first failure per quadrant:")
        for lo, hi, name in ((0,90,"Q1 +x+y"),(90,180,"Q2 -x+y"),
                             (180,270,"Q3 -x-y"),(270,360,"Q4 +x-y")):
            q = [r for r in bad if lo <= r["theta_deg"] < hi]
            if q:
                f0 = min(q, key=lambda d: d["r"])
                print(f"    {name}: r={f0['r']:.3f} m  ({f0['failed_stage']})")
            else:
                print(f"    {name}: none")


def plot(results, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed -- skipping plot)")
        return

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot([r["x"] for r in results], [r["y"] for r in results],
            "-", color="0.85", linewidth=0.7, zorder=0)
    for r in results:
        ax.plot(r["x"], r["y"], "o",
                color="tab:green" if r["success"] else "tab:red", markersize=6)
    for radius, style in ((SPIRAL_R0, ":"), (SPIRAL_R_MAX, "--")):
        t = [i * 2 * math.pi / 72 for i in range(73)]
        ax.plot([radius*math.cos(a) for a in t],
                [radius*math.sin(a) for a in t],
                style, color="0.6", linewidth=0.8)
    ax.plot(0, 0, "k+", markersize=12)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Reachability: grasp z={GRASP_Z:.2f}m hover z={GRASP_Z+HOVER_DZ:.2f}m "
                 f"(green=ok, red=fail)")
    ax.grid(alpha=0.3)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"Wrote {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--screen", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--diagnose", action="store_true")
    parser.add_argument("--outdir", default="/tmp")
    args = parser.parse_args()

    execute = args.execute

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    time.sleep(1.5)

    mycobot = build_moveit()
    arm = mycobot.get_planning_component(GROUP_NAME)
    io_client = RobotIOClient()

    points = generate_spiral()
    if args.max_points:
        points = points[:args.max_points]

    print(f"Generated {len(points)} spiral points, "
          f"r={SPIRAL_R0:.2f}..{SPIRAL_R_MAX:.2f} m")
    print(f"Mode: {'EXECUTE (home after every point)' if execute else 'SCREEN (no motion)'}")
    print(f"Grasp z={GRASP_Z:.2f}m  Hover z={GRASP_Z+HOVER_DZ:.2f}m")
    print(f"Filters: |j4|<{ELBOW_CEILING}  (wrist filter removed: joint6output is mimic joint)")
    print(f"Orientation: yaw-rotated with base (joint6output stays near 0 across spiral)")

    # Quaternion sanity check -- delta=0 must equal base grasp quat exactly
    print(f"\n[quat] base grasp (delta=0): x={GRASP_QX:.4f} y={GRASP_QY:.4f} "
          f"z={GRASP_QZ:.4f} w={GRASP_QW:.4f}  "
          f"norm={math.sqrt(GRASP_QX**2+GRASP_QY**2+GRASP_QZ**2+GRASP_QW**2):.4f}")
    print(f"[quat] (all deltas relative to BASE_YAW_REF, set after calibration)")
    for deg in (0, 45, 90, 135, 180):
        delta = math.radians(deg)
        qx, qy, qz, qw = yaw_rotated_grasp_quat(delta)
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        print(f"[quat] delta={deg:>4}°  x={qx:.4f} y={qy:.4f} z={qz:.4f} w={qw:.4f}  norm={norm:.4f}")

    print("\n=== Return to home pose ===")
    if not go_home(mycobot, arm):
        print("Could not reach home. Aborting.")
        return

    calibrate_base_yaw(mycobot)

    results = []
    for pt in points:
        print(f"\n=== pt {pt['index']:>3}  r={pt['r']:.3f}  "
              f"θ={pt['theta_deg']:6.1f}°  ({pt['x']:+.3f},{pt['y']:+.3f}) ===")
        try:
            if execute:
                ok, stage, yaw_deg = visit_point_execute(
                    mycobot, arm, io_client, pt,
                    use_gripper=args.gripper,
                    verbose=args.verbose,
                    run_diagnose=args.diagnose)
            else:
                ok, stage, yaw_deg = visit_point_screen(
                    mycobot, pt,
                    verbose=args.verbose,
                    run_diagnose=args.diagnose)
        except Exception as exc:
            print(f"  EXCEPTION: {exc!r}")
            ok, stage, yaw_deg = False, "exception", float("nan")
            if execute:
                go_home(mycobot, arm)

        results.append({
            "index": pt["index"],
            "x": round(pt["x"], 4), "y": round(pt["y"], 4), "z": round(pt["z"], 4),
            "r": round(pt["r"], 4), "theta_deg": round(pt["theta_deg"], 2),
            "base_yaw_deg": round(yaw_deg, 2),
            "success": ok, "failed_stage": stage or "",
        })

        if not ok:
            print(f"  FAILED at '{stage}'")

    if execute:
        print("\n=== Return to home (final) ===")
        go_home(mycobot, arm)

    summarize(results)
    write_csv(results, os.path.join(args.outdir, "spiral_results.csv"))
    plot(results, os.path.join(args.outdir, "spiral_results.png"))

    io_client.destroy_node()
    del mycobot
    rclpy.shutdown()

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()