#!/usr/bin/env python3
"""
zone_calibrate.py -- drive the jaws to the centre and the four tag vertices of a
zone, and record where they actually ended up.

RUN THIS ON MARS, with Terminal 1 (robot hardware) and Terminal 2 (mars
planning) up. No vision, no DetectBlock service, no camera -- this measures the
ARM against the zone geometry that zone_vision.py already defines, so that when
vision does come back into the loop there is a known-good mapping underneath it.

WHY, in one line: every calibration constant in pick_place.py was fitted at ONE
point -- the zone centre, (0, +0.2286) -- and nothing has ever checked whether
it still holds 2 inches away in any direction.

WHAT THIS CAN AND CANNOT SEE
----------------------------
The CSV has two very different kinds of column and mixing them up wastes a
hardware session:

  FK columns (fk_flange_*, fk_grip_*, err_*) come from /joint_states through
  forward kinematics. They answer "did the arm reach the joint angles it was
  commanded to". Droop, backlash and the servo dead zone all land here, which
  is exactly the residual a feedforward is supposed to remove -- so THIS is the
  column set worth turning into a lookup table.

  Hand columns (meas_*, from --interactive) are the only thing that can see a
  disagreement between the URDF and the physical robot. A wrong GRASP_OFFSET_Z,
  a wrong JAW_RADIAL_OFFSET_M, a gripper that hangs 3 mm off where the model
  says -- FK is blind to all of it, by construction. It reports the model's
  opinion of where the tool is, and the model is the thing under test.

So: FK error large  -> control problem, feedforward territory.
    FK error small but the tape disagrees -> geometry constant is wrong.
That split is the entire point of running this instead of eyeballing a grasp.

REACH IS THE FIRST THING THIS TELLS YOU
---------------------------------------
The pre-flight runs before any motion and screens every waypoint twice: against
MAX_FLANGE_RADIUS_M, then against real IK. Points that fail are SKIPPED, not
attempted, because move_arm_to's OMPL fallback will happily "succeed" at an
unreachable target by parking the flange short and tilted inside its 4 cm
position sphere -- which looks like a completed move and poisons every number
measured at it. tag_pick_place.py's MAX_FLANGE_RADIUS_M comment documents the
runaway that came out of exactly that.

Three things the first version of this pre-flight got wrong, all fixed. They all
made the arm look less capable than it is:

  REACH IS NOT A SCALAR. It screened against a flat 0.245, which is the limit at
  a flange z of ~0.220 because it was measured at a 0.280 hover. At the grasp
  height the real limit is 0.2793 -- 34 mm further. Worse, the screen REJECTED
  points on it before IK was ever called, so corners the arm can plainly reach
  were thrown out by a constant and the solver was blamed. The radius map is now
  advisory and height-aware (pick_place.max_flange_radius), and IK decides.

  WHICH SQUARE. It screened the TAG square, and the tags are fiducials the arm
  never has to reach. What must be reachable is the range a block's CENTRE may
  occupy -- 1.82 in against the tag square's 4 in. See square_sides(), and
  --square.

  RADIUS IS NOT THE ONLY BOUND. A corner at flange radius 0.1472 failed IK
  while its mirror image at 0.1468 solved. That is the WRIST, not reach: the
  jaws hold a fixed world yaw, so joint6output has to absorb the bearing change
  and it ran off its limit by 1.9 deg. See YAW_RETRIES_DEG.

Net result: at the current 9 in zone centre the whole block-centre square is
reachable, and the mat does not need to move.

Use --dry-run to see the reach map with the robot untouched, and --zone-radius
to ask the same question about a zone that has not been taped down yet.
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pick_place as pp  # noqa: E402
import zone_vision as zv  # noqa: E402

# REACH IS pick_place.max_flange_radius(z), NOT A CONSTANT.
#
# This script used to screen against tag_pick_place.MAX_FLANGE_RADIUS_M = 0.245,
# a flat number, and REJECTED points on it before IK ever saw them. That was
# wrong twice over: 0.245 is the limit at a flange z of ~0.220 (it was measured
# at a 0.280 hover), and at the grasp height of ~0.151 the real limit is 0.2793
# -- 34 mm further. Corners the arm can plainly reach were being thrown out by
# the screen, not by the solver.
#
# tag_pick_place still uses the flat constant, correctly, for a different job:
# clamping the vision correction loop at the DETECTION hover, where a runaway
# pushed the flange outward until planning failed. That guard is about one
# specific height, so one number is fine there. It is not fine here.
JOINT_NAMES = list(pp.HOME_RADIANS.keys())

CSV_FIELDS = [
    "label", "tag_id", "repeat", "grasp_yaw_deg",
    "tip_x", "tip_y", "tip_z",
    "cmd_x", "cmd_y", "cmd_z",
    "fk_flange_x", "fk_flange_y", "fk_flange_z",
    "fk_grip_x", "fk_grip_y", "fk_grip_z",
    "err_x_mm", "err_y_mm", "err_z_mm",
    "cmd_radius_mm", "fk_radius_mm", "grip_radius_mm", "grip_above_mat_mm",
] + [f"j{i + 1}_rad" for i in range(6)] + [
    "meas_height_mm", "meas_radius_in", "note",
]


def square_sides(zone_size, tag_size, block_size):
    """The THREE nested squares a zone actually has, and which one a survey
    should care about.

    Conflating these is the trap zone_vision.py's DEFAULT_ZONE_SIZE comment
    already warns about, and it is what made the first version of this script
    report a far harsher reach limit than the truth:

      tags   the square joining the four TAG CENTRES. The arm never has to
             reach a tag -- they are fiducials, not targets.
      span   the clear square between the tags' INNER edges. Physically
             unobstructed, but a block sitting with its centre on the edge
             overhangs it.
      blocks the range a block's CENTRE may occupy. THIS is the one that has to
             be reachable for a detected block to be pickable, and it is the
             smallest by 2*tag_size + ... -- more than half the tag square.

    Returns [(kind, side_metres)] outermost first.
    """
    return [
        ("tags", zone_size),
        ("span", zone_size - tag_size),
        ("blocks", zone_size - tag_size - block_size),
    ]


def corner_label(zone_name, sx, sy):
    """Human name for a corner, with "near" meaning nearer the robot on
    whichever side of the base this zone sits."""
    sign = -1.0 if zone_name == "place" else +1.0
    near = "far" if sy * sign > 0 else "near"
    side = "R" if sx > 0 else "L"
    return f"{near}-{side}"


def zone_waypoints(zone_name, centre_radius, side, kind="blocks",
                   include_centre=True, include_vertices=True):
    """[(label, tag_id, x, y)] for a zone's centre and the four corners of the
    chosen square, in robot base coordinates.

    Corner ORDER follows zone_vision.ZONE_CORNER_SIGNS so that when kind is
    "tags" -- and only then, since only then is a vertex a tag centre -- each
    measurement pairs with the tag id printed on the mat. Keeping the order
    fixed across kinds means the CSV columns line up between surveys.

    Sign convention: the pickup zone sits at +Y, the place zone at -Y. The
    VERTEX offsets are not flipped with it -- ZoneSpec.zone_to_world at
    world_yaw = 0 makes zone-local axes world axes, so zone +Y is world +Y for
    both zones.
    """
    sign = -1.0 if zone_name == "place" else +1.0
    tag_ids = (zv.PLACE_TAG_IDS if zone_name == "place" else zv.PICKUP_TAG_IDS)
    cy = sign * centre_radius
    half = side / 2.0

    points = []
    if include_centre:
        points.append(("centre", None, 0.0, cy))
    if include_vertices:
        for tag_id, (sx, sy) in zip(tag_ids, zv.ZONE_CORNER_SIGNS):
            if kind == "tags":
                label, tid = f"tag{tag_id}", tag_id
            else:
                label, tid = corner_label(zone_name, sx, sy), None
            points.append((label, tid, sx * half, cy + sy * half))
    return points


def max_zone_centre_radius(side, zone_name="pickup", flange_z=0.1508):
    """Largest zone-centre radius at which all four corners of `side` are still
    inside the reach envelope AT flange height `flange_z`.

    Derived, not tuned: the binding vertex is a far corner, whose flange target
    sits at (side/2, Yc + side/2 + push) where `push` is how far
    compensate_for_tip_swing shoves the flange outward to put the JAWS on the
    target. Solve hypot(x, y) = max_flange_radius(flange_z) for Yc.

    flange_z MATTERS AND IS THE POINT. The envelope shrinks with height, so the
    answer at the grasp is very different from the answer at a hover 40 mm up --
    and it is the hover that binds. Call it twice.

    This is the OUTER geometric bound only, and not the whole story: a corner
    well inside it can still fail because the WRIST runs out of travel at that
    bearing (see YAW_RETRIES_DEG). Advisory. IK decides.

`push` is RADIAL (see JAW_RADIAL_OFFSET_M -- it was modelled as world-frame
    until the place survey proved otherwise), so it points outward at both zones
    and the sign no longer differs between them. It is still sampled per zone
    because the tilt, and therefore the tip swing folded into it, does differ.
    """
    half = side / 2.0
    # Sampled ON the zone's own bearing, not at the origin. The lateral terms
    # are RADIAL now, so r_hat has to be defined -- at (0, 0) they are dropped
    # entirely and this would read a push of zero.
    probe_y = -0.2 if zone_name == "place" else 0.2
    px, py, _ = pp.compensate_for_tip_swing(0.0, probe_y, 0.0)
    push_x = px
    push_y = abs(py - probe_y)   # outward along this zone's radius
    # push_x is small but signed, so it pushes one of the two far corners
    # further out than the other -- take the worse one, or this reports a
    # limit at which one corner is still a fraction of a millimetre over.
    half_x = half + abs(push_x)
    limit = pp.max_flange_radius(flange_z)
    inner = limit ** 2 - half_x ** 2
    if inner <= 0.0:
        return 0.0
    return math.sqrt(inner) - half - push_y


def flange_target(x, y, tip_z, yaw_deg=0.0):
    """The flange z for a tip at height tip_z, and the fully compensated,
    clamped (x, y, z) that will actually be commanded. Mirrors what
    move_arm_to/make_grasp_pose do internally, so the pre-flight screens the
    real target rather than the caller's.

    yaw_deg matters: the mount-tilt tip offset is a TOOL-frame vector, so it
    rotates with the grasp yaw. Screening at one yaw and moving at another
    would check a target the arm is never asked for."""
    fz = tip_z + pp.GRASP_OFFSET_Z
    cx, cy, cz = pp.compensate_for_tip_swing(x, y, fz, yaw_deg)
    return fz, (cx, cy, pp.clamp_flange_z(cz, "zone_calibrate"))


# Yaws to try, in order, when the default grasp yaw fails IK.
#
# FREE FOR A SQUARE BLOCK. A parallel gripper closing on a cube is invariant
# under a quarter turn -- the jaws meet the same pair of opposite faces either
# way -- so rotating the grasp 90 deg costs nothing physically and is already a
# supported per-call argument (block_yaw_deg), not a global override.
#
# WHY IT IS NEEDED, measured 2026-08-03 at a 6.96 in zone centre: the near-left
# corner failed IK at a flange radius of 0.1472 while its MIRROR IMAGE at 0.1468
# solved cleanly. That is not reach. The jaws hold a fixed WORLD yaw
# (GRIPPER_YAW_DEG), so as J1 swings to a different bearing the wrist has to
# take up the difference, and joint6output ran off its limit by 0.032 rad --
# 1.9 deg. A quarter turn moves it 90 deg back into the middle of its range.
YAW_RETRIES_DEG = (0.0, 90.0, -90.0, 180.0)


def hover_flange(x, y, tip_z, yaw_deg=0.0):
    """The compensated flange target for the pre-grasp hover above (x, y).

    Uses the RADIUS-AWARE hover_z. The envelope shrinks with height, so at the
    outer edge of a zone the grasp is comfortably reachable while a hover 40 mm
    above it is not -- the descent's starting point fails, not the grasp, and it
    surfaces as an IK miss that reads like a reach problem at the target."""
    fz, (cx, cy, _cz) = flange_target(x, y, tip_z, yaw_deg)
    hz = pp.hover_z(fz, radius=math.hypot(cx, cy))
    return pp.compensate_for_tip_swing(x, y, hz, yaw_deg)


def _solves(io_client, x, y, tip_z, yaw_deg):
    """Does the downward grasp at this point solve at BOTH the hover and the
    grasp height, at this yaw?"""
    _fz, (cx, cy, cz) = flange_target(x, y, tip_z, yaw_deg)
    hcx, hcy, hcz = hover_flange(x, y, tip_z, yaw_deg)
    q = pp.grasp_quat_for(yaw_deg, x, y, False)
    for _what, (zx, zy, zz) in (("hover", (hcx, hcy, hcz)),
                                ("grasp", (cx, cy, cz))):
        if pp.solve_ik_state(io_client, zx, zy, zz, *q,
                             block_yaw_deg=yaw_deg) is None:
            return False
    return True


def preflight(io_client, points, tip_z, check_ik=True, yaw_retry=True):
    """Screen every waypoint before anything moves. Returns
    [(label, tag_id, x, y, yaw_deg)] for the ones worth visiting.

    THE RADIUS MAP IS ADVISORY. It used to be the gate, and it was rejecting
    corners before the solver was ever asked about them -- on a flat reach
    constant that was 34 mm pessimistic at the grasp height. Now it prints what
    the geometry says and IK decides, which is the right order: the envelope is
    an upper bound that knows nothing about the wrist, self-collision, or
    convergence, while IK knows all three.
    """
    fz, _ = flange_target(0.0, 0.0, tip_z)
    print("\n=== Pre-flight: reach map (ADVISORY -- IK decides) ===")
    print(f"  tip z {tip_z:+.4f}  ->  flange z {fz:.4f} "
          f"(safety floor {pp.MAT_SURFACE_Z + pp.GRASP_OFFSET_Z + pp.MIN_TIP_CLEARANCE_M:.4f})")
    print(f"  {'point':<8} {'flange r':>9} | {'grasp z':>8} {'limit':>7} "
          f"{'margin':>8} | {'hover z':>8} {'limit':>7} {'margin':>8}")

    descent_risk = []
    for label, _tag_id, x, y in points:
        _fz, (cx, cy, cz) = flange_target(x, y, tip_z)
        _hx, _hy, hcz = hover_flange(x, y, tip_z)
        r = math.hypot(cx, cy)
        gmax, hmax = pp.max_flange_radius(cz), pp.max_flange_radius(hcz)
        print(f"  {label:<8} {r:>9.4f} | {cz:>8.4f} {gmax:>7.4f} "
              f"{1000 * (gmax - r):>+7.1f}mm | {hcz:>8.4f} {hmax:>7.4f} "
              f"{1000 * (hmax - r):>+7.1f}mm")
        if gmax < r <= gmax + pp.ORI_TOLERANCE_REACH_BONUS_M:
            descent_risk.append((label, r, gmax))

    if descent_risk:
        # The band where the two reach questions disagree, and the one case the
        # IK pass below CANNOT catch. solve_ik_state gets IK_ORI_XY_TOLERANCE's
        # 5.73 deg of tilt and will happily solve here; cartesian_move_to then
        # re-imposes the EXACT downward quaternion and has nothing to follow. So
        # the pre-flight passes, the hover succeeds, and the descent falls short
        # -- which looks like a planner failure and is really a reach one.
        print(f"\n  DESCENT RISK -- inside IK's tilt window, outside exact "
              f"vertical:")
        for label, r, gmax in descent_risk:
            print(f"    {label:<8} r {r:.4f} vs {gmax:.4f} exact-vertical "
                  f"({1000 * (r - gmax):+.1f}mm over). IK will solve this at "
                  f"the hover; the Cartesian descent will not.")

    if not check_ik:
        print("\n  --no-ik-check: nothing has been verified, only estimated. "
              "Every point is being passed through.")
        return [(label, tag_id, x, y, 0.0) for label, tag_id, x, y in points]

    print("\n=== Pre-flight: IK (downward grasp orientation) -- THE AUTHORITY ===")
    yaws = YAW_RETRIES_DEG if yaw_retry else (0.0,)
    confirmed = []
    for label, tag_id, x, y in points:
        for yaw in yaws:
            if _solves(io_client, x, y, tip_z, yaw):
                if yaw:
                    print(f"  {label:<8} hover + grasp solve at a {yaw:+.0f} deg "
                          f"grasp yaw (the default yaw has no solution here -- "
                          f"WRIST TRAVEL, not reach)")
                else:
                    print(f"  {label:<8} hover + grasp both solve")
                confirmed.append((label, tag_id, x, y, yaw))
                break
        else:
            print(f"  {label:<8} NO IK SOLUTION at any of "
                  f"{[int(v) for v in yaws]} deg -- skipping. An OMPL fallback "
                  f"here would park short and tilted and report success.")

    skipped = len(points) - len(confirmed)
    if skipped:
        print(f"\n  {skipped} of {len(points)} waypoints have no IK solution "
              f"and will be SKIPPED.")
    return confirmed


def sample(io_client, label, tag_id, repeat, x, y, tip_z, yaw_deg, interactive):
    """Read where the arm actually is, as one CSV row."""
    fz, (cx, cy, cz) = flange_target(x, y, tip_z, yaw_deg)
    try:
        current = io_client.current_joint_positions(JOINT_NAMES)
        values = [current[n] for n in JOINT_NAMES]
    except Exception as exc:
        print(f"[zone_cal] could not read /joint_states: {exc!r}")
        return None

    flange, _approach = pp._fk_flange(values)
    grip = pp._fk_gripper_base(values)
    err = (flange[0] - cx, flange[1] - cy, flange[2] - cz)

    row = {
        "label": label, "tag_id": "" if tag_id is None else tag_id,
        "repeat": repeat, "grasp_yaw_deg": round(yaw_deg, 1),
        "tip_x": round(x, 5), "tip_y": round(y, 5), "tip_z": round(tip_z, 5),
        "cmd_x": round(cx, 5), "cmd_y": round(cy, 5), "cmd_z": round(cz, 5),
        "fk_flange_x": round(flange[0], 5), "fk_flange_y": round(flange[1], 5),
        "fk_flange_z": round(flange[2], 5),
        "fk_grip_x": round(grip[0], 5), "fk_grip_y": round(grip[1], 5),
        "fk_grip_z": round(grip[2], 5),
        "err_x_mm": round(1000 * err[0], 2),
        "err_y_mm": round(1000 * err[1], 2),
        "err_z_mm": round(1000 * err[2], 2),
        "cmd_radius_mm": round(1000 * math.hypot(cx, cy), 2),
        "fk_radius_mm": round(1000 * math.hypot(flange[0], flange[1]), 2),
        "grip_radius_mm": round(1000 * math.hypot(grip[0], grip[1]), 2),
        "grip_above_mat_mm": round(1000 * (grip[2] - pp.MAT_SURFACE_Z), 2),
        "meas_height_mm": "", "meas_radius_in": "", "note": "",
    }
    for i, v in enumerate(values):
        row[f"j{i + 1}_rad"] = round(v, 6)

    print(f"[zone_cal] {label} rep{repeat}: "
          f"flange err dx {row['err_x_mm']:+.1f} dy {row['err_y_mm']:+.1f} "
          f"dz {row['err_z_mm']:+.1f} mm | "
          f"jaw radius {row['grip_radius_mm']:.1f} mm "
          f"({row['grip_radius_mm'] / 25.4:.2f} in) | "
          f"jaw {row['grip_above_mat_mm']:.1f} mm above the mat")

    if interactive:
        # The ONLY columns that can catch a wrong geometry constant. FK cannot
        # -- it reports the model's opinion, and the model is under test.
        print("[zone_cal] Measure the jaws now. Blank to skip any of these.")
        row["meas_height_mm"] = _ask("  fingertip height above the mat, mm: ")
        row["meas_radius_in"] = _ask("  jaw centre distance from the base, in: ")
        row["note"] = input("  note: ").strip()
    return row


def _ask(prompt):
    raw = input(prompt).strip()
    if not raw:
        return ""
    try:
        return float(raw)
    except ValueError:
        print("    not a number, recorded blank")
        return ""


def visit(io_client, label, tag_id, repeat, x, y, tip_z, yaw_deg, interactive,
          dwell):
    """Hover -> Cartesian descent -> settle -> measure -> retreat."""
    fz, (cx, cy, _cz) = flange_target(x, y, tip_z, yaw_deg)
    hz = pp.hover_z(fz, radius=math.hypot(cx, cy))
    note = f" (grasp yaw {yaw_deg:+.0f} deg)" if yaw_deg else ""

    print(f"\n=== {label} rep{repeat}: pre-grasp hover{note} ===")
    # unidirectional=True for the same reason pick_place's pre-grasp move uses
    # it: this is where J1 takes its big swing, and the survey is worthless if
    # every point carries a different slice of J1's backlash.
    if pp.move_arm_to(io_client, x, y, hz, block_yaw_deg=yaw_deg,
                      unidirectional=True) is False:
        print(f"[zone_cal] hover FAILED at {label}, skipping this point")
        return None

    print(f"=== {label} rep{repeat}: descend ===")
    if pp.cartesian_move_to(io_client, x, y, fz,
                            block_yaw_deg=yaw_deg) is False:
        print(f"[zone_cal] descent FAILED at {label}, retreating")
        pp.cartesian_move_to(io_client, x, y, hz, allow_fallback=True,
                             block_yaw_deg=yaw_deg)
        return None

    pp.settle_pause(io_client, f"the {label} sample")
    pp.report_reached(io_client, x, y, fz, what=f"{label} rep{repeat}")
    row = sample(io_client, label, tag_id, repeat, x, y, tip_z, yaw_deg,
                 interactive)

    if dwell > 0 and not interactive:
        print(f"[zone_cal] holding {dwell:.1f}s -- look at the jaws")
        time.sleep(dwell)

    print(f"=== {label} rep{repeat}: retreat ===")
    pp.cartesian_move_to(io_client, x, y, hz, allow_fallback=True,
                         block_yaw_deg=yaw_deg)
    return row


def summarise(rows):
    """What the survey says about the constants, rather than about one pose.

    A CONSTANT error across the zone is a calibration offset and belongs in
    DESCENT_BIAS_Z / JAW_RADIAL_OFFSET_M. An error that SWINGS with position is
    pose-dependent -- droop that varies with reach -- and no single constant
    will fix it; that is the part a feedforward lookup table has to carry.
    """
    if not rows:
        print("\n[zone_cal] no samples collected, nothing to summarise.")
        return

    print("\n=== Summary ===")
    print(f"  {'point':<8} {'dx mm':>8} {'dy mm':>8} {'dz mm':>8} "
          f"{'radial mm':>10} {'jaw h mm':>9}")
    for r in rows:
        radial = r["fk_radius_mm"] - r["cmd_radius_mm"]
        print(f"  {r['label']:<8} {r['err_x_mm']:>+8.2f} {r['err_y_mm']:>+8.2f} "
              f"{r['err_z_mm']:>+8.2f} {radial:>+10.2f} "
              f"{r['grip_above_mat_mm']:>9.1f}")

    # Below this, neither a constant nor a lookup table is worth fitting: it is
    # under the servo dead zone and under what a tape measure can resolve, so
    # any structure found in it is noise being dressed up as a model.
    noise_floor_mm = 1.0

    def verdict(name, vals, knob, unit="mm", scale=1000.0):
        mean = sum(vals) / len(vals)
        spread = max(vals) - min(vals)
        print(f"\n  {name}: mean {mean:+.2f} {unit}, spread {spread:.2f} "
              f"{unit} across {len(vals)} samples")
        if unit == "mm" and abs(mean) < noise_floor_mm and spread < noise_floor_mm:
            print(f"    -> negligible, both under {noise_floor_mm:.0f} mm. "
                  f"Leave {knob} alone.")
        elif abs(mean) > spread:
            print(f"    -> mostly a CONSTANT offset. Move {knob} by "
                  f"{-mean / scale:+.4f}{' deg' if unit == 'deg' else ' m'}.")
        else:
            print(f"    -> mostly POSE-DEPENDENT ({spread:.2f} {unit} of swing "
                  f"around a {mean:+.2f} {unit} mean). No single value of "
                  f"{knob} fixes this; it is what the lookup table is for.")
        return mean

    # ------------------------------------------------------------------
    # FK residuals: the ARM against its own commanded joints.
    # ------------------------------------------------------------------
    verdict("height (dz)", [r["err_z_mm"] for r in rows], "DESCENT_BIAS_Z")
    verdict("lateral (dy)", [r["err_y_mm"] for r in rows],
            "JAW_RADIAL_OFFSET_M")

    # dx IS NOT A TOOL OFFSET, and calling it one was a bug. At every waypoint
    # here the bearing is near +Y, so world X is TANGENTIAL -- and a tangential
    # miss with the radius correct is J1 not reaching its commanded angle. It is
    # a joint tracking error, and it fits far better as an ANGLE than as a
    # distance (measured 2026-08-03: 0.26 deg of spread against 1.66 mm, i.e.
    # a third tighter once the radius is divided out). Compensating it in
    # Cartesian X would only be right at one radius.
    angles = [math.degrees(r["err_x_mm"] / r["fk_radius_mm"]) for r in rows]
    verdict("tangential (dx), as a J1 angle", angles,
            "the J1 approach bias", unit="deg", scale=1.0)
    verdict("tangential (dx), as distance", [r["err_x_mm"] for r in rows],
            "nothing -- the angle above is the better-conditioned fit")

    # ------------------------------------------------------------------
    # Hand measurements: the only check on the TOOL MODEL. FK cannot make it.
    # ------------------------------------------------------------------
    heights = [r for r in rows if r["meas_height_mm"] != ""]
    if heights:
        # AGAINST THE PREDICTED FINGERTIP, not against fk_grip_*. The first
        # version of this compared a fingertip measurement to gripper_base --
        # two different points about 85 mm apart -- and reported the difference
        # between them as an error. It is not; it is the length of the gripper.
        print("\n  hand-measured fingertip height vs PREDICTED "
              "(flange z - GRASP_OFFSET_Z):")
        deltas = []
        for r in heights:
            pred = 1000.0 * ((r["fk_flange_z"] - pp.GRASP_OFFSET_Z)
                             - pp.MAT_SURFACE_Z)
            d = float(r["meas_height_mm"]) - pred
            deltas.append(d)
            print(f"    {r['label']:<8} measured {float(r['meas_height_mm']):5.1f} "
                  f"vs predicted {pred:5.1f} mm  ({d:+.1f})")
        mean = verdict("    fingertip height", deltas, "GRASP_OFFSET_Z")
        print(f"    -> implied GRASP_OFFSET_Z = "
              f"{pp.GRASP_OFFSET_Z - mean / 1000.0:.4f} "
              f"(currently {pp.GRASP_OFFSET_Z:.4f}). POSITIVE delta means the "
              f"tips sit HIGHER than the model says, so the offset is too big "
              f"and every grasp lands high.")

    radii = [r for r in rows if r["meas_radius_in"] != ""]
    if radii:
        print("\n  hand-measured jaw radius vs the TARGET the jaws were aimed at:")
        deltas = []
        for r in radii:
            tgt = 1000.0 * math.hypot(r["tip_x"], r["tip_y"])
            d = float(r["meas_radius_in"]) * 25.4 - tgt
            deltas.append(d)
            print(f"    {r['label']:<8} measured {float(r['meas_radius_in']) * 25.4:5.1f} "
                  f"vs target {tgt:5.1f} mm  ({d:+.1f})")
        mean = verdict("    jaw radius", deltas, "JAW_RADIAL_OFFSET_M")
        print(f"    -> implied JAW_RADIAL_OFFSET_M = "
              f"{pp.JAW_RADIAL_OFFSET_M + mean / 1000.0:.4f} "
              f"(currently {pp.JAW_RADIAL_OFFSET_M:.4f}).")
        print(f"    NOTE: a single zone cannot separate a world-frame offset "
              f"from a radial one -- every waypoint in one zone shares a "
              f"bearing to within a few degrees. Surveying BOTH zones settled "
              f"it on 2026-08-04: radial.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Survey a zone's centre and tag vertices with the jaws.")
    p.add_argument("--zone", choices=("pickup", "place"), default="pickup",
                   help="which zone to survey (default: pickup)")
    p.add_argument("--zone-radius", type=float, default=pp.ZONE_RADIUS_M,
                   help="distance from the base to the ZONE CENTRE, metres. "
                        "Override to ask the reach question about a zone that "
                        f"has not been taped down yet (default: {pp.ZONE_RADIUS_M:.4f})")
    p.add_argument("--zone-size", type=float, default=zv.DEFAULT_ZONE_SIZE,
                   help="side of the TAG square, metres. Changing this for real "
                        "means re-taping the mat -- see zone_vision.py "
                        f"(default: {zv.DEFAULT_ZONE_SIZE:.4f})")
    p.add_argument("--square", choices=("tags", "span", "blocks"),
                   default="blocks",
                   help="which of the zone's three nested squares to survey. "
                        "'blocks' is the range a block's CENTRE may occupy and "
                        "is the one that has to be reachable for a detected "
                        "block to be pickable; 'span' is the clear area between "
                        "the tags' inner edges; 'tags' joins the tag centres, "
                        "which the arm never actually has to reach "
                        "(default: blocks)")
    p.add_argument("--block-size", type=float, default=pp.BLOCK_HEIGHT_M,
                   help="block side, metres. Sets how far in from the clear "
                        "span a block's centre must stay "
                        f"(default: {pp.BLOCK_HEIGHT_M:.4f})")
    p.add_argument("--no-yaw-retry", action="store_true",
                   help="do not retry a failed point at +/-90 deg of grasp yaw. "
                        "The retry is free for a square block and separates "
                        "'out of reach' from 'the wrist ran out of travel at "
                        "this bearing', which look identical otherwise")
    p.add_argument("--tip-z", type=float,
                   default=pp.MAT_SURFACE_Z + pp.BLOCK_HEIGHT_M / 2.0,
                   help="height the JAW TIPS should reach, metres, in base "
                        "coordinates. The default is a block's centre resting "
                        f"on the mat (default: {pp.MAT_SURFACE_Z + pp.BLOCK_HEIGHT_M / 2.0:+.4f})")
    p.add_argument("--points", choices=("all", "centre", "vertices"),
                   default="all", help="which waypoints to visit (default: all)")
    p.add_argument("--repeats", type=int, default=1,
                   help="visits per waypoint. >1 measures repeatability, which "
                        "is what says whether a residual is real or noise "
                        "(default: 1)")
    p.add_argument("--dwell", type=float, default=0.0,
                   help="seconds to hold at each sample so the jaws can be "
                        "looked at (default: 0, ignored with --interactive)")
    p.add_argument("--interactive", action="store_true",
                   help="pause at each sample and prompt for hand measurements. "
                        "These are the only columns that can catch a wrong "
                        "geometry constant -- FK is blind to those by "
                        "construction")
    p.add_argument("--dry-run", action="store_true",
                   help="pre-flight only: print the reach map and exit without "
                        "moving the arm. Needs the robot up for the IK service, "
                        "but commands no motion")
    p.add_argument("--no-ik-check", action="store_true",
                   help="skip the IK half of the pre-flight (radius screen "
                        "only). Faster, and the fallback if the IK service is "
                        "unavailable")
    p.add_argument("--out", default="zone_calibration.csv",
                   help="CSV to write (default: zone_calibration.csv)")
    return p.parse_args()


def main():
    args = parse_args()

    sides = dict(square_sides(args.zone_size, zv.DEFAULT_TAG_SIZE,
                              args.block_size))
    side = sides[args.square]

    print(f"[zone_cal] {args.zone} zone: centre radius "
          f"{args.zone_radius:.4f} m ({args.zone_radius / 0.0254:.2f} in), "
          f"tag square {args.zone_size / 0.0254:.2f} in")
    # Both heights, because the envelope shrinks with height and it is the
    # HOVER that binds, not the grasp -- the thing that made the far corners
    # look unreachable when the grasp itself is comfortably inside.
    grasp_z = args.tip_z + pp.GRASP_OFFSET_Z
    hover_z_est = pp.hover_z(grasp_z)
    print(f"[zone_cal] the zone's three nested squares, and how far out each "
          f"can sit with all four corners inside the reach envelope:")
    print(f"    {'square':<7} {'side':>6}   {'at grasp z':>18}   "
          f"{'at hover z':>18}")
    for kind, s in square_sides(args.zone_size, zv.DEFAULT_TAG_SIZE,
                                args.block_size):
        g = max_zone_centre_radius(s, args.zone, grasp_z)
        h = max_zone_centre_radius(s, args.zone, hover_z_est)
        mark = "  <-- surveying this one" if kind == args.square else ""
        print(f"    {kind:<7} {s / 0.0254:>5.2f}in   {g:.4f}m "
              f"({g / 0.0254:>5.2f}in)   {h:.4f}m ({h / 0.0254:>5.2f}in){mark}")

    g = max_zone_centre_radius(side, args.zone, grasp_z)
    h = max_zone_centre_radius(side, args.zone, hover_z_est)
    if args.zone_radius <= h:
        print(f"[zone_cal] This zone's '{args.square}' corners clear the "
              f"envelope at both heights ({1000 * (h - args.zone_radius):.0f} mm "
              f"of margin at the binding one).")
    elif args.zone_radius <= g:
        # The interesting middle, and the case that matters at 9 in: the GRASP
        # is fine and the hover above it is not. hover_z clamps for this, so the
        # descent just starts lower. Not a reason to move anything.
        print(f"[zone_cal] This zone's '{args.square}' corners clear at the "
              f"GRASP height ({1000 * (g - args.zone_radius):.0f} mm of margin) "
              f"but not at a full {1000 * pp.APPROACH_HEIGHT:.0f} mm hover above "
              f"it. hover_z clamps to the envelope, so the descent starts lower "
              f"there -- nothing needs to move.")
    else:
        print(f"[zone_cal] This zone's '{args.square}' corners are "
              f"{1000 * (args.zone_radius - g):.0f} mm outside the envelope even "
              f"at the grasp height. Advisory only -- IK below is what decides.")

    points = zone_waypoints(
        args.zone, args.zone_radius, side, kind=args.square,
        include_centre=args.points in ("all", "centre"),
        include_vertices=args.points in ("all", "vertices"))
    if not points:
        print("[zone_cal] no waypoints selected.")
        return

    rclpy.init()
    io_client = pp.RobotIOClient()

    if not io_client.wait_for_joint_states(timeout_sec=30.0):
        print("\nABORTING: no /joint_states in 30s. Nothing here can measure "
              "anything without it -- see pick_place.py's identical check for "
              "what to look at.")
        io_client.destroy_node()
        rclpy.shutdown()
        return

    reachable = preflight(io_client, points, args.tip_z,
                          check_ik=not args.no_ik_check,
                          yaw_retry=not args.no_yaw_retry)

    if args.dry_run:
        print("\n[zone_cal] --dry-run: arm not moved.")
        io_client.destroy_node()
        rclpy.shutdown()
        return
    if not reachable:
        print("\n[zone_cal] nothing reachable to visit.")
        io_client.destroy_node()
        rclpy.shutdown()
        return

    print("\n=== Return to home pose ===")
    pp.go_home(io_client)
    print("\n=== Toggle gripper (pre-start) ===")
    pp.toggle_gripper(io_client)

    rows = []
    for repeat in range(1, args.repeats + 1):
        for label, tag_id, x, y, yaw in reachable:
            row = visit(io_client, label, tag_id, repeat, x, y, args.tip_z,
                        yaw, args.interactive, args.dwell)
            if row is not None:
                rows.append(row)

    print("\n=== Return to home pose (final) ===")
    pp.go_home(io_client)

    if rows:
        with open(args.out, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n[zone_cal] wrote {len(rows)} samples to {args.out}")

    summarise(rows)

    io_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
