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
  a wrong JAW_LATERAL_OFFSET, a gripper that hangs 3 mm off where the model
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

# MAX_FLANGE_RADIUS_M lives in tag_pick_place.py, which is where the runaway it
# guards against was diagnosed. Imported rather than copied so there is one
# number, but imported DEFENSIVELY: tag_pick_place pulls in swarm_interfaces and
# the vision stack, and this script needs neither. A calibration run should not
# be blocked on the vision package being built.
try:
    from tag_pick_place import MAX_FLANGE_RADIUS_M  # noqa: E402
except Exception as exc:  # pragma: no cover - depends on the workspace build
    MAX_FLANGE_RADIUS_M = 0.245
    print(f"[zone_cal] could not import tag_pick_place ({exc!r}); using the "
          f"built-in MAX_FLANGE_RADIUS_M = {MAX_FLANGE_RADIUS_M}. Check it "
          f"still matches tag_pick_place.py.")

JOINT_NAMES = list(pp.HOME_RADIANS.keys())

CSV_FIELDS = [
    "label", "tag_id", "repeat",
    "tip_x", "tip_y", "tip_z",
    "cmd_x", "cmd_y", "cmd_z",
    "fk_flange_x", "fk_flange_y", "fk_flange_z",
    "fk_grip_x", "fk_grip_y", "fk_grip_z",
    "err_x_mm", "err_y_mm", "err_z_mm",
    "cmd_radius_mm", "fk_radius_mm", "grip_radius_mm", "grip_above_mat_mm",
] + [f"j{i + 1}_rad" for i in range(6)] + [
    "meas_height_mm", "meas_radius_in", "note",
]


def zone_waypoints(zone_name, centre_radius, zone_size, include_centre=True,
                   include_vertices=True):
    """[(label, tag_id, x, y)] for a zone's centre and its four tag vertices,
    in robot base coordinates.

    The vertices ARE the tag centres: ZoneSpec.tag_corner_targets() puts tag i
    at (sx*zone_size/2, sy*zone_size/2) for ZONE_CORNER_SIGNS[i], so walking
    the same list in the same order pairs each measurement with the tag id
    printed on the mat. That pairing is the reason to do it here rather than
    hardcode four offsets -- if the mat is ever re-taped, zone_vision is the
    single place the layout changes.

    Sign convention: the pickup zone sits at +Y, the place zone at -Y. The
    VERTEX offsets are not flipped with it -- ZoneSpec.zone_to_world at
    world_yaw = 0 makes zone-local axes world axes, so zone +Y is world +Y for
    both zones.
    """
    sign = -1.0 if zone_name == "place" else +1.0
    tag_ids = (zv.PLACE_TAG_IDS if zone_name == "place" else zv.PICKUP_TAG_IDS)
    cy = sign * centre_radius
    half = zone_size / 2.0

    points = []
    if include_centre:
        points.append(("centre", None, 0.0, cy))
    if include_vertices:
        for tag_id, (sx, sy) in zip(tag_ids, zv.ZONE_CORNER_SIGNS):
            points.append((f"tag{tag_id}", tag_id, sx * half, cy + sy * half))
    return points


def max_zone_centre_radius(zone_size, zone_name="pickup"):
    """Largest zone-centre radius at which all four vertices are still inside
    MAX_FLANGE_RADIUS_M, given the lateral tool offset.

    Derived, not tuned: the binding vertex is a far corner, whose flange target
    sits at (zone_size/2, Yc + zone_size/2 + push) where `push` is how far
    compensate_for_tip_swing shoves the flange outward to put the JAWS on the
    target. Solve hypot(x, y) = MAX_FLANGE_RADIUS_M for Yc.

    `push` is a WORLD vector, not a radial one (see JAW_LATERAL_OFFSET), so it
    points away from the base at +Y and toward it at -Y. That makes the pickup
    zone the harder of the two by 2*push, which is why the sign is carried
    rather than dropped.
    """
    half = zone_size / 2.0
    push_x, push_y, _ = pp.compensate_for_tip_swing(0.0, 0.0, 0.0)
    if zone_name == "place":
        push_y = -push_y
    # push_x is small but signed, so it pushes one of the two far corners
    # further out than the other -- take the worse one, or this reports a
    # limit at which one corner is still a fraction of a millimetre over.
    half_x = half + abs(push_x)
    inner = MAX_FLANGE_RADIUS_M ** 2 - half_x ** 2
    if inner <= 0.0:
        return 0.0
    return math.sqrt(inner) - half - push_y


def flange_target(x, y, tip_z):
    """The flange z for a tip at height tip_z, and the fully compensated,
    clamped (x, y, z) that will actually be commanded. Mirrors what
    move_arm_to/make_grasp_pose do internally, so the pre-flight screens the
    real target rather than the caller's."""
    fz = tip_z + pp.GRASP_OFFSET_Z
    cx, cy, cz = pp.compensate_for_tip_swing(x, y, fz)
    return fz, (cx, cy, pp.clamp_flange_z(cz, "zone_calibrate"))


def preflight(io_client, points, tip_z, check_ik=True):
    """Screen every waypoint before anything moves. Returns the reachable
    subset, and prints the reach map either way."""
    fz, _ = flange_target(0.0, 0.0, tip_z)
    print("\n=== Pre-flight: reach map ===")
    print(f"  tip z {tip_z:+.4f}  ->  flange z {fz:.4f} "
          f"(safety floor {pp.MAT_SURFACE_Z + pp.GRASP_OFFSET_Z + pp.MIN_TIP_CLEARANCE_M:.4f})")
    print(f"  {'point':<8} {'tip x':>8} {'tip y':>8} {'tip r':>8} "
          f"{'flange r':>9} {'margin':>8}   verdict")

    ok = []
    for label, tag_id, x, y in points:
        _fz, (cx, cy, cz) = flange_target(x, y, tip_z)
        r_tip = math.hypot(x, y)
        r_cmd = math.hypot(cx, cy)
        margin = MAX_FLANGE_RADIUS_M - r_cmd
        verdict = "reachable" if margin >= 0.0 else "OUT OF REACH"
        print(f"  {label:<8} {x:>+8.4f} {y:>+8.4f} {r_tip:>8.4f} "
              f"{r_cmd:>9.4f} {1000 * margin:>+7.1f}mm   {verdict}")
        if margin >= 0.0:
            ok.append((label, tag_id, x, y))

    # The radius screen is geometry only. IK is the authority on whether the
    # arm can hold the DOWNWARD orientation there, which is a strictly harder
    # question and the one that actually fails first near the workspace edge.
    if check_ik and ok:
        print("\n=== Pre-flight: IK (downward grasp orientation) ===")
        confirmed = []
        for label, tag_id, x, y in ok:
            fz, (cx, cy, cz) = flange_target(x, y, tip_z)
            hcx, hcy, hcz = pp.compensate_for_tip_swing(x, y, pp.hover_z(fz))
            q = pp.grasp_quat_for(0.0, x, y, False)
            good = True
            for what, (zx, zy, zz) in (("hover", (hcx, hcy, hcz)),
                                       ("grasp", (cx, cy, cz))):
                state = pp.solve_ik_state(io_client, zx, zy, zz, *q)
                if state is None:
                    print(f"  {label:<8} {what:<6} NO IK SOLUTION -- skipping "
                          f"this point (an OMPL fallback here would park short "
                          f"and tilted and report success)")
                    good = False
                    break
            if good:
                print(f"  {label:<8} hover + grasp both solve")
                confirmed.append((label, tag_id, x, y))
        ok = confirmed

    skipped = len(points) - len(ok)
    if skipped:
        print(f"\n  {skipped} of {len(points)} waypoints unreachable and will "
              f"be SKIPPED.")
    return ok


def sample(io_client, label, tag_id, repeat, x, y, tip_z, interactive):
    """Read where the arm actually is, as one CSV row."""
    fz, (cx, cy, cz) = flange_target(x, y, tip_z)
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
        "repeat": repeat,
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


def visit(io_client, label, tag_id, repeat, x, y, tip_z, interactive, dwell):
    """Hover -> Cartesian descent -> settle -> measure -> retreat."""
    fz, _cmd = flange_target(x, y, tip_z)
    hz = pp.hover_z(fz)

    print(f"\n=== {label} rep{repeat}: pre-grasp hover ===")
    # unidirectional=True for the same reason pick_place's pre-grasp move uses
    # it: this is where J1 takes its big swing, and the survey is worthless if
    # every point carries a different slice of J1's backlash.
    if pp.move_arm_to(io_client, x, y, hz, unidirectional=True) is False:
        print(f"[zone_cal] hover FAILED at {label}, skipping this point")
        return None

    print(f"=== {label} rep{repeat}: descend ===")
    if pp.cartesian_move_to(io_client, x, y, fz) is False:
        print(f"[zone_cal] descent FAILED at {label}, retreating")
        pp.cartesian_move_to(io_client, x, y, hz, allow_fallback=True)
        return None

    pp.settle_pause(io_client, f"the {label} sample")
    pp.report_reached(io_client, x, y, fz, what=f"{label} rep{repeat}")
    row = sample(io_client, label, tag_id, repeat, x, y, tip_z, interactive)

    if dwell > 0 and not interactive:
        print(f"[zone_cal] holding {dwell:.1f}s -- look at the jaws")
        time.sleep(dwell)

    print(f"=== {label} rep{repeat}: retreat ===")
    pp.cartesian_move_to(io_client, x, y, hz, allow_fallback=True)
    return row


def summarise(rows):
    """What the survey says about the constants, rather than about one pose.

    A CONSTANT error across the zone is a calibration offset and belongs in
    DESCENT_BIAS_Z / JAW_LATERAL_OFFSET. An error that SWINGS with position is
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

    for field, name, knob in (
            ("err_z_mm", "height (dz)", "DESCENT_BIAS_Z"),
            ("err_x_mm", "lateral (dx)", "JAW_LATERAL_OFFSET[0]"),
            ("err_y_mm", "lateral (dy)", "JAW_LATERAL_OFFSET[1]")):
        vals = [r[field] for r in rows]
        mean = sum(vals) / len(vals)
        spread = max(vals) - min(vals)
        print(f"\n  {name}: mean {mean:+.2f} mm, spread {spread:.2f} mm "
              f"across {len(vals)} samples")
        if abs(mean) < noise_floor_mm and spread < noise_floor_mm:
            print(f"    -> negligible, both under {noise_floor_mm:.0f} mm. "
                  f"Leave {knob} alone.")
        elif abs(mean) > spread:
            print(f"    -> mostly a CONSTANT offset. Move {knob} by "
                  f"{-mean / 1000.0:+.4f} m.")
        else:
            print(f"    -> mostly POSE-DEPENDENT ({spread:.2f} mm of swing "
                  f"around a {mean:+.2f} mm mean). No single value of {knob} "
                  f"fixes this; it is what the lookup table is for.")

    measured = [r for r in rows if r["meas_height_mm"] != ""]
    if measured:
        print("\n  hand-measured vs FK jaw height (the geometry check FK "
              "cannot make):")
        for r in measured:
            d = float(r["meas_height_mm"]) - r["grip_above_mat_mm"]
            print(f"    {r['label']:<8} measured {float(r['meas_height_mm']):.1f} "
                  f"vs FK {r['grip_above_mat_mm']:.1f} mm  ({d:+.1f})")
        deltas = [float(r["meas_height_mm"]) - r["grip_above_mat_mm"]
                  for r in measured]
        mean = sum(deltas) / len(deltas)
        print(f"    mean {mean:+.1f} mm -- if this is consistent, it is "
              f"GRASP_OFFSET_Z that is off by it, not the arm.")


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
                   help="side of the tag square, metres. Changing this for real "
                        "means re-taping the mat -- see zone_vision.py "
                        f"(default: {zv.DEFAULT_ZONE_SIZE:.4f})")
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

    points = zone_waypoints(
        args.zone, args.zone_radius, args.zone_size,
        include_centre=args.points in ("all", "centre"),
        include_vertices=args.points in ("all", "vertices"))
    if not points:
        print("[zone_cal] no waypoints selected.")
        return

    print(f"[zone_cal] {args.zone} zone: centre radius "
          f"{args.zone_radius:.4f} m ({args.zone_radius / 0.0254:.2f} in), "
          f"tag square {args.zone_size:.4f} m "
          f"({args.zone_size / 0.0254:.2f} in)")
    limit = max_zone_centre_radius(args.zone_size, args.zone)
    print(f"[zone_cal] all four vertices of a {args.zone_size / 0.0254:.0f} in "
          f"square are reachable only out to a centre radius of "
          f"{limit:.4f} m ({limit / 0.0254:.2f} in).")
    if args.zone_radius > limit:
        print(f"[zone_cal] THIS ZONE IS BEYOND THAT by "
              f"{1000 * (args.zone_radius - limit):.0f} mm. Its far vertices "
              f"cannot be grasped at any orientation, so a block detected "
              f"there cannot be picked up -- that is a zone-placement problem, "
              f"not a calibration one.")

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
                          check_ik=not args.no_ik_check)

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
        for label, tag_id, x, y in reachable:
            row = visit(io_client, label, tag_id, repeat, x, y, args.tip_z,
                        args.interactive, args.dwell)
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
