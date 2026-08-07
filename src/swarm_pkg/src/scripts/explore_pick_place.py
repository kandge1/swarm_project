#!/usr/bin/env python3
"""Find both zones, pick the block out of one, put it in the other. One run.

    python3 explore_pick_place.py

That is the whole demo. It sweeps J1 looking for BOTH tag squares at once,
solves each one's centre and yaw, then hands the pickup zone to
tag_pick_place.run_stage1 and the place zone's centre to its release step.

WHY ONE PROCESS AND NOT TWO SCRIPTS PIPED TOGETHER. explore.py already prints a
tag_pick_place.py command line for a human to paste, and that worked -- but it
re-homes the arm, re-inits ROS and re-discovers the detector service between the
survey and the pick, and every one of those is a place the run can die halfway.
It also throws away the place zone, which the same sweep already saw. Keeping it
in one process means the survey's answer goes straight into the grasp with
nothing retyped, which is also the only version of this that can be demonstrated
without narration.

WHAT IT DOES NOT DO. The place is deliberately lax: the block is released at the
place zone's SURVEYED CENTRE, with no second detection and no attempt to put it
anywhere particular inside the square. That is the stated requirement, and it is
also the honest one -- placing precisely would need the same four-view refinement
the pick gets, from a pose holding a block that occludes the very tags it would
be measuring.

Everything about the pick itself -- the four-view survey, the tag identity, the
parallax correction, the jaw offset, the confirm prompts -- is tag_pick_place's,
unchanged and uncopied. This file is a driver, not a fork.
"""

import argparse
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import calibration  # noqa: E402
import explore  # noqa: E402
import pick_place as pp  # noqa: E402
import tag_pick_place as tpp  # noqa: E402


INCH = 0.0254

# Named bench positions for the pickup zone's CENTRE, in inches, so a
# calibration sweep is `--position A` rather than four decimals retyped from a
# notebook. Whole inches on purpose: they are what a person can lay out
# repeatably against a ruler, and a position you cannot reproduce is a position
# whose error you cannot attribute.
#
# WHAT THIS SET IS FOR. ORIGIN_RADIAL_BIAS_M was measured at n=2 -- bearings
# -0.5 and +90 deg, radii 203 and 229 mm -- and a constant fitted to two points
# is a constant that has never been contradicted. These eleven span 160-229 mm
# of radius and -113 to +18 deg of bearing, which is the axis that decides
# whether one number is enough.
#
# THE GAP, stated because it will matter when the numbers come back: bearings
# +20 to +90 deg are not covered. H is the only position on the +Y side of the
# workspace, and the +90 deg calibration point came from the PLACE zone, which
# this sweep does not move. Two more at roughly (0, +7) and (-3, +7) would close
# it; without them a bearing-dependent term cannot be ruled out on that side.
SURVEY_POSITIONS = {
    "A": (-3.0, -7.0), "B": (0.0, -7.0), "C": (3.0, -7.0),
    "D": (5.0, -5.0), "E": (6.0, -3.0), "F": (7.0, -2.0),
    "G": (8.0, -1.0), "H": (9.0, 0.0), "I": (8.0, 1.0),
    "J": (7.0, 2.0), "K": (6.0, 2.0),
}


def position_xy(name):
    """-> (x_m, y_m) for a SURVEY_POSITIONS label, or None."""
    inches = SURVEY_POSITIONS.get(name.upper())
    return None if inches is None else (inches[0] * INCH, inches[1] * INCH)


def describe_positions():
    lines = []
    for name in sorted(SURVEY_POSITIONS):
        ix, iy = SURVEY_POSITIONS[name]
        x, y = position_xy(name)
        lines.append("  %s  (%+.0f, %+.0f) in = (%+.4f, %+.4f) m   "
                     "r %.2f in   bearing %+.1f deg"
                     % (name, ix, iy, x, y, math.hypot(ix, iy),
                        math.degrees(math.atan2(iy, ix))))
    return "\n".join(lines)


def coarse_then_fine_both(io_client, detector, args, zones=("pickup", "place")):
    """One coarse sweep for both zones, then a fine arc over each one it found.

    -> ({zone: [Sighting, ...]}, step_that_produced_them)

    Same reasoning as explore.coarse_then_fine, and the same arithmetic reason
    it exists: a fine pass over the whole 270 deg is ~110 stops, and this run
    detects TWO zones at every stop. The coarse pass only has to answer "roughly
    which way is each square", which does not need a good origin -- tag count is
    the honest measure of that, so it is what the anchor is chosen on.

    The fine arcs ask for ONE zone each, not both. A zone's fine arc is centred
    on where that zone was seen, so the other one is behind the camera there and
    a second detect call would be a service round trip spent on an empty frame.
    """
    patience = getattr(args, "coarse_patience", 0)
    print("[explore] coarse sweep J1 %+.0f..%+.0f step %.0f, watching for %s%s\n"
          % (args.start, args.end, args.step, " and ".join(zones),
             ", stopping %d stop(s) after the last one is passed" % patience
             if patience else ""))
    coarse = sweep_both(io_client, detector, args, args.start, args.end,
                        args.step, zones, patience)

    out = {}
    for zone in zones:
        if not coarse[zone]:
            out[zone] = []
            continue
        anchor = sorted(coarse[zone],
                        key=lambda s: (-len(s.tag_ids), s.offset_mm))[0]
        print("\n[explore] %s coarse best: J1 %+.1f with %d tag(s) -- refining "
              "+-%.1f deg around it\n"
              % (zone, anchor.j1_deg, len(anchor.tag_ids), args.fine_span))
        fine = sweep_both(io_client, detector, args,
                          anchor.j1_deg - args.fine_span,
                          anchor.j1_deg + args.fine_span,
                          args.fine_step, (zone,))[zone]
        # The fine pass REPLACES the coarse one rather than adding to it, for
        # the reason explore.py gives: mixing them lets a coarse sighting --
        # taken exactly where the framing is worst -- win on tag count by luck.
        out[zone] = fine or coarse[zone]
    return out


def sweep_both(io_client, detector, args, start, end, step, zones,
               stop_after_misses=0):
    """explore.sweep_both over an explicit arc, leaving args untouched."""
    saved = (args.start, args.end, args.step)
    args.start, args.end, args.step = start, end, step
    try:
        return explore.sweep_both(io_client, detector, args, zones,
                                  stop_after_misses)
    finally:
        args.start, args.end, args.step = saved


def record_survey(name, fit, taped, note):
    """Log a surveyed zone centre against its taped one. Never raises.

    THIS IS THE ROW survey_error HAS BEEN WAITING FOR. It has read 0.0 +- 0.0
    for the life of the project because every run passed the same number as both
    --zone-origin and --truth-block-world -- asserting the survey was right
    rather than checking it. The 28 mm bias found on 2026-08-06 was invisible
    for exactly that reason.

    truth_zone is [0, 0] because the thing being compared IS the zone centre,
    which sits at zone-local (0, 0) by definition. That is also what makes
    calibration.survey_error willing to compute on the row.

    No block is involved, so no grasp is needed: a --survey-only pass measures
    this in about four minutes with the gripper never leaving home.
    """
    if fit is None or taped is None:
        return
    error = math.hypot(fit.origin[0] - taped[0], fit.origin[1] - taped[1])
    print("[survey] %s vs taped (%.4f, %.4f): off by %.1f mm"
          % (name, taped[0], taped[1], error * 1000.0))
    calibration.record(
        None,
        zone_origin=[fit.origin[0], fit.origin[1]],
        zone_yaw_deg=math.degrees(fit.yaw),
        truth_world=[taped[0], taped[1]],
        truth_zone=[0.0, 0.0],
        views=len(fit.sightings),
        view_spread_m=float(fit.residual),
        grasped=False,
        note="%s survey %s" % (note, name))


def survey_zone(name, sightings, zone_size, step, yaw_fixed):
    """-> explore.ZoneFit for one zone, or None with the reason printed."""
    if not sightings:
        print("[survey] %s zone: never seen. Its tags are %s."
              % (name, "0-3" if name == "pickup" else "4-7"))
        return None

    fits, rejected, rejected_fits = explore.choose(sightings, zone_size, step,
                                                   yaw_fixed)
    for sighting, why in rejected:
        print("[survey] %s J1 %+7.1f REJECTED: %s" % (name, sighting.j1_deg, why))
    for run, why in rejected_fits:
        print("[survey] %s run J1 %+.1f..%+.1f (%d views) REJECTED: %s"
              % (name, run[0].j1_deg, run[-1].j1_deg, len(run), why))
    if not fits:
        print("[survey] %s zone: %d sighting(s), none of them usable. Nothing "
              "is handed off -- a guessed origin sends the arm at a physical "
              "target." % (name, len(sightings)))
        return None

    if len(fits) > 1:
        print("[survey] %s zone: %d SEPARATE tag squares carry these ids. "
              "Taking the one with the most views." % (name, len(fits)))
        for fit in fits:
            print("[survey]     %s" % fit.describe())
    best = fits[0]
    print("[survey] %s zone: %s" % (name, best.describe()))
    return best


def main():
    parser = argparse.ArgumentParser(
        description="Explore for both zones, then pick and place in one run.")
    # --- sweep, same names and defaults as explore.py so a habit transfers ---
    parser.add_argument("--start", type=float, default=explore.J1_START_DEG)
    parser.add_argument("--end", type=float, default=explore.J1_END_DEG)
    parser.add_argument("--step", type=float, default=explore.J1_COARSE_STEP_DEG,
                        help="coarse step in degrees (default %(default)s)")
    parser.add_argument("--fine-step", type=float,
                        default=explore.J1_FINE_STEP_DEG,
                        help="fine step in degrees (default %(default)s)")
    parser.add_argument("--fine-span", type=float,
                        default=explore.J1_FINE_SPAN_DEG,
                        help="half-width of the fine arc around each zone "
                             "(default %(default)s deg)")
    parser.add_argument("--coarse-patience", type=int, default=2, metavar="N",
                        help="end the coarse sweep once every zone has been "
                             "seen and N consecutive stops have gone by with "
                             "none in view (default %(default)s). 0 always "
                             "sweeps the full range")
    parser.add_argument("--single-pass", action="store_true",
                        help="skip the fine arcs -- one coarse sweep only. "
                             "Faster and a worse fit; use it to check a layout, "
                             "not to pick with")
    parser.add_argument("--pitch", type=float, default=explore.EXPLORE_PITCH_DEG)
    parser.add_argument("--wrist", type=float, default=explore.EXPLORE_WRIST_DEG)
    parser.add_argument("--settle", type=float, default=explore.SETTLE_SECONDS)
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--zone-yaw", type=float, default=None,
                        help="fix BOTH zones' yaw in degrees instead of "
                             "solving each from its sweep. Rarely wanted -- "
                             "solving it is what fixed the radius error")
    parser.add_argument("--origin-radial-bias", type=float, default=None,
                        metavar="M",
                        help="override explore's measured radial correction on "
                             "a surveyed origin, in metres (default %.4f). "
                             "Pass 0 to use the raw fit -- see "
                             "ORIGIN_RADIAL_BIAS_M in explore.py"
                             % explore.ORIGIN_RADIAL_BIAS_M)
    parser.add_argument("--position", default=None, metavar="LABEL",
                        help="named bench position for the PICKUP zone centre "
                             "(%s). Sets --truth-pickup and the note from "
                             "SURVEY_POSITIONS, so a calibration sweep is one "
                             "letter per run" % ", ".join(sorted(SURVEY_POSITIONS)))
    parser.add_argument("--list-positions", action="store_true",
                        help="print the position table and exit")
    parser.add_argument("--skip-pick", action="store_true",
                        help="survey, park over the block, let you dial the "
                             "jaws onto it with 'dx dy dyaw', record it and "
                             "STOP. Nothing descends, so the block does not "
                             "move between positions. This is the calibration "
                             "sweep mode")
    parser.add_argument("--truth-pickup", type=float, nargs=2, metavar=("X", "Y"),
                        default=None,
                        help="the pickup zone centre's TAPED world position, "
                             "metres. Recorded against what the survey found, "
                             "which is the only way survey_error is ever "
                             "anything but 0.0 -- see calibration.py")
    parser.add_argument("--truth-place", type=float, nargs=2, metavar=("X", "Y"),
                        default=None,
                        help="the place zone centre's TAPED world position")
    parser.add_argument("--place-at", type=float, nargs=2, metavar=("X", "Y"),
                        default=None,
                        help="release the block at this world XY and do not "
                             "survey the place zone at all. When the place zone "
                             "never moves, surveying it every run is a fine arc "
                             "and a detect call at every coarse stop spent "
                             "re-deriving a number you already have")
    parser.add_argument("--survey-only", action="store_true",
                        help="find both zones, print them, and stop. Nothing "
                             "is picked. The cheap way to check a new bench "
                             "layout before committing the arm to it")
    # --- pass-through to tag_pick_place ---
    parser.add_argument("--block-class", default=None,
                        help="which block to pick (default: whatever "
                             "tag_pick_place defaults to)")
    parser.add_argument("--any-block", action="store_true",
                        help="pick whatever is best measured, ignoring "
                             "identity. Only safe with ONE block in the zone")
    parser.add_argument("--yes", action="store_true",
                        help="no operator checkpoints -- survey, pick, place, "
                             "hands off")
    parser.add_argument("--note", default="explore_pick_place")
    args = parser.parse_args()

    if args.list_positions:
        print("pickup zone centre positions (world metres, base at the origin):")
        print(describe_positions())
        return 0
    if args.position is not None:
        xy = position_xy(args.position)
        if xy is None:
            parser.error("unknown --position %r; known: %s"
                         % (args.position, ", ".join(sorted(SURVEY_POSITIONS))))
        # Set, not overridden: an explicit --truth-pickup wins, because a taped
        # measurement beats a nominal layout every time and that is the whole
        # lesson of this file's calibration history.
        if args.truth_pickup is None:
            args.truth_pickup = list(xy)
        args.note = "%s position %s" % (args.note, args.position.upper())

    if args.step <= 0:
        parser.error("--step must be positive")
    yaw_fixed = math.radians(args.zone_yaw) if args.zone_yaw is not None else None
    if args.origin_radial_bias is not None:
        explore.ORIGIN_RADIAL_BIAS_M = args.origin_radial_bias
    print("[explore] surveyed origins corrected radially by %+.1f mm "
          "(measured; see ORIGIN_RADIAL_BIAS_M)"
          % (explore.ORIGIN_RADIAL_BIAS_M * 1000))

    rclpy.init()
    io_client = None
    try:
        io_client = pp.RobotIOClient()
        if not io_client.wait_for_joint_states(timeout_sec=10.0):
            print("No /joint_states -- the robot side is not up. "
                  "PROJECT_CONTEXT.md: nan/absent joint states means zero "
                  "publishers, never bad data from the arm.")
            return 1

        # Nominal pose only. explore reads tag_ids and camera_zx/zy, both in the
        # mat's own frame, so the world pose handed in here cannot bias the
        # survey. The detector is rebuilt with the SOLVED pickup pose below,
        # before anything is converted to world coordinates.
        detector = tpp.Detector(io_client, 0.0, explore.NOMINAL_ZONE_RADIUS_M,
                                0.0, 0.0, tpp.zv.DEFAULT_ZONE_SIZE)
        if not detector.wait_for_service(timeout=15.0):
            return 1

        if not args.no_reset:
            print("[explore] reset to %s" % (explore.RESET_JOINTS_DEG,))
            if not explore.send_joints(
                    io_client,
                    [math.radians(d) for d in explore.RESET_JOINTS_DEG],
                    explore.MOVE_SECONDS, "reset"):
                return 1

        print("[explore] pitch %.0f\n" % args.pitch)
        sweep_args = argparse.Namespace(
            start=args.start, end=args.end, step=args.step,
            fine_step=args.fine_step, fine_span=args.fine_span,
            pitch=args.pitch, wrist=args.wrist, settle=args.settle,
            zone_yaw=yaw_fixed, coarse_patience=max(0, args.coarse_patience))
        # HALVES THE DETECT CALLS when the place zone is fixed. Every coarse
        # stop otherwise pays a second service round trip for a zone whose
        # answer is already known, and the place zone earns a fine arc of its
        # own on top of that.
        zones = ("pickup",) if args.place_at is not None else ("pickup", "place")
        if args.single_pass:
            seen = sweep_both(io_client, detector, sweep_args, args.start,
                              args.end, args.step, zones,
                              max(0, args.coarse_patience))
            fit_step = args.step
        else:
            seen = coarse_then_fine_both(io_client, detector, sweep_args, zones)
            fit_step = args.fine_step
        seen.setdefault("place", [])

        print()
        pickup = survey_zone("pickup", seen["pickup"], detector.zone_size,
                             fit_step, yaw_fixed)
        if args.place_at is not None:
            place = None
            print("[survey] place zone: not surveyed -- releasing at the "
                  "(%.4f, %.4f) you gave." % tuple(args.place_at))
        else:
            place = survey_zone("place", seen["place"], detector.zone_size,
                                fit_step, yaw_fixed)
        record_survey("pickup", pickup, args.truth_pickup, args.note)
        record_survey("place", place, args.truth_place, args.note)

        if pickup is None:
            print("\n[run] no pickup zone, so there is nothing to pick. "
                  "Stopping before the arm moves.")
            return 1
        if place is None and args.place_at is None and not args.skip_pick:
            print("\n[run] pickup found but no place zone. Refusing to pick up "
                  "a block with nowhere to put it -- it would end the run held "
                  "in the jaws.")
            return 1
        if args.survey_only:
            print("\n[run] --survey-only: both zones found, nothing picked.")
            return 0

        place_xy = args.place_at if args.place_at is not None else (
            place.origin if place is not None else None)
        print("\n[run] pick from (%.4f, %.4f) yaw %+.1f deg  ->  place at %s"
              % (pickup.origin[0], pickup.origin[1], math.degrees(pickup.yaw),
                 "(%.4f, %.4f)" % tuple(place_xy) if place_xy else
                 "nowhere (--skip-pick)"))

        argv = ["--zone-origin", "%.4f" % pickup.origin[0],
                "%.4f" % pickup.origin[1], "0.0",
                "--zone-yaw", "%.1f" % math.degrees(pickup.yaw),
                "--note", args.note]
        if args.skip_pick:
            # Nothing is carried anywhere, so the place zone is not needed and
            # may not even have been seen.
            argv.append("--skip-pick")
        elif args.place_at is not None:
            argv += ["--place-origin", "%.4f" % args.place_at[0],
                     "%.4f" % args.place_at[1]]
        elif place is not None:
            argv += ["--place-origin", "%.4f" % place.origin[0],
                     "%.4f" % place.origin[1]]
        if args.any_block:
            argv.append("--any-block")
        elif args.block_class:
            argv += ["--block-class", args.block_class]
        if args.yes:
            argv.append("--yes")
        tpp_args = tpp.parse_args(argv)
        tpp_args.zone_z = tpp_args.zone_origin[2]

        print("[run] jaw offset from the flange: radial %+.1f mm, tangential "
              "%+.1f mm" % (pp.JAW_RADIAL_OFFSET_M * 1000,
                            pp.JAW_TANGENTIAL_OFFSET_M * 1000))

        # Rebuilt with the SOLVED pose. Detector.zone_to_world is what turns a
        # block's zone-local position into somewhere the arm can be sent, so it
        # has to carry the survey's answer, not the nominal pose the sweep used.
        detector = tpp.Detector(io_client, pickup.origin[0], pickup.origin[1],
                                tpp_args.zone_z, pickup.yaw,
                                tpp_args.zone_size)
        log = tpp.CorrectionLog(tpp_args.log)
        try:
            ok = tpp.run_stage1(io_client, detector, tpp_args, log)
        finally:
            log.flush()
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1
    finally:
        if io_client is not None:
            print("\n=== Returning home ===")
            try:
                pp.go_home(io_client)
            except Exception as exc:                        # noqa: BLE001
                print("go_home failed on the way out: %s" % exc)
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
