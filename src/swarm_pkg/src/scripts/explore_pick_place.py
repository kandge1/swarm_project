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
#
# L AND M ADDED 2026-08-07 to close that gap, and they are the highest-value two
# rows in the set. The radial model being fitted is
# c + kx*cos(bearing) + ky*sin(bearing) + kr*(r - r_bar) -- cos/sin rather than
# the bearing itself because a world-fixed offset vector projects onto the radial
# direction as exactly that, whereas a term linear in the angle has no physical
# referent and cannot represent one. Over A-K the two trig columns correlate at
# 0.80 and prediction variance blows up past +20 deg; adding L (+90) and M (+67)
# drops the design's condition number from 7.7 to 3.9 and roughly halves the
# standard error on every bearing coefficient.
#
# If the day runs short, drop C, E and J before dropping L or M: bearing
# coverage buys more than sample count here.
SURVEY_POSITIONS = {
    "A": (-3.0, -7.0), "B": (0.0, -7.0), "C": (3.0, -7.0),
    "D": (5.0, -5.0), "E": (6.0, -3.0), "F": (7.0, -2.0),
    "G": (8.0, -1.0), "H": (9.0, 0.0), "I": (8.0, 1.0),
    "J": (7.0, 2.0), "K": (6.0, 2.0),
    "L": (0.0, 7.0), "M": (3.0, 7.0),
    "N": (5.0, 0.0), "O": (7.0, 0.0),
}

# N AND O ARE A RADIUS LADDER, not more bearing coverage. Added 2026-08-07 after
# simulating the fit at the measured sigma of 0.5 mm: with A-M alone the three
# bearing coefficients come out to a standard error of 0.22-0.31 mm, which is
# fine, but kr -- the term that scales with reach -- lands at 8.2 mm/m. Over the
# sweep's 68 mm of radius span that is 0.6 mm of unresolved model, at a target of
# 1-2 mm total.
#
# The cause is that A-M was designed for BEARING: only G, H and I sit at or past
# 8 in and all three are within 7 deg of bearing 0, so radius and bearing are not
# independently excited. N (5,0) and O (7,0) sit at the SAME bearing as H (9,0)
# with radii 127 / 178 / 229 mm, which is a clean radius ladder at fixed bearing.
# Simulated effect: kr's standard error 8.2 -> 5.7 mm/m.
#
# Both clear the reach envelope by 88 mm or more, so they cost only bench time.

# A taped truth further than this from the surveyed origin is a mistake, not a
# measurement -- a mistyped --truth-pickup, the mat on the wrong marks, or a
# survey that latched the wrong square. Every modelled error in this project is
# well inside 30 mm, and the largest one ever found (the 28 mm survey bias) is
# only just inside it, which is why the limit sits here rather than tighter.
TRUTH_SANITY_M = 0.030


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
        # SOLVE THIS MAT'S YAW BEFORE READING ANY OF ITS COARSE RADII. The coarse
        # origins were built with `zone_yaw_for(args, zone) or 0.0`, and with no
        # yaw flag that zero is 91 deg wrong for the pickup mat -- which scattered
        # its coarse radii over 240 mm and made refine_pitch refuse. Per zone,
        # because the two mats are ~180 deg apart. See
        # explore.reseat_coarse_origins.
        explore.reseat_coarse_origins(coarse[zone],
                                      explore.zone_yaw_for(args, zone),
                                      "%s coarse yaw" % zone)
        anchor = sorted(coarse[zone],
                        key=lambda s: (-len(s.tag_ids), s.offset_mm))[0]
        print("\n[explore] %s coarse best: J1 %+.1f with %d tag(s) -- refining "
              "+-%.1f deg around it\n"
              % (zone, anchor.j1_deg, len(anchor.tag_ids), args.fine_span))
        # PER-ZONE PITCH, aimed at the radius the coarse pass found for THIS mat.
        # The two zones can sit at different radii, and each fine arc is swept
        # separately, so each one gets its own aim -- which the single shared
        # args.pitch could not express. See explore.refine_pitch for what the
        # fixed pitch cost on 2026-08-12.
        saved_pitch = args.pitch
        # ALL the coarse sightings, not just the anchor -- see refine_pitch. The
        # anchor is chosen for tag count, which says nothing about whether its
        # projected origin is any good.
        args.pitch = explore.refine_pitch(anchor, args.pitch,
                                          "%s fine arc" % zone,
                                          sightings=coarse[zone])
        try:
            fine = sweep_both(io_client, detector, args,
                              anchor.j1_deg - args.fine_span,
                              anchor.j1_deg + args.fine_span,
                              args.fine_step, (zone,))[zone]
        finally:
            args.pitch = saved_pitch
        # NOTE the fine arc sweeps ONE zone, so explore.zone_yaw_for picks that
        # zone's own yaw for every sighting in it -- which is the half that
        # matters, because these are the sightings the fit is built from.
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
    # SANITY, AND IT HAS ALREADY EARNED ITS KEEP. The 2026-08-07 sweep produced
    # two rows at 176 mm and 319 mm, both from a --truth-place or --truth-pickup
    # left over from the previous position while the mat had moved. The surveyed
    # origins were fine; the truth they were compared against was stale.
    #
    # There WAS a 30 mm refusal for this, but it guarded the block row's argv and
    # --survey-only returns long before that code -- so the mode used for the
    # entire sweep had no check at all. Marked rather than dropped: the row is
    # evidence about the session, and calibration.load() already drops `invalid`
    # rows loudly, which is exactly the behaviour wanted.
    invalid = None
    if error > TRUTH_SANITY_M:
        invalid = ("surveyed origin %.0f mm from the taped truth -- beyond any "
                   "modelled error, so the truth column is stale, the mat moved, "
                   "or the survey latched the wrong square" % (error * 1000.0))
        print("[survey] %s: REFUSING to treat this as a calibration point. %s"
              % (name, invalid))
        print("[survey] %s: check --truth-%s against where the mat actually is."
              % (name, name))
    calibration.record(
        None,
        invalid=invalid,
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
    # A ONE-VIEW FIT HAS RESIDUAL 0 BY CONSTRUCTION. describe() then prints
    # "residual 0.0 mm over 1 view(s)", which reads as perfect and is arithmetic:
    # one view cannot disagree with itself. Lesson 4 -- internal agreement is not
    # accuracy -- and this path is reachable exactly when it matters, because
    # --zone-yaw skips the baseline check that otherwise rejects a single view.
    if len(best.sightings) < 2:
        print("[survey] %s zone: WARNING -- that origin comes from ONE sighting. "
              "Its residual is 0.0 mm because there is nothing to compare it "
              "with, NOT because it is exact. Nothing cross-checks the origin, "
              "and --zone-yaw means nothing cross-checks the yaw either. Keep "
              "--confirm on and look at the jaws." % name)
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
    parser.add_argument("--no-truth-block-on-centre", dest="truth_block_on_centre",
                        action="store_false", default=True,
                        help="do NOT tell tag_pick_place the block is on the "
                             "zone centre. The default assumes it is, which is "
                             "what makes survey_error and vision_error "
                             "computable on the block row -- pass this when the "
                             "block is deliberately placed off-centre, as in "
                             "the in-zone sweep, or the truth column is a lie")
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
    parser.add_argument("--force-grasp-yaw", type=float, default=None,
                        metavar="DEG",
                        help="forwarded to tag_pick_place: command this grasp "
                             "yaw instead of the measured one. Use 0 for x/y "
                             "calibration with the block's far side toward -Y")
    parser.add_argument("--yes", action="store_true",
                        help="no operator checkpoints -- survey, pick, place, "
                             "hands off")
    parser.add_argument("--dump-sightings", default=None, metavar="PATH",
                        help="write every sighting to JSON, so a survey that "
                             "found nothing can be re-fitted offline instead of "
                             "reverse-engineered from the log. See "
                             "explore.load_sightings")
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
        # BEFORE the gate runs, so a survey that rejects everything still leaves
        # the evidence behind. That is the case the dump exists for.
        if args.dump_sightings:
            explore.dump_sightings(args.dump_sightings, seen)

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

        # --survey-only IS CHECKED FIRST, and it used to be checked last. The
        # place-zone refusal below is about not stranding a block in the jaws,
        # which cannot happen on a run that never picks anything up -- so a
        # survey sweep with no place mat on the bench was exiting 1 with
        # "Refusing to pick up a block with nowhere to put it", a message about a
        # pick that was never requested. The pickup row was already written by
        # record_survey above, so no data was lost, but the run looked failed and
        # the exit code said so.
        #
        # This is the mode the 15-position survey sweep uses, and the sweep does
        # not need the place mat, the block, or a caliper.
        if args.survey_only:
            print("\n[run] --survey-only: %s. Nothing picked, the gripper never "
                  "left home."
                  % ("both zones surveyed" if pickup is not None
                     and place is not None else
                     "pickup surveyed" if pickup is not None else
                     "NOTHING surveyed"))
            return 0 if pickup is not None else 1
        if pickup is None:
            print("\n[run] no pickup zone, so there is nothing to pick. "
                  "Stopping before the arm moves.")
            return 1
        if place is None and args.place_at is None and not args.skip_pick:
            print("\n[run] pickup found but no place zone. Refusing to pick up "
                  "a block with nowhere to put it -- it would end the run held "
                  "in the jaws.")
            return 1

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
        # Pinning the grasp yaw is useless unless it reaches the child -- this
        # argv list is an allowlist, so a flag absent here is silently dropped.
        if args.force_grasp_yaw is not None:
            argv += ["--force-grasp-yaw", "%.3f" % args.force_grasp_yaw]
        # THE TRUTH COLUMN, forwarded at last. Without this the block row carries
        # truth_world=None and truth_zone=None, so calibration.vision_error and
        # calibration.survey_error both return None on it and the only signal in
        # a whole sweep is the total nudge -- survey, vision and jaw offset
        # lumped into one number with no way to attribute any of it. Position A's
        # row is exactly that: a 3.2 mm nudge that cannot be assigned to anything.
        #
        # --truth-block-zone is the one that makes survey_error computable: it
        # asserts the block is ON the zone centre, which is what makes the
        # block's true world position also the zone origin's.
        #
        # UNITS DIFFER AND IT IS NOT SYMMETRIC: --truth-block-world is METRES,
        # --truth-block-zone is MILLIMETRES (tag_pick_place.py:2172-2183). A
        # silent factor of 1000 here would poison the truth column, which is
        # Lesson 5's exact failure mode.
        if args.truth_block_on_centre and args.truth_pickup is not None:
            gap = math.hypot(args.truth_pickup[0] - pickup.origin[0],
                             args.truth_pickup[1] - pickup.origin[1])
            if gap > TRUTH_SANITY_M:
                # REFUSE rather than record. Past this distance the survey and
                # the tape disagree by more than any modelled error, so one of
                # them is wrong -- a mistyped --truth-pickup, the mat on the
                # wrong marks, or a survey that latched the other zone. Writing
                # the row anyway produces a confident 40 mm calibration point,
                # and Saturday's fit has no way to tell it from a real one.
                print("\n[run] REFUSING to record a truth column: the taped "
                      "pickup centre (%.4f, %.4f) is %.1f mm from the surveyed "
                      "origin (%.4f, %.4f), over the %.0f mm sanity limit."
                      % (args.truth_pickup[0], args.truth_pickup[1], gap * 1000,
                         pickup.origin[0], pickup.origin[1],
                         TRUTH_SANITY_M * 1000))
                print("[run] Check --truth-pickup, the mat position, and that "
                      "the survey found the PICKUP square. Re-run with "
                      "--no-truth-block-on-centre to record without a truth.")
                return 1
            argv += ["--truth-block-world",
                     "%.4f" % args.truth_pickup[0],
                     "%.4f" % args.truth_pickup[1],
                     # MILLIMETRES, and zero by definition: "the block is on the
                     # zone centre" is the assertion being made.
                     "--truth-block-zone", "0", "0"]
            print("[run] truth: block taped on the zone centre at (%.4f, %.4f) "
                  "m; survey is %.1f mm from it"
                  % (args.truth_pickup[0], args.truth_pickup[1], gap * 1000))
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
