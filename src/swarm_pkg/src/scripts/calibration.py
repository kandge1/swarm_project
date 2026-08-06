#!/usr/bin/env python3
"""Append-only record of what the arm was told, what it saw, and what was true.

PURE PYTHON. No ROS, no OpenCV -- so it imports anywhere, and the arithmetic
that decides whether a correction is justified can be tested offline instead of
on a robot.

    python3 calibration.py --report              # what the history says so far
    python3 calibration.py --report --by pose    # split by grasp pose
    python3 calibration.py --selftest

------------------------------------------------------------------------------
GROUND TRUTH DOES NOT LIVE IN THIS FILE, OR IN ANY OTHER
------------------------------------------------------------------------------
Every truth value here arrives as an ARGUMENT and is stored in the row it
belongs to. Nothing in the repo defaults to "the zone is 9 inches out".

That is a deliberate constraint and it is worth stating why, because hardcoding
it is obviously more convenient. A constant that encodes today's bench layout
turns every future measurement into a comparison against a stale assumption --
and the failure is silent, because the numbers still come out looking plausible.
This project has already paid for that once: explore.py defaulted zone_yaw to 0,
the mat was at -89 deg, and eleven internally-consistent views produced a radius
that was wrong by 24 mm with a 3 mm fit residual. A wrong constant with a good
residual is the worst instrument there is.

So the bench setup is an input. Move the mat and the old rows stay honest,
because each one carries the truth it was taken against.

------------------------------------------------------------------------------
WHAT A KNOWN BLOCK POSITION BUYS, WHICH IS MORE THAN IT LOOKS
------------------------------------------------------------------------------
Until now the pipeline's errors were not separable. A grasp that missed by
20 mm could be the zone survey, the vision, or the arm, and no run could say
which -- the same reason tag_pick_place's --verify is off by default.

Putting the block ON the zone centre breaks that open, because it makes the
block's ZONE-LOCAL position known independently of everything else:

  * VISION error = measured zone-local minus its true zone-local. Owes nothing
    to the arm (the tags fix the frame, not the encoders) and nothing to the
    zone survey (zone-local is upstream of it). This is the one measurement in
    the whole pipeline with no confounds.

  * SURVEY error = the zone origin that was passed in, minus the block's true
    world position -- valid only when the block really is at the zone centre,
    which is exactly the arrangement that makes it easy.

  * ARM error = truth minus what the flange was actually commanded, and it is
    only meaningful on a run that GRASPED, because "the jaws ended up on the
    block" is what pins the arm's true position. A missed grasp bounds it and
    does not measure it.

The operator nudge is not a fudge factor; it is the arm-error measurement. Some-
one looked at the jaws, read the offset, and typed it. Recording it is the whole
point of recording anything.

------------------------------------------------------------------------------
WHY THIS DOES NOT AUTOMATICALLY CORRECT ANYTHING
------------------------------------------------------------------------------
Because one measurement cannot tell a camera-model error from an arm error, and
the two want opposite fixes. Sag, backlash and the servo dead band all vary with
configuration, so an arm error measured at the detect hover and applied at the
grasp pose is an extrapolation. A camera-model error does not vary that way.

The history is what tells them apart, and the test is stated up front so it
cannot be rationalised afterwards:

    If the RADIAL error agrees across clearly different poses to within a few
    millimetres over several runs, it is the camera model, and ONE constant
    fixes it everywhere.

    If it tracks the pose, it is the arm, and no constant will do -- that is
    the disturbance-observer dataset tag_pick_place's --log was already
    collecting for.

Until the rows exist, both stories fit, and applying either one has a coin-flip
chance of making the grasp worse. So: record now, decide later, and let
report() say which way the evidence points rather than deciding here.
"""
import argparse
import json
import math
import os
import sys
import time

# Where the rows go. Beside logs.txt, because it is the same kind of artefact --
# something produced by running the robot, not something the robot needs.
DEFAULT_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "logs", "calibration_history.jsonl")

# JSON Lines rather than CSV: rows gain fields as the pipeline grows, and a CSV
# whose column set changes halfway down is a file nobody can load. One JSON
# object per line stays readable, appendable and diffable, and a reader that
# does not know a field simply does not ask for it.
SCHEMA = 1


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def record(path=None, **row):
    """Append one run. Returns the row as written.

    Never raises on a write problem -- losing a calibration row must not fail a
    grasp that otherwise worked. It reports and moves on.
    """
    path = path or DEFAULT_LOG
    row.setdefault("schema", SCHEMA)
    row.setdefault("time", time.time())
    row.setdefault("time_iso", time.strftime("%Y-%m-%dT%H:%M:%S",
                                             time.localtime(row["time"])))
    try:
        directory = os.path.dirname(os.path.abspath(path))
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(path, "a") as handle:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    except Exception as exc:                            # noqa: BLE001
        print("[calibration] could NOT write %s (%s: %s) -- the run is "
              "unaffected, the row is lost" % (path, type(exc).__name__, exc))
    return row


def load(path=None):
    """Every readable row. A corrupt line is skipped loudly, not fatally."""
    path = path or DEFAULT_LOG
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as handle:
        for number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError as exc:
                print("[calibration] %s:%d is not JSON (%s) -- skipped"
                      % (path, number, exc))
    return rows


# ---------------------------------------------------------------------------
# The three errors
# ---------------------------------------------------------------------------
def vision_error(row):
    """(dx, dy) metres in the ZONE frame: measured block minus its truth.

    The confound-free one. See the module docstring.
    """
    measured, truth = row.get("measured_zone"), row.get("truth_zone")
    if not (measured and truth):
        return None
    return (measured[0] - truth[0], measured[1] - truth[1])


def survey_error(row):
    """(dx, dy) metres in WORLD: the zone origin used, minus where it really is.

    Only defined when the block was ON the zone centre, because that is what
    makes the block's true world position also the zone origin's. A run with the
    block elsewhere records fine and simply does not answer this question.
    """
    origin, truth_world = row.get("zone_origin"), row.get("truth_world")
    truth_zone = row.get("truth_zone")
    if not (origin and truth_world and truth_zone):
        return None
    if math.hypot(truth_zone[0], truth_zone[1]) > 1e-6:
        return None
    return (origin[0] - truth_world[0], origin[1] - truth_world[1])


def open_loop_error(row):
    """(dx, dy) metres in WORLD: where the pipeline SAID the block was, minus
    where it was. Survey error and vision error added together -- the number a
    grasp would miss by if the arm were perfect."""
    measured, truth = row.get("measured_world"), row.get("truth_world")
    if not (measured and truth):
        return None
    return (measured[0] - truth[0], measured[1] - truth[1])


def arm_error(row):
    """(dx, dy) metres in WORLD: truth minus what the flange was commanded.

    ONLY on a run that grasped. "The jaws closed on the block" is what fixes
    where the arm actually was; a miss bounds the error and does not measure it,
    and averaging bounds into measurements is how a confident wrong constant
    gets made.
    """
    if not row.get("grasped"):
        return None
    commanded, truth = row.get("commanded_world"), row.get("truth_world")
    if not (commanded and truth):
        return None
    return (truth[0] - commanded[0], truth[1] - commanded[1])


def jaw_offset(row):
    """(radial, tangential) metres from the FLANGE to the JAWS, measured.

    The one quantity in this file that needs no frame assumed. On a confirmed
    grasp the jaws were, by definition, on the block, so truth_world IS the jaw
    position; flange_fk is where the flange was at that instant. The difference
    is the offset, and resolving it about the bearing to the flange puts it in
    the same (radial, tangential) terms compensate_for_tip_swing uses.

    Requires flange_fk, which only rows written on or after 2026-08-06 carry.

    WHY THIS EXISTS: the flange-to-jaw offset has now been calibrated three
    times and has been wrong in a new frame each time -- world in August 3,
    radial in August 4, neither at bearing 0 in August 6. Every one of those was
    reconstructed after the fact from printed logs. This records the measurement
    at the moment it is made, so the fourth attempt fits data instead of prose.
    """
    if not row.get("grasped"):
        return None
    flange, truth = row.get("flange_fk"), row.get("truth_world")
    if not (flange and truth) or len(flange) < 2:
        return None
    return radial_and_lateral((truth[0] - flange[0], truth[1] - flange[1]),
                              (flange[0], flange[1]))


def jaw_offset_report(rows):
    """Lines describing the measured flange-to-jaw offset, grouped by bearing.

    Grouped because the frame is unresolved: one bearing cannot distinguish a
    radial offset from a world-fixed one, and printing a single mean across
    bearings is exactly the mistake that produced the last two constants.
    """
    seen = []
    for row in rows:
        offset = jaw_offset(row)
        if offset is None:
            continue
        flange = row["flange_fk"]
        seen.append((math.degrees(math.atan2(flange[1], flange[0])), offset,
                     row.get("jaw_offset")))
    if not seen:
        return ["  FLANGE-TO-JAW: no confirmed grasp carries flange FK yet. "
                "One grasp with", "  --truth-block-world and a 'y' at the final "
                "prompt starts this table."]

    buckets = {}
    for bearing, offset, model in seen:
        buckets.setdefault(round(bearing / 15.0) * 15, []).append((offset, model))
    lines = ["  FLANGE-TO-JAW, measured on %d confirmed grasp(s):" % len(seen)]
    for bearing in sorted(buckets):
        got = buckets[bearing]
        rad, tan = Stat("radial"), Stat("tangential")
        for offset, _model in got:
            rad.add(offset[0])
            tan.add(offset[1])
        lines.append("    bearing %+4d deg  n=%d  radial %+5.1f mm%s  "
                     "tangential %+5.1f mm%s"
                     % (bearing, len(got), rad.mean * 1000,
                        " (sd %.1f)" % (rad.spread * 1000) if rad.spread
                        is not None else "",
                        tan.mean * 1000,
                        " (sd %.1f)" % (tan.spread * 1000) if tan.spread
                        is not None else ""))
    if len(buckets) < 2:
        lines.append("    ONE BEARING ONLY -- radial, world-fixed and tool-fixed "
                     "offsets are")
        lines.append("    indistinguishable here. A second bearing 30+ deg away "
                     "decides the frame.")
    return lines


def radial_and_lateral(vector, at_xy):
    """Split a world-frame error into (radial, lateral) about the base.

    Radial is the component along the line from the base, and it is the one that
    matters: every systematic this project has found -- explore's 24 mm, the
    survey hover's 22 mm -- has been radial, which is what a camera-model or
    reach error looks like. Lateral is mostly bearing error and backlash.
    """
    if vector is None or at_xy is None:
        return None
    norm = math.hypot(at_xy[0], at_xy[1])
    if norm < 1e-9:
        return None
    ux, uy = at_xy[0] / norm, at_xy[1] / norm
    return (vector[0] * ux + vector[1] * uy, -vector[0] * uy + vector[1] * ux)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
class Stat(object):
    __slots__ = ("name", "values")

    def __init__(self, name):
        self.name = name
        self.values = []

    def add(self, value):
        if _finite(value):
            self.values.append(value)

    @property
    def n(self):
        return len(self.values)

    @property
    def mean(self):
        return sum(self.values) / len(self.values) if self.values else None

    @property
    def spread(self):
        """Sample standard deviation. None below two points, deliberately --
        a "spread" computed from one measurement reads as certainty."""
        if len(self.values) < 2:
            return None
        mean = self.mean
        return math.sqrt(sum((v - mean) ** 2 for v in self.values)
                         / (len(self.values) - 1))

    def line(self):
        if not self.values:
            return "  %-34s no data" % self.name
        spread = ("+-%5.1f" % (self.spread * 1000)) if self.spread is not None \
            else "   -- "
        return ("  %-34s %+7.1f mm  %s   n=%d"
                % (self.name, self.mean * 1000, spread, self.n))


# Agreement tighter than this across DIFFERENT poses is what would justify a
# single correction constant. Chosen against the two numbers already in hand --
# explore's +24 mm at the lookout pose and the survey hover's +22 mm -- which
# differ by 2 mm; if further poses stay inside this band the case is made.
CONSISTENT_MM = 5.0

# Below this many runs, say so rather than reporting a mean. Two points can
# agree by luck; three that agree across different poses cannot as easily.
ENOUGH_RUNS = 3

# How different two poses have to be before repeating a measurement at both
# counts as evidence about WHICH subsystem is wrong. Five runs with the block on
# the same spot prove the pipeline is repeatable and say nothing about whether
# the error is carried by the camera or by the arm -- both are constant when the
# pose is constant. The first version of verdict() missed this and called five
# same-place runs CONSISTENT, counting rounding noise in the grasp yaw as four
# "distinct poses". Repeatability is a real and useful result; it is just not
# this one.
#
# 30 mm is about an eighth of the working radius and well outside the ~3 mm
# scatter the runs actually show. 20 deg of bearing similarly swings the arm
# into a visibly different configuration.
MIN_POSE_SPREAD_M = 0.030
MIN_POSE_SPREAD_DEG = 20.0


def pose_spread(rows):
    """(radius spread m, bearing spread deg) over the poses actually visited."""
    places = [row.get("truth_world") or row.get("measured_world")
              for row in rows]
    places = [p for p in places if p]
    if len(places) < 2:
        return 0.0, 0.0
    radii = [math.hypot(p[0], p[1]) for p in places]
    bearings = [math.degrees(math.atan2(p[1], p[0])) for p in places]
    return max(radii) - min(radii), max(bearings) - min(bearings)


def summarise(rows):
    """-> {label: Stat}. Everything the history can currently say."""
    stats = {name: Stat(name) for name in (
        "vision radial (zone)", "vision lateral (zone)",
        "survey radial", "survey lateral",
        "open-loop radial", "open-loop lateral",
        "arm radial (grasped runs)", "arm lateral (grasped runs)",
        "operator nudge radial", "operator nudge lateral")}

    for row in rows:
        at = row.get("truth_world") or row.get("measured_world")
        for label, vector in (("survey", survey_error(row)),
                              ("open-loop", open_loop_error(row)),
                              ("arm", arm_error(row))):
            split = radial_and_lateral(vector, at)
            if split is None:
                continue
            key = ("arm radial (grasped runs)" if label == "arm"
                   else "%s radial" % label)
            stats[key].add(split[0])
            key = ("arm lateral (grasped runs)" if label == "arm"
                   else "%s lateral" % label)
            stats[key].add(split[1])

        # Vision error is already in the zone frame, where "radial" means along
        # zone +X. Not rotated into world: the point of this number is that it
        # is upstream of the zone pose, and rotating it by the zone yaw would
        # drag the survey's error back into it.
        vision = vision_error(row)
        if vision is not None:
            stats["vision radial (zone)"].add(vision[0])
            stats["vision lateral (zone)"].add(vision[1])

        nudge = row.get("nudge")
        split = radial_and_lateral(nudge, at) if nudge else None
        if split is not None:
            stats["operator nudge radial"].add(split[0])
            stats["operator nudge lateral"].add(split[1])

    return stats


def verdict(rows):
    """What the history currently supports. Text, not a correction."""
    at = lambda row: row.get("truth_world") or row.get("measured_world")
    open_loop = [r for r in (radial_and_lateral(open_loop_error(row), at(row))
                             for row in rows) if r is not None]
    arm = [r for r in (radial_and_lateral(arm_error(row), at(row))
                       for row in rows) if r is not None]
    values = [r[0] for r in open_loop]
    if len(values) < ENOUGH_RUNS:
        return ("NOT ENOUGH DATA: %d run(s) with a truth attached, %d wanted.\n"
                "  One or two runs cannot separate a camera-model error from an\n"
                "  arm error, and those want opposite fixes. Keep running with\n"
                "  --truth-block-world / --truth-block-zone and re-read this."
                % (len(values), ENOUGH_RUNS))

    spread = max(values) - min(values)
    mean = sum(values) / len(values)
    dr, dbearing = pose_spread(rows)

    arm_note = ""
    if len(arm) >= 2:
        radial = [a[0] for a in arm]
        lateral = [a[1] for a in arm]
        arm_note = ("\n\n  ARM, from %d confirmed grasp(s): radial %+.1f mm "
                    "(span %.1f), lateral %+.1f mm (span %.1f).\n"
                    "  This is the actionable one -- it is what the operator "
                    "nudged out by hand every\n  run, and it is measured at the "
                    "GRASP pose rather than extrapolated to it."
                    % (len(arm), sum(radial) / len(radial) * 1000,
                       (max(radial) - min(radial)) * 1000,
                       sum(lateral) / len(lateral) * 1000,
                       (max(lateral) - min(lateral)) * 1000))

    if dr < MIN_POSE_SPREAD_M and dbearing < MIN_POSE_SPREAD_DEG:
        return ("REPEATABLE AT ONE POSE: %d runs, radial open-loop error spans "
                "%.1f mm (mean %+.1f mm),\n"
                "  but every run sat within %.0f mm and %.1f deg of the same "
                "place. That is a real\n"
                "  result -- the pipeline returns the same answer twice -- and "
                "it CANNOT say whether\n"
                "  the error is the camera model or the arm, because both are "
                "constant when the pose\n"
                "  is. Move the block to a clearly different radius (%.0f mm+) "
                "or bearing (%.0f deg+)\n"
                "  and run again; that one run decides it."
                % (len(values), spread * 1000, mean * 1000, dr * 1000, dbearing,
                   MIN_POSE_SPREAD_M * 1000, MIN_POSE_SPREAD_DEG) + arm_note)

    if spread * 1000 <= CONSISTENT_MM:
        return ("CONSISTENT ACROSS POSES: %d runs spanning %.0f mm of radius "
                "and %.0f deg of bearing,\n"
                "  radial open-loop error spans only %.1f mm (mean %+.1f mm). "
                "An error that does not\n"
                "  move with the pose is carried by the CAMERA MODEL, not the "
                "arm, and one constant\n  would fix it everywhere."
                % (len(values), dr * 1000, dbearing, spread * 1000,
                   mean * 1000) + arm_note)

    return ("POSE-DEPENDENT: %d runs spanning %.0f mm of radius and %.0f deg of "
            "bearing,\n"
            "  radial open-loop error spans %.1f mm (%.1f to %.1f). That is too "
            "much for one\n"
            "  constant. An error that tracks the pose is the ARM -- sag, "
            "backlash, dead band --\n"
            "  and wants the disturbance-observer model, not an offset. "
            "tag_pick_place's --log\n  CSV is the dataset for that."
            % (len(values), dr * 1000, dbearing, spread * 1000,
               min(values) * 1000, max(values) * 1000) + arm_note)


def report(path=None, rows=None):
    rows = load(path) if rows is None else rows
    print("calibration history: %d run(s) from %s"
          % (len(rows), path or DEFAULT_LOG))
    if not rows:
        print("\n  Nothing recorded yet. Runs with --truth-block-world and\n"
              "  --truth-block-zone land here automatically.")
        return 0

    print("\n  %-34s %10s  %6s   %s" % ("", "mean", "sd", ""))
    stats = summarise(rows)
    for name in ("vision radial (zone)", "vision lateral (zone)",
                 "survey radial", "survey lateral",
                 "open-loop radial", "open-loop lateral",
                 "arm radial (grasped runs)", "arm lateral (grasped runs)",
                 "operator nudge radial", "operator nudge lateral"):
        print(stats[name].line())

    print("\n  most recent runs:")
    for row in rows[-5:]:
        vision = vision_error(row)
        openloop = open_loop_error(row)
        print("    %s  %-12s  vision %s  open-loop %s  %s"
              % (row.get("time_iso", "?"), row.get("block_class", "?"),
                 "(%+5.1f,%+5.1f) mm" % (vision[0] * 1000, vision[1] * 1000)
                 if vision else "        --      ",
                 "(%+5.1f,%+5.1f) mm" % (openloop[0] * 1000, openloop[1] * 1000)
                 if openloop else "        --      ",
                 "GRASPED" if row.get("grasped") else "no grasp"))

    print("\n%s\n" % verdict(rows))
    for line in jaw_offset_report(rows):
        print(line)
    print("")
    return 0


# ---------------------------------------------------------------------------
def _selftest():
    failures = []

    def check(name, ok, detail=""):
        print("  %-52s %s%s" % (name, "ok" if ok else "FAIL",
                                "" if ok else "  " + detail))
        if not ok:
            failures.append(name)

    # A run where the vision is perfect, the survey is 24 mm long, and the arm
    # is perfect: the miss should be all survey, no vision.
    row = dict(zone_origin=[0.2526, 0.0027], zone_yaw_deg=-179.1,
               truth_world=[0.2286, 0.0], truth_zone=[0.0, 0.0],
               measured_zone=[0.0, 0.0], measured_world=[0.2526, 0.0027],
               commanded_world=[0.2526, 0.0027], nudge=[-0.024, -0.0027],
               grasped=True, block_class="orange_cube", grasp_yaw_deg=1.2)
    check("perfect vision reads zero error",
          max(abs(v) for v in vision_error(row)) < 1e-9)
    radial = radial_and_lateral(survey_error(row), row["truth_world"])
    check("survey error comes out +24 mm radial",
          abs(radial[0] - 0.0240) < 1e-4, "%.4f" % radial[0])
    radial = radial_and_lateral(arm_error(row), row["truth_world"])
    check("a grasped run with a commanded miss shows it as arm error",
          abs(radial[0] + 0.0240) < 1e-4, "%.4f" % radial[0])
    check("arm error is None when the run did not grasp",
          arm_error(dict(row, grasped=False)) is None)
    check("survey error is None when the block was NOT at the zone centre",
          survey_error(dict(row, truth_zone=[0.01, 0.0])) is None)

    # Vision that is genuinely off, with a perfect survey.
    off = dict(row, zone_origin=[0.2286, 0.0], measured_zone=[0.006, -0.002],
               measured_world=[0.2226, -0.002])
    check("vision error is reported in the ZONE frame, unrotated",
          abs(vision_error(off)[0] - 0.006) < 1e-9)
    check("a perfect survey reads zero survey error",
          max(abs(v) for v in survey_error(off)) < 1e-9)

    check("one run refuses to draw a conclusion",
          "NOT ENOUGH DATA" in verdict([row]))
    tight = [dict(row, measured_world=[0.2286 + d, 0.0],
                  commanded_world=[0.2286 + d, 0.0])
             for d in (0.0230, 0.0240, 0.0245)]
    check("agreeing runs at ONE pose do NOT claim a camera-model error",
          "REPEATABLE AT ONE POSE" in verdict(tight), verdict(tight)[:40])
    spread_out = [dict(row, truth_world=[r, 0.0], measured_world=[r + 0.024, 0.0],
                       commanded_world=[r + 0.024, 0.0])
                  for r in (0.180, 0.229, 0.280)]
    check("agreeing runs at DIFFERENT radii do claim it",
          "CONSISTENT ACROSS POSES" in verdict(spread_out),
          verdict(spread_out)[:40])
    check("pose_spread sees the difference",
          abs(pose_spread(spread_out)[0] - 0.100) < 1e-9)
    loose = [dict(row, truth_world=[r, 0.0], measured_world=[r + d, 0.0],
                  commanded_world=[r + d, 0.0])
             for r, d in ((0.180, 0.004), (0.229, 0.024), (0.280, 0.041))]
    check("three disagreeing runs read as POSE-DEPENDENT (arm)",
          "POSE-DEPENDENT" in verdict(loose), verdict(loose)[:40])

    stats = summarise(tight)
    check("stats count every run", stats["open-loop radial"].n == 3)
    # Units caught a real bug: the arm line printed metres into a "mm" format
    # and read +0.0 against a table saying +7.0.
    armed = [dict(row, truth_world=[0.2286, 0.0],
                  measured_world=[0.2286, 0.0],
                  commanded_world=[0.2286 - 0.007, 0.0], grasped=True)
             for _ in range(ENOUGH_RUNS)]
    check("the arm line reports millimetres, not metres",
          "radial +7.0 mm" in verdict(armed), verdict(armed)[-140:])
    check("spread is None for a single point", Stat("x").spread is None)

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", default=None,
                        help="history file (default: %s)" % DEFAULT_LOG)
    parser.add_argument("--report", action="store_true",
                        help="summarise what the history says")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return _selftest()
    return report(args.log)


if __name__ == "__main__":
    sys.exit(main())
