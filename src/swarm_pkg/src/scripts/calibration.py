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


def load(path=None, keep_invalid=False):
    """Every readable row. A corrupt line is skipped loudly, not fatally.

    Rows carrying an `invalid` field are dropped, and the count is printed. That
    field is written by nobody in this repo and read by nobody -- it was added by
    hand to ten Stage 2a rows whose truth column had been pasted identically
    across all of them, i.e. rows known to be wrong and left in the file anyway.
    Every statistic in this module has been consuming them as if they were good.

    Filtered LOUDLY, like the corrupt-line path above, for the same reason: a
    silent filter is how a hand-added field nobody wrote and nobody read got
    there in the first place. keep_invalid=True to see them again.
    """
    path = path or DEFAULT_LOG
    rows = []
    dropped = []
    if not os.path.exists(path):
        return rows
    with open(path) as handle:
        for number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError as exc:
                print("[calibration] %s:%d is not JSON (%s) -- skipped"
                      % (path, number, exc))
                continue
            if row.get("invalid") and not keep_invalid:
                dropped.append((number, row["invalid"]))
                continue
            rows.append(row)
    if dropped:
        print("[calibration] dropped %d row(s) marked invalid:" % len(dropped))
        reasons = sorted({reason for _n, reason in dropped})
        for reason in reasons:
            lines = [str(n) for n, r in dropped if r == reason]
            print("[calibration]   line%s %s: %s"
                  % ("s" if len(lines) > 1 else "", ",".join(lines), reason))
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


# ---------------------------------------------------------------------------
# Fitting
#
# WHY cos/sin AND NOT THE BEARING ITSELF. The thing being separated is a radial
# constant from a world-fixed offset vector, and a world-fixed vector (vx, vy)
# projects onto the radial direction as exactly vx*cos(b) + vy*sin(b). A term
# linear in the angle has no physical referent at all and cannot represent one --
# it is not a conditioning preference, it is the difference between fitting the
# quantity and fitting a proxy for it. The four coefficients then have four
# distinct homes:
#
#   c   a genuinely radial constant   -> explore.apply_origin_radial_bias
#   kx,ky a world-fixed offset        -> a world-frame term, NOT a radial one
#   kr  scales with reach             -> a link-length or compliance term
#
# Pure Python on purpose, like the rest of this module: it has to run offline
# with no numpy so it can be tested without a robot.
# ---------------------------------------------------------------------------
def linear_fit(xs, ys):
    """Least-squares (slope, intercept, R^2), or None if x has no spread.

    Lifted verbatim from serial_rate_probe.py, where it fitted the gravity droop
    coefficients. Kept for one-regressor diagnostics; it gains its first tests
    here, since nothing tested it there.
    """
    n = len(xs)
    if n < 2:
        return None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx < 1e-12:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sxx
    intercept = mean_y - slope * mean_x
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return slope, intercept, r2


def normal_equations(design, ys):
    """Least squares for any number of columns. -> (coef, r2, ses, dof) or None.

    Gaussian elimination with partial pivoting on the normal equations, the same
    body serial_rate_probe.multi_fit uses -- but general in the column count.
    multi_fit was NOT ported: it hard-wires three columns via `range(3)` in four
    places, and the model here needs four now and probably five later.

    Returns None when the design is rank deficient, which is exactly what
    collinear regressors produce -- the failure the +Y sweep positions exist to
    avoid. `ses` are the per-coefficient standard errors, and they are the part
    that matters: a coefficient smaller than its own standard error is a
    coefficient the data did not measure, and printing it without one invites
    exactly the kind of confident wrong constant this file exists to prevent.
    """
    n = len(design)
    if not n:
        return None
    p = len(design[0])
    if n <= p:
        return None
    ata = [[sum(design[k][i] * design[k][j] for k in range(n)) for j in range(p)]
           for i in range(p)]
    atb = [sum(design[k][i] * ys[k] for k in range(n)) for i in range(p)]
    # Solve, and invert in the same pass, so the standard errors come for free.
    aug = [ata[i][:] + [atb[i]] + [1.0 if j == i else 0.0 for j in range(p)]
           for i in range(p)]
    width = 1 + 2 * p
    for col in range(p):
        pivot = max(range(col, p), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            return None
        aug[col], aug[pivot] = aug[pivot], aug[col]
        pdiv = aug[col][col]
        for c in range(col, width):
            aug[col][c] /= pdiv
        for r in range(p):
            if r == col:
                continue
            f = aug[r][col]
            if f:
                for c in range(col, width):
                    aug[r][c] -= f * aug[col][c]
    coef = [aug[i][p] for i in range(p)]
    inv = [[aug[i][p + 1 + j] for j in range(p)] for i in range(p)]
    mean_y = sum(ys) / n
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    ss_res = sum((ys[k] - sum(coef[i] * design[k][i] for i in range(p))) ** 2
                 for k in range(n))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    dof = n - p
    sigma2 = ss_res / dof if dof > 0 else float("nan")
    ses = [math.sqrt(sigma2 * inv[i][i]) if sigma2 == sigma2 and inv[i][i] > 0
           else float("nan") for i in range(p)]
    return coef, r2, ses, dof


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


# Minimum bearing span before a cos/sin fit is allowed to claim anything. Below
# this the two trig columns are collinear to within the noise and the solve, if
# it succeeds at all, is splitting a constant between three coefficients.
MIN_FIT_BEARING_SPAN_DEG = 60.0

# Below this many usable rows there is no point: the model has four parameters.
MIN_FIT_ROWS = 6


def row_zone(row):
    """"pickup", "place", or None -- which zone a survey row measured.

    record_survey writes note "<note> survey <name>", so the name is the last
    token. Read from the note rather than a field because every row in the file
    predates any such field; a `zone` key, if one is ever added, should be
    preferred here over the note.
    """
    note = (row.get("note") or "").strip().split()
    if len(note) >= 2 and note[-2] == "survey":
        return note[-1]
    return row.get("zone")


def fit_rows(rows, channel="survey", min_views=3, max_clearance_mm=None,
             zone="pickup"):
    """Collect (bearing, radius, radial, tangential) for one error channel.

    channel: "survey"    -> zone origin vs taped truth. NEEDS NO HAND MEASUREMENT
                            -- record_survey writes it with the gripper never
                            leaving home -- which since 2026-08-07 makes it the
                            cheapest and most trustworthy channel in the file.
             "open-loop" -> what the pipeline said vs where the block was.
             "nudge"     -> what the operator dialled in. USE max_clearance_mm.

    min_views rejects single-still rows (views=1, spread=0.0 exists in the
    history): one still gets no cross-view averaging and no parallax correction,
    so its position is not comparable with a fused one.

    zone keeps only survey rows for that zone ("pickup", "place", or None for
    both). IT DEFAULTS TO PICKUP AND THAT DEFAULT IS THE POINT. The two zones are
    different measurements and pooling them is not conservative, it is wrong:

      fit_zone computes  origin = mean(hit) - R(yaw) . mean(camera_zone),
      so the origin's sensitivity to fitted yaw IS |mean(camera_zone)|.

    Measured 2026-08-08 -- pickup 22.4 mm (0.39 mm/deg), place 99.3 mm
    (1.73 mm/deg). The pickup zone is the sweep's target so its views straddle
    it; the place zone is caught from wherever the pickup sweep happened to
    point, every view off to one side. 4.4x the sensitivity on a shorter yaw
    baseline, which is why the place survey is bimodal across two basins 5.9 mm
    apart while the pickup zone repeats to 0.30 mm.

    Pooled, 21 place rows carrying 4-10 mm errors at a different radius dragged
    the survey fit's kr to -157 +- 32 mm/m -- "measured" at 4.9 se, and an
    artefact of mixing two populations. Rows without a zone (every nudge and
    open-loop row) are unaffected by this filter.

    max_clearance_mm rejects nudges read from too far away. Measured 2026-08-07:
    the same pose read +4.7 mm radial eyeballed from a 25 mm fingertip clearance
    and +0.53 mm with a caliper at 8 mm, with the SIGN of the far reading opposite
    to the arm's true error. A nudge is only as good as the distance it was read
    from, so fitting them together fits the viewing angle.
    """
    getter = {"survey": survey_error, "open-loop": open_loop_error}.get(channel)
    out = []
    for row in rows:
        if zone is not None:
            in_zone = row_zone(row)
            # None means the row is not a survey row at all -- leave it alone.
            if in_zone is not None and in_zone != zone:
                continue
        if channel == "nudge":
            if not row.get("nudge_measured"):
                continue
            vector = row.get("nudge")
        else:
            vector = getter(row) if getter else None
        if vector is None:
            continue
        # AN EXACTLY ZERO ERROR IS AN ASSERTION, NOT A MEASUREMENT, and this
        # single line is the difference between fitting the survey and fitting
        # nothing. For most of the project's life every run passed the same
        # number as BOTH --zone-origin and --truth-block-world, which makes
        # survey_error identically (0, 0) by construction -- it is why
        # survey_error read "0.0 +- 0.0" for months while the origin was 28 mm
        # out. 41 of the first 68 rows are like that, and fed to a fitter they
        # are 39 structural zeros that drag every coefficient toward nothing and
        # return R^2 1.00 while measuring the identity function.
        #
        # A real survey landing on the tape to within a nanometre is not a thing
        # that happens. Only record_survey rows, and runs given an independently
        # taped truth, survive this.
        if math.hypot(vector[0], vector[1]) < 1e-9:
            continue
        views = row.get("views")
        if min_views and views is not None and views < min_views:
            continue
        if max_clearance_mm is not None:
            clearance = (row.get("constants") or {}).get("measure_clearance_mm")
            # No field at all means it predates the measurement park, i.e. it was
            # read from the 40 mm hover. Reject rather than assume.
            if clearance is None or clearance > max_clearance_mm:
                continue
        at = row.get("truth_world") or row.get("measured_world")
        split = radial_and_lateral(vector, at)
        if split is None:
            continue
        out.append((math.atan2(at[1], at[0]), math.hypot(at[0], at[1]),
                    split[0], split[1], row))
    return out


def fit_report(rows, channel="survey", **kwargs):
    """Lines fitting radial = c + kx*cos(b) + ky*sin(b) + kr*(r - r_bar).

    Reports each coefficient against its own standard error and refuses to
    recommend anything the data did not measure. See normal_equations for why
    cos/sin rather than the bearing.
    """
    samples = fit_rows(rows, channel, **kwargs)
    head = ["  %s channel: %d usable row(s)" % (channel.upper(), len(samples))]
    if len(samples) < MIN_FIT_ROWS:
        return head + [
            "    NOT ENOUGH DATA to fit -- 4 parameters need at least %d rows."
            % MIN_FIT_ROWS,
            "    This is the honest answer, not a failure. Run the sweep."]
    bearings = [math.degrees(s[0]) for s in samples]
    span = max(bearings) - min(bearings)
    radii = [s[1] for s in samples]
    r_bar = sum(radii) / len(radii)
    head.append("    bearing %+.0f..%+.0f deg (span %.0f), radius %.0f..%.0f mm"
                % (min(bearings), max(bearings), span,
                   min(radii) * 1000, max(radii) * 1000))
    if span < MIN_FIT_BEARING_SPAN_DEG:
        head.append("    BEARING SPAN TOO NARROW (%.0f < %.0f deg). cos and sin "
                    "are collinear here;" % (span, MIN_FIT_BEARING_SPAN_DEG))
        head.append("    a fit would split one constant across three "
                    "coefficients. Reporting means only:")
        for name, idx in (("radial", 2), ("tangential", 3)):
            stat = Stat(name)
            for s in samples:
                stat.add(s[idx])
            head.append("  " + stat.line())
        return head
    design = [[1.0, math.cos(s[0]), math.sin(s[0]), s[1] - r_bar]
              for s in samples]
    labels = ("constant (radial)", "kx (world +X)", "ky (world +Y)",
              "kr (per m of reach)")
    for name, idx in (("radial", 2), ("tangential", 3)):
        ys = [s[idx] for s in samples]
        fit = normal_equations(design, ys)
        head.append("")
        if fit is None:
            head.append("    %s: RANK DEFICIENT -- the regressors are collinear "
                        "in these poses." % name)
            continue
        coef, r2, ses, dof = fit
        head.append("    %s = c + kx*cos + ky*sin + kr*(r - %.3f)   R^2 %.2f, "
                    "dof %d" % (name, r_bar, r2, dof))
        for label, value, se in zip(labels, coef, ses):
            scale = 1000.0
            verdict = "measured" if abs(value) > 2 * se else "NOT measured (< 2 se)"
            head.append("      %-20s %+8.2f mm  +-%5.2f   %s"
                        % (label, value * scale,
                           se * scale if se == se else float("nan"), verdict))
        # A LOW R^2 AND A WELL-MEASURED CONSTANT ARE NOT THE SAME VERDICT, and
        # collapsing them into "do not apply it" throws away the one number the
        # sweep was run to get. R^2 asks how much of the SCATTER the model
        # explains; the standard error asks how well each coefficient is pinned.
        # A constant at 4 se inside a cloud of unmodelled scatter is a real
        # constant -- the scatter is simply something else (for the 2026-08-07
        # survey sweep, a 6 mm bimodal basin jump no smooth model can capture).
        shaped = [i for i in range(1, len(coef)) if abs(coef[i]) > 2 * ses[i]]
        const_ok = abs(coef[0]) > 2 * ses[0]
        if r2 < 0.7 and const_ok and not shaped:
            head.append("      R^2 %.2f, but the CONSTANT is measured at %.1f se "
                        "while no shape term is." % (r2, abs(coef[0] / ses[0])))
            head.append("      -> apply the constant (%+.2f mm), do NOT apply "
                        "the shape. The leftover" % (coef[0] * 1000))
            head.append("         scatter is real but is not a function of "
                        "bearing or radius.")
        elif r2 < 0.7:
            head.append("      R^2 %.2f -- the model does not explain this "
                        "channel. Do not apply it." % r2)
    return head


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

    # --- fitting -----------------------------------------------------------
    # linear_fit, ported from serial_rate_probe where it was never tested.
    fit = linear_fit([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0])
    check("linear_fit recovers a planted slope and intercept",
          fit is not None and abs(fit[0] - 2.0) < 1e-9
          and abs(fit[1] - 1.0) < 1e-9 and abs(fit[2] - 1.0) < 1e-9, repr(fit))
    check("linear_fit refuses a column with no spread",
          linear_fit([2.0, 2.0, 2.0], [1.0, 2.0, 3.0]) is None)

    # normal_equations against a model planted exactly: a 3 mm radial constant,
    # a 5 mm world +X offset, no +Y, and 20 mm per metre of reach.
    truth = (0.003, 0.005, 0.0, 0.020)
    design, ys = [], []
    for bearing_deg, radius in ((-113, 0.193), (-90, 0.178), (-45, 0.180),
                                (-16, 0.185), (0, 0.229), (18, 0.161),
                                (67, 0.193), (90, 0.178)):
        b = math.radians(bearing_deg)
        row = [1.0, math.cos(b), math.sin(b), radius - 0.187]
        design.append(row)
        ys.append(sum(c * v for c, v in zip(truth, row)))
    got = normal_equations(design, ys)
    check("normal_equations recovers a planted 4-parameter model",
          got is not None and max(abs(a - b) for a, b in zip(got[0], truth)) < 1e-9,
          repr(got[0]) if got else "None")
    check("a noiseless planted model gives R^2 = 1",
          got is not None and abs(got[1] - 1.0) < 1e-9)
    check("standard errors are ~0 on a noiseless fit",
          got is not None and max(got[2]) < 1e-6, repr(got[2]) if got else "")
    # Duplicate a column: cos and a scaled copy of cos cannot both be fitted.
    bad = [[1.0, r[1], 2.0 * r[1], r[3]] for r in design]
    check("normal_equations returns None on a rank-deficient design",
          normal_equations(bad, ys) is None)
    check("normal_equations refuses fewer rows than parameters",
          normal_equations(design[:3], ys[:3]) is None)

    # fit_report has to REFUSE things, and that is most of its value.
    # i starts at 1 so no row has an exactly-zero survey error -- a zero would be
    # dropped by the assertion filter below, which is correct behaviour and would
    # make the counts here misleading.
    narrow = []
    for i in range(1, 9):
        narrow.append(dict(zone_origin=[0.2032 + 0.001 * i, 0.0],
                           truth_world=[0.2032, 0.0], truth_zone=[0.0, 0.0],
                           views=3))
    text = "\n".join(fit_report(narrow, "survey"))
    check("fit_report refuses a bearing span that is too narrow",
          "BEARING SPAN TOO NARROW" in text, text[:120])
    check("... and falls back to reporting means instead of a model",
          "radial" in text and "kx" not in text)
    text = "\n".join(fit_report(narrow[:3], "survey"))
    check("fit_report refuses too few rows outright",
          "NOT ENOUGH DATA" in text, text[:120])

    # min_views and max_clearance_mm must actually exclude.
    one_view = [dict(r, views=1) for r in narrow]
    check("min_views rejects single-still rows",
          len(fit_rows(one_view, "survey", min_views=3)) == 0)
    check("min_views=0 keeps them",
          len(fit_rows(one_view, "survey", min_views=0)) == 8)
    nudged = [dict(nudge=[0.005, 0.0], nudge_measured=True, views=3,
                   measured_world=[0.2032, 0.0],
                   constants={"measure_clearance_mm": clear})
              for clear in (8.0, 8.0, 25.0)]
    check("max_clearance_mm rejects nudges read from too far away",
          len(fit_rows(nudged, "nudge", max_clearance_mm=10.0)) == 2)
    check("a row with no clearance field is rejected, not assumed",
          len(fit_rows([dict(nudge=[0.005, 0.0], nudge_measured=True, views=3,
                             measured_world=[0.2032, 0.0])],
                       "nudge", max_clearance_mm=10.0)) == 0)
    check("nudge_measured=False is never fitted",
          len(fit_rows([dict(nudge=[0.0, 0.0], nudge_measured=False, views=3,
                             measured_world=[0.2032, 0.0])], "nudge")) == 0)

    # The assertion filter. A row whose "truth" is a copy of its own origin has
    # a survey error of exactly zero and must never reach a fit.
    asserted = dict(zone_origin=[0.2032, 0.0], truth_world=[0.2032, 0.0],
                    truth_zone=[0.0, 0.0], views=3)
    check("an exactly-zero survey error is treated as an assertion, not data",
          len(fit_rows([asserted], "survey")) == 0)
    real_one = dict(asserted, zone_origin=[0.2035, 0.0])
    check("... but a genuine 0.3 mm survey error is kept",
          len(fit_rows([real_one], "survey")) == 1)

    # The "measured constant inside unmodelled scatter" branch, which is the
    # verdict the 2026-08-07 survey sweep actually produced and the one most
    # likely to be mis-read. Plant a 3 mm constant, no shape, and heavy scatter.
    # DETERMINISTIC, not seeded. A first attempt used random.gauss and the seed
    # happened to draw a 2.20 mm constant at 1.5 se, so the branch under test
    # never fired and the failure looked like a code bug. A selftest whose
    # verdict depends on a PRNG draw is a selftest that will fail on someone
    # else's Python.
    #
    # 3 mm constant, no shape, and a +-1.5 mm scatter that alternates around the
    # arc so it stays roughly orthogonal to cos, sin and r.
    scattered = []
    wobble = (1.5, -1.5, 1.4, -1.6, 1.6, -1.4, 1.5, -1.5, 1.4, -1.6, 1.6, -1.4)
    for (bearing_deg, radius), w in zip(
            ((-113, 0.193), (-90, 0.178), (-67, 0.193), (-45, 0.180),
             (-27, 0.170), (0, 0.229), (18, 0.161), (67, 0.193),
             (90, 0.178), (7, 0.205), (16, 0.185), (-7, 0.205)), wobble):
        b = math.radians(bearing_deg)
        rad = 0.003 + w / 1000.0
        ux, uy = math.cos(b), math.sin(b)
        x, y = radius * ux, radius * uy
        scattered.append(dict(zone_origin=[x + rad * ux, y + rad * uy],
                              truth_world=[x, y], truth_zone=[0.0, 0.0],
                              views=11))
    text = "\n".join(fit_report(scattered, "survey"))
    check("a measured constant inside unmodelled scatter says APPLY THE CONSTANT",
          "apply the constant" in text, text[-260:])
    check("... and explicitly says not to apply the shape",
          "do NOT apply" in text)

    # The real history is reported, not asserted on: it grows every session, so
    # an assertion about its contents expires. Before the 2026-08-07 sweep it
    # held 2 survey rows and was refused; it now holds 20 and fits a constant.
    real = load()
    if real:
        text = "\n".join(fit_report(real, "survey"))
        verdicts = [w for w in ("NOT ENOUGH DATA", "TOO NARROW",
                                "apply the constant", "does not explain")
                    if w in text]
        check("the real history reaches a stated verdict",
              bool(verdicts), text[:160])
        print("    (real history currently says: %s)"
              % ", ".join(verdicts) if verdicts else "")

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", default=None,
                        help="history file (default: %s)" % DEFAULT_LOG)
    parser.add_argument("--report", action="store_true",
                        help="summarise what the history says (the default)")
    parser.add_argument("--fit", action="store_true",
                        help="fit radial/tangential error against bearing and "
                             "radius: c + kx*cos + ky*sin + kr*(r-r_bar). Every "
                             "coefficient is printed against its own standard "
                             "error and anything under 2 se is reported as NOT "
                             "measured")
    parser.add_argument("--channel", default="survey",
                        choices=("survey", "open-loop", "nudge"),
                        help="which error to fit (default %(default)s). survey "
                             "needs no hand measurement at all -- record_survey "
                             "writes it with the gripper at home")
    parser.add_argument("--min-views", type=int, default=3, metavar="N",
                        help="reject rows fused from fewer than N stills "
                             "(default %(default)s); 1-view rows get no "
                             "cross-view averaging and no parallax correction")
    parser.add_argument("--max-clearance-mm", type=float, default=None,
                        metavar="MM",
                        help="for --channel nudge: reject nudges read from more "
                             "than MM of fingertip clearance above the block's "
                             "top face. Try 10. The same pose read +4.7 mm from "
                             "25 mm and +0.53 mm from 8 mm on 2026-08-07, with "
                             "opposite sign -- so pooling them fits the viewing "
                             "angle, not the arm")
    parser.add_argument("--zone", default="pickup",
                        choices=("pickup", "place", "both"),
                        help="which zone's survey rows to fit (default "
                             "%(default)s). The pickup zone repeats to 0.30 mm; "
                             "the place zone is 4.4x more yaw-sensitive because "
                             "its views all sit off to one side, and is bimodal "
                             "across two basins 5.9 mm apart. 'both' pools them "
                             "and is almost always a mistake -- it is what put "
                             "kr at -157 mm/m")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return _selftest()
    rows = load(args.log)
    if args.fit:
        print("calibration fit: %d run(s) from %s\n"
              % (len(rows), args.log or DEFAULT_LOG))
        for line in fit_report(rows, args.channel,
                               min_views=args.min_views,
                               max_clearance_mm=args.max_clearance_mm,
                               zone=None if args.zone == "both" else args.zone):
            print(line)
        return 0
    # --report is the default, and used to be dead: args.report was never read.
    return report(args.log, rows)


if __name__ == "__main__":
    sys.exit(main())
