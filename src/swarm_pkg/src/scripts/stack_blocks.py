#!/usr/bin/env python3
"""Pick two NAMED blocks out of the pickup zone and stack them in the place zone.

    python3 stack_blocks.py                          # orange, then green on it
    python3 stack_blocks.py --stack "orange block" "the green one"
    python3 stack_blocks.py --dry-run                # every pose, nothing grasped

One process, one survey of each zone, two picks, two places. The second block
lands on top of the first.

------------------------------------------------------------------------------
WHAT THIS IS BUILT ON, AND WHAT IT DELIBERATELY DOES NOT COPY
------------------------------------------------------------------------------
Every measurement and every arm move here belongs to code that already works:

    explore_pick_place  the coarse-then-fine sweep that finds BOTH zones
    tag_pick_place      the 5-still survey, tag identity, parallax correction,
                        block selection, the confirm/park/nudge protocol
    pick_place          move_arm_to, cartesian_move_to, the jaw-offset model,
                        the J1 unidirectional approach

What is NOT reused is `tag_pick_place.run_stage1`, and that is a decision rather
than an oversight. run_stage1 is one straight line -- survey, pick, release --
with three things baked into it that stacking has to change:

  1. it releases at wrist yaw 0.0 (the place moves pass no block_yaw_deg), so a
     block always lands square to the WORLD, not to the robot;
  2. its release height is `place_surface + thickness/2 + GRASP_OFFSET_Z` with
     the surface hardcoded to PLACE_XYZ.z, i.e. always the mat -- there is no
     notion of a level;
  3. it surveys, picks and places ONCE, and returns.

Forking it to add those would leave two copies of the pick logic drifting apart,
which is the failure this project has already paid for elsewhere. So this file
is a driver: it calls the same functions in a different order, and every number
it computes that run_stage1 also computes is computed by the same code.

------------------------------------------------------------------------------
FOUR DESIGN DECISIONS, AND WHY
------------------------------------------------------------------------------
**1. ONE pickup survey for BOTH blocks. This is the real speedup.**

`detect_multiview` takes 5 stills, and re-centres the flange for each one
(survey_flange_for_yaw) because the lens sits 40 mm off the flange axis. That is
by far the most expensive thing in a run -- ~5 arm moves plus 5 detect calls at
up to DETECT_SERVICE_TIMEOUT each, against 4 arm moves for an entire pick.

It only has to happen once. `identify_blocks` already returns a class PER
CONTOUR, so one survey locates the orange block and the green one together, and
lifting the orange one does not move the green one. Re-surveying between picks
would re-measure a scene that did not change.

The cost is honest and bounded: block 2's position is measured BEFORE block 1 is
removed, so if lifting block 1 disturbs block 2, the cached position is stale.
`--resurvey` forces a fresh survey per block for exactly that case. Blocks that
start touching each other should use it.

**2. "The near side faces the robot" means the block is SQUARE TO THE RADIAL
direction, and for a cube that is the only thing it can mean.**

    place_yaw = reduce_yaw(bearing of the place point, symmetry)

A cube has 4-fold symmetry, so its faces repeat every 90 deg and there is no
"which" face -- only whether a face is square-on or cornered-on. Commanding the
wrist to the place point's own bearing puts one face normal along +radial (away
from the base) and therefore the opposite face square to the robot.

Two things make this work, and both are worth stating because neither is
obvious:

  - The wrist yaw at the grasp IS the block's folded yaw
    (`grasp_yaw = reduce_yaw(block_yaw_world, symmetry)`), so the block's faces
    and the jaw axis are locked together from the moment the jaws close. Turning
    the wrist to `place_yaw` therefore turns the BLOCK to `place_yaw`, mod 90.
  - At today's bench the place zone sits at bearing -90 deg, and
    `reduce_yaw(-90 deg, 4) == 0.0` exactly -- which is what run_stage1 already
    commands by accident. So this changes nothing until the place mat moves off
    the -Y axis, and then it changes the right thing.

For the non-cubic blocks in `block_database/` this is NOT sufficient: a 2.4 x
1.2 in cuboid has 2-fold symmetry, so which face is which matters, and the
answer has to come from the decoded SIDE tag index
(`block_coordinates.block_yaw_from_side`). That is step 2's problem, and this
file refuses rather than guesses -- see `require_cube`.

**3. Stack Z is arithmetic, not a measurement, and the ceiling is level 1.**

    resting surface of level n  =  MAT_SURFACE_Z + n * block_height
    release flange z            =  surface + block_height/2 + GRASP_OFFSET_Z

At level 0 that is -0.004 + 0.015 + 0.1345 = 0.1455, which is GRASP_FLANGE_Z to
the digit -- the same height the pick side already grasps this block at, arrived
at from the other direction. That agreement is the only check available on the
formula, and it passes.

**LEVEL 2 IS BLOCKED, and by MAX_HOVER_Z rather than by reach.** Level 2 wants a
release flange z of 0.2055 m. `MAX_HOVER_Z` is 0.205, so `hover_z_for` clamps
the PRE-PLACE HOVER to 0.5 mm BELOW the release point and the descent inverts --
the arm would rise into the block it is placing. The flange itself can reach
0.2055 at the zone radius (the envelope allows ~0.2358 at r = 0.2316), so this
is a hover-ceiling limit, not a workspace one.

That is a different constraint from the one APRIL_TAGS_DEV.md tabulates. Its
"3 tall is exactly the ceiling" is about PICKING from level 2 (+7.5 mm of
envelope margin at the zone centre, negative at the corners). PLACING at level 2
fails earlier and for an unrelated reason. Both are real; this one bites first.
`--max-level` exists to raise it deliberately, and `check_stack_geometry` refuses
by default with the arithmetic printed.

**4. A systematic place error CANCELS in the stack. This is what makes stacking
possible on an uncalibrated place zone.**

The place side has never been calibrated -- APRIL_TAGS_DEV.md stage 0 retired
the problem by declaring +-25 mm acceptable for a block tossed into a 4 in box.
Stacking needs the second block's centre within roughly a third of a block of
the first, so ~10 mm, which sounds like it needs that retired calibration back.

It does not, because the stack is a RELATIVE measurement. Block 2 is commanded
to the same world XY as block 1, so any error common to both -- the place zone's
surveyed origin, the jaw-offset model at that bearing, J1's lost motion at that
angle -- displaces the whole stack together and does not tip it. The arm's
same-direction repeatability is 0.20 mm (TESTS.md Test 1, below its own 0.088 deg
readback quantum), so the common part cancels to well under a millimetre.

What does NOT cancel, in descending order of size:

  a. **The per-block grasp residual.** The two blocks sit at different points in
     the pickup zone, so each is grasped with its own residual error, and each
     block therefore sits slightly differently in the jaws. Measured pick
     residual is 0.33 mm mean / 1.0 mm worst (2026-08-11, six runs), so this is
     ~1 mm and it is the floor on stack accuracy.
  b. **Level 0 vs level 1 droop.** The two releases happen at flange z 0.1455
     and 0.1755, which is a different arm configuration and therefore a
     different gravity sag. DESCENT_BIAS_Z and the far-corner compliance term
     were both measured at level 0 and neither has been checked 30 mm higher.
     UNMEASURED. This is the one number that could exceed (a).
  c. Whether the block rotated or slipped in the jaws between grasp and release.

So the first runs of this script should keep `--confirm` on and use the `m`
command at the place park. That records the place-side open-loop error into
`calibration_history.jsonl` with `kind: "place"`, which is the measurement (b)
needs and which nothing in this project has ever taken.

------------------------------------------------------------------------------
THE POSE MEMORY -- what it does and does not buy
------------------------------------------------------------------------------
`PoseMemory` caches the COMMANDED joint target of every joint-space move against
the world pose it was asked for, and replays it when the same pose comes round
again. See the class docstring for the arithmetic. Two things it is important not
to oversell:

  - It saves IK and PLANNING latency -- up to 19 `/compute_ik` round trips plus
    an OMPL solve -- not arm motion. Arm motion plus settle time is most of a
    run, and replay does not shorten it by a millisecond.
  - Its real value is that it BYPASSES IK, which is where everything fragile in
    this project has lived. A replay cannot fail the way a fresh solve can.

The speedup that actually matters in this file is decision 1 above.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import block_coordinates as bc  # noqa: E402
import calibration  # noqa: E402
import explore  # noqa: E402
import explore_pick_place as epp  # noqa: E402
import pick_place as pp  # noqa: E402
import tag_pick_place as tpp  # noqa: E402


# ---------------------------------------------------------------------------
# Naming a block in words
# ---------------------------------------------------------------------------
# The operator says "the orange block", "orange top", "pick up the one that
# says orange". All of those have to land on the string BLOCK_CLASSES uses, and
# an ambiguous one has to REFUSE rather than choose -- picking the wrong block
# is a confident wrong answer, and select_block's own comment makes the same
# point about unidentified contours ("picking one would be a coin flip").
#
# Filler is dropped rather than matched, so this stays a whitelist of NOISE and
# the signal is whatever survives. Adding a block class needs no change here:
# the tokens come from BLOCK_CLASSES itself.
_FILLER = frozenset("""
    a an and at block blocks called cube cubes for from get grab i in is it its
    like me my named next of on one ones pick place please put says side that
    the their them then there to top up want we with
""".split())


def _tokens(text):
    """Lowercase word tokens, punctuation stripped."""
    cleaned = "".join(c if c.isalnum() or c.isspace() else " "
                      for c in text.lower())
    return [t for t in cleaned.split() if t]


def resolve_block_class(text, classes=bc.BLOCK_CLASSES):
    """'the orange block' -> 'orange_cube'. Raises ValueError on ambiguity.

    Matched against the class name's own tokens, so `orange_cube` is found by
    "orange", "cube" (if unique), "orange_cube" or "orange cube". Raising rather
    than returning None: every caller here has to stop, and a name that cannot
    be resolved is an operator error worth reading in full.
    """
    words = set(_tokens(text))
    if not words:
        raise ValueError("no block name in %r" % (text,))

    # An exact class name always wins, before any token scoring, so
    # --stack orange_cube is never at the mercy of the filler list.
    for name in classes:
        if name.lower() in (" ".join(_tokens(text)), text.strip().lower()):
            return name

    signal = words - _FILLER
    hits = []
    for name in classes:
        name_tokens = set(name.lower().split("_"))
        if signal & name_tokens:
            hits.append(name)
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise ValueError(
            "%r does not name any known block. Known classes: %s. The words "
            "that were looked at were %s (the rest is filler)."
            % (text, ", ".join(classes), sorted(signal) or "none"))
    raise ValueError(
        "%r matches %d blocks (%s) -- say which. A wrong block is picked "
        "silently and confidently." % (text, len(hits), ", ".join(hits)))


def require_cube(block_class, symmetry, allow):
    """True if this block may be placed by the radial-yaw rule. Prints why not.

    THE RULE IS ONLY VALID FOR A 4-FOLD FOOTPRINT. "Near side faces the robot"
    on a cuboid picks out ONE of its two face pairs, and a symmetry-2 fold
    cannot tell them apart -- so a 2.4 x 1.2 in block would be laid down in
    whichever of the two orientations the fold happened to land on, 90 deg
    apart. That needs the decoded SIDE tag index
    (block_coordinates.block_yaw_from_side), which is step 2's build.

    Deliberately checked against the MEASURED symmetry, not the class name:
    the footprint classifier is the thing that has historically been wrong here
    (a 30 mm square read as `circle` in 2 of 3 stills, symmetry 0), and the
    class name would happily assert "cube" over a detection that disagreed.
    """
    if symmetry == 4:
        return True
    if allow:
        print("[stack] --allow-any-symmetry: placing %s at the radial yaw even "
              "though its footprint measured symmetry %s. For anything but a "
              "square that picks one of two face pairs at random."
              % (block_class, symmetry))
        return True
    print("[stack] REFUSING to place %s: its footprint measured symmetry %s, "
          "not 4." % (block_class, symmetry))
    print("[stack] 'near side faces the robot' needs a 4-fold footprint to be "
          "well defined. On a 2-fold block the yaw fold cannot tell the long "
          "face pair from the short one, so the block would be laid down 90 deg "
          "out about half the time.")
    print("[stack] This is what the SIDE tags are for "
          "(block_coordinates.block_yaw_from_side) and it is not built yet. "
          "--allow-any-symmetry overrides if you are watching.")
    return False


# ---------------------------------------------------------------------------
# Stack geometry
# ---------------------------------------------------------------------------
# Extra height above the resting surface at which the carried block is parked
# for inspection before release. Referenced to the SURFACE IT WILL LAND ON --
# the mat at level 0, the lower block's top face at level 1 -- so the operator
# always sights the same gap whatever the level.
#
# 8 mm, matching tag_pick_place.MEASURE_CLEARANCE_M, and for the same measured
# reason: judging a lateral offset from the standard 40 mm hover was found on
# 2026-08-07 to read +4.7 mm where a caliper read +0.53, with the SIGN opposite
# to the arm's true error. The park is what makes a place-side reading worth
# recording at all.
PLACE_CLEARANCE_M = 0.008

# Highest level this file will place at without being told twice. See design
# decision 3 in the module docstring: level 2's release flange z (0.2055) is
# 0.5 mm above MAX_HOVER_Z (0.205), so hover_z_for returns a "hover" BELOW the
# target and the pre-place descent inverts.
DEFAULT_MAX_LEVEL = 1


def stack_surface_z(level, block_height=None):
    """World z of the surface level `level` rests on. Level 0 is the mat."""
    if block_height is None:
        block_height = pp.BLOCK_HEIGHT_M
    return pp.MAT_SURFACE_Z + level * block_height


def release_flange_z(level, block_height=None):
    """Flange z at which a block held in the jaws is released onto `level`.

    Same shape as run_stage1's release height, with the surface a function of
    the level instead of the constant PLACE_XYZ.z:

        surface + block_height/2 + GRASP_OFFSET_Z

    At level 0 this returns GRASP_FLANGE_Z (0.1455) -- the height the pick side
    independently grasps this block at. The two derivations agreeing is the only
    available check on this formula.
    """
    if block_height is None:
        block_height = pp.BLOCK_HEIGHT_M
    return (stack_surface_z(level, block_height) + block_height / 2.0
            + pp.GRASP_OFFSET_Z)


def check_stack_geometry(level, place_x, place_y, block_height=None,
                         max_level=DEFAULT_MAX_LEVEL, yaw_deg=None):
    """(release_z, hover_z) for a place at `level`, or (None, None) with why not.

    Checks three things that each fail differently and each read like something
    else when they do:

      1. the level is inside max_level (see DEFAULT_MAX_LEVEL);
      2. the hover is genuinely ABOVE the release, i.e. the descent is a
         descent -- this is the MAX_HOVER_Z clamp at level 2, and it presents as
         an upward "descent" into the block being placed;
      3. the release flange radius is inside the reach envelope AT ITS HEIGHT.
         The envelope shrinks with z, so a place that is fine at level 0 can be
         outside it at level 1 -- APRIL_TAGS_DEV.md tabulates -1.5 mm of margin
         at the far pickup corner for exactly this reason.

    yaw_deg defaults to the radial yaw this file would command. It is passed
    explicitly when the operator has nudged the wrist, because the yaw feeds
    compensate_for_tip_swing through JAW_PERP_OFFSET_M -- a TOOL-frame term, so
    it rotates with the wrist -- and checking the reach at a yaw other than the
    one about to be commanded is checking a different pose.
    """
    if block_height is None:
        block_height = pp.BLOCK_HEIGHT_M
    if level > max_level:
        print("[stack] level %d is above --max-level %d. Level 2 wants a "
              "release flange z of %.4f against MAX_HOVER_Z %.3f, so the "
              "pre-place hover clamps BELOW the release point and the descent "
              "inverts. Raise --max-level only if you have re-measured that."
              % (level, max_level, release_flange_z(2, block_height),
                 pp.MAX_HOVER_Z))
        return None, None

    release_z = release_flange_z(level, block_height)
    place_yaw = (radial_yaw_deg(place_x, place_y) if yaw_deg is None
                 else yaw_deg)
    hover = pp.hover_z_for(place_x, place_y, release_z, place_yaw)
    descent = hover - release_z
    if descent < pp.MIN_USEFUL_DESCENT_M:
        print("[stack] level %d: release z %.4f but the hover clamps to %.4f, "
              "leaving %.1f mm of descent (need %.0f mm). The jaws would come "
              "in from the side, or upward."
              % (level, release_z, hover, descent * 1000,
                 pp.MIN_USEFUL_DESCENT_M * 1000))
        return None, None

    # The COMPENSATED flange radius, not hypot(place_x, place_y):
    # compensate_for_tip_swing pushes the flange ~12 mm further out than the jaw
    # target, and checking the jaw radius understates it by exactly that.
    cx, cy, _cz = pp.compensate_for_tip_swing(place_x, place_y, release_z,
                                              place_yaw)
    flange_r = math.hypot(cx, cy)
    r_max = pp.max_flange_radius(release_z)
    margin = r_max - flange_r
    print("[stack] level %d: surface z %.4f, release flange z %.4f, hover "
          "%.4f (%.0f mm descent)"
          % (level, stack_surface_z(level, block_height), release_z, hover,
             descent * 1000))
    print("[stack] level %d: flange radius %.4f at that height, envelope "
          "allows %.4f -- margin %+.1f mm"
          % (level, flange_r, r_max, margin * 1000))
    if margin < 0:
        print("[stack] REFUSING: that release is outside the reach envelope. "
              "Move the place mat inward -- APRIL_TAGS_DEV.md, 'tall stacks "
              "must live near the zone centre'.")
        return None, None
    if margin < 0.005:
        print("[stack] WARNING: under 5 mm of envelope margin. IK may find it "
              "and the Cartesian retreat may not.")
    return release_z, hover


def radial_yaw_deg(x, y):
    """Wrist yaw that puts a cube's faces square to the robot at (x, y).

    The bearing of the target, folded mod 90 -- so one face normal points
    outward along +radial and the opposite one faces the base. Folding keeps
    joint6output_to_joint6 away from its -2.4434 rad limit, which
    _is_near_joint_limit() rejects outright (see reduce_yaw).
    """
    return math.degrees(tpp.reduce_yaw(math.atan2(y, x), 4))


# ---------------------------------------------------------------------------
# Pose memory
# ---------------------------------------------------------------------------
class PoseMemory:
    """Commanded joint targets, keyed by the world pose they were asked for.

    WHAT IS STORED, AND WHY IT IS THE COMMANDED ANGLES AND NOT THE ACHIEVED
    ONES. This is the whole correctness question in the idea.

    J1 loses a measured 0.94 deg (1.88 deg full backlash) of lost motion in
    whichever direction it last travelled -- 4.1 mm of arc at r = 126 mm, 7.5 mm
    at r = 229. `j1_unidirectional_approach` cancels it by re-approaching THE
    COMMANDED TARGET with a J1_RESIDUAL_BIAS_DEG overshoot, and its comment in
    move_arm_to says explicitly that re-approaching the ACHIEVED position "would
    bake in whatever backlash offset the arrival happened to leave, which is the
    thing being removed".

    So a memory built from /joint_states after the move would store the arm's
    arrival error and re-command it as a target. Replaying it would look
    beautifully repeatable -- every visit landing in the same place -- while
    sitting a backlash width from where it was asked to be, and each generation
    of replay would inherit the last one's error. That is Lesson 4 exactly:
    internal agreement is not accuracy.

    `pick_place.LAST_ARM_GOAL` exists for this. It is the final point of the
    trajectory move_arm_to planned, snapshotted before execution.

    KEYED ON THE TARGET, NOT ON A LABEL, and that is what makes persisting this
    to disk safe. The mats move between sessions, so yesterday's "pickup zone
    hover" is a different world pose today -- and a moved mat simply MISSES the
    cache, which is the safe direction. A label-keyed memory would hit, and
    drive the arm at where the mat used to be. Labels are carried for the log
    only and are never matched on.

    Tolerances are tight on purpose. 2 mm is the width at which two targets are
    the same target on an arm whose own repeatability is 0.20 mm and whose
    calibration is being held to 1-2 mm; anything looser trades the accuracy
    this project just spent a week buying for a few seconds of planning time.
    """

    def __init__(self, path=None, tol_m=0.002, tol_z_m=0.002, tol_deg=1.0,
                 enabled=True):
        self.path = path
        self.tol_m = tol_m
        self.tol_z_m = tol_z_m
        self.tol_deg = tol_deg
        self.enabled = enabled
        self.entries = []
        self.hits = 0
        self.misses = 0
        self.replays = 0
        if path:
            self.load(path)

    # -- keys ---------------------------------------------------------------
    def _matches(self, entry, x, y, z, yaw_deg, holding):
        if bool(entry.get("holding")) != bool(holding):
            return False
        if abs(entry["z"] - z) > self.tol_z_m:
            return False
        if math.hypot(entry["x"] - x, entry["y"] - y) > self.tol_m:
            return False
        # Wrapped: +179 and -179 deg are 2 deg apart, not 358. A cube's yaw is
        # folded to +-45 so this cannot bite today, but the place yaw comes from
        # a bearing and a place mat at bearing 180 would sit exactly on it.
        dyaw = (entry["yaw_deg"] - yaw_deg + 180.0) % 360.0 - 180.0
        return abs(dyaw) <= self.tol_deg

    def recall(self, x, y, z, yaw_deg, holding=False):
        """The stored entry for this pose, or None. Newest match wins."""
        if not self.enabled:
            return None
        for entry in reversed(self.entries):
            if self._matches(entry, x, y, z, yaw_deg, holding):
                self.hits += 1
                return entry
        self.misses += 1
        return None

    def remember(self, label, x, y, z, yaw_deg, holding=False):
        """Store pick_place.LAST_ARM_GOAL against this pose. Returns it or None.

        Called AFTER a successful move_arm_to. Silently does nothing when
        LAST_ARM_GOAL is empty, which happens when the move went through
        cartesian_move_to instead -- a Cartesian path has no single commanded
        joint target worth replaying, and inventing one from its last waypoint
        would store a pose the planner reached incidentally.
        """
        if not self.enabled or not pp.LAST_ARM_GOAL:
            return None
        entry = {"label": label, "x": x, "y": y, "z": z,
                 "yaw_deg": yaw_deg, "holding": bool(holding),
                 "joints": dict(pp.LAST_ARM_GOAL),
                 "time": time.time()}
        # Replace an existing entry for the same pose rather than growing a pile
        # of near-duplicates that recall() then has to walk.
        self.entries = [e for e in self.entries
                        if not self._matches(e, x, y, z, yaw_deg, holding)]
        self.entries.append(entry)
        return entry

    # -- replay -------------------------------------------------------------
    def replay(self, io_client, entry, label="replay", unidirectional=True):
        """Re-command a stored joint target. -> True/False.

        Plans a joint-space goal to the stored angles and executes it, then runs
        the same `j1_unidirectional_approach` move_arm_to would have run. The
        stored angles are the UNBIASED commanded target (the bias is applied to
        a copy inside j1_unidirectional_approach), so this reproduces the
        original two-leg arrival exactly rather than stacking a second bias on
        top of the first.

        Deliberately still goes through plan_motion. Executing a bare one-point
        trajectory would skip collision checking entirely, and this arm has
        already driven its camera into its own links once
        (STACKED_BLOCKS_GUIDE.md, 'THE POSE IS 107 49 -103 0 0 135'). Planning
        to an explicit joint goal is cheap -- what is being saved is the IK seed
        loop, up to 19 /compute_ik round trips, not the planner.
        """
        joints = entry.get("joints") or {}
        if not joints:
            return False
        trajectory = io_client.plan_motion(
            [pp.make_joint_goal_constraints(joints)])
        if trajectory is None:
            print("[memory] planning to the remembered %s FAILED -- falling "
                  "back to a fresh solve." % label)
            return False
        print("[memory] replaying the remembered %s: joint goal, no IK solve"
              % label)
        ok = io_client.arm_execute(trajectory)
        pp.record_flange_fk(io_client)
        if ok is not False and unidirectional:
            pp.j1_unidirectional_approach(io_client, trajectory)
        self.replays += 1
        return ok is not False

    # -- persistence --------------------------------------------------------
    def load(self, path):
        """Read a saved memory. A missing or corrupt file is not an error."""
        try:
            with open(path) as handle:
                data = json.load(handle)
        except (IOError, OSError):
            return
        except ValueError as exc:
            print("[memory] %s is not readable JSON (%s) -- starting empty."
                  % (path, exc))
            return
        entries = data.get("entries") if isinstance(data, dict) else None
        if not isinstance(entries, list):
            print("[memory] %s has no entries list -- starting empty." % path)
            return
        good = [e for e in entries
                if isinstance(e, dict) and isinstance(e.get("joints"), dict)
                and all(k in e for k in ("x", "y", "z", "yaw_deg"))]
        if len(good) != len(entries):
            print("[memory] %d of %d entries in %s were malformed and dropped."
                  % (len(entries) - len(good), len(entries), path))
        self.entries = good
        print("[memory] loaded %d pose(s) from %s. Keyed on the world target, "
              "so any that belong to a mat that has since moved will simply "
              "miss." % (len(good), path))

    def save(self, path=None):
        path = path or self.path
        if not path or not self.enabled:
            return
        try:
            directory = os.path.dirname(os.path.abspath(path))
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            with open(path, "w") as handle:
                json.dump({"schema": 1, "saved": time.time(),
                           "entries": self.entries}, handle, indent=1,
                          sort_keys=True)
        except Exception as exc:                            # noqa: BLE001
            print("[memory] could NOT write %s (%s: %s) -- the run is "
                  "unaffected, the memory is lost"
                  % (path, type(exc).__name__, exc))

    def report(self):
        looked = self.hits + self.misses
        print("[memory] %d pose(s) held; %d hit / %d miss over %d lookup(s), "
              "%d replayed. Replay saves IK and planning latency, not arm "
              "motion." % (len(self.entries), self.hits, self.misses, looked,
                           self.replays))


# ---------------------------------------------------------------------------
# Moves, with the memory in front of them
# ---------------------------------------------------------------------------
def go_to(io_client, memory, label, x, y, z, yaw_deg=0.0, holding=False,
          unidirectional=True):
    """move_arm_to, replayed from memory when this pose has been solved before.

    Falls THROUGH to a fresh solve when the replay does not converge, rather
    than failing: a stale or unreachable memory must never be worse than not
    having one. The fresh solve then overwrites the entry.
    """
    entry = memory.recall(x, y, z, yaw_deg, holding)
    if entry is not None:
        if memory.replay(io_client, entry, label, unidirectional):
            return True
        print("[memory] the replay did not converge; solving %s fresh." % label)
    ok = pp.move_arm_to(io_client, x, y, z, block_yaw_deg=yaw_deg,
                        holding_block=holding, unidirectional=unidirectional)
    if ok is not False:
        memory.remember(label, x, y, z, yaw_deg, holding)
    return ok is not False


# ---------------------------------------------------------------------------
# The operator checkpoint, shared by the pick and the place
# ---------------------------------------------------------------------------
def confirm_at_park(io_client, memory, target, target_z, park_z, yaw_deg, what,
                    holding=False, unidirectional=True):
    """Approach, lower to `park_z`, let the operator correct it, -> dict.

    Returns {"ok", "x", "y", "yaw_deg", "nudges", "measurements"}, with x/y/yaw
    carrying whatever the operator dialled in.

    TWO HEIGHTS, NOT ONE, and the split matters. The APPROACH hover is
    `hover_z_for(x, y, target_z)` -- computed from the grasp or release height,
    which is the number the pick path has been validated at. `park_z` is then a
    separate straight-down lower for the operator's benefit. Deriving the hover
    from the park instead would silently raise the approach by the clearance and
    lengthen the final descent, changing a move that is already calibrated.

    THE PROTOCOL IS tag_pick_place's, verbatim in behaviour, because it has
    already been debugged on hardware and the bugs were not obvious:

      - 'm dx dy' RECORDS and moves nothing; 'dx dy' moves. A nudge is a control
        action contaminated by J1's dead band; a reading is a measurement.
        Fitting the nudge column fits the dead band.
      - The sign convention is the CORRECTION, not the error -- the same numbers
        you would type as a nudge. Any fit on these must negate them.
      - THE INNER LOOP IS LOAD-BEARING. When 'm' was first added to
        tag_pick_place it ended its branch with `continue`, which jumped to the
        OUTER loop and re-parked the arm while printing "moves NOTHING". So did
        every typo. Caught on hardware 2026-08-11 at position N: two 'm 0 5'
        readings walked the gaps 3.86/6.00 -> 6.00/3.00 -> 3.86/6.00, i.e. the
        arm alternated between two poses 2.6 mm apart while the operator was
        told nothing had moved. Only a real nudge or ENTER leaves the inner loop
        here.
    """
    x, y = target
    nudges, measurements = [], []
    while True:
        print("\n=== Move over the %s ===" % what)
        hover = pp.hover_z_for(x, y, target_z, yaw_deg)
        if not go_to(io_client, memory, what, x, y, hover, yaw_deg, holding,
                     unidirectional):
            return {"ok": False, "x": x, "y": y, "yaw_deg": yaw_deg,
                    "nudges": nudges, "measurements": measurements}
        here = hover
        if park_z is not None and park_z < hover - 1e-4:
            print("\n=== Lower to the measurement clearance ===")
            # BEST EFFORT, as in tag_pick_place: a failed lower leaves the arm
            # at the hover, which is a worse view rather than a broken run.
            if pp.cartesian_move_to(io_client, x, y, park_z,
                                    block_yaw_deg=yaw_deg,
                                    holding_block=holding) is False:
                print("[confirm] could not lower to the clearance; measuring "
                      "from the hover instead.")
            else:
                here = park_z

        print("\n[confirm] %s target: world (%.4f, %.4f) yaw %+.1f deg, parked "
              "at flange z %.4f, %.0f mm above the %s height"
              % (what, x, y, yaw_deg, here, (here - target_z) * 1000,
                 "release" if holding else "grasp"))
        if nudges:
            total = (sum(n["dx_mm"] for n in nudges),
                     sum(n["dy_mm"] for n in nudges))
            print("[confirm] correction dialled in so far: (%+.1f, %+.1f) mm "
                  "over %d step(s)" % (total[0], total[1], len(nudges)))
        print("[confirm] LOOK AT THE JAWS.")
        print("[confirm]   'm dx dy' RECORDS a reading and moves NOTHING. Do "
              "this FIRST -- the first one is the open-loop error at this pose.")
        print("[confirm]   'dx dy' mm nudges in WORLD axes and re-parks; "
              "'dx dy dyaw' also turns the wrist.")
        print("[confirm]   sign: give 'm' the SAME numbers you would type as a "
              "nudge (the correction), not the error.")

        action = None
        while True:
            answer = tpp._ask(
                "[confirm] ENTER = go   'dx dy' = nudge   'm dx dy' = record "
                "  q = abort > ")
            if answer in ("q", "quit", "n", "no"):
                return {"ok": False, "x": x, "y": y, "yaw_deg": yaw_deg,
                        "nudges": nudges, "measurements": measurements}
            if (answer[:1] in ("m", "M")
                    and tpp._parse_nudge(answer[1:]) is not None):
                seen = tpp._parse_nudge(answer[1:])
                measurements.append({"dx_mm": seen[0] * 1000,
                                     "dy_mm": seen[1] * 1000,
                                     "after_nudges": len(nudges)})
                print("[confirm] recorded (%+.2f, %+.2f) mm after %d nudge(s). "
                      "THE ARM HAS NOT MOVED."
                      % (seen[0] * 1000, seen[1] * 1000, len(nudges)))
                if len(measurements) == 1:
                    print("[confirm] that is the OPEN-LOOP error at this pose.")
                continue
            nudge = tpp._parse_nudge(answer)
            if nudge is not None:
                x += nudge[0]
                y += nudge[1]
                yaw_deg += nudge[2]
                nudges.append({"dx_mm": nudge[0] * 1000,
                               "dy_mm": nudge[1] * 1000,
                               "dyaw_deg": nudge[2], "at_park": True})
                print("[confirm] re-parking")
                action = "nudge"
                break
            if answer == "":
                action = "go"
                break
            print("[confirm] did not understand %r. Nothing moved." % answer)
        if action == "go":
            return {"ok": True, "x": x, "y": y, "yaw_deg": yaw_deg,
                    "nudges": nudges, "measurements": measurements}


# ---------------------------------------------------------------------------
# Survey
# ---------------------------------------------------------------------------
def survey_pickup_blocks(io_client, detector, args):
    """One 5-still survey of the pickup zone -> [(FusedDetection, class), ...].

    None with the reason printed on failure. This is tag_pick_place's survey,
    called directly: same stills, same fusion, same furniture drop, same
    identification. See the module docstring, decision 1, for why it is called
    ONCE for a whole stack.

    PAIRS, not a list plus an index map. identify_blocks keys its answer on
    positions in the fused list, and this file removes blocks from that list as
    it picks them -- which renumbers every key after the one removed. Carrying
    the class alongside its own detection makes that impossible to get wrong;
    the index map is rebuilt from the pairs at each select_block call.

    UNIDENTIFIED CONTOURS ARE DROPPED. This script picks by name, so a contour
    with no decoded top tag is not a candidate for anything -- and keeping it
    would let select_block's "single view, unverified but not contradicted"
    rank pick up a tape edge.
    """
    hover = tpp.DETECT_HOVER_Z
    flange = tpp.survey_flange_for_yaw(
        detector, tpp.MULTIVIEW_YAW_OFFSETS_DEG[0], hover)
    print("[stack] pickup survey: %d stills at hover %.3f, flange re-centred "
          "per wrist yaw" % (len(tpp.MULTIVIEW_YAW_OFFSETS_DEG), hover))
    debug_prefix = (os.path.splitext(args.debug_image)[0]
                    if args.debug_image else None)
    fused, views_used, _tags, block_tags = tpp.detect_multiview(
        io_client, detector, "pickup", flange[0], flange[1], hover, 0.0,
        debug_prefix=debug_prefix)
    if views_used == 0:
        print("[stack] no still saw enough tags to fuse. Framing, focus or "
              "lighting -- look at the debug frames before the geometry.")
        return None
    if not fused:
        print("[stack] pickup zone is empty.")
        return None
    fused = tpp.drop_zone_furniture(fused, args.zone_size, args.tag_size)
    if not fused:
        print("[stack] every candidate was the zone's own tags.")
        return None
    identity = tpp.identify_blocks(fused, block_tags)
    if not identity:
        print("[stack] NOTHING was identified -- no block top tag decoded. "
              "This script picks blocks BY NAME, so there is nothing it can "
              "act on. block_detector_node.py logs px/module for every decode.")
        return None
    candidates = [(det, identity[i]) for i, det in enumerate(fused)
                  if i in identity]
    dropped = len(fused) - len(candidates)
    if dropped:
        print("[stack] %d contour(s) carried no block tag and are not "
              "candidates. A block whose top tag did not decode looks exactly "
              "like the other one from above." % dropped)
    return candidates


def have_every_block(candidates, wanted):
    """True if every name in `wanted` was identified. Prints the tally.

    Checked BEFORE the first block is lifted, so a missing green block is not
    discovered with the orange one already in the stack and the jaws empty over
    a zone that has nothing left to give.
    """
    tally = {}
    for _det, klass in candidates:
        tally[klass] = tally.get(klass, 0) + 1
    print("[stack] identified: %s"
          % (", ".join("%s x%d" % (k, n) for k, n in sorted(tally.items()))
             or "nothing"))
    missing = [k for k in wanted if k not in tally]
    if missing:
        print("[stack] REFUSING: %s not found in the pickup zone. Check the "
              "tag is stuck on, facing up and lit -- block_detector_node.py "
              "logs px/module for every decode." % ", ".join(missing))
        return False
    return True


# ---------------------------------------------------------------------------
# One pick
# ---------------------------------------------------------------------------
def pick_block(io_client, detector, args, memory, block, want_class):
    """Grasp `block`. -> dict with the pose it was grasped at, or None.

    Everything here is run_stage1's arithmetic, in the same order, with the
    confirm loop factored out so the place side can use it too.
    """
    block_yaw_world = block.zyaw + detector.zone_yaw
    grasp_yaw_deg = math.degrees(tpp.reduce_yaw(block_yaw_world,
                                                block.symmetry))
    grasp_x, grasp_y = detector.zone_to_world(block.zx, block.zy)
    grasp_z = args.grasp_z

    print("\n[stack] %s: %.1f x %.1f mm %s, zone (%+.1f, %+.1f) mm, world "
          "(%.4f, %.4f), yaw %+.1f -> grasp yaw %+.1f (symmetry %d), "
          "views=%d spread=%.1f mm"
          % (want_class, block.width * 1000, block.length * 1000, block.shape,
             block.zx * 1000, block.zy * 1000, grasp_x, grasp_y,
             math.degrees(block_yaw_world), grasp_yaw_deg, block.symmetry,
             block.n_views, block.spread_m * 1000))

    # The footprint check tag_pick_place added on 2026-08-11, repeated here
    # because a mis-sized footprint is the failure that displaces a centroid and
    # this file trusts that centroid twice -- once to grasp and once, via the
    # release, to stack on.
    for axis, measured in (("width", block.width), ("length", block.length)):
        if abs(measured - tpp.BLOCK_NOMINAL_M) > tpp.BLOCK_SIZE_WARN_M:
            print("[stack] WARNING: measured %s %.1f mm against a nominal "
                  "%.1f mm. A footprint that reads N mm too long displaces its "
                  "own centroid by N/2, and that error is carried into the "
                  "stack." % (axis, measured * 1000,
                              tpp.BLOCK_NOMINAL_M * 1000))

    grasp_hover = pp.hover_z_for(grasp_x, grasp_y, grasp_z, grasp_yaw_deg)
    nudges, measurements = [], []

    if args.confirm and not args.dry_run:
        # measure_park_z returns None when the hover is already at or below the
        # clearance -- at a far corner "lowering" to it would be a LIFT, which
        # re-arms the very J1 slack the unidirectional approach just settled.
        result = confirm_at_park(
            io_client, memory, (grasp_x, grasp_y), grasp_z,
            tpp.measure_park_z(grasp_hover), grasp_yaw_deg,
            "%s block" % want_class, holding=False)
        if not result["ok"]:
            print("[stack] aborted at the pick park. Nothing descended.")
            return None
        grasp_x, grasp_y = result["x"], result["y"]
        grasp_yaw_deg = result["yaw_deg"]
        nudges, measurements = result["nudges"], result["measurements"]
        grasp_hover = pp.hover_z_for(grasp_x, grasp_y, grasp_z, grasp_yaw_deg)
    else:
        print("\n=== Move over the %s block ===" % want_class)
        if not go_to(io_client, memory, "%s block" % want_class,
                     grasp_x, grasp_y, grasp_hover, grasp_yaw_deg):
            return None

    pose = {"block_class": want_class, "x": grasp_x, "y": grasp_y,
            "z": grasp_z, "yaw_deg": grasp_yaw_deg, "hover_z": grasp_hover,
            "zone": [block.zx, block.zy], "views": int(block.n_views),
            "spread_m": float(block.spread_m), "symmetry": block.symmetry,
            "nudges": nudges, "measurements": measurements,
            "flange_fk": None, "grasped": False}

    if args.dry_run:
        print("\n[stack] --dry-run: parked over the %s block. No descent, no "
              "grasp." % want_class)
        return pose

    steps = [
        ("Descend to grasp",
         lambda: pp.cartesian_move_to(io_client, grasp_x, grasp_y, grasp_z,
                                      block_yaw_deg=grasp_yaw_deg)),
        ("Close gripper",
         lambda: pp.gripper_close_until_contact(io_client)),
    ]
    for name, action in steps:
        print("\n=== %s ===" % name)
        if not action():
            print("[stack] step FAILED: %s" % name)
            return None
        if name == "Descend to grasp" and len(pp.LAST_FLANGE_FK) >= 2:
            # Snapshot AT THE GRASP. The retreat and both place moves come
            # after this, and the first row ever written with a late snapshot
            # recorded the PLACE flange and made jaw_offset() report a 213 mm
            # offset. The grasp is the only pose where the jaws are known to be
            # on the block.
            pose["flange_fk"] = list(pp.LAST_FLANGE_FK)
        time.sleep(0.5)

    print("\n=== Retreat after grasp ===")
    if pp.cartesian_move_to(io_client, grasp_x, grasp_y, grasp_hover,
                            allow_fallback=True, block_yaw_deg=grasp_yaw_deg,
                            holding_block=True) is False:
        print("[stack] step FAILED: Retreat after grasp")
        return None

    if args.confirm:
        pose["grasped"] = tpp._ask(
            "\n[confirm] did the jaws actually close on the %s block? y = yes, "
            "anything else = no > " % want_class) in ("y", "yes")
        if not pose["grasped"]:
            print("[stack] the grasp was not confirmed. Stopping before the "
                  "place -- releasing nothing at the stack would leave a level "
                  "that the next block is then stacked onto.")
            return None
    else:
        # ROS logs are not evidence of motion: arm_group_controller reports
        # "Goal reached, success!" from elapsed time alone. --yes buys speed by
        # giving up the one check that a block is in the jaws.
        print("[stack] --yes: nothing confirms the grasp physically. Recording "
              "it as UNCONFIRMED.")
    return pose


# ---------------------------------------------------------------------------
# One place
# ---------------------------------------------------------------------------
def place_block(io_client, args, memory, place_x, place_y, level, pick_pose):
    """Release the held block onto `level` at (place_x, place_y). -> dict/None.

    holding_block=True on every move. That is a NO-OP at today's constants --
    SAG_PRECOMP_PAYLOAD_RADIAL_DEG and _TANGENTIAL_DEG are both 0.0, so
    sag_precomp_angles returns the same pair either way -- and it is passed
    anyway so that measuring them later fixes this path for free rather than
    leaving it as a second place to remember.
    """
    release_z, place_hover = check_stack_geometry(
        level, place_x, place_y, args.block_thickness, args.max_level)
    if release_z is None:
        return None
    place_yaw_deg = radial_yaw_deg(place_x, place_y)
    print("[stack] place yaw %+.1f deg: the bearing to (%.4f, %.4f) folded mod "
          "90, so a face points outward and the NEAR side faces the robot."
          % (place_yaw_deg, place_x, place_y))

    nudges, measurements = [], []
    park_z = release_z + PLACE_CLEARANCE_M

    if args.confirm and not args.dry_run:
        result = confirm_at_park(
            io_client, memory, (place_x, place_y), release_z, park_z,
            place_yaw_deg, "stack level %d" % level, holding=True)
        if not result["ok"]:
            print("[stack] aborted at the place park. STILL HOLDING THE BLOCK "
                  "-- it has not been released.")
            return None
        place_x, place_y = result["x"], result["y"]
        place_yaw_deg = result["yaw_deg"]
        nudges, measurements = result["nudges"], result["measurements"]
        # The target moved, so re-check the geometry against the new radius AND
        # the nudged yaw rather than reusing a hover computed for the old pose.
        release_z, place_hover = check_stack_geometry(
            level, place_x, place_y, args.block_thickness, args.max_level,
            yaw_deg=place_yaw_deg)
        if release_z is None:
            print("[stack] the nudge moved the release outside what is "
                  "reachable. STILL HOLDING THE BLOCK.")
            return None
    else:
        print("\n=== Move to pre-place, level %d ===" % level)
        if not go_to(io_client, memory, "stack level %d" % level,
                     place_x, place_y, place_hover, place_yaw_deg,
                     holding=True):
            return None

    # "ok" is separate from "released" ON PURPOSE. A retreat that fails AFTER
    # the jaws opened leaves a placed block and a stopped run, and those are two
    # different facts -- the caller has to record the first and act on the
    # second. Returning None for both would throw away the place-side reading
    # that had already been taken, which is the one number this run exists to
    # produce.
    placed = {"level": level, "x": place_x, "y": place_y, "z": release_z,
              "yaw_deg": place_yaw_deg, "hover_z": place_hover,
              "surface_z": stack_surface_z(level, args.block_thickness),
              "nudges": nudges, "measurements": measurements,
              "released": False, "ok": True}

    if args.dry_run:
        print("\n[stack] --dry-run: parked over stack level %d. Nothing "
              "released." % level)
        return placed

    steps = [
        ("Descend to place",
         lambda: pp.cartesian_move_to(io_client, place_x, place_y, release_z,
                                      block_yaw_deg=place_yaw_deg,
                                      holding_block=True)),
        ("Open gripper (release)",
         lambda: io_client.gripper_move_to(pp.GRIPPER_OPEN)),
        ("Retreat after release",
         lambda: pp.cartesian_move_to(io_client, place_x, place_y, place_hover,
                                      allow_fallback=True,
                                      block_yaw_deg=place_yaw_deg)),
    ]
    for name, action in steps:
        print("\n=== %s ===" % name)
        if not action():
            print("[stack] step FAILED: %s" % name)
            print("[stack] STOPPING. The block was %s. Look at the bench before "
                  "running anything else -- nothing further will be stacked on "
                  "a level that may be empty."
                  % ("RELEASED, so it is on the stack or beside it"
                     if placed["released"] else "NOT released, so it is still "
                     "in the jaws"))
            placed["ok"] = False
            return placed
        if name == "Open gripper (release)":
            placed["released"] = True
        time.sleep(0.5)

    if args.confirm:
        placed["stacked"] = tpp._ask(
            "\n[confirm] is the block sitting squarely on level %d? y = yes, "
            "anything else = no > " % level) in ("y", "yes")
        if not placed["stacked"]:
            print("[stack] the placement was not confirmed. Nothing further "
                  "will be stacked on it.")
    return placed


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------
def record_stack_row(args, pick_pose, placed, place_origin, level):
    """Append one row per PLACE. Never raises -- calibration.record handles that.

    Split from stack_row() so the selftest can check the row's SHAPE without a
    log file, which is what makes the "inert to every existing statistic" claim
    below a test rather than a comment.
    """
    calibration.record(args.calibration_log,
                       **stack_row(args, pick_pose, placed, place_origin,
                                   level))


def stack_row(args, pick_pose, placed, place_origin, level):
    """The row dict for one place, tagged `kind: "place"`.

    DELIBERATELY DOES NOT WRITE THE PICK-SIDE FIELD NAMES. `nudge`,
    `measured_zone`, `open_loop_offset` and friends are what calibration.py's
    statistics read, and every one of them uses `.get()` -- so a place row that
    filled them in would be pooled straight into the pick-side means without
    anything saying so. That is exactly how the first 61 rows became
    un-poolable, one week ago.

    So the place-side numbers travel under their own names and `kind` says which
    kind of row this is. calibration.py has no `kind` filter yet: until it does,
    these rows are inert to every existing statistic, which is the safe state.
    """
    place_measured = placed.get("measurements") or []
    return dict(
        kind="place",
        stack_level=level,
        # Where the place zone was surveyed to, against where the block was
        # actually commanded -- the second differs from the first by whatever
        # the operator dialled in at the park.
        place_zone_origin=[place_origin[0], place_origin[1]],
        place_commanded_world=[placed["x"], placed["y"]],
        place_yaw_deg=placed["yaw_deg"],
        place_release_z=placed["z"],
        place_surface_z=placed["surface_z"],
        place_radius_m=math.hypot(placed["x"], placed["y"]),
        place_nudge_steps=list(placed.get("nudges") or []),
        place_measured_offsets=list(place_measured),
        # THE PLACE-SIDE OPEN-LOOP ERROR, in metres to match every other vector
        # in the schema, and only when a reading was actually taken at the park.
        # SIGN: this is the CORRECTION the operator read, so a fit must negate
        # it -- the same convention as the pick side's open_loop_offset.
        place_open_loop_offset=([place_measured[0]["dx_mm"] / 1000.0,
                                 place_measured[0]["dy_mm"] / 1000.0]
                                if place_measured else None),
        place_measured_flag=bool(args.confirm and not args.dry_run),
        released=bool(placed.get("released")),
        stacked=placed.get("stacked"),
        # The pick that supplied the block, so a place error can be attributed
        # to the grasp it inherited rather than to the place pose.
        pick_block_class=pick_pose.get("block_class"),
        pick_commanded_world=[pick_pose["x"], pick_pose["y"]],
        pick_zone=list(pick_pose.get("zone") or []),
        pick_grasp_yaw_deg=pick_pose.get("yaw_deg"),
        pick_nudge_steps=list(pick_pose.get("nudges") or []),
        pick_measured_offsets=list(pick_pose.get("measurements") or []),
        pick_flange_fk=pick_pose.get("flange_fk"),
        pick_views=pick_pose.get("views"),
        pick_view_spread_m=pick_pose.get("spread_m"),
        grasped=bool(pick_pose.get("grasped")),
        constants=tpp.model_provenance(args),
        note=args.note or "")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        description="Pick two named blocks and stack them in the place zone.",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument("--stack", nargs="+", default=["orange", "green"],
                        metavar="NAME",
                        help="blocks to stack, BOTTOM FIRST. Plain English is "
                             "fine: 'orange block' 'the green one'. Default: "
                             "%(default)s")
    parser.add_argument("--max-level", type=int, default=DEFAULT_MAX_LEVEL,
                        help="highest stack level to place at (default "
                             "%(default)s). Level 2 needs a release flange z of "
                             "0.2055 against MAX_HOVER_Z 0.205, so its "
                             "pre-place hover clamps BELOW the release point")
    parser.add_argument("--allow-any-symmetry", action="store_true",
                        help="place a block whose footprint did not measure "
                             "symmetry 4. 'Near side faces the robot' is only "
                             "well defined on a square footprint")
    parser.add_argument("--resurvey", action="store_true",
                        help="survey the pickup zone again before EVERY pick. "
                             "The default surveys once -- lifting one block "
                             "does not move another. Use this when blocks start "
                             "touching")

    # --- sweep, same names and defaults as explore.py and explore_pick_place ---
    parser.add_argument("--start", type=float, default=explore.J1_START_DEG)
    parser.add_argument("--end", type=float, default=explore.J1_END_DEG)
    parser.add_argument("--step", type=float,
                        default=explore.J1_COARSE_STEP_DEG)
    parser.add_argument("--fine-step", type=float,
                        default=explore.J1_FINE_STEP_DEG)
    parser.add_argument("--fine-span", type=float,
                        default=explore.J1_FINE_SPAN_DEG)
    parser.add_argument("--coarse-patience", type=int, default=2, metavar="N")
    parser.add_argument("--single-pass", action="store_true",
                        help="skip the fine arcs -- one coarse sweep only. "
                             "Faster and a worse fit")
    parser.add_argument("--pitch", type=float, default=explore.EXPLORE_PITCH_DEG)
    parser.add_argument("--wrist", type=float, default=explore.EXPLORE_WRIST_DEG)
    parser.add_argument("--settle", type=float, default=explore.SETTLE_SECONDS)
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--zone-yaw", type=float, default=None,
                        help="fix BOTH zones' yaw instead of solving each from "
                             "its sweep. Rarely wanted")
    parser.add_argument("--truth-pickup", type=float, nargs=2,
                        metavar=("X", "Y"), default=None,
                        help="the pickup zone centre's TAPED world position")
    parser.add_argument("--truth-place", type=float, nargs=2,
                        metavar=("X", "Y"), default=None,
                        help="the place zone centre's TAPED world position. "
                             "This is the ONLY way the place survey's error is "
                             "ever measured -- nothing else in the project "
                             "records it")
    parser.add_argument("--place-at", type=float, nargs=2, metavar=("X", "Y"),
                        default=None,
                        help="stack at this world XY and do not survey the "
                             "place zone at all")
    parser.add_argument("--survey-only", action="store_true",
                        help="find both zones and the blocks, print the whole "
                             "plan, and stop. Nothing is grasped")

    # --- zone / block geometry, names matching tag_pick_place -----------------
    parser.add_argument("--zone-size", type=float,
                        default=tpp.zv.DEFAULT_ZONE_SIZE)
    parser.add_argument("--tag-size", type=float,
                        default=tpp.zv.DEFAULT_TAG_SIZE)
    parser.add_argument("--block-thickness", type=float,
                        default=tpp.DEFAULT_BLOCK_THICKNESS,
                        help="block height, metres (default %(default)s). Sets "
                             "BOTH the release height and the per-level step, "
                             "so it is the one number a stack's Z depends on")
    parser.add_argument("--grasp-z", type=float, default=tpp.GRASP_FLANGE_Z,
                        help="FLANGE z to descend to for a grasp, metres "
                             "(default %(default).4f). Level 0 only -- picking "
                             "FROM a stack is not built")

    # --- memory ---------------------------------------------------------------
    parser.add_argument("--memory", default=None, metavar="PATH",
                        help="load and save the pose memory here, so it "
                             "survives between runs. Keyed on the world "
                             "target, so a mat that has moved simply misses")
    parser.add_argument("--no-memory", dest="memory_enabled",
                        action="store_false", default=True,
                        help="solve every pose from scratch. Use this to "
                             "measure what the memory is worth, or when the "
                             "bench has changed under it")
    parser.add_argument("--memory-tol-mm", type=float, default=2.0,
                        help="how close two targets must be to count as the "
                             "same pose (default %(default)s mm). Tight on "
                             "purpose: the arm's own repeatability is 0.20 mm "
                             "and the calibration is held to 1-2 mm")

    # --- run control ----------------------------------------------------------
    parser.add_argument("--dry-run", action="store_true",
                        help="every pose computed and parked at, nothing "
                             "grasped and nothing released")
    parser.add_argument("--yes", dest="confirm", action="store_false",
                        default=True,
                        help="no operator checkpoints. Gives up the place-side "
                             "measurement, which is the one number this script "
                             "can take that nothing else can")
    parser.add_argument("--debug-image", default=None, metavar="PATH",
                        help="write the survey's annotated frames, one per "
                             "still, prefixed from this path")
    parser.add_argument("--calibration-log", default=None)
    parser.add_argument("--note", default="stack_blocks")
    # Listed so --help mentions it. main() intercepts it from sys.argv BEFORE
    # rclpy.init(), so the selftest needs no ROS, no robot and no network.
    parser.add_argument("--selftest", action="store_true",
                        help="run the offline checks and exit: the name "
                             "resolver, the stack arithmetic, the level-2 "
                             "refusal, the yaw fold and the pose memory. No "
                             "ROS and no arm")
    return parser


# ---------------------------------------------------------------------------
# Offline selftest
# ---------------------------------------------------------------------------
def _selftest():
    """No ROS, no arm. Every claim in this file that is arithmetic or logic.

    Written because the two things most likely to be wrong here cannot be seen
    at the bench: a name resolver that quietly picks the wrong block, and a
    stack height that is half a millimetre out per level. Both produce a run
    that looks fine.
    """
    failures = []

    def check(name, ok, detail=""):
        print("  %-58s %s%s" % (name, "ok" if ok else "FAIL",
                                "" if ok else "  " + detail))
        if not ok:
            failures.append(name)

    print("naming")
    for phrase, expect in (
            ("orange", "orange_cube"),
            ("orange top", "orange_cube"),
            ("the orange block", "orange_cube"),
            ("pick up the orange block", "orange_cube"),
            ("orange_cube", "orange_cube"),
            ("orange cube", "orange_cube"),
            ("the block named green", "green_cube"),
            ("place the green one on top", "green_cube"),
            ("GREEN", "green_cube")):
        try:
            got = resolve_block_class(phrase)
        except ValueError as exc:
            got = "raised: %s" % exc
        check("%r -> %s" % (phrase, expect), got == expect, str(got))

    def refuses(phrase):
        try:
            resolve_block_class(phrase)
        except ValueError:
            return True
        return False

    # "cube" alone is in the filler list, so it never reaches the matcher and
    # the phrase resolves to nothing rather than to whichever cube came first.
    check("'cube' alone is refused, not guessed", refuses("cube"))
    check("'the block' is refused", refuses("the block"))
    check("'purple' is refused", refuses("purple"))
    check("'' is refused", refuses(""))
    check("an ambiguous phrase naming both is refused",
          refuses("the orange and green blocks"))

    print("stack geometry")
    check("level 0 release == GRASP_FLANGE_Z, the height the pick side uses",
          abs(release_flange_z(0) - tpp.GRASP_FLANGE_Z) < 1e-12,
          "%.6f vs %.6f" % (release_flange_z(0), tpp.GRASP_FLANGE_Z))
    check("one level == one block height",
          abs((release_flange_z(1) - release_flange_z(0))
              - pp.BLOCK_HEIGHT_M) < 1e-12)
    check("level 0 rests on the mat",
          abs(stack_surface_z(0) - pp.MAT_SURFACE_Z) < 1e-12)
    check("level 1 rests on the level-0 block's top face",
          abs(stack_surface_z(1)
              - (pp.MAT_SURFACE_Z + pp.BLOCK_HEIGHT_M)) < 1e-12)
    # THE FINDING THIS FILE'S DEFAULT MAX LEVEL IS BUILT ON.
    check("level 2's release is ABOVE MAX_HOVER_Z (why level 2 is refused)",
          release_flange_z(2) > pp.MAX_HOVER_Z,
          "%.4f vs %.4f" % (release_flange_z(2), pp.MAX_HOVER_Z))
    check("a thinner block would make level 2 legal",
          release_flange_z(2, 0.025) < pp.MAX_HOVER_Z,
          "%.4f" % release_flange_z(2, 0.025))

    print("place yaw")
    check("place zone on -Y folds to yaw 0",
          abs(radial_yaw_deg(0.0, -pp.ZONE_RADIUS_M)) < 1e-9,
          "%.6f" % radial_yaw_deg(0.0, -pp.ZONE_RADIUS_M))
    check("pickup zone on +Y also folds to yaw 0",
          abs(radial_yaw_deg(0.0, +pp.ZONE_RADIUS_M)) < 1e-9)
    check("bearing +30 gives yaw +30",
          abs(radial_yaw_deg(math.cos(math.radians(30)),
                             math.sin(math.radians(30))) - 30.0) < 1e-9)
    check("bearing +60 folds to -30, not +60 (mod 90, nearest)",
          abs(radial_yaw_deg(math.cos(math.radians(60)),
                             math.sin(math.radians(60))) + 30.0) < 1e-9)
    worst = max(abs(radial_yaw_deg(math.cos(math.radians(b)),
                                   math.sin(math.radians(b))))
                for b in range(-180, 181))
    check("the fold bounds every bearing to +-45 deg (joint6output limit)",
          worst <= 45.0 + 1e-9, "%.3f" % worst)

    print("geometry gate")
    px, py = 0.0, -pp.ZONE_RADIUS_M
    for level, want in ((0, True), (1, True), (2, False)):
        release, hover = check_stack_geometry(level, px, py, pp.BLOCK_HEIGHT_M,
                                             DEFAULT_MAX_LEVEL)
        check("level %d at the place zone centre is %s"
              % (level, "accepted" if want else "refused"),
              (release is not None) == want)
        if release is not None:
            check("level %d hover is above its release" % level,
                  hover > release + pp.MIN_USEFUL_DESCENT_M - 1e-12,
                  "%.4f vs %.4f" % (hover, release))
    # Raising --max-level does NOT rescue level 2: the hover clamp is the
    # binding constraint, not the level check. Worth asserting, because the flag
    # reads as if it would.
    release, _hover = check_stack_geometry(2, px, py, pp.BLOCK_HEIGHT_M, 2)
    check("--max-level 2 still refuses level 2 (the hover clamp catches it)",
          release is None)

    print("pose memory")
    mem = PoseMemory(tol_m=0.002, tol_z_m=0.002, tol_deg=1.0)
    pp.LAST_ARM_GOAL.clear()
    pp.LAST_ARM_GOAL.update({"joint1": 0.1, "joint2": -0.2})
    mem.remember("hover", 0.10, 0.20, 0.15, 0.0)
    check("a remembered pose is recalled exactly",
          mem.recall(0.10, 0.20, 0.15, 0.0) is not None)
    check("1 mm away still hits (inside the 2 mm tolerance)",
          mem.recall(0.101, 0.20, 0.15, 0.0) is not None)
    check("3 mm away misses",
          mem.recall(0.103, 0.20, 0.15, 0.0) is None)
    check("3 mm away in Z misses",
          mem.recall(0.10, 0.20, 0.153, 0.0) is None)
    check("2 deg of yaw away misses",
          mem.recall(0.10, 0.20, 0.15, 2.0) is None)
    check("holding a block is a DIFFERENT pose (different sag model)",
          mem.recall(0.10, 0.20, 0.15, 0.0, holding=True) is None)
    check("the stored angles are the COMMANDED ones",
          mem.entries[0]["joints"] == {"joint1": 0.1, "joint2": -0.2})
    # Yaw wrap: +179 and -179 are 2 deg apart, not 358.
    mem2 = PoseMemory(tol_deg=5.0)
    pp.LAST_ARM_GOAL.update({"joint1": 1.0})
    mem2.remember("wrap", 0.1, 0.1, 0.1, 179.0)
    check("yaw wraps: -179 matches +179 at a 5 deg tolerance",
          mem2.recall(0.1, 0.1, 0.1, -179.0) is not None)
    check("yaw wraps: +170 does not match +179",
          mem2.recall(0.1, 0.1, 0.1, 170.0) is None)
    # Re-remembering the same pose replaces rather than piles up.
    before = len(mem.entries)
    mem.remember("hover again", 0.1001, 0.20, 0.15, 0.0)
    check("re-remembering the same pose replaces it",
          len(mem.entries) == before, "%d entries" % len(mem.entries))
    check("--no-memory never recalls",
          PoseMemory(enabled=False).recall(0.0, 0.0, 0.0, 0.0) is None)

    # Round trip through disk.
    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "mem.json")
    mem.save(path)
    reloaded = PoseMemory(path=path)
    check("a saved memory reloads and still recalls",
          reloaded.recall(0.1001, 0.20, 0.15, 0.0) is not None)
    with open(path, "w") as handle:
        handle.write("{not json")
    check("a corrupt memory file starts empty instead of raising",
          PoseMemory(path=path).entries == [])

    print("block tally")

    class _Det(object):
        pass

    pairs = [(_Det(), "orange_cube"), (_Det(), "green_cube")]
    check("both blocks present passes",
          have_every_block(pairs, ["orange_cube", "green_cube"]))
    check("a missing block refuses",
          not have_every_block(pairs[:1], ["orange_cube", "green_cube"]))

    print("calibration row")
    # THE CLAIM stack_row's docstring makes, tested rather than asserted: a place
    # row must be INERT to every pick-side statistic. If one of these ever
    # returns a number, the place rows are being pooled into the pick-side means
    # and nothing in the file says so -- which is how the first 61 rows of this
    # project became un-poolable.
    row_args = build_parser().parse_args([])
    pick_pose = {"block_class": "orange_cube", "x": 0.13, "y": 0.001,
                 "z": 0.1455, "yaw_deg": -1.5, "zone": [0.0016, -0.0008],
                 "views": 4, "spread_m": 0.0016, "nudges": [],
                 "measurements": [], "flange_fk": [0.144, 0.010, 0.187],
                 "grasped": True}
    placed = {"level": 1, "x": 0.0, "y": -0.2286, "z": 0.1755,
              "yaw_deg": 0.0, "hover_z": 0.2050, "surface_z": 0.026,
              "nudges": [{"dx_mm": 0.0, "dy_mm": -2.0, "dyaw_deg": 0.0,
                          "at_park": True}],
              "measurements": [{"dx_mm": 0.0, "dy_mm": -2.0,
                                "after_nudges": 0}],
              "released": True, "ok": True, "stacked": True}
    row = stack_row(row_args, pick_pose, placed, (0.0, -0.2286), 1)
    check("the row is tagged kind=place", row.get("kind") == "place")
    for metric in ("vision_error", "survey_error", "arm_error",
                   "open_loop_error", "jaw_offset"):
        fn = getattr(calibration, metric, None)
        if fn is None:
            check("calibration.%s exists" % metric, False, "not found")
            continue
        check("calibration.%s is None on a place row" % metric,
              fn(row) is None, repr(fn(row)))
    check("the row carries NO pick-side field names",
          not any(k in row for k in ("nudge", "measured_zone", "measured_world",
                                     "commanded_world", "truth_world",
                                     "truth_zone", "open_loop_offset")),
          sorted(row))
    check("the place-side open-loop reading is recorded, in metres",
          row["place_open_loop_offset"] == [0.0, -0.002],
          repr(row["place_open_loop_offset"]))
    check("a place with no reading records None, not a zero",
          stack_row(row_args, pick_pose, dict(placed, measurements=[]),
                    (0.0, -0.2286), 1)["place_open_loop_offset"] is None)
    check("the row names the constants that produced it",
          isinstance(row.get("constants"), dict)
          and "JAW_PERP_OFFSET_M" in row["constants"])
    check("the row is JSON-serialisable (calibration.record uses json.dumps)",
          isinstance(json.dumps(row, sort_keys=True), str))

    print("symmetry gate")
    check("symmetry 4 is placeable", require_cube("orange_cube", 4, False))
    check("symmetry 2 is refused", not require_cube("x", 2, False))
    check("symmetry 0 is refused (the 'circle' misclassification)",
          not require_cube("x", 0, False))
    check("--allow-any-symmetry overrides", require_cube("x", 2, True))

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


def main():
    if "--selftest" in sys.argv[1:]:
        return _selftest()
    args = build_parser().parse_args()

    # Resolve every name BEFORE the arm moves. A typo in the last block of the
    # stack should not be discovered after the first one is already placed.
    try:
        wanted = [resolve_block_class(name) for name in args.stack]
    except ValueError as exc:
        print("[stack] %s" % exc)
        return 2
    if len(set(wanted)) != len(wanted):
        print("[stack] --stack names the same block twice (%s). There is one of "
              "each on the bench, and the second pick would be sent at a block "
              "that is already in the stack." % ", ".join(wanted))
        return 2
    if len(wanted) - 1 > args.max_level:
        print("[stack] %d blocks means a top level of %d, above --max-level %d."
              % (len(wanted), len(wanted) - 1, args.max_level))
        print("[stack] Level 2's release flange z is %.4f against MAX_HOVER_Z "
              "%.3f -- see check_stack_geometry."
              % (release_flange_z(2, args.block_thickness), pp.MAX_HOVER_Z))
        return 2
    print("[stack] stacking bottom-first: %s" % " -> ".join(wanted))
    for i, name in enumerate(wanted):
        print("[stack]   level %d  %s  release flange z %.4f"
              % (i, name, release_flange_z(i, args.block_thickness)))

    if args.block_thickness != pp.BLOCK_HEIGHT_M:
        print("[stack] NOTE: --block-thickness %.4f differs from "
              "pick_place.BLOCK_HEIGHT_M %.4f. The stack step uses yours; "
              "GRASP_OFFSET_Z and the measurement parks are referenced to the "
              "constant." % (args.block_thickness, pp.BLOCK_HEIGHT_M))

    yaw_fixed = (math.radians(args.zone_yaw) if args.zone_yaw is not None
                 else None)
    memory = PoseMemory(path=args.memory,
                        tol_m=args.memory_tol_mm / 1000.0,
                        enabled=args.memory_enabled)

    rclpy.init()
    io_client = None
    try:
        io_client = pp.RobotIOClient()
        if not io_client.wait_for_joint_states(timeout_sec=10.0):
            print("No /joint_states -- the robot side is not up. "
                  "PROJECT_CONTEXT.md: nan/absent joint states means zero "
                  "publishers, never bad data from the arm.")
            return 1

        # Nominal pose only, exactly as explore_pick_place does it: the sweep
        # reads tag_ids and camera_zx/zy, both in the mat's own frame, so the
        # world pose handed in here cannot bias the survey. Rebuilt with the
        # solved pose below, before anything becomes a world coordinate.
        detector = tpp.Detector(io_client, 0.0, explore.NOMINAL_ZONE_RADIUS_M,
                                0.0, 0.0, tpp.zv.DEFAULT_ZONE_SIZE)
        if not detector.wait_for_service(timeout=15.0):
            return 1

        if not args.no_reset:
            print("[stack] reset to %s" % (explore.RESET_JOINTS_DEG,))
            if not explore.send_joints(
                    io_client,
                    [math.radians(d) for d in explore.RESET_JOINTS_DEG],
                    explore.MOVE_SECONDS, "reset"):
                return 1

        # --- 1. find both zones -------------------------------------------
        sweep_args = argparse.Namespace(
            start=args.start, end=args.end, step=args.step,
            fine_step=args.fine_step, fine_span=args.fine_span,
            pitch=args.pitch, wrist=args.wrist, settle=args.settle,
            zone_yaw=yaw_fixed,
            coarse_patience=max(0, args.coarse_patience))
        zones = ("pickup",) if args.place_at is not None else ("pickup", "place")
        if args.single_pass:
            seen = epp.sweep_both(io_client, detector, sweep_args, args.start,
                                  args.end, args.step, zones,
                                  max(0, args.coarse_patience))
            fit_step = args.step
        else:
            seen = epp.coarse_then_fine_both(io_client, detector, sweep_args,
                                             zones)
            fit_step = args.fine_step
        seen.setdefault("place", [])

        print()
        pickup = epp.survey_zone("pickup", seen["pickup"], detector.zone_size,
                                 fit_step, yaw_fixed)
        if args.place_at is not None:
            place = None
            print("[survey] place zone: not surveyed -- stacking at the "
                  "(%.4f, %.4f) you gave." % tuple(args.place_at))
        else:
            place = epp.survey_zone("place", seen["place"],
                                    detector.zone_size, fit_step, yaw_fixed)
        epp.record_survey("pickup", pickup, args.truth_pickup, args.note)
        epp.record_survey("place", place, args.truth_place, args.note)

        if pickup is None:
            print("\n[stack] no pickup zone, so there is nothing to pick.")
            return 1
        if place is None and args.place_at is None:
            print("\n[stack] pickup found but no place zone. Refusing to pick "
                  "up a block with nowhere to put it -- it would end the run "
                  "held in the jaws.")
            return 1

        place_origin = (tuple(args.place_at) if args.place_at is not None
                        else tuple(place.origin))
        print("\n[stack] pick from (%.4f, %.4f) yaw %+.1f  ->  stack at "
              "(%.4f, %.4f) yaw %+.1f"
              % (pickup.origin[0], pickup.origin[1],
                 math.degrees(pickup.yaw), place_origin[0], place_origin[1],
                 radial_yaw_deg(*place_origin)))
        print("[stack] the stack's ABSOLUTE position is only as good as that "
              "place survey (a few mm, and it needs +-25 mm). Its own "
              "straightness does not depend on it -- see design decision 4.")

        # Check every level's geometry before the first block is lifted.
        for level in range(len(wanted)):
            if check_stack_geometry(level, place_origin[0], place_origin[1],
                                    args.block_thickness,
                                    args.max_level)[0] is None:
                print("[stack] Stopping before anything is grasped.")
                return 1

        # Rebuilt with the SOLVED pickup pose -- zone_to_world has to carry the
        # survey's answer, not the nominal pose the sweep used.
        detector = tpp.Detector(io_client, pickup.origin[0], pickup.origin[1],
                                0.0, pickup.yaw, args.zone_size)

        if not pp.go_home(io_client):
            return 1
        if not io_client.gripper_move_to(pp.GRIPPER_OPEN):
            print("[stack] could not open the gripper")
            return 1

        # --- 2. survey the pickup zone once, for every block --------------
        candidates = survey_pickup_blocks(io_client, detector, args)
        if candidates is None:
            return 1
        if not have_every_block(candidates, wanted):
            return 1

        if args.survey_only:
            print("\n[stack] --survey-only: both zones surveyed, every block "
                  "located and named, geometry checked at every level. Nothing "
                  "was grasped.")
            return 0

        # --- 3. pick and place, bottom first ------------------------------
        stack_xy = place_origin
        for level, want_class in enumerate(wanted):
            print("\n" + "=" * 70)
            print("LEVEL %d -- %s" % (level, want_class))
            print("=" * 70)

            if args.resurvey and level > 0:
                candidates = survey_pickup_blocks(io_client, detector, args)
                if candidates is None:
                    return 1
                if not have_every_block(candidates, wanted[level:]):
                    return 1

            # select_block is index-keyed, so the map is rebuilt from the
            # CURRENT candidate list every time. Reindexing it by hand after a
            # removal is how an off-by-one sends the jaws at the wrong block.
            fused = [det for det, _klass in candidates]
            identity = {i: klass for i, (_det, klass) in enumerate(candidates)}
            block = tpp.select_block(fused, identity, want_class)
            if block is None:
                return 1
            if not require_cube(want_class, block.symmetry,
                                args.allow_any_symmetry):
                return 1

            pick_pose = pick_block(io_client, detector, args, memory, block,
                                   want_class)
            if pick_pose is None:
                return 1

            placed = place_block(io_client, args, memory, stack_xy[0],
                                 stack_xy[1], level, pick_pose)
            # RECORD FIRST, THEN DECIDE. place_block returns its dict even on a
            # step failure once the operator has been at the park, and that
            # reading is the whole point of the run -- see its "ok" comment.
            if placed is not None:
                record_stack_row(args, pick_pose, placed, place_origin, level)
            if placed is None or not placed.get("ok"):
                return 1
            if placed.get("stacked") is False:
                print("[stack] stopping: level %d was not confirmed square, so "
                      "stacking on it would build on a block that is already "
                      "off." % level)
                return 1

            # THE STACK'S XY IS THE COMMANDED ONE, carried forward from the
            # level below. This is the user's own point and it is the right
            # one: any error common to both releases displaces the whole stack
            # together instead of tipping it, so re-deriving the XY per level
            # would only add fresh noise. It also means an operator nudge at
            # level 0 is inherited by level 1, which is what makes a nudged
            # stack straight rather than stepped.
            if placed.get("released"):
                stack_xy = (placed["x"], placed["y"])
                print("[stack] level %d released at (%.4f, %.4f). Level %d "
                      "will target the same XY, %.0f mm higher."
                      % (level, stack_xy[0], stack_xy[1], level + 1,
                         args.block_thickness * 1000))

            # The block just picked is no longer in the zone. Dropped by object
            # identity, so the pairing of detection to class survives -- which
            # is the whole reason candidates is a list of pairs and not two
            # parallel structures with indices joining them.
            candidates = [(det, klass) for det, klass in candidates
                          if det is not block]

        print("\n[stack] stack complete: %s, bottom first, at (%.4f, %.4f)."
              % (" -> ".join(wanted), stack_xy[0], stack_xy[1]))
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1
    finally:
        # SAVED WHATEVER HAPPENED. A run that failed at the second pick still
        # solved every pose up to that point, and those are the poses the retry
        # is about to ask for again. Keeping them only on success would throw the
        # memory away in exactly the case it is most useful.
        memory.report()
        memory.save()
        if io_client is not None:
            print("\n=== Returning home ===")
            try:
                pp.go_home(io_client)
            except Exception as exc:                        # noqa: BLE001
                print("go_home failed on the way out: %s" % exc)
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
