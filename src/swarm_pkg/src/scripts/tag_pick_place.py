#!/usr/bin/env python3
"""Stage 1: pick a block from a tag-marked zone, wherever it is, however it sits.

RUNS ON MARS, like pick_place.py -- it needs move_group for IK and OMPL. The
vision lives on the Pi behind the /detect_block service; no image ever crosses
the network. See APRIL_TAGS.md for the whole design.

    # Pi:   camera + detector, alongside real_robot_hardware.launch.py
    ros2 launch mycobot_280pi_camera_moveit2 camera.launch.py
    python3 block_detector_node.py

    # mars: planning, then this
    ros2 launch mycobot_280pi_camera_moveit2 real_robot_planning.launch.py
    python3 tag_pick_place.py --zone-origin 0.0 0.2286 0.050 --dry-run
    python3 tag_pick_place.py --zone-origin 0.0 0.2286 0.050

Everything about arm motion is imported from pick_place.py rather than
reimplemented -- same RobotIOClient, same seeded IK, same trapezoidal timing,
same settle logic. The only addition there was a per-move block_yaw_deg, which
defaults to 0.0 and leaves every existing caller untouched.

------------------------------------------------------------------------------
WHAT THE CORRECTION LOOP ACTUALLY MEASURES
------------------------------------------------------------------------------
Worth being explicit, because the intuitive reading is wrong.

Detecting the block again after moving verifies NOTHING about the arm. The
block's zone-local position is derived from the tags, so it is the same answer
no matter where the arm is standing -- move and re-detect and you re-measure the
block, not the move.

What does measure the arm is where the CAMERA ended up: the zone-local point
under the image centre (see zone_vision.camera_in_zone). That is an external
observation of the arm's position that owes nothing to its encoders, and so is
blind to the gravity droop and dead-zone effects that make the encoders
untrustworthy in the first place. TESTS.md names "external metrology" as the
fallback if the residuals turn out not to be plainly correctable; this is it.

So the loop is: command a flange position, measure where the camera actually
went, correct by the difference, repeat. The block's position is measured ONCE
and is not what is being converged on.
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import block_coordinates as bc  # noqa: E402
import calibration  # noqa: E402
import pick_place as pp  # noqa: E402  -- module handle, for the tunables below
import detection_wire as wire  # noqa: E402
import tool_frame_check  # noqa: E402
import zone_vision as zv  # noqa: E402
from pick_place import (  # noqa: E402
    GRASP_OFFSET_Z,
    GRIPPER_OPEN,
    HOME_RADIANS,
    PICK_XYZ,
    PLACE_XYZ,
    ZONE_RADIUS_M,
    RobotIOClient,
    cartesian_move_to,
    go_home,
    gripper_close_until_contact,
    hover_z_for,
    move_arm_to,
    quat_multiply,
)
from swarm_interfaces.srv import DetectBlock  # noqa: E402

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
# Stage 1 block: 1.18 in square, the thickness quoted in APRIL_TAGS.md.
DEFAULT_BLOCK_THICKNESS = 0.030      # m

# NOTHING IN THIS FILE COMPUTES THE GRASP HEIGHT. It is PICK_XYZ.z, unchanged.
#
# This is worth stating loudly because the obvious-looking alternative is wrong
# and was briefly implemented: grasp_z = zone_z + block_thickness/2 +
# GRASP_OFFSET_Z, reading --zone-origin's Z as the mat surface. That treats
# PICK_XYZ.z as if it were geometry. It is not -- it was HAND-TUNED against the
# real robot, so it already contains the arm's z error at the pick pose, and no
# amount of mat-and-block arithmetic reproduces that term.
#
# The proof it is not geometry is sitting in the two constants. PICK_XYZ.z is a
# block CENTRE (0.0650) and PLACE_XYZ.z a resting SURFACE (0.0750) on the same
# flat mat, which as pure geometry would need the block to be -20 mm thick. They
# differ because pick and place sit at opposite ends of the workspace (+Y vs -Y)
# and the arm droops differently at each. Both numbers are right; neither is
# derivable.
#
# So Stage 1 measures X, Y and yaw -- the things a top-down camera can actually
# see -- and takes Z from the configuration that already picked this block off
# this mat. The 2026-08-02 changes (zone in to 9 in, a sheet of paper under the
# mat) moved neither the block nor the mat height.
#
# --zone-origin's Z is now cosmetic: ZoneSpec stores world_z and nothing reads
# it, the homography being entirely in-plane. It is kept only because zone_z is
# part of the DetectBlock service contract.
GRASP_FLANGE_Z = PICK_XYZ[2] + GRASP_OFFSET_Z          # 0.1550, hover 0.1950

# ---- MEASURED from TESTS.md Test 1, 2026-07-29 ----------------------------
# 108 trials, 6 decorrelated postures, joints 0/1/2, both directions, 3 repeats.
# Raw data: src/swarm_pkg/testing/test1_full.csv. Full reduction in APRIL_TAGS.md.
#
# Converted to metres at the r = 0.25 m pick/place radius, since that is where
# these corrections actually happen.
#
# CONVERGED -- "close enough, stop correcting."
# The arm's same-direction repeatability is 0.045 deg = 0.20 mm, which is BELOW
# its own 0.088 deg readback quantum (36% of cells returned bit-identical
# residuals across all three repeats). So repeatability is emphatically not the
# binding constraint and a threshold set from it would be absurdly tight: this
# arm lands in the same place every time, that place is just the wrong one.
# 2 mm is comfortably above the noise floor and below the dead band, so
# reaching it means the correction genuinely worked rather than got lucky.
CORRECTION_CONVERGED_M = 0.002       # MEASURED basis: 0.20 mm repeatability
#
# DEADZONE -- "the arm physically cannot fix an error this small, stop trying."
# Bias required before a joint moves at all: J0 0.99 deg, J1 1.36 deg, J2
# 1.08 deg (medians), worst case seen 2.78 deg. At r = 0.25 m that is 4.3 /
# 5.9 / 4.7 mm typical. Below this a correction command produces literally zero
# motion -- the joint settles at the bit-identical encoder count -- so retrying
# is guaranteed to do nothing and the right move is to descend and log it.
#
# Set to 5 mm, the typical dead band rather than the 12 mm worst case: too high
# and the loop gives up while it could still have helped.
CORRECTION_DEADZONE_M = 0.005        # MEASURED: 0.99-1.36 deg dead band
#
# BACKLASH -- measured, NOT yet compensated. See APRIL_TAGS.md "Open".
# Joint 0 loses 1.83 deg (median, max 2.01) of lost motion on a direction
# reversal = 8.0 mm at r = 0.25 m, LARGER than the dead zone itself. J0 is the
# joint that swings the gripper laterally across the zone, so a correction that
# reverses direction spends its first 8 mm taking up slack and does nothing
# visible. J1 (0.53 deg) and J2 (0.80 deg) are far better behaved.
#
# The standard fix is a unidirectional final approach -- always arrive at a
# hover from the same side, so the slack is already taken up. Deliberately NOT
# implemented yet: it changes how every hover move is planned, and it should be
# added against a measured before/after on real frames rather than on the
# strength of this number alone. Until then, expect a correction that reverses
# direction to under-deliver by roughly this much.
BACKLASH_J0_DEG = 1.83               # MEASURED, for logging and for that fix
# ---------------------------------------------------------------------------

MAX_CORRECTIONS = 2                  # then abort rather than descend blind

# Raised 20 -> 45s on 2026-07-31. A detect call that decoded tags fine (2 tags,
# rms 4.4px) still blew past 20s end to end -- the Pi's own log showed a burst of
# repeating DDS deserialization errors ('invalid data size' / 'string data is not
# null-terminated') before it recovered and answered. A 640x480 frame is ~920KB
# against this link's deliberately small MaxMessageSize=1400B (the campus-WiFi
# MTU fix), so it fragments into 700+ RTPS pieces per frame -- plausible that
# reassembly occasionally stalls under this Galactic Cyclone DDS build. That is a
# transport problem to fix separately; in the meantime, treating "slow" the same
# as "broken" was throwing away answers that had already arrived correctly.
DETECT_SERVICE_TIMEOUT = 45.0

SETTLE_AFTER_MOVE_SEC = 0.6          # let the arm stop ringing before a still

# Hover height for DETECTION only (not the grasp approach -- see GRASP_OFFSET_Z
# and hover_z() in pick_place.py for that, unrelated).
#
# MEASURED 2026-07-30/31, not guessed:
#   - lens needs >= ~220mm above the mat to focus at all (below that: Laplacian
#     focus metric 27-84, no tag decodes even when pointed straight at one;
#     above: 207-221, clean decodes).
#   - a STRAIGHT-DOWN camera cannot reach that height at this zone's 254mm
#     radius -- all 19 deterministic IK seeds fail, and letting OMPL sample
#     freely is worse, not better (it found a mirror-configuration solve with
#     the base swung 180deg, arm reaching back over itself, on hardware).
#   - the fix is look_at_quat(): tilt the approach axis a few degrees off
#     vertical so the flange position IS reachable, rather than demanding an
#     orientation the arm cannot hold there. Verified end to end on hardware
#     2026-07-31 at this exact height: IK converged on real seeds (not the
#     fallback), and the detector decoded 2 tags at rms 4.4px.
# Lowered 0.280 -> 0.240 on 2026-07-31. At 0.280 the flange could not reach the
# zone at all: measured ceiling ~0.245 m radial, against a zone centre at 0.254,
# so grasp-hover hit "Planning FAILED" on three consecutive runs at radial
# 0.2490 / 0.2498 / 0.2643. OMPL constraint sampling already had the wider
# 8.6 deg window at those targets and still found nothing, so this was a genuine
# reach limit, not a tolerance or seeding problem.
#
# 0.240 keeps focus: the lens sits ~16.5 mm below the flange at the detection
# wrist yaw (flange 0.280 -> lens 0.2635, measured), so this puts the lens at
# ~0.2235 -- still above the measured ~0.220 focus floor, with ~20 mm more reach.
#
# NOT raised back up when the zone moved in to 0.2286 on 2026-08-02, even though
# the reach argument that forced it down is now much slacker. The 0.220 focus
# floor is the OTHER constraint on this number and it did not move: at 0.240 the
# lens sits ~3 mm above it, so there is far more room to go DOWN-and-lose-focus
# than there is reason to go up. Raising it would trade a measured-good
# detection height for reach margin that is no longer scarce.
# Lens-to-subject distance below which this camera does not focus, measured
# 2026-07-30/31. px/m x distance = 551 makes every detection report its own
# distance for free, so this is checkable on every still rather than assumed.
FOCUS_FLOOR_M = 0.220
PX_M_INVARIANT = 551.0

# Raised 0.240 -> 0.255 on 2026-08-06, and the old value was not a mistake so
# much as a design with no margin. At flange z 0.240 the model puts the lens
# 0.2215 m from the mat -- 1.5 mm above the focus floor -- and the arm's own z
# error is larger than that. Nine consecutive stills measured 0.2177-0.2217 m
# (from their own px/m), mean 0.2196, with FIVE OF NINE below the floor. The
# frames are visibly soft and the block tag decodes about half the time.
#
# It also fixes framing, which was failing by two pixels. At 2500 px/m the
# zone's half-diagonal is 180 px and the lens sat 24 mm (62 px) off centre, for
# 242 px against the 240 a 480-tall frame allows -- so a corner tag was always
# just outside. Backing off to 0.255 takes the half-diagonal to 169 px.
DETECT_HOVER_Z = 0.255

# If the raised hover is out of reach, drop back rather than skipping the still.
# Reach shrinks with height and this height has not been proven on hardware; a
# blurred still beats no still, and the log says which one you got.
DETECT_HOVER_Z_FALLBACK = 0.240

# How far to pull the flange IN from the zone centre, toward the base, before
# aiming with look_at_quat.
#
# 0.054 until 2026-08-02, and that number was a COMPROMISE, not a target: it was
# the pull-in that put the flange at y=0.200 against a zone centre at y=0.254,
# which was the deepest pose actually verified reachable at the time. Centring
# the lens over the zone would have wanted the flange at 0.254 - 0.041 = 0.213,
# which that run had already failed to reach. So the survey ran 13 mm off-centre
# because that was what the arm could do.
#
# The zone moving in to ZONE_RADIUS_M = 0.2286 removes the compromise. Lens
# centring now wants the flange at 0.2286 - 0.041 = 0.1876, which sits INSIDE
# the band already proven good on hardware (0.1529 reached fine and gave the
# best still of that run; 0.200 reached fine). So take the geometrically right
# answer instead of the reachable-compromise one.
#
# 0.041 is not a fresh guess either -- it is the measured lateral lens offset in
# world metres at the DETECTION wrist yaw (camera_offset_world(180) = +41.0 mm,
# pointing outward, away from the base). Pull the flange in by exactly that and
# the lens lands on the zone centre. Framing at the four survey yaws improves
# across the board, computed against this geometry:
#
#     yaw 180 -> 13.0 mm off centre  ==>   0.4 mm      (3.1 deg tilt -> 0.1)
#     yaw 150 -> 26.8 mm             ==>  20.3 mm      (6.4 deg -> 4.8)
#     yaw 210 -> 27.5 mm             ==>  21.1 mm      (6.5 deg -> 5.0)
#     yaw 120 -> 47.5 mm             ==>  39.6 mm      (11.2 deg -> 9.4)
#
# and the starting flange sits 37.6 mm above MIN_FLANGE_RADIUS_M rather than
# 24.6 mm, so the correction loop has more room to walk inward before it clamps.
#
# THE SIGN TRAP IS STILL LIVE, so read this before touching it: 0.041 is used
# here as a fixed RADIAL pull-in, not by calling camera_offset_world() per move.
# That offset's sign FLIPS with wrist yaw -- -38.9 mm at yaw 0, +41.0 mm at
# yaw 180 -- and an earlier version that derived the flange position from it at
# the wrong yaw pushed the survey OUTWARD to y=0.293 instead of inward, past
# anything reachable, failing all 19 IK seeds on all 4 stills. The multiview
# stills deliberately sweep the wrist yaw, so the offset is genuinely different
# for each one and there is no single value to derive from. look_at_quat aims by
# ROTATING, so the flange does not have to sit at a yaw-precise lens distance --
# it only has to be somewhere reachable near the zone. The 41 mm below buys good
# framing at the yaw the survey leans on; it is not a per-still correction.
DETECT_HOVER_PULLIN_M = 0.041

# Hard floor on how close to the base the flange may be commanded, in the XY
# plane. NOT a reachability limit -- IK converges happily inside it, and
# move_group plans a clean trajectory to it. The arm then physically collides
# with itself and stalls, because this URDF's collision geometry does not
# describe the real gripper well enough for MoveIt to reject the state.
#
# Measured on hardware 2026-07-31, at DETECT_HOVER_Z:
#     radial 0.2000 m -- survey pose, fine
#     radial 0.1529 m -- grasp-hover start, fine (best still of the run, 4 tags)
#     radial 0.1209 m -- COLLIDED, arm stalled 0.089 rad short and stayed there
#
# So the floor sits just under the deepest pose known to work, not at some
# padded guess: 0.1529 m is worth keeping, it produced the best detection of the
# whole run. Nothing between 0.121 and 0.153 has been tested, so treat the gap
# as unknown rather than safe.
#
# This is enforced on the CORRECTION loop specifically. Each correction is
# "flange += measured camera error", which has no notion of the workspace at
# all: a large error near the inner edge of the reach walks the arm straight
# into its own base, one correction at a time. Ask, do not assume, that a
# vision-driven delta lands somewhere the arm can physically go.
MIN_FLANGE_RADIUS_M = 0.150

# Outward companion to the floor above, and it exists because the failure is
# WORSE than "the move fails".
#
# Hardware 2026-08-02, at DETECT_HOVER_Z: commanded flange radial 0.2271 m. All
# 19 IK seeds failed. The OMPL fallback did NOT refuse -- it satisfied its 4 cm
# position sphere by parking the arm short and low, and the homography scale
# proves how far: 2891 px/m against the survey's 2466 puts the lens at 0.1906 m
# instead of 0.2235, i.e. 33 mm low and below the 0.220 m focus floor. The next
# detection then measured that shortfall as a 55 mm "position error", and the
# correction loop dutifully pushed the flange FURTHER OUT, to 0.2770, where
# planning finally failed outright.
#
# That is a runaway, not a miss: every correction makes the next reading worse,
# because commanding an unreachable target produces a short pose, and a short
# pose looks exactly like an error pointing outward. The loop has no way to tell
# "I did not get there" from "the target moved", so it must be stopped from
# asking in the first place.
#
# 0.245 m is the measured ceiling at DETECT_HOVER_Z's predecessor (0.280) and is
# therefore CONSERVATIVE here -- 0.240 reaches at least as far. Deliberately not
# raised without a fresh reach_probe.py sweep at 0.2286: an over-tight guard
# refuses a reachable target and says why, which is recoverable, while an
# over-loose one restores the runaway.
MAX_FLANGE_RADIUS_M = 0.245

# Orientation window for the CAMERA-AIMING poses only, overriding
# pick_place.IK_ORI_XY_TOLERANCE (0.10 rad / 5.7 deg) for these moves.
#
# Measured 2026-07-31: the orientation look_at_quat asks for at the survey
# pose sits between 5.7 and 8.6 deg from anything this arm can hold, so the
# tight grasp window admits NO solution and all 27 seeds fail on every still.
# Each one then fell through to OMPL constraint sampling, whose window is the
# make_orientation_constraint default of 0.15 rad -- so the arm was already
# being commanded into that band, just at a randomly chosen point in it. Two
# runs of identical code sampled poses 0.371 rad apart and returned 3 usable
# views against 1.
#
# Matching the fallback's window here changes nothing about where the arm may
# go; it only lets the deterministic, least-travel seeded solve claim those
# poses instead of leaving them to random sampling. Deliberately NOT applied
# to the descent or the grasp -- tilt there is the error this whole project
# is trying to remove, and those keep the tight 0.10 rad window.
DETECT_ORI_XY_TOLERANCE = 0.15

# How long to wait for the block_detections topic after a /detect_block call.
# The node publishes it BEFORE building the reply, so it has normally already
# arrived and this costs nothing; it exists for the case where the executor did
# not get a chance to deliver it during the call.
WIRE_WAIT_SEC = 1.5

# Below this a 36h11 decode is possible but not to be leaned on -- see
# zone_vision.BLOCK_TAG_MODULES and block_detector_node's own thresholds. Only
# used to annotate the printout; a marginal decode is still a decode, and the
# id it produced either exists in the scheme or it does not.
BLOCK_TAG_PX_PER_MODULE_GOOD = 4.0

# How close a block tag has to be to a fused contour before it is taken to be
# ON that block, metres. The tag is stuck to the block's TOP face, so it is a
# RAISED point projected onto the mat plane and carries a parallax offset
# outward from the camera -- hardware 2026-08-05 measured 7 mm of it at the
# survey hover (contour at zone (-13.6, +2.3), its own top tag at (-20.3,
# +0.4)). 20 mm covers that with margin while staying well inside the ~50 mm
# that separates two blocks sitting side by side in a 4 in zone.
BLOCK_TAG_MATCH_M = 0.020


def _pullin_toward_base(x, y, pullin_m):
    """(x, y) moved pullin_m closer to the base along the line to it.

    Radial, not a fixed axis, so it still does something sane for a zone that
    is not sitting on the Y axis (this pickup zone happens to be, so radial and
    "subtract from y" are the same thing here, but generalizing costs nothing).
    """
    radial = math.hypot(x, y)
    if radial <= pullin_m:
        return (0.0, 0.0)
    frac = 1.0 - pullin_m / radial
    return (x * frac, y * frac)


# ---------------------------------------------------------------------------
# Multi-view acquisition
# ---------------------------------------------------------------------------
# The gripper hangs in front of the lens and hides the far pair of tags from
# every hover the arm can reach, so ONE still sees 2 of the 4 tags. That fit is
# over-determined but poorly conditioned across the thin direction of the pair
# (zone_vision.MIN_TAGS has the arithmetic). The fix is several stills with the
# wrist rotated between them, so a different pair is hidden each time.
#
# WRIST yaw, via joint6output_to_joint6, NOT a base rotation. Two reasons:
#   - J0 has 1.83 deg of measured backlash (8.0 mm at r = 0.25 m), so rotating
#     the base between stills would move the camera by more than the thing being
#     measured. The wrist joint is far better behaved.
#   - the camera sits 40 mm off the flange axis, so a wrist rotation swings the
#     lens around the zone and changes which tags are occluded, which is exactly
#     the effect wanted.
#
# The angles do NOT need to be accurate, or even known. Each still is solved
# independently from the tags visible in it alone -- see the note above
# zone_vision.analyze_multi. The rotation only has to CHANGE the occlusion. That
# is what makes this robust on a robot whose encoders disagree with its links by
# degrees (APRIL_TAGS.md, ROOT CAUSE).
#
# Four offsets at 90 deg was the professor's suggestion and it is a good one in
# principle: it guarantees every tag is visible in at least one still regardless
# of which pair the gripper starts out hiding. Two hardware runs have now shaped
# it into something the arm can actually do.
#
# The lens sits ~40 mm off the flange axis, so as the wrist turns the lens swings
# around a circle of that radius, and there is no single flange position that
# frames the zone at more than one yaw. survey_flange_for_yaw() therefore moves
# the flange per still. What that costs is REACH, and the cost is not symmetric:
# at yaw 90 the offset points radially outward, so the flange pulls IN and the
# radius DROPS; at 0 and 180 the offset is lateral and the radius goes up
# whatever you do; at 270 the flange is pushed out past the zone entirely.
#
#     yaw   flange that centres the lens     radius    measured
#       0   (0.2287, +0.0399)                0.2321    ALL 19 IK SEEDS FAILED
#      30   (0.2087, +0.0346)                0.2115
#      60   (0.1941, +0.0200)                0.1951
#      90   (0.1887, +0.0001)                0.1887    reached, 4 of 4 tags
#     120   (0.1940, -0.0199)                0.1950
#     150   (0.2085, -0.0345)                0.2114
#     180   (0.2285, -0.0399)                0.2320    ALL 19 IK SEEDS FAILED
#     270   (0.2685, -0.0001)                0.2685
#
# (Zone at (0.2286, 0), DETECT_HOVER_Z. Recompute with survey_flange_for_yaw if
# the zone moves -- the shape of the curve is fixed, its position is not.)
#
# So the yaws are ORDERED NEAREST-FIRST around 90, and the first three are all
# within 7 mm of the one radius hardware has actually reached. 30 and 150 are
# held in reserve: at 0.2115 they are plausible and untested, and they only get
# tried if one of the first three fails. MULTIVIEW_ENOUGH_VIEWS stops the pass
# as soon as three stills have produced a usable homography.
#
# 60 deg of wrist rotation looks like less occlusion diversity than 90, and the
# 2026-08-05 run says that is not the binding constraint: with the lens actually
# ON the zone centre, ONE still at yaw 90 decoded all four tags. Framing was
# doing the damage, not the gripper's shadow.
MULTIVIEW_YAW_OFFSETS_DEG = (90.0, 60.0, 120.0, 30.0, 150.0)

# A still resting on fewer than this many zone tags is not fused in. Two tags
# make a homography that is over-determined and badly conditioned across the
# thin direction of the pair (zone_vision.MIN_TAGS has the arithmetic), and its
# residual comes out LOW because 8 correspondences against 8 DOF nearly fit by
# construction -- so a bad still looks like a good one. On 2026-08-05 two such
# stills, taken from poses the arm never reached, pulled the fused block 3 mm
# off a known truth and pushed the view spread past its gate; the one well-
# framed still had it to 3.2 mm on its own. Two tags is enough to detect a zone.
# It is not enough to vote on where a block is.
MULTIVIEW_MIN_TAGS = 3

# Largest lens-position error the survey will correct for framing. Measured
# readings sit at 20-28 mm; 60 mm is well past anything framing explains, and a
# reading that big means the arm is not where FK thinks or the zone origin is
# wrong -- moving the arm on it would make things worse, not better.
MAX_LENS_BIAS_M = 0.060

# THE BLOCK'S TOP FACE IS NOT ON THE MAT PLANE, AND THE HOMOGRAPHY ONLY KNOWS
# ABOUT THE MAT PLANE.
#
# The four zone tags are printed on paper lying flat, so the homography maps the
# image to z = 0. A block's top face floats BLOCK_HEIGHT_M above that, which
# means it projects outward from the camera's nadir by the ratio of the two
# heights -- a pure magnification about the nadir, zero at the nadir itself and
# growing linearly with distance from it.
#
#     apparent_offset = true_offset * h / (h - t)
#
# MEASURED 2026-08-06, and it is exactly this. Eight placements at a
# tape-measured 20 mm from the zone centre, four directions, read back:
#
#     22.4  22.5  23.3  23.3  22.7  22.6  22.9  22.9   ->  mean 22.82 mm
#
# and h/(h-t) at the 0.2396 m mean lens height predicts 20 * 1.1431 = 22.86 mm.
# Agreement to 0.04 mm across both axes and all four directions.
#
# This is why twenty-four runs with the block on the zone centre never saw it:
# the correction is identically zero at the nadir, and the lens re-centring puts
# the nadir on the zone centre. It only appears once the block is off-centre,
# where it costs 14% of the offset -- 2.9 mm at 20 mm, 6.5 mm at the zone edge.
#
# Applied per still, using THAT still's own lens height and nadir, before the
# views are fused. Not on the Pi: the correction needs the nadir in zone
# coordinates, which is the one quantity the camera model supplies, and doing it
# here keeps the detector node free of any assumption about what it is looking
# at. The nadir estimate carries the camera model's ~5 mm uncertainty, which
# enters the correction multiplied by 0.14 -- 0.7 mm, well under the effect.
TOP_FACE_HEIGHT_M = DEFAULT_BLOCK_THICKNESS

# short/long above which a TOP face carrying a tag is called square, and so
# four-fold symmetric. See promote_tagged_tops_to_square.
#
# Looser than zone_vision's SQUARE_ASPECT_TOL (0.88) on purpose. That constant
# decides square-vs-rectangle with no other evidence; this one only has to rule
# out a genuinely elongated block, and it runs on a contour already confirmed by
# a decoded top tag. The measured footprints of the 30 mm cube on 2026-08-06 ran
# 30.8x31.2 up to 31.8x35.4 -- ratios 0.99 down to 0.90 -- and every one of them
# is the same cube. 0.80 accepts all of them and still refuses a 30x50 cuboid.
SQUARE_TOP_ASPECT_MIN = 0.80

# Give up on the multi-view pass once this many stills have produced a usable
# homography. 4 tags across >=2 views is already enough to fuse; the remaining
# stills cost a wrist move and ~1 s each for diminishing return.
MULTIVIEW_ENOUGH_VIEWS = 3

# Fused spread above this means the views disagree badly enough that their
# average should not be descended on. Deliberately larger than
# CORRECTION_CONVERGED_M: this is view-to-view disagreement about a STATIONARY
# block, so it is pure measurement scatter, whereas the correction threshold also
# has to absorb the arm's dead band.
MULTIVIEW_MAX_SPREAD_M = 0.006

# Companion to MULTIVIEW_MAX_SPREAD_M for ORIENTATION agreement, in degrees.
#
# Position spread alone does not catch a bad fusion. Hardware 2026-07-31: a
# candidate at 2.0 mm position spread -- comfortably "agreeing" -- carried a
# 177.2 deg yaw spread. Two views cannot see the same rigid object 177 deg
# apart; that is two different things merged into one cluster. The real block
# in that same run fused at 0.8 mm / 0.2 deg, so the separation between a good
# fusion and a bad one is stark and this threshold is nowhere near either edge.
#
# Skipped entirely for symmetry == 0 (a circle), where yaw carries no meaning
# and any spread is expected.
MULTIVIEW_MAX_SPREAD_YAW_DEG = 20.0


def quat_to_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


# The URDF puts the lens on the OPPOSITE side of the flange from where it
# physically is. Established 2026-07-30 on hardware, and it is a model error, not
# a code error: the chain to wrist_camera_link goes through two hand-authored
# right angles (joint6output_to_camera_flange rpy="1.5708 1.5708 0", then
# camera_flange_to_camera_link), the same kind of hand-authored tool geometry
# that already proved untrustworthy for the gripper mount earlier the same day.
#
# The evidence. The survey hover was commanded so the URDF-modelled lens would
# sit on the zone centre, and exactly ONE of four tags was detected. Predicted
# off-axis angles for the two competing models, at the joint angles actually
# achieved:
#
#     tag   URDF as written   lens 180 deg opposite
#      0        30.9 deg           24.3 deg
#      1        29.7 deg           21.3 deg
#      2        30.4 deg           43.7 deg
#      3        31.6 deg           44.5 deg
#
# The URDF model says all four sit at the same ~30 deg, so it cannot explain why
# tag 0 was seen and tags 2 and 3 were not -- they are no further off-axis than
# the one that worked. The flipped model puts 2 and 3 at ~44 deg, outside any
# plausible lens, and 0 and 1 comfortably inside. Only the second model is
# consistent with the observation. (Tag 1, inside the frame but undetected, is
# separately explained by the gripper occluding it -- the known 2-of-4 problem.)
CAMERA_MOUNT_FLIPPED = True

# Wrist yaw used for DETECTION hovers, over and above the grasp yaw.
#
# This is NOT redundant with the flip above, and the reachability arithmetic is
# why. With the lens physically on the near side of the flange, putting it over
# the zone centre would need the FLANGE ~41 mm FURTHER OUT than the zone --
# 295 mm when the zone was at 254, and 270 mm now that it is at 228.6. Both are
# past this arm's reach at DETECT_HOVER_Z. Rotating the wrist 180 deg swings the
# lens to the far side, so the same lens position needs the flange 41 mm INSIDE
# the zone instead: 213 mm then, 188 mm now (== DETECT_HOVER_PULLIN_M's whole
# derivation). The flip tells the code which side the lens is on; this picks the
# wrist angle that makes the required flange position reachable.
#
# Note the zone move did not make this optional. It bought ~25 mm of margin; the
# wrong wrist yaw costs ~82 mm (41 out instead of 41 in). Still nowhere near.
#
# Detection-only. The grasp descent still uses the block's own yaw -- rotating the
# wrist about its own axis moves the jaws' orientation, not the flange position,
# so the two are independent.
DETECT_WRIST_YAW_DEG = 180.0


def camera_offset_world(block_yaw_deg, x=None, y=None):
    """(dx, dy): where the lens sits relative to the flange, in world metres.

    The lens is 40 mm off the flange axis, which is 40% of the zone's width --
    command the flange to the zone centre and the camera looks at the zone's
    edge. Framing only: the block's measured position comes from the tags and
    does not care whether this is right. Getting it wrong loses tags out of
    frame; it does not bias the answer.

    Depends on the grasp yaw because the whole tool assembly rotates with it, and
    on the target position because the sag pre-compensation tips the whole assembly
    off vertical. That tip swings the lens by 40 mm * sin(~4 deg) = ~3 mm, which
    is small next to the framing margin but free to get right -- pass the target
    and the offset matches the orientation actually commanded.
    """
    from pick_place import grasp_quat_for

    offset, _view = tool_frame_check.flange_to_camera()
    if CAMERA_MOUNT_FLIPPED:
        # Negate the LATERAL components only. The axial (+Z, along the flange
        # axis) component is unaffected by a 180 deg rotation about that axis, so
        # negating it too would move the lens up the tool rather than around it.
        offset = (-offset[0], -offset[1], offset[2])
    rotation = quat_to_matrix(grasp_quat_for(block_yaw_deg, x, y))
    world = [sum(rotation[i][k] * offset[k] for k in range(3)) for i in range(3)]
    return world[0], world[1]


# Fixed geometry from the URDF chain joint6_flange -> wrist_camera_optical_frame,
# WITH the CAMERA_MOUNT_FLIPPED correction already baked in.
#
# BUG, found 2026-07-31 on hardware: the first version of this took these two
# constants straight from the raw URDF chain and never applied the flip. It then
# aimed confidently and precisely -- at the position the WRONG-side camera model
# predicts. camera_offset_world() (above) already knew about the flip; this did
# not, because it was added the next day without cross-checking. Costed a run:
# the detector's own tag-based measurement put the camera 104mm off in X, 27mm
# in Y from where the (buggy) model claimed 0.37mm of error.
#
# The physical correction is a 180deg rotation about the flange's own axis
# (== the optical axis, see below), which negates a vector's LATERAL (X, Y)
# components and leaves its AXIAL (Z) component alone -- same operation
# camera_offset_world() already does. Applied here directly rather than at
# call time so every user of these two constants gets it automatically instead
# of needing to remember to flip, which is exactly the mistake that happened.
_RAW_OPTICAL_AXIS_IN_FLANGE = (0.0058, 0.0058, 1.0000)
_RAW_LENS_OFFSET_IN_FLANGE = (-0.0282, 0.0283, 0.0185)
if CAMERA_MOUNT_FLIPPED:
    OPTICAL_AXIS_IN_FLANGE = (-_RAW_OPTICAL_AXIS_IN_FLANGE[0],
                              -_RAW_OPTICAL_AXIS_IN_FLANGE[1],
                              _RAW_OPTICAL_AXIS_IN_FLANGE[2])
    LENS_OFFSET_IN_FLANGE = (-_RAW_LENS_OFFSET_IN_FLANGE[0],
                             -_RAW_LENS_OFFSET_IN_FLANGE[1],
                             _RAW_LENS_OFFSET_IN_FLANGE[2])
else:
    OPTICAL_AXIS_IN_FLANGE = _RAW_OPTICAL_AXIS_IN_FLANGE
    LENS_OFFSET_IN_FLANGE = _RAW_LENS_OFFSET_IN_FLANGE


def _mat_vec(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v] if n > 1e-12 else list(v)


def look_at_quat(flange_xyz, target_xyz, block_yaw_deg=0.0, iterations=2):
    """Flange orientation that AIMS THE LENS at target_xyz.

    Why this exists, and why "no orientation constraint" is not the alternative:
    detection needs the camera >= ~220 mm above the mat to focus (measured
    2026-07-30), and the arm cannot hold a straight-DOWN camera that high over a
    zone 254 mm out -- every IK seed fails. (The zone has since moved in to
    228.6 mm, which relieves that particular case; this function stays because
    the reasoning below about WHY "no constraint" is the wrong fix is unaffected
    by how far out the zone is, and because aiming beats a free planner at any
    radius.) The tempting shortcut is to drop the
    orientation constraint entirely and let the planner reach the position any
    way it likes. That was tried on hardware and is actively bad: OMPL is then
    free to satisfy position alone, and it chose joint6_to_joint5 = -2.40 rad at
    one height and swung the BASE to -1.69 rad (the mirror solution, arm reaching
    back over itself) at another. Both produced beautifully sharp frames -- focus
    221 and 207, up from 27-43 -- containing no tags at all, because the lens was
    pointed at the wall.

    So the fix is not fewer constraints, it is the RIGHT one: stop demanding
    "point straight down" (which is a grasp requirement that detection never
    needed) and demand "point at the zone" instead. That frees exactly the degree
    of freedom the reach problem needs while keeping the only property detection
    actually cares about.

    Method: start from the known-reachable downward grasp orientation, then apply
    the minimal rotation carrying its optical axis onto the direction from lens to
    target. Iterated because the lens position itself moves when the orientation
    changes -- the lens is 40 mm off the flange axis, so rotating the flange
    swings it. Two passes converge well below a millimetre.
    """
    from pick_place import grasp_quat_for

    q = grasp_quat_for(block_yaw_deg, flange_xyz[0], flange_xyz[1])
    for _ in range(max(1, iterations)):
        R = quat_to_matrix(q)
        lens = [flange_xyz[i] + _mat_vec(R, LENS_OFFSET_IN_FLANGE)[i]
                for i in range(3)]
        desired = _unit([target_xyz[i] - lens[i] for i in range(3)])
        current = _unit(_mat_vec(R, OPTICAL_AXIS_IN_FLANGE))

        dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(current, desired))))
        angle = math.acos(dot)
        if angle < 1e-9:
            break
        axis = _unit([current[1] * desired[2] - current[2] * desired[1],
                      current[2] * desired[0] - current[0] * desired[2],
                      current[0] * desired[1] - current[1] * desired[0]])
        half = angle / 2.0
        sin_h = math.sin(half)
        delta = (axis[0] * sin_h, axis[1] * sin_h, axis[2] * sin_h, math.cos(half))
        # World-frame correction, so it pre-multiplies.
        q = quat_multiply(delta, q)
    return q


def reduce_yaw(yaw_rad, symmetry):
    """Fold a block's yaw into the smallest equivalent rotation.

    A square looks the same every 90 degrees, so there is no reason to twist the
    wrist 80 degrees when 10 the other way is the same grasp. Beyond saving
    motion this keeps joint6output_to_joint6 away from its -2.4434 rad limit,
    which _is_near_joint_limit() rejects outright -- an unreduced yaw is a way
    to make IK fail for reasons that look like nothing to do with yaw.
    """
    if not symmetry:
        # 0 = continuous, i.e. "the detector thinks this is round, so any yaw
        # grasps it". Returning 0.0 here was the obvious reading and it is not
        # safe, because symmetry 0 is not a measurement of roundness -- it is
        # what _classify() returns for ANY un-elongated blob whose fill_ratio
        # falls below CIRCLE_FILL_MAX (0.86). A square with slightly rounded
        # corners, or one whose contour picked up its own shadow, lands there.
        #
        # Hardware 2026-08-02: the 30 mm SQUARE block was classified "circle" in
        # two of three survey stills and "square" in the third, so its fill_ratio
        # sits right on that threshold. The fused answer was "circle".
        #
        # The asymmetry of the mistake is the point. If it really is round, any
        # yaw works, so folding mod 90 costs nothing. If it is actually a square,
        # forcing yaw 0 on a block sitting at 45 deg drives the jaws at its
        # DIAGONAL -- 1.41x the footprint, which for a 37 mm block is 52 mm and
        # may not fit the jaws at all. Folding mod 90 is correct in that case and
        # harmless in the other, so fold.
        #
        # Not folded to 0 and not left unreduced: mod 90 bounds the result to
        # +/-45 deg, which keeps joint6output_to_joint6 clear of its -2.4434 rad
        # limit exactly as the docstring above requires.
        symmetry = 4
    period = 2.0 * math.pi / symmetry
    return (yaw_rad + period / 2.0) % period - period / 2.0


class Detector:
    """Client for the Pi's /detect_block service."""

    def __init__(self, node, zone_x, zone_y, zone_z, zone_yaw, zone_size):
        self.node = node
        self.zone_x = zone_x
        self.zone_y = zone_y
        self.zone_z = zone_z
        self.zone_yaw = zone_yaw
        self.zone_size = zone_size
        self.client = node.create_client(DetectBlock, "detect_block")
        # Identity travels on the topic, not in the response: DetectBlock.srv
        # reports contours, and a contour cannot say which block it is. See
        # detection_wire.py's "WHY BLOCK TAGS ARE ON THE WIRE AT ALL".
        try:
            self.wire = wire.DetectionSubscriber(node)
        except Exception as exc:                        # noqa: BLE001
            print("[detect] no %s subscriber (%s) -- block IDENTITY will be "
                  "unavailable, positions are unaffected" % (wire.TOPIC, exc))
            self.wire = None
        self.last_block_tags = []
        # The wire's own block list from the last still, kept for the COLOUR
        # join in _response_to_detections. Same frame, same order as the service
        # response, so the join is by index -- and checked there.
        self.last_wire_blocks = []

    def wait_for_service(self, timeout=15.0):
        if self.client.wait_for_service(timeout_sec=timeout):
            return True
        print("[detect] /detect_block never appeared in %.0fs.\n"
              "         Is block_detector_node.py running ON THE PI?\n"
              "         Remember `ros2 node list` does not show the robot's "
              "nodes from mars even when they are up (PROJECT_CONTEXT.md) -- "
              "check with `ros2 service list | grep detect_block` instead."
              % timeout)
        return False

    def detect(self, zone="pickup", debug_image=None):
        request = DetectBlock.Request()
        request.zone = zone
        request.zone_x = float(self.zone_x)
        request.zone_y = float(self.zone_y)
        request.zone_z = float(self.zone_z)
        request.zone_yaw = float(self.zone_yaw)
        request.zone_size = float(self.zone_size)
        request.expected_size = 0.0
        request.save_debug_image = bool(debug_image)
        request.debug_image_path = debug_image or ""

        if self.wire is not None:
            # Cleared so a stale message from the detector's own free-running
            # loop, taken at the PREVIOUS pose, cannot be mistaken for this
            # still. The node publishes the topic before it builds the reply,
            # so the matching message normally lands during the call below.
            self.wire.latest = None
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=DETECT_SERVICE_TIMEOUT)
        if not future.done():
            print("[detect] service call timed out after %.0fs"
                  % DETECT_SERVICE_TIMEOUT)
            return None

        response = future.result()
        status = "OK" if response.success else "FAIL"
        print("[detect] %s: %s" % (status, response.message))
        # px/m IS a distance gauge -- see PX_M_INVARIANT. Printing it here
        # means a soft frame announces itself instead of being inferred later
        # from a tag that would not decode.
        distance = (PX_M_INVARIANT / response.scale_px_per_m
                    if response.scale_px_per_m > 0 else float("nan"))
        floor_note = ""
        if response.scale_px_per_m > 0 and distance < FOCUS_FLOOR_M:
            floor_note = ("  <-- %.0f mm BELOW the %.0f mm focus floor; this "
                          "frame is soft" % ((FOCUS_FLOOR_M - distance) * 1000,
                                             FOCUS_FLOOR_M * 1000))
        print("[detect] tags %s | rms %.2f px | %.0f px/m = %.4f m from the mat "
              "| camera at zone (%+.1f, %+.1f) mm%s"
              % (list(response.tag_ids), response.homography_rms,
                 response.scale_px_per_m, distance,
                 response.camera_zx * 1000.0, response.camera_zy * 1000.0,
                 floor_note))
        for index, block in enumerate(response.blocks):
            print("[detect]   [%d] zone (%+.1f, %+.1f) mm  yaw %+.1f deg  "
                  "%.1f x %.1f mm  %s sym=%d"
                  % (index, block.zx * 1000, block.zy * 1000,
                     math.degrees(block.yaw), block.width * 1000,
                     block.length * 1000, block.shape, block.symmetry))
        self.last_block_tags = self._collect_block_tags()
        self.last_wire_blocks = self._collect_wire_blocks()
        for index, b in enumerate(self.last_wire_blocks):
            if getattr(b, "colour", "unknown") != "unknown":
                print("[detect]   [%d] colour %s (%.2f)"
                      % (index, b.colour, b.colour_score))
        for tag in self.last_block_tags:
            where = ("zone (%+.1f, %+.1f) mm" % (tag.zone_xy[0] * 1000,
                                                 tag.zone_xy[1] * 1000)
                     if tag.zone_xy else "no mat-plane position (side face)")
            print("[detect]   tag id %-2d %-22s %s  %.1f px (%.1f px/module%s)"
                  % (tag.tag_id, tag.label, where, tag.px, tag.px_per_module,
                     "" if tag.px_per_module >= BLOCK_TAG_PX_PER_MODULE_GOOD
                     else ", MARGINAL"))
        if not self.last_block_tags:
            print("[detect]   no block tags in this still -- nothing here "
                  "identifies which block is which")
        return response if response.success else None

    def _collect_wire_blocks(self):
        """The wire's blocks from the still just taken. Best effort, like tags.

        Does NOT spin: _collect_block_tags has already waited for this same
        message and left it in self.wire.latest, so spinning again here would
        risk picking up the NEXT free-running publication -- a message from a
        frame the arm has since moved away from, which is precisely the stale
        pairing the index join refuses.
        """
        if self.wire is None or self.wire.latest is None:
            return []
        return list(self.wire.latest.blocks)

    def _collect_block_tags(self):
        """Block tags from the still just taken, off the wire topic.

        Best effort by design: identity failing must never break a detection
        that otherwise worked. A missing topic degrades to "no identity", which
        select_block reports rather than papers over.
        """
        if self.wire is None:
            return []
        deadline = time.monotonic() + WIRE_WAIT_SEC
        while self.wire.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.wire.error:
            print("[detect] %s decode failed (%s) -- is detection_wire.py the "
                  "same version on the Pi?" % (wire.TOPIC, self.wire.error))
            return []
        if self.wire.latest is None:
            return []
        return list(self.wire.latest.block_tags)

    def zone_to_world(self, zx, zy):
        c, s = math.cos(self.zone_yaw), math.sin(self.zone_yaw)
        return self.zone_x + c * zx - s * zy, self.zone_y + s * zx + c * zy

    def zone_delta_to_world(self, dzx, dzy):
        """Rotate a zone-frame DIFFERENCE into world. No translation."""
        c, s = math.cos(self.zone_yaw), math.sin(self.zone_yaw)
        return c * dzx - s * dzy, s * dzx + c * dzy

    def world_to_zone(self, wx, wy):
        c, s = math.cos(self.zone_yaw), math.sin(self.zone_yaw)
        dx, dy = wx - self.zone_x, wy - self.zone_y
        return c * dx + s * dy, -s * dx + c * dy


def _response_to_detections(response, wire_blocks=()):
    """The service's BlockDetection[] as zone_vision.Detection objects.

    fuse_detections only reads zone-local fields, so the world-frame ones are
    left out on purpose -- they are derived from the caller's zone survey and
    would add the survey's error to a comparison between views that all share it.

    COLOUR IS JOINED BY INDEX, from the wire. DetectBlock.srv carries no colour
    -- see detection_wire.py schema 3 for why it stays off the interface -- but
    both lists are built from the same `result.blocks` in the same order in the
    same callback, so index IS the join key and it is exact.

    The join is CHECKED, not assumed. If the two lists differ in length the wire
    message came from a different frame (the node free-runs, and a stale message
    is exactly what Detector.detect clears `latest` to prevent) -- and pairing a
    colour with the wrong contour is worse than having no colour, because it puts
    a confident wrong name on a real position. So: drop the colours, say so, and
    let the caller refuse for want of an identity.
    """
    wire_blocks = list(wire_blocks or ())
    if wire_blocks and len(wire_blocks) != len(response.blocks):
        print("[detect] the wire reported %d block(s) and the service %d -- "
              "not the same frame, so colour is DROPPED for this still rather "
              "than joined to the wrong contour."
              % (len(wire_blocks), len(response.blocks)))
        wire_blocks = []
    out = []
    for index, b in enumerate(response.blocks):
        colour, score = "unknown", 0.0
        if index < len(wire_blocks):
            colour = getattr(wire_blocks[index], "colour", "unknown")
            score = float(getattr(wire_blocks[index], "colour_score", 0.0))
        out.append(zv.Detection(
            zx=b.zx, zy=b.zy, zyaw=b.zyaw, width=b.width, length=b.length,
            shape=b.shape, symmetry=b.symmetry, fill_ratio=b.fill_ratio,
            area_px=b.area_px, box_px=None, colour=colour, colour_score=score))
    return out


def promote_tagged_tops_to_square(detections, block_tags):
    """A contour with a TOP tag on it and a square footprint IS four-fold
    symmetric, whatever the fill ratio said. Mutates in place, returns how many.

    THE BUG THIS FIXES, seen on hardware 2026-08-06 and predicted verbatim in
    block_coordinates' docstring: a 30 mm cube's rounded corners put its fill
    ratio right on CIRCLE_FILL_MAX, so it classifies as `circle` in some stills
    and `square` in others. Majority vote across stills then hands fusion
    symmetry 0, and zone_vision's fuse_detections does this:

        else:
            zyaw = 0.0

    -- the yaw is not averaged badly, it is DISCARDED. grasp yaw comes out 0,
    the wrist never turns, and the jaws close on the block's 44 mm diagonal
    instead of its 31 mm face. Two runs aborted at the hover for exactly this.

    The per-still yaws in those runs were -91.0, -90.0, +88.6, -2.0 degrees:
    reduced mod 90 that is -1.0, 0.0, -1.4, -2.0. The contour's yaw was never
    the problem. Only the fold was missing.

    Deliberately NOT keyed on the class name. "orange_cube" is a label that can
    be renamed without touching hardware, and block_coordinates is explicit that
    nothing infers dimensions from it. The two facts used here are geometric and
    checkable in the frame: a TOP tag proves this is the mat-parallel face, and
    an aspect ratio near 1 proves that face is square. Four-fold symmetry
    follows from those, not from what the block is called.
    """
    tops = [t for t in block_tags
            if t.zone_xy is not None and t.face and t.face.kind == "top"]
    promoted = 0
    for d in detections:
        if d.symmetry == 4:
            continue
        near = [t for t in tops
                if math.hypot(d.zx - t.zone_xy[0], d.zy - t.zone_xy[1])
                <= BLOCK_TAG_MATCH_M]
        if not near:
            continue
        short, long_ = min(d.width, d.length), max(d.width, d.length)
        if long_ <= 0 or short / long_ < SQUARE_TOP_ASPECT_MIN:
            continue
        d.shape, d.symmetry = "square", 4
        promoted += 1
    return promoted


def correct_top_face_parallax(detections, camera_zx, camera_zy, lens_height_m,
                              face_height_m=TOP_FACE_HEIGHT_M):
    """Pull each detection back onto the mat plane the homography actually maps.

    See TOP_FACE_HEIGHT_M. Everything the contour finder measures -- position
    AND footprint -- is magnified about the nadir by h / (h - t), so the inverse
    is one scale factor applied to both. Mutates in place and returns the factor
    so the caller can report it.

    Refuses rather than guesses when the geometry is not sane: a lens height at
    or below the face height would divide by zero or flip the sign, and both
    mean the distance estimate is wrong, not that the block is 10 m wide.
    """
    if lens_height_m <= face_height_m * 1.5:
        return None
    factor = (lens_height_m - face_height_m) / lens_height_m
    for d in detections:
        d.zx = camera_zx + (d.zx - camera_zx) * factor
        d.zy = camera_zy + (d.zy - camera_zy) * factor
        d.width *= factor
        d.length *= factor
    return factor


def survey_flange_for_yaw(detector, yaw_deg, z, iterations=3):
    """Flange (x, y) that puts the LENS over the zone centre at THIS wrist yaw.

    The whole framing problem in one function. The lens sits 40 mm off the
    flange axis, so as the wrist turns it swings around a 40 mm circle -- which
    means there is no single flange position that frames the zone at more than
    one yaw. Using one anyway is what the survey did until 2026-08-05, and the
    arithmetic of that is stark: the fixed flange (0.1876, 0.0020) is the
    correct one for yaw 90, and the survey then took its stills at 150, 180 and
    210, where the correct flange is (0.2085, -0.0321), (0.2285, -0.0375) and
    (0.2485, -0.0322). The lens was 40, 57 and 70 mm off the zone centre. At a
    zone only 101.6 mm across that walks the far tags out of frame, which is
    exactly what the log shows: 2 of 4 tags in two stills out of three.

    Iterated because the orientation depends on the flange position (look_at_quat
    aims FROM the flange) and the offset depends on the orientation. Converges
    in two or three passes -- same fixed point look_at_quat itself runs.

    This is FRAMING ONLY. Getting it wrong loses tags out of frame; it never
    biases the position that comes back, which is measured from the tags.
    """
    fx, fy = detector.zone_x, detector.zone_y
    target = (detector.zone_x, detector.zone_y, detector.zone_z)
    for _ in range(max(1, iterations)):
        q = look_at_quat((fx, fy, z), target, block_yaw_deg=yaw_deg)
        offset = _mat_vec(quat_to_matrix(q), LENS_OFFSET_IN_FLANGE)
        fx, fy = detector.zone_x - offset[0], detector.zone_y - offset[1]
    return fx, fy


def survey_start_flange(detector, z, base_yaw_deg=0.0):
    """Flange for the FIRST still of a multiview survey. -> (x, y)

    Exists so callers cannot disagree with detect_multiview about where the
    survey starts. Both go through survey_flange_for_yaw with the SAME
    bearing-relative yaw -- see the long note in detect_multiview. Before this,
    run_stage1 computed its starting flange from the raw offset while
    detect_multiview used the bearing-relative one, so the first arm move of
    every survey went somewhere the survey then did not want to be.
    """
    bearing = math.degrees(math.atan2(detector.zone_y, detector.zone_x))
    return survey_flange_for_yaw(
        detector, bearing + base_yaw_deg + MULTIVIEW_YAW_OFFSETS_DEG[0], z)


def detect_multiview(io_client, detector, zone, x, y, z, base_yaw_deg,
                     holding_block=False, debug_prefix=None,
                     recentre_lens=True):
    """Several stills at different wrist yaws, fused into one answer.

    Returns (fused_blocks, views_used, tag_ids_union, block_tags) --
    fused_blocks is a list of zone_vision.FusedDetection sorted by how many
    views agreed on them, and block_tags is every block face tag seen in any of
    the stills, deduplicated by id, keeping the sighting with the most pixels
    per module because that is the one whose position is worth trusting.

    A still that fails is logged and skipped rather than aborting the pass: with
    four offsets, losing one to glare or a marginal tag still leaves plenty.
    """
    per_view = []
    ids = set()
    used = 0
    best_tag = {}
    # World (dx, dy) that would put the lens where the TAGS say the zone centre
    # is, learned from the first usable still and reused for the rest.
    #
    # This is framing, and framing is the one job camera_in_zone is
    # unambiguously right for. Using it to correct a grasp POSITION is what
    # --verify does and why --verify is off by default: its ~22 mm offset is
    # uncalibrated, so nulling it there can inject exactly as much error as it
    # removes. Here the goal IS to null that reading -- "put the lens where the
    # tags say the centre is" -- so the measurement and the objective are the
    # same quantity and there is nothing to get wrong. It never touches a block
    # position.
    #
    # Measured 2026-08-06 across nine stills: the lens sat 19.8-27.8 mm off the
    # zone centre, always the same direction. At 2500 px/m that is 62 px, and
    # the zone's half-diagonal is 180 px against the 240 a 480-tall frame
    # allows -- so a corner tag fell outside by two pixels, every time.
    lens_bias = (0.0, 0.0)
    bias_known = False

    # THE WRIST YAWS ARE RELATIVE TO THE MAT'S BEARING, not to world +X.
    #
    # Added 2026-08-12, and it is the whole reason a pickup zone off the +Y axis
    # did not work. MULTIVIEW_YAW_OFFSETS_DEG is an ABSOLUTE wrist angle, but
    # survey_flange_for_yaw puts the flange at `zone centre - lens offset`, and
    # the lens offset direction is set by that absolute angle. So whether a given
    # still pulls the flange IN toward the base or pushes it OUT past the mat
    # depends on the angle between the wrist yaw and the mat's bearing -- and
    # that relationship was only ever right for the bearing it was tuned at.
    #
    # Measured, worst-of-5 flange radius, before -> after:
    #   bearing    0 (N, O, H)      unchanged, to the last digit
    #   bearing  +90 (the standard pickup zone)   0.2640 -> 0.2115   -52.4 mm
    #   bearing  -41 (logs.txt run 4, 9.34 in)    0.2478 -> 0.2200   -27.8 mm
    #   bearing -135 (logs.txt run 2, 9.15 in)    0.2711 -> 0.2152   -55.9 mm
    #
    # Run 2 asked for 0.2253-0.2711 m and every one of its five stills was
    # REFUSED -- "[multiview] NO usable view" -- while a still at 0.2087 on the
    # same day reached fine. The five radii are now bearing-invariant to within
    # the mats' own radius difference, so any bearing frames as well as bearing 0
    # does, which is the case with the track record.
    #
    # Nothing about occlusion is lost: the offsets still span 30-150 deg, so the
    # gripper still rotates 120 deg across the stills, and the set is now fixed
    # relative to the MAT rather than to the world -- so which tag pair hides on
    # which still is reproducible between mat placements instead of depending on
    # where the mat happens to be.
    #
    # base_yaw_deg stays an ADDITIONAL offset on top, so callers keep their say
    # and a caller that passes 0.0 (every one of them) gets this for free rather
    # than having to remember it.
    zone_bearing_deg = math.degrees(math.atan2(detector.zone_y, detector.zone_x))
    print("[multiview] wrist yaws are relative to the mat's bearing %+.1f deg "
          "(see the note in detect_multiview)" % zone_bearing_deg)

    for index, offset in enumerate(MULTIVIEW_YAW_OFFSETS_DEG):
        if used >= MULTIVIEW_ENOUGH_VIEWS:
            print("[multiview] %d usable views, skipping the remaining %d still(s)"
                  % (used, len(MULTIVIEW_YAW_OFFSETS_DEG) - index))
            break

        yaw = zone_bearing_deg + base_yaw_deg + offset
        # Re-centre the LENS for this yaw. See survey_flange_for_yaw: one
        # flange position cannot frame the zone at more than one wrist angle.
        vx, vy = survey_flange_for_yaw(detector, yaw, z)
        vx, vy = vx + lens_bias[0], vy + lens_bias[1]
        print("\n[multiview] still %d/%d at wrist yaw %+.0f deg (offset %+.0f)"
              " -- flange (%.4f, %+.4f), radius %.4f m%s"
              % (index + 1, len(MULTIVIEW_YAW_OFFSETS_DEG), yaw, offset,
                 vx, vy, math.hypot(vx, vy),
                 ", lens re-centred by (%+.1f, %+.1f) mm"
                 % (lens_bias[0] * 1000, lens_bias[1] * 1000) if bias_known
                 else ", lens on the zone centre by model"))
        # look_at_quat, not a straight-down block_yaw_deg move: at DETECT_HOVER_Z
        # straight-down is unreachable (see that constant). yaw still does its
        # original job -- it is passed straight through to look_at_quat's own
        # block_yaw_deg, which rotates the WRIST before the aiming tilt is
        # applied, so it still changes which tag pair the gripper occludes.
        target = (detector.zone_x, detector.zone_y, detector.zone_z)
        # allow_constraint_sampling=False: a still is only worth taking from the
        # pose it was planned for. See move_arm_to. Heights are tried tallest
        # first because the tall one focuses and the short one is known to
        # reach -- degrading to a soft still beats losing the view entirely.
        heights = [z] if z != DETECT_HOVER_Z else [z, DETECT_HOVER_Z_FALLBACK]
        arrived = None
        for candidate in heights:
            q = look_at_quat((vx, vy, candidate), target, block_yaw_deg=yaw)
            if move_arm_to(io_client, vx, vy, candidate,
                           orientation_override=q,
                           holding_block=holding_block,
                           ori_xy_tolerance=DETECT_ORI_XY_TOLERANCE,
                           allow_constraint_sampling=False):
                arrived = candidate
                break
            print("[multiview]   z %.3f out of reach at radius %.4f m"
                  % (candidate, math.hypot(vx, vy)))
        if arrived is None:
            print("[multiview]   no reachable height for this view, skipping. "
                  "The offsets are ordered nearest-first, so a yaw whose "
                  "framing flange is further out is the first to go.")
            continue
        if arrived != z:
            print("[multiview]   fell back to z %.3f -- expect a softer frame "
                  "(focus floor is %.0f mm from the mat)"
                  % (arrived, FOCUS_FLOOR_M * 1000))
        time.sleep(SETTLE_AFTER_MOVE_SEC)

        debug = ("%s_view%d.png" % (debug_prefix, index)) if debug_prefix else None
        response = detector.detect(zone, debug_image=debug)
        if response is None:
            print("[multiview]   no usable homography from this view")
            continue
        if len(response.tag_ids) < MULTIVIEW_MIN_TAGS:
            print("[multiview]   only %d tag(s); %d needed to VOTE on a block "
                  "position. Not fused -- see MULTIVIEW_MIN_TAGS."
                  % (len(response.tag_ids), MULTIVIEW_MIN_TAGS))
            for tag in detector.last_block_tags:
                previous = best_tag.get(tag.tag_id)
                if previous is None or tag.px_per_module > previous.px_per_module:
                    best_tag[tag.tag_id] = tag
            continue

        # Learn the framing correction once, from the first still that saw
        # enough tags to be believed. Capped: a large reading means something
        # other than framing is wrong, and chasing it would walk the arm.
        # THE RE-CENTRING MAY COST MORE THAN IT BUYS, and 2026-08-13 is the
        # first run where the arithmetic was done. It is learned from ONE still
        # at ONE wrist yaw and applied as a fixed world translation to the rest --
        # but the lens residual has a component that ROTATES with the wrist
        # (~11 mm amplitude, fitted over five stills to 0.6/1.4 mm rms), so a
        # constant cannot remove it and is wrong by up to ~22 mm at the other
        # yaws. Meanwhile it pushes the framing flange 13-15 mm further out:
        #
        #   offset  model r   flown r   IK at DETECT_HOVER_Z
        #     +90    0.1822   0.1828    yes
        #     +60    0.1886   0.2031    NO -> fell back to z 0.240
        #    +120    0.1886   0.2016    NO
        #     +30    0.2052   0.2199    NO
        #    +150    0.2050   0.2176    NO
        #
        # Four of five stills lost DETECT_HOVER_Z because of the correction, drop
        # to 0.240, soften, and then fail MULTIVIEW_MIN_TAGS -- which is why only
        # 10 of 20 stills across these logs ever reached the vote.
        #
        # recentre_lens=False turns it off so the two can be COMPARED on hardware
        # in one run. Default unchanged, because this is a hypothesis with good
        # arithmetic behind it and no measurement yet.
        if not recentre_lens:
            if not bias_known:
                print("[multiview]   --no-lens-recentre: NOT correcting the "
                      "framing. Every still flies its modelled flange, which is "
                      "13-15 mm nearer the base and inside DETECT_HOVER_Z's "
                      "reach. Compare the usable-view count against a normal run.")
                bias_known = True
        elif not bias_known:
            measured = math.hypot(response.camera_zx, response.camera_zy)
            if measured <= MAX_LENS_BIAS_M:
                lens_bias = detector.zone_delta_to_world(-response.camera_zx,
                                                         -response.camera_zy)
                bias_known = True
                print("[multiview]   lens measured %.1f mm off the zone centre; "
                      "correcting the remaining stills by (%+.1f, %+.1f) mm"
                      % (measured * 1000, lens_bias[0] * 1000,
                         lens_bias[1] * 1000))
            else:
                print("[multiview]   lens measured %.1f mm off the zone centre, "
                      "past the %.0f mm this will correct. Left alone -- that "
                      "is not a framing error."
                      % (measured * 1000, MAX_LENS_BIAS_M * 1000))

        used += 1
        ids.update(response.tag_ids)
        view = _response_to_detections(response, detector.last_wire_blocks)
        # Before fusing, not after: each still has its own lens height and its
        # own nadir, so the correction is per-still. Fusing first would average
        # views taken at different heights and then apply one wrong factor.
        lens_height = PX_M_INVARIANT / response.scale_px_per_m
        factor = correct_top_face_parallax(view, response.camera_zx,
                                           response.camera_zy, lens_height)
        if factor is None:
            print("[multiview]   lens height %.4f m is not sane against a "
                  "%.0f mm block -- top-face parallax NOT corrected for this "
                  "still" % (lens_height, TOP_FACE_HEIGHT_M * 1000))
        elif view:
            print("[multiview]   top-face parallax: lens %.4f m over a %.0f mm "
                  "block, scaling by %.4f about the nadir "
                  "(%+.1f mm at the zone edge)"
                  % (lens_height, TOP_FACE_HEIGHT_M * 1000, factor,
                     (factor - 1.0) * 50.8))
        # Before fusing, because fusion is where symmetry 0 destroys the yaw.
        promoted = promote_tagged_tops_to_square(view, detector.last_block_tags)
        if promoted:
            print("[multiview]   %d contour(s) carry a TOP tag on a square "
                  "footprint -- symmetry 4, so the yaw survives fusion"
                  % promoted)
        per_view.append(view)
        for tag in detector.last_block_tags:
            previous = best_tag.get(tag.tag_id)
            if previous is None or tag.px_per_module > previous.px_per_module:
                best_tag[tag.tag_id] = tag

    block_tags = [best_tag[k] for k in sorted(best_tag)]

    if not per_view:
        print("[multiview] NO usable view. This is a framing, focus or lighting "
              "problem -- check that any tag is visible at all before "
              "suspecting the geometry.")
        return [], 0, sorted(ids), block_tags

    fused = zv.fuse_detections(per_view)
    print("\n[multiview] %d usable view(s), tags seen across all of them: %s"
          % (used, sorted(ids)))
    for index, f in enumerate(fused):
        flag = "" if f.trustworthy else "  <-- ONE VIEW ONLY, no cross-check"
        print("[multiview]   [%d] zone (%+.1f, %+.1f) mm  yaw %+.1f deg  "
              "%.1f x %.1f mm  %s  views=%d spread=%.1f mm/%.1f deg%s"
              % (index, f.zx * 1000, f.zy * 1000, math.degrees(f.zyaw),
                 f.width * 1000, f.length * 1000, f.shape, f.n_views,
                 f.spread_m * 1000, math.degrees(f.spread_yaw_rad), flag))
    if block_tags:
        print("[multiview] block tags seen across the stills:")
        for tag in block_tags:
            where = ("zone (%+.1f, %+.1f) mm" % (tag.zone_xy[0] * 1000,
                                                 tag.zone_xy[1] * 1000)
                     if tag.zone_xy else "side face, no mat-plane position")
            print("[multiview]   id %-2d %-22s %s  best %.1f px/module"
                  % (tag.tag_id, tag.label, where, tag.px_per_module))
    return fused, used, sorted(ids), block_tags


class CorrectionLog:
    """(commanded correction, measured result) pairs.

    Written for its own sake: this is the dataset the disturbance-observer
    branch needs -- commanded delta against externally-measured delta, at known
    poses -- and collecting it here costs nothing because the loop produces it
    anyway.
    """

    def __init__(self, path):
        self.path = path
        self.rows = []

    def add(self, **row):
        self.rows.append(row)

    def flush(self):
        if not (self.path and self.rows):
            return
        fields = list(self.rows[0].keys())
        exists = os.path.exists(self.path)
        with open(self.path, "a", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            if not exists:
                writer.writeheader()
            writer.writerows(self.rows)
        print("[log] %d correction row(s) -> %s" % (len(self.rows), self.path))


def hover_and_detect(io_client, detector, log, target_zone_xy, block_yaw_deg,
                     hover_height, label, debug_image=None):
    """Put the CAMERA over target_zone_xy, then correct until it is really there.

    Aims via look_at_quat at the ZONE CENTRE (fixed, not target_zone_xy) rather
    than holding a straight-down orientation. This is not optional at
    DETECT_HOVER_Z: straight-down cannot reach that height at this radius at
    all (see DETECT_HOVER_Z). Aiming at the fixed zone centre rather than the
    per-attempt target keeps the orientation identical across correction
    attempts, which is what the plain "flange += delta" correction below
    assumes -- re-aiming at a moving target on every attempt would couple the
    position and orientation corrections together.

    Returns (response, converged, flange_world_xy) or (None, False, None).
    """
    target_world = detector.zone_to_world(*target_zone_xy)
    zone_centre_world = (detector.zone_x, detector.zone_y, detector.zone_z)
    # STARTING flange position only -- the correction loop below is what
    # actually converges on the target. Pulled toward the base by the same
    # MEASURED amount as the survey (DETECT_HOVER_PULLIN_M), not derived from
    # camera_offset_world(): that offset's sign is YAW-DEPENDENT (flips between
    # +41mm and -39mm depending on block_yaw_deg, measured 2026-07-31) and using
    # it to place the flange pushed the survey OUTWARD past anything reachable,
    # failing every IK seed. block_yaw_deg here can be any value the block's
    # detected orientation produces, so that failure mode is not hypothetical
    # for this call either -- look_at_quat aims by rotating, so the flange does
    # not need to sit at a yaw-precise lens-offset distance to begin with.
    flange = _pullin_toward_base(target_world[0], target_world[1],
                                 DETECT_HOVER_PULLIN_M)

    # The pull-in is a fixed 41 mm and takes no account of how far out the
    # target started, so a block on the near side of the zone can land the
    # STARTING pose inside the self-collision floor before a single correction
    # has run. Clamp outward rather than refuse: from the floor the loop still
    # gets a real detection, and it will report honestly if it cannot converge.
    start_radius = math.hypot(flange[0], flange[1])
    if start_radius < MIN_FLANGE_RADIUS_M and start_radius > 1e-6:
        scale = MIN_FLANGE_RADIUS_M / start_radius
        flange = (flange[0] * scale, flange[1] * scale)
        print("\n[%s] starting flange was radial %.4f m, inside the %.4f m "
              "self-collision floor -- clamped outward to (%.4f, %.4f)."
              % (label, start_radius, MIN_FLANGE_RADIUS_M, flange[0], flange[1]))

    print("\n[%s] want camera at zone (%+.1f, %+.1f) mm -> world (%.4f, %.4f)"
          % (label, target_zone_xy[0] * 1000, target_zone_xy[1] * 1000,
             target_world[0], target_world[1]))
    print("[%s] starting flange (%.4f, %.4f), %.0fmm pulled toward base, yaw "
          "%+.1f deg" % (label, flange[0], flange[1],
                        DETECT_HOVER_PULLIN_M * 1000, block_yaw_deg))

    response = None
    for attempt in range(MAX_CORRECTIONS + 1):
        q = look_at_quat((flange[0], flange[1], hover_height), zone_centre_world,
                         block_yaw_deg=block_yaw_deg)
        if not move_arm_to(io_client, flange[0], flange[1], hover_height,
                           orientation_override=q,
                           ori_xy_tolerance=DETECT_ORI_XY_TOLERANCE):
            print("[%s] move failed" % label)
            return None, False, None
        time.sleep(SETTLE_AFTER_MOVE_SEC)

        response = detector.detect(debug_image=debug_image)
        if response is None:
            return None, False, None

        error_zone = (target_zone_xy[0] - response.camera_zx,
                      target_zone_xy[1] - response.camera_zy)
        error = math.hypot(*error_zone)
        print("[%s] attempt %d: camera off target by %.1f mm (%+.1f, %+.1f)"
              % (label, attempt, error * 1000,
                 error_zone[0] * 1000, error_zone[1] * 1000))

        log.add(label=label, attempt=attempt,
                cmd_flange_x=flange[0], cmd_flange_y=flange[1],
                target_zone_x=target_zone_xy[0], target_zone_y=target_zone_xy[1],
                measured_camera_zx=response.camera_zx,
                measured_camera_zy=response.camera_zy,
                error_x=error_zone[0], error_y=error_zone[1], error=error,
                homography_rms=response.homography_rms,
                tags_seen=response.tags_seen, block_yaw_deg=block_yaw_deg)

        # Stopping conditions. Both thresholds are GUESSES until Stage 0a runs
        # -- see CORRECTION_CONVERGED_M / CORRECTION_DEADZONE_M.
        if error <= CORRECTION_CONVERGED_M:
            print("[%s] converged (within %.1f mm)" % (label, CORRECTION_CONVERGED_M * 1000))
            return response, True, flange
        if error <= CORRECTION_DEADZONE_M:
            print("[%s] residual %.1f mm is below the arm's dead-zone floor "
                  "(%.1f mm): it physically cannot correct this. Proceeding "
                  "anyway -- retrying would do nothing."
                  % (label, error * 1000, CORRECTION_DEADZONE_M * 1000))
            return response, True, flange
        if attempt == MAX_CORRECTIONS:
            break

        delta = detector.zone_delta_to_world(*error_zone)
        proposed = (flange[0] + delta[0], flange[1] + delta[1])

        # The correction is a raw "flange += camera error" step and knows nothing
        # about the workspace -- see MIN_FLANGE_RADIUS_M. Check BEFORE commanding
        # it: past this radius the arm collides with itself, and neither IK nor
        # move_group will refuse the goal on our behalf.
        radius = math.hypot(proposed[0], proposed[1])
        if radius > MAX_FLANGE_RADIUS_M:
            print("[%s] REFUSING this correction: it puts the flange at radial "
                  "%.4f m, past the %.4f m the arm can reach at this height. "
                  "Commanding it does not fail cleanly -- IK finds no solution, "
                  "constraint sampling parks the arm SHORT, and the next "
                  "detection reads that shortfall as a still-larger error and "
                  "corrects further out. The loop runs away from the target."
                  % (label, radius, MAX_FLANGE_RADIUS_M))
            print("[%s] hardware 2026-08-02: exactly this, 0.2271 -> 0.2770 -> "
                  "planning FAILED. If the target really is out there, the fix "
                  "is a lower hover or a closer zone, not more corrections."
                  % label)
            return response, False, flange
        if radius < MIN_FLANGE_RADIUS_M:
            print("[%s] REFUSING this correction: it puts the flange at radial "
                  "%.4f m, inside the %.4f m self-collision floor. The arm would "
                  "stall against itself rather than reach it (measured "
                  "2026-07-31 at 0.1209 m)."
                  % (label, radius, MIN_FLANGE_RADIUS_M))
            print("[%s] the camera cannot be brought over zone (%+.1f, %+.1f) mm "
                  "from this approach -- that target is too close to the base. "
                  "Not descending on a %.1f mm error."
                  % (label, target_zone_xy[0] * 1000, target_zone_xy[1] * 1000,
                     error * 1000))
            return response, False, flange

        flange = proposed
        print("[%s] correcting by (%+.1f, %+.1f) mm -> flange (%.4f, %.4f, "
              "radial %.4f m)"
              % (label, delta[0] * 1000, delta[1] * 1000, flange[0], flange[1],
                 radius))

    print("[%s] did NOT converge after %d corrections. Refusing to descend on "
          "a position this uncertain." % (label, MAX_CORRECTIONS))
    return response, False, flange


def drop_zone_furniture(fused, zone_size, tag_size):
    """Fused detections that are the zone's own corner tags, not blocks.

    The blob detector finds the printed AprilTags. They are dark quadrilaterals
    on a light mat, which is exactly what it is looking for, and it has no way
    to know they are furniture. Hardware 2026-08-05: a survey returned four
    "blocks", of which THREE were tag corners --

        [1] zone (-44.3, +44.9) mm  12.4 x 13.1 mm
        [3] zone (+44.5, +45.0) mm  11.4 x 13.0 mm

    -- sitting at the tag positions and roughly the tag's decoded size. That run
    picked the real block only because it happened to be seen in more views. It
    was luck, and with an empty zone the same code descends on a tag.

    The test is position, not size: a candidate whose centre is outside the
    square that the tags leave clear cannot be a graspable block anyway. A block
    further out than this is already covering a tag (see APRIL_TAGS.md 'Usable
    area'), which breaks the homography that measured it.
    """
    limit = zone_size / 2.0 - tag_size / 2.0
    kept, dropped = [], []
    for f in fused:
        (dropped if max(abs(f.zx), abs(f.zy)) > limit else kept).append(f)
    for f in dropped:
        print("[stage1] ignoring %.1f x %.1f mm at zone (%+.1f, %+.1f) mm -- "
              "outside the %.1f mm the tags leave clear, so it is the zone's "
              "own furniture, not a block"
              % (f.width * 1000, f.length * 1000, f.zx * 1000, f.zy * 1000,
                 limit * 1000))
    return kept


# Below this fused colour score a contour is left UNIDENTIFIED rather than
# named. 0.45 is a hue within ~10 units (~20 deg) of a prototype, which on the
# painted wooden set is comfortable; the point of the gate is to catch a shaded
# facet or a half-mat blob landing between two prototypes, where the nearest one
# wins by default and means nothing.
COLOUR_MIN_SCORE = 0.45

# ... and below this fraction of AGREEING views. Two views calling a block two
# different colours is not a weak measurement, it is a contradiction -- the same
# argument select_block makes about disagreeing positions. 0.6 lets 2-of-3
# through and stops 1-of-2.
COLOUR_MIN_AGREE = 0.6


def identify_blocks_by_colour(fused, min_score=COLOUR_MIN_SCORE,
                              min_agree=COLOUR_MIN_AGREE):
    """-> {index into fused: colour name}. The AprilTag-free identity path.

    Same contract as identify_blocks -- an index-keyed dict that select_block
    consumes unchanged -- so the two are interchangeable and the caller picks
    which one supplies identity. That is the whole reason select_block takes an
    `identity` dict rather than reading tags itself.

    THREE REASONS A CONTOUR GOES UNNAMED, each printed, because they want
    different fixes:

      - `unknown`: no prototype was within COLOUR_MAX_HUE_DIST. A colour we have
        no entry for, or an achromatic blob (a white block on a white mat is not
        separable at all -- see zone_vision.find_blocks).
      - low score: a hue sitting between two prototypes. Usually a blob that is
        part mat or part shaded side wall.
      - low agreement: the views disagreed. Lighting, or a merged contour whose
        two blocks are different colours -- which is worth knowing, because that
        contour must not be grasped at all.

    DUPLICATES ARE ALLOWED AND REPORTED, unlike the tag path, where two contours
    claiming one class is a conflict. Two red blocks on the mat is an ordinary
    scene; the caller decides between them on position and agreement, and
    select_block already ranks by agreement.
    """
    identity = {}
    for index, f in enumerate(fused):
        colour = getattr(f, "colour", "unknown")
        score = float(getattr(f, "colour_score", 0.0))
        agree = float(getattr(f, "colour_agree", 1.0))
        where = ("contour %d at zone (%+.1f, %+.1f) mm"
                 % (index, f.zx * 1000, f.zy * 1000))
        if colour == "unknown":
            print("[identify] %s has no colour this build recognises -- left "
                  "unidentified" % where)
            continue
        if score < min_score:
            print("[identify] %s looks %s but only scores %.2f (need %.2f) -- "
                  "left unidentified. A hue between two prototypes is usually a "
                  "blob that is part mat or part shaded side wall."
                  % (where, colour, score, min_score))
            continue
        if agree < min_agree:
            print("[identify] %s: only %.0f%% of its %d view(s) agreed it is %s "
                  "(need %.0f%%) -- left unidentified. Views that disagree on "
                  "colour can also mean ONE contour over TWO blocks."
                  % (where, agree * 100, f.n_views, colour, min_agree * 100))
            continue
        identity[index] = colour
        print("[identify] %s is %s (score %.2f, %.0f%% of %d view(s) agree)"
              % (where, colour, score, agree * 100, f.n_views))

    named = {}
    for index, colour in identity.items():
        named.setdefault(colour, []).append(index)
    for colour, indices in sorted(named.items()):
        if len(indices) > 1:
            print("[identify] NOTE: %d contours are %s (%s). Not a conflict -- "
                  "select_block will rank them by view agreement."
                  % (len(indices), colour,
                     ", ".join(str(i) for i in indices)))
    if not identity:
        print("[identify] nothing was named by colour. Is the detector running "
              "with method:=colour? block_detector_node.py logs the colour of "
              "every contour it finds.")
    return identity


def identify_blocks(fused, block_tags):
    """-> {index into fused: block class}. Prints what it decided and why.

    A contour says where something is; only a tag says WHAT it is. The two are
    joined by proximity, which works here for a specific reason: a top tag is a
    raised point projected onto the mat plane, so it lands a few millimetres
    outward of the contour it belongs to, while two blocks in a 4 in zone are
    tens of millimetres apart. The parallax is much smaller than the spacing --
    see BLOCK_TAG_MATCH_M for the measured numbers.

    SIDE tags are skipped, not matched loosely. A side face stands perpendicular
    to the mat, so it has no mat-plane position at all (zone_xy is None) and
    guessing one from the frame would invent a number. It still identifies that
    the block is PRESENT, which is why it is reported.
    """
    identity = {}
    # Cleared per call, so a stale conflict from the previous survey cannot
    # refuse a grasp in this one. See LAST_IDENTITY_CONFLICTS.
    LAST_IDENTITY_CONFLICTS.clear()
    positioned = [t for t in block_tags if t.zone_xy is not None]
    sideways = [t for t in block_tags if t.zone_xy is None]

    for tag in positioned:
        best_index, best_distance = None, None
        for index, f in enumerate(fused):
            distance = math.hypot(f.zx - tag.zone_xy[0], f.zy - tag.zone_xy[1])
            if best_distance is None or distance < best_distance:
                best_index, best_distance = index, distance
        if best_index is None or best_distance > BLOCK_TAG_MATCH_M:
            print("[identify] %s sits %s from any contour -- not matched"
                  % (tag.label,
                     "%.0f mm" % (best_distance * 1000) if best_distance
                     is not None else "an unmeasurable distance"))
            continue
        existing = identity.get(best_index)
        if existing and existing != tag.face.block_class:
            # Two different blocks' tags claiming one contour means the contour
            # is not one block, or a tag was misread. Neither is safe to grasp.
            print("[identify] CONFLICT on contour %d: both %s and %s claim it. "
                  "Dropping its identity rather than guessing."
                  % (best_index, existing, tag.face.block_class))
            identity[best_index] = None
            # RECORDED, not just dropped. Losing the identity already stops
            # --block-class picking this contour, but --any-block would still
            # happily descend on it -- and the comment above says exactly why
            # that is unsafe: the contour is two blocks touching. This is the
            # definitive merged-contour signal and it was being thrown away.
            LAST_IDENTITY_CONFLICTS.add(best_index)
            continue
        if existing is None and best_index in identity:
            continue
        identity[best_index] = tag.face.block_class
        print("[identify] contour %d at zone (%+.1f, %+.1f) mm is the %s "
              "(%s, %.0f mm away)"
              % (best_index, fused[best_index].zx * 1000,
                 fused[best_index].zy * 1000, tag.face.block_class,
                 tag.label, best_distance * 1000))

    for tag in sideways:
        print("[identify] %s is in frame but is a side face -- it says the %s "
              "is here, not where" % (tag.label, tag.face.block_class))

    return {k: v for k, v in identity.items() if v is not None}


def _ask(prompt):
    """One line from the operator. EOF (piped stdin, no tty) aborts.

    Aborting on EOF rather than proceeding is deliberate: --confirm exists so a
    human sees the number before the arm reaches at it, and a run with no human
    attached has not satisfied that. Use --yes for unattended runs.
    """
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print("\n[confirm] stdin closed -- treating as ABORT. Use --yes to run "
              "without a human.")
        return "q"


def print_block_report(block, block_yaw_world, grasp_x, grasp_y, grasp_z,
                       grasp_yaw_deg, grasp_hover, nudge=(0.0, 0.0),
                       block_class=None):
    """Everything known about where the arm is about to reach, in one place.

    WORLD is what matters and is printed in full -- metres from the base, plus
    the same thing as radius and bearing, because that is how the zone gets
    placed and measured on the bench and comparing 8.92 in against a tape is a
    check anyone can do in five seconds. Zone-local is printed alongside because
    it is the RAW measurement: it comes from the tag homography and owes nothing
    to the zone survey, so when world looks wrong and zone-local looks right,
    the zone origin is what is wrong.
    """
    radius = math.hypot(grasp_x, grasp_y)
    print("\n[confirm] block  %s  %.1f x %.1f mm  %s"
          % (block_class if block_class else "UNIDENTIFIED",
             block.width * 1000, block.length * 1000, block.shape))
    # THE ONE PIECE OF GROUND TRUTH THAT COSTS NOTHING. The block is 30 mm by
    # construction, so the measured footprint is a free check on the whole
    # vision chain -- threshold, contour, homography, parallax -- needing no
    # tape, no caliper and no operator.
    #
    # Earned on 2026-08-09: position G measured 23.0 x 23.5 mm and needed a
    # nudge, while N and O measured ~30 x 30 and did not. A footprint 7 mm
    # short on BOTH axes means the segmentation lost the block's edges, and a
    # blob eroded unevenly moves its own centroid -- so that run's offset was a
    # detection failure, not a calibration point. choose() gates only on view
    # spread and yaw spread, so nothing upstream noticed.
    #
    # WARNING, NOT A REJECTION, and deliberately so: tightening a gate in the
    # middle of a calibration changes which rows survive and makes the day's
    # data un-poolable with the morning's. Print it, let the operator throw the
    # row out, and decide the gate afterwards from the numbers.
    #
    # PER SIDE, short against short: on a non-square block one nominal cannot
    # serve both axes, and comparing the 61 mm side of the green brick against
    # 30 mm reports a 30 mm "error" on a measurement that is right to 1 mm.
    short_nom, long_nom = nominal_footprint(block_class)
    worst = max(abs(min(block.width, block.length) - short_nom),
                abs(max(block.width, block.length) - long_nom))
    if worst > BLOCK_SIZE_WARN_M:
        print("[confirm]   *** FOOTPRINT IS %.1f mm OFF NOMINAL %.0f x %.0f mm "
              "-- the vision lost the block's edges."
              % (worst * 1000, short_nom * 1000, long_nom * 1000))
        print("[confirm]   *** An unevenly eroded blob moves its own centroid, "
              "so treat this run's offset as a DETECTION FAILURE, not data.")
    print("[confirm]   zone-local  (%+.1f, %+.1f) mm      <- raw, straight from "
          "the tags" % (block.zx * 1000, block.zy * 1000))
    print("[confirm]   WORLD       (%.4f, %.4f) m   r %.4f m = %.2f in, "
          "bearing %+.1f deg"
          % (grasp_x, grasp_y, radius, radius / 0.0254,
             math.degrees(math.atan2(grasp_y, grasp_x))))
    print("[confirm]   block yaw   %+.1f deg in world  ->  grasp yaw %+.1f deg "
          "(symmetry %d)"
          % (math.degrees(block_yaw_world), grasp_yaw_deg, block.symmetry))
    print("[confirm]   flange z    hover %.4f -> grasp %.4f, a %.0f mm descent. "
          "Z is NOT measured -- it is PICK_XYZ.z + GRASP_OFFSET_Z."
          % (grasp_hover, grasp_z, (grasp_hover - grasp_z) * 1000))
    if nudge[0] or nudge[1]:
        print("[confirm]   nudged by   (%+.1f, %+.1f) mm of yours, already "
              "included in WORLD above" % (nudge[0] * 1000, nudge[1] * 1000))


def _parse_nudge(answer):
    """(dx_m, dy_m, dyaw_deg) from 'dx dy' or 'dx dy dyaw', or None.

    Millimetres and degrees, because those are the units a person reads off a
    ruler and a protractor at the bench. The yaw term is optional and defaults
    to zero, so every 'dx dy' typed before this existed still means what it did.
    """
    parts = answer.replace(",", " ").split()
    if len(parts) not in (2, 3):
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    return (values[0] / 1000.0, values[1] / 1000.0,
            values[2] if len(values) == 3 else 0.0)


# ---------------------------------------------------------------------------
# Neighbour-aware grasping
# ---------------------------------------------------------------------------
# Nothing in this pipeline looked at what was BESIDE the block it was about to
# grasp, so two blocks in one zone meant the open jaw came down on the neighbour.
# Added 2026-08-12.
#
# THE THREE NUMBERS BELOW ARE NOT MEASURED, and that is stated rather than
# hidden. The repo records a 0.75 rad total jaw span and infers "~5 mm of margin
# over a 30 mm block" -- so ~40 mm of aperture -- and nowhere records the
# aperture, the finger thickness, or the finger width. A clearance rule built on
# a guessed tolerance is a guessed rule, so every run prints that these are
# provisional until JAW_GEOMETRY_MEASURED is set.
#
# To measure, with the gripper at GRIPPER_OPEN:
#   JAW_APERTURE_OPEN_M       inner face to inner face
#   JAW_FINGER_THICKNESS_M    one finger's extent ALONG the closing axis
#   JAW_FINGER_WIDTH_M        one finger's extent ACROSS it
# Then set JAW_GEOMETRY_MEASURED = True and the warning stops.
JAW_APERTURE_OPEN_M = 0.040
JAW_FINGER_THICKNESS_M = 0.008
JAW_FINGER_WIDTH_M = 0.018
JAW_GEOMETRY_MEASURED = False

# Added to every neighbour's radius. The block positions themselves carry
# ~1 mm of fused vision error and the arm lands within ~1 mm, so 2 mm is one
# error bar of slack rather than a comfort blanket.
NEIGHBOUR_SAFETY_M = 0.002

# A footprint at least this long cannot be one 30 mm block, so it is probably
# two of them merged into one contour.
#
# THE MARGIN HERE IS UNCOMFORTABLE AND THE NUMBERS SAY SO. Two touching 30 mm
# blocks read as one 30 x 60 mm blob -- and MAX_BLOCK_LENGTH_M is exactly 60 mm,
# so the touching case is ACCEPTED as a single block whose centroid sits in the
# seam between the two. Meanwhile a genuinely single block has been observed
# reading 34 x 44 mm on this bench (logs.txt 2026-08-12), i.e. 14 mm over
# nominal. So the usable window between "one block, badly measured" and "two
# blocks, merged" is 44-60 mm, and 50 mm splits it with 6 mm either side.
#
# EXPRESSED AS A MARGIN OVER NOMINAL, not as an absolute length, so that the
# same tuning serves a block set with more than one shape. 30 mm nominal + 20 mm
# reproduces the 50 mm above exactly; the 61 mm green brick gets 81 mm. It is the
# margin that was tuned on this bench, and it is the margin that transfers -- an
# absolute 50 mm does not, and refused the green brick on every run of
# 2026-08-13.
#
# BOTH AXES ARE TESTED, and for the cube that changes nothing (if min >= T then
# max >= T, so the long test already dominates). It matters for an ELONGATED
# block: two green bricks touching along their long sides read 61 x 61, whose
# LONGEST side is 61 mm and passes an 81 mm long-side test. It is the SHORT side,
# 61 against a nominal 30.5, that gives it away. On a cube those two merge
# geometries are the same rectangle; on a brick they are not.
#
# WHAT NO FOOTPRINT TEST CAN DO. Two touching 30.5 mm cubes and one 61 x 30.5 mm
# brick are the SAME RECTANGLE. There is no threshold that separates them,
# which is precisely why the guard must be told which block it is looking at
# rather than inferring it from size. That is nominal_footprint's job.
#
# This is the FALLBACK signal. The definitive one is two different blocks' TOP
# tags matching the same contour, which identify_blocks already detects; see
# LAST_IDENTITY_CONFLICTS. Prefer that whenever tags are on the blocks, and rely
# on this only for the untagged/colour path.
MERGED_MARGIN_M = 0.020

# Contours that two different block classes both claimed, by index into the
# fused list. Written by identify_blocks, read by merged_contour_reason.
#
# A module-level scratch rather than a second return value, matching
# pick_place.LAST_FLANGE_FK / LAST_ARM_GOAL: identify_blocks' {index: class}
# return is consumed in three places and none of them wants a tuple.
LAST_IDENTITY_CONFLICTS = set()


def block_radius(detection):
    """Conservative disc radius for a detection -- its half-diagonal.

    A DISC, not the block's own rectangle, and deliberately so: the neighbour's
    YAW is the least trustworthy number available about it. A 30 mm square has
    classified as `circle` with symmetry 0 (yaw discarded to 0.0) in two stills
    of three on this hardware, and a footprint has read 34 x 44 mm when it is
    30 x 30. Rotating a rectangle by a yaw that may be 45 deg wrong turns a
    safety check into a coin flip; a disc is yaw-free and errs outward.
    """
    half_diagonal = math.hypot(detection.width, detection.length) / 2.0
    return half_diagonal


def jaw_footprint_rects(grip_width_m):
    """The two rectangles the OPEN fingers occupy, in the jaw frame.

    Jaw frame: +along is the closing axis, +across is perpendicular, origin at
    the target block's centre. Each finger is a plate whose face is normal to
    the closing axis, so along that axis it is only its THICKNESS, and across it
    is its WIDTH -- which is the asymmetry the whole rule turns on. A neighbour
    sitting along the closing axis blocks the grasp; the same neighbour sitting
    across it usually does not, and rotating the wrist 90 deg swaps which.

    grip_width_m is the target's extent along the closing axis -- the SHORT side,
    since that is the face the jaws must span. max() against the aperture keeps
    the geometry sane for a block too wide to grip at all, which is a different
    refusal and not this function's job.
    """
    half = max(JAW_APERTURE_OPEN_M, grip_width_m) / 2.0
    across = JAW_FINGER_WIDTH_M / 2.0
    return ((half, half + JAW_FINGER_THICKNESS_M, -across, across),
            (-half - JAW_FINGER_THICKNESS_M, -half, -across, across))


def _disc_rect_gap(cx, cy, rect):
    """Shortest distance from a point to an axis-aligned rectangle. 0 if inside."""
    a0, a1, c0, c1 = rect
    return math.hypot(max(a0 - cx, 0.0, cx - a1), max(c0 - cy, 0.0, cy - c1))


def grasp_clearance(target, others, jaw_axis_deg):
    """Room for the open jaws around `target` at this jaw axis.

    -> (ok, margin_m, blocker_index). margin is the smallest gap between a
    finger and a neighbour: negative means overlap, and its magnitude is how far
    into the finger the neighbour reaches. blocker_index indexes `others`.

    FRAME: jaw_axis_deg must be measured in the SAME frame as the detections'
    zx/zy -- zone-local if these are FusedDetections. Everything here is
    differences between block positions, so the frame cancels as long as the
    angle agrees with the coordinates. Passing a WORLD wrist yaw against
    zone-local positions is a silent rotation error of exactly zone_yaw, which on
    this bench is ~90 deg -- i.e. it would check the wrong axis entirely. The
    callers convert; see run_stage1.
    """
    if not others:
        return True, None, None
    ca = math.cos(math.radians(jaw_axis_deg))
    sa = math.sin(math.radians(jaw_axis_deg))
    rects = jaw_footprint_rects(min(target.width, target.length))
    worst, blocker = None, None
    for index, other in enumerate(others):
        dx, dy = other.zx - target.zx, other.zy - target.zy
        along = dx * ca + dy * sa
        across = -dx * sa + dy * ca
        need = block_radius(other) + NEIGHBOUR_SAFETY_M
        for rect in rects:
            gap = _disc_rect_gap(along, across, rect) - need
            if worst is None or gap < worst:
                worst, blocker = gap, index
    return worst >= 0.0, worst, blocker


# Offset from the block's MAJOR (long) axis to the wrist yaw commanded for the
# grasp, degrees.
#
# MEASURED 2026-08-13. It is 90, and here is the observation that settled it.
#
# What was always verified: zone_vision's zyaw is the footprint's MAJOR axis (see
# the note in find_blocks, and test_rectangle asserts it), and the jaws must close
# across the SHORT side, because that is the only side narrower than the aperture
# on most of this block set. So the closing axis has to end up perpendicular to
# the major axis.
#
# What was NOT verified until today: whether pick_place's `block_yaw_deg` names
# the CLOSING AXIS or the BLOCK'S MAJOR AXIS. The two differ by exactly 90 deg,
# and for a 30 mm CUBE -- every block this project had grasped, on every run,
# ever -- reduce_yaw(.., 4) folds 90 deg away, so both conventions produced the
# identical wrist angle and no run could distinguish them.
#
# THE GREEN BRICK DISTINGUISHED THEM. First colour stack, 2026-08-13:
#
#   [stack] green: 30.5 x 60.2 mm rect ... yaw +0.2 -> grasp yaw +0.2 (symmetry 2)
#   [grip]  Its LONG axis lies at +0.2 deg in the world; the wrist is commanded
#           to +0.2 deg.
#   [confirm] ENTER = go  'dx dy' = nudge ... > 0 0 -90
#
# The operator turned the wrist 90 deg to grasp it, which is the whole question
# answered: `block_yaw_deg` is the BLOCK'S MAJOR AXIS, so the closing axis needs
# +90 on top of it. Elongated and symmetry 2, so nothing folded the answer away.
#
# 90 AND NOT -90, and they are the same thing here: the closing axis is a LINE,
# period 180 deg, so +90 and -90 name one axis. reduce_yaw then folds by the
# block's own symmetry. The operator typed -90 because that was the shorter turn
# from where the wrist already was.
#
# WHAT THIS CHANGES AND WHAT IT DOES NOT. On a symmetry-4 block -- every tagged
# cube, every validated calibration row -- reduce_yaw(.., 4) has period 90, so
# adding 90 lands on the identical wrist angle and NO existing result moves. It
# only changes blocks that are not 4-fold, which before today could not be
# grasped at all without an operator turning the wrist by hand.
#
# AND NOTE WHAT DOES *NOT* CATCH THIS: grip_span_ok checks the block's geometry,
# not the wrist. It refuses a block whose short side is wider than the aperture,
# which is a different question -- with the offset wrong, a 61 x 30.5 mm block
# passes the span check and the jaws still close on the long axis. Only the
# operator at the park caught it, which is what happened.
GRASP_YAW_FROM_MAJOR_DEG = 90.0


def grasp_yaw_report(block, grasp_yaw_deg, zone_yaw_deg, label="block"):
    """Print what the wrist is about to do against what the block needs.

    Exists so the GRASP_YAW_FROM_MAJOR_DEG question was answerable by looking,
    rather than by reasoning about a convention nothing had written down. It was
    answered on 2026-08-13, so this now CHECKS the answer instead of asking it:
    the two angles have to come out perpendicular, and that is arithmetic anyone
    can read off the line above.
    """
    major_world = math.degrees(block.zyaw) + zone_yaw_deg
    print("[grip] %s: %.1f mm short side x %.1f mm long side. Its LONG axis "
          "lies at %+.1f deg in the world; the wrist is commanded to %+.1f deg."
          % (label, block.width * 1000, block.length * 1000,
             major_world, grasp_yaw_deg))
    if block.width < block.length * 0.9:
        # How far off perpendicular, as an angle between two LINES -- so the
        # answer lives in [0, 90] and 90 is correct.
        gap = abs((grasp_yaw_deg - major_world + 90.0) % 180.0 - 90.0)
        if gap > 80.0:
            print("[grip]   ELONGATED and the wrist is %.0f deg off the long "
                  "axis, so the fingers span the %.1f mm side. That is what "
                  "GRASP_YAW_FROM_MAJOR_DEG = %.0f is for."
                  % (gap, block.width * 1000, GRASP_YAW_FROM_MAJOR_DEG))
        else:
            print("[grip]   *** ELONGATED and the wrist is only %.0f deg off the "
                  "long axis -- the fingers would close on the %.1f mm side, "
                  "which is %.1f mm wider than they open."
                  % (gap, block.length * 1000,
                     (block.length - JAW_APERTURE_OPEN_M) * 1000))
            print("[grip]   *** GRASP_YAW_FROM_MAJOR_DEG is %.0f. It was measured "
                  "as 90 on 2026-08-13; if it has been changed back, that is why."
                  % GRASP_YAW_FROM_MAJOR_DEG)


def grip_span_ok(block, aperture_m=None, label="block"):
    """Can the jaws open wide enough for this block's SHORT side? -> (ok, span_m)

    The jaws close across the short side (see jaw_footprint_rects), so `width` is
    the span they have to open to. This is a check on the BLOCK, not on the
    wrist -- see GRASP_YAW_FROM_MAJOR_DEG for why that distinction matters and
    what this does not protect against.

    JAW_APERTURE_OPEN_M IS A GUESS (JAW_GEOMETRY_MEASURED is False). So this is
    a REFUSAL WITH THE NUMBERS PRINTED, not a silent filter: the operator can see
    both figures and decide, and one caliper reading turns the guess into a fact.
    """
    if aperture_m is None:
        aperture_m = JAW_APERTURE_OPEN_M
    span = float(block.width)
    ok = span <= aperture_m
    if not ok:
        print("[grip] REFUSING the %s: the jaws must close across its SHORT "
              "side, %.1f mm, and the open aperture is %.1f mm. Its long side "
              "is %.1f mm."
              % (label, span * 1000, aperture_m * 1000, block.length * 1000))
        print("[grip]   Turn it onto a narrower face. On this block set every "
              "graspable dimension is 1.4 in (35.6 mm) or less -- 1.6 in is "
              "40.6 mm and already over.")
        if not JAW_GEOMETRY_MEASURED:
            print("[grip]   NOTE: JAW_APERTURE_OPEN_M is an UNMEASURED estimate. "
                  "Caliper the open jaws at GRIPPER_OPEN and set "
                  "JAW_GEOMETRY_MEASURED = True before trusting this refusal.")
    return ok, span


def candidate_jaw_axes_deg(base_deg, symmetry):
    """Jaw axes that grasp this block identically, base first.

    A jaw axis is a LINE, so it repeats every 180 deg and not every 360. That is
    what decides how much freedom a block gives:

      symmetry 4 (a square face)  -> base and base+90 are DIFFERENT axes, both
                                     valid grasps. Two chances to dodge a
                                     neighbour, and this is the case that makes
                                     "try the other orientation" work.
      symmetry 2 (a rectangle)    -> base+180 is the SAME axis. There is no
                                     alternative: the jaws must span the short
                                     face, so the grasp is either reachable or
                                     it is not.
      symmetry 1                  -> one axis, same as above.
      symmetry 0                  -> reduce_yaw folds this to 4 (see its
                                     comment: symmetry 0 means "un-elongated
                                     blob", not "measured round"), so it gets
                                     both axes too.
    """
    if symmetry in (0, 4):
        return [base_deg, base_deg + 90.0]
    return [base_deg]


def choose_jaw_axis(target, others, base_deg, symmetry, label="block"):
    """Pick a jaw axis with room for the fingers. -> (axis_deg, ok)

    Prefers `base_deg`: it is the measured orientation and costs no extra wrist
    travel. Only when that one is blocked does it try the alternative, and it
    says so -- a silent 90 deg rotation would be a surprising thing to watch the
    arm do.
    """
    if not JAW_GEOMETRY_MEASURED:
        print("[clearance] NOTE: jaw aperture %.0f mm, finger %.0f x %.0f mm are "
              "NOT MEASURED -- see JAW_GEOMETRY_MEASURED. Treat a pass as "
              "provisional and watch the first descent."
              % (JAW_APERTURE_OPEN_M * 1000, JAW_FINGER_THICKNESS_M * 1000,
                 JAW_FINGER_WIDTH_M * 1000))
    if not others:
        print("[clearance] %s is alone in the zone -- nothing to collide with."
              % label)
        return base_deg, True

    axes = candidate_jaw_axes_deg(base_deg, symmetry)
    results = []
    for axis in axes:
        ok, margin, blocker = grasp_clearance(target, others, axis)
        results.append((axis, ok, margin, blocker))
        print("[clearance] %s at jaw axis %+.1f deg: %s (%.1f mm %s"
              "%s)"
              % (label, axis, "CLEAR" if ok else "BLOCKED",
                 abs(margin) * 1000,
                 "of room" if margin >= 0 else "of overlap",
                 "" if blocker is None else ", nearest neighbour #%d" % blocker))
    for axis, ok, _margin, _blocker in results:
        if ok:
            if axis != base_deg:
                print("[clearance] %s: the measured axis %+.1f is blocked, so "
                      "grasping at %+.1f instead -- a %.0f deg rotation, which "
                      "is the SAME grasp on a %d-fold face."
                      % (label, base_deg, axis, axis - base_deg, symmetry or 4))
            return axis, True

    best = max(results, key=lambda r: r[2] if r[2] is not None else -1e9)
    print("[clearance] %s: NO jaw axis has room. Best was %+.1f deg, still "
          "%.1f mm into a neighbour." % (label, best[0], abs(best[2]) * 1000))
    # WHICH WAY TO MOVE IT, because "move the blocks apart" is not enough
    # information and following it in the wrong direction makes things worse.
    #
    # The fingers are JAW_FINGER_THICKNESS_M along the closing axis and
    # JAW_FINGER_WIDTH_M across it, so the two directions cost very differently.
    # Measured on the 2026-08-13 numbers, with the green brick as the target:
    # a neighbour 34 mm away ACROSS the closing axis clears by +8.7 mm, while one
    # 40 mm away ALONG it is still 11.3 mm INTO the fingers. Distance is the wrong
    # variable and it is the one an operator reaches for.
    blocker = _decompose_blocker(target, others, best[0], best[3])
    if blocker is not None:
        along_mm, across_mm, index = blocker
        print("[clearance] neighbour #%d sits %.0f mm ALONG the closing axis and "
              "%.0f mm across it. The fingers are %.0f mm thick along that axis "
              "and %.0f mm wide across, so ALONG is the expensive direction."
              % (index, abs(along_mm), abs(across_mm),
                 JAW_FINGER_THICKNESS_M * 1000, JAW_FINGER_WIDTH_M * 1000))
        print("[clearance] MOVE IT OFF THE END of the target instead of beside "
              "it -- i.e. increase the %.0f mm and let the %.0f mm shrink. "
              "Sliding it further along the closing axis buys much less per mm."
              % (abs(across_mm), abs(along_mm)))
    if len(axes) == 1:
        print("[clearance] This block's footprint is %d-fold, so base+180 is the "
              "SAME jaw axis -- there is no alternative orientation to try. Move "
              "the blocks apart AS ABOVE, or pick the neighbour first."
              % (symmetry or 1))
    else:
        print("[clearance] Both axes are blocked. In a %.0f mm zone the usable "
              "box for a block centre is only %.0f mm across, so two 30 mm "
              "blocks can be at most %.0f mm apart -- there may simply not be "
              "room. Pick the more isolated block first."
              % (zv.DEFAULT_ZONE_SIZE * 1000,
                 (zv.DEFAULT_ZONE_SIZE - zv.DEFAULT_TAG_SIZE
                  - pp.BLOCK_HEIGHT_M) * 1000,
                 (zv.DEFAULT_ZONE_SIZE - zv.DEFAULT_TAG_SIZE
                  - pp.BLOCK_HEIGHT_M) * 1000))
    return best[0], False


def _decompose_blocker(target, others, jaw_axis_deg, index):
    """(along_mm, across_mm, index) for the neighbour that blocked, or None.

    Same frame as jaw_footprint_rects: +along is the closing axis, +across is
    perpendicular, origin at the target's centre. Exists only so the refusal can
    name a DIRECTION -- see the note at its call site for why distance alone is
    misleading advice.
    """
    if index is None or index < 0 or index >= len(others):
        return None
    other = others[index]
    dx = other.zx - target.zx
    dy = other.zy - target.zy
    c, s = (math.cos(math.radians(jaw_axis_deg)),
            math.sin(math.radians(jaw_axis_deg)))
    return ((dx * c + dy * s) * 1000.0, (-dx * s + dy * c) * 1000.0, index)


def merged_contour_reason(detection, index, label=None):
    """Why this contour is probably two blocks rather than one, or None.

    TWO SIGNALS, strongest first.

    1. TWO BLOCK CLASSES CLAIMED IT. identify_blocks already computes this and
       already says "the contour is not one block, or a tag was misread. Neither
       is safe to grasp" -- and then only dropped the contour's IDENTITY, leaving
       it a perfectly good grasp candidate for --any-block. Definitive, needs no
       threshold, and free.

    2. THE FOOTPRINT IS TOO BIG TO BE ONE BLOCK. The fallback for untagged
       blocks, and the only signal the colour path will have. Both axes are
       compared against THIS block's nominal footprint plus MERGED_MARGIN_M --
       see that constant for why a margin and not an absolute length, and why
       the short side has to be tested as well as the long one.

    `label` names the block so nominal_footprint can look its size up; None
    means "assume the 30 mm cube", which is what the tag path wants.

    Returns a string to print and refuse on, or None.
    """
    if index in LAST_IDENTITY_CONFLICTS:
        return ("two different block classes' TOP tags both matched this "
                "contour, so it is two blocks touching, not one")
    short_nom, long_nom = nominal_footprint(label)
    short_seen = min(detection.width, detection.length)
    long_seen = max(detection.width, detection.length)
    for seen, nominal, axis in ((long_seen, long_nom, "long"),
                                (short_seen, short_nom, "short")):
        if seen >= nominal + MERGED_MARGIN_M:
            return ("its %s footprint side is %.0f mm, past the %.0f mm at "
                    "which a %.0f mm side becomes implausible -- two blocks "
                    "touching read as one blob whose centroid sits in the seam "
                    "between them"
                    % (axis, seen * 1000, (nominal + MERGED_MARGIN_M) * 1000,
                       nominal * 1000))
    return None


def select_block(fused, identity=None, want_class=None):
    """Pick the block to descend on, ranked by AGREEMENT rather than by count.

    zone_vision.fuse_detections sorts purely by -n_views, and taking [0] from
    that was wrong on hardware 2026-07-31 in a way that would have driven the
    jaws at a strip of tape:

        [0]  9.3 x 23.1 mm  views=3  spread=10.3 mm/136.6 deg   <-- was chosen
        [3] 37.5 x 45.4 mm  views=2  spread= 0.8 mm/  0.2 deg   <-- the block

    Three views "agreeing" to within 10 mm and 137 deg are not agreeing at all;
    they are three unrelated slivers landing in one match radius. The real
    block, fused to 0.8 mm and 0.2 deg, lost on view count alone.

    So a candidate has to EARN its extra views: if the views that produced it
    disagree, that is evidence against it, not for it. Ordering is

        1. multi-view candidates whose views actually agree   (best)
        2. single-view candidates, unverified but not contradicted
        3. multi-view candidates whose views disagree         (never)

    Rank 3 sits below rank 2 deliberately. A disagreeing fusion is not a weak
    measurement, it is a wrong one -- distinct objects averaged into a position
    matching neither. An honest single view is worth more.
    """
    if want_class is not None:
        identity = identity or {}
        wanted = [f for i, f in enumerate(fused) if identity.get(i) == want_class]
        unknown = [f for i, f in enumerate(fused) if i not in identity]
        if not wanted:
            print("[stage1] asked for the %s and NO contour carries its tag. "
                  "Refusing to pick." % want_class)
            if unknown:
                print("[stage1]   %d contour(s) are unidentified. A block whose "
                      "top tag did not decode looks exactly like the other "
                      "block from above, so picking one would be a coin flip.\n"
                      "[stage1]   Check the tag is stuck on, facing up and lit; "
                      "block_detector_node.py logs px/module for every decode."
                      % len(unknown))
            return None
        print("[stage1] %d of %d contour(s) identified as the %s"
              % (len(wanted), len(fused), want_class))
        fused = wanted

    agreeing, single, disagreeing = [], [], []

    for f in fused:
        if f.n_views < 2:
            single.append(f)
            continue

        why = None
        if f.spread_m > MULTIVIEW_MAX_SPREAD_M:
            why = ("position spread %.1f mm exceeds %.1f mm"
                   % (f.spread_m * 1000, MULTIVIEW_MAX_SPREAD_M * 1000))
        elif (f.symmetry != 0
              and math.degrees(f.spread_yaw_rad) > MULTIVIEW_MAX_SPREAD_YAW_DEG):
            # symmetry == 0 is a circle: yaw is meaningless, any spread is fine.
            why = ("yaw spread %.1f deg exceeds %.1f deg"
                   % (math.degrees(f.spread_yaw_rad),
                      MULTIVIEW_MAX_SPREAD_YAW_DEG))

        if why is None:
            agreeing.append(f)
        else:
            disagreeing.append((f, why))

    for f, why in disagreeing:
        print("[stage1] rejected %.1f x %.1f mm at zone (%+.1f, %+.1f) mm: "
              "%d views but %s -- they are not looking at one object."
              % (f.width * 1000, f.length * 1000, f.zx * 1000, f.zy * 1000,
                 f.n_views, why))

    if agreeing:
        # Most views first, then tightest agreement as the tie-break.
        agreeing.sort(key=lambda f: (-f.n_views, f.spread_m))
        best = agreeing[0]
        print("[stage1] block confirmed across %d views, spread %.1f mm / "
              "%.1f deg" % (best.n_views, best.spread_m * 1000,
                            math.degrees(best.spread_yaw_rad)))
        return best

    if single:
        # Largest footprint, not first: among unverified candidates the slivers
        # shed by tag borders and tape edges are exactly the small ones.
        single.sort(key=lambda f: -(f.width * f.length))
        best = single[0]
        print("[stage1] WARNING: the chosen block was seen in only 1 view, no "
              "cross-check. Position may be less reliable than usual.")
        return best

    return None


# Fingertip clearance above the block's TOP FACE at the measurement park, in
# metres. Set from --measure-clearance-mm.
#
# REFERENCED TO THE TOP FACE, NOT THE GRASP, and the difference is a whole
# BLOCK_HEIGHT_M/2. At the grasp the fingertips sit level with the block CENTRE,
# 15 mm BELOW the top face, because that is what straddling a block means. So
# "grasp + 15 mm" is flush with the top face and would touch it; the number in
# this flag is the gap you can actually see, and it stays meaningful if the block
# size changes.
MEASURE_CLEARANCE_M = 0.008

# The block's true edge length. Not a tuning knob -- it is what was printed.
BLOCK_NOMINAL_M = 0.030
# How far the measured footprint may sit from nominal before it is called out.
# 4 mm is wide on purpose: view-to-view footprint scatter of 1-2 mm is normal at
# these ranges, and the failure this is aimed at was 7 mm on both axes.
BLOCK_SIZE_WARN_M = 0.004

# NOMINAL FOOTPRINTS FOR THE COLOUR PATH, (short side, long side) in metres.
#
# WHY THIS HAD TO EXIST. Every "is this footprint plausible" check in this file
# used to compare against BLOCK_NOMINAL_M alone, which is right for the tag path
# -- every tagged block on this bench is a 30 mm cube -- and structurally wrong
# for the colour path, whose whole point is a block set with different shapes.
# On 2026-08-13 it cost two full hardware runs: the green brick is 30.5 x 61.0 mm
# BY CONSTRUCTION, read 29.7 x 60.3 (within 1 mm, a good measurement), and was
# refused as "two blocks touching" because 60 mm is past the 50 mm at which one
# 30 mm block becomes implausible. It is not a 30 mm block. Nothing was wrong
# except the number it was compared against.
#
# ONE POSE PER BLOCK, the least-tall one -- the scope cut of 2026-08-13. Both of
# these are 30.5 mm tall in that pose, which is what lets one --block-thickness
# serve a stack of two different blocks.
#
# THESE ARE READ OFF THE AMAZON SHEET, NOT MEASURED HERE. 1.2 in = 30.5,
# 1.4 in = 35.6, 2.4 in = 61.0. The blue is a hexagonal PRISM: 30.5 across the
# flats, so its bounding rectangle is 30.5 x 35.2 and the sheet's 1.4 in is the
# across-corners figure. Add a colour by adding a row; an unlisted label falls
# back to the cube, which is what the tag path wants.
#
# THIS TABLE IS BENCH STATE, NOT A BLOCK LIBRARY, and the distinction bit on
# 2026-08-13. It is keyed by COLOUR, and the set has several blocks of each
# colour -- two reds, two blues, two greens. So an entry is only correct while
# that particular block is the one on the mat. When the demo pair changed from
# "green brick + blue frustum" to "red trapezoid + blue frustum", the `green` row
# stopped describing anything present. Update the rows when the blocks change; a
# stale row is worse than a missing one, because a missing one falls back to the
# cube and says so.
COLOUR_FOOTPRINT_M = {
    "green": (0.0305, 0.0610),   # 1.2 x 1.2 x 2.4 in brick, lying down
    "blue": (0.0305, 0.0356),    # 1.2 in frustum, standing: 1.2 x 1.4 in base
    "red": (0.0305, 0.0356),     # 1 in trapezoid, sitting: 1.2 x 1.4 in base
}

# HEIGHT in the same assumed rest pose, metres. Read off the same sheet.
#
# WHY IT IS SEPARATE FROM THE FOOTPRINT: the footprint answers "is this one block
# and can the jaws span it", the height answers "how far up is the next level".
# Different consumers, and one of them is the stack step.
#
# 30.5 mm, NOT the 30.0 of pick_place.BLOCK_HEIGHT_M -- and that 0.5 mm is real.
# The first colour stack, 2026-08-13, put the blue down on a green whose top face
# is at MAT_SURFACE_Z + 30.5 while the level-1 arithmetic assumed 30.0, so the
# release was 0.5 mm INSIDE the block below before the arm's own z error was
# added. See stack_blocks.PLACE_DROP_M for the rest of that number.
# THESE ARE NOT ALL THE SAME, which is the whole reason height_at takes a
# sequence. Red 25.4 under blue 30.5 is the 2026-08-13 demo pair.
COLOUR_HEIGHT_M = {
    "green": 0.0305,             # 1.2 in brick, lying down
    "blue": 0.0305,              # 1.2 in frustum, standing
    "red": 0.0254,               # 1 in trapezoid -- 5.1 mm SHORTER than the rest
}

# Blocks whose sides SLOPE. Recorded rather than acted on: the footprint the
# camera measures is somewhere between the base and the (smaller, and optically
# magnified) top face, so its nominal is a range and not a number, and a block
# clamped on a slope is driven up and out of the jaws by its own taper.
#
# BOTH of the 2026-08-13 demo blocks are in here, so the operator should watch the
# first close on each. What makes the pair work anyway: the taper NARROWS upward,
# and the jaws close 15 mm up, so they meet a section narrower than the measured
# footprint -- the error is toward a looser grip, not a wider one. Stacking ONTO a
# tapered block is the risk, because its top face is the small end.
COLOUR_TAPERED = ("red", "blue", "yellow", "purple")


def nominal_height(label=None):
    """Block height in metres for a label, falling back to the 30 mm cube."""
    if label:
        found = COLOUR_HEIGHT_M.get(str(label).lower())
        if found:
            return found
    return pp.BLOCK_HEIGHT_M


def nominal_footprint(label=None):
    """(short side, long side) nominal footprint in metres for a block label.

    Falls back to the 30 mm cube for anything not in COLOUR_FOOTPRINT_M, which
    is every tag-path block and every colour whose dimensions are not yet on
    the sheet. The fallback is the OLD behaviour exactly, so nothing that used
    to pass starts failing.
    """
    if label:
        found = COLOUR_FOOTPRINT_M.get(str(label).lower())
        if found:
            return found
    return (BLOCK_NOMINAL_M, BLOCK_NOMINAL_M)

# WHY THIS EXISTS. The standard hover is APPROACH_HEIGHT = 40 mm above the grasp,
# which puts the fingertips 25 mm above the block's top face. Judging a lateral
# offset from up there, by eye, was measured on 2026-08-07 as the weakest link in
# the whole pipeline -- and it is the number every calibration constant is fitted
# from. 8 mm of clearance puts the jaws beside a face you can sight along.
#
# NOT A DESCENT ONTO THE BLOCK. The tips stop ABOVE the top face and never come
# down beside it, so the ~5 mm jaw-opening margin over a 30 mm block is never in
# play. That margin is the reason the park is not simply put at grasp height.


_GIT_SHA = []


def measure_park_z(hover_z):
    """Flange z for the measurement park, or None to stay at the hover.

    None when the clearance is disabled, or when the hover is already at or
    below it -- hover_z_for's reach clamp only ever LOWERS the hover, so at a far
    corner the hover can already be under this height and "lowering" to it would
    be a lift, re-arming the very slack the unidirectional approach just settled.
    """
    if MEASURE_CLEARANCE_M <= 0:
        return None
    top_face = pp.MAT_SURFACE_Z + pp.BLOCK_HEIGHT_M
    z = top_face + MEASURE_CLEARANCE_M + pp.GRASP_OFFSET_Z
    return z if z < hover_z - 1e-4 else None


def _git_sha():
    """Short HEAD sha, with '+dirty' when the tree has uncommitted changes.

    Cached, because it shells out and save_calibration is on the path of every
    run. Returns None rather than raising: a row that cannot name its commit is
    strictly better than a grasp that dies at the end.
    """
    if _GIT_SHA:
        return _GIT_SHA[0]
    sha = None
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        sha = subprocess.check_output(
            ["git", "-C", here, "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL).decode().strip()
        dirty = subprocess.check_output(
            ["git", "-C", here, "status", "--porcelain"],
            stderr=subprocess.DEVNULL).decode().strip()
        if dirty:
            sha += "+dirty"
    except Exception:                                   # noqa: BLE001
        sha = None
    _GIT_SHA.append(sha)
    return sha


def model_provenance(args=None):
    """Which model produced this row. Recorded on EVERY calibration row.

    WHY THIS EXISTS. save_calibration recorded twenty fields and not one of them
    said which constants were live. The 61 rows of history are consequently
    un-poolable: rows 44-52 show radial nudges of +20 to +25 mm and rows 53-57
    show +4 to +5, because JAW_RADIAL_OFFSET_M changed underneath them and
    nothing marks where. The parallax correction and the yaw-fold are code
    changes with no marker at all, and ten rows carry a hand-added `invalid`
    field that nothing in the repo writes or reads.

    On a day that fits constants in the morning and verifies them in the
    afternoon, that failure repeats within hours unless the row says which side
    of the change it is on. The git sha covers every code change at once; the
    named constants cover the ones that get tuned without a commit (they are all
    read at call time as module globals, which is what --jaw-tangential-offset
    and friends rely on).

    ORIGIN_RADIAL_BIAS_M is read out of sys.modules rather than imported:
    tag_pick_place does not depend on explore, only explore_pick_place does, and
    adding the import edge to record one float would make a cycle. When explore
    is not loaded the survey did not come from it, so None is the honest value.
    """
    explore = sys.modules.get("explore")
    return {
        "git": _git_sha(),
        # Tool geometry and the corrections layered on it.
        "GRASP_OFFSET_Z": pp.GRASP_OFFSET_Z,
        "DESCENT_BIAS_Z": pp.DESCENT_BIAS_Z,
        "JAW_RADIAL_OFFSET_M": pp.JAW_RADIAL_OFFSET_M,
        "JAW_TANGENTIAL_OFFSET_M": pp.JAW_TANGENTIAL_OFFSET_M,
        # The tangential term is a LINE in reach as of 2026-08-11, so the base
        # alone no longer identifies it -- a row without the slope cannot be
        # reproduced. This is exactly the provenance gap that made the first 61
        # rows un-poolable.
        "JAW_TANGENTIAL_PER_M_REACH": pp.JAW_TANGENTIAL_PER_M_REACH,
        # TOOL-frame, so it rotates with the commanded wrist yaw while the two
        # above rotate with the bearing. A row missing this cannot be compared
        # with one taken at a different grasp yaw.
        "JAW_PERP_OFFSET_M": pp.JAW_PERP_OFFSET_M,
        "GRIPPER_YAW_DEG": pp.GRIPPER_YAW_DEG,
        "GRIPPER_MOUNT_TILT_X_DEG": pp.GRIPPER_MOUNT_TILT_X_DEG,
        "GRIPPER_MOUNT_TILT_Y_DEG": pp.GRIPPER_MOUNT_TILT_Y_DEG,
        "SAG_PRECOMP_RADIAL_DEG": pp.SAG_PRECOMP_RADIAL_DEG,
        "MAT_SURFACE_Z": pp.MAT_SURFACE_Z,
        "BLOCK_HEIGHT_M": pp.BLOCK_HEIGHT_M,
        # J1 lost motion. Both matter and they are not independent:
        # J1_RESIDUAL_BIAS_DEG is applied ONLY inside j1_unidirectional_approach,
        # so it is inert on any move that did not ask for it.
        "J1_UNIDIRECTIONAL_ENABLED": pp.J1_UNIDIRECTIONAL_ENABLED,
        "J1_RESIDUAL_BIAS_DEG": pp.J1_RESIDUAL_BIAS_DEG,
        "J1_APPROACH_DIR": pp.J1_APPROACH_DIR,
        # The survey side. None when explore was not the source of the origin.
        "ORIGIN_RADIAL_BIAS_M": (getattr(explore, "ORIGIN_RADIAL_BIAS_M", None)
                                 if explore is not None else None),
        "DETECT_HOVER_Z": DETECT_HOVER_Z,
        # HOW FAR AWAY THE OPERATOR WAS WHEN THEY JUDGED THE OFFSET, in mm of
        # fingertip clearance above the block's top face. This is not a detail.
        #
        # Measured 2026-08-07: at the old 40 mm hover the fingertips sit 25 mm
        # above the top face, and judging a lateral offset from there produced a
        # repeatable +4.7 mm radial reading (spread 0.46 over three runs) where a
        # caliper at 8 mm reads +0.53. Not noise -- a 9.5 deg viewing angle, and
        # the reported sign was OPPOSITE to the arm's true error, which FK puts
        # at 1.2 mm INWARD. The whole "5.8 mm nudge" this session set out to
        # remove was mostly that.
        #
        # So a nudge is only as good as the distance it was read from, and
        # without this field the history cannot tell a 0.5 mm measurement from a
        # 5 mm illusion. Fit nudge rows only where this is small.
        "measure_clearance_mm": MEASURE_CLEARANCE_M * 1000.0,
    }


def run_stage1(io_client, detector, args, log):
    # DETECT_HOVER_Z, not MAX_HOVER_Z: this is the height that actually FOCUSES
    # (measured 2026-07-30/31 -- see the constant). MAX_HOVER_Z = 0.205m was the
    # reachable ceiling for a STRAIGHT-DOWN camera, and every hover at or below
    # it produced a Laplacian focus metric of 27-84 -- too blurred to decode a
    # tag even pointed straight at one. Height was never the limiting factor;
    # focus distance was, and DETECT_HOVER_Z + look_at_quat (see hover_and_detect
    # and detect_multiview) is what makes that height reachable at all.
    hover = DETECT_HOVER_Z
    print("[stage1] detect hover height %.4f m (measured focus floor is ~0.220m "
          "lens height; this clears it)" % hover)

    if not go_home(io_client):
        return False
    if not io_client.gripper_move_to(GRIPPER_OPEN):
        print("[stage1] could not open the gripper")
        return False

    # --- 1. survey the zone: FOUR stills, wrist rotated 90deg between them,
    #        fused into one answer -----------------------------------------
    # This is the actual answer to the gripper occlusion problem (the gripper
    # hangs in front of the lens and hides 2 of the 4 tags from any single
    # still): rotate the wrist between stills so a DIFFERENT pair is hidden
    # each time, solve each still independently, then fuse. See
    # zone_vision.analyze_multi's docstring for why independently matters on
    # this arm specifically -- pooling correspondences across stills would
    # need to trust how far the wrist actually turned, which the worn servo
    # gearing makes exactly the wrong thing to trust (APRIL_TAGS.md ROOT CAUSE).
    # Starting point only. Each still now moves the flange to whatever puts the
    # LENS on the zone centre at ITS wrist yaw -- see survey_flange_for_yaw,
    # and the table above MULTIVIEW_YAW_OFFSETS_DEG for why one fixed flange
    # cannot do that job.
    survey_flange = survey_start_flange(detector, hover)
    print("[stage1] survey: %d stills, flange re-centred per wrist yaw so the "
          "lens is over the zone centre in every one" %
          len(MULTIVIEW_YAW_OFFSETS_DEG))
    debug_prefix = (os.path.splitext(args.debug_image)[0]
                    if args.debug_image else None)
    fused_blocks, views_used, tags_union, block_tags = detect_multiview(
        io_client, detector, "pickup",
        survey_flange[0], survey_flange[1], hover, 0.0,
        debug_prefix=debug_prefix)

    if views_used == 0:
        print("[stage1] no still saw enough tags to fuse. This is a framing, "
              "focus or lighting problem -- check the debug frames before "
              "suspecting the geometry.")
        return False
    if not fused_blocks:
        print("[stage1] zone is empty -- nothing to pick.")
        return False

    fused_blocks = drop_zone_furniture(fused_blocks, args.zone_size,
                                       args.tag_size)
    if not fused_blocks:
        print("[stage1] every candidate was the zone's own tags -- nothing to "
              "pick.")
        return False

    identity = identify_blocks(fused_blocks, block_tags)

    # Truth first, and against EVERY candidate. The vision error needs no arm:
    # a block's zone-local position comes from the tag homography, so comparing
    # it against a block placed on a known zone-local point measures the vision
    # alone. Printed here rather than after selection so a survey that gets
    # REJECTED still tells you how good it was -- on 2026-08-05 a run failed its
    # agreement gate and produced no diagnosis at all, which is the one time the
    # number is most wanted.
    truth_zone = (tuple(v / 1000.0 for v in args.truth_block_zone)
                  if args.truth_block_zone else None)
    truth_world = tuple(args.truth_block_world) if args.truth_block_world else None
    if truth_zone is not None:
        print("[stage1] against the truth you supplied, zone (%+.1f, %+.1f) mm:"
              % (truth_zone[0] * 1000, truth_zone[1] * 1000))
        for index, f in enumerate(fused_blocks):
            dzx, dzy = f.zx - truth_zone[0], f.zy - truth_zone[1]
            print("[stage1]   [%d] %-12s zone (%+5.1f, %+5.1f) mm  VISION error "
                  "%5.1f mm   views=%d spread=%.1f mm"
                  % (index, identity.get(index) or "unidentified",
                     f.zx * 1000, f.zy * 1000, math.hypot(dzx, dzy) * 1000,
                     f.n_views, f.spread_m * 1000))

    block = select_block(fused_blocks, identity, args.block_class)
    if block is None:
        print("[stage1] no candidate survived the agreement check -- nothing "
              "here is measured well enough to descend on.")
        return False
    # block.yaw does not exist on a FusedDetection -- fusion happens entirely in
    # ZONE-LOCAL coordinates (see _response_to_detections), deliberately, so
    # comparing views never mixes in the caller's own zone-survey error. World
    # yaw is what grasp_quat_for actually needs (it rotates the jaws about
    # world +Z), so convert explicitly here rather than at the FusedDetection
    # boundary -- and explicitly, not by relying on zone_yaw being 0.0 today,
    # which would silently break the moment a zone is surveyed at an angle.
    #
    # GRASP_YAW_FROM_MAJOR_DEG was missing from THIS path while stack_blocks had
    # it, so the two files would have disagreed about the wrist the moment the
    # constant stopped being zero. It is 90 as of 2026-08-13. On a symmetry-4
    # block reduce_yaw's 90 deg period absorbs it exactly, so every tagged-cube
    # result is unchanged.
    block_yaw_world = (block.zyaw + detector.zone_yaw
                       + math.radians(GRASP_YAW_FROM_MAJOR_DEG))
    grasp_yaw = reduce_yaw(block_yaw_world, block.symmetry)
    grasp_yaw_deg = math.degrees(grasp_yaw)
    # YAW OUT OF THE LOOP, on request, for x/y calibration runs.
    #
    # The measured yaw has two ways to go wrong on a 4-fold block, and both bit
    # on 2026-08-11 when the gripper came up 22.5 deg off world X:
    #
    #   1. FUSION DEGENERACY. Folding a symmetry-4 yaw maps 90 deg onto a full
    #      circle, so two views disagreeing by 45 deg are ANTIPODAL and their
    #      circular mean is undefined -- zone_vision._circular_mean has an
    #      explicit s~0 and c~0 guard for exactly this. 22.5 deg is the midpoint
    #      of 0 and 45, i.e. the fingerprint of that degeneracy.
    #   2. SYMMETRY MISCLASSIFICATION. The surveyed zone_yaw flipped sign
    #      between 2026-08-08 (-90.2) and 2026-08-09 (+88.9). At symmetry 4 that
    #      is absorbed (180 mod 90 = 0) and stays invisible; if a marginal
    #      footprint ever classifies the square as "rect" (symmetry 2, period
    #      180) it is NOT absorbed and the grasp yaw jumps ~90 deg. Footprints
    #      have been marginal -- position G measured 23.0 x 23.5 mm.
    #
    # For a calibration sweep neither risk is worth carrying: the block is
    # oriented the same way every run by hand, so its yaw is KNOWN and does not
    # need measuring. Pinning it also fixes the jaw axis to a world axis, which
    # is the difference between a caliper gap that means something and one that
    # is measuring a 22.5 deg projection of an unknown mixture of x and y.
    #
    # NOT for picking real blocks in anger -- there the measured yaw is the whole
    # point and this flag must stay off.
    if getattr(args, "force_grasp_yaw", None) is not None:
        grasp_yaw_deg = float(args.force_grasp_yaw)
        grasp_yaw = math.radians(grasp_yaw_deg)
        print("[stage1] --force-grasp-yaw: commanding %+.1f deg instead of the "
              "measured %+.1f deg. The jaw axis is now pinned to a known world "
              "direction." % (grasp_yaw_deg, math.degrees(reduce_yaw(
                  block_yaw_world, block.symmetry))))
    print("\n[stage1] block: %.1f x %.1f mm %s, zone (%+.1f, %+.1f) mm, "
          "yaw %+.1f deg -> grasp yaw %+.1f deg (symmetry %d)"
          % (block.width * 1000, block.length * 1000, block.shape,
             block.zx * 1000, block.zy * 1000, math.degrees(block_yaw_world),
             grasp_yaw_deg, block.symmetry))

    # --- 1b. is this contour one block, and can the jaws get to it? --------
    chosen_index = next((i for i, f in enumerate(fused_blocks) if f is block),
                        None)
    # The identity, where there is one, names the block so the guard measures
    # against the right nominal footprint. On the tag path that is always a
    # 30 mm cube and the lookup is a no-op; on --by-colour it is the difference
    # between grasping the green brick and refusing it. See COLOUR_FOOTPRINT_M.
    merged = (merged_contour_reason(block, chosen_index,
                                    label=identity.get(chosen_index))
              if chosen_index is not None else None)
    if merged and not getattr(args, "ignore_merged", False):
        print("[stage1] REFUSING to grasp this contour: %s." % merged)
        print("[stage1] Its centroid is not on a block, so descending would put "
              "the jaws in the gap. Separate the blocks, or pass "
              "--ignore-merged if you are certain and watching.")
        return False
    if merged:
        print("[stage1] --ignore-merged: proceeding despite '%s'." % merged)

    # THE JAW AXIS IS CHECKED IN THE ZONE FRAME, because that is the frame the
    # detections live in. grasp_yaw_deg is a WORLD wrist yaw, and the two differ
    # by the surveyed zone yaw -- ~90 deg on this bench, so mixing them would
    # check an axis perpendicular to the real one and pass exactly the layouts it
    # should refuse. Convert here, once, and hand grasp_clearance a zone angle.
    others = [f for i, f in enumerate(fused_blocks) if i != chosen_index]
    zone_yaw_deg = math.degrees(detector.zone_yaw)
    axis_zone_deg, clear = choose_jaw_axis(
        block, others, grasp_yaw_deg - zone_yaw_deg, block.symmetry,
        label=args.block_class or "block")
    if not clear and not getattr(args, "ignore_clearance", False):
        print("[stage1] REFUSING to descend: the open jaws would strike a "
              "neighbouring block. Nothing has moved.")
        print("[stage1] --ignore-clearance overrides, and the jaw geometry is "
              "unmeasured (JAW_GEOMETRY_MEASURED), so a refusal here may be "
              "conservative -- but check the bench before overriding it.")
        return False
    if not clear:
        print("[stage1] --ignore-clearance: descending anyway. WATCH THE JAWS.")
    # Back to world for everything downstream. Applied even when the axis did not
    # change, so there is exactly one path and no branch to get wrong.
    rotated = axis_zone_deg + zone_yaw_deg
    if abs(rotated - grasp_yaw_deg) > 1e-6:
        grasp_yaw_deg = rotated
        grasp_yaw = math.radians(grasp_yaw_deg)
        print("[stage1] grasp yaw is now %+.1f deg to clear the neighbour."
              % grasp_yaw_deg)

    usable = args.zone_size / 2.0 - args.tag_size / 2.0 - max(block.width, block.length) / 2.0
    off_centre = max(abs(block.zx), abs(block.zy))
    if off_centre > usable:
        print("[stage1] WARNING: block centre is %.1f mm off, past the %.1f mm "
              "at which it starts covering a tag. See APRIL_TAGS.md 'Usable "
              "area'." % (off_centre * 1000, usable * 1000))
        # REFUSE the one combination that cannot be a real block. Outside the
        # usable area the block is covering the very tags the homography needs,
        # so a lone view claiming a position out there has nothing corroborating
        # it and nothing constraining it.
        #
        # Both halves of this were already WARNINGS and both fired on the same
        # run -- 2026-08-11 11:39, views=1, spread=0.0, block placed 52.4 mm from
        # the surveyed centre, 2.3x outside the usable area. It was accepted, the
        # arm drove there, and the operator hand-dragged the jaws (-35, +50) mm to
        # the real block. Two warnings that each mean "probably wrong" add up to
        # certainly wrong, and the run cost 8 minutes of a short day.
        #
        # Deliberately narrow: it needs BOTH a single view AND an impossible
        # position, so no legitimately fused detection can trip it, including the
        # Q2 in-zone corners at +-20 mm (inside the 23.1 mm usable area anyway).
        if getattr(block, "n_views", 2) < 2:
            print("[stage1] REFUSING: a SINGLE view placing the block outside "
                  "the usable area has nothing corroborating it. Out there the "
                  "block covers the tags the homography is built on, so this is "
                  "a tag border or tape edge, not the block.")
            print("[stage1] Re-run. If it repeats, fix framing/lighting -- do "
                  "not nudge your way to the block, it records as arm error.")
            return False

    # --- 2. work out where to put the JAWS --------------------------------
    # The tags already answer this. The block's zone-local position came from
    # the homography, the zone's world pose is surveyed, so the block's world
    # position is known without the arm having measured anything. That is the
    # whole point of the design, and it is why the default path below is
    # open loop: it is the SAME thing pick_place.py did successfully for weeks
    # against a hardcoded PICK_XYZ, with a measured target substituted for the
    # hardcoded one.
    grasp_x, grasp_y = detector.zone_to_world(block.zx, block.zy)
    grasp_z = args.grasp_z              # PICK_XYZ.z + GRASP_OFFSET_Z, unchanged
    # Provisional: recomputed below, AFTER --verify may have moved grasp_x/y.
    # The hover height depends on the radius, so it cannot be finalised until
    # the target is.
    grasp_hover = hover_z_for(grasp_x, grasp_y, grasp_z, grasp_yaw_deg)
    print("\n[stage1] block world position from the tags: (%.4f, %.4f)"
          % (grasp_x, grasp_y))

    # --- 2a-bis. what the truth, if supplied, already says ----------------
    # Printed before anything moves, because the VISION error needs no arm at
    # all: the block's zone-local position comes from the tag homography, so
    # comparing it against a block placed on a known zone-local point measures
    # the vision on its own. It is the only number in this pipeline with no
    # confounds -- see calibration.py.
    if truth_world is not None:
        dx, dy = grasp_x - truth_world[0], grasp_y - truth_world[1]
        radius = math.hypot(truth_world[0], truth_world[1]) or 1.0
        radial = (dx * truth_world[0] + dy * truth_world[1]) / radius
        lateral = (-dx * truth_world[1] + dy * truth_world[0]) / radius
        print("[stage1] OPEN-LOOP error %.1f mm (radial %+.1f, lateral %+.1f) "
              "-- vision and zone survey together, before the arm moves."
              % (math.hypot(dx, dy) * 1000, radial * 1000, lateral * 1000))

    # --- 2b. OPTIONAL camera-verified correction (--verify) ---------------
    #
    # OFF BY DEFAULT, and the reason is a real measurement, not caution.
    #
    # The survey commands the flange to a position that should put the lens
    # exactly over the zone centre (DETECT_HOVER_PULLIN_M == the lens offset at
    # the detection yaw). Hardware 2026-08-02: the detector then measured the
    # camera at zone (+21.5, +4.3) mm. Something is ~22 mm out -- but the run
    # cannot say WHICH of two things it is:
    #
    #   (a) the ARM is 22 mm from where it was told to go, in which case the
    #       open-loop grasp below misses by 22 mm and this correction fixes it;
    #   (b) the CAMERA MODEL is 22 mm out -- the 41 mm lens offset, the
    #       uncalibrated principal point, or the optical axis not being normal
    #       to the mat. zone_vision.camera_in_zone's own docstring flags all
    #       three and says the absolute number is uncalibrated while the CHANGE
    #       between hovers is not. In this case the open-loop grasp is fine and
    #       applying the correction INJECTS 22 mm of error.
    #
    # Both produce the identical reading, so guessing has a 50% chance of making
    # the grasp worse. Resolve it by measurement instead: --dry-run now parks
    # the arm over its own answer at grasp height, so one photo says which it is.
    # Until that photo exists, prefer the path with a track record.
    #
    # RESOLVED 2026-08-05 IN FAVOUR OF (b), by explore.py rather than by a photo.
    # explore fits the zone origin from 11 views across a 25 deg base pan and
    # lands 24 mm too far out, against a hand-placed 9 in -- same sign and
    # essentially the same magnitude as the 22 mm here, but at a completely
    # different pose, joint configuration and view tilt. An ARM positioning
    # error is pose-specific and would not reproduce itself at both; a CAMERA
    # MODEL error is carried by the camera and does. See STACKED_BLOCKS_GUIDE.md
    # ("zone_yaw is solved, not assumed" and the section after it).
    #
    # The good news is in WHERE that error can reach. camera_in_zone maps the
    # IMAGE CENTRE through the homography, and the image centre is the one point
    # whose zone coordinate depends on the principal point being where we assume
    # it is. A block's position does not: it is measured from the block's own
    # pixels, interpolated inside the tag square, and the homography is fitted
    # from the tag corners. So the open-loop target below is untouched by this
    # 22 mm, and --verify -- which is built on camera_in_zone -- would inject it.
    # That is now a reason, not a caution.
    if args.verify:
        # Target the camera AT the block, not at block + lens offset.
        #
        # BUG this replaces, hardware 2026-08-02: the old target was
        # block + camera_offset_world(), the idea being that when the camera
        # arrived the FLANGE would be on the block. Arithmetically true, and
        # unreachable -- it puts the flange at the block's own radius, 0.2271 m
        # at DETECT_HOVER_Z, where all 19 IK seeds failed. Constraint sampling
        # then satisfied its 4 cm position sphere by parking the arm 33 mm low
        # (independently confirmed: 2891 px/m against the survey's 2466 puts the
        # lens at 0.191 m, BELOW the 0.220 m focus floor), the loop read that
        # shortfall as a 55 mm position error, and "corrected" outward to
        # radial 0.2770 -- further out of reach -- until planning failed.
        #
        # Aiming the camera at the block instead puts the flange 41 mm inside it,
        # ~0.187 m, which is the radius the survey already converges at on real
        # seeds. Converging the camera over the block also kills the parallax
        # bias for free: a point directly under the lens projects to its true
        # position whatever its height, so the final detection is unbiased in a
        # way the survey's fused answer is not.
        detect_yaw_deg = grasp_yaw_deg + DETECT_WRIST_YAW_DEG
        response, converged, conv_flange = hover_and_detect(
            io_client, detector, log, (block.zx, block.zy),
            detect_yaw_deg, hover, "grasp-hover",
            debug_image=args.debug_image)
        if response is None or not converged:
            print("[stage1] --verify did not converge. NOT falling back to the "
                  "open-loop target silently: the whole reason to ask for "
                  "verification is that you did not want to trust it unchecked.")
            return False

        # The loop learned "commanding conv_flange puts the LENS on the block".
        # The lens sits camera_offset_world() from the flange, so the true flange
        # is at block - offset, and the arm's error is (block - offset) -
        # conv_flange. Putting the true flange ON the block therefore means
        # commanding block - error == conv_flange + offset.
        offset = camera_offset_world(detect_yaw_deg, conv_flange[0], conv_flange[1])
        corrected = (conv_flange[0] + offset[0], conv_flange[1] + offset[1])
        print("[stage1] --verify: open-loop said (%.4f, %.4f), camera-corrected "
              "says (%.4f, %.4f) -- a %.1f mm difference"
              % (grasp_x, grasp_y, corrected[0], corrected[1],
                 math.hypot(corrected[0] - grasp_x, corrected[1] - grasp_y) * 1000))
        print("[stage1] NOTE: this correction is measured at the DETECTION pose "
              "(z %.3f, wrist yaw %+.0f) and applied at the GRASP pose (z %.3f, "
              "wrist yaw %+.0f). Sag and dead-zone are pose-dependent, so this "
              "is an extrapolation, not a measurement of the grasp pose itself."
              % (hover, detect_yaw_deg, grasp_hover, grasp_yaw_deg))
        grasp_x, grasp_y = corrected
        # The target moved, so the radius moved, so the hover ceiling moved.
        # Recomputed here rather than left stale: the reach envelope shrinks
        # with height, and a correction that pushes the block outward can put a
        # hover that was reachable outside it -- which then fails as an IK miss
        # at the hover and reads like a reach problem at the block.
        grasp_hover = hover_z_for(grasp_x, grasp_y, grasp_z, grasp_yaw_deg)

    print("\n[stage1] grasp target: world (%.4f, %.4f, %.4f), yaw %+.1f deg"
          % (grasp_x, grasp_y, grasp_z, grasp_yaw_deg))
    print("[stage1] approach hover %.4f m, then a %.0f mm straight-down descent"
          % (grasp_hover, (grasp_hover - grasp_z) * 1000))
    print("[stage1] descent height is PICK_XYZ.z + GRASP_OFFSET_Z = %.4f, the "
          "same flange height pick_place.py grasps this block at. Stage 1 "
          "measures X, Y and yaw; Z is not measured and is not recomputed."
          % GRASP_FLANGE_Z)

    # --- 3. approach, descend, grasp, retreat -----------------------------
    #
    # move_arm_to the hover FIRST, then descend straight down. The descent used
    # to run as a single cartesian_move_to from wherever the detection left the
    # arm, which is 41 mm to the side and 100+ mm up -- a long diagonal through
    # the workspace rather than the vertical approach GRASP_OFFSET_Z is defined
    # against, and the one motion most likely to clip the block on the way in.
    # --- 2c. the operator checkpoint --------------------------------------
    #
    # The number above is the whole answer, and until now the only way to look
    # at it before the arm moved was --dry-run, which parks over the block and
    # then ENDS THE RUN. So the descent could never be checked: you either
    # inspected the hover and got nothing else, or you ran blind through the
    # grasp. That is the gap this closes -- inspect, then continue.
    #
    # The nudge exists because there is a known, unresolved ~22 mm systematic
    # offset between where the camera model says things are and where they are
    # (see the (a)-vs-(b) note above and STACKED_BLOCKS_GUIDE.md). Typing the
    # offset you can SEE is a measurement; it is also the only way to complete a
    # grasp today without first calibrating it out. It is applied in world X/Y,
    # printed, and never persisted -- nothing here writes a calibration.
    nudge_total = [0.0, 0.0]
    # A list, not a float, so the confirm loops can mutate it the same way
    # nudge_total is mutated -- they are closures over this scope.
    yaw_nudge_total = [0.0]
    # Set once the operator has been shown a PARKED gripper and given the chance
    # to correct it. That is the difference between "measured zero" and "never
    # asked", and nudge_total alone cannot tell them apart -- see
    # nudge_measured in save_calibration.
    nudge_offered = [False]
    # EVERY NUDGE SEPARATELY, in the order it was typed, mm. nudge_total is a
    # SUM, and a sum cannot show that the first correction did nothing.
    #
    # Measured 2026-08-09 at N and O: a 5 mm tangential nudge barely moved the
    # arm, so a second identical 5 mm was typed and that one landed. The row
    # recorded +10 mm -- but the ARM only moved about 5, so the file says the
    # open-loop error was twice what it was. A 5 mm tangential nudge is 1.63 deg
    # of J1 at r=176 against a full backlash of 1.88 deg, i.e. the correction is
    # SMALLER THAN THE SLACK, so whether it lands at all depends on which flank
    # the joint was resting on.
    #
    # This is the plan's "a nudge is a control action, not a measurement" made
    # concrete: fitting the nudge column fits the dead band. Recording the steps
    # is what makes the two separable after the fact, and it costs one append.
    nudge_steps = []
    # Caliper readings typed at the park with 'm dx dy'. The FIRST one is the
    # open-loop error at this pose; later ones track what the nudges achieved.
    measured_offsets = []

    # Filled by descend(), read by save_calibration(). Both are closures over
    # run_stage1, and the descent happens between them.
    grasp_flange_fk = []

    def save_calibration(grasped):
        """One row per run. Truth is passed through, never assumed.

        RECORDED EVEN WITH NO TRUTH, since 2026-08-06. It used to return here
        unless a --truth-block-* was given, which meant every explore-driven run
        -- the ones with no tape measure anywhere in them -- wrote nothing at
        all. Two full pick-and-place runs produced zero rows.

        The truth is what makes vision_error and survey_error computable, and
        those still read None without it. But the NUDGE does not need a truth:
        it is the operator looking at the jaws over the block and saying how far
        off they are, which is a direct measurement of the residual at that pose,
        taken at the grasp. Throwing that away because nobody held a tape measure
        is throwing away the cheapest calibration data this project produces.
        """
        calibration.record(
            args.calibration_log,
            zone_origin=[args.zone_origin[0], args.zone_origin[1]],
            zone_yaw_deg=args.zone_yaw,
            truth_world=list(truth_world) if truth_world else None,
            truth_zone=list(truth_zone) if truth_zone else None,
            measured_zone=[block.zx, block.zy],
            measured_world=[grasp_x - nudge_total[0], grasp_y - nudge_total[1]],
            commanded_world=[grasp_x, grasp_y],
            nudge=list(nudge_total),
            yaw_nudge_deg=yaw_nudge_total[0],
            grasped=bool(grasped),
            block_class=args.block_class,
            grasp_yaw_deg=grasp_yaw_deg,
            grasp_z=grasp_z,
            hover_z=grasp_hover,
            views=int(block.n_views),
            view_spread_m=float(block.spread_m),
            identified=bool(identity),
            # The two halves of the flange-to-jaw measurement, recorded together
            # or not at all: where the flange really was, and what the jaw offset
            # model was set to when it went there. On a row with grasped=True and
            # a truth, those plus truth_world give the offset directly, with no
            # frame assumed. This is the evidence the next recalibration needs.
            flange_fk=list(grasp_flange_fk) or None,
            jaw_offset=[pp.JAW_RADIAL_OFFSET_M, pp.JAW_TANGENTIAL_OFFSET_M],
            # DID THE OPERATOR ACTUALLY MEASURE, or was no nudge ever offered?
            # 27 of the first 61 rows carry nudge [0.0, 0.0] and summarise()
            # counts every one of them as a measured zero, because `if nudge:` is
            # true for a non-empty list. They come from --survey-only, --dry-run
            # and the abort paths, where the operator was never shown a parked
            # gripper at all. Harmless while the numbers were being eyeballed in
            # a terminal; fatal the moment anything fits them, since they drag
            # every mean toward zero with rows that measured nothing.
            nudge_measured=bool(nudge_offered[0]),
            nudge_steps=list(nudge_steps),
            nudge_n_steps=len(nudge_steps),
            measured_offsets=list(measured_offsets),
            # The open-loop error at this pose, in METRES to match every other
            # vector in the schema. This -- not `nudge` -- is what a fit wants.
            open_loop_offset=([measured_offsets[0]["dx_mm"] / 1000.0,
                               measured_offsets[0]["dy_mm"] / 1000.0]
                              if measured_offsets else None),
            # THREE RADII, because a radial correction has to be fitted against
            # the radius it will be applied at, and these differ by ~20 mm:
            #   zone_r    the surveyed zone origin -- where a survey/lens-scale
            #             bias lives, and what apply_origin_radial_bias scales
            #   jaw_r     the commanded jaw target
            #   flange_r  where the flange actually went, i.e. jaw_r pushed out
            #             by compensate_for_tip_swing
            # Fitting against the wrong one visibly moves the radius coefficient,
            # which is what tells us which slot Saturday's correction belongs in.
            zone_radius_m=math.hypot(args.zone_origin[0], args.zone_origin[1]),
            jaw_radius_m=math.hypot(grasp_x, grasp_y),
            flange_radius_m=(math.hypot(grasp_flange_fk[0], grasp_flange_fk[1])
                             if len(grasp_flange_fk) >= 2 else None),
            constants=model_provenance(args),
            note=args.note or "")

    if args.survey_only:
        # Measure and stop, without touching the block. The point is that the
        # block never moves: repeating this measures the VISION's own
        # repeatability, where repeating a full pick measures vision plus
        # however precisely a human puts the block back afterwards. Ten
        # grasp-and-replace runs on 2026-08-05 put the lateral error at
        # +3.7 +- 2.4 mm, which is real but cannot be attributed -- a block
        # placed 3.7 mm off centre every time reads identically.
        print("\n[stage1] --survey-only: measured and recorded. Nothing moved "
              "toward the block, so the block is exactly where it was. Run it "
              "again to measure repeatability with the scene held still.")
        save_calibration(False)
        return True

    if args.confirm and not args.dry_run:
        while True:
            print_block_report(block, block_yaw_world, grasp_x, grasp_y,
                               grasp_z, grasp_yaw_deg, grasp_hover,
                               nudge_total, args.block_class)
            answer = _ask("[confirm] ENTER = go to the hover   "
                          "'dx dy' mm = nudge   q = abort > ")
            if answer in ("q", "quit", "n", "no"):
                print("[stage1] aborted before moving.")
                save_calibration(False)
                return False
            nudge = _parse_nudge(answer)
            if nudge is not None:
                grasp_x += nudge[0]
                grasp_y += nudge[1]
                nudge_total[0] += nudge[0]
                nudge_total[1] += nudge[1]
                grasp_yaw_deg += nudge[2]
                yaw_nudge_total[0] += nudge[2]
                # at_park False: typed BEFORE the arm moved, so the operator
                # could not see the jaws. Not a measurement of anything.
                nudge_steps.append({"dx_mm": nudge[0] * 1000,
                                    "dy_mm": nudge[1] * 1000,
                                    "dyaw_deg": nudge[2], "at_park": False})
                grasp_hover = hover_z_for(grasp_x, grasp_y, grasp_z,
                                          grasp_yaw_deg)
                continue
            if answer == "":
                break
            print("[confirm] did not understand %r." % answer)

    # unidirectional=True for the reason spelled out at the confirm-loop park
    # below: J1 undershoots by a measured 0.94 deg in whichever direction it
    # last travelled, worth 6.4 mm of tangential scatter, and this is the move
    # that decides which flank of its slack the joint rests on.
    #
    # THIS is the pre-grasp move on the UNATTENDED path (--yes), which is how
    # explore_pick_place drives a sweep -- so leaving it out here would have left
    # the fix inactive on exactly the runs Saturday depends on.
    #
    # Deliberately NOT on the descent that follows. compensate_for_tip_swing's
    # lateral terms have no z dependence, so the hover and the grasp share an
    # identical commanded flange XY: J1's target does not change during the
    # descent, the joint does not turn, and it keeps the flank this move left it
    # on. Adding it there would inject a J1_APPROACH_LEAD_DEG base rotation into
    # a move whose whole job is not to travel sideways.
    steps = [
        ("Move over the block",
         lambda: move_arm_to(io_client, grasp_x, grasp_y, grasp_hover,
                             block_yaw_deg=grasp_yaw_deg,
                             unidirectional=True)),
    ]

    if args.dry_run:
        # Stop HERE, hovering over the answer, rather than before moving at all.
        # A dry run that stops earlier prints numbers nobody can check; this one
        # parks the jaws over the block at grasp height and grasp yaw, where the
        # offset between jaws and block is directly visible. That photo is the
        # measurement that settles the (a)-vs-(b) question above.
        print("\n[stage1] --dry-run: moving to the grasp hover and STOPPING "
              "there. No descent, no grasp.")
        for name, action in steps:
            print("\n=== %s ===" % name)
            if not action():
                print("[stage1] step FAILED: %s" % name)
                return False
        # "at grasp height" was wrong and it mattered: the only step above moves
        # to grasp_hover, which is APPROACH_HEIGHT above the grasp. Judging a
        # lateral offset from there is the weakest measurement in the pipeline
        # (see --measure-clearance-mm), and a message claiming otherwise is how
        # a hover reading gets written down as a grasp reading.
        print("\n[stage1] --dry-run: parked over the block at grasp YAW, %.0f mm "
              "above the grasp height. LOOK AT THE ARM. How far, and which way, "
              "are the jaws off the block? That number is the arm's true error "
              "at this pose -- it is not derivable from anything in this log."
              % ((grasp_hover - grasp_z) * 1000))
        if len(pp.LAST_FLANGE_FK) >= 2:
            grasp_flange_fk[:] = list(pp.LAST_FLANGE_FK)
        save_calibration(False)
        return True

    if args.confirm:
        # Park over the answer and hold there until a human agrees, re-parking
        # after any nudge so what you approve is what you are looking at.
        while True:
            print("\n=== Move over the block ===")
            # unidirectional=True: MEASURED ON HARDWARE 2026-08-07, 73 moves
            # across both session-1 trials. J1 undershoots its commanded angle by
            # 0.94 deg (sd 0.12) in whichever direction it travelled -- +ve
            # commands land -0.89, -ve commands land +0.97, and the magnitude is
            # the same for a 0.9 deg move as for a 22 deg one, so it is lost
            # motion and not a tracking error. Approaching the SAME commanded
            # point from opposite sides landed 6.3 / 6.0 / 6.9 mm apart.
            #
            # j1_unidirectional_approach fixes both halves at once: it makes the
            # last leg always travel in +J1_APPROACH_DIR, which turns the coin
            # flip into a constant, and it is the only place
            # J1_RESIDUAL_BIAS_DEG is applied, which cancels the constant. That
            # bias was fitted at 1.10 deg from the zone survey and these trials
            # measure 0.94 -- right to 0.16 deg, so nothing needs refitting.
            #
            # WHY THIS WAS MISSING: move_arm_to defaults unidirectional=False,
            # and no call site in this file ever passed it. Only pick_place's
            # legacy fixed-coordinate flow and zone_calibrate did -- which is why
            # the survey that MEASURED the bias saw it work while the code that
            # actually picks blocks never got it.
            if not move_arm_to(io_client, grasp_x, grasp_y, grasp_hover,
                               block_yaw_deg=grasp_yaw_deg,
                               unidirectional=True):
                print("[stage1] step FAILED: Move over the block")
                return False
            park_z = measure_park_z(grasp_hover)
            if park_z is not None:
                print("\n=== Lower to the measurement clearance ===")
                if cartesian_move_to(io_client, grasp_x, grasp_y, park_z,
                                     block_yaw_deg=grasp_yaw_deg) is False:
                    # BEST EFFORT. A failed descent leaves the arm at the hover,
                    # which is where it used to sit anyway -- a worse view, not a
                    # broken run, and refusing the measurement outright would be
                    # worse than offering a harder one.
                    print("[confirm] could not lower to the measurement "
                          "clearance; measuring from the hover instead.")
                    park_z = None
            print_block_report(block, block_yaw_world, grasp_x, grasp_y,
                               grasp_z, grasp_yaw_deg, grasp_hover,
                               nudge_total, args.block_class)
            here = park_z if park_z is not None else grasp_hover
            print("[confirm] parked HERE, %.0f mm above the grasp height, "
                  "fingertips %.0f mm above the block's top face."
                  % ((here - grasp_z) * 1000,
                     (here - pp.GRASP_OFFSET_Z
                      - (pp.MAT_SURFACE_Z + pp.BLOCK_HEIGHT_M)) * 1000))
            print("[confirm] LOOK AT THE JAWS. If they are off the block, type "
                  "the offset you can see:")
            print("[confirm]   'dx dy' in mm, world axes -- +x is world +X, "
                  "+y is world +Y.")
            print("[confirm]   'dx dy dyaw' also turns the wrist, dyaw in "
                  "DEGREES.")
            print("[confirm]   'm dx dy' RECORDS a caliper reading and moves "
                  "NOTHING. Do this FIRST.")
            # SIGN, PINNED. 2026-08-11 at N the operator typed 'm -3 -6' and then
            # nudged '-3 -6' -- the same numbers -- so what lands in
            # measured_offsets is the CORRECTION, not the error, and the two
            # differ by a minus sign. Either convention works as long as it never
            # changes, and asking for the same numbers as the nudge is the one
            # that matches what a person at the bench actually does.
            # ANY FIT ON open_loop_offset MUST NEGATE IT TO GET THE ERROR.
            print("[confirm]   sign: give 'm' the SAME numbers you would type as "
                  "a nudge (the correction), not the error.")
            # The gripper is parked and the operator can see it. From here a
            # nudge of zero is a measurement.
            nudge_offered[0] = True
            # AN INNER LOOP, AND THIS IS THE WHOLE POINT OF IT. `continue` in the
            # OUTER loop jumps back to "Move over the block" and re-parks the arm
            # -- so when 'm' was first added it ended its branch with `continue`
            # and every "moves NOTHING" reading silently re-parked. So did every
            # typo, which fell off the bottom of the body into the same re-park.
            #
            # Caught on hardware 2026-08-11 at position N: typing 'm 0 5' twice
            # walked the jaw-axis gaps 3.86/6.00 -> 6.00/3.00 -> 3.86/6.00, i.e.
            # the arm ALTERNATED between two positions 2.6 mm apart on the radial
            # axis while the operator was told nothing had moved. Every reading
            # taken that way is of a different pose than the one before it.
            #
            # Only a real nudge or ENTER leaves this loop. Measurements and typos
            # re-prompt, and the arm holds absolutely still.
            action = None
            while True:
                answer = _ask("[confirm] ENTER = descend and grasp   'dx dy' mm "
                              "= nudge and re-park   'm dx dy' = record a "
                              "caliper reading   q = abort > ")
                if answer in ("q", "quit", "n", "no"):
                    print("[stage1] aborted at the hover. Nothing descended.")
                    save_calibration(False)
                    return False
                # THE MEASUREMENT, KEPT APART FROM THE CONTROL ACTION. A nudge is
                # what the operator DID; this is what they SAW. They are not the
                # same number and this run proved it -- 2026-08-09 at position N
                # the caliper read -9.70 mm and the operator typed +15 mm of
                # nudge to correct it, because the first 5 mm nudge delivered
                # only 2.42 mm (one J1 dead band, 2.58 mm at r=126) and the third
                # overshot. The row recorded +15: the nudge column overstated a
                # 9.70 mm error by 5.3 mm, and no amount of care in the nudge
                # loop can undo that, because the caliper reading was never
                # written down anywhere.
                #
                # Moves nothing -- see the inner-loop comment above for the bug
                # that made that false for one afternoon. Read it FIRST, before
                # any nudge, and the first entry is the open-loop error at this
                # pose, which is the number the whole calibration wants and the
                # only one admissible in a fit.
                if (answer[:1] in ("m", "M")
                        and _parse_nudge(answer[1:]) is not None):
                    seen = _parse_nudge(answer[1:])
                    measured_offsets.append({"dx_mm": seen[0] * 1000,
                                             "dy_mm": seen[1] * 1000,
                                             "after_nudges": len(nudge_steps)})
                    print("[confirm] recorded caliper reading (%+.2f, %+.2f) mm "
                          "after %d nudge(s). THE ARM HAS NOT MOVED."
                          % (seen[0] * 1000, seen[1] * 1000, len(nudge_steps)))
                    if len(measured_offsets) == 1:
                        print("[confirm] that is the OPEN-LOOP error at this "
                              "pose.")
                    continue
                nudge = _parse_nudge(answer)
                if nudge is not None:
                    grasp_x += nudge[0]
                    grasp_y += nudge[1]
                    nudge_total[0] += nudge[0]
                    nudge_total[1] += nudge[1]
                    grasp_yaw_deg += nudge[2]
                    yaw_nudge_total[0] += nudge[2]
                    nudge_steps.append({"dx_mm": nudge[0] * 1000,
                                        "dy_mm": nudge[1] * 1000,
                                        "dyaw_deg": nudge[2], "at_park": True})
                    grasp_hover = hover_z_for(grasp_x, grasp_y, grasp_z,
                                              grasp_yaw_deg)
                    print("[confirm] re-parking")
                    action = "nudge"
                    break
                if answer == "":
                    action = "descend"
                    break
                print("[confirm] did not understand %r. Nothing moved."
                      % answer)
            if action == "descend":
                break
        # Already parked; do not queue the hover again below.
        steps = []

    if args.skip_pick:
        # THE ARM AS THE MEASURING INSTRUMENT. The operator has just driven the
        # jaws onto the block by eye, so nudge_total is how far the open-loop
        # pipeline was wrong AT THIS POSE -- survey, vision and jaw offset
        # together, read off at the grasp height rather than extrapolated to it.
        # That is the same number a tape measure gives and it takes ten seconds
        # instead of two minutes, which is what makes a ten-position sweep
        # something a person will actually finish.
        #
        # Nothing descends and nothing is grasped, so the block does not move
        # and the next run at this position starts from an identical scene.
        print("\n[stage1] --skip-pick: parked over the block, measured, "
              "recorded. Total correction you dialled in: (%+.1f, %+.1f) mm, "
              "%+.1f deg."
              % (nudge_total[0] * 1000, nudge_total[1] * 1000,
                 yaw_nudge_total[0]))
        print("[stage1] Nothing descended. The block is exactly where it was, "
              "so move the ZONE and run the next position.")
        # THE FLANGE AT THE APPROVED PARK. pp.LAST_FLANGE_FK is already current
        # here -- the park at the top of the confirm loop goes through
        # move_arm_to, which calls record_flange_fk -- and it was being thrown
        # away, because grasp_flange_fk is only written by descend() and this
        # path returns before descend() is ever reached. Every skip-pick row in
        # the history therefore has flange_fk: null.
        #
        # NOT A JAW-OFFSET MEASUREMENT, and it must not be used as one. The
        # flange was COMMANDED to (jaw target - the modelled offset), so
        # commanded_world - flange_fk recovers the constant already configured,
        # plus ~0.5 mm of tracking error. It is bearing-independent by
        # construction and would produce a beautifully tight table across every
        # bearing bin that says nothing at all -- the exact "internal agreement
        # is not accuracy" trap of Lesson 4, and the mechanism behind three
        # previous wrong jaw constants. calibration.jaw_offset() still requires
        # grasped=True and an external truth, correctly.
        #
        # What it IS good for: confirming the compensation was applied and the
        # arm tracked it, and supplying flange_radius_m so a radial fit can be
        # tested against the radius it will be applied at.
        if len(pp.LAST_FLANGE_FK) >= 2:
            grasp_flange_fk[:] = list(pp.LAST_FLANGE_FK)
        save_calibration(False)
        return True

    # Straight-down Cartesian by default because GRASP_OFFSET_Z is defined
    # against a vertical approach and a joint-space move arcs. --ik-descent
    # trades that for a single seeded-IK goal, which is worth having on an arm
    # where multi-waypoint trajectories have their own history (see the
    # JTC/pymycobot note in pick_place.py): over a 40 mm drop from directly
    # above, the arc is small and a goal that actually executes beats a
    # perfect path that does not.
    def descend():
        if args.ik_descent:
            ok = move_arm_to(io_client, grasp_x, grasp_y, grasp_z,
                             block_yaw_deg=grasp_yaw_deg)
        else:
            ok = cartesian_move_to(io_client, grasp_x, grasp_y, grasp_z,
                                   block_yaw_deg=grasp_yaw_deg)
        # Snapshot NOW. pp.LAST_FLANGE_FK is whatever the most recent Cartesian
        # move left behind, and the retreat and the place both come after this
        # one -- the first row written with it recorded the PLACE flange at -Y
        # and made jaw_offset() report a 213 mm offset. The grasp is the only
        # pose where the jaws are known to be on the block, so it is the only
        # one worth keeping.
        grasp_flange_fk[:] = list(pp.LAST_FLANGE_FK)
        return ok

    steps += [
        ("Descend to grasp", descend),
        ("Close gripper",
         lambda: gripper_close_until_contact(io_client)),
        ("Retreat after grasp",
         lambda: cartesian_move_to(io_client, grasp_x, grasp_y, grasp_hover,
                                   allow_fallback=True,
                                   block_yaw_deg=grasp_yaw_deg)),
    ]

    # --- 4. place, unchanged from pick_place.py ---------------------------
    # Stage 1 places at the hardcoded PLACE_XYZ. Stage 2 replaces this with a
    # second detection at the place zone.
    #
    # PLACE_XYZ.z is used exactly as pick_place.py uses it, for the same reason
    # the grasp height is not recomputed: it is hand-tuned, not geometric, and
    # the place pose's droop is its own. It reads 25 mm above what the pick side
    # implies for the same mat; that difference is the two poses' z error, not a
    # stale constant, so it is left alone.
    #
    # --place-origin is that Stage 2, in its lax form. The place zone only has
    # to receive the block somewhere inside a 4 in square, so its SURVEYED
    # centre is a good enough target and no second detection is needed once
    # explore has found it. X and Y come from the survey; Z does not, for the
    # reason above -- a surveyed origin's z is the mat, and PLACE_XYZ.z is a
    # hand-tuned release height that already accounts for this pose's droop.
    if args.place_origin is not None:
        place_x, place_y = args.place_origin[0], args.place_origin[1]
        place_surface_z = PLACE_XYZ[2]
        source = "surveyed place zone centre"
    else:
        place_x, place_y, place_surface_z = PLACE_XYZ
        source = "PLACE_XYZ, unchanged from pick_place.py"
    place_z = place_surface_z + args.block_thickness / 2.0 + GRASP_OFFSET_Z
    print("\n[stage1] place: (%.4f, %.4f) surface z %.4f -> release flange z "
          "%.4f (%s)" % (place_x, place_y, place_surface_z, place_z, source))
    place_hover = hover_z_for(place_x, place_y, place_z)
    steps += [
        ("Move to pre-place",
         lambda: move_arm_to(io_client, place_x, place_y, place_hover)),
        ("Descend to place",
         lambda: cartesian_move_to(io_client, place_x, place_y, place_z)),
        ("Open gripper (release)",
         lambda: io_client.gripper_move_to(GRIPPER_OPEN)),
        ("Retreat after release",
         lambda: cartesian_move_to(io_client, place_x, place_y, place_hover,
                                   allow_fallback=True)),
    ]

    for name, action in steps:
        print("\n=== %s ===" % name)
        if not action():
            print("[stage1] step FAILED: %s" % name)
            save_calibration(False)
            return False
        time.sleep(0.5)

    # ASK, do not assume. Every step "succeeding" is not evidence that the jaws
    # closed on anything -- arm_group_controller reports success from elapsed
    # time alone (no constraints: block in ros2_controllers.yaml). arm_error is
    # only meaningful on a run that really grasped, so a guess here would
    # poison the one measurement that pins the arm down.
    grasped = None
    if args.confirm:
        grasped = _ask("\n[confirm] did the jaws actually close on the block? "
                       "y = yes, anything else = no > ") in ("y", "yes")
    elif truth_zone is not None or truth_world is not None:
        print("[stage1] --yes was used, so nothing confirms the grasp "
              "physically. Recording it as UNCONFIRMED: it will count as a "
              "vision and open-loop point, not as an arm measurement.")
    save_calibration(bool(grasped))
    return True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone-origin", type=float, nargs=3,
                        metavar=("X", "Y", "Z"),
                        default=[0.0, ZONE_RADIUS_M, 0.0],
                        help="SURVEYED world pose of the pickup zone centre and "
                             "the plane the blocks sit on. Nothing measures this "
                             "-- every world coordinate this script reports is "
                             "only as good as this number (default: %(default)s)")
    parser.add_argument("--zone-yaw", type=float, default=0.0,
                        help="rotation of the tag square about world +Z, degrees")
    parser.add_argument("--zone-size", type=float, default=zv.DEFAULT_ZONE_SIZE,
                        help="side of the square joining the TAG CENTRES, metres "
                             "(default: %(default)s = 4 in, around a ~3 in "
                             "working area -- see APRIL_TAGS.md 'Usable area')")
    parser.add_argument("--tag-size", type=float, default=zv.DEFAULT_TAG_SIZE,
                        help="printed tag side, metres, for the usable-area "
                             "warning only (default: %(default)s = 1 in)")
    parser.add_argument("--block-thickness", type=float,
                        default=DEFAULT_BLOCK_THICKNESS,
                        help="Stage 1 block thickness, metres (default: "
                             "%(default)s). Affects the RELEASE height only "
                             "(place surface + thickness/2 + GRASP_OFFSET_Z); "
                             "the grasp descent is --grasp-z and does not "
                             "depend on it")
    parser.add_argument("--grasp-z", type=float, default=GRASP_FLANGE_Z,
                        help="FLANGE z to descend to for the grasp, metres. "
                             "Defaults to PICK_XYZ.z + GRASP_OFFSET_Z "
                             "(%%(default).4f) -- the height pick_place.py "
                             "already grasps this block at. Not derived from a "
                             "mat height or block thickness: PICK_XYZ.z is "
                             "hand-tuned and carries the arm's z error, which "
                             "no geometry reproduces")
    parser.add_argument("--dry-run", action="store_true",
                        help="do everything up to and including the move that "
                             "parks the jaws over the block at grasp height and "
                             "grasp yaw, then STOP. No descent, no grasp. Look "
                             "at the arm: the jaw-to-block offset you can see is "
                             "the arm's true error, which nothing in the log "
                             "measures")
    parser.add_argument("--truth-block-world", type=float, nargs=2,
                        metavar=("X", "Y"), default=None,
                        help="the block's TRUE world position in metres, if you "
                             "measured it. Recorded, never applied -- it turns a "
                             "run into a calibration point instead of just a "
                             "grasp. Nothing in the code assumes a bench layout")
    parser.add_argument("--truth-block-zone", type=float, nargs=2,
                        metavar=("ZX", "ZY"), default=None,
                        help="the block's TRUE zone-local position in "
                             "MILLIMETRES. '0 0' means it is on the zone "
                             "centre, which is what makes the vision error "
                             "measurable on its own -- see calibration.py")
    parser.add_argument("--skip-pick", action="store_true",
                        help="park over the block, let you dial the jaws onto "
                             "it with 'dx dy dyaw', record what you dialled, "
                             "and STOP without descending. The correction you "
                             "type is the open-loop error at that pose -- the "
                             "cheap way to sweep many zone positions, since the "
                             "block never moves between runs")
    parser.add_argument("--survey-only", action="store_true",
                        help="survey the zone, report the error against --truth, "
                             "record it and STOP. No approach, no grasp. This is "
                             "the cheap repeatability loop: the block never "
                             "moves, so it measures the vision alone")
    parser.add_argument("--note", default="",
                        help="free text stored with the calibration row -- what "
                             "changed on the bench, so a later reader can tell "
                             "two runs apart")
    parser.add_argument("--calibration-log", default=None,
                        help="where calibration rows go (default: %s)"
                             % calibration.DEFAULT_LOG)
    parser.add_argument("--force-grasp-yaw", type=float, default=None,
                        metavar="DEG",
                        help="command this grasp yaw instead of the one measured "
                             "from the block. For CALIBRATION, where the block "
                             "is deliberately oriented the same way every run "
                             "and only x/y is being measured: it pins the jaw "
                             "axis to a known world direction, which is what "
                             "makes a caliper gap reading mean anything. Use 0 "
                             "with the block's far side toward -Y")
    parser.add_argument("--ignore-clearance", action="store_true",
                        help="descend even when the open jaws would strike a "
                             "neighbouring block. The jaw geometry the check "
                             "uses is UNMEASURED (see JAW_GEOMETRY_MEASURED), so "
                             "a refusal can be conservative -- but look at the "
                             "bench before you use this")
    parser.add_argument("--ignore-merged", action="store_true",
                        help="grasp a contour that looks like two blocks merged "
                             "into one. Its centroid sits in the seam between "
                             "them, so the jaws close on nothing")
    parser.add_argument("--block-class", choices=bc.BLOCK_CLASSES,
                        default=bc.BLOCK_CLASSES[0],
                        help="which block to pick, identified by the face tags "
                             "stuck to it (default: %(default)s)")
    parser.add_argument("--any-block", dest="block_class", action="store_const",
                        const=None,
                        help="pick whatever is best measured, ignoring "
                             "identity. Only safe with ONE block in the zone -- "
                             "with two it is a coin flip")
    parser.add_argument("--place-origin", type=float, nargs=2,
                        metavar=("X", "Y"), default=None,
                        help="world XY to release the block at, normally the "
                             "place zone's surveyed centre. Z is NOT taken from "
                             "here -- the release height stays PLACE_XYZ.z, "
                             "which is hand-tuned for that pose. Default: the "
                             "whole hardcoded PLACE_XYZ")
    parser.add_argument("--jaw-radial-offset", type=float, default=None,
                        metavar="M",
                        help="override JAW_RADIAL_OFFSET_M (metres, negative = "
                             "jaws inboard of the flange). The default is "
                             "calibrated at bearing ~0 and UNVERIFIED "
                             "elsewhere -- see the constant's comment in "
                             "pick_place.py before trusting it at a new pose")
    parser.add_argument("--jaw-tangential-offset", type=float, default=None,
                        metavar="M",
                        help="override JAW_TANGENTIAL_OFFSET_M (metres)")
    parser.add_argument("--yes", dest="confirm", action="store_false",
                        help="run without the operator checkpoints. By default "
                             "the run prints where it thinks the block is and "
                             "waits for ENTER, then parks over it and waits "
                             "again before descending -- and at that second "
                             "prompt you can type 'dx dy' in mm to nudge the "
                             "target by the offset you can see")
    parser.add_argument("--measure-clearance-mm", type=float,
                        default=MEASURE_CLEARANCE_M * 1000.0, metavar="MM",
                        help="fingertip clearance above the block's TOP FACE at "
                             "the confirm park, mm (default %(default).0f). The "
                             "arm goes to the normal hover, then straight down "
                             "to here, so you judge the offset from ~8 mm away "
                             "instead of 25. Pass 0 to keep the old hover park")
    parser.add_argument("--ik-descent", action="store_true",
                        help="descend with a seeded-IK joint-space goal instead "
                             "of a straight-down Cartesian path. Arcs slightly "
                             "over the 40 mm drop; use it when the Cartesian "
                             "descent does not execute")
    parser.add_argument("--verify", action="store_true",
                        help="before grasping, converge the camera over the "
                             "block and correct the grasp target by what that "
                             "measures. OFF by default: the correction and the "
                             "camera model's own uncalibrated offset are "
                             "indistinguishable in a single run, so this can "
                             "just as easily inject error as remove it. Settle "
                             "it with a --dry-run photo first")
    parser.add_argument("--debug-image", metavar="PATH",
                        help="path ON THE PI for the detector's annotated frame")
    parser.add_argument("--log", metavar="CSV",
                        help="append (commanded correction, measured result) rows "
                             "here -- the disturbance-observer dataset")
    return parser.parse_args(argv)


def main():
    args = parse_args()
    args.zone_z = args.zone_origin[2]
    zone_yaw_rad = math.radians(args.zone_yaw)

    # Written back into pick_place because compensate_for_tip_swing reads them
    # as module globals -- same pattern --gripper-yaw-deg already uses. Announced
    # rather than applied silently: these decide where the jaws end up, and a run
    # whose grasp lands 26 mm off should say so in its own log.
    if args.jaw_radial_offset is not None:
        pp.JAW_RADIAL_OFFSET_M = args.jaw_radial_offset
    if args.jaw_tangential_offset is not None:
        pp.JAW_TANGENTIAL_OFFSET_M = args.jaw_tangential_offset
    global MEASURE_CLEARANCE_M
    MEASURE_CLEARANCE_M = max(0.0, args.measure_clearance_mm / 1000.0)
    print("[stage1] jaw offset from the flange: radial %+.1f mm, tangential "
          "%+.1f mm%s"
          % (pp.JAW_RADIAL_OFFSET_M * 1000, pp.JAW_TANGENTIAL_OFFSET_M * 1000,
             "  (overridden on the command line)"
             if (args.jaw_radial_offset is not None
                 or args.jaw_tangential_offset is not None) else
             "  (calibrated at bearing ~0 -- unverified elsewhere)"))

    rclpy.init()
    io_client = RobotIOClient()
    detector = Detector(io_client, args.zone_origin[0], args.zone_origin[1],
                        args.zone_z, zone_yaw_rad, args.zone_size)
    log = CorrectionLog(args.log)

    ok = False
    try:
        if not io_client.wait_for_joint_states():
            print("No /joint_states -- the robot side is not up. "
                  "PROJECT_CONTEXT.md: nan/absent joint states means zero "
                  "publishers, never bad data from the arm.")
            return 1
        if not detector.wait_for_service():
            return 1
        ok = run_stage1(io_client, detector, args, log)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        log.flush()
        print("\n=== Returning home ===")
        try:
            go_home(io_client)
        except Exception as exc:                    # noqa: BLE001
            print("go_home failed on the way out: %s" % exc)
        io_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("\n%s" % ("STAGE 1 COMPLETE" if ok else "STAGE 1 DID NOT COMPLETE"))
    print("Confirm the grasp VISUALLY. arm_group_controller reports "
          "'Goal reached, success!' from elapsed time alone -- there is no "
          "constraints: block in ros2_controllers.yaml, so ROS logs are not "
          "evidence of physical motion on this setup.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
