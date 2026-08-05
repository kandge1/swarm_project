#!/usr/bin/env python3
"""EXPLORE MODE: find a zone by panning, and report where it is in the world.

Runs on mars. Drives the arm to a raised, tilted "lookout" pose and pans the
BASE joint through a sweep, asking /detect_block at each step which zone tags it
can see. The output is a world-frame zone origin, which is exactly the argument
tag_pick_place.py already takes:

    python3 explore.py                         # find the pickup zone
    python3 explore.py --zone place            # find the place zone
    python3 explore.py --dry-run               # pan and report, no handoff
    python3 explore.py --print-command         # emit the tag_pick_place line

------------------------------------------------------------------------------
WHY THIS IS SMALL
------------------------------------------------------------------------------
It replaces ONE hard-coded number. tag_pick_place.py's --zone-origin has always
been a hand-surveyed constant (0, 0.2286, 0) -- the mat taped down where the
code was told it would be. Everything downstream of that number already works
and is calibrated. So explore does not need to localise blocks, measure yaw, or
plan a grasp; it needs to produce that one coordinate pair, and then get out of
the way.

------------------------------------------------------------------------------
THE POSE, AND THE ONE NUMBER THAT MATTERS IN IT
------------------------------------------------------------------------------
    [J1, 0, 0, EXPLORE_PITCH_DEG, 0, -135]

J2 and J3 stay at zero, so the arm is STRAIGHT UP and only the wrist is bent.
That is deliberate: it is the configuration least able to collide with itself
while J1 sweeps 270 degrees. Folded poses (high J2, strongly negative J3) buy
camera distance and drive the camera into the arm's own links -- found on
hardware 2026-08-04. Joint angles are commanded directly, so MoveIt's collision
model is NOT consulted; keeping the arm straight is what makes that safe.

EXPLORE_PITCH_DEG = -71, not -50. At -50 the optical axis lands 15.5 in from
the base while the zone sits at 9 in, which puts the near third of the zone
outside the frame entirely (28.8 deg off-axis against a +-23.5 deg vertical
FOV). At -71 the axis lands on the zone centre and all four tags sit at
4.2-5.1 px per module -- comfortably decodable. See STACKED_BLOCKS_GUIDE.md.

------------------------------------------------------------------------------
HOW THE ZONE ORIGIN IS RECOVERED
------------------------------------------------------------------------------
The detector reports camera_zx/camera_zy: the ZONE-LOCAL coordinates of the
image centre, straight out of the tag homography. Forward kinematics gives
where the optical axis meets the mat in WORLD. The zone origin is the
difference:

    zone_origin_world = axis_hit_world - R(zone_yaw) . (camera_zx, camera_zy)

This is exact for a plane homography REGARDLESS OF VIEWING TILT -- the image
centre maps to a real mat point whatever the angle, which is the whole appeal
of the homography design. Its accuracy is limited by two things and neither is
the tilt: the principal point is assumed to be the image centre (uncalibrated,
see zone_vision.camera_in_zone), and FK is only as good as the arm's own
positioning, which this project documents at heart as untrustworthy.

ZONE_YAW IS NOT KNOWN A PRIORI, AND ASSUMING IT IS ZERO WRECKS THE RADIUS.
camera_zx/zy are in the MAT's axes; axis_hit is in the WORLD's. R(zone_yaw) is
the only thing connecting them, and the 2026-08-05 sweep ran with the default
zone_yaw = 0 against a mat whose true yaw was -89 deg. The correction is ~48 mm
of pure RADIAL offset in mat axes; rotated by the wrong 89 degrees it came out
almost entirely TANGENTIAL, so the reported bearing stayed roughly right while
the reported radius never got corrected at all -- it just tracked the axis-hit
radius, a constant 196 mm that is a property of the POSE and says nothing about
where the mat is. Eleven views smeared from 149 mm to 238 mm.

So explore SOLVES for zone_yaw instead of being told it. Across a pan the mat
is stationary and the camera is not, which makes yaw observable: pick the yaw
that makes every view agree on one point. That is a closed-form 2-D Procrustes
fit (see fit_zone) whose residual is then a real, self-validating quality
number. Re-fitting the failed sweep's own numbers this way collapses all eleven
views onto a single point 3.7 mm wide.

So the answer here is COARSE ON PURPOSE -- good to a couple of centimetres. It
does not need to be better, because tag_pick_place.py then runs its four-view
survey and its correction loop from this starting point, and those close on the
tags rather than on this estimate. Explore's job is to get the arm into the
right postcode; the existing pipeline does the rest.

------------------------------------------------------------------------------
WHAT IT DOES NOT DO
------------------------------------------------------------------------------
No blocks, no grasp, no heights. It reads tag_ids and camera_zx/zy, which are
PRIMITIVE fields of the response -- so it is unaffected by any trouble in the
nested BlockDetection[] path.
"""
import argparse
import math
import os
import subprocess
import sys

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pick_place as pp  # noqa: E402
import tag_pick_place as tpp  # noqa: E402

ARM_JOINT_NAMES = list(pp.HOME_RADIANS.keys())

# Park pose. Camera ends up horizontal here (tool tilt 90 deg), which sees
# nothing useful -- it is a known, safe place to start from, not a viewpoint.
RESET_JOINTS_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, -45.0)

# The lookout pose. See the module docstring for why -71 and why J2/J3 are 0.
EXPLORE_PITCH_DEG = -71.0
EXPLORE_WRIST_DEG = -135.0

J1_START_DEG = -135.0
J1_END_DEG = 135.0

# COARSE step. 30 deg was the first value and it was too big -- reasoning from
# the +-30.1 deg horizontal FOV, which is the wrong number. The FOV is what the
# camera can SEE; what matters is the much smaller window in which ALL FOUR tags
# are in frame at once, because that is what the homography needs. The mat
# subtends ~28 deg at this distance, so its centre has to sit within roughly
# 9 deg of the optical axis for the corners to survive -- and the 2026-08-05
# sweep duly caught the mat at three bearings, missed it entirely at the one
# aimed straight at it, and produced estimates disagreeing by 335 mm.
J1_COARSE_STEP_DEG = 10.0

# FINE step, swept +-J1_FINE_SPAN_DEG either side of the best coarse hit. At
# this radius 2.5 deg is about 10 mm of arc, below the tolerance the downstream
# survey then closes anyway.
J1_FINE_STEP_DEG = 2.5
J1_FINE_SPAN_DEG = 12.5

MOVE_SECONDS = 3.0
STEP_SECONDS = 1.6          # shorter hops between adjacent pan steps
SETTLE_SECONDS = 1.2        # the arm must be STATIONARY before a detect call

# Zone origin handed to the detector during the sweep. It does NOT affect tag
# detection or camera_zx/zy -- only the world conversion of block positions,
# which explore ignores. It has to be something, so it is the historical
# hand-surveyed value.
NOMINAL_ZONE_RADIUS_M = 0.2286


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------
def fk_flange_pose(joint_values):
    """(position, 3x3 rotation) of the flange. pick_place._fk_flange returns
    only the approach axis, and the lens offset needs the full orientation to
    be rotated into world."""
    t = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for (xyz, rpy, axis), angle in zip(pp._FK_CHAIN, joint_values):
        t = pp._mat_mul(t, pp._origin(xyz, rpy))
        t = pp._mat_mul(t, pp._axis_rot(axis, angle))
    position = [t[0][3], t[1][3], t[2][3]]
    rotation = [[t[i][j] for j in range(3)] for i in range(3)]
    return position, rotation


def lens_pose(joint_values):
    """(lens position, optical axis unit vector) in world."""
    flange, R = fk_flange_pose(joint_values)
    offset = tpp._mat_vec(R, tpp.LENS_OFFSET_IN_FLANGE)
    lens = [flange[i] + offset[i] for i in range(3)]
    axis = tpp._mat_vec(R, tpp.OPTICAL_AXIS_IN_FLANGE)
    norm = math.sqrt(sum(a * a for a in axis)) or 1.0
    return lens, [a / norm for a in axis]


def axis_hits_mat(joint_values, mat_z=None):
    """Where the optical axis meets the mat plane, in world. None if it points
    up or along the plane -- which is not an error, just a pose looking at the
    wall, and the caller should skip it rather than divide by ~0."""
    mat_z = pp.MAT_SURFACE_Z if mat_z is None else mat_z
    lens, axis = lens_pose(joint_values)
    if axis[2] > -1e-6:
        return None
    scale = (mat_z - lens[2]) / axis[2]
    return [lens[i] + scale * axis[i] for i in range(3)]


def joints_for(j1_deg, pitch_deg, wrist_deg):
    return [math.radians(j1_deg), 0.0, 0.0, math.radians(pitch_deg), 0.0,
            math.radians(wrist_deg)]


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
def send_joints(io_client, joint_values, seconds, label):
    traj = JointTrajectory()
    traj.joint_names = ARM_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in joint_values]
    point.velocities = [0.0] * len(ARM_JOINT_NAMES)
    point.time_from_start.sec = int(seconds)
    point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
    traj.points = [point]
    if not io_client.arm_execute(traj):
        print("[explore] FAILED to reach %s" % label)
        return False
    return True


def achieved_joints(io_client, commanded):
    """Measured joint values if /joint_states is available, else the command.

    Preferred because the zone estimate is built on FK, and FK of the COMMANDED
    pose inherits every bit of this arm's tracking error -- J1 alone under-
    travels by over a degree (pick_place.J1_RESIDUAL_BIAS_DEG), which at the
    zone's radius is several millimetres of bearing error.

    current_joint_positions() takes joint_names and returns 0.0 for anything it
    has not heard about, so "missing" cannot be detected from the values -- an
    all-zeros arm is a legitimate pose. wait_for_joint_states() in main() is
    what establishes the topic is live; this only reads it.

    NOTE: the first version of this called current_joint_positions() with NO
    arguments, which is a TypeError, which the except below swallowed -- so
    every sighting in the 2026-08-05 sweep silently used commanded joints while
    reporting that /joint_states was unavailable. The except is now narrow and
    logs, because a broad silent one is exactly how that hid.
    """
    try:
        current = io_client.current_joint_positions(ARM_JOINT_NAMES)
    except Exception as exc:                            # noqa: BLE001
        print("[explore] /joint_states read failed (%s: %s) -- falling back to "
              "commanded joints" % (type(exc).__name__, exc))
        return list(commanded), False
    return [float(current[name]) for name in ARM_JOINT_NAMES], True


# ---------------------------------------------------------------------------
# The sweep
# ---------------------------------------------------------------------------
class Sighting(object):
    __slots__ = ("j1_deg", "joints", "tag_ids", "camera_zx", "camera_zy",
                 "rms", "scale", "zone_origin", "measured")

    def __init__(self, j1_deg, joints, response, zone_origin, measured):
        self.j1_deg = j1_deg
        self.joints = joints
        self.tag_ids = list(response.tag_ids)
        self.camera_zx = float(response.camera_zx)
        self.camera_zy = float(response.camera_zy)
        self.rms = float(response.homography_rms)
        self.scale = float(response.scale_px_per_m)
        self.zone_origin = zone_origin
        self.measured = measured

    @property
    def offset_mm(self):
        return math.hypot(self.camera_zx, self.camera_zy) * 1000.0


def zone_origin_from(joints, camera_zx, camera_zy, zone_yaw=0.0):
    """World (x, y) of the zone centre. See the module docstring."""
    hit = axis_hits_mat(joints)
    if hit is None:
        return None
    c, s = math.cos(zone_yaw), math.sin(zone_yaw)
    return (hit[0] - (c * camera_zx - s * camera_zy),
            hit[1] - (s * camera_zx + c * camera_zy))


# --- solving for the mat, rather than assuming half of it -------------------
# The camera has to actually MOVE in the mat's frame for the yaw to be
# observable at all. Two views 3 mm apart fix a point and say nothing about
# rotation, and the fit would happily return a confident garbage angle. The
# baseline is the widest separation between any two views measured IN THE MAT'S
# FRAME; 25 mm is comfortably exceeded by any real pan (the 2026-08-05 fine
# pass moved the image centre 72 mm across the mat over 25 deg of J1, the
# coarse pass 149 mm). Yaw uncertainty is roughly residual/baseline, and it
# only reaches the origin multiplied by the ~50 mm centre offset, so even a
# marginal baseline costs millimetres.
MIN_YAW_BASELINE_M = 0.025

# How well the single-point model has to hold. The fine pass fits to 3.7 mm and
# the coarse pass to 8.8 mm on real data; 20 mm is loose enough for a sloppy
# sweep and tight enough that two mats averaged together (which lands near
# 250 mm) can never pass.
MAX_FIT_RESIDUAL_M = 0.020


class ZoneFit(object):
    """One mat, solved from several views of it."""

    __slots__ = ("origin", "yaw", "residual", "baseline", "sightings")

    def __init__(self, origin, yaw, residual, baseline, sightings):
        self.origin = origin
        self.yaw = yaw
        self.residual = residual
        self.baseline = baseline
        self.sightings = list(sightings)

    @property
    def radius(self):
        return math.hypot(*self.origin)

    @property
    def bearing_deg(self):
        return math.degrees(math.atan2(self.origin[1], self.origin[0]))

    def describe(self):
        return ("origin (%+.4f, %+.4f)  r %.4f m = %.2f in  bearing %+.1f deg"
                "  |  zone yaw %+.1f deg  residual %.1f mm over %d view(s)"
                % (self.origin[0], self.origin[1], self.radius,
                   self.radius / 0.0254, self.bearing_deg,
                   math.degrees(self.yaw), self.residual * 1000.0,
                   len(self.sightings)))


def fit_zone(sightings, yaw_fixed=None):
    """(ZoneFit, None) or (None, why not).

    Least-squares over BOTH the origin and the zone yaw, closed form. Each view
    gives  hit_i = origin + R(yaw) . p_i  with p_i = (camera_zx, camera_zy).
    Centring both sides removes the origin, leaving a pure rotation fit:
    maximise  cos(yaw)*A + sin(yaw)*B, i.e. yaw = atan2(B, A). No iteration, no
    starting guess, and the leftover residual is the honest error bar.
    """
    rows = []
    for sighting in sightings:
        hit = axis_hits_mat(sighting.joints)
        if hit is not None:
            rows.append((hit[0], hit[1], sighting.camera_zx, sighting.camera_zy))
    if not rows:
        return None, "no view had the optical axis meeting the mat"

    n = float(len(rows))
    hx = sum(r[0] for r in rows) / n
    hy = sum(r[1] for r in rows) / n
    px = sum(r[2] for r in rows) / n
    py = sum(r[3] for r in rows) / n
    baseline = max((math.hypot(a[2] - b[2], a[3] - b[3])
                    for a in rows for b in rows), default=0.0)

    if yaw_fixed is not None:
        yaw = yaw_fixed
    elif baseline < MIN_YAW_BASELINE_M:
        return None, ("the camera moved only %.0f mm across the mat -- too "
                      "little to solve the zone yaw (need %.0f mm). Pan "
                      "further, or pass --zone-yaw if you have measured it."
                      % (baseline * 1000.0, MIN_YAW_BASELINE_M * 1000.0))
    else:
        a = sum((r[0] - hx) * (r[2] - px) + (r[1] - hy) * (r[3] - py)
                for r in rows)
        b = sum((r[1] - hy) * (r[2] - px) - (r[0] - hx) * (r[3] - py)
                for r in rows)
        yaw = math.atan2(b, a)

    c, s = math.cos(yaw), math.sin(yaw)
    origin = (hx - (c * px - s * py), hy - (s * px + c * py))
    residual = math.sqrt(sum((r[0] - (origin[0] + c * r[2] - s * r[3])) ** 2 +
                             (r[1] - (origin[1] + s * r[2] + c * r[3])) ** 2
                             for r in rows) / n)
    return ZoneFit(origin, yaw, residual, baseline, sightings), None


def split_runs(sightings, step):
    """Group sightings into contiguous stretches of J1.

    Fitting is only valid over views of ONE mat, and averaging two mats
    together produces a confident point in the empty space between them. A gap
    in the sweep is the cheapest evidence that the tags went out of sight and
    came back, which for a camera that only looks outward means a different
    mat. The 2026-08-05 coarse pass split exactly here: J1 -135..-65 saw one
    set of ids 0-3, nothing for 90 degrees, then J1 +45..+115 saw ANOTHER, and
    fitting each separately puts them 9.5 and 10.0 in out on opposite sides.
    """
    runs = []
    for sighting in sorted(sightings, key=lambda s: s.j1_deg):
        if runs and sighting.j1_deg - runs[-1][-1].j1_deg <= step * 1.5 + 1e-6:
            runs[-1].append(sighting)
        else:
            runs.append([sighting])
    return runs


def sweep_range(io_client, detector, args, start, end, step, label):
    """One pass over a J1 range. Returns the sightings it managed to take."""
    print("[explore] %s pass: J1 %+.1f -> %+.1f in %.1f deg steps"
          % (label, start, end, step))
    saved_start, saved_end, saved_step = args.start, args.end, args.step
    args.start, args.end, args.step = start, end, step
    try:
        return sweep(io_client, detector, args)
    finally:
        args.start, args.end, args.step = saved_start, saved_end, saved_step


def coarse_then_fine(io_client, detector, args):
    """Coarse pass to find the zone, fine pass to sit on its centre.

    Two passes rather than one fine pass over the whole 270 deg because a fine
    pass everywhere costs ~110 detect calls at a couple of seconds each. The
    coarse pass only has to answer "roughly which way", which surviving on 3-4
    tags does not require -- one tag is enough to know the mat is over there.
    """
    coarse = sweep_range(io_client, detector, args, args.start, args.end,
                         args.step, "coarse")
    if not coarse:
        return coarse, []

    # Centre the fine pass on the coarse hit with the MOST TAGS, not on the one
    # with the best origin estimate -- at coarse spacing the origin is expected
    # to be poor, and tag count is the honest measure of "the mat is this way".
    anchor = sorted(coarse, key=lambda s: (-len(s.tag_ids), s.offset_mm))[0]
    print("\n[explore] coarse best: J1 %+.1f with %d tag(s) -- refining +-%.1f "
          "deg around it\n" % (anchor.j1_deg, len(anchor.tag_ids),
                               args.fine_span))
    fine = sweep_range(io_client, detector, args,
                       anchor.j1_deg - args.fine_span,
                       anchor.j1_deg + args.fine_span,
                       args.fine_step, "fine")
    return coarse, fine


def sweep(io_client, detector, args):
    sightings = []
    j1 = args.start
    first = True
    while j1 <= args.end + 1e-9:
        commanded = joints_for(j1, args.pitch, args.wrist)
        seconds = MOVE_SECONDS if first else STEP_SECONDS
        if not send_joints(io_client, commanded, seconds, "J1 %+.0f" % j1):
            j1 += args.step
            continue
        first = False
        # The serial link is half-duplex and the detector shares the Pi with a
        # 100 Hz control loop -- a detect call while the arm is still moving
        # reads a blurred frame at best.
        _settle(io_client, args.settle)

        joints, measured = achieved_joints(io_client, commanded)
        response = detector.detect(zone=args.zone)
        if response is None or not response.tag_ids:
            print("[explore] J1 %+7.1f  no %s tags" % (j1, args.zone))
            j1 += args.step
            continue

        # PROVISIONAL. The zone yaw is not known until the sweep is over and
        # fit_zone has solved it, so this per-step number is only here to show
        # the sweep is alive. The answer comes from the fit, not from this.
        origin = zone_origin_from(joints, response.camera_zx,
                                  response.camera_zy, args.zone_yaw or 0.0)
        sighting = Sighting(j1, joints, response, origin, measured)
        sightings.append(sighting)
        print("[explore] J1 %+7.1f  tags %-14s rms %.2f px  centre offset "
              "%5.1f mm  camera at zone (%+6.1f, %+6.1f) mm%s"
              % (j1, str(sighting.tag_ids), sighting.rms, sighting.offset_mm,
                 sighting.camera_zx * 1000.0, sighting.camera_zy * 1000.0,
                 "" if measured else "  [commanded joints -- no /joint_states]"))
        j1 += args.step
    return sightings


def _settle(io_client, seconds):
    end = None
    try:
        import time
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(io_client, timeout_sec=0.05)
    except Exception:                                   # noqa: BLE001
        if end is not None:
            import time
            time.sleep(max(0.0, end - time.monotonic()))


# --- gates -----------------------------------------------------------------
# Nothing below is a tuning parameter; each one exists because its absence
# produced a specific bad outcome on 2026-08-05, when explore handed
# tag_pick_place an origin of (-0.003, -0.113) -- bearing -92 deg, radius
# 4.4 in -- and the arm dutifully went home, opened the gripper and reached for
# a point next to its own base. The estimate was already known to be bad: the
# view-to-view spread printed 334.8 mm on the same run. It was printed and then
# ignored. These turn that warning into a refusal.

# The mat is reachable and on a bench, not on the robot and not across the room.
MIN_ZONE_RADIUS_M = 0.120
MAX_ZONE_RADIUS_M = 0.320

# camera_zx/zy is the image centre carried through the tag homography. Inside
# the tag square that is interpolation; well outside it is EXTRAPOLATION, and
# zone_vision's own docstring warns a two-tag fit extrapolates ~6x across its
# thin direction. So how far out is too far depends on how well the tags span
# the square, which is exactly what the tag count reports: measured in
# half-diagonals of the zone (71.8 mm for a 101.6 mm square), four tags spanning
# both directions are trusted to two of them and three tags to one. Chosen
# against the 2026-08-05 coarse pass, where every 4-tag view out to 129 mm sits
# on the same solved mat to within 9 mm and the 3-tag views past that do not.
MAX_CENTRE_OFFSET_HALF_DIAGONALS = {4: 2.0, 3: 1.0}

# Two tags is enough for a homography and NOT enough to trust one this far from
# the tags. Four spans the zone in both directions.
MIN_TAGS_FOR_ORIGIN = 3


def gate(sighting, zone_size):
    """None if the sighting is fit for use as INPUT, else why it is not.

    Radius is deliberately not checked here. A single view's zone_origin is
    provisional -- it is built on a zone yaw nobody has solved yet -- so
    rejecting a view for implying an implausible radius would throw away good
    pixels over a bad assumption. Radius is checked once, on the fit.
    """
    if axis_hits_mat(sighting.joints) is None:
        return "optical axis does not meet the mat"
    if len(sighting.tag_ids) < MIN_TAGS_FOR_ORIGIN:
        return ("only %d tag(s); %d needed before the homography is trusted "
                "this far from them" % (len(sighting.tag_ids),
                                        MIN_TAGS_FOR_ORIGIN))
    half_diagonal = zone_size * math.sqrt(2.0) / 2.0
    allowed = half_diagonal * MAX_CENTRE_OFFSET_HALF_DIAGONALS.get(
        len(sighting.tag_ids), 1.0)
    if sighting.offset_mm / 1000.0 > allowed:
        return ("image centre is %.0f mm from the zone centre; %d tags are "
                "trusted only to %.0f mm out (%.1f half-diagonals of a %.0f mm "
                "square) -- beyond that it is extrapolated, not measured"
                % (sighting.offset_mm, len(sighting.tag_ids), allowed * 1000.0,
                   MAX_CENTRE_OFFSET_HALF_DIAGONALS.get(
                       len(sighting.tag_ids), 1.0), zone_size * 1000.0))
    return None


def gate_fit(fit):
    """None if the solved mat is believable, else why it is not."""
    if fit.residual > MAX_FIT_RESIDUAL_M:
        return ("views disagree by %.1f mm about where the mat is, past the "
                "%.0f mm limit -- one mat cannot do that, so an input is wrong"
                % (fit.residual * 1000.0, MAX_FIT_RESIDUAL_M * 1000.0))
    if not (MIN_ZONE_RADIUS_M <= fit.radius <= MAX_ZONE_RADIUS_M):
        return ("puts the mat %.0f mm from the base, outside the plausible "
                "%.0f-%.0f mm" % (fit.radius * 1000, MIN_ZONE_RADIUS_M * 1000,
                                  MAX_ZONE_RADIUS_M * 1000))
    return None


def choose(sightings, zone_size, step, yaw_fixed=None):
    """(fits, rejected_sightings, rejected_fits).

    fits are the believable mats, best first. More than one is not an error
    here -- it is a finding, and the caller says so out loud rather than
    silently picking.
    """
    usable, rejected = [], []
    for sighting in sightings:
        why = gate(sighting, zone_size)
        (rejected if why else usable).append((sighting, why))

    fits, rejected_fits = [], []
    for run in split_runs([pair[0] for pair in usable], step):
        fit, why = fit_zone(run, yaw_fixed)
        if fit is None:
            rejected_fits.append((run, why))
            continue
        why = gate_fit(fit)
        (rejected_fits if why else fits).append((run, why) if why else fit)

    # Most views first: the yaw solution improves with baseline, and a long run
    # is also the strongest evidence that this is the mat rather than a glimpse
    # of something else. Residual breaks ties.
    fits.sort(key=lambda f: (-len(f.sightings), f.residual))
    return fits, rejected, rejected_fits


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone", choices=("pickup", "place"), default="pickup")
    parser.add_argument("--start", type=float, default=J1_START_DEG)
    parser.add_argument("--end", type=float, default=J1_END_DEG)
    parser.add_argument("--step", type=float, default=J1_COARSE_STEP_DEG,
                        help="coarse step in degrees (default %(default)s)")
    parser.add_argument("--fine-step", type=float, default=J1_FINE_STEP_DEG,
                        help="fine step in degrees (default %(default)s)")
    parser.add_argument("--fine-span", type=float, default=J1_FINE_SPAN_DEG,
                        help="how far either side of the best coarse hit the "
                             "fine pass sweeps (default %(default)s deg)")
    parser.add_argument("--single-pass", action="store_true",
                        help="one sweep at --step, no fine refinement")
    parser.add_argument("--force", action="store_true",
                        help="hand off even when the views disagree. Look at "
                             "the camera first.")
    parser.add_argument("--pitch", type=float, default=EXPLORE_PITCH_DEG,
                        help="J4 in degrees; -71 aims the optical axis at a "
                             "zone 9 in out (default %(default)s)")
    parser.add_argument("--wrist", type=float, default=EXPLORE_WRIST_DEG)
    parser.add_argument("--zone-yaw", type=float, default=None,
                        help="zone rotation about +Z in DEGREES. Omit it -- "
                             "explore solves the yaw from the sweep, and a "
                             "wrong one silently ruins the radius (this used "
                             "to default to 0 and did exactly that).")
    parser.add_argument("--settle", type=float, default=SETTLE_SECONDS)
    parser.add_argument("--no-reset", action="store_true",
                        help="skip the move to the reset pose first")
    parser.add_argument("--dry-run", action="store_true",
                        help="report the geometry of the sweep and exit "
                             "WITHOUT moving the arm")
    parser.add_argument("--print-command", action="store_true",
                        help="print the tag_pick_place.py command line to run")
    parser.add_argument("--run-pick", action="store_true",
                        help="run tag_pick_place.py --dry-run at the found "
                             "origin once the sweep finishes")
    args = parser.parse_args()

    if args.step <= 0:
        raise SystemExit("--step must be positive")
    if args.zone_yaw is not None:
        args.zone_yaw = math.radians(args.zone_yaw)

    if args.dry_run:
        print("EXPLORE SWEEP, geometry only -- the arm is not touched.\n")
        print("  pose [J1, 0, 0, %.0f, 0, %.0f]" % (args.pitch, args.wrist))
        print("  %d steps of %.0f deg from J1 %+.0f to %+.0f\n"
              % (int((args.end - args.start) / args.step) + 1, args.step,
                 args.start, args.end))
        print("     J1     axis hits mat at        bearing")
        j1 = args.start
        while j1 <= args.end + 1e-9:
            hit = axis_hits_mat(joints_for(j1, args.pitch, args.wrist))
            if hit is None:
                print("   %+6.1f   (axis does not meet the mat)" % j1)
            else:
                r = math.hypot(hit[0], hit[1])
                print("   %+6.1f   r %.3f m (%4.1f in)      %+7.1f deg"
                      % (j1, r, r / 0.0254, math.degrees(math.atan2(hit[1], hit[0]))))
            j1 += args.step
        return 0

    rclpy.init()
    try:
        io_client = pp.RobotIOClient()
        io_client.wait_for_joint_states(timeout_sec=10.0)

        # The zone pose handed to the detector only affects the WORLD
        # conversion of block positions, which explore ignores -- it reads
        # tag_ids and camera_zx/zy, both of which are in the mat's own frame
        # and untouched by this. Zero, not args.zone_yaw, so nothing downstream
        # can quietly depend on a yaw that has not been solved yet.
        detector = tpp.Detector(io_client, 0.0, NOMINAL_ZONE_RADIUS_M, 0.0,
                                0.0, tpp.zv.DEFAULT_ZONE_SIZE
                                if hasattr(tpp, "zv") else 0.1016)
        if not detector.wait_for_service(timeout=15.0):
            return 1

        if not args.no_reset:
            print("[explore] reset to %s" % (RESET_JOINTS_DEG,))
            if not send_joints(io_client,
                               [math.radians(d) for d in RESET_JOINTS_DEG],
                               MOVE_SECONDS, "reset"):
                return 1

        print("[explore] pitch %.0f\n" % args.pitch)
        if args.single_pass:
            sightings = sweep(io_client, detector, args)
        else:
            coarse, fine = coarse_then_fine(io_client, detector, args)
            # The fine pass replaces the coarse one rather than adding to it.
            # Mixing them would let a coarse sighting -- taken specifically
            # where the framing is worst -- win on tag count by luck.
            sightings = fine or coarse

        print()
        if not sightings:
            print("[explore] the %s zone was never seen.\n"
                  "  - are its tags (%s) actually on the mat?\n"
                  "  - is block_detector_node.py running ON THE PI?\n"
                  "  - try --pitch -80 to look closer in, or --step 20 for a\n"
                  "    finer sweep; --dry-run prints where each step aims."
                  % (args.zone,
                     "0-3" if args.zone == "pickup" else "4-7"))
            return 1

        zone_size = detector.zone_size
        step = args.step if args.single_pass else args.fine_step
        fits, rejected, rejected_fits = choose(sightings, zone_size, step,
                                               args.zone_yaw)

        if rejected:
            print("[explore] %d sighting(s) REJECTED as input:" % len(rejected))
            for sighting, why in rejected:
                print("            J1 %+7.1f  %s" % (sighting.j1_deg, why))
        for run, why in rejected_fits:
            print("[explore] run J1 %+.1f..%+.1f (%d views) REJECTED: %s"
                  % (run[0].j1_deg, run[-1].j1_deg, len(run), why))

        if not fits:
            print("\n[explore] NO USABLE MAT. Nothing is handed off.\n"
                  "  Any origin printed here would be a guess -- and a guess\n"
                  "  sends the arm at a physical target. Look at what the\n"
                  "  camera actually sees:\n"
                  "    robot:  python3 block_tag_probe.py --show\n"
                  "  then drive to the most promising J1 by hand:\n"
                  "    mars :  python3 joint_trajectory_test.py --degrees "
                  "<J1> 0 0 %.0f 0 %.0f" % (args.pitch, args.wrist))
            return 1

        best = fits[0]
        if len(fits) > 1:
            print("\n[explore] %d SEPARATE TAG SQUARES with %s ids in the "
                  "workspace:" % (len(fits), args.zone))
            for fit in fits:
                print("            J1 %+.1f..%+.1f  %s"
                      % (fit.sightings[0].j1_deg, fit.sightings[-1].j1_deg,
                         fit.describe()))
            print("          Each one is internally consistent, so this is not "
                  "noise -- there\n"
                  "          really is more than one mat carrying these tags. "
                  "Taking the one\n"
                  "          with the most views; remove the other, or pass "
                  "--start/--end to\n"
                  "          restrict the sweep, if that is the wrong choice.")

        print("\n[explore] solved from %d view(s) J1 %+.1f..%+.1f"
              % (len(best.sightings), best.sightings[0].j1_deg,
                 best.sightings[-1].j1_deg))
        c, s = math.cos(best.yaw), math.sin(best.yaw)
        for sighting in best.sightings:
            hit = axis_hits_mat(sighting.joints)
            here_x = hit[0] - (c * sighting.camera_zx - s * sighting.camera_zy)
            here_y = hit[1] - (s * sighting.camera_zx + c * sighting.camera_zy)
            print("            J1 %+7.1f  %d tags  offset %5.1f mm  -> "
                  "(%+.4f, %+.4f)"
                  % (sighting.j1_deg, len(sighting.tag_ids),
                     sighting.offset_mm, here_x, here_y))

        print("\n[explore] ZONE ORIGIN  x %+.4f  y %+.4f  (r %.4f m = %.2f in, "
              "bearing %+.1f deg)"
              % (best.origin[0], best.origin[1], best.radius,
                 best.radius / 0.0254, best.bearing_deg))
        print("[explore] zone yaw %+.1f deg (%s), views agree to %.1f mm over "
              "a %.0f mm baseline"
              % (math.degrees(best.yaw),
                 "given" if args.zone_yaw is not None else "solved",
                 best.residual * 1000.0, best.baseline * 1000.0))

        # --zone-yaw MUST be passed. tag_pick_place defaults it to 0, and a
        # zone rotated 180 deg with yaw 0 assumed mirrors every block position
        # through the zone centre -- a block 30 mm one side of centre is
        # reached for 30 mm the other side, 60 mm out, and the correction loop
        # closes on the CAMERA rather than the block so it never notices.
        # Explore solves the yaw; dropping it on the floor here would waste it.
        # Degrees, because that is what tag_pick_place's --zone-yaw takes.
        command = ("python3 tag_pick_place.py --zone-origin %.4f %.4f 0.0 "
                   "--zone-yaw %.1f"
                   % (best.origin[0], best.origin[1], math.degrees(best.yaw)))
        if args.print_command or args.run_pick:
            print("\n%s --dry-run" % command)

        if args.run_pick:
            if len(fits) > 1 and not args.force:
                print("\n[explore] NOT handing off -- more than one mat "
                      "matched, and picking\n"
                      "  the wrong one sends the arm at a real point on the "
                      "other side of\n"
                      "  the robot. Restrict the sweep, or re-run with --force.")
                return 1
            print("\n[explore] handing off...\n")
            here = os.path.dirname(os.path.abspath(__file__))
            return subprocess.call(command.split() + ["--dry-run"], cwd=here)
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------------------------
# Replay of the 2026-08-05 sweep
# ---------------------------------------------------------------------------
# Real numbers off the real arm, copied out of logs.txt: (J1 deg, camera_zx mm,
# camera_zy mm) for every step that saw >= 3 tags. Kept here because they are
# the only evidence this project has that the fit works on hardware rather than
# on made-up geometry, and because they encode the two-mat surprise.
REPLAY_COARSE = [
    (-125.0, -96.2, -121.6), (-115.0, -76.6, -97.6), (-105.0, -61.6, -70.2),
    (-95.0, -52.0, -41.4), (-85.0, -47.4, -11.2), (-75.0, -48.1, 19.3),
    (55.0, 105.1, 120.1), (65.0, 86.6, 95.9), (75.0, 73.1, 68.7),
    (85.0, 64.1, 39.7), (95.0, 60.3, 9.8), (105.0, 61.7, -20.6),
    (115.0, 68.3, -50.3),
]
REPLAY_FINE = [
    (-97.5, -52.5, -42.7), (-95.0, -52.1, -40.7), (-92.5, -50.6, -33.9),
    (-90.0, -49.2, -25.9), (-87.5, -48.2, -17.3), (-85.0, -47.6, -9.6),
    (-82.5, -47.4, -3.4), (-80.0, -47.4, 5.3), (-77.5, -47.7, 12.4),
    (-75.0, -48.5, 20.9), (-72.5, -49.6, 28.8),
]


class _ReplaySighting(object):
    """Just enough of a Sighting for fit_zone and split_runs."""

    def __init__(self, j1_deg, zx_mm, zy_mm, pitch, wrist):
        self.j1_deg = j1_deg
        self.joints = joints_for(j1_deg, pitch, wrist)
        self.camera_zx = zx_mm / 1000.0
        self.camera_zy = zy_mm / 1000.0
        self.tag_ids = [0, 1, 2, 3]

    @property
    def offset_mm(self):
        return math.hypot(self.camera_zx, self.camera_zy) * 1000.0


def selftest(pitch=EXPLORE_PITCH_DEG, wrist=EXPLORE_WRIST_DEG):
    failures = []

    def check(name, ok, detail=""):
        print("  %-46s %s%s" % (name, "ok" if ok else "FAIL",
                                "" if ok else "  " + detail))
        if not ok:
            failures.append(name)

    def replay(rows):
        return [_ReplaySighting(j1, zx, zy, pitch, wrist) for j1, zx, zy in rows]

    print("fine pass, solved yaw:")
    fit, why = fit_zone(replay(REPLAY_FINE))
    check("fine pass fits", fit is not None, str(why))
    if fit:
        print("    %s" % fit.describe())
        # 9 in = 0.2286 m, hand-placed. 20 mm of slack covers the tape measure,
        # MAT_SURFACE_Z and the uncalibrated principal point together.
        check("fine radius within 20 mm of the 9 in it was placed at",
              abs(fit.radius - 0.2286) < 0.020, "%.4f m" % fit.radius)
        check("fine residual under the gate", gate_fit(fit) is None,
              str(gate_fit(fit)))
        check("solved yaw is near -90 deg",
              abs(math.degrees(fit.yaw) + 90.0) < 5.0,
              "%+.1f deg" % math.degrees(fit.yaw))

    # The bug, reproduced. Assuming yaw = 0 leaves the radial correction
    # unapplied, so the answer collapses onto the axis-hit radius.
    bad, _ = fit_zone(replay(REPLAY_FINE), yaw_fixed=0.0)
    print("\nfine pass, yaw forced to 0 (the 2026-08-05 bug):")
    print("    %s" % bad.describe())
    check("yaw=0 is rejected by the fit gate", gate_fit(bad) is not None,
          "it passed, which means the gate is useless")

    print("\ncoarse pass, two mats:")
    fits, rejected, rejected_fits = choose(replay(REPLAY_COARSE), 0.1016,
                                           J1_COARSE_STEP_DEG)
    check("coarse pass finds exactly 2 mats", len(fits) == 2,
          "%d found, %d runs rejected" % (len(fits), len(rejected_fits)))
    for one in fits:
        print("    J1 %+.1f..%+.1f  %s"
              % (one.sightings[0].j1_deg, one.sightings[-1].j1_deg,
                 one.describe()))
    if len(fits) == 2:
        apart = abs(fits[0].bearing_deg - fits[1].bearing_deg)
        check("the two are ~180 deg apart", abs(apart - 180.0) < 5.0,
              "%.1f deg" % apart)
        check("both sit 9-10 in out",
              all(0.22 < one.radius < 0.27 for one in fits),
              str(["%.4f" % one.radius for one in fits]))

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(main())
