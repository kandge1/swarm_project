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

        origin = zone_origin_from(joints, response.camera_zx,
                                  response.camera_zy, args.zone_yaw)
        sighting = Sighting(j1, joints, response, origin, measured)
        sightings.append(sighting)
        print("[explore] J1 %+7.1f  tags %-14s rms %.2f px  centre offset "
              "%5.1f mm  -> zone (%+.4f, %+.4f)%s"
              % (j1, str(sighting.tag_ids), sighting.rms, sighting.offset_mm,
                 origin[0] if origin else float("nan"),
                 origin[1] if origin else float("nan"),
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

# Independent views of one mat must agree. Anything worse means the estimate is
# not merely imprecise -- some input is wrong, and averaging wrong inputs is how
# a confident wrong answer gets made.
MAX_VIEW_SPREAD_M = 0.040

# camera_zx/zy is the image centre carried through the tag homography. Inside
# the tag square that is interpolation; well outside it is EXTRAPOLATION, and
# zone_vision's own docstring warns a two-tag fit extrapolates ~6x across its
# thin direction. Every sighting in the failed run sat 59-186 mm out, i.e.
# entirely outside a 101.6 mm square.
MAX_CENTRE_OFFSET_M = 0.070

# Two tags is enough for a homography and NOT enough to trust one this far from
# the tags. Four spans the zone in both directions.
MIN_TAGS_FOR_ORIGIN = 3


def gate(sighting, zone_size):
    """None if the sighting is usable, else why it is not."""
    if sighting.zone_origin is None:
        return "optical axis does not meet the mat"
    if len(sighting.tag_ids) < MIN_TAGS_FOR_ORIGIN:
        return ("only %d tag(s); %d needed before the homography is trusted "
                "this far from them" % (len(sighting.tag_ids),
                                        MIN_TAGS_FOR_ORIGIN))
    if sighting.offset_mm / 1000.0 > MAX_CENTRE_OFFSET_M:
        return ("image centre is %.0f mm from the zone centre, outside the "
                "%.0f mm tag square -- extrapolated, not measured"
                % (sighting.offset_mm, zone_size * 1000.0))
    radius = math.hypot(*sighting.zone_origin)
    if not (MIN_ZONE_RADIUS_M <= radius <= MAX_ZONE_RADIUS_M):
        return ("implies a zone %.0f mm from the base, outside the plausible "
                "%.0f-%.0f mm" % (radius * 1000, MIN_ZONE_RADIUS_M * 1000,
                                  MAX_ZONE_RADIUS_M * 1000))
    return None


def choose(sightings, zone_size):
    """(best, accepted, rejected). best is None when nothing survives.

    Tag count ranks first because it drives the homography's conditioning --
    four tags spanning the zone beat two spanning 25 mm of it. Centre offset
    breaks ties, standing in for "least extrapolation".
    """
    accepted, rejected = [], []
    for sighting in sightings:
        why = gate(sighting, zone_size)
        (rejected if why else accepted).append((sighting, why))
    if not accepted:
        return None, [], rejected
    accepted.sort(key=lambda pair: (-len(pair[0].tag_ids), pair[0].offset_mm))
    return accepted[0][0], [pair[0] for pair in accepted], rejected


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
    parser.add_argument("--zone-yaw", type=float, default=0.0,
                        help="zone rotation about +Z, radians (default 0)")
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

        detector = tpp.Detector(io_client, 0.0, NOMINAL_ZONE_RADIUS_M, 0.0,
                                args.zone_yaw, tpp.zv.DEFAULT_ZONE_SIZE
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
        best, accepted, rejected = choose(sightings, zone_size)

        if rejected:
            print("[explore] %d sighting(s) REJECTED:" % len(rejected))
            for sighting, why in rejected:
                print("            J1 %+7.1f  %s" % (sighting.j1_deg, why))
        if best is None:
            print("\n[explore] NO USABLE SIGHTING. Nothing is handed off.\n"
                  "  Every view failed a sanity gate, so any origin printed\n"
                  "  here would be a guess -- and a guess sends the arm at a\n"
                  "  physical target. Look at what the camera actually sees:\n"
                  "    robot:  python3 block_tag_probe.py --show\n"
                  "  then drive to the most promising J1 by hand:\n"
                  "    mars :  python3 joint_trajectory_test.py --degrees "
                  "<J1> 0 0 %.0f 0 %.0f" % (args.pitch, args.wrist))
            return 1

        spread = _spread(accepted)
        print("\n[explore] %d usable sighting(s); best is J1 %+.1f with %d tags"
              % (len(accepted), best.j1_deg, len(best.tag_ids)))
        for sighting in accepted:
            print("            J1 %+7.1f  %d tags  offset %5.1f mm  -> "
                  "(%+.4f, %+.4f)"
                  % (sighting.j1_deg, len(sighting.tag_ids), sighting.offset_mm,
                     sighting.zone_origin[0], sighting.zone_origin[1]))

        radius = math.hypot(*best.zone_origin)
        print("\n[explore] ZONE ORIGIN  x %+.4f  y %+.4f  (r %.4f m = %.2f in, "
              "bearing %+.1f deg)"
              % (best.zone_origin[0], best.zone_origin[1], radius,
                 radius / 0.0254,
                 math.degrees(math.atan2(best.zone_origin[1],
                                         best.zone_origin[0]))))

        trustworthy = True
        if spread is not None:
            print("[explore] independent views agree to %.1f mm" % (spread * 1000))
            if spread > MAX_VIEW_SPREAD_M:
                trustworthy = False
                print("[explore] ^ that is worse than the %.0f mm limit. Views "
                      "of ONE mat cannot\n"
                      "          disagree this much unless an input is wrong "
                      "-- suspect the\n"
                      "          arm not being where FK thinks, or more than "
                      "one tag set in\n"
                      "          the workspace." % (MAX_VIEW_SPREAD_M * 1000))

        command = ("python3 tag_pick_place.py --zone-origin %.4f %.4f 0.0"
                   % (best.zone_origin[0], best.zone_origin[1]))
        if args.print_command or args.run_pick:
            print("\n%s --dry-run" % command)

        if args.run_pick:
            if not trustworthy and not args.force:
                print("\n[explore] NOT handing off -- the views disagree.\n"
                      "  On 2026-08-05 a 335 mm disagreement was printed and\n"
                      "  handed off anyway, and the arm reached for a point\n"
                      "  next to its own base. Re-run with --force only if you\n"
                      "  have looked at the camera and believe this number.")
                return 1
            print("\n[explore] handing off...\n")
            here = os.path.dirname(os.path.abspath(__file__))
            return subprocess.call(command.split() + ["--dry-run"], cwd=here)
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _spread(sightings):
    """Largest disagreement between independent views, metres.

    Several views of the same mat should place it in the same spot. When they
    do not, the estimate is not merely imprecise -- something upstream is
    wrong (FK, the principal-point assumption, or a mis-set --zone-yaw), and a
    single confident-looking number would hide it.
    """
    origins = [s.zone_origin for s in sightings if s.zone_origin is not None]
    if len(origins) < 2:
        return None
    return max(math.dist(a, b) for a in origins for b in origins)


if __name__ == "__main__":
    sys.exit(main())
