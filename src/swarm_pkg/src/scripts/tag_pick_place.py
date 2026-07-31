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
    python3 tag_pick_place.py --zone-origin 0.0 0.25 0.0 --dry-run
    python3 tag_pick_place.py --zone-origin 0.0 0.25 0.0

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
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import tool_frame_check  # noqa: E402
import zone_vision as zv  # noqa: E402
from pick_place import (  # noqa: E402
    GRASP_OFFSET_Z,
    GRIPPER_OPEN,
    HOME_RADIANS,
    MAX_HOVER_Z,
    PLACE_XYZ,
    RobotIOClient,
    cartesian_move_to,
    go_home,
    gripper_close_until_contact,
    hover_z,
    move_arm_to,
)
from swarm_interfaces.srv import DetectBlock  # noqa: E402

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
# Stage 1 block: 1.18 in square, the thickness quoted in APRIL_TAGS.md.
DEFAULT_BLOCK_THICKNESS = 0.030      # m

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
DETECT_SERVICE_TIMEOUT = 20.0        # s; a still plus vision on the Pi
SETTLE_AFTER_MOVE_SEC = 0.6          # let the arm stop ringing before a still

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
# Four offsets at 90 deg is the professor's suggestion and it is a good one: it
# guarantees every tag is visible in at least one still regardless of which pair
# the gripper starts out hiding.
MULTIVIEW_YAW_OFFSETS_DEG = (0.0, 90.0, 180.0, 270.0)

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
# a zone centre 254 mm out would need the FLANGE at 295 mm -- past this arm's
# reach. Rotating the wrist 180 deg swings the lens to the far side, so the same
# lens position needs the flange at only 213 mm, which is precisely where the arm
# already parked this run. The flip tells the code which side the lens is on; this
# picks the wrist angle that makes the required flange position reachable.
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


def reduce_yaw(yaw_rad, symmetry):
    """Fold a block's yaw into the smallest equivalent rotation.

    A square looks the same every 90 degrees, so there is no reason to twist the
    wrist 80 degrees when 10 the other way is the same grasp. Beyond saving
    motion this keeps joint6output_to_joint6 away from its -2.4434 rad limit,
    which _is_near_joint_limit() rejects outright -- an unreduced yaw is a way
    to make IK fail for reasons that look like nothing to do with yaw.
    """
    if not symmetry:                 # 0 = continuous (a circle): any yaw works
        return 0.0
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
        print("[detect] tags %s | rms %.2f px | %.0f px/m | camera at zone "
              "(%+.1f, %+.1f) mm"
              % (list(response.tag_ids), response.homography_rms,
                 response.scale_px_per_m,
                 response.camera_zx * 1000.0, response.camera_zy * 1000.0))
        for index, block in enumerate(response.blocks):
            print("[detect]   [%d] zone (%+.1f, %+.1f) mm  yaw %+.1f deg  "
                  "%.1f x %.1f mm  %s sym=%d"
                  % (index, block.zx * 1000, block.zy * 1000,
                     math.degrees(block.yaw), block.width * 1000,
                     block.length * 1000, block.shape, block.symmetry))
        return response if response.success else None

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


def _response_to_detections(response):
    """The service's BlockDetection[] as zone_vision.Detection objects.

    fuse_detections only reads zone-local fields, so the world-frame ones are
    left out on purpose -- they are derived from the caller's zone survey and
    would add the survey's error to a comparison between views that all share it.
    """
    return [zv.Detection(zx=b.zx, zy=b.zy, zyaw=b.zyaw, width=b.width,
                         length=b.length, shape=b.shape, symmetry=b.symmetry,
                         fill_ratio=b.fill_ratio, area_px=b.area_px, box_px=None)
            for b in response.blocks]


def detect_multiview(io_client, detector, zone, x, y, z, base_yaw_deg,
                     holding_block=False, debug_prefix=None):
    """Several stills at different wrist yaws, fused into one answer.

    Returns (fused_blocks, views_used, tag_ids_union) -- fused_blocks is a list
    of zone_vision.FusedDetection sorted by how many views agreed on them.

    A still that fails is logged and skipped rather than aborting the pass: with
    four offsets, losing one to glare or a marginal tag still leaves plenty.
    """
    per_view = []
    ids = set()
    used = 0

    for index, offset in enumerate(MULTIVIEW_YAW_OFFSETS_DEG):
        if used >= MULTIVIEW_ENOUGH_VIEWS:
            print("[multiview] %d usable views, skipping the remaining %d still(s)"
                  % (used, len(MULTIVIEW_YAW_OFFSETS_DEG) - index))
            break

        yaw = base_yaw_deg + offset
        print("\n[multiview] still %d/%d at wrist yaw %+.0f deg (offset %+.0f)"
              % (index + 1, len(MULTIVIEW_YAW_OFFSETS_DEG), yaw, offset))
        if not move_arm_to(io_client, x, y, z, block_yaw_deg=yaw,
                           holding_block=holding_block):
            print("[multiview]   move failed, skipping this view")
            continue
        time.sleep(SETTLE_AFTER_MOVE_SEC)

        debug = ("%s_view%d.png" % (debug_prefix, index)) if debug_prefix else None
        response = detector.detect(zone, debug_image=debug)
        if response is None:
            print("[multiview]   no usable homography from this view")
            continue

        used += 1
        ids.update(response.tag_ids)
        per_view.append(_response_to_detections(response))

    if not per_view:
        print("[multiview] NO usable view. This is a framing, focus or lighting "
              "problem -- check that any tag is visible at all before "
              "suspecting the geometry.")
        return [], 0, sorted(ids)

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
    return fused, used, sorted(ids)


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

    Returns (response, converged, flange_world_xy) or (None, False, None).
    """
    target_world = detector.zone_to_world(*target_zone_xy)
    offset = camera_offset_world(block_yaw_deg, target_world[0], target_world[1])
    # Command the FLANGE such that the CAMERA lands on the target.
    flange = (target_world[0] - offset[0], target_world[1] - offset[1])

    print("\n[%s] want camera at zone (%+.1f, %+.1f) mm -> world (%.4f, %.4f)"
          % (label, target_zone_xy[0] * 1000, target_zone_xy[1] * 1000,
             target_world[0], target_world[1]))
    print("[%s] lens offset (%+.1f, %+.1f) mm -> flange to (%.4f, %.4f), yaw %+.1f deg"
          % (label, offset[0] * 1000, offset[1] * 1000, flange[0], flange[1],
             block_yaw_deg))

    response = None
    for attempt in range(MAX_CORRECTIONS + 1):
        if not move_arm_to(io_client, flange[0], flange[1], hover_height,
                           block_yaw_deg=block_yaw_deg):
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
        flange = (flange[0] + delta[0], flange[1] + delta[1])
        print("[%s] correcting by (%+.1f, %+.1f) mm -> flange (%.4f, %.4f)"
              % (label, delta[0] * 1000, delta[1] * 1000, flange[0], flange[1]))

    print("[%s] did NOT converge after %d corrections. Refusing to descend on "
          "a position this uncertain." % (label, MAX_CORRECTIONS))
    return response, False, flange


def run_stage1(io_client, detector, args, log):
    # DETECTION height, not grasp-approach height -- these are different
    # questions and using one formula for both was a real bug (2026-07-30,
    # first hardware run: hovered at 0.16 m, computed from block_thickness +
    # GRASP_OFFSET_Z + APPROACH_HEIGHT, and saw ZERO of 4 tags -- while
    # MAX_HOVER_Z = 0.205 m of reachable height sat unused).
    #
    # A short hover is right for a plain Cartesian approach to a KNOWN point --
    # that is what hover_z(target_z) is for, and pick_place.py's PICK_XYZ flow
    # uses it correctly. But hover_and_detect's job is to SEE the tags, and
    # every millimetre of height only helps that: it widens the camera's view
    # of the zone with no accuracy cost, because the actual grasp descent is a
    # separate, later Cartesian move from wherever this hover ends up -- this
    # height does not propagate into grasp_z (computed independently below).
    # There is no reason to hover any lower than the arm can reach.
    #
    # So: hover at the reachable ceiling for BOTH detect passes (survey and
    # grasp-hover), full stop. If tags are still not seen from here, the cause
    # is not hover height -- see the framing checklist in APRIL_TAGS.md
    # "Stage 0b.2 FOV go/no-go".
    hover = MAX_HOVER_Z
    print("[stage1] detect hover height %.4f m (MAX_HOVER_Z, for widest "
          "camera view of the zone)" % hover)

    if not go_home(io_client):
        return False
    if not io_client.gripper_move_to(GRIPPER_OPEN):
        print("[stage1] could not open the gripper")
        return False

    # --- 1. survey the zone, camera over the zone centre -------------------
    # DETECT_WRIST_YAW_DEG, not 0: the lens is on the near side of the flange, so
    # at yaw 0 reaching the zone centre would need a 295 mm flange. See the
    # constant for the arithmetic.
    response, converged, _ = hover_and_detect(
        io_client, detector, log, (0.0, 0.0), DETECT_WRIST_YAW_DEG, hover,
        "survey", debug_image=args.debug_image)
    if response is None:
        return False
    if not converged:
        print("[stage1] the survey hover never settled onto the zone centre. "
              "The block position below is still valid (it comes from the "
              "tags), but framing may be marginal.")
    if not response.blocks:
        print("[stage1] zone is empty -- nothing to pick.")
        return False

    block = response.blocks[0]
    grasp_yaw = reduce_yaw(block.yaw, block.symmetry)
    grasp_yaw_deg = math.degrees(grasp_yaw)
    print("\n[stage1] block: %.1f x %.1f mm %s, zone (%+.1f, %+.1f) mm, "
          "yaw %+.1f deg -> grasp yaw %+.1f deg (symmetry %d)"
          % (block.width * 1000, block.length * 1000, block.shape,
             block.zx * 1000, block.zy * 1000, math.degrees(block.yaw),
             grasp_yaw_deg, block.symmetry))

    usable = args.zone_size / 2.0 - args.tag_size / 2.0 - max(block.width, block.length) / 2.0
    if max(abs(block.zx), abs(block.zy)) > usable:
        print("[stage1] WARNING: block centre is %.1f mm off, past the %.1f mm "
              "at which it starts covering a tag. See APRIL_TAGS.md 'Usable "
              "area'." % (max(abs(block.zx), abs(block.zy)) * 1000, usable * 1000))

    # --- 2. hover with the GRIPPER over the block, at the block's yaw ------
    # Target the camera at the block position PLUS the lens offset, so that when
    # the camera gets there the flange -- and therefore the jaws -- is on the
    # block. Reusing the same converge-on-the-camera loop is the point: it is
    # the only thing here that measures the arm.
    offset = camera_offset_world(grasp_yaw_deg, detector.zone_x, detector.zone_y)
    offset_zone = detector.world_to_zone(detector.zone_x + offset[0],
                                         detector.zone_y + offset[1])
    camera_target = (block.zx + offset_zone[0], block.zy + offset_zone[1])

    # Detect at the flipped wrist yaw for the same reachability reason as the
    # survey. The DESCENT below still uses grasp_yaw_deg -- rotating the wrist
    # about its own axis changes the jaws' orientation, not the flange position,
    # so the grasp is unaffected by having detected from the other side.
    response, converged, _ = hover_and_detect(
        io_client, detector, log, camera_target,
        grasp_yaw_deg + DETECT_WRIST_YAW_DEG, hover, "grasp-hover",
        debug_image=args.debug_image)
    if response is None:
        return False
    if not converged:
        return False

    grasp_x, grasp_y = detector.zone_to_world(block.zx, block.zy)
    grasp_z = args.zone_z + args.block_thickness / 2.0 + GRASP_OFFSET_Z
    print("\n[stage1] grasp target: world (%.4f, %.4f, %.4f), yaw %+.1f deg"
          % (grasp_x, grasp_y, grasp_z, grasp_yaw_deg))

    if args.dry_run:
        print("[stage1] --dry-run: stopping before the descent. Measure where "
              "the block actually is and compare against the numbers above.")
        return True

    # --- 3. descend, grasp, retreat ---------------------------------------
    steps = [
        ("Descend to grasp",
         lambda: cartesian_move_to(io_client, grasp_x, grasp_y, grasp_z,
                                   block_yaw_deg=grasp_yaw_deg)),
        ("Close gripper",
         lambda: gripper_close_until_contact(io_client)),
        ("Retreat after grasp",
         lambda: cartesian_move_to(io_client, grasp_x, grasp_y, hover,
                                   allow_fallback=True,
                                   block_yaw_deg=grasp_yaw_deg)),
    ]

    # --- 4. place, unchanged from pick_place.py ---------------------------
    # Stage 1 places at the hardcoded PLACE_XYZ. Stage 2 replaces this with a
    # second detection at the place zone.
    place_x, place_y, place_surface_z = PLACE_XYZ
    place_z = place_surface_z + args.block_thickness / 2.0 + GRASP_OFFSET_Z
    place_hover = hover_z(place_z)
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
            return False
        time.sleep(0.5)
    return True


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone-origin", type=float, nargs=3,
                        metavar=("X", "Y", "Z"), default=[0.0, 0.25, 0.0],
                        help="SURVEYED world pose of the pickup zone centre and "
                             "the plane the blocks sit on. Nothing measures this "
                             "-- every world coordinate this script reports is "
                             "only as good as this number (default: %(default)s)")
    parser.add_argument("--zone-yaw", type=float, default=0.0,
                        help="rotation of the tag square about world +Z, degrees")
    parser.add_argument("--zone-size", type=float, default=zv.DEFAULT_ZONE_SIZE,
                        help="side of the square joining the TAG CENTRES, metres "
                             "(default: %(default)s = 6 in, around a ~4 in "
                             "working area -- see APRIL_TAGS.md 'Usable area')")
    parser.add_argument("--tag-size", type=float, default=zv.DEFAULT_TAG_SIZE,
                        help="printed tag side, metres, for the usable-area "
                             "warning only (default: %(default)s = 1 in)")
    parser.add_argument("--block-thickness", type=float,
                        default=DEFAULT_BLOCK_THICKNESS,
                        help="Stage 1 block thickness, metres (default: %(default)s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="hover and detect and print the grasp pose, but do "
                             "not descend or grasp")
    parser.add_argument("--debug-image", metavar="PATH",
                        help="path ON THE PI for the detector's annotated frame")
    parser.add_argument("--log", metavar="CSV",
                        help="append (commanded correction, measured result) rows "
                             "here -- the disturbance-observer dataset")
    return parser.parse_args()


def main():
    args = parse_args()
    args.zone_z = args.zone_origin[2]
    zone_yaw_rad = math.radians(args.zone_yaw)

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
