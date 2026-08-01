#!/usr/bin/env python3
"""Synthetic round-trip test for zone_vision.py. No robot, no camera, no ROS.

Renders a zone with a block at a KNOWN zone-local pose, warps it through a
random perspective to imitate an off-axis camera, and checks that analyze()
recovers what was put in.

This exists because the failure mode that matters most here is SILENT. A
homography fitted with the tag ids assigned to the wrong corners, or with the
tag corner order mirrored, is still a perfect fit -- the residual stays near
zero and every reported position is confidently wrong, mirrored or rotated by
90 degrees. No amount of staring at a live image reliably catches that. A
synthetic scene with known ground truth does, in about a second.

    python3 zone_vision_selftest.py            # both segmentation methods
    python3 zone_vision_selftest.py --method otsu

What it does NOT test: real lighting, real lens distortion, real block
materials, motion blur. Those are what the still-image corpus is for. A pass
here means the geometry is right, not that the thresholds are.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402

RENDER_SCALE = 4000.0     # px per metre in the synthetic top-down render
# Must clear the zone (tag-to-tag) plus perspective jitter with margin, or tags
# and blocks near a corner clip against the canvas edge -- a rendering artifact
# that looks exactly like a real detection bug (partial tags, split contours).
# Sized for zv.DEFAULT_ZONE_SIZE at import time below; if that grows again,
# this must grow with it.
CANVAS = 1100             # px
MAT_GREY = 210
BLOCK_GREY = 60

# Accuracy gates. Generous relative to what the pipeline actually achieves
# (~0.3 mm / ~2 deg at the time of writing) so this fails on a geometry BUG,
# not on a threshold tweak. Tighten only alongside a real measurement.
MAX_POSITION_ERR_M = 0.002
MAX_YAW_ERR_RAD = math.radians(5.0)
MAX_DIMENSION_ERR_M = 0.006


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker_image(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def _zone_to_render_px(x, y):
    """Zone metres -> synthetic top-down pixels.

    Image +Y is down and zone +Y is up, so this flips Y. That flip is exactly
    what makes cv2.aruco's corner order (top-left first, in the marker's own
    frame) line up with zone_vision.TAG_CORNER_OFFSETS -- if the two ever
    disagree, this test reports a mirrored yaw and that is the place to look.
    """
    return (CANVAS / 2.0 + x * RENDER_SCALE, CANVAS / 2.0 - y * RENDER_SCALE)


def render_zone(zone, blocks=(), skip_tags=()):
    """Top-down synthetic image. blocks: [(zx, zy, yaw, width, length)]."""
    img = np.full((CANVAS, CANVAS), MAT_GREY, np.uint8)
    dictionary = _dictionary()
    tag_px = int(round(zone.tag_size * RENDER_SCALE))
    half = zone.zone_size / 2.0

    for tag_id, (sx, sy) in zip(zone.tag_ids, zv.ZONE_CORNER_SIGNS):
        if tag_id in skip_tags:
            continue
        marker = _marker_image(dictionary, tag_id, tag_px)
        cx, cy = _zone_to_render_px(sx * half, sy * half)
        x0 = int(round(cx - tag_px / 2.0))
        y0 = int(round(cy - tag_px / 2.0))
        img[y0:y0 + tag_px, x0:x0 + tag_px] = marker

    for zx, zy, yaw, width, length in blocks:
        c, s = math.cos(yaw), math.sin(yaw)
        corners = []
        for dx, dy in ((-1, -1), (+1, -1), (+1, +1), (-1, +1)):
            ox, oy = dx * length / 2.0, dy * width / 2.0
            corners.append(_zone_to_render_px(zx + c * ox - s * oy,
                                              zy + s * ox + c * oy))
        cv2.fillConvexPoly(img, np.array(corners, np.int32), BLOCK_GREY)
    return img


def perspective_warp(img, rng, strength=0.06):
    """Imitate viewing the mat from an off-axis camera pose."""
    h, w = img.shape[:2]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    jitter = rng.uniform(-strength, strength, size=(4, 2)) * np.float32([w, h])
    warp = cv2.getPerspectiveTransform(src, (src + jitter).astype(np.float32))
    return cv2.warpPerspective(img, warp, (w, h), borderValue=MAT_GREY)


def usable_half_extent(zone, block_size):
    """How far off centre a block can sit before it covers a tag.

    The tags are centred ON the zone's vertices, so each one reaches
    tag_size/2 INWARD from the corner. A block whose corner gets that far out
    sits on top of the tag and occludes it. This is a property of the physical
    mat layout, not of the code -- see APRIL_TAGS.md, "Usable area".
    """
    return zone.zone_size / 2.0 - zone.tag_size / 2.0 - block_size / 2.0


def yaw_error(measured, truth, symmetry):
    """Angular error, folded into the block's symmetry."""
    if symmetry == 0:
        return 0.0
    period = 2.0 * math.pi / symmetry if symmetry else 2.0 * math.pi
    delta = (measured - truth + period / 2.0) % period - period / 2.0
    return abs(delta)


class Failures(list):
    def check(self, ok, label, detail=""):
        if ok:
            return True
        self.append("%s: %s" % (label, detail))
        print("  FAIL %s: %s" % (label, detail))
        return False


def test_square_sweep(method, failures):
    """A 1.18in square block across the usable area, at four rotations."""
    rng = np.random.default_rng(7)
    zone = zv.zone_for("pickup", world_x=0.0, world_y=0.25, world_yaw=0.0)
    size = 0.030
    reach = usable_half_extent(zone, size) * 0.90

    worst = {"pos": 0.0, "yaw": 0.0, "dim": 0.0, "rms": 0.0}
    trials = 0
    for zx in (-reach, 0.0, reach):
        for zy in (-reach, 0.0, reach):
            for yaw_deg in (0.0, 17.0, 33.0, 45.0):
                yaw = math.radians(yaw_deg)
                img = perspective_warp(
                    render_zone(zone, [(zx, zy, yaw, size, size)]), rng)
                res = zv.analyze(img, zone, method=method)
                label = "square (%.0f, %.0f) mm @ %.0fdeg" % (
                    zx * 1000, zy * 1000, yaw_deg)

                if not failures.check(res.success, label, res.message):
                    continue
                if not failures.check(len(res.blocks) == 1, label,
                                      "expected 1 block, got %d" % len(res.blocks)):
                    continue

                d = res.blocks[0]
                pos = math.hypot(d.zx - zx, d.zy - zy)
                yerr = yaw_error(d.zyaw, yaw, 4)
                dim = max(abs(d.width - size), abs(d.length - size))

                failures.check(d.shape == "square", label,
                               "shape %r symmetry %d" % (d.shape, d.symmetry))
                failures.check(pos <= MAX_POSITION_ERR_M, label,
                               "position off by %.2f mm" % (pos * 1000))
                failures.check(yerr <= MAX_YAW_ERR_RAD, label,
                               "yaw off by %.2f deg" % math.degrees(yerr))
                failures.check(dim <= MAX_DIMENSION_ERR_M, label,
                               "dimension off by %.2f mm" % (dim * 1000))

                worst["pos"] = max(worst["pos"], pos)
                worst["yaw"] = max(worst["yaw"], yerr)
                worst["dim"] = max(worst["dim"], dim)
                worst["rms"] = max(worst["rms"], res.homography_rms)
                trials += 1

    print("  %d trials | worst: %.2f mm, %.2f deg, %.2f mm dim, %.2f px rms"
          % (trials, worst["pos"] * 1000, math.degrees(worst["yaw"]),
             worst["dim"] * 1000, worst["rms"]))


def test_rectangle(method, failures):
    """A 1x2in rectangle: checks symmetry 2 and that yaw follows the LONG axis.

    Stage 3's whole grasp-axis rule depends on width being the short side and
    yaw naming the long one. Getting those swapped would rotate every grasp 90
    degrees -- and for a square block it would be invisible.
    """
    rng = np.random.default_rng(11)
    zone = zv.zone_for("pickup")
    width, length = 0.0254, 0.0508

    for yaw_deg in (0.0, 30.0, 75.0, 120.0):
        yaw = math.radians(yaw_deg)
        img = perspective_warp(
            render_zone(zone, [(0.0, 0.0, yaw, width, length)]), rng, strength=0.04)
        res = zv.analyze(img, zone, method=method)
        label = "rect @ %.0fdeg" % yaw_deg

        if not failures.check(res.success and res.blocks, label, res.message):
            continue
        d = res.blocks[0]
        failures.check(d.shape == "rect" and d.symmetry == 2, label,
                       "shape %r symmetry %d" % (d.shape, d.symmetry))
        failures.check(d.width < d.length, label,
                       "width %.1f not shorter than length %.1f"
                       % (d.width * 1000, d.length * 1000))
        failures.check(abs(d.width - width) < MAX_DIMENSION_ERR_M, label,
                       "short side %.1f mm, expected %.1f" % (d.width * 1000,
                                                              width * 1000))
        failures.check(abs(d.length - length) < MAX_DIMENSION_ERR_M, label,
                       "long side %.1f mm, expected %.1f" % (d.length * 1000,
                                                             length * 1000))
        failures.check(yaw_error(d.zyaw, yaw, 2) <= MAX_YAW_ERR_RAD, label,
                       "yaw %.1f deg, expected %.1f (long axis)"
                       % (math.degrees(d.zyaw), yaw_deg))


def test_grid_line_rejected(method, failures):
    """A printed-grid-line-shaped sliver must be REJECTED; the largest real
    block must still be ACCEPTED. Both in one test because the fix is a single
    threshold trying to sit between them.

    Regression test for 2026-07-31, found on real hardware: the mat is printed
    with an inch-square reference grid, and Canny fires on those lines exactly
    like it fires on a block edge. Real detections from that run measured
    134.1x16.7mm and 121.1x12.5mm objects -- physically impossible on a 152.4mm
    zone with no block anywhere near that shape. See MAX_BLOCK_LENGTH_M.
    """
    rng = np.random.default_rng(23)
    zone = zv.zone_for("pickup")
    label = "grid line rejected"

    # Matches the measured artifact shape, not just "very long" -- if the fix
    # regresses to a looser threshold this should still catch it.
    sliver_img = perspective_warp(
        render_zone(zone, [(0.0, 0.0, 0.3, 0.015, 0.130)]), rng, strength=0.04)
    res = zv.analyze(sliver_img, zone, method=method)
    too_long = [d for d in (res.blocks if res.success else [])
               if d.length > zv.MAX_BLOCK_LENGTH_M]
    failures.check(not too_long, label,
                   "a %.1fx%.1fmm sliver survived filtering as a block"
                   % (too_long[0].width * 1000, too_long[0].length * 1000)
                   if too_long else "")

    # The largest block actually in this project (Stage 3's 1x3in cuboid) must
    # NOT be caught by the same net that catches the sliver above.
    # 1.2 in (30.5 mm) -- the largest block in the design as of 2026-07-31.
    # This was a 1in x 3in cuboid; that block was dropped along with the 6in
    # zone it needed, and MAX_BLOCK_LENGTH_M came down from 0.090 to 0.060 with
    # it. Asserting the 3in block still passes would now be asserting the old
    # design, so the fixture moves with it.
    rng2 = np.random.default_rng(29)
    largest_img = perspective_warp(
        render_zone(zone, [(0.0, 0.0, 0.2, 0.0305, 0.0305)]), rng2, strength=0.04)
    res2 = zv.analyze(largest_img, zone, method=method)
    label2 = "largest real block accepted"
    if failures.check(res2.success and res2.blocks, label2, res2.message):
        d = res2.blocks[0]
        failures.check(abs(d.length - 0.0305) < 0.004, label2,
                       "measured length %.1fmm, expected ~30.5mm"
                       % (d.length * 1000))


def test_three_tag_fallback(method, failures):
    """One tag occluded. 12 corner correspondences is still an over-determined
    fit, so this needs no special-case geometry -- but it must not silently
    degrade either, so the accuracy gate is the same."""
    rng = np.random.default_rng(3)
    zone = zv.zone_for("pickup")
    size = 0.030
    img = perspective_warp(render_zone(zone, [(0.01, -0.008, 0.3, size, size)],
                                       skip_tags=(zone.tag_ids[2],)), rng)
    res = zv.analyze(img, zone, method=method)
    label = "three-tag fallback"

    if not failures.check(res.success and res.blocks, label, res.message):
        return
    failures.check(res.tags_seen == 3, label, "tags_seen %d" % res.tags_seen)
    d = res.blocks[0]
    pos = math.hypot(d.zx - 0.01, d.zy + 0.008)
    failures.check(pos <= MAX_POSITION_ERR_M, label,
                   "position off by %.2f mm on three tags" % (pos * 1000))


def test_two_tag_pair(method, failures):
    """TWO tags, an adjacent pair -- the case real hardware always produces.

    The gripper hides the far pair from every reachable hover, so this is not an
    edge case to tolerate, it is the normal path. 8 corners = 16 equations for a
    homography's 8 DOF, so the fit is over-determined and the residual still
    means something; what suffers is conditioning across the thin direction of
    the pair. The gate is therefore LOOSER than the four-tag gate, and saying so
    out loud is the point -- a two-tag fix is usable but is not four tags.
    """
    rng = np.random.default_rng(11)
    zone = zv.zone_for("pickup")
    size = 0.030
    label = "two-tag adjacent pair"
    # Occlude tags 2 and 3, leaving the 0-1 edge: the worst realistic geometry.
    img = perspective_warp(
        render_zone(zone, [(0.008, -0.006, 0.4, size, size)],
                    skip_tags=(zone.tag_ids[2], zone.tag_ids[3])), rng)
    res = zv.analyze(img, zone, method=method)

    if not failures.check(res.success and res.blocks, label, res.message):
        return
    failures.check(res.tags_seen == 2, label, "tags_seen %d" % res.tags_seen)
    d = res.blocks[0]
    pos = math.hypot(d.zx - 0.008, d.zy + 0.006)
    # 3x the four-tag gate. Justified, not arbitrary: the pair spans the zone one
    # way and only tag_size the other, so the fit extrapolates ~6x across the
    # thin direction and amplifies corner noise by about that factor.
    failures.check(pos <= MAX_POSITION_ERR_M * 3.0, label,
                   "position off by %.2f mm on two tags" % (pos * 1000))


def test_tag_spread_metric(method, failures):
    """The conditioning metric must ORDER the constellations correctly and admit
    every case real hardware produces.

    Written after the floor was first set to 0.12, which silently rejected every
    DIAGONAL pair -- worse conditioned (0.117) than an adjacent pair (0.164)
    because its 8 corners bunch along the diagonal. Which pair the gripper leaves
    visible depends on wrist angle, so that floor banned a case the acquisition
    plan actively produces.
    """
    zone = zv.zone_for("pickup")
    t = zone.tag_ids
    label = "tag spread metric"
    r = lambda ids: zv.tag_spread_ratio({i: None for i in ids}, zone)

    four, three = r(t), r([t[0], t[1], t[2]])
    adj, diag = r([t[0], t[1]]), r([t[0], t[2]])

    failures.check(four > three > adj, label,
                   "ordering wrong: four=%.3f three=%.3f adjacent=%.3f"
                   % (four, three, adj))
    failures.check(diag < adj, label,
                   "diagonal pair (%.3f) should be THINNER than adjacent (%.3f)"
                   % (diag, adj))
    # Both two-tag cases must clear the floor: the gripper produces both.
    failures.check(min(adj, diag) >= zv.TAG_SPREAD_MIN_RATIO, label,
                   "floor %.3f rejects a real case (adjacent %.3f, diagonal %.3f)"
                   % (zv.TAG_SPREAD_MIN_RATIO, adj, diag))
    failures.check(zv.TAG_SPREAD_MIN_RATIO > 0.0, label, "floor disabled")


def test_multi_view_fusion(method, failures):
    """Four stills, each seeing a DIFFERENT pair -- the real acquisition plan.

    Simulates rotating the wrist between stills so the occluded pair changes.
    Fusion must beat the single worst view, and the reported spread must be an
    honest bound on the error rather than decoration.
    """
    rng = np.random.default_rng(17)
    zone = zv.zone_for("pickup")
    size = 0.030
    truth = (0.010, -0.007)
    label = "multi-view fusion"

    pairs = [(2, 3), (0, 3), (0, 1), (1, 2)]     # which two are hidden
    images = [
        perspective_warp(
            render_zone(zone, [(truth[0], truth[1], 0.35, size, size)],
                        skip_tags=(zone.tag_ids[a], zone.tag_ids[b])),
            np.random.default_rng(100 + i))
        for i, (a, b) in enumerate(pairs)
    ]

    fused = zv.analyze_multi(images, zone, method=method)
    if not failures.check(fused.success and fused.blocks, label, fused.message):
        return
    failures.check(fused.good_views == 4, label,
                   "only %d/4 stills usable" % fused.good_views)
    failures.check(sorted(fused.tag_ids_union) == sorted(zone.tag_ids), label,
                   "union of tags seen was %s" % (fused.tag_ids_union,))

    f = fused.blocks[0]
    err = math.hypot(f.zx - truth[0], f.zy - truth[1])
    failures.check(f.n_views >= 3, label, "fused from only %d views" % f.n_views)
    failures.check(err <= MAX_POSITION_ERR_M * 2.0, label,
                   "fused position off by %.2f mm" % (err * 1000))

    # The headline claim: fusing is better than trusting one still. Compare
    # against the WORST contributing view, since that is the one a single-still
    # pipeline could have picked.
    worst = max(math.hypot(d.zx - truth[0], d.zy - truth[1]) for d in f.views)
    failures.check(err <= worst + 1e-9, label,
                   "fusion (%.2f mm) worse than the worst single view (%.2f mm)"
                   % (err * 1000, worst * 1000))

    # And the spread has to actually bound the error, or it is worse than not
    # reporting one -- a number that reads as confidence while meaning nothing.
    failures.check(f.spread_m >= err * 0.5, label,
                   "spread %.2f mm implausibly small next to %.2f mm of error"
                   % (f.spread_m * 1000, err * 1000))
    print("      fusion: %.2f mm err, worst single view %.2f mm, spread %.2f mm"
          % (err * 1000, worst * 1000, f.spread_m * 1000))


def test_yaw_wrap_fusion(method, failures):
    """Yaw fusion across the symmetry wrap.

    A square's 0 and 90 degrees are the same pose. Averaging them naively gives
    45 -- a pose the block is never in, and off by enough that the jaws miss the
    faces entirely. This is the one fusion bug that would be catastrophic rather
    than merely noisy, so it gets its own test with no rendering involved.
    """
    label = "yaw wrap fusion"
    period = math.pi / 2.0

    class D:
        def __init__(self, yaw):
            self.zx = self.zy = 0.0
            self.zyaw = yaw
            self.width = self.length = 0.030
            self.shape = "square"
            self.symmetry = 4

    # Readings straddling the wrap: just under 90 deg and just over 0 deg.
    near = [math.radians(89.0), math.radians(1.0), math.radians(0.5)]
    fused = zv.fuse_detections([[D(y)] for y in near])
    failures.check(len(fused) == 1, label, "expected one cluster")
    got = fused[0].zyaw % period
    dev = min(got, period - got)
    failures.check(dev <= math.radians(3.0), label,
                   "wrapped yaws averaged to %.1f deg, expected ~0"
                   % math.degrees(got))


def test_camera_position(method, failures):
    """Shift the mat under a fixed camera and check camera_in_zone tracks it.

    This is the measurement the whole correction loop rests on -- it is the only
    thing in the pipeline that observes the ARM rather than the mat. If it were
    silently wrong (a sign flip, say), every correction would drive the arm the
    wrong way and the loop would diverge while every individual detection still
    looked perfect.

    No perspective warp here, only translation, so the expected answer is exact
    arithmetic rather than something to eyeball.
    """
    zone = zv.zone_for("pickup")
    base = render_zone(zone, [(0.0, 0.0, 0.0, 0.030, 0.030)])

    # Translating the CONTENT right by tx px means the image centre now looks at
    # a point tx px to the LEFT of the zone centre, i.e. zone x = -tx/scale.
    # Image +Y is down and zone +Y is up, so a downward shift is zone +Y.
    for tx, ty in ((0, 0), (60, 0), (0, -40), (-50, 35)):
        matrix = np.float32([[1, 0, tx], [0, 1, ty]])
        shifted = cv2.warpAffine(base, matrix, (CANVAS, CANVAS),
                                 borderValue=MAT_GREY)
        res = zv.analyze(shifted, zone, method=method)
        label = "camera position (%+d, %+d) px" % (tx, ty)
        if not failures.check(res.success, label, res.message):
            continue

        expect_zx = -tx / RENDER_SCALE
        expect_zy = ty / RENDER_SCALE
        err = math.hypot(res.camera_zx - expect_zx, res.camera_zy - expect_zy)
        failures.check(
            err <= MAX_POSITION_ERR_M, label,
            "camera reported at (%+.2f, %+.2f) mm, expected (%+.2f, %+.2f), "
            "off by %.2f mm" % (res.camera_zx * 1000, res.camera_zy * 1000,
                                expect_zx * 1000, expect_zy * 1000, err * 1000))

        # The block must NOT appear to move: its zone-local position comes from
        # the tags and is independent of where the camera is. This is the exact
        # asymmetry that makes re-detecting the block useless as a check on the
        # arm, and camera_in_zone necessary.
        if res.blocks:
            drift = math.hypot(res.blocks[0].zx, res.blocks[0].zy)
            failures.check(drift <= MAX_POSITION_ERR_M, label,
                           "block appeared to move %.2f mm when only the camera "
                           "moved" % (drift * 1000))


def test_empty_zone_is_success(method, failures):
    """An empty zone is a SUCCESS with zero blocks. Stage 2 asks exactly this
    before releasing, and Stage 4 asks it again -- if it came back as a failure
    those checks could never distinguish 'empty' from 'camera broken'."""
    rng = np.random.default_rng(5)
    zone = zv.zone_for("pickup")
    res = zv.analyze(perspective_warp(render_zone(zone), rng), zone, method=method)
    failures.check(res.success, "empty zone", res.message)
    failures.check(not res.blocks, "empty zone",
                   "found %d phantom block(s): %r" % (len(res.blocks), res.blocks))


def test_no_tags_fails_cleanly(method, failures):
    """A frame with no tags must fail with a message, never return a position.
    This is the difference between 'I cannot see' and a confident wrong answer."""
    zone = zv.zone_for("pickup")
    blank = np.full((CANVAS, CANVAS), MAT_GREY, np.uint8)
    res = zv.analyze(blank, zone, method=method)
    failures.check(not res.success, "no tags", "reported success on a blank frame")
    failures.check(bool(res.message), "no tags", "failed without a message")
    failures.check(not res.blocks, "no tags", "returned blocks with no tags")


def test_wrong_zone_ignored(method, failures):
    """Tags from the OTHER zone in frame must not be mistaken for this one.
    Both zones can easily be in shot at once."""
    rng = np.random.default_rng(9)
    place = zv.zone_for("place")
    img = perspective_warp(render_zone(place, [(0.0, 0.0, 0.0, 0.030, 0.030)]), rng)
    res = zv.analyze(img, zv.zone_for("pickup"), method=method)
    failures.check(not res.success, "wrong zone",
                   "matched the place zone's tags while asked for pickup")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--method", choices=["canny", "otsu", "both"], default="both",
                        help="segmentation method to exercise (default: both)")
    args = parser.parse_args()

    methods = ["canny", "otsu"] if args.method == "both" else [args.method]
    failures = Failures()

    zone = zv.zone_for("pickup")
    print("zone %.1f mm across, tags %.1f mm, usable half-extent for a 30 mm "
          "block: %.1f mm" % (zone.zone_size * 1000, zone.tag_size * 1000,
                              usable_half_extent(zone, 0.030) * 1000))

    for method in methods:
        print("\n--- method=%s ---" % method)
        test_square_sweep(method, failures)
        test_rectangle(method, failures)
        test_grid_line_rejected(method, failures)
        test_three_tag_fallback(method, failures)
        test_two_tag_pair(method, failures)
        test_tag_spread_metric(method, failures)
        test_multi_view_fusion(method, failures)
        test_yaw_wrap_fusion(method, failures)
        test_camera_position(method, failures)
        test_empty_zone_is_success(method, failures)
        test_no_tags_fails_cleanly(method, failures)
        test_wrong_zone_ignored(method, failures)

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
