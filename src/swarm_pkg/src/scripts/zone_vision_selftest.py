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
        test_three_tag_fallback(method, failures)
        test_camera_position(method, failures)
        test_empty_zone_is_success(method, failures)
        test_no_tags_fails_cleanly(method, failures)
        test_wrong_zone_ignored(method, failures)

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
