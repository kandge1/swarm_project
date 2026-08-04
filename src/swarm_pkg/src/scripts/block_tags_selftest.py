#!/usr/bin/env python3
"""Synthetic regression for the block-tag scheme. No robot, no ROS, ~1 s.

    python3 block_tags_selftest.py          # expect "0 failure(s)"

Companion to zone_vision_selftest.py, which covers the zone homography. This
one covers what was added for stacked blocks:

  * the id scheme round-trips, and does not collide with zone ids 0-7
  * a rendered sheet tag decodes back to the face it was printed for
  * duplicate ids survive find_block_tags() -- the dict form loses them, and
    that is the failure mode that turns two identical blocks into one
  * a side tag is never given a zone-local position, because a mat-plane
    homography does not apply to a face perpendicular to the mat
  * the px/module floor is where the constants claim it is

The last one is the reason this file earns its place: the decision to print
24 mm tags rather than 22 rests on a threshold that was quoted from a document
and never tested. Here it is tested.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import block_coordinates as bc  # noqa: E402
import zone_vision as zv  # noqa: E402

FAILURES = []


def check(condition, what):
    if condition:
        print("  ok   %s" % what)
    else:
        print("  FAIL %s" % what)
        FAILURES.append(what)


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def _paste(canvas, tag_id, top_left, side_px, dictionary,
           quiet_modules=bc.QUIET_ZONE_MODULES):
    """Tag plus its white quiet zone, onto a canvas of whatever colour."""
    quiet = max(2, int(round(side_px / 8.0 * quiet_modules)))
    x, y = top_left
    canvas[y - quiet:y + side_px + quiet, x - quiet:x + side_px + quiet] = 255
    canvas[y:y + side_px, x:x + side_px] = _marker(dictionary, tag_id, side_px)


def test_scheme():
    print("id scheme")
    ids = set()
    for block_class in bc.BLOCK_CLASSES:
        for face in bc.FACE_ORDER:
            tag_id = bc.tag_id_for(block_class, face)
            info = bc.describe(tag_id)
            check(info.block_class == block_class and info.face == face,
                  "id %d round-trips to %s %s" % (tag_id, block_class, face))
            ids.add(tag_id)
    check(len(ids) == 12, "12 distinct ids, got %d" % len(ids))
    check(min(ids) >= 8, "no collision with zone ids 0-7 (lowest is %d)" % min(ids))
    check(all(bc.describe(z) is None for z in range(8)),
          "zone ids 0-7 are not block tags")

    # Four distinct sides is the whole reason yaw comes for free; a shared id
    # would make these equal and the assertion below is what would catch it.
    side_ids = {bc.tag_id_for("cube", "side%d" % k) for k in range(4)}
    check(len(side_ids) == 4, "the cube's four sides have four distinct ids")

    # A block turned so its SIDE2 faces world 0 deg is turned 90 deg.
    check(abs(bc.block_yaw_from_side(2, 0.0) - 90.0) < 1e-9,
          "block_yaw_from_side(SIDE2, 0 deg) == 90 deg")
    check(abs(bc.block_yaw_from_side(0, 90.0) - 0.0) < 1e-9,
          "block_yaw_from_side(SIDE0, 90 deg) == 0 deg")


def test_face_sizing():
    print("\nface sizing")
    tag = bc.max_tag_size_for_face(0.030)
    check(abs(tag - 0.0225) < 1e-9,
          "a 30 mm face takes a 22.5 mm tag (got %.4f m)" % tag)
    quiet_mm = (0.030 - tag) / 2 * 1000
    modules = quiet_mm / (tag * 1000 / 8)
    check(modules >= bc.QUIET_ZONE_MODULES,
          "the leftover is %.2f modules of quiet zone, >= the %.2f required"
          % (modules, bc.QUIET_ZONE_MODULES))
    check(tag + 2 * (quiet_mm / 1000) <= 0.030 + 1e-12,
          "tag plus quiet zone still fits inside the face")
    # Rounding must never round UP, which would overflow the face.
    for face_mm in (25.0, 28.0, 30.0, 33.3, 40.0):
        got = bc.max_tag_size_for_face(face_mm / 1000.0)
        exact = (face_mm / 1000.0) / (1.0 + bc.QUIET_ZONE_MODULES / 4.0)
        check(got <= exact + 1e-12,
              "%.1f mm face: %.1f mm tag never exceeds the exact %.2f mm"
              % (face_mm, got * 1000, exact * 1000))


def test_quiet_zone_floor():
    """One module of quiet zone is NOT enough, and that is why the constant is
    1.25. The AprilTag spec says one; against anything but a white background
    OpenCV's detector disagrees, and it disagrees by failing completely rather
    than degrading. This is the test that turned that from folklore into a
    number, so it is the one to re-run if the constant is ever questioned.
    """
    print("\nquiet zone floor")
    dictionary = _dictionary()

    def decodes(tag_px, quiet_modules, background):
        quiet = max(1, int(round(tag_px / 8.0 * quiet_modules)))
        pad = quiet + 30
        canvas = np.full((tag_px + 2 * pad, tag_px + 2 * pad), background,
                         np.uint8)
        _paste(canvas, 8, (pad, pad), tag_px, dictionary, quiet_modules)
        return 8 in [t for t, _ in zv.detect_all_tags_list(canvas)]

    # The cliff: 1.0 module against a dark background, at a tag big enough for
    # the adaptive threshold window to sit entirely inside it.
    check(not decodes(64, 1.00, 40),
          "1.00 module on a dark background FAILS (the cliff being avoided)")
    check(decodes(64, 1.00, 255),
          "the same tag on WHITE decodes -- so it is the background, not the tag")
    for background in (40, 120, 180, 255):
        check(decodes(64, bc.QUIET_ZONE_MODULES, background),
              "%.2f modules decodes against background %d"
              % (bc.QUIET_ZONE_MODULES, background))


def test_detection_and_duplicates():
    print("\ndetection")
    dictionary = _dictionary()
    canvas = np.full((480, 640), 120, np.uint8)      # mid-grey "mat"
    _paste(canvas, bc.tag_id_for("cube", "top"), (60, 60), 64, dictionary)
    _paste(canvas, bc.tag_id_for("cube", "side1"), (260, 60), 64, dictionary)
    # The SAME id twice: two identically-tagged cubes in one frame.
    _paste(canvas, bc.tag_id_for("cube", "top"), (450, 260), 64, dictionary)

    sightings = zv.find_block_tags(canvas)
    check(len(sightings) == 3,
          "3 block tags found including the duplicate id (got %d)" % len(sightings))
    check(len(zv.detect_all_tags(canvas)) == 2,
          "the dict form loses the duplicate, which is why the list form exists")

    labels = sorted(s.face.label for s in sightings)
    check(labels == ["cube SIDE1", "cube TOP", "cube TOP"],
          "decoded faces are %s" % labels)
    check(all(abs(s.px - 64) < 2.0 for s in sightings),
          "measured pixel size is ~64 px (got %s)"
          % [round(s.px, 1) for s in sightings])
    check(all(abs(s.px_per_module - 8.0) < 0.3 for s in sightings),
          "px/module is ~8 for a 64 px tag")

    # A zone tag in the same frame must be ignored by find_block_tags.
    _paste(canvas, 0, (60, 300), 64, dictionary)
    check(len(zv.find_block_tags(canvas)) == 3,
          "a zone tag in shot is not reported as a block tag")


def test_side_tags_get_no_position():
    print("\nmat-plane projection")
    dictionary = _dictionary()
    canvas = np.full((480, 640), 120, np.uint8)
    _paste(canvas, bc.tag_id_for("cuboid", "top"), (100, 100), 64, dictionary)
    _paste(canvas, bc.tag_id_for("cuboid", "side3"), (320, 100), 64, dictionary)

    # Any invertible homography will do; the point is which tags get used.
    H_px_to_zone = np.array([[1e-4, 0, -0.03],
                             [0, 1e-4, -0.02],
                             [0, 0, 1.0]])
    by_face = {s.face.face: s for s in zv.find_block_tags(canvas, H_px_to_zone)}
    check(by_face["top"].zone_xy is not None,
          "a TOP tag gets a zone-local position")
    check(by_face["side3"].zone_xy is None,
          "a SIDE tag does NOT -- it is perpendicular to the mat plane")

    # Without a homography nothing gets a position, not even a top tag.
    none_h = {s.face.face: s for s in zv.find_block_tags(canvas)}
    check(none_h["top"].zone_xy is None,
          "with no homography, even a TOP tag has no position")


def test_px_per_module_floor():
    """The threshold the printed tag size was chosen against.

    Renders one tag foreshortened like a block side face at the 36 deg survey
    pose, downsamples to a target size, blurs and adds noise, and counts
    decodes. Confirms that the 3.0 px/module constant in block_detector_node is
    a real floor and not folklore.
    """
    print("\npx/module floor (foreshortened side face, mild blur + noise)")
    dictionary = _dictionary()
    rng = np.random.default_rng(0)

    def decode_rate(tag_px, trials=20):
        hi, ok = 8 * 40, 0
        base = np.full((hi * 3 // 2, hi * 3 // 2), 255, np.uint8)
        off = hi // 4
        base[off:off + hi, off:off + hi] = _marker(dictionary, 10, hi)
        squashed = cv2.resize(base, (int(base.shape[1] * 0.59), base.shape[0]),
                              interpolation=cv2.INTER_AREA)
        scale = tag_px / float(hi)
        small = cv2.resize(squashed,
                           (max(4, int(squashed.shape[1] * scale)),
                            max(4, int(squashed.shape[0] * scale))),
                           interpolation=cv2.INTER_AREA).astype(np.float32)
        small = 128 + (small - 128) * 0.85            # real paper is not pure white
        small = cv2.GaussianBlur(small, (0, 0), 0.8)  # mild defocus
        for _ in range(trials):
            noisy = np.clip(small + rng.normal(0, 6.0, small.shape), 0, 255)
            frame = np.full((noisy.shape[0] + 40, noisy.shape[1] + 40), 200,
                            np.uint8)
            frame[20:-20, 20:-20] = noisy.astype(np.uint8)
            if 10 in [t for t, _ in zv.detect_all_tags_list(frame)]:
                ok += 1
        return 100.0 * ok / trials

    below = decode_rate(int(round(2.5 * 8)))
    at_floor = decode_rate(int(round(3.0 * 8)))
    at_good = decode_rate(int(round(4.0 * 8)))
    print("     2.5 px/module -> %3.0f%% decoded" % below)
    print("     3.0 px/module -> %3.0f%% decoded  (TAG_PX_PER_MODULE_MIN)" % at_floor)
    print("     4.0 px/module -> %3.0f%% decoded  (TAG_PX_PER_MODULE_GOOD)" % at_good)
    check(below < 20.0,
          "below the floor, decoding essentially fails (%.0f%%)" % below)
    # 85 rather than 90 because this is 20 stochastic trials at a fixed seed and
    # the measured value sits at 90 -- a threshold set flush against the
    # observation would fail on any harmless change to the noise model.
    check(at_good >= 85.0,
          "at the GOOD threshold, decoding is reliable (%.0f%%)" % at_good)
    check(at_good > at_floor,
          "GOOD (%.0f%%) beats MIN (%.0f%%), so the two constants are ordered "
          "the way they are used" % (at_good, at_floor))


def main():
    test_scheme()
    test_face_sizing()
    test_quiet_zone_floor()
    test_detection_and_duplicates()
    test_side_tags_get_no_position()
    test_px_per_module_floor()
    print("\n%d failure(s)" % len(FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
