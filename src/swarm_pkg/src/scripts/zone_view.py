#!/usr/bin/env python3
"""Look at what zone_vision.py sees in a still. Tuning tool, not a robot tool.

Runs the SAME zone_vision.analyze() the Pi runs in production, and draws what it
found: the tags it matched, the fitted zone square, the region it searched, and
every block with its measured size and yaw.

    # on mars, against the corpus
    python3 zone_view.py frames/*.png --show
    python3 zone_view.py frames/corner_45.png --method otsu --show

    # on the Pi, headless
    python3 zone_view.py /tmp/still.png --write /tmp/still_annotated.png

    # sweep a threshold across the whole corpus and print the table
    python3 zone_view.py frames/*.png --summary

This is NOT a compute offload. The Pi does all the real work at runtime,
homography included -- cv2.findHomography on 16 points is microseconds. This
script exists so a threshold can be tried against 20 saved frames in seconds
instead of a robot session per attempt. See APRIL_TAGS.md.
"""
import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402

COLOR_TAG = (0, 200, 255)       # amber
COLOR_ZONE = (0, 255, 0)        # green
COLOR_BLOCK = (255, 80, 80)     # blue
COLOR_BAD = (0, 0, 255)         # red
COLOR_TEXT = (255, 255, 255)
COLOR_BLOCK_TAG = (255, 0, 255)  # magenta -- distinct from the amber zone tags


def annotate(image, result, zone, block_tags=None):
    """BGR overlay of everything analyze() decided.

    block_tags is the optional output of BlockDetector._find_block_tags: block
    face tags are not part of analyze()'s job, so they are drawn only when a
    caller has already found them. Each is labelled with its px/module, which is
    the number that says whether a decode can be trusted at this distance.
    """
    canvas = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Dim everything outside the searched region, so a mask that is in the wrong
    # place is obvious at a glance rather than something to be inferred.
    if result.mask is not None:
        outside = cv2.bitwise_not(result.mask)
        canvas[outside > 0] = (canvas[outside > 0] * 0.45).astype(np.uint8)

    for tag_id, corners in sorted(result.tag_corners.items()):
        pts = corners.astype(np.int32)
        cv2.polylines(canvas, [pts], True, COLOR_TAG, 2)
        cv2.putText(canvas, str(tag_id), tuple(pts[0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TAG, 1, cv2.LINE_AA)

    if result.H_zone_to_px is not None:
        quad = zv.zone_quad_px(result.H_zone_to_px, zone).astype(np.int32)
        cv2.polylines(canvas, [quad], True, COLOR_ZONE, 2)
        # Zone axes: the single fastest way to spot a mirrored or 90-deg-rotated
        # corner assignment, which a good homography residual will NOT catch.
        origin = zv.zone_to_px(result.H_zone_to_px, [(0.0, 0.0)])[0]
        for vec, label in (((0.02, 0.0), "+X"), ((0.0, 0.02), "+Y")):
            tip = zv.zone_to_px(result.H_zone_to_px, [vec])[0]
            cv2.arrowedLine(canvas, tuple(origin.astype(int)),
                            tuple(tip.astype(int)), COLOR_ZONE, 2, tipLength=0.3)
            cv2.putText(canvas, label, tuple(tip.astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_ZONE, 1, cv2.LINE_AA)

    for index, block in enumerate(result.blocks):
        box = block.box_px.astype(np.int32)
        cv2.drawContours(canvas, [box], 0, COLOR_BLOCK, 2)
        centre = box.mean(axis=0).astype(int)
        cv2.drawMarker(canvas, tuple(centre), COLOR_BLOCK, cv2.MARKER_CROSS, 12, 2)
        cv2.putText(canvas,
                    "%d: %.0fx%.0fmm %s %.0fdeg" % (
                        index, block.width * 1000, block.length * 1000,
                        block.shape, math.degrees(block.zyaw)),
                    (centre[0] + 10, centre[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_BLOCK, 1, cv2.LINE_AA)

    for face, corners, px, per_module, _zone_xy in (block_tags or []):
        pts = corners.astype(np.int32)
        cv2.polylines(canvas, [pts], True, COLOR_BLOCK_TAG, 2)
        top = pts[pts[:, 1].argmin()]
        cv2.putText(canvas, "%s %.1fpx/mod" % (face.label, per_module),
                    (top[0], max(12, top[1] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, COLOR_BLOCK_TAG, 1,
                    cv2.LINE_AA)

    header = "%s | tags %d %s | rms %.2f px | %.0f px/m" % (
        "OK" if result.success else "FAIL", result.tags_seen,
        result.tag_ids, result.homography_rms, result.scale_px_per_m)
    if block_tags:
        header += " | %d block tag(s)" % len(block_tags)
    cv2.putText(canvas, header, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                COLOR_TEXT if result.success else COLOR_BAD, 1, cv2.LINE_AA)
    if not result.success:
        # Wrap the message rather than letting it run off the frame -- the
        # message is the whole point of a failing frame.
        words, line, y = result.message.split(), "", 40
        for word in words:
            if len(line) + len(word) > 60:
                cv2.putText(canvas, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.42, COLOR_BAD, 1, cv2.LINE_AA)
                line, y = "", y + 16
            line += word + " "
        if line:
            cv2.putText(canvas, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, COLOR_BAD, 1, cv2.LINE_AA)
    return canvas


def describe(path, result, zone, verbose):
    name = os.path.basename(path)
    print("%-28s %-4s tags=%d rms=%5.2fpx blocks=%d  %s"
          % (name, "OK" if result.success else "FAIL", result.tags_seen,
             result.homography_rms, len(result.blocks),
             "" if result.success else result.message))
    if not verbose:
        return
    for index, block in enumerate(result.blocks):
        wx, wy, wyaw = block.world_pose(zone)
        print("    [%d] zone (%+7.2f, %+7.2f) mm  yaw %+7.2f deg  "
              "%5.1f x %5.1f mm  %-7s sym=%d fill=%.2f"
              % (index, block.zx * 1000, block.zy * 1000,
                 math.degrees(block.zyaw), block.width * 1000,
                 block.length * 1000, block.shape, block.symmetry,
                 block.fill_ratio))
        print("        world (%+7.4f, %+7.4f) m  yaw %+7.2f deg"
              % (wx, wy, math.degrees(wyaw)))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+",
                        help="still image files (globs are expanded)")
    parser.add_argument("--zone", default="pickup", choices=["pickup", "place"])
    parser.add_argument("--method", default="canny", choices=["canny", "otsu"],
                        help="segmentation method (default: canny)")
    parser.add_argument("--zone-size", type=float, default=zv.DEFAULT_ZONE_SIZE,
                        help="side of the square joining the TAG CENTRES, metres "
                             "(default: %(default)s = 6 in, around a ~4 in "
                             "working area -- see APRIL_TAGS.md 'Usable area')")
    parser.add_argument("--tag-size", type=float, default=zv.DEFAULT_TAG_SIZE,
                        help="printed tag side, metres (default: %(default)s = 1 in)")
    parser.add_argument("--max-rms", type=float, default=zv.MAX_HOMOGRAPHY_RMS_PX,
                        help="reject a homography above this px residual "
                             "(default: %(default)s)")
    parser.add_argument("--world", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                        default=[0.0, 0.0, 0.0],
                        help="surveyed zone centre in world coords, for the world "
                             "pose printout. Vision does not measure this.")
    parser.add_argument("--show", action="store_true", help="open a window per image")
    parser.add_argument("--write", metavar="PATH",
                        help="write the overlay here (a directory if multiple images)")
    parser.add_argument("--summary", action="store_true",
                        help="one line per image, no per-block detail")
    args = parser.parse_args()

    paths = []
    for pattern in args.images:
        expanded = sorted(glob.glob(pattern))
        paths.extend(expanded if expanded else [pattern])

    zone = zv.zone_for(args.zone, world_x=args.world[0], world_y=args.world[1],
                       world_yaw=args.world[2], zone_size=args.zone_size,
                       tag_size=args.tag_size)

    failures = 0
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            print("%-28s FAIL cannot read" % os.path.basename(path))
            failures += 1
            continue

        result = zv.analyze(image, zone, method=args.method, max_rms_px=args.max_rms)
        failures += 0 if result.success else 1
        describe(path, result, zone, verbose=not args.summary)

        if not (args.show or args.write):
            continue
        canvas = annotate(image, result, zone)
        if args.write:
            if len(paths) > 1 or os.path.isdir(args.write):
                os.makedirs(args.write, exist_ok=True)
                out = os.path.join(args.write, os.path.basename(path))
            else:
                out = args.write
            cv2.imwrite(out, canvas)
            print("    wrote %s" % out)
        if args.show:
            cv2.imshow(os.path.basename(path), canvas)
            print("    any key = next, q = quit")
            if cv2.waitKey(0) in (ord("q"), 27):
                break
            cv2.destroyAllWindows()

    if args.show:
        cv2.destroyAllWindows()
    print("\n%d/%d frame(s) failed" % (failures, len(paths)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
