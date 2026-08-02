#!/usr/bin/env python3
"""Generate a printable sheet for a pickup or place zone.

Draws the ACTUAL AprilTag (36h11) bitmaps at the ACTUAL physical size and
spacing zone_vision.py expects -- imports DEFAULT_ZONE_SIZE, DEFAULT_TAG_SIZE,
PICKUP_TAG_IDS/PLACE_TAG_IDS and ZONE_CORNER_SIGNS directly rather than
re-stating them, so a printed mat can never quietly drift out of sync with
what the detector is looking for.

    python3 print_zone_tags.py                     # both zones, 300 dpi
    python3 print_zone_tags.py --zone pickup
    python3 print_zone_tags.py --out-dir ~/Desktop --dpi 600

WHY THE PRINT SCALE MATTERS, NOT JUST THE PATTERN:
AprilTag detection decodes the bit pattern regardless of size -- a tag printed
too small or too big is still read correctly. But zone_vision's homography
fits image pixels to WORLD MILLIMETRES using DEFAULT_ZONE_SIZE and
DEFAULT_TAG_SIZE as ground truth. If the printer rescales the page, EVERY
position and dimension the detector reports comes out scaled by that same
wrong factor -- silently, because it has no way to know your printer lied to
it. That is why this sheet carries a calibration ruler on each axis.

HOW THESE PRINTERS ACTUALLY BEHAVE, AND WHY THE OBVIOUS FIX DOES NOT WORK
-------------------------------------------------------------------------
The campus printers here apply FIT TO PAGE unconditionally: they rescale
whatever image you hand them to fill the paper's printable area, ignoring
both the pixel dimensions and the DPI metadata in the file. There is no
"Actual Size" option to turn it off.

The tempting fix -- render the page bigger so the printer's shrink cancels
out -- DOES NOT WORK, and the failure is silent. If you scale the canvas and
the drawing together (the natural thing to do, by scaling px_per_mm), the
ratio of content to canvas is unchanged, so fit-to-page produces a
BYTE-IDENTICAL physical result. Measured here: pre-scaling by 1.0926x moved
the printed tag from 23 mm to 23 mm and the zone square from 94 mm to 93 mm,
i.e. not at all.

What fit-to-page actually preserves is the RATIO of the drawn content to the
canvas. So the canvas must stay FIXED at the paper size while only the
CONTENT is scaled, about the page centre. That is what CONTENT_SCALE below
does, and it is the only knob that has any effect on this class of printer.

CALIBRATING IT
--------------
Print once, measure the two rulers on the sheet (top = horizontal, right side
= vertical). Each is nominally RULER_LENGTH_MM and is labelled in cm. Then:

    new_scale_x = current_scale_x * (RULER_LENGTH_MM / measured_horizontal_mm)
    new_scale_y = current_scale_y * (RULER_LENGTH_MM / measured_vertical_mm)

and rerun with --content-scale-x / --content-scale-y. The factor is a
property of the printer + paper, so once found it stays put.

PAGE SIZE: authored as US Letter (215.9 x 279.4 mm), since that is what these
printers feed. A page-size mismatch under a non-aspect-preserving fit-to-page
is what produced NON-SQUARE output earlier (3.25 x 3.8125 in from a 4x4 in
target while the canvas was A4); on a Letter canvas the output measured
square, confirming the aspect now matches.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402

PAGE_MM = (215.9, 279.4)        # US Letter, portrait, (width, height)
RULER_LENGTH_MM = 150.0         # long baseline: measurement error divides out
RULER_CLEARANCE_MM = 14.0       # gap between the zone bbox and each ruler
LABEL_GAP_MM = 5.0              # gap between a tag's outer edge and its ID label

# Content-to-canvas scale that cancels this printer's fit-to-page shrink.
# Derived from a measured print at scale 1.0: the 101.6 mm zone square came
# out 93 mm, so 101.6 / 93 = 1.0925. (The 25.4 mm tag measured 23 mm on the
# same sheet, giving 1.1043 -- consistent within the ~1 mm reading error of a
# 23 mm feature, which is exactly why the rulers below are 150 mm long.)
CONTENT_SCALE = 101.6 / 93.0

ZONES = {
    "pickup": zv.PICKUP_TAG_IDS,
    "place": zv.PLACE_TAG_IDS,
}


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker_bitmap(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def render_sheet(zone_name, dpi, scale_x=1.0, scale_y=1.0):
    tag_ids = ZONES[zone_name]
    zone = zv.zone_for(zone_name)
    zone_size_mm = zone.zone_size * 1000.0
    tag_size_mm = zone.tag_size * 1000.0
    bbox_mm = zone_size_mm + tag_size_mm   # tags overhang the zone square by tag/2 a side

    # CANVAS IS FIXED at the paper size -- see module docstring. Scaling this
    # alongside the content is precisely the no-op that wasted several print
    # runs.
    px_per_mm = dpi / 25.4
    page_px = (round(PAGE_MM[0] * px_per_mm), round(PAGE_MM[1] * px_per_mm))
    canvas = np.full((page_px[1], page_px[0]), 255, dtype=np.uint8)

    centre_x_mm = PAGE_MM[0] / 2.0
    centre_y_mm = PAGE_MM[1] / 2.0

    def mm_to_px(x_mm, y_mm):
        """Page-local mm -> pixels, with content scaled about the page centre.

        Only the OFFSET from the centre is scaled, so the zone square's centre
        stays pinned to the middle of the sheet at every scale factor.
        """
        sx = centre_x_mm + (x_mm - centre_x_mm) * scale_x
        sy = centre_y_mm + (y_mm - centre_y_mm) * scale_y
        return int(round(sx * px_per_mm)), int(round(sy * px_per_mm))

    # Everything that must fit, measured from the page centre outward.
    half_extent_x = (bbox_mm / 2.0 + RULER_CLEARANCE_MM + 6.0) * scale_x
    half_extent_y = (bbox_mm / 2.0 + RULER_CLEARANCE_MM + 6.0) * scale_y
    half_ruler_x = RULER_LENGTH_MM / 2.0 * scale_x
    half_ruler_y = RULER_LENGTH_MM / 2.0 * scale_y
    if max(half_extent_x, half_ruler_x) > centre_x_mm or \
            max(half_extent_y, half_ruler_y) > centre_y_mm:
        raise ValueError(
            "content at scale (%.4f, %.4f) overflows the %.0f x %.0f mm page. "
            "Reduce the scale, the zone size, or RULER_LENGTH_MM."
            % (scale_x, scale_y, *PAGE_MM))

    font = cv2.FONT_HERSHEY_SIMPLEX
    dictionary = _dictionary()
    half_zone = zone_size_mm / 2.0
    tag_positions_mm = []
    for tag_id, (sx, sy) in zip(tag_ids, zv.ZONE_CORNER_SIGNS):
        # zone +Y is up; page mm is down, matching the y-flip used everywhere
        # else in this feature (zone_vision_selftest.render_zone does the same).
        tag_cx = centre_x_mm + sx * half_zone
        tag_cy = centre_y_mm - sy * half_zone
        tag_positions_mm.append((tag_id, sx, sy, tag_cx, tag_cy))

        # The tag bitmap is scaled the same way its position is, so a tag stays
        # square only if scale_x == scale_y. When they differ the printer is
        # about to stretch it back to square, which is the whole point.
        side_px_x = max(1, int(round(tag_size_mm * scale_x * px_per_mm)))
        side_px_y = max(1, int(round(tag_size_mm * scale_y * px_per_mm)))
        bitmap = _marker_bitmap(dictionary, tag_id, max(side_px_x, side_px_y))
        if bitmap.shape[:2] != (side_px_y, side_px_x):
            bitmap = cv2.resize(bitmap, (side_px_x, side_px_y),
                                interpolation=cv2.INTER_NEAREST)
        cx_px, cy_px = mm_to_px(tag_cx, tag_cy)
        x0, y0 = cx_px - side_px_x // 2, cy_px - side_px_y // 2
        canvas[y0:y0 + side_px_y, x0:x0 + side_px_x] = bitmap

        # ID label OUTSIDE the tag, away from the zone centre, clear of the
        # quiet zone every AprilTag needs around its border to detect reliably.
        label_x = tag_cx + sx * (tag_size_mm / 2.0 + LABEL_GAP_MM)
        label_y = tag_cy - sy * (tag_size_mm / 2.0 + LABEL_GAP_MM)
        lx_px, ly_px = mm_to_px(label_x, label_y)
        text = "ID %d" % tag_id
        (tw, th), _ = cv2.getTextSize(text, font, 0.5, 2)
        cv2.putText(canvas, text, (lx_px - tw // 2, ly_px + (th if sy > 0 else -2)),
                    font, 0.5, 0, 2, cv2.LINE_AA)

    # Dashed line joining the tag centres -- the square the homography is
    # actually fitted to. Not required for detection; purely so a human can
    # see at a glance whether the print came out looking like what this
    # script intended.
    centres_px = [mm_to_px(cx, cy) for _, _, _, cx, cy in tag_positions_mm]
    for i in range(4):
        p0, p1 = centres_px[i], centres_px[(i + 1) % 4]
        length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        steps = max(1, int(length // 10))
        for s in range(0, steps, 2):
            a = (p0[0] + (p1[0] - p0[0]) * s / steps, p0[1] + (p1[1] - p0[1]) * s / steps)
            b = (p0[0] + (p1[0] - p0[0]) * (s + 1) / steps,
                 p0[1] + (p1[1] - p0[1]) * (s + 1) / steps)
            cv2.line(canvas, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), 160, 1)

    # Two calibration rulers, one per axis. These are the measuring
    # instruments the scale factor is derived from, so they are long (150 mm)
    # and ticked every 10 mm with cm numerals -- a 150 mm baseline read to the
    # nearest millimetre pins the scale to ~0.7%, where a 23 mm tag read the
    # same way is off by 4%.
    def _ruler(x0_mm, y0_mm, length_mm, vertical):
        if vertical:
            p0, p1 = (x0_mm, y0_mm), (x0_mm, y0_mm + length_mm)
        else:
            p0, p1 = (x0_mm, y0_mm), (x0_mm + length_mm, y0_mm)
        cv2.line(canvas, mm_to_px(*p0), mm_to_px(*p1), 0, 2)
        for t in range(0, int(length_mm) + 1, 10):
            big = (t % 50 == 0)
            arm = 5.0 if big else 2.5
            if vertical:
                a, b = (x0_mm, y0_mm + t), (x0_mm + arm, y0_mm + t)
            else:
                a, b = (x0_mm + t, y0_mm), (x0_mm + t, y0_mm + arm)
            cv2.line(canvas, mm_to_px(*a), mm_to_px(*b), 0, 2 if big else 1)
            if big:
                tx, ty = mm_to_px(*b)
                off = (6, 4) if vertical else (-5, 14)
                cv2.putText(canvas, "%d" % (t // 10), (tx + off[0], ty + off[1]),
                            font, 0.35, 0, 1, cv2.LINE_AA)

    _ruler(centre_x_mm - RULER_LENGTH_MM / 2.0,
           centre_y_mm - bbox_mm / 2.0 - RULER_CLEARANCE_MM,
           RULER_LENGTH_MM, vertical=False)
    _ruler(centre_x_mm + bbox_mm / 2.0 + RULER_CLEARANCE_MM,
           centre_y_mm - RULER_LENGTH_MM / 2.0,
           RULER_LENGTH_MM, vertical=True)

    return canvas, dpi


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone", choices=["pickup", "place", "both"], default="both")
    parser.add_argument("--dpi", type=int, default=300,
                        help="canvas resolution (default: %(default)s). Does NOT "
                             "affect printed size on a fit-to-page printer.")
    parser.add_argument("--out-dir", default=".",
                        help="where to write the PNG(s) (default: current directory)")
    parser.add_argument("--content-scale-x", type=float, default=CONTENT_SCALE,
                        help="horizontal content-to-canvas scale (default: %.4f). "
                             "Recalibrate: current * (%.0f / measured_horizontal_mm)."
                             % (CONTENT_SCALE, RULER_LENGTH_MM))
    parser.add_argument("--content-scale-y", type=float, default=CONTENT_SCALE,
                        help="vertical content-to-canvas scale (default: %.4f). "
                             "Recalibrate: current * (%.0f / measured_vertical_mm)."
                             % (CONTENT_SCALE, RULER_LENGTH_MM))
    args = parser.parse_args()

    zones = ["pickup", "place"] if args.zone == "both" else [args.zone]
    os.makedirs(args.out_dir, exist_ok=True)

    for zone_name in zones:
        canvas, dpi = render_sheet(zone_name, args.dpi,
                                   args.content_scale_x, args.content_scale_y)
        path = os.path.join(args.out_dir, "%s_zone_A4.png" % zone_name)
        Image.fromarray(canvas).save(path, dpi=(dpi, dpi))
        print("wrote %s  (%s, IDs %s, %dx%d px, content scale x%.4f y%.4f)"
              % (path, zone_name, list(ZONES[zone_name]),
                 canvas.shape[1], canvas.shape[0],
                 args.content_scale_x, args.content_scale_y))

    print("\nPrint on LETTER paper, fit to page (that is what these printers do "
          "regardless). Then measure the two rulers -- top = horizontal, right "
          "= vertical, each nominally %.0f mm / %.0f cm."
          % (RULER_LENGTH_MM, RULER_LENGTH_MM / 10.0))
    print("If either is off, rerun with:")
    print("  --content-scale-x %.4f * (%.0f / measured_horizontal_mm)"
          % (args.content_scale_x, RULER_LENGTH_MM))
    print("  --content-scale-y %.4f * (%.0f / measured_vertical_mm)"
          % (args.content_scale_y, RULER_LENGTH_MM))


if __name__ == "__main__":
    main()
