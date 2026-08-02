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
DEFAULT_TAG_SIZE as ground truth. If the printer rescales the page, the
physical tags come out a different size than the code assumes, and EVERY
position and dimension it reports comes out scaled by that same wrong factor
-- silently, because the detector has no way to know your printer lied to it.
That is why every sheet carries a calibration ruler on each axis: measure both
with an actual ruler after printing, before trusting anything else.

PAGE SIZE: authored as US Letter (215.9 x 279.4 mm), since that is what most
campus printers feed by default even when asked for A4 -- a page-size
mismatch under a non-aspect-preserving "fit to page" is what produces
NON-SQUARE output (measured 3.25 x 3.8125 in from a 4x4 in target on one
printer here) even after a single uniform --print-correction was applied.

IF THE OUTPUT IS STILL OFF AFTER SWITCHING TO LETTER: the two axes can be
corrected independently. Print once with --print-correction-x 1.0
--print-correction-y 1.0 (the default), measure the two rulers this sheet
prints (one horizontal, one vertical, each nominally 100.0 mm), then rerun
with:

    --print-correction-x = 100.0 / measured_horizontal_mm
    --print-correction-y = 100.0 / measured_vertical_mm

PRINT SETTINGS: 100% / "Actual size" if your printer offers it. If it
doesn't (many campus print stations hide or remove that option), the
correction factors above are what you have instead.
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
RULER_LENGTH_MM = 100.0
RULER_CLEARANCE_MM = 14.0       # gap between the zone bbox and each ruler
LABEL_GAP_MM = 5.0              # gap between a tag's outer edge and its ID label

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


def render_sheet(zone_name, dpi, correction_x=1.0, correction_y=1.0):
    tag_ids = ZONES[zone_name]
    zone = zv.zone_for(zone_name)
    zone_size_mm = zone.zone_size * 1000.0
    tag_size_mm = zone.tag_size * 1000.0
    bbox_mm = zone_size_mm + tag_size_mm   # tags overhang the zone square by tag/2 a side

    needed_h = bbox_mm + 2 * (RULER_CLEARANCE_MM + 20.0)   # rulers top & bottom
    needed_w = bbox_mm + 2 * (RULER_CLEARANCE_MM + 20.0)   # rulers left & right
    if needed_w > PAGE_MM[0] or needed_h > PAGE_MM[1]:
        raise ValueError(
            "a %.1f mm zone with %.1f mm tags does not fit on a %.0f x %.0f mm "
            "page with room for both rulers. Shrink zone_size or tag_size."
            % (zone_size_mm, tag_size_mm, *PAGE_MM))

    px_per_mm_x = dpi / 25.4 * correction_x
    px_per_mm_y = dpi / 25.4 * correction_y
    page_px = (round(PAGE_MM[0] * px_per_mm_x), round(PAGE_MM[1] * px_per_mm_y))
    canvas = np.full((page_px[1], page_px[0]), 255, dtype=np.uint8)

    def mm_to_px(x_mm, y_mm):
        """Page-local mm (origin top-left, +Y down) -> pixel coords."""
        return int(round(x_mm * px_per_mm_x)), int(round(y_mm * px_per_mm_y))

    font = cv2.FONT_HERSHEY_SIMPLEX

    # Zone square centred on the page in BOTH axes -- nothing else on the page
    # competes with that centring, so the tag-centre square's centre IS the
    # page centre.
    centre_x_mm = PAGE_MM[0] / 2.0
    centre_y_mm = PAGE_MM[1] / 2.0

    dictionary = _dictionary()
    half_zone = zone_size_mm / 2.0
    tag_positions_mm = []
    for tag_id, (sx, sy) in zip(tag_ids, zv.ZONE_CORNER_SIGNS):
        # zone +Y is up; page mm is down, matching the y-flip used everywhere
        # else in this feature (zone_vision_selftest.render_zone does the same).
        tag_cx = centre_x_mm + sx * half_zone
        tag_cy = centre_y_mm - sy * half_zone
        tag_positions_mm.append((tag_id, sx, sy, tag_cx, tag_cy))

        side_px_x = int(round(tag_size_mm * px_per_mm_x))
        side_px_y = int(round(tag_size_mm * px_per_mm_y))
        bitmap = _marker_bitmap(dictionary, tag_id, max(side_px_x, side_px_y))
        if (side_px_x, side_px_y) != bitmap.shape[::-1]:
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

    # Two calibration rulers, one per axis -- see module docstring. Not
    # decorative: this is the only way to detect and correct anisotropic
    # print scaling. Kept to a bare line + end ticks + two numbers, no prose,
    # so nothing crowds the page edges.
    def _ruler(p_start_mm, p_end_mm, tick_dir):
        p0 = mm_to_px(*p_start_mm)
        p1 = mm_to_px(*p_end_mm)
        cv2.line(canvas, p0, p1, 0, 2)
        for p_mm, p_px, label in ((p_start_mm, p0, "0"), (p_end_mm, p1, "100")):
            tx, ty = p_px
            dx, dy = tick_dir
            cv2.line(canvas, (tx - dx * 6, ty - dy * 6), (tx + dx * 6, ty + dy * 6), 0, 2)
            lx = tx + (10 if dx else -14)
            ly = ty + (14 if dy else 4)
            cv2.putText(canvas, label, (lx, ly), font, 0.35, 0, 1, cv2.LINE_AA)

    ruler_x0_mm = centre_x_mm - RULER_LENGTH_MM / 2.0
    ruler_top_y_mm = centre_y_mm - bbox_mm / 2.0 - RULER_CLEARANCE_MM
    _ruler((ruler_x0_mm, ruler_top_y_mm), (ruler_x0_mm + RULER_LENGTH_MM, ruler_top_y_mm),
           tick_dir=(0, 1))

    ruler_y0_mm = centre_y_mm - RULER_LENGTH_MM / 2.0
    ruler_right_x_mm = centre_x_mm + bbox_mm / 2.0 + RULER_CLEARANCE_MM
    _ruler((ruler_right_x_mm, ruler_y0_mm), (ruler_right_x_mm, ruler_y0_mm + RULER_LENGTH_MM),
           tick_dir=(1, 0))

    return canvas, dpi


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zone", choices=["pickup", "place", "both"], default="both")
    parser.add_argument("--dpi", type=int, default=300,
                        help="print resolution embedded in the PNG (default: %(default)s)")
    parser.add_argument("--out-dir", default=".",
                        help="where to write the PNG(s) (default: current directory)")
    parser.add_argument("--print-correction-x", type=float, default=1.0,
                        help="horizontal pre-scale = 100 / measured_horizontal_ruler_mm "
                             "from a --print-correction-x 1.0 print (default: 1.0)")
    parser.add_argument("--print-correction-y", type=float, default=1.0,
                        help="vertical pre-scale = 100 / measured_vertical_ruler_mm "
                             "from a --print-correction-y 1.0 print (default: 1.0)")
    args = parser.parse_args()

    zones = ["pickup", "place"] if args.zone == "both" else [args.zone]
    os.makedirs(args.out_dir, exist_ok=True)

    for zone_name in zones:
        canvas, dpi = render_sheet(zone_name, args.dpi,
                                    args.print_correction_x, args.print_correction_y)
        path = os.path.join(args.out_dir, "%s_zone_A4.png" % zone_name)
        # dpi metadata so a print dialog's "Actual Size" reproduces the real
        # mm scale -- still verify with the rulers, since not every viewer or
        # driver respects it.
        Image.fromarray(canvas).save(path, dpi=(dpi, dpi))
        print("wrote %s  (%s, AprilTag 36h11 IDs %s, %d dpi, correction x%.4f y%.4f)"
              % (path, zone_name, list(ZONES[zone_name]), dpi,
                 args.print_correction_x, args.print_correction_y))

    print("\nPrint on Letter paper. If your printer offers 100%% / Actual Size, use "
          "it. Either way, measure BOTH rulers on the sheet (top = horizontal, "
          "right side = vertical; each should read 100.0 mm) before trusting the "
          "layout. If either is off, rerun with:")
    print("  --print-correction-x = 100.0 / measured_horizontal_mm")
    print("  --print-correction-y = 100.0 / measured_vertical_mm")


if __name__ == "__main__":
    main()
