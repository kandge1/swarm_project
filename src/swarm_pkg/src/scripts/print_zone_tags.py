#!/usr/bin/env python3
"""Generate a printable A4 sheet for a pickup or place zone.

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
DEFAULT_TAG_SIZE as ground truth. If the printer rescales the page ("fit to
page", "scale to fit"), the physical tags come out a different size than the
code assumes, and EVERY position and dimension it reports comes out scaled by
that same wrong factor -- silently, because the detector has no way to know
your printer lied to it. That is why every sheet carries a calibration ruler:
measure it with an actual ruler after printing, before trusting anything else.

PRINT SETTINGS: 100% / "Actual size". Do NOT use "Fit to page" or "Shrink to
fit" -- either one breaks the physical scale this whole feature depends on.
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

A4_MM = (210.0, 297.0)          # portrait, (width, height)
MARGIN_MM = 12.0
LABEL_GAP_MM = 6.0               # gap between a tag's outer edge and its ID label
RULER_LENGTH_MM = 100.0

ZONES = {
    "pickup": (zv.PICKUP_TAG_IDS, "PICKUP ZONE"),
    "place": (zv.PLACE_TAG_IDS, "PLACE ZONE"),
}


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker_bitmap(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def render_sheet(zone_name, dpi):
    tag_ids, title = ZONES[zone_name]
    zone = zv.zone_for(zone_name)
    zone_size_mm = zone.zone_size * 1000.0
    tag_size_mm = zone.tag_size * 1000.0
    bbox_mm = zone_size_mm + tag_size_mm   # tags overhang the zone square by tag/2 a side

    if bbox_mm + 2 * MARGIN_MM > min(A4_MM):
        raise ValueError(
            "a %.1f mm zone with %.1f mm tags needs a %.1f mm bounding box, which "
            "does not fit on A4 (%.0f x %.0f mm) even in landscape. Shrink "
            "zone_size or tag_size, or print on a larger sheet."
            % (zone_size_mm, tag_size_mm, bbox_mm, *A4_MM))

    px_per_mm = dpi / 25.4
    page_px = (round(A4_MM[0] * px_per_mm), round(A4_MM[1] * px_per_mm))
    canvas = np.full((page_px[1], page_px[0]), 255, dtype=np.uint8)

    def mm_to_px(x_mm, y_mm):
        """Page-local mm (origin top-left, +Y down) -> pixel coords."""
        return int(round(x_mm * px_per_mm)), int(round(y_mm * px_per_mm))

    # Layout: tag square centred horizontally, positioned in the upper portion
    # of the page so there is room below for the calibration ruler and notes.
    #
    # HEADER_CLEARANCE_MM must clear BOTH lines of title text AND the top-row
    # ID labels, which sit LABEL_GAP_MM above the tags' own top edge -- with
    # too little clearance here the "ID 3" label lands directly on top of the
    # title (this happened at 10mm; 26mm leaves a clean gap).
    HEADER_CLEARANCE_MM = 26.0
    centre_x_mm = A4_MM[0] / 2.0
    centre_y_mm = MARGIN_MM + bbox_mm / 2.0 + HEADER_CLEARANCE_MM

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, "%s -- AprilTag 36h11, IDs %s" % (title, list(tag_ids)),
                mm_to_px(MARGIN_MM, MARGIN_MM + 4), font, 0.9, 0, 2, cv2.LINE_AA)
    cv2.putText(canvas, "Tag-centre square: %.1f mm (%.2f in). Tag size: %.1f mm (%.2f in)."
                % (zone_size_mm, zone_size_mm / 25.4, tag_size_mm, tag_size_mm / 25.4),
                mm_to_px(MARGIN_MM, MARGIN_MM + 12), font, 0.42, 0, 1, cv2.LINE_AA)

    dictionary = _dictionary()
    half_zone = zone_size_mm / 2.0
    tag_positions_mm = []
    for tag_id, (sx, sy) in zip(tag_ids, zv.ZONE_CORNER_SIGNS):
        # zone +Y is up; page mm is down, matching the y-flip used everywhere
        # else in this feature (zone_vision_selftest.render_zone does the same).
        tag_cx = centre_x_mm + sx * half_zone
        tag_cy = centre_y_mm - sy * half_zone
        tag_positions_mm.append((tag_id, sx, sy, tag_cx, tag_cy))

        side_px = int(round(tag_size_mm * px_per_mm))
        bitmap = _marker_bitmap(dictionary, tag_id, side_px)
        cx_px, cy_px = mm_to_px(tag_cx, tag_cy)
        x0, y0 = cx_px - side_px // 2, cy_px - side_px // 2
        canvas[y0:y0 + side_px, x0:x0 + side_px] = bitmap

        # ID label OUTSIDE the tag, away from the zone centre, clear of the
        # quiet zone every AprilTag needs around its border to detect reliably.
        label_x = tag_cx + sx * (tag_size_mm / 2.0 + LABEL_GAP_MM)
        label_y = tag_cy - sy * (tag_size_mm / 2.0 + LABEL_GAP_MM)
        lx_px, ly_px = mm_to_px(label_x, label_y)
        text = "ID %d" % tag_id
        (tw, th), _ = cv2.getTextSize(text, font, 0.55, 2)
        cv2.putText(canvas, text, (lx_px - tw // 2, ly_px + (th if sy > 0 else -2)),
                    font, 0.55, 0, 2, cv2.LINE_AA)

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

    # Zone axes, matching the +X/+Y convention tag_pick_place.py and
    # zone_vision.py use everywhere else -- useful when deciding --zone-yaw.
    #
    # Drawn as a small LEGEND in the margin below the ruler, NOT inside the
    # zone interior. An earlier version put this at the zone centre, which is
    # exactly where a real block gets placed -- feeding the rendered sheet
    # back through zv.analyze() as a check on this script caught it directly:
    # the arrows themselves were segmented as a phantom "23.6 x 44.2 mm
    # unknown" object. The working area must stay visually empty.
    legend_y_mm = A4_MM[1] - MARGIN_MM - 15.0
    legend_x_mm = MARGIN_MM + 15.0
    cv2.putText(canvas, "Zone axes (for --zone-yaw reference):",
                mm_to_px(MARGIN_MM, legend_y_mm - 6), font, 0.42, 0, 1, cv2.LINE_AA)
    origin_px = mm_to_px(legend_x_mm, legend_y_mm)
    for (dx, dy), label in (((15, 0), "+X"), ((0, -15), "+Y")):
        tip_px = mm_to_px(legend_x_mm + dx, legend_y_mm + dy)
        cv2.arrowedLine(canvas, origin_px, tip_px, 0, 2, tipLength=0.15)
        cv2.putText(canvas, label, tip_px, font, 0.5, 0, 1, cv2.LINE_AA)

    # Calibration ruler -- see module docstring. This is not decorative.
    ruler_y_mm = centre_y_mm + bbox_mm / 2.0 + 18.0
    ruler_x0_mm = centre_x_mm - RULER_LENGTH_MM / 2.0
    p0 = mm_to_px(ruler_x0_mm, ruler_y_mm)
    p1 = mm_to_px(ruler_x0_mm + RULER_LENGTH_MM, ruler_y_mm)
    cv2.line(canvas, p0, p1, 0, 2)
    for mark_mm in (0, 50, 100):
        tx, ty = mm_to_px(ruler_x0_mm + mark_mm, ruler_y_mm)
        cv2.line(canvas, (tx, ty - 6), (tx, ty + 6), 0, 2)
        cv2.putText(canvas, "%d" % mark_mm, (tx - 8, ty + 20), font, 0.4, 0, 1, cv2.LINE_AA)
    cv2.putText(canvas,
                "^ measure this line with a ruler: must be EXACTLY 100.0 mm.",
                mm_to_px(ruler_x0_mm, ruler_y_mm + 12), font, 0.4, 0, 1, cv2.LINE_AA)
    cv2.putText(canvas,
                "If it isn't, your printer rescaled the page -- reprint at",
                mm_to_px(ruler_x0_mm, ruler_y_mm + 18), font, 0.4, 0, 1, cv2.LINE_AA)
    cv2.putText(canvas,
                "100% / Actual Size, NOT 'Fit to page' or 'Shrink to fit'.",
                mm_to_px(ruler_x0_mm, ruler_y_mm + 24), font, 0.4, 0, 1, cv2.LINE_AA)

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
    args = parser.parse_args()

    zones = ["pickup", "place"] if args.zone == "both" else [args.zone]
    os.makedirs(args.out_dir, exist_ok=True)

    for zone_name in zones:
        canvas, dpi = render_sheet(zone_name, args.dpi)
        path = os.path.join(args.out_dir, "%s_zone_A4.png" % zone_name)
        # dpi metadata so a print dialog's "Actual Size" reproduces the real
        # mm scale -- still verify with the ruler, since not every viewer or
        # driver respects it.
        Image.fromarray(canvas).save(path, dpi=(dpi, dpi))
        ids, _ = ZONES[zone_name]
        print("wrote %s  (%s, AprilTag 36h11 IDs %s, %d dpi)"
              % (path, zone_name, list(ids), dpi))

    print("\nPrint at 100% / Actual Size -- NOT 'Fit to page'. Verify the ruler "
          "on the printed sheet measures exactly 100.0 mm before trusting anything else.")


if __name__ == "__main__":
    main()
