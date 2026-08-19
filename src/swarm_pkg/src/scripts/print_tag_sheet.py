#!/usr/bin/env python3
"""All 8 AprilTags on ONE A4 sheet, as individual cut-out squares.

WHY THIS EXISTS ALONGSIDE print_zone_tags.py. That script renders each zone as a
complete pre-laid-out mat, which is lovely when the printer honours "100% /
Actual Size" and useless when it does not -- and campus printers routinely bury
or ignore that setting. A mat printed at 96% is silently wrong in a way nothing
downstream can detect, because the tags still decode perfectly; only the
GEOMETRY is off, and the homography will happily return a confident wrong
position from it.

This sheet sidesteps the whole problem by not depending on print scale at all:

  1. The tags are cut out individually and positioned BY HAND with a ruler, so
     the zone size is set by your measurement, not the printer's.
  2. Whatever scale the printer applied, you MEASURE the printed tag and put
     that number in the config. tag_size is a parameter (zone_vision.ZoneSpec),
     not a constant -- a uniformly scaled tag is not a defect, it is just a
     different tag_size.

So the only thing you have to get right is knowing what came out, and the ruler
printed on the sheet tells you that.

  python3 print_tag_sheet.py --out ../../../../print_sheets/all_tags_A4.png

WHAT MUST BE TRUE WHEN YOU PLACE THEM
  - All 8 tags share one "up" direction, aligned with zone +Y. The arrow printed
    under each tag is that direction. zone_vision.TAG_CORNER_OFFSETS assumes it;
    get it wrong and the homography residual will be large and roughly tag-sized.
  - Tag CENTRES sit on the corners of the square, not their edges. The centre
    cross-hair is there to measure to.
  - Which id goes at which corner is fixed by zone_vision.ZONE_CORNER_SIGNS and
    printed on each tag's label. Swapping two of them yields a rotated or
    mirrored zone frame, which the residual will NOT catch -- a mirrored fit is
    still a perfect fit.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402

A4_MM = (210.0, 297.0)          # portrait, (width, height)
MARGIN_MM = 10.0
RULER_LENGTH_MM = 150.0

# Some printers (campus ones in particular) apply a fixed "fit to page" shrink
# with no way to disable it, regardless of the image's DPI metadata or an
# "Actual Size" setting that simply isn't offered. Measured on the lab's
# printer: a nominal 1.000 in tag came out 14/16 in (0.875 in), and the 4.000
# in zone square came out 3.500 in -- the SAME 0.875 ratio, confirming it is a
# uniform page scale, not a tag-specific artifact. Pre-scaling every mm
# dimension by the inverse (1/0.875 = 16/14) before rendering cancels that
# shrink, so the page draws "too big" on screen but comes out correct on
# paper. If your printer's shrink differs, recalibrate with
# --print-correction = (nominal size) / (actual measured size) using the
# CURRENT default (1.0 == no correction) print.
DEFAULT_PRINT_CORRECTION = 16.0 / 14.0

# White border around each tag's black square. AprilTag detection REQUIRES a
# quiet zone -- a tag cut flush to its black edge is substantially harder to
# detect and can fail outright against a dark mat. One module of a 36h11 tag is
# tag_size/10, so 4 mm is comfortably over one module at 25.4 mm.
QUIET_ZONE_MM = 4.0

# Extra paper outside the quiet zone, to cut along. Keeps scissors away from the
# quiet zone itself.
CUT_MARGIN_MM = 3.0

LABEL_H_MM = 9.0                # text strip under each tag
COL_GAP_MM = 6.0
ROW_GAP_MM = 5.0

CORNER_NAMES = ("-X-Y", "+X-Y", "+X+Y", "-X+Y")


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker_bitmap(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def render_minimal(dpi, tag_size_mm, print_correction=1.0):
    """Tags and cut outlines only. Nothing else on the page.

    For printers whose UI hides or ignores "Actual Size": stripping the sheet to
    the tags themselves removes every element that could be mistaken for content
    to fit-to-page around, and two centred columns keep every tag far from the
    paper edge, where scaling and margin-clipping do their damage.

    ORDER IS THE ONLY LABELLING, and it matters. Reading down the LEFT column
    then down the RIGHT:

        left  (top->bottom): 0, 1, 2, 3   -- PICKUP, counter-clockwise from -X-Y
        right (top->bottom): 4, 5, 6, 7   -- PLACE,  counter-clockwise from -X-Y

    Pencil the id on the BACK of each square as you cut. Without it you have
    eight near-identical squares, and the failure that causes is the one the
    homography residual cannot see: rotate all four tags of a zone consistently
    (0->1->2->3) and the fit is still PERFECT while the zone frame is silently
    turned 90 degrees. A random scramble blows up the RMS and is caught; a
    consistent rotation is not.
    """
    px_per_mm = dpi / 25.4 * print_correction

    def mm(v):
        return int(round(v * px_per_mm))

    def mm_to_px(x, y):
        return (mm(x), mm(y))

    page = np.full((mm(A4_MM[1]), mm(A4_MM[0])), 255, dtype=np.uint8)
    dictionary = _dictionary()

    cell_mm = tag_size_mm + 2 * QUIET_ZONE_MM + 2 * CUT_MARGIN_MM
    cols, rows = 2, 4
    col_gap_mm = 30.0
    row_gap_mm = 18.0
    grid_w_mm = cols * cell_mm + (cols - 1) * col_gap_mm
    grid_h_mm = rows * cell_mm + (rows - 1) * row_gap_mm

    # Centre the whole block on the page in BOTH axes -- the point of this
    # layout is that nothing lands near an edge.
    x0_mm = (A4_MM[0] - grid_w_mm) / 2.0
    y0_mm = (A4_MM[1] - grid_h_mm) / 2.0
    if min(x0_mm, y0_mm) < 20.0:
        raise SystemExit(
            "layout leaves only %.1f mm of edge clearance; reduce tag size or "
            "the gaps." % min(x0_mm, y0_mm))

    all_ids = list(zv.PICKUP_TAG_IDS) + list(zv.PLACE_TAG_IDS)
    for index, tag_id in enumerate(all_ids):
        col, row = index // rows, index % rows        # fill DOWN each column
        cx_mm = x0_mm + col * (cell_mm + col_gap_mm)
        cy_mm = y0_mm + row * (cell_mm + row_gap_mm)

        _dashed_rect(page, mm_to_px(cx_mm, cy_mm),
                     mm_to_px(cx_mm + cell_mm, cy_mm + cell_mm),
                     dash_px=mm(1.5))

        side_px = mm(tag_size_mm)
        marker = _marker_bitmap(dictionary, tag_id, side_px)
        tx = mm(cx_mm + CUT_MARGIN_MM + QUIET_ZONE_MM)
        ty = mm(cy_mm + CUT_MARGIN_MM + QUIET_ZONE_MM)
        page[ty:ty + side_px, tx:tx + side_px] = marker

    return page, x0_mm, y0_mm


def render_sheet(dpi, tag_size_mm, zone_size_mm, print_correction=1.0):
    px_per_mm = dpi / 25.4 * print_correction

    def mm(v):
        return int(round(v * px_per_mm))

    def mm_to_px(x, y):
        return (mm(x), mm(y))

    page = np.full((mm(A4_MM[1]), mm(A4_MM[0])), 255, dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    dictionary = _dictionary()

    # One cut-out cell: tag + quiet zone + label strip + cut margin.
    cell_w_mm = tag_size_mm + 2 * QUIET_ZONE_MM + 2 * CUT_MARGIN_MM
    cell_h_mm = cell_w_mm + LABEL_H_MM

    header_mm = 30.0
    cols, rows = 4, 2
    grid_w_mm = cols * cell_w_mm + (cols - 1) * COL_GAP_MM
    grid_h_mm = rows * cell_h_mm + (rows - 1) * ROW_GAP_MM
    if grid_w_mm + 2 * MARGIN_MM > A4_MM[0]:
        raise SystemExit(
            "8 tags at %.1f mm do not fit across A4 at this layout (%.1f mm "
            "needed, %.1f available)." % (tag_size_mm, grid_w_mm, A4_MM[0]))

    x0_mm = (A4_MM[0] - grid_w_mm) / 2.0
    y0_mm = MARGIN_MM + header_mm

    cv2.putText(page, "APRILTAG 36h11  --  CUT OUT AND PLACE BY HAND",
                mm_to_px(MARGIN_MM, MARGIN_MM + 5), font, 0.62, 0, 2, cv2.LINE_AA)
    cv2.putText(page,
                "Nominal tag %.1f mm. DO NOT trust the printer -- measure the ruler "
                "below, then measure a tag." % tag_size_mm,
                mm_to_px(MARGIN_MM, MARGIN_MM + 11), font, 0.38, 0, 1, cv2.LINE_AA)
    cv2.putText(page,
                "Tags 0-3 = PICKUP zone, tags 4-7 = PLACE zone. Place tag CENTRES on "
                "the corners of a %.1f mm square." % zone_size_mm,
                mm_to_px(MARGIN_MM, MARGIN_MM + 16), font, 0.38, 0, 1, cv2.LINE_AA)
    cv2.putText(page,
                "Every arrow must point the SAME way (zone +Y) once placed.",
                mm_to_px(MARGIN_MM, MARGIN_MM + 21), font, 0.38, 0, 1, cv2.LINE_AA)
    if print_correction != 1.0:
        cv2.putText(page,
                    "Pre-scaled by %.4fx (--print-correction) to cancel a known "
                    "printer shrink. The ruler below MUST still read %.0f mm."
                    % (print_correction, RULER_LENGTH_MM),
                    mm_to_px(MARGIN_MM, MARGIN_MM + 26), font, 0.38, 0, 1, cv2.LINE_AA)

    all_ids = list(zv.PICKUP_TAG_IDS) + list(zv.PLACE_TAG_IDS)
    for index, tag_id in enumerate(all_ids):
        col, row = index % cols, index // cols
        cx_mm = x0_mm + col * (cell_w_mm + COL_GAP_MM)
        cy_mm = y0_mm + row * (cell_h_mm + ROW_GAP_MM)

        # Dashed cut outline around the whole cell.
        _dashed_rect(page, mm_to_px(cx_mm, cy_mm),
                     mm_to_px(cx_mm + cell_w_mm, cy_mm + cell_h_mm),
                     dash_px=mm(1.5))

        # The tag itself, centred in the cell's square part.
        side_px = mm(tag_size_mm)
        marker = _marker_bitmap(dictionary, tag_id, side_px)
        tx_mm = cx_mm + CUT_MARGIN_MM + QUIET_ZONE_MM
        ty_mm = cy_mm + CUT_MARGIN_MM + QUIET_ZONE_MM
        tx, ty = mm(tx_mm), mm(ty_mm)
        page[ty:ty + side_px, tx:tx + side_px] = marker

        # Centre cross-hair in the quiet zone, extending just outside the black
        # square, so the tag's CENTRE can be measured to without marking the tag.
        mid_x_mm = tx_mm + tag_size_mm / 2.0
        mid_y_mm = ty_mm + tag_size_mm / 2.0
        tick = 2.0
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            p0 = mm_to_px(mid_x_mm + dx * (tag_size_mm / 2.0 + 0.6),
                          mid_y_mm + dy * (tag_size_mm / 2.0 + 0.6))
            p1 = mm_to_px(mid_x_mm + dx * (tag_size_mm / 2.0 + 0.6 + tick),
                          mid_y_mm + dy * (tag_size_mm / 2.0 + 0.6 + tick))
            cv2.line(page, p0, p1, 0, max(1, mm(0.25)), cv2.LINE_AA)

        # Label strip: id, zone, corner, and the +Y arrow.
        zone = "PICKUP" if tag_id in zv.PICKUP_TAG_IDS else "PLACE"
        corner = CORNER_NAMES[index % 4]
        label_y_mm = cy_mm + cell_w_mm + 4.0
        cv2.putText(page, "ID %d  %s" % (tag_id, zone),
                    mm_to_px(cx_mm + 1.0, label_y_mm), font, 0.34, 0, 1, cv2.LINE_AA)
        cv2.putText(page, "corner %s" % corner,
                    mm_to_px(cx_mm + 1.0, label_y_mm + 4.0), font, 0.30, 0, 1,
                    cv2.LINE_AA)
        # Up-arrow inside the quiet zone at the tag's top edge.
        ax_mm = mid_x_mm
        cv2.arrowedLine(page,
                        mm_to_px(ax_mm, ty_mm - 1.0),
                        mm_to_px(ax_mm, ty_mm - 3.4),
                        0, max(1, mm(0.3)), cv2.LINE_AA, tipLength=0.45)

    # Ruler, for the measurement the whole sheet depends on.
    ruler_y_mm = y0_mm + grid_h_mm + 16.0
    rx0 = (A4_MM[0] - RULER_LENGTH_MM) / 2.0
    cv2.putText(page,
                "SCALE CHECK -- this line is exactly %.0f mm when printed at 100%%:"
                % RULER_LENGTH_MM,
                mm_to_px(rx0, ruler_y_mm - 4.0), font, 0.38, 0, 1, cv2.LINE_AA)
    cv2.line(page, mm_to_px(rx0, ruler_y_mm),
             mm_to_px(rx0 + RULER_LENGTH_MM, ruler_y_mm), 0, max(1, mm(0.3)))
    for t in range(0, int(RULER_LENGTH_MM) + 1, 10):
        big = (t % 50 == 0)
        cv2.line(page, mm_to_px(rx0 + t, ruler_y_mm),
                 mm_to_px(rx0 + t, ruler_y_mm + (4.0 if big else 2.0)),
                 0, max(1, mm(0.25)))
        if big:
            cv2.putText(page, str(t), mm_to_px(rx0 + t - 2.0, ruler_y_mm + 9.0),
                        font, 0.32, 0, 1, cv2.LINE_AA)

    # The instructions that make a mis-scaled print harmless.
    notes_y = ruler_y_mm + 20.0
    for i, line in enumerate([
        "IF THE RULER IS NOT %.0f mm, THE PRINT IS SCALED. That is FINE:" % RULER_LENGTH_MM,
        "  1. measure one tag's black square, edge to edge, as accurately as you can",
        "  2. pass it in:  --tag-size <measured_metres>   (or set it in the detect request)",
        "  Scale only matters because the code needs the TRUE tag size; it does not",
        "  need the printer to have cooperated.",
        "",
        "PLACING THEM:",
        "  - tag CENTRES (use the cross-hairs) on the corners of a %.1f mm square" % zone_size_mm,
        "  - all arrows pointing the same way = zone +Y",
        "  - ids 0,1,2,3 counter-clockwise from the -X-Y corner; same for 4,5,6,7",
        "  - keep the white border, it is the quiet zone the detector needs",
    ]):
        cv2.putText(page, line, mm_to_px(MARGIN_MM, notes_y + i * 4.6),
                    font, 0.33, 0, 1, cv2.LINE_AA)

    return page


def _dashed_rect(img, p0, p1, dash_px):
    x0, y0 = p0
    x1, y1 = p1
    for x in range(x0, x1, dash_px * 2):
        cv2.line(img, (x, y0), (min(x + dash_px, x1), y0), 160, 1)
        cv2.line(img, (x, y1), (min(x + dash_px, x1), y1), 160, 1)
    for y in range(y0, y1, dash_px * 2):
        cv2.line(img, (x0, y), (x0, min(y + dash_px, y1)), 160, 1)
        cv2.line(img, (x1, y), (x1, min(y + dash_px, y1)), 160, 1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="output PNG path")
    parser.add_argument("--dpi", type=float, default=300.0)
    parser.add_argument("--tag-size", type=float, default=zv.DEFAULT_TAG_SIZE,
                        help="printed tag size in METRES (default %(default)s)")
    parser.add_argument("--zone-size", type=float, default=zv.DEFAULT_ZONE_SIZE,
                        help="tag-centre square in METRES (default %(default)s)")
    parser.add_argument("--minimal", action="store_true",
                        help="tags and cut outlines ONLY, two centred columns, "
                             "nothing near a page edge")
    parser.add_argument("--print-correction", type=float,
                        default=DEFAULT_PRINT_CORRECTION,
                        help="pre-scale factor to cancel a printer's fixed "
                             "shrink: (nominal size) / (actual measured size) "
                             "from a print made with --print-correction 1.0. "
                             "Default %(default).4f is calibrated for a 1.000 "
                             "in tag coming out 14/16 in.")
    args = parser.parse_args()

    edges = None
    if args.minimal:
        page, x0, y0 = render_minimal(args.dpi, args.tag_size * 1000.0,
                                       args.print_correction)
        edges = (x0, y0)
    else:
        page = render_sheet(args.dpi, args.tag_size * 1000.0,
                            args.zone_size * 1000.0, args.print_correction)

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    if not cv2.imwrite(out, page):
        raise SystemExit("could not write %s" % out)
    print("wrote %s  (%dx%d px at %.0f dpi, print-correction %.4fx)"
          % (out, page.shape[1], page.shape[0], args.dpi, args.print_correction))
    print("tag %.1f mm" % (args.tag_size * 1000))
    if edges:
        print("edge clearance: %.1f mm horizontal, %.1f mm vertical" % edges)
        print("ORDER IS THE ONLY LABELLING -- left column top to bottom is "
              "0,1,2,3 (PICKUP); right column is 4,5,6,7 (PLACE).")
        print("Pencil the id on the BACK of each square as you cut.")
    else:
        print("zone square %.1f mm" % (args.zone_size * 1000))
    if args.print_correction != 1.0:
        print("This page is intentionally LARGER than A4 (%.4fx) so that a "
              "printer's fixed fit-to-page shrink lands back on the nominal "
              "size. Print on A4 letting it scale to fit -- do NOT print at "
              "100%%. Then measure the ruler/tag; it must read the nominal "
              "size. If it doesn't, recompute --print-correction = %.4f x "
              "(nominal / actual measured)." % (args.print_correction,
                                                 args.print_correction))
    else:
        print("Print at 100%% / Actual Size if you can -- then measure a tag anyway.")


if __name__ == "__main__":
    main()
