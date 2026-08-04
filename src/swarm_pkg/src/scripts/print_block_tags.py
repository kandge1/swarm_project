#!/usr/bin/env python3
"""One A4 sheet per block: the six face tags, cut out and stuck on by hand.

    python3 print_block_tags.py --out-dir ../../../../print_sheets

writes block_tags_cube_A4.png and block_tags_cuboid_A4.png.

The id scheme lives in block_coordinates.py -- TOP, BOTTOM and four distinct
SIDE ids per block, 8-19 -- and this script only draws it. See that module for
why the four sides are not one shared id.

------------------------------------------------------------------------------
HOW BIG THE TAGS ARE, AND WHY IT IS THE WHOLE PROBLEM
------------------------------------------------------------------------------
A 36h11 tag is EIGHT modules across its black square (6x6 of data plus a
one-module black border). This script asserts that against OpenCV at run time
rather than trusting it, because everything below is divided by it. Note that
print_tag_sheet.py:67 claims tag_size/10; that comment is wrong.

The tag is limited by the block FACE, not by the paper: with one module of
quiet zone on each side, a 30 mm face takes a 24 mm tag (block_coordinates.
max_tag_size_for_face). That is the default. --tag-size overrides it, and
--face-size recomputes it for a different block.

Then the pixel budget decides whether any of this works. The project's own
invariant is px/m * distance = 551 (APRIL_TAGS_DEV.md, "px/m is a free height
gauge"), so at the agreed angled survey pose, 0.305 m from the zone centre:

    24 mm tag, flat on to the lens        43 px      5.4 px/module
    seen as a block TOP  (x0.81)          35 px      4.4 px/module   ok
    seen as a block SIDE (x0.59)          26 px      3.2 px/module   marginal

Roughly 3 px/module is where 36h11 stops decoding reliably, so SIDE tags at
that pose are a coin flip and TOP tags are fine. --report prints this table for
whatever size and distance you ask about; use it before printing, not after.

------------------------------------------------------------------------------
STICKING THEM ON
------------------------------------------------------------------------------
Printed on every sheet, because that is where it is needed:

    Stand the block with its TOP tag upward and that tag's arrow pointing AWAY
    from you. SIDE0 is the far face. Going counter-clockwise seen FROM ABOVE:
    SIDE1 left, SIDE2 near, SIDE3 right.

Get a side index wrong and nothing complains -- the tag still decodes, and the
block yaw it implies is silently 90 degrees out. Check the four sides read
0,1,2,3 counter-clockwise before the glue dries.

Scale does NOT have to be perfect, but it does have to be KNOWN: the tag size
is a parameter everywhere downstream, and the tag-scale height estimate
(block_coordinates.height_from_scale) divides by it. Measure the ruler on the
sheet, then measure a tag.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import block_coordinates as bc  # noqa: E402

A4_MM = (210.0, 297.0)          # portrait, (width, height)
MARGIN_MM = 12.0
RULER_LENGTH_MM = 150.0

# 1.0 = draw the page at true A4 and let the printer print it 1:1.
#
# MEASURED ON THIS PRINTER 2026-08-04, and it is the opposite of what
# print_tag_sheet.py assumes. That script pre-scales by 16/14 to cancel a fixed
# fit-to-page shrink measured on a different printer. Run through this one, a
# 22.5 mm nominal tag came out at 19.6 mm -- and 22.5 / (16/14) = 19.69. So this
# printer honoured fit-to-page exactly, scaling the deliberately-oversized page
# back to true A4 and cancelling the correction perfectly. The pre-scale WAS the
# entire error.
#
# If a tag ever comes out wrong again, set this to (nominal / measured) from a
# print made at 1.0 -- do not guess, and do not copy print_tag_sheet.py's value.
DEFAULT_PRINT_CORRECTION = 1.0

# Default square face the tag has to fit inside. pick_place.BLOCK_HEIGHT_M is
# 0.030 (measured on hardware 2026-08-03) and the cube is 30 mm on every face.
DEFAULT_FACE_SIZE_M = 0.030

# Expressed in MODULES, not mm, so it tracks the tag size. Lives in
# block_coordinates because max_tag_size_for_face has to divide by it, and the
# measured reason for it being 1.25 rather than the spec's 1.0 is documented
# there.
QUIET_ZONE_MODULES = bc.QUIET_ZONE_MODULES

CUT_MARGIN_MM = 3.0             # paper outside the quiet zone, to cut in
LABEL_H_MM = 12.0
COL_GAP_MM = 22.0
ROW_GAP_MM = 7.0

# Text is sized in MILLIMETRES OF CAP HEIGHT ON PAPER, not in cv2 font scale.
# cv2's scale is relative to a ~22 px reference, so at 300 dpi a "0.38" that
# looks reasonable in an image viewer prints at 0.67 mm -- about a quarter the
# height of normal body text, and illegible. print_tag_sheet.py and
# print_zone_tags.py both have this; measure their output before trusting it.
FONT = cv2.FONT_HERSHEY_SIMPLEX
TITLE_MM = 4.0
BODY_MM = 2.4
LABEL_MM = 3.0
HINT_MM = 2.2
LINE_LEADING = 1.75             # line pitch as a multiple of cap height

# The "up" arrow sits ABOVE each cell's cut line, so the top row needs this much
# clear air under the header or the arrows land in the last line of text.
ARROW_CLEARANCE_MM = 6.0

# Foreshortening of a block's top and side faces at the agreed angled survey
# pose `107 49 -103 0 0 135`, whose tool axis is 36 deg from vertical
# (APRIL_TAGS_DEV.md, CONSTRAINT 2). Used only by --report.
ANGLED_VIEW_DISTANCE_M = 0.305
ANGLED_VIEW_TOP_FORESHORTEN = 0.81
ANGLED_VIEW_SIDE_FORESHORTEN = 0.59
# px/m * distance, anchored on the survey (2466 px/m at 0.2235 m).
PX_M_INVARIANT = 551.0
# Below MIN, 36h11 decoding becomes unreliable; at or above GOOD it is not the
# limiting factor. Between them is a coin flip that will look like intermittent
# vision bugs rather than a sizing problem, which is the reason to settle it
# with arithmetic before printing.
MIN_PX_PER_MODULE = 3.0
GOOD_PX_PER_MODULE = 4.0

MODULES_ACROSS = 8              # asserted against OpenCV in _dictionary()


def _dictionary():
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)


def _marker_bitmap(dictionary, tag_id, side_px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_px)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_px)


def _assert_module_count(dictionary):
    """Confirm a rendered tag really is MODULES_ACROSS modules wide.

    The whole pixel budget is per-MODULE, so an assumed module count is an
    assumed answer. Rendering one tag and taking the gcd of every black/white
    transition gives the module size directly; it costs a millisecond.
    """
    from math import gcd
    side = MODULES_ACROSS * 30
    marker = _marker_bitmap(dictionary, bc.BLOCK_TAG_ID_MIN, side)
    step = 0
    for line in list(marker) + list(marker.T):
        for index in np.nonzero(np.diff(line.astype(int)))[0] + 1:
            step = gcd(step, int(index))
    step = gcd(step, side)
    if step == 0 or side // step != MODULES_ACROSS:
        raise SystemExit(
            "this OpenCV renders DICT_APRILTAG_36h11 at %s modules, not %d -- "
            "every px/module number in this script and in block_coordinates."
            "max_tag_size_for_face is wrong for it."
            % (side // step if step else "?", MODULES_ACROSS))


def _cap_unit():
    """cv2 cap height in px at font scale 1.0, measured rather than assumed."""
    return cv2.getTextSize("X", FONT, 1.0, 1)[0][1]


def _text_writer(page, px_per_mm):
    """put(text, x_mm, y_mm, cap_mm) -> width in mm. Sizes text on PAPER.

    Returning the drawn width lets the caller check that a line actually fits
    between the margins, which is the other half of the legibility problem: a
    string that runs off the page is as useless as one too small to read.
    """
    unit = _cap_unit()

    def put(text, x_mm, y_mm, cap_mm, bold=False):
        scale = cap_mm * px_per_mm / unit
        thickness = max(1, int(round(px_per_mm * (0.32 if bold else 0.20))))
        cv2.putText(page, text,
                    (int(round(x_mm * px_per_mm)), int(round(y_mm * px_per_mm))),
                    FONT, scale, 0, thickness, cv2.LINE_AA)
        return cv2.getTextSize(text, FONT, scale, thickness)[0][0] / px_per_mm

    return put


def _dashed_rect(img, p0, p1, dash_px):
    x0, y0 = p0
    x1, y1 = p1
    for x in range(x0, x1, dash_px * 2):
        cv2.line(img, (x, y0), (min(x + dash_px, x1), y0), 160, 1)
        cv2.line(img, (x, y1), (min(x + dash_px, x1), y1), 160, 1)
    for y in range(y0, y1, dash_px * 2):
        cv2.line(img, (x0, y), (x0, min(y + dash_px, y1)), 160, 1)
        cv2.line(img, (x1, y), (x1, min(y + dash_px, y1)), 160, 1)


# Printed under each side tag so the counter-clockwise rule is checkable at the
# moment of sticking it on, not afterwards.
SIDE_HINTS = {
    "side0": "FAR face (arrow points at it)",
    "side1": "LEFT face",
    "side2": "NEAR face (toward you)",
    "side3": "RIGHT face",
    "top": "arrow -> the FAR face",
    "bottom": "arrow -> the FAR face",
}


def render_sheet(block_class, dpi, tag_size_mm, face_size_mm,
                 print_correction=1.0):
    px_per_mm = dpi / 25.4 * print_correction

    def mm(v):
        return int(round(v * px_per_mm))

    def mm_to_px(x, y):
        return (mm(x), mm(y))

    page = np.full((mm(A4_MM[1]), mm(A4_MM[0])), 255, dtype=np.uint8)
    put = _text_writer(page, px_per_mm)
    dictionary = _dictionary()
    _assert_module_count(dictionary)

    quiet_mm = tag_size_mm / MODULES_ACROSS * QUIET_ZONE_MODULES
    cell_w_mm = tag_size_mm + 2 * quiet_mm + 2 * CUT_MARGIN_MM
    cell_h_mm = cell_w_mm + LABEL_H_MM

    cols, rows = 2, 3
    grid_w_mm = cols * cell_w_mm + (cols - 1) * COL_GAP_MM
    grid_h_mm = rows * cell_h_mm + (rows - 1) * ROW_GAP_MM

    # ONE short title line and nothing else. Every paragraph, note and ruler was
    # removed 2026-08-04: they are content a fit-to-page printer will scale the
    # tags around, and the tags are the only thing on this page whose size has
    # to be right. What they said is in this file's docstring and in the console
    # output, both of which are free.
    ids = bc.tag_ids_for_block(block_class)
    title = "%s  36h11  id %d-%d" % (block_class.upper(), min(ids), max(ids))

    header_mm = TITLE_MM * 1.8 + ARROW_CLEARANCE_MM
    grid_total_mm = header_mm + grid_h_mm
    x0_mm = (A4_MM[0] - grid_w_mm) / 2.0
    # Centre the whole block vertically too, so nothing lands near an edge --
    # page margins are where a fit-to-page shrink does its damage.
    y0_mm = max(MARGIN_MM + header_mm, (A4_MM[1] - grid_total_mm) / 2.0 + header_mm)
    if x0_mm < MARGIN_MM:
        raise SystemExit(
            "a %.1f mm tag does not fit two across A4 at this layout "
            "(%.1f mm needed, %.1f available)"
            % (tag_size_mm, grid_w_mm, A4_MM[0] - 2 * MARGIN_MM))

    put(title, x0_mm, y0_mm - header_mm + TITLE_MM, TITLE_MM, bold=True)

    for index, tag_id in enumerate(ids):
        info = bc.describe(tag_id)
        col, row = index % cols, index // cols          # fill ACROSS each row
        cx_mm = x0_mm + col * (cell_w_mm + COL_GAP_MM)
        cy_mm = y0_mm + row * (cell_h_mm + ROW_GAP_MM)

        _dashed_rect(page, mm_to_px(cx_mm, cy_mm),
                     mm_to_px(cx_mm + cell_w_mm, cy_mm + cell_h_mm),
                     dash_px=mm(1.5))

        side_px = mm(tag_size_mm)
        marker = _marker_bitmap(dictionary, tag_id, side_px)
        tx_mm = cx_mm + CUT_MARGIN_MM + quiet_mm
        ty_mm = cy_mm + CUT_MARGIN_MM + quiet_mm
        tx, ty = mm(tx_mm), mm(ty_mm)
        page[ty:ty + side_px, tx:tx + side_px] = marker

        # Centre cross-hairs, in the quiet zone rather than on the tag, so the
        # tag's centre can be measured to without marking the tag itself.
        mid_x_mm = tx_mm + tag_size_mm / 2.0
        mid_y_mm = ty_mm + tag_size_mm / 2.0
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            reach = tag_size_mm / 2.0 + 0.5
            cv2.line(page,
                     mm_to_px(mid_x_mm + dx * reach, mid_y_mm + dy * reach),
                     mm_to_px(mid_x_mm + dx * (reach + 1.6),
                              mid_y_mm + dy * (reach + 1.6)),
                     0, max(1, mm(0.25)), cv2.LINE_AA)

        # "Up" arrow, drawn OUTSIDE the cut line so it cannot eat quiet zone.
        cv2.arrowedLine(page,
                        mm_to_px(mid_x_mm, cy_mm - 0.8),
                        mm_to_px(mid_x_mm, cy_mm - 3.6),
                        0, max(1, mm(0.35)), cv2.LINE_AA, tipLength=0.45)

        label_y_mm = cy_mm + cell_w_mm + LABEL_MM + 1.0
        put("id %d   %s" % (tag_id, info.face.upper()),
            cx_mm + 1.0, label_y_mm, LABEL_MM, bold=True)
        put(SIDE_HINTS[info.face], cx_mm + 1.0,
            label_y_mm + HINT_MM * LINE_LEADING + 0.6, HINT_MM)

    # The ruler. Without it a fit-to-page shrink is undetectable, and a tag that
    # decodes perfectly at the wrong size is exactly the silent failure the zone
    # sheet was designed around.
    bottom_mm = y0_mm + grid_h_mm
    if bottom_mm > A4_MM[1] - MARGIN_MM:
        raise SystemExit(
            "layout overflows the page by %.1f mm at a %.1f mm tag"
            % (bottom_mm - A4_MM[1] + MARGIN_MM, tag_size_mm))

    return page


def legibility_report(tag_size_mm, distance_m):
    """Predicted pixels per module for the three viewing cases.

    Purely arithmetic on the project's px/m invariant -- no camera model, no
    intrinsics. It is a go/no-go on the tag size BEFORE anything is printed.
    """
    px_per_m = PX_M_INVARIANT / distance_m
    flat_px = tag_size_mm / 1000.0 * px_per_m
    rows = [("flat on to the lens", 1.0),
            ("block TOP  at 36 deg", ANGLED_VIEW_TOP_FORESHORTEN),
            ("block SIDE at 36 deg", ANGLED_VIEW_SIDE_FORESHORTEN)]

    lines = ["%.1f mm tag at %.3f m  (%.0f px/m)"
             % (tag_size_mm, distance_m, px_per_m)]
    for name, foreshorten in rows:
        px = flat_px * foreshorten
        per_module = px / MODULES_ACROSS
        verdict = "ok" if per_module >= GOOD_PX_PER_MODULE else (
            "marginal" if per_module >= MIN_PX_PER_MODULE else "TOO SMALL")
        lines.append("  %-22s %5.1f px   %.1f px/module   %s"
                     % (name, px, per_module, verdict))

    # How close the lens has to get for the WORST case -- a side face -- to stop
    # being the limiting factor. This is the number that decides whether the
    # angled survey pose can read side tags at all, or whether stage 2 needs a
    # closer vantage.
    for threshold, name in ((MIN_PX_PER_MODULE, "decode at all"),
                            (GOOD_PX_PER_MODULE, "decode comfortably")):
        need_px_per_m = (threshold * MODULES_ACROSS
                         / ANGLED_VIEW_SIDE_FORESHORTEN
                         / (tag_size_mm / 1000.0))
        lines.append("  SIDE tags %-18s (%.1f px/module): lens within %.3f m"
                     % (name, threshold, PX_M_INVARIANT / need_px_per_m))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", default=".",
                        help="directory for block_tags_<class>_A4.png "
                             "(default %(default)s)")
    parser.add_argument("--block", choices=list(bc.BLOCK_CLASSES) + ["both"],
                        default="both", help="which sheet(s) (default %(default)s)")
    parser.add_argument("--dpi", type=float, default=300.0)
    parser.add_argument("--face-size", type=float, default=DEFAULT_FACE_SIZE_M,
                        help="square face the tag must fit, METRES "
                             "(default %(default)s). The cuboid's is its SQUARE "
                             "top/bottom face, not its length.")
    parser.add_argument("--tag-size", type=float, default=None,
                        help="printed tag size in METRES; overrides the size "
                             "derived from --face-size")
    parser.add_argument("--print-correction", type=float,
                        default=DEFAULT_PRINT_CORRECTION,
                        help="pre-scale to cancel a printer's fixed shrink: "
                             "(nominal / measured) from a 1.0 print. "
                             "Default %(default).4f matches print_tag_sheet.py.")
    parser.add_argument("--report", action="store_true",
                        help="print the px/module legibility table and exit "
                             "without drawing anything")
    parser.add_argument("--distance", type=float, default=ANGLED_VIEW_DISTANCE_M,
                        help="lens-to-block distance for --report, METRES "
                             "(default %(default)s = the angled survey pose)")
    args = parser.parse_args()

    tag_size_m = (args.tag_size if args.tag_size is not None
                  else bc.max_tag_size_for_face(args.face_size,
                                                QUIET_ZONE_MODULES))
    tag_size_mm = tag_size_m * 1000.0
    face_size_mm = args.face_size * 1000.0

    if tag_size_m > args.face_size:
        raise SystemExit(
            "--tag-size %.1f mm is larger than the %.1f mm face: it cannot be "
            "stuck on flat, let alone keep a quiet zone."
            % (tag_size_mm, face_size_mm))

    print(legibility_report(tag_size_mm, args.distance))
    if args.report:
        return

    classes = (list(bc.BLOCK_CLASSES) if args.block == "both" else [args.block])
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print()
    for block_class in classes:
        page = render_sheet(block_class, args.dpi, tag_size_mm, face_size_mm,
                            args.print_correction)
        path = os.path.join(out_dir, "block_tags_%s_A4.png" % block_class)
        if not cv2.imwrite(path, page):
            raise SystemExit("could not write %s" % path)
        ids = bc.tag_ids_for_block(block_class)
        print("wrote %s  (%dx%d px)" % (path, page.shape[1], page.shape[0]))
        print("  %-6s ids %s"
              % (block_class,
                 ", ".join("%d=%s" % (i, bc.describe(i).face.upper())
                           for i in ids)))

    if args.print_correction != 1.0:
        print("\nPage drawn %.4fx oversized to cancel a printer shrink. Print it "
              "TO FIT THE PAGE, not at 100%%." % args.print_correction)
    else:
        print("\nPrint at 100%% / Actual Size. Then measure one tag's black square:"
              "\nit must be %.1f mm. If it is not, re-run with"
              "\n  --print-correction %.1f/<measured_mm>" % (tag_size_mm, tag_size_mm))

    # The placement rules used to be printed on the sheet. They are here instead:
    # anything on the page is content a fit-to-page printer scales the tags
    # around, and console text costs nothing.
    print("""
STICKING THEM ON
  Stand the block with its TOP tag upward and that tag's arrow pointing AWAY
  from you. SIDE0 is the far face. Then counter-clockwise SEEN FROM ABOVE:
      SIDE0 far      SIDE1 left      SIDE2 near      SIDE3 right
  Keep the white border -- it is the quiet zone the detector needs.

  A wrong SIDE index is SILENT: the tag still decodes and the block yaw it
  implies is 90 deg out. Check the four sides read 0,1,2,3 counter-clockwise
  from above before the glue dries.""")


if __name__ == "__main__":
    main()
