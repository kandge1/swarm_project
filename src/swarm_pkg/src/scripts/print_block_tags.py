#!/usr/bin/env python3
"""One sheet per block: the six face tags, cut out and stuck on by hand.

    python3 print_block_tags.py --out-dir ../../../../print_sheets

writes block_tags_<class>_LETTER.pdf and .png. PRINT THE PDF, at Actual Size,
on the paper named in the filename. print_zone_tags.py's docstring is the
reference for why both of those matter; this sheet imports its calibration.

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

There are two competing constraints and the default resolves them in favour of
pixels, deliberately.

QUIET ZONE says smaller. With QUIET_ZONE_MODULES = 1.25 of white each side --
1.0 is the spec and is measurably not enough, see block_coordinates -- a 30 mm
face takes a 22.5 mm tag (block_coordinates.max_tag_size_for_face). --fit-face
prints that size.

PIXELS say bigger. The project's invariant is px/m * distance = 551
(APRIL_TAGS_DEV.md, "px/m is a free height gauge"), so at the agreed angled
survey pose, 0.305 m from the zone centre:

                          flat      TOP (x0.81)     SIDE (x0.59)
    22.5 mm tag         5.1 px/m      4.1  ok        3.0  dead
    25.4 mm tag         5.7 px/m      4.6  ok        3.4  marginal

Roughly 3 px/module is where 36h11 stops decoding -- the offline sweep in
STACKED_BLOCKS_GUIDE.md puts the decode rate there at 10%, against 95% at 4.0.
So neither size makes SIDE tags work at that pose; 25.4 mm moves the distance
at which they might from 0.229 m to 0.258 m, and moves TOP tags further clear.

THE DEFAULT IS 22.5 mm, the face-derived size. --tag-size overrides it.

25.4 mm was the default for part of 2026-08-04 and was reverted the same day,
which is worth recording because the reasoning was sound and the answer was
still no. On a 30 mm face a 1 in tag leaves 2.3 mm of white a side = 0.72
modules, UNDER the 1.00-module cliff at which a tag fails against any
non-white background. The bet was that the white does not have to stop at the
sticker edge -- that a light-coloured block face would carry the rest of the
quiet zone itself -- in exchange for 0.5 px/module. Printed and looked at, the
tags were too big for the face to spare that white, so the bet lost and the
0.5 px/module was not worth chasing: it does not move SIDE tags into range at
the survey pose (nothing does, see STACKED_BLOCKS_GUIDE.md) and TOP tags clear
the threshold at 22.5 mm anyway.

The general shape of it stands, though, and applies to any future block: the
quiet zone need not be paper. On a light block face a larger tag is worth
trying; on a dark one it is not.

The cut square never exceeds the face: past that point the quiet zone is
squeezed rather than the square grown, so cutting on the line always gives a
sticker that lies flat on the block. --report prints the whole budget for any
size and distance; use it before printing, not after.

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
(block_coordinates.height_from_scale) divides by it. Measure the 150 mm ruler
on the sheet -- not a tag, which is six times shorter and therefore six times
worse to measure -- and either re-run with a corrected --print-correction or
set block_detector_node's block_tag_size to the size you actually got.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import block_coordinates as bc  # noqa: E402
# For the paper these printers feed and the ruler length, which describe the
# hardware rather than this sheet, so they get one home. NOT for CONTENT_SCALE
# -- that is calibrated on a different print chain and measurably wrong for
# this one, see DEFAULT_PRINT_CORRECTION. print_zone_tags.py reached the same
# conclusions first and the long way round; its docstring is the reference.
import print_zone_tags as pz  # noqa: E402

# Portrait, (width, height). The zone sheets are authored on LETTER because
# that is what these printers feed, and a page-size mismatch under fit-to-page
# is what produced non-square output there. Same default here.
PAPER_MM = {
    "a4": (210.0, 297.0),
    "letter": pz.PAGE_MM,
}
DEFAULT_PAPER = "letter"
A4_MM = PAPER_MM["a4"]
MARGIN_MM = 12.0

# The tag size is DERIVED from the block face, not fixed here: it is whatever
# keeps a full QUIET_ZONE_MODULES of white on a --face-size face, which is
# 22.5 mm on the 30 mm blocks. --tag-size overrides it for a one-off.
#
# There is deliberately no DEFAULT_TAG_SIZE_M constant. A fixed default stops
# tracking --face-size the moment anyone changes the block, and a tag size that
# silently no longer fits its face is the failure this whole file exists to
# avoid. 25.4 mm sat here for part of 2026-08-04; see the docstring for why it
# went away.

# A 150 mm baseline measures to ~0.7% with a ruler that reads to 1 mm; a 25 mm
# tag measures to 4%. Two rounds of this were spent measuring tags, so the
# sheet carries the long baseline again. Same length as the zone sheets, for
# the same reason and calibrating the same constant.
RULER_LENGTH_MM = pz.RULER_LENGTH_MM

# Multiplies the CONTENT against a page that stays the size of the paper, to
# cancel the shrink these printers apply. 1.0 = draw at nominal size.
#
# THE PRINT CHAIN, every row the same 22.5 mm nominal tag, measured off paper:
#
#     PNG, correction 16/14              19.6  mm   0.871
#     PNG, correction 1.0                19.58 mm   0.870   <- correction inert
#     PDF on A4 geometry, Actual Size    21.27 mm   0.945   <- pHYs fix landed
#     PDF, content-scaled                21    mm
#
# Rows 1 and 2 are the finding: two corrections 14% apart printed the SAME tag,
# so the correction was not over-cancelling a shrink, it had no effect on
# physical size at all. It scaled px_per_mm, which scales the canvas along with
# the content, leaving the tag at the same FRACTION of the page -- and a
# fraction of a page is precisely what survives a printer mapping an image onto
# paper. print_zone_tags.py measured the identical no-op independently: a
# 1.0926x pre-scale moved its printed tag from 23 mm to 23 mm.
#
# Underneath that, a cv2.imwrite PNG carries no pHYs chunk, so it never states
# its size in mm and "Actual Size" has nothing to be actual against. write_sheet
# emits a PDF with a real page box now, which is what row 3 recovered.
#
# MEASURED ON THIS CHAIN 2026-08-04, off the 150 mm ruler: a sheet drawn at
# print_zone_tags.CONTENT_SCALE (1.0925) printed its ruler at 154.42 mm, i.e.
# 3% OVER. So this chain -- PDF, Actual Size, this printer -- needs
#
#     1.0925 * 150/154.42 = 1.0612
#
# and that is independently corroborated: the implied shrink, 1/1.0612 = 0.942,
# matches the 0.945 measured off the A4 PDF two prints earlier. Two different
# papers, two different measurements, the same number -- so what is left is the
# printer's own printable-area inset, not a paper-size fit, and it is
# deterministic enough to cancel.
#
# CONTENT_SCALE is NOT wrong; it is fitted to a different chain (PNG through a
# campus printer applying fit-to-page unconditionally, print_zone_tags.py's
# docstring). Sharing one constant across both was tried here and this
# measurement is what ruled it out. If the zone mat is ever reprinted through
# THIS chain it needs recalibrating the same way -- and if it was printed
# through this one already, its 101.6 mm square is ~3% oversized, which
# zone_vision would silently pass into every position it reports.
#
# Recalibrate with --measured-ruler; do not do the arithmetic by hand.
DEFAULT_PRINT_CORRECTION = 1.0612

# Default square face the tag has to fit inside. pick_place.BLOCK_HEIGHT_M is
# 0.030 (measured on hardware 2026-08-03) and the cube is 30 mm on every face.
DEFAULT_FACE_SIZE_M = 0.030

# Expressed in MODULES, not mm, so it tracks the tag size. Lives in
# block_coordinates because max_tag_size_for_face has to divide by it, and the
# measured reason for it being 1.25 rather than the spec's 1.0 is documented
# there.
QUIET_ZONE_MODULES = bc.QUIET_ZONE_MODULES

# A tag larger than the face can carry gets its quiet zone SQUEEZED to whatever
# is left, rather than the cut square growing past the face. Cutting on the
# line then gives a sticker exactly the size of the block face -- which is the
# only thing that can actually be stuck on flat.
#
# At 25.4 mm on a 30 mm face that leaves 2.3 mm a side = 0.72 modules, under
# the 1.00-module cliff where a tag stops decoding against any non-white
# background (block_tags_selftest.test_quiet_zone_floor). The white does not
# have to end at the sticker edge to work, so a light-coloured block face may
# carry it; a dark one will not. Chosen deliberately 2026-08-04, to be settled
# on hardware. --fit-face returns to the size the arithmetic allows.
QUIET_ZONE_SQUEEZE_ALLOWED = True

CUT_MARGIN_MM = 3.0             # paper outside the quiet zone, to cut in

# How far outside the quiet-zone boundary the cut line is stroked. Small, but
# not zero: a line centred on the boundary puts half its width inside the white,
# and the white is the whole point. Consumes CUT_MARGIN_MM, never quiet zone.
CUT_LINE_CLEARANCE_MM = 0.25
LABEL_H_MM = 12.0
COL_GAP_MM = 22.0
ROW_GAP_MM = 7.0
RULER_GAP_MM = 9.0              # grid bottom to the ruler baseline
RULER_TICK_MM = 3.2             # major tick height; minor ticks are half

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
                 print_correction=1.0, paper="a4"):
    # print_correction draws the CONTENT larger inside a canvas that stays
    # exactly the paper size, by shrinking the logical page it is laid out on.
    # Everything below works in logical mm; physical mm on the sheet is logical
    # x print_correction. Same mechanism as print_zone_tags.CONTENT_SCALE,
    # which scales about the page centre instead.
    #
    # It used to scale px_per_mm alone, which also scaled the canvas -- the tag
    # kept the same FRACTION of the page (22.5/210), so every printer that maps
    # the image onto the paper printed the identical physical size and the
    # correction did nothing but change the raster resolution. Measured
    # 2026-08-04: 16/14 gave 19.6 mm and 1.0 gave 19.58 mm, the same tag from
    # settings 14% apart. See DEFAULT_PRINT_CORRECTION.
    paper_mm = PAPER_MM[paper]
    page_mm = (paper_mm[0] / print_correction, paper_mm[1] / print_correction)
    px_per_mm = dpi / 25.4 * print_correction

    def mm(v):
        return int(round(v * px_per_mm))

    def mm_to_px(x, y):
        return (mm(x), mm(y))

    page = np.full((mm(page_mm[1]), mm(page_mm[0])), 255, dtype=np.uint8)
    put = _text_writer(page, px_per_mm)
    dictionary = _dictionary()
    _assert_module_count(dictionary)

    # Squeezed to the face when the tag is too big to carry a full one, so the
    # cut square never exceeds the block face -- see QUIET_ZONE_SQUEEZE_ALLOWED.
    quiet_mm = tag_size_mm / MODULES_ACROSS * QUIET_ZONE_MODULES
    if QUIET_ZONE_SQUEEZE_ALLOWED and tag_size_mm + 2 * quiet_mm > face_size_mm:
        quiet_mm = max(0.0, (face_size_mm - tag_size_mm) / 2.0)

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
    title = "%s  36h11  id %d-%d" % (block_class.replace("_", " ").upper(),
                                     min(ids), max(ids))

    header_mm = TITLE_MM * 1.8 + ARROW_CLEARANCE_MM
    grid_total_mm = header_mm + grid_h_mm
    x0_mm = (page_mm[0] - grid_w_mm) / 2.0
    # Centre the whole block vertically too, so nothing lands near an edge --
    # page margins are where a fit-to-page shrink does its damage.
    y0_mm = max(MARGIN_MM + header_mm, (page_mm[1] - grid_total_mm) / 2.0 + header_mm)
    if x0_mm < MARGIN_MM:
        raise SystemExit(
            "a %.1f mm tag does not fit two across the page at this layout "
            "(%.1f mm needed, %.1f available at --print-correction %.4f)"
            % (tag_size_mm, grid_w_mm, page_mm[0] - 2 * MARGIN_MM,
               print_correction))

    put(title, x0_mm, y0_mm - header_mm + TITLE_MM, TITLE_MM, bold=True)

    for index, tag_id in enumerate(ids):
        info = bc.describe(tag_id)
        col, row = index % cols, index // cols          # fill ACROSS each row
        cx_mm = x0_mm + col * (cell_w_mm + COL_GAP_MM)
        cy_mm = y0_mm + row * (cell_h_mm + ROW_GAP_MM)

        _dashed_rect(page, mm_to_px(cx_mm, cy_mm),
                     mm_to_px(cx_mm + cell_w_mm, cy_mm + cell_h_mm),
                     dash_px=mm(1.5))

        # THE CUT LINE. Added 2026-08-06, after a session lost to its absence:
        # the dashed rectangle is the CELL (tag + quiet zone + CUT_MARGIN +
        # label), and with nothing else marked the obvious thing to cut to is
        # the black border of the tag itself. Doing that removes the entire
        # quiet zone and leaves the block's own surface as the only background
        # -- on an orange block that is a mid-tone, the quad's edge contrast
        # collapses, and decoding becomes a coin flip that depends on the
        # lighting. Two runs an hour apart, same pose and same sharpness, went
        # 2 block tags and 0.
        #
        # Drawn a hair OUTSIDE the quiet-zone boundary so the stroke itself
        # cannot eat into the white it exists to protect. Cutting on it gives
        # tag + 2 * quiet, which QUIET_ZONE_SQUEEZE_ALLOWED guarantees never
        # exceeds the face.
        cut_lo_mm = cx_mm + CUT_MARGIN_MM - CUT_LINE_CLEARANCE_MM
        cut_hi_mm = cut_lo_mm + tag_size_mm + 2 * quiet_mm + 2 * CUT_LINE_CLEARANCE_MM
        cv2.rectangle(page,
                      mm_to_px(cut_lo_mm, cy_mm + CUT_MARGIN_MM - CUT_LINE_CLEARANCE_MM),
                      mm_to_px(cut_hi_mm, cy_mm + CUT_MARGIN_MM - CUT_LINE_CLEARANCE_MM
                               + tag_size_mm + 2 * quiet_mm + 2 * CUT_LINE_CLEARANCE_MM),
                      0, max(1, mm(0.2)))

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

    # The calibration ruler, back on the sheet 2026-08-04 after three prints
    # were diagnosed by measuring a ~22 mm tag with a ruler that reads to 1 mm,
    # i.e. to 4%. Over 150 mm the same reading error is 0.7%. It is the same
    # instrument, and the same length, as the one on the zone sheets, and it
    # calibrates the same constant. Scaling the sheet scales the ruler with it,
    # which is exactly what makes it work.
    ruler_y_mm = y0_mm + grid_h_mm + RULER_GAP_MM
    ruler_x_mm = (page_mm[0] - RULER_LENGTH_MM) / 2.0
    bottom_mm = ruler_y_mm + RULER_TICK_MM + BODY_MM * 2.2

    if ruler_x_mm < MARGIN_MM:
        raise SystemExit(
            "the %.0f mm ruler does not fit the page at --print-correction %.4f"
            % (RULER_LENGTH_MM, print_correction))
    if bottom_mm > page_mm[1] - MARGIN_MM:
        raise SystemExit(
            "layout overflows the page by %.1f mm at a %.1f mm tag "
            "(--print-correction %.4f)"
            % (bottom_mm - page_mm[1] + MARGIN_MM, tag_size_mm, print_correction))

    cv2.line(page, mm_to_px(ruler_x_mm, ruler_y_mm),
             mm_to_px(ruler_x_mm + RULER_LENGTH_MM, ruler_y_mm),
             0, max(1, mm(0.3)), cv2.LINE_AA)
    for tick_mm in range(0, int(RULER_LENGTH_MM) + 1, 10):
        major = tick_mm % 50 == 0
        h_mm = RULER_TICK_MM if major else RULER_TICK_MM * 0.5
        x_mm = ruler_x_mm + tick_mm
        cv2.line(page, mm_to_px(x_mm, ruler_y_mm), mm_to_px(x_mm, ruler_y_mm + h_mm),
                 0, max(1, mm(0.3 if major else 0.2)), cv2.LINE_AA)
        if major:
            put("%d" % (tick_mm // 10), x_mm - 1.2,
                ruler_y_mm + h_mm + BODY_MM + 0.5, BODY_MM)
    put("%.0f mm nominal -- MEASURE ME. If it is not, rerun with "
        "--measured-ruler <mm>  [correction %.4f]"
        % (RULER_LENGTH_MM, print_correction),
        ruler_x_mm, ruler_y_mm - 1.8, BODY_MM)

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


def write_sheet(page, png_path, dpi):
    """Write the sheet as PNG *and* PDF, both carrying their physical size.

    This is the half of the sizing problem that no amount of --print-correction
    could reach. A PNG from cv2.imwrite has no pHYs chunk, i.e. no statement of
    how large it is in millimetres -- so "Actual Size" has nothing to be actual
    against and every print dialog falls back to fitting the pixels to the
    paper. That is why the tag came out the same 19.6 mm from two different
    corrections.

    The PDF is the one to print. Its page box is true A4 geometry (2480 px at
    300 dpi = 210.0 mm), so 100%/Actual Size is well defined and any remaining
    error is the printer's own scaling, which --print-correction can then
    cancel. Returns both paths.
    """
    from PIL import Image                      # only needed to write, not draw

    img = Image.fromarray(page)
    img.save(png_path, dpi=(dpi, dpi))
    pdf_path = os.path.splitext(png_path)[0] + ".pdf"
    img.save(pdf_path, "PDF", resolution=dpi)
    return png_path, pdf_path


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-dir", default=".",
                        help="directory for block_tags_<class>_<paper>.pdf/.png "
                             "(default %(default)s)")
    parser.add_argument("--paper", choices=sorted(PAPER_MM),
                        default=DEFAULT_PAPER,
                        help="page size to emit (default %(default)s, which is "
                             "what these printers feed and what the zone sheets "
                             "are authored on). Set it to what is actually in "
                             "the tray -- a page-size mismatch under fit-to-page "
                             "is its own scale error, on top of the one "
                             "--print-correction cancels.")
    parser.add_argument("--block", choices=list(bc.BLOCK_CLASSES) + ["both"],
                        default="both", help="which sheet(s) (default %(default)s)")
    parser.add_argument("--dpi", type=float, default=300.0)
    parser.add_argument("--face-size", type=float, default=DEFAULT_FACE_SIZE_M,
                        help="square face the tag must fit, METRES "
                             "(default %(default)s). Applies to EVERY sheet this "
                             "run emits, so two blocks of different sizes need "
                             "two runs. For a non-cubic block it is the SQUARE "
                             "top/bottom face, not its length.")
    parser.add_argument("--tag-size", type=float, default=None,
                        help="printed tag size in METRES. Default is DERIVED "
                             "from --face-size: the largest tag keeping a full "
                             "%.2f-module quiet zone, %.1f mm on a %.0f mm face."
                             % (QUIET_ZONE_MODULES,
                                bc.max_tag_size_for_face(DEFAULT_FACE_SIZE_M,
                                                         QUIET_ZONE_MODULES) * 1000,
                                DEFAULT_FACE_SIZE_M * 1000))
    parser.add_argument("--fit-face", action="store_true",
                        help="force the face-derived size even when --tag-size "
                             "is given. Already the default without --tag-size.")
    parser.add_argument("--print-correction", type=float,
                        default=DEFAULT_PRINT_CORRECTION,
                        help="scale the CONTENT on a page that stays the paper "
                             "size, to cancel the printer's shrink. Default "
                             "%(default).4f, measured on this chain. A PLAIN "
                             "NUMBER -- to recalibrate use --measured-ruler "
                             "instead of doing the arithmetic here.")
    parser.add_argument("--measured-ruler", type=float, default=None,
                        metavar="MM",
                        help="what the %.0f mm ruler on the last sheet actually "
                             "measured. Rescales --print-correction by "
                             "%.0f/MM and reprints, which is the entire "
                             "calibration loop." % (RULER_LENGTH_MM,
                                                    RULER_LENGTH_MM))
    parser.add_argument("--report", action="store_true",
                        help="print the px/module legibility table and exit "
                             "without drawing anything")
    parser.add_argument("--distance", type=float, default=ANGLED_VIEW_DISTANCE_M,
                        help="lens-to-block distance for --report, METRES "
                             "(default %(default)s = the angled survey pose)")
    args = parser.parse_args()

    if args.measured_ruler is not None:
        if args.measured_ruler <= 0:
            raise SystemExit("--measured-ruler must be a positive length in mm")
        was = args.print_correction
        args.print_correction *= RULER_LENGTH_MM / args.measured_ruler
        print("ruler measured %.2f mm against %.0f nominal (%.1f%% %s): "
              "--print-correction %.4f -> %.4f\n"
              % (args.measured_ruler, RULER_LENGTH_MM,
                 abs(args.measured_ruler / RULER_LENGTH_MM - 1.0) * 100,
                 "over" if args.measured_ruler > RULER_LENGTH_MM else "under",
                 was, args.print_correction))

    # Derived unless explicitly overridden, so --face-size alone is enough to
    # re-size everything for a different block.
    if args.tag_size is None or args.fit_face:
        tag_size_m = bc.max_tag_size_for_face(args.face_size, QUIET_ZONE_MODULES)
    else:
        tag_size_m = args.tag_size
    tag_size_mm = tag_size_m * 1000.0
    face_size_mm = args.face_size * 1000.0

    if tag_size_m > args.face_size:
        raise SystemExit(
            "--tag-size %.1f mm is larger than the %.1f mm face: it cannot be "
            "stuck on flat, let alone keep a quiet zone."
            % (tag_size_mm, face_size_mm))

    print(legibility_report(tag_size_mm, args.distance))

    # The quiet zone the face can actually carry at this tag size, stated
    # before anything is printed. Below 1.00 module a tag stops decoding
    # against a non-white background, and nothing about the sheet will say so.
    quiet_mm = min(tag_size_mm / MODULES_ACROSS * QUIET_ZONE_MODULES,
                   max(0.0, (face_size_mm - tag_size_mm) / 2.0))
    quiet_modules = quiet_mm / (tag_size_mm / MODULES_ACROSS)
    print("\nquiet zone on a %.0f mm face: %.2f mm a side = %.2f modules"
          % (face_size_mm, quiet_mm, quiet_modules))
    if quiet_modules < 1.0:
        print("  BELOW THE 1.00-MODULE CLIFF. Measured in "
              "block_tags_selftest.test_quiet_zone_floor:\n"
              "  at exactly 1.00 a 48 px-or-larger tag fails against ANY "
              "non-white background\n"
              "  and succeeds against white. The failure is total, not "
              "gradual. A light block\n"
              "  face may carry it anyway, since the white does not have to "
              "stop at the sticker\n"
              "  edge; a dark one will not. --fit-face gives the size that "
              "keeps the margin.")
    elif quiet_modules < QUIET_ZONE_MODULES:
        print("  under the %.2f modules this project uses, but above the "
              "1.00 cliff." % QUIET_ZONE_MODULES)
    print("\nCUT ON THE SOLID SQUARE, %.2f mm, NOT on the tag's black border "
          "and not on\nthe dashed rectangle. The dashed one is the cell "
          "(%.1f mm) and includes the\nlabel; the black border is the tag "
          "(%.1f mm) and cutting there throws the\nquiet zone away entirely, "
          "which on a dark block face stops it decoding."
          % (tag_size_mm + 2 * quiet_mm,
             tag_size_mm + 2 * quiet_mm + 2 * CUT_MARGIN_MM, tag_size_mm))

    if args.report:
        return

    classes = (list(bc.BLOCK_CLASSES) if args.block == "both" else [args.block])
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print()
    for block_class in classes:
        page = render_sheet(block_class, args.dpi, tag_size_mm, face_size_mm,
                            args.print_correction, args.paper)
        path = os.path.join(out_dir, "block_tags_%s_%s.png"
                            % (block_class, args.paper.upper()))
        _, pdf_path = write_sheet(page, path, args.dpi)
        ids = bc.tag_ids_for_block(block_class)
        print("wrote %s  (%dx%d px)" % (pdf_path, page.shape[1], page.shape[0]))
        print("       %s" % path)
        print("  %-6s ids %s"
              % (block_class,
                 ", ".join("%d=%s" % (i, bc.describe(i).face.upper())
                           for i in ids)))

    paper_mm = PAPER_MM[args.paper]
    print("\nPRINT THE PDF, not the PNG, and at 100%% / Actual Size / Scale 100"
          "\n-- NOT 'fit to page', 'shrink to fit' or 'scale to paper size'."
          "\nThe PDF is the only one of the two that states its own size in mm."
          "\nPage is %s, %.1f x %.1f mm: LOAD THAT PAPER. A page fitted to a "
          "different\nsize is the whole reason --print-correction exists."
          % (args.paper.upper(), paper_mm[0], paper_mm[1]))
    if args.print_correction != 1.0:
        print("\nContent drawn %.4fx oversized on a page that is still true %s, "
              "to cancel\na measured printer shrink. It is deliberately the "
              "wrong size on screen." % (args.print_correction,
                                         args.paper.upper()))
    print("\nTHEN MEASURE THE RULER, not a tag: %.0f mm over a %.0f mm baseline "
          "reads to\n0.7%%, the same 1 mm error on a %.1f mm tag reads to %.0f%%. "
          "If it is not %.0f mm,\nre-run with the SAME paper and dialog "
          "settings, adding"
          "\n  --measured-ruler <what you measured, mm>"
          "\nwhich redoes the correction for you. A correct sheet then has "
          "%.1f mm tags."
          % (RULER_LENGTH_MM, RULER_LENGTH_MM, tag_size_mm,
             100.0 / tag_size_mm, RULER_LENGTH_MM, tag_size_mm))

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
