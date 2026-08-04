#!/usr/bin/env python3
"""The block-tag id scheme: which AprilTag id means which face of which block.

PURE PYTHON. No ROS, no OpenCV. Imported by the printer (print_block_tags.py),
the Pi-side detector (block_detector_node.py) and anything on mars that has to
turn an id back into "the cuboid's left face" -- so there is exactly one
definition of the scheme and the printed paper cannot drift from the code.

------------------------------------------------------------------------------
THE SCHEME
------------------------------------------------------------------------------
Six tags per block: one TOP, one BOTTOM, and FOUR SIDES, every one a distinct
id. Zone tags own 0-7 (zone_vision.PICKUP_TAG_IDS / PLACE_TAG_IDS), so blocks
start at 8:

    cube    TOP  8   BOTTOM  9   SIDE0 10  SIDE1 11  SIDE2 12  SIDE3 13
    cuboid  TOP 14   BOTTOM 15   SIDE0 16  SIDE1 17  SIDE2 18  SIDE3 19

WHY THE FOUR SIDES ARE NOT ONE SHARED ID, which is the obvious economy:

  1. It hands over BLOCK YAW for free, and yaw is a known-broken signal in this
     project. APRIL_TAGS_DEV.md, "Circle vs square is unstable": a 30 mm square
     block classified as `circle` in two stills of three because its fill_ratio
     sits on CIRCLE_FILL_MAX, symmetry then reads 0, and zone_vision.py:942
     forces the yaw to 0.0 during fusion -- so the jaws get driven at the
     block's 52 mm diagonal. Reading "that is SIDE2" off a decoded id gives the
     yaw with no contour, no fill ratio and no classification step, and it
     cannot be destroyed downstream because it is an integer, not an angle.

  2. Duplicate ids in one frame are SILENTLY DROPPED. zone_vision.detect_all_
     tags() returns dict(_aruco_detect(gray)), so two tags sharing an id
     collapse to one -- verified, not inferred. Four same-id faces of one block
     are rarely both visible, but stage 0 of the stacked-block plan puts several
     blocks in the zone at once, and that is where a shared id turns into a
     confident wrong answer instead of an error. detect_block_tags() below goes
     through the list form for exactly this reason.

  3. Occlusion. The gripper hides part of every reachable view (APRIL_TAGS_DEV.md
     is explicit that only 2 of the 4 ZONE tags survive a real hover), and a
     neighbouring block hides more. Four distinct sides means any one of them
     still identifies the block and still fixes its yaw.

------------------------------------------------------------------------------
WHICH FACE IS SIDE 0
------------------------------------------------------------------------------
Defined so it can be checked by hand while sticking tags on, with no reference
to the code:

    Stand the block with its TOP tag upward and the TOP tag's arrow pointing
    AWAY from you. SIDE0 is the far face -- the one the arrow points at. Then
    go counter-clockwise seen FROM ABOVE: SIDE1 left, SIDE2 near, SIDE3 right.

In the block's own frame that is SIDE0 facing +Y, SIDE1 -X, SIDE2 -Y, SIDE3 +X,
i.e. outward bearing 90 + 90*k degrees. Counter-clockwise-from-above matches
zone_vision.ZONE_CORNER_SIGNS, so the two schemes never need reconciling.

------------------------------------------------------------------------------
THE CUBOID
------------------------------------------------------------------------------
Square faces are TOP and BOTTOM; the four long faces are the SIDES. So the
cuboid's cross-section is square and all four of its side tags are the same
size as each other -- the tag is limited by the SQUARE's side, not by the
cuboid's length.
"""
import math
from typing import NamedTuple, Optional

# ---------------------------------------------------------------------------
# The scheme
# ---------------------------------------------------------------------------
# Zone tags occupy 0-7. Starting at 8 keeps a single detectMarkers() pass able
# to see zone and block tags together with no id ambiguity, which is why both
# stay in DICT_APRILTAG_36h11 rather than block tags moving to a coarser family.
BLOCK_TAG_ID_BASE = 8

BLOCK_CLASSES = ("cube", "cuboid")

# Order matters: it IS the id assignment. Do not reorder without reprinting.
FACE_ORDER = ("top", "bottom", "side0", "side1", "side2", "side3")
FACES_PER_BLOCK = len(FACE_ORDER)

# Outward bearing of each side face in the BLOCK's own frame, degrees CCW from
# block +X. See "WHICH FACE IS SIDE 0" above.
SIDE_BEARING_DEG = (90.0, 180.0, 270.0, 0.0)


class BlockFace(NamedTuple):
    """What a decoded block-tag id means."""
    tag_id: int
    block_class: str          # "cube" | "cuboid"
    face: str                 # "top" | "bottom" | "side0".."side3"
    kind: str                 # "top" | "bottom" | "side"
    side_index: Optional[int] # 0-3 for a side face, None otherwise

    @property
    def label(self) -> str:
        """Short human string, e.g. 'cuboid SIDE2'. What the logs print."""
        return "%s %s" % (self.block_class, self.face.upper())

    @property
    def is_mat_parallel(self) -> bool:
        """True for TOP/BOTTOM only.

        The tag-scale height trick (height_from_scale) and any mat-plane
        homography projection are valid ONLY for a tag lying parallel to the
        mat. A side tag is perpendicular to it, so projecting one through the
        zone homography returns a number that looks like a position and is not.
        """
        return self.kind in ("top", "bottom")


def _build_tables():
    forward, reverse = {}, {}
    for class_index, block_class in enumerate(BLOCK_CLASSES):
        for face_index, face in enumerate(FACE_ORDER):
            tag_id = (BLOCK_TAG_ID_BASE
                      + class_index * FACES_PER_BLOCK
                      + face_index)
            kind = "side" if face.startswith("side") else face
            side_index = int(face[4:]) if kind == "side" else None
            forward[tag_id] = BlockFace(tag_id, block_class, face, kind,
                                        side_index)
            reverse[(block_class, face)] = tag_id
    return forward, reverse


_BY_ID, _BY_FACE = _build_tables()

BLOCK_TAG_IDS = tuple(sorted(_BY_ID))
BLOCK_TAG_ID_MIN = BLOCK_TAG_IDS[0]
BLOCK_TAG_ID_MAX = BLOCK_TAG_IDS[-1]


# ---------------------------------------------------------------------------
# Lookups
# ---------------------------------------------------------------------------
def is_block_tag(tag_id) -> bool:
    """True for 8-19. False for zone tags and for anything unrecognised."""
    return int(tag_id) in _BY_ID


def describe(tag_id) -> Optional[BlockFace]:
    """BlockFace for a block-tag id, or None if it is not one.

    Returns None rather than raising: a frame legitimately contains zone tags
    and stray ids, and callers filter rather than handle exceptions.
    """
    return _BY_ID.get(int(tag_id))


def tag_id_for(block_class: str, face: str) -> int:
    """Inverse of describe(). Raises on an unknown combination."""
    try:
        return _BY_FACE[(block_class, face)]
    except KeyError:
        raise ValueError(
            "no tag for %r %r; classes are %s and faces are %s"
            % (block_class, face, list(BLOCK_CLASSES), list(FACE_ORDER)))


def tag_ids_for_block(block_class: str):
    """The six ids of one block class, in FACE_ORDER."""
    return tuple(tag_id_for(block_class, face) for face in FACE_ORDER)


def label(tag_id) -> str:
    """'cube SIDE2', or 'zone tag 3' / 'unknown tag 42' so log lines never lie."""
    face = describe(tag_id)
    if face is not None:
        return face.label
    tag_id = int(tag_id)
    if 0 <= tag_id <= 7:
        return "zone tag %d" % tag_id
    return "unknown tag %d" % tag_id


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def block_yaw_from_side(side_index: int, face_bearing_deg: float) -> float:
    """Block yaw in degrees, from seeing SIDE<side_index> face a known bearing.

    face_bearing_deg is the outward normal of the observed face measured in
    whatever frame you want the answer in (world or zone-local). Because the
    side index names the face in the block's own frame, the difference is the
    block's rotation:

        block_yaw = face_bearing - SIDE_BEARING_DEG[side_index]

    Result wrapped to [0, 360). Unlike a yaw inferred from a footprint contour
    this is unambiguous -- there is no modulo-90 fold and no symmetry order to
    apply, because the id already says which of the four faces it is.
    """
    if side_index not in (0, 1, 2, 3):
        raise ValueError("side_index must be 0-3, got %r" % (side_index,))
    return (face_bearing_deg - SIDE_BEARING_DEG[side_index]) % 360.0


def height_from_scale(tag_scale_px_per_m: float,
                      mat_scale_px_per_m: float,
                      lens_height_m: float,
                      view_tilt_deg: float = 0.0) -> float:
    """Height of a mat-PARALLEL tag above the mat, from how much bigger it reads.

    A tag h above the mat plane sits (d - h) from a lens d above it, so it is
    magnified by d / (d - h). Inverting, with scale expressed as px per metre so
    the tag's own printed size cancels:

        h = d * (1 - mat_scale / tag_scale)

    This is the same relation tag_pick_place.py already uses in reverse to catch
    a hover that came out too low (APRIL_TAGS_DEV.md, "px/m is a free height
    gauge"), and it needs no camera intrinsics -- which matters because there
    are none anywhere in this repo, so solvePnP is not available.

    view_tilt_deg corrects for an oblique view. Raising a tag by h shortens the
    range along the optical axis by h*cos(tilt), not by h, so a naive reading at
    the 36 deg angled survey pose UNDER-reports height by about 19%.

    ONLY VALID FOR TOP/BOTTOM TAGS -- check BlockFace.is_mat_parallel first. A
    side tag is perpendicular to the mat and its apparent size is set by the
    viewing angle, not by mat-plane parallax.

    The result is returned SIGNED. A negative height is not clamped away: it
    means the tag read smaller than the mat plane predicts, which cannot happen
    physically and is therefore evidence that lens_height_m is wrong or the tag
    was mis-measured. Swallowing it would hide the one number that says so.
    """
    if tag_scale_px_per_m <= 0 or mat_scale_px_per_m <= 0:
        raise ValueError("scales must be positive px/m")
    naive = lens_height_m * (1.0 - mat_scale_px_per_m / tag_scale_px_per_m)
    return naive / math.cos(math.radians(view_tilt_deg))


# White border each side of the black square, in MODULES. The AprilTag spec
# says one module; one module is NOT enough here, and that is measured rather
# than assumed. Sweeping quiet zone against background grey (block_tags_selftest
# .test_quiet_zone_floor) gives a clean cliff: at exactly 1.00 module a 48 px or
# larger tag fails to decode against ANY non-white background and succeeds
# against white, while 1.25 and above decode at every background and size tried.
# The failure is total, not gradual, so the margin here is deliberate but small
# -- quiet zone is bought with tag size, and tag size is the binding constraint
# at the angled survey pose.
QUIET_ZONE_MODULES = 1.25


def max_tag_size_for_face(face_size_m: float,
                          quiet_modules: float = QUIET_ZONE_MODULES,
                          round_down_to_m: float = 0.0005) -> float:
    """Largest 36h11 tag that fits a square face of face_size_m, with quiet zone.

    A 36h11 tag rendered by OpenCV is EIGHT modules across the black square
    (6x6 of data plus a one-module black border) -- measured, not assumed; see
    print_block_tags._assert_module_count. So one module is tag/8, and requiring
    quiet_modules of white on each side gives

        tag + 2 * quiet_modules * tag/8 = face   ->   tag = face / (1 + q/4)

    Rounded DOWN to round_down_to_m, default 0.5 mm, because the printed size
    has to be measured by hand against a ruler and confirming "22.5" is a great
    deal easier than confirming "22.86". Rounding down also spends the remainder
    on quiet zone, which is the safe direction.

    Note that print_tag_sheet.py:67 says a module is tag_size/10; that comment
    is wrong, and it makes its 4 mm quiet zone look more generous than it is.
    """
    if face_size_m <= 0:
        raise ValueError("face_size_m must be positive")
    exact = face_size_m / (1.0 + quiet_modules / 4.0)
    if round_down_to_m > 0:
        exact = math.floor(exact / round_down_to_m) * round_down_to_m
    return exact


if __name__ == "__main__":
    print("block tag scheme -- %d tags, ids %d-%d\n"
          % (len(BLOCK_TAG_IDS), BLOCK_TAG_ID_MIN, BLOCK_TAG_ID_MAX))
    for block_class in BLOCK_CLASSES:
        for face in FACE_ORDER:
            tag_id = tag_id_for(block_class, face)
            info = describe(tag_id)
            assert info.block_class == block_class and info.face == face
            bearing = ("" if info.side_index is None
                       else "  outward bearing %5.1f deg"
                            % SIDE_BEARING_DEG[info.side_index])
            print("  id %2d  %-14s%s" % (tag_id, info.label, bearing))
        print()

    assert not is_block_tag(7) and is_block_tag(8) and is_block_tag(19)
    assert not is_block_tag(20)
    assert describe(3) is None
    assert label(3) == "zone tag 3"

    # A cube whose SIDE2 is observed facing world bearing 0 deg is turned 90.
    assert abs(block_yaw_from_side(2, 0.0) - 90.0) < 1e-9

    print("max tag on a 30.0 mm face: %.1f mm (1 module quiet zone)"
          % (max_tag_size_for_face(0.030) * 1000))
    print("height of a tag reading 2891 px/m against a 2466 px/m mat, lens 0.2235 m:"
          "\n  top-down %.1f mm   at 36 deg tilt %.1f mm"
          % (height_from_scale(2891, 2466, 0.2235) * 1000,
             height_from_scale(2891, 2466, 0.2235, 36.0) * 1000))
    print("\nself-check OK")
