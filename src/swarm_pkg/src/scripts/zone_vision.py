#!/usr/bin/env python3
"""Locate a block inside an AprilTag-marked zone, from a single still image.

PURE OPENCV. No rclpy, no ROS message types, no TF. Everything here runs
identically on the Pi (in block_detector_node.py, in production) and on mars
(in zone_view.py, against saved stills). That is deliberate: threshold tuning
against a 20-frame corpus takes seconds offline and a robot session on hardware.

------------------------------------------------------------------------------
WHY A HOMOGRAPHY, AND WHAT IT BUYS
------------------------------------------------------------------------------
The zone is a square whose four vertices carry the CENTRES of four 1in AprilTags.
Those vertices are at known coordinates in a "zone-local" frame. So the tags give
a plane-to-plane homography between image pixels and zone-local metres.

The consequence is the entire point of this design: the block's position in the
zone falls out of that homography and depends on NOTHING ELSE. Not the arm's
actual position when the still was taken, not the camera's pose on the flange,
not the camera intrinsics. All three cancel -- they decide whether the tags are
in frame, not where the block is once they are.

That matters here specifically. PROJECT_CONTEXT.md documents that this arm's
absolute positioning is not trustworthy (IK_POS_TOLERANCE = 0.02, a residual
grasp tilt that is mechanical and not tunable, and a controller with no
constraints: block, so "Goal reached, success!" is inferred from elapsed time).
Any design that derived block position from where the arm THINKS it is would
inherit every bit of that. This one doesn't.

What it does NOT give: the zone's own pose in the world. Nothing here measures
that -- it is surveyed and passed in by the caller. So a block's world position
is only ever as good as the zone survey, and the honest split is:

    world_block = zone_survey (caller's number)  o  block_in_zone (measured here)

which is why detections carry BOTH frames. A bad zone-local reading is a vision
bug; a good zone-local reading with a missed grasp is a survey or arm problem.

------------------------------------------------------------------------------
WHY TAG CORNERS AND NOT TAG CENTRES
------------------------------------------------------------------------------
A homography has 8 degrees of freedom and four point correspondences determine
it EXACTLY. Fit it from the four tag centres and the reprojection residual is
identically zero -- for a good fit and a garbage one alike. The health metric
would be a constant.

So the fit uses all four corners of every tag: 16 correspondences for four tags,
12 for three. Over-determined, so the residual finally carries information, and
homography_rms becomes the number that says whether to trust the answer. A bad
homography otherwise fails silently and returns a confident wrong position,
which is the worst failure mode available to this feature.

It also makes the three-tag case fall out for free -- 12 correspondences is
still plenty -- with no special-case geometry to reconstruct a missing corner.
Two tags is refused: 8 points would be enough in principle, but they span a thin
band across the image and the fit is badly conditioned along the other axis.

------------------------------------------------------------------------------
ASSUMPTION THAT MUST BE CHECKED AGAINST THE CORPUS
------------------------------------------------------------------------------
All four tags are printed in the SAME orientation as each other and square to
the zone (each tag's "up" points along zone +Y). The corner ordering below
depends on it. If the printed sheet has them rotated per-corner, TAG_CORNER_
OFFSETS needs a per-tag rotation and the homography residual will be the thing
that tells you -- it will be large and roughly tag-sized.
"""
import math

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Zone geometry
# ---------------------------------------------------------------------------
# Side of the square joining the four TAG CENTRES. Not the tag size, and not
# whatever outline is printed on the mat. 4 in.
DEFAULT_ZONE_SIZE = 0.1016      # m
# Printed side length of one AprilTag's black border. 1 in.
DEFAULT_TAG_SIZE = 0.0254       # m

# Which tag id sits at which vertex, and in what order. Index 0..3 map to the
# ZONE_CORNER_SIGNS below, so this list is what physically ties an id to a
# corner of the mat -- get it wrong and the zone frame comes out rotated or
# mirrored, which the homography residual will NOT catch (a mirrored fit is
# still a perfect fit). Verify against a still with a block at a known corner.
PICKUP_TAG_IDS = (0, 1, 2, 3)
PLACE_TAG_IDS = (4, 5, 6, 7)

# Vertex order for the ids above, counter-clockwise from the -X/-Y corner, in
# units of half the zone size.
ZONE_CORNER_SIGNS = ((-1, -1), (+1, -1), (+1, +1), (-1, +1))

# Offsets of one tag's four corners from its own centre, in units of half the
# tag size, in the order cv2.aruco returns them: top-left, top-right,
# bottom-right, bottom-left in the MARKER's own frame. Zone +Y is "up".
TAG_CORNER_OFFSETS = ((-1, +1), (+1, +1), (+1, -1), (-1, -1))

MIN_TAGS = 3                    # see module docstring; 2 is refused, not degraded

# ---------------------------------------------------------------------------
# Detection thresholds
# ---------------------------------------------------------------------------
# These are the numbers the still-image corpus exists to tune. They are STARTING
# POINTS, not measurements -- APRIL_TAGS.md's Measurements table is where the
# tuned values get recorded once there are frames to tune against.

# Reject a homography whose corner reprojection RMS exceeds this. Sized to be
# generous initially so the corpus can show what "normal" looks like before it
# is tightened; a value that rejects nothing is at least visible in the logs,
# whereas one that rejects everything looks like a hardware fault.
MAX_HOMOGRAPHY_RMS_PX = 6.0

# Ignore contours smaller than this fraction of the zone area -- specks, mat
# texture, printed markings.
MIN_BLOCK_AREA_FRAC = 0.004     # ~0.4% of a 4in square = a 6mm speck
# ...and larger than this, which means the segmentation leaked out of the zone
# and grabbed the mat itself rather than a block on it.
MAX_BLOCK_AREA_FRAC = 0.60

# A detected object's CENTRE must land this far inside the zone edge. Only the
# centre is tested, so a block whose corner overhangs the boundary is still
# measured at full size -- see build_masks() on why clipping is not used here.
ZONE_INTERIOR_MARGIN = 0.004    # m
# Grow each tag's quad by this before painting it out; the printed white quiet
# zone around a tag reads as an edge otherwise.
TAG_EXCLUSION_MARGIN = 0.004    # m

# Shape classification. fill_ratio = contour area / its minAreaRect area:
# a rectangle fills its own bounding rect (~1.0), a circle fills pi/4 (~0.785).
SQUARE_ASPECT_TOL = 0.88        # short/long above this counts as "not elongated"
CIRCLE_FILL_MAX = 0.86          # below this, and un-elongated, reads as round
RECT_FILL_MIN = 0.80            # below this for an elongated blob: shape unknown


class ZoneSpec:
    """Where a zone is and how it is marked.

    world_x/world_y/world_yaw/world_z are the SURVEYED pose of the zone centre,
    supplied by the caller. Nothing in this module measures them -- see the
    module docstring on what the homography does and does not give you.
    """

    def __init__(self, tag_ids, world_x=0.0, world_y=0.0, world_yaw=0.0,
                 world_z=0.0, zone_size=DEFAULT_ZONE_SIZE,
                 tag_size=DEFAULT_TAG_SIZE):
        if len(tag_ids) != 4:
            raise ValueError("a zone is marked by exactly 4 tags, got %d" % len(tag_ids))
        self.tag_ids = tuple(int(t) for t in tag_ids)
        self.world_x = float(world_x)
        self.world_y = float(world_y)
        self.world_yaw = float(world_yaw)
        self.world_z = float(world_z)
        self.zone_size = float(zone_size)
        self.tag_size = float(tag_size)

    def tag_corner_targets(self):
        """{tag_id: 4x2 array of that tag's corners in zone-local metres}.

        Corner order matches cv2.aruco's, so this pairs elementwise with what
        detect_tags returns -- no re-ordering at the call site.
        """
        half_zone = self.zone_size / 2.0
        half_tag = self.tag_size / 2.0
        targets = {}
        for tag_id, (sx, sy) in zip(self.tag_ids, ZONE_CORNER_SIGNS):
            cx, cy = sx * half_zone, sy * half_zone
            targets[tag_id] = np.array(
                [[cx + ox * half_tag, cy + oy * half_tag]
                 for ox, oy in TAG_CORNER_OFFSETS], dtype=np.float64)
        return targets

    def zone_to_world(self, zx, zy):
        c, s = math.cos(self.world_yaw), math.sin(self.world_yaw)
        return (self.world_x + c * zx - s * zy,
                self.world_y + s * zx + c * zy)

    def zone_yaw_to_world(self, zyaw):
        return wrap_angle(zyaw + self.world_yaw)


def zone_for(name, **kwargs):
    """ZoneSpec for the well-known zone names used by the DetectBlock service."""
    ids = {"pickup": PICKUP_TAG_IDS, "place": PLACE_TAG_IDS}.get(name)
    if ids is None:
        raise ValueError("unknown zone %r (expected 'pickup' or 'place')" % (name,))
    return ZoneSpec(ids, **kwargs)


def wrap_angle(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Detection:
    """One object found in the zone. Mirrors swarm_interfaces/BlockDetection."""

    def __init__(self, zx, zy, zyaw, width, length, shape, symmetry,
                 fill_ratio, area_px, box_px):
        self.zx = zx
        self.zy = zy
        self.zyaw = zyaw
        self.width = width          # SHORT footprint dimension, m
        self.length = length        # LONG footprint dimension, m
        self.shape = shape
        self.symmetry = symmetry
        self.fill_ratio = fill_ratio
        self.area_px = area_px
        self.box_px = box_px        # 4x2 pixel corners, for the debug overlay

    def world_pose(self, zone):
        x, y = zone.zone_to_world(self.zx, self.zy)
        return x, y, zone.zone_yaw_to_world(self.zyaw)

    def __repr__(self):
        return ("Detection(zone=(%.4f, %.4f) yaw=%.1fdeg %s %.1fx%.1fmm "
                "sym=%d fill=%.2f)"
                % (self.zx, self.zy, math.degrees(self.zyaw), self.shape,
                   self.width * 1000.0, self.length * 1000.0,
                   self.symmetry, self.fill_ratio))


class ZoneResult:
    """Everything one still frame produced."""

    def __init__(self):
        self.success = False
        self.message = ""
        self.tag_ids = []
        self.tag_corners = {}       # {id: 4x2 px} -- for the overlay
        self.homography_rms = 0.0
        self.scale_px_per_m = 0.0
        self.camera_zx = 0.0        # zone-local point under the image centre
        self.camera_zy = 0.0        # -- see camera_in_zone()
        self.blocks = []
        self.H_zone_to_px = None
        self.H_px_to_zone = None
        self.mask = None            # search mask, for the overlay
        self.accept_mask = None     # centre-acceptance mask, for the overlay
        self.flat_gray = None       # grayscale with the tags painted out

    @property
    def tags_seen(self):
        return len(self.tag_ids)


# ---------------------------------------------------------------------------
# AprilTag detection
# ---------------------------------------------------------------------------
# OpenCV moved the aruco API in 4.7: Dictionary_get/DetectorParameters_create/
# detectMarkers became getPredefinedDictionary/DetectorParameters/ArucoDetector,
# and the old spelling was later removed. mars runs 4.6 and the Pi runs 4.2, so
# BOTH machines currently want the legacy path -- but pinning to it would break
# the moment either machine is updated, and this shim costs eight lines.
_ARUCO_CACHE = {}


def _aruco_detect(gray):
    """[(tag_id, 4x2 float32 corners)], newest OpenCV API or legacy."""
    if "detect" not in _ARUCO_CACHE:
        _ARUCO_CACHE["detect"] = _build_aruco_detector()
    return _ARUCO_CACHE["detect"](gray)


def _build_aruco_detector():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "this OpenCV build has no aruco module. On the Pi, python3-opencv "
            "normally ships contrib; if it does not, add pupil-apriltags to "
            "pi_setup/requirements.txt and add a branch here.")

    if hasattr(cv2.aruco, "ArucoDetector"):        # OpenCV >= 4.7
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        def detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return _pack_aruco(corners, ids)
        return detect

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters_create()
    # Sub-pixel corner refinement. The whole accuracy argument rests on the tag
    # corners, and integer-pixel corners at ~0.2 m put a visible floor under the
    # homography residual for free.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def detect(gray):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        return _pack_aruco(corners, ids)
    return detect


def _pack_aruco(corners, ids):
    if ids is None or len(ids) == 0:
        return []
    return [(int(i), c.reshape(4, 2).astype(np.float64))
            for i, c in zip(ids.flatten(), corners)]


def detect_tags(gray, zone):
    """{tag_id: 4x2 px corners} for the tags belonging to this zone.

    Tags from another zone in the same frame are dropped, not an error -- the
    two zones may well both be in shot. The caller sees which ids were used.
    """
    wanted = set(zone.tag_ids)
    found = {}
    for tag_id, corners in _aruco_detect(gray):
        if tag_id in wanted:
            found[tag_id] = corners
    return found


# ---------------------------------------------------------------------------
# Homography
# ---------------------------------------------------------------------------
def fit_homography(tag_corners_px, zone):
    """(H_zone_to_px, rms_px) from every corner of every visible tag.

    Least-squares over 4*n correspondences, so the residual is meaningful --
    see the module docstring on why centres alone would not be.
    """
    targets = zone.tag_corner_targets()
    src, dst = [], []
    for tag_id in sorted(tag_corners_px):
        src.append(targets[tag_id])
        dst.append(tag_corners_px[tag_id])
    src = np.concatenate(src, axis=0)
    dst = np.concatenate(dst, axis=0)

    # method=0 is a plain least-squares fit over all points. Deliberately NOT
    # RANSAC: with 12-16 points that all matter, an outlier here means a
    # misdetected tag, and silently discarding it would hide exactly the fault
    # homography_rms exists to expose.
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None:
        return None, float("inf")

    projected = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected - dst) ** 2, axis=1))))
    return H, rms


def _scale_at_centre(H_zone_to_px, zone):
    """Local px-per-metre at the zone centre. Diagnostic only."""
    probe = np.array([[[0.0, 0.0]], [[0.01, 0.0]]], dtype=np.float64)
    pts = cv2.perspectiveTransform(probe, H_zone_to_px).reshape(2, 2)
    return float(np.linalg.norm(pts[1] - pts[0]) / 0.01)


def camera_in_zone(H_px_to_zone, width_px, height_px):
    """Zone-local (x, y) that the centre of the image looks at.

    THIS IS THE ONLY THING HERE THAT MEASURES THE ARM, and it is worth being
    precise about why, because the obvious reading of "detect again to check the
    move worked" is wrong.

    A block's zone-local position comes from the tags and is completely
    independent of where the arm is -- move the arm and re-detect, and you get
    the same answer, because the same physical block is still in the same place
    on the same mat. Re-detecting the BLOCK therefore verifies nothing about the
    arm. It re-measures the block.

    The camera's own position is different. The image centre corresponds to a
    specific point on the zone plane, and mapping it back through the homography
    says where the camera actually ended up over the mat -- an external
    measurement of the arm's position that owes nothing to its encoders, and so
    is blind to exactly the gravity droop and dead-zone effects that make the
    encoders untrustworthy (see TESTS.md, which names "external metrology" as
    the fallback if the residuals turn out not to be correctable).

    TWO CAVEATS, both systematic and both constant for a given pose:

    1. The image centre is used as the principal point. Without an intrinsic
       calibration the true principal point can sit a few percent off centre.
    2. The optical axis is assumed perpendicular to the zone plane. It is not
       exactly -- the URDF puts it 0.5 deg off at the grasp orientation, and
       the arm has a documented mechanical tilt on top of that. A 3 deg tilt at
       0.20 m puts this ~10 mm out.

    Both are OFFSETS, not noise. So the absolute number should be treated as
    uncalibrated, while the CHANGE between two hovers at the same orientation is
    accurate -- and it is the change that a correction step actually needs.
    """
    centre = px_to_zone(H_px_to_zone, [(width_px / 2.0, height_px / 2.0)])[0]
    return float(centre[0]), float(centre[1])


def zone_to_px(H_zone_to_px, pts_zone):
    pts = np.asarray(pts_zone, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_zone_to_px).reshape(-1, 2)


def px_to_zone(H_px_to_zone, pts_px):
    pts = np.asarray(pts_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_px_to_zone).reshape(-1, 2)


# ---------------------------------------------------------------------------
# Search mask
# ---------------------------------------------------------------------------
def zone_quad_px(H_zone_to_px, zone, inset=0.0):
    half = zone.zone_size / 2.0 - inset
    return zone_to_px(H_zone_to_px, [(-half, -half), (half, -half),
                                     (half, half), (-half, half)])


def build_masks(shape_hw, H_zone_to_px, zone):
    """(search_mask, accept_mask) in pixels.

    search_mask  where edges may be looked for at all -- the zone square.
    accept_mask  where a detected object's CENTRE must lie -- the same square
                 inset by ZONE_INTERIOR_MARGIN.

    Two masks rather than one, because clipping and accepting want different
    shapes. Clip a block against the inset square and any block whose corner
    overhangs the boundary comes back truncated: wrong size, wrong centre, wrong
    yaw. Testing only the centre against the inset square rejects the mat's own
    printed outline (whose centre is inside, but whose area is rejected anyway)
    without mutilating a legitimately edge-adjacent block.
    """
    search = np.zeros(shape_hw, dtype=np.uint8)
    cv2.fillConvexPoly(search, zone_quad_px(H_zone_to_px, zone).astype(np.int32), 255)

    accept = np.zeros(shape_hw, dtype=np.uint8)
    cv2.fillConvexPoly(
        accept,
        zone_quad_px(H_zone_to_px, zone, ZONE_INTERIOR_MARGIN).astype(np.int32), 255)
    return search, accept


def flatten_tags(gray, H_zone_to_px, zone, tag_corners_px, fill_value):
    """Paint the tags out of the image so their borders stop being edges.

    The tags sit ON the zone vertices, so half of each lies inside the search
    area, and their black-on-white borders are the strongest edges in the frame.
    Left in, every detection is a tag.

    The obvious fix -- punch holes in the binary image where the tags are -- was
    tried first and is WRONG. A block sitting in a corner of the zone genuinely
    overlaps the tag region, so the hole bites a chunk out of the block's
    contour: measured 26-37 mm for a 30 mm block, with fill_ratio collapsing to
    ~0.11 because the contour was no longer closed. Painting the tag over at
    the GRAYSCALE stage instead removes the tag's edges while leaving every
    edge belonging to a neighbouring block intact.

    fill_value should be the mat's own median brightness, so the painted quad
    does not itself become a step edge.
    """
    flat = gray.copy()
    targets = zone.tag_corner_targets()
    grow = 1.0 + 2.0 * TAG_EXCLUSION_MARGIN / zone.tag_size
    for tag_id in tag_corners_px:
        # Grow in ZONE space, where "4 mm" means something, rather than dilating
        # pixels -- perspective foreshortening makes a fixed pixel margin cover
        # different physical distances at the near and far corners of the mat.
        quad = targets[tag_id]
        centre = quad.mean(axis=0)
        grown = centre + (quad - centre) * grow
        cv2.fillConvexPoly(flat, zone_to_px(H_zone_to_px, grown).astype(np.int32),
                           float(fill_value))
    return flat


# ---------------------------------------------------------------------------
# Block segmentation
# ---------------------------------------------------------------------------
def _segment(gray, search_mask, method):
    """Binary image of candidate objects inside search_mask."""
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    inside = blurred[search_mask > 0]
    if inside.size == 0:
        return np.zeros_like(gray)

    if method == "otsu":
        # Otsu picks its threshold from the histogram, so it must only see the
        # zone interior -- show it the mat outside the zone too and the split
        # lands between "zone" and "not zone" rather than "block" and "mat".
        # THRESH_BINARY_INV because blocks are assumed darker than the mat; if
        # the corpus says otherwise, that assumption is this line.
        level, _ = cv2.threshold(inside, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, binary = cv2.threshold(blurred, level, 255, cv2.THRESH_BINARY_INV)
    else:
        # Canny thresholds taken from the masked median rather than fixed: no
        # absolute brightness assumption, so a lighting change that would move a
        # hardcoded pair leaves this alone. Makes no light-vs-dark assumption
        # either, which is why it is the default.
        median = float(np.median(inside))
        lo = int(max(0, 0.66 * median))
        hi = int(min(255, 1.33 * median))
        binary = cv2.Canny(blurred, lo, hi)
        # One dilation, not two. Each iteration fattens the outline by ~1 px on
        # every side, and since the OUTER boundary of that ring is what gets
        # measured, every iteration is added directly to the reported block
        # size. Two cost ~1 mm of systematic oversize at this working distance.
        binary = cv2.dilate(binary, np.ones((3, 3), np.uint8), iterations=1)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    return cv2.bitwise_and(binary, binary, mask=search_mask)


def _classify(width, length, fill_ratio):
    """(shape, symmetry). See BlockDetection.msg for what symmetry means."""
    if length <= 0.0:
        return "unknown", 1
    aspect = width / length

    if aspect >= SQUARE_ASPECT_TOL:
        if fill_ratio < CIRCLE_FILL_MAX:
            return "circle", 0          # yaw is meaningless, grab at any angle
        return "square", 4              # 90 deg symmetry
    if fill_ratio >= RECT_FILL_MIN:
        return "rect", 2                # 180 deg symmetry
    # Elongated but not filling its bounding box: an L, a wedge, two touching
    # blocks segmented as one. Reported rather than dropped, but symmetry 1 so
    # the caller does not rotate the wrist on a yaw it should not trust.
    return "unknown", 1


def find_blocks(gray, H_zone_to_px, H_px_to_zone, zone, search_mask, accept_mask,
                method="canny"):
    binary = _segment(gray, search_mask, method)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    zone_area = zone.zone_size ** 2
    height, width_px = accept_mask.shape[:2]
    detections = []
    for contour in contours:
        area_px = float(cv2.contourArea(contour))
        if area_px < 4.0:
            continue

        rect = cv2.minAreaRect(contour)
        # Centre-only acceptance: see build_masks(). A block overhanging the
        # zone edge stays whole; the mat's own outline is caught by area below.
        cx, cy = int(round(rect[0][0])), int(round(rect[0][1]))
        if not (0 <= cx < width_px and 0 <= cy < height and accept_mask[cy, cx]):
            continue

        box_px = cv2.boxPoints(rect)

        # Measure in ZONE space, not pixels. A rect that is square in pixels is
        # not square on the mat once perspective is in play, and the px->m scale
        # differs across the frame -- mapping the four corners through the
        # homography and measuring there gets both right at once.
        box_zone = px_to_zone(H_px_to_zone, box_px)
        side_a = float(np.linalg.norm(box_zone[1] - box_zone[0]))
        side_b = float(np.linalg.norm(box_zone[2] - box_zone[1]))
        if side_a <= 0.0 or side_b <= 0.0:
            continue

        footprint_area = side_a * side_b
        if not (MIN_BLOCK_AREA_FRAC * zone_area <= footprint_area
                <= MAX_BLOCK_AREA_FRAC * zone_area):
            continue

        centre_zone = box_zone.mean(axis=0)
        if side_a >= side_b:
            length, width = side_a, side_b
            major = box_zone[1] - box_zone[0]
        else:
            length, width = side_b, side_a
            major = box_zone[2] - box_zone[1]

        # Contour area in metric terms, via the same box the sides came from --
        # cheaper and less perspective-sensitive than warping the whole contour.
        px_area_of_box = float(cv2.contourArea(box_px.astype(np.float32)))
        fill_ratio = area_px / px_area_of_box if px_area_of_box > 0 else 0.0

        shape, symmetry = _classify(width, length, fill_ratio)
        detections.append(Detection(
            zx=float(centre_zone[0]), zy=float(centre_zone[1]),
            zyaw=wrap_angle(math.atan2(major[1], major[0])),
            width=width, length=length, shape=shape, symmetry=symmetry,
            fill_ratio=fill_ratio, area_px=area_px, box_px=box_px))

    detections.sort(key=lambda d: d.width * d.length, reverse=True)
    return detections


# ---------------------------------------------------------------------------
# Top level
# ---------------------------------------------------------------------------
def analyze(image, zone, method="canny", max_rms_px=MAX_HOMOGRAPHY_RMS_PX):
    """Full pipeline on one frame. Never raises for an ordinary bad frame --
    returns a ZoneResult with success=False and a message saying which stage
    failed, because "no tags" and "wrong answer" must not look alike to the
    caller."""
    result = ZoneResult()

    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    tag_corners = detect_tags(gray, zone)
    result.tag_corners = tag_corners
    result.tag_ids = sorted(tag_corners)
    if len(tag_corners) < MIN_TAGS:
        result.message = (
            "saw %d of zone's 4 tags %s, need %d. Check framing, focus and "
            "lighting before suspecting anything else."
            % (len(tag_corners), list(zone.tag_ids), MIN_TAGS))
        return result

    H, rms = fit_homography(tag_corners, zone)
    result.homography_rms = rms
    if H is None:
        result.message = "homography fit failed (degenerate tag layout?)"
        return result
    if rms > max_rms_px:
        result.message = (
            "homography RMS %.2f px exceeds %.2f. The tag->zone mapping is not "
            "trustworthy, so no position from this frame is either. Likely a "
            "misdetected tag, a tag printed rotated, or PICKUP/PLACE_TAG_IDS "
            "not matching the physical mat." % (rms, max_rms_px))
        return result

    result.H_zone_to_px = H
    result.H_px_to_zone = np.linalg.inv(H)
    result.scale_px_per_m = _scale_at_centre(H, zone)

    # Where the camera itself is, in zone coordinates. See camera_in_zone().
    height, width_px = gray.shape[:2]
    result.camera_zx, result.camera_zy = camera_in_zone(
        result.H_px_to_zone, width_px, height)

    search_mask, accept_mask = build_masks(gray.shape[:2], H, zone)
    result.mask = search_mask
    result.accept_mask = accept_mask

    # Paint the tags out at grayscale, using the mat's own median so the painted
    # quad is not itself a step edge. Must happen before any edge detection.
    interior = gray[search_mask > 0]
    fill_value = float(np.median(interior)) if interior.size else 0.0
    result.flat_gray = flatten_tags(gray, H, zone, tag_corners, fill_value)

    result.blocks = find_blocks(result.flat_gray, H, result.H_px_to_zone, zone,
                                search_mask, accept_mask, method=method)

    result.success = True
    # An empty zone is a SUCCESS with zero blocks, not a failure. Stage 2 asks
    # exactly this question before releasing, and Stage 4 asks it again.
    result.message = "%d tag(s), rms %.2f px, %d block(s)" % (
        result.tags_seen, rms, len(result.blocks))
    return result
