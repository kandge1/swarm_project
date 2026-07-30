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

It also makes the reduced-tag cases fall out for free, with no special-case
geometry to reconstruct a missing corner: three tags give 12 correspondences and
two give 8 (16 equations for 8 DOF -- still over-determined, so the residual
still means something).

TWO IS THE FLOOR, AND IT IS THE NORMAL CASE ON HARDWARE. The gripper hangs in
front of the lens and hides the far pair of tags from every hover the arm can
reach, so a real still shows 2 of the 4. What two tags cost is not degrees of
freedom but CONDITIONING -- an adjacent pair spans the zone one way and only
their own 25.4mm the other, so the fit extrapolates ~6x across the thin
direction. tag_spread_ratio() measures that, and analyze_multi() is the answer
to it: several stills at different wrist yaws, each solved independently, then
fused. Independently is the operative word -- see the note above analyze_multi
for why pooling the correspondences instead would reintroduce exactly the
encoder error this whole design exists to avoid.

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
# the ~4in working area a block actually gets to sit in.
#
# 6 in, not 4 in. Originally this was 4in with the tags AT the vertices of the
# nominal 4in working square -- but a tag centred on a vertex reaches tag_size/2
# INWARD from it, so a block near a corner sat on top of the tag. Usable area is
#     zone_size/2 - tag_size/2 - block_size/2
# which at 4in zone / 1in tag / 1.18in block was only +-23mm (1.82in) -- less
# than half the intended working square, and a 3in Stage 3 block did not fit at
# all. Moving the tags out to a 6in square around the same ~4in working area
# gives +-48.5mm (3.8in), essentially the whole intended area. Decided
# 2026-07-29; see APRIL_TAGS.md "Usable area" for the derivation and the
# synthetic test that found it.
DEFAULT_ZONE_SIZE = 0.1524      # m
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

# Lowered 3 -> 2 on 2026-07-30, because on real hardware 3 is unachievable: the
# GRIPPER occludes the far pair of tags from every hover the arm can reach, so a
# single still sees 2 of the 4, never more. Refusing 2 refused every real frame.
#
# 2 tags is not a degraded fit in the way the old comment claimed. Each tag
# contributes 4 corners and each corner 2 equations, so 2 tags give 16 equations
# for a homography's 8 DOF -- genuinely over-determined, and the RMS residual
# stays meaningful. (1 tag would be 8 equations for 8 DOF: exactly determined,
# zero residual by construction, and therefore worthless as a health check.
# That is why the floor is 2 and not 1.)
#
# The real hazard with 2 tags is not the DOF count, it is CONDITIONING. Two
# adjacent tags span the zone in one direction but only their own 25.4 mm in the
# perpendicular one, so the fit extrapolates that direction ~6x out to the zone
# edge and amplifies corner-localisation noise by the same factor. That is what
# TAG_SPREAD_MIN_RATIO guards, and it is the reason analyze_multi() exists:
# fusing several weak single-still fits from different wrist yaws both averages
# the error down and, more usefully, makes the spread ACROSS stills an honest
# independent estimate of the total error.
MIN_TAGS = 2

# Conditioning floor for the tag-corner constellation: the ratio of its minor to
# major spread (PCA) in ZONE coordinates. MEASURED, not estimated:
#
#   all four tags      1.0000
#   any three          0.5890
#   adjacent pair      0.1644
#   diagonal pair      0.1170
#   single tag         1.0000  <- see below
#
# Two corrections to the obvious intuitions, both of which cost a first draft:
#
# 1. A DIAGONAL pair is WORSE conditioned than an adjacent one (0.117 vs 0.164),
#    which is backwards from the "diagonal spans the zone better" reading. For a
#    homography what matters is whether the points are in general position, and
#    two diagonal tags put all 8 corners in a thin band along the diagonal --
#    the perpendicular direction is pinned only by the 25.4mm tag width, over a
#    longer (x sqrt 2) baseline than the adjacent case. Genuinely thinner.
#
# 2. A SINGLE tag scores 1.0000, because this metric measures the constellation's
#    SHAPE and a lone tag's four corners are a perfect square. It says nothing
#    about scale. MIN_TAGS = 2 is what excludes the single-tag case; do not
#    expect this number to.
#
# 0.08 sits below both real two-tag cases deliberately. An earlier 0.12 would
# have rejected every diagonal pair -- and which pair the gripper leaves visible
# is a function of wrist angle, so that is a case the acquisition plan actively
# produces. Hard-rejecting a weak-but-usable view is the wrong trade when
# analyze_multi() can fuse it and report the spread: throwing data away in
# exchange for a cleaner-looking single answer is how a system ends up confident
# and wrong. The floor is here only for TRUE degeneracy -- collinear points, or
# two "different" tags detected on top of each other -- where the fit is singular
# and its answer is arbitrary rather than merely noisy. Quality of everything
# above the floor is expressed by homography_rms and the multi-view spread.
TAG_SPREAD_MIN_RATIO = 0.08

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
MIN_BLOCK_AREA_FRAC = 0.0018    # ~0.18% of a 6in zone square = a 6mm speck
                                 # (kept the same ABSOLUTE speck size as the old
                                 # 4in zone's 0.4% -- the fraction changed, the
                                 # thing it's meant to filter did not)
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
        self.tag_spread = 0.0       # conditioning, see tag_spread_ratio()
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


def detect_all_tags(gray):
    """{tag_id: 4x2 px corners} for EVERY tag in the frame, any zone or none.

    detect_tags() filters to one ZoneSpec's ids because analyze() only ever
    wants to know about the zone it was asked about. Tooling that wants to
    show everything the camera can see -- a live diagnostic viewer, say --
    wants the unfiltered set instead.
    """
    return dict(_aruco_detect(gray))


def _build_tag_zone_lookup():
    lookup = {}
    for ids, zone_name in ((PICKUP_TAG_IDS, "pickup"), (PLACE_TAG_IDS, "place")):
        for tag_id, signs in zip(ids, ZONE_CORNER_SIGNS):
            lookup[tag_id] = (zone_name, signs)
    return lookup


_TAG_ZONE_LOOKUP = _build_tag_zone_lookup()


def describe_tag_id(tag_id):
    """(zone_name, (sx, sy)) for a tag id that belongs to a configured zone,
    or None if it doesn't. (sx, sy) are the ZONE_CORNER_SIGNS entry -- e.g.
    (-1, -1) is the -X,-Y vertex, matching the +X/+Y legend printed on the
    zone sheets by print_zone_tags.py."""
    return _TAG_ZONE_LOOKUP.get(tag_id)


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


def tag_spread_ratio(tag_corners_px, zone):
    """Minor/major spread of the visible tag corners in ZONE coordinates.

    A pure geometry number -- it uses only WHICH tags were seen, never the pixel
    measurements, so it is a property of the occlusion pattern and cannot be
    fooled by a bad detection. See TAG_SPREAD_MIN_RATIO.

    1.0 is an ideal square constellation; 0.0 is collinear, where the homography
    is singular in one direction and its answer there is arbitrary rather than
    just noisy.
    """
    targets = zone.tag_corner_targets()
    pts = np.concatenate([targets[t] for t in sorted(tag_corners_px)], axis=0)
    if len(pts) < 4:
        return 0.0
    centred = pts - pts.mean(axis=0)
    # Singular values of the centred point set are its principal spreads.
    sv = np.linalg.svd(centred, compute_uv=False)
    if sv[0] <= 1e-12:
        return 0.0
    return float(sv[1] / sv[0])


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

    result.tag_spread = tag_spread_ratio(tag_corners, zone)
    if result.tag_spread < TAG_SPREAD_MIN_RATIO:
        result.message = (
            "tags %s are too collinear (spread %.3f < %.3f). A homography fitted "
            "to them is singular across the thin direction, so it would return a "
            "confident-looking but arbitrary answer there. Rotate the wrist and "
            "take another still -- see analyze_multi()."
            % (result.tag_ids, result.tag_spread, TAG_SPREAD_MIN_RATIO))
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


# ---------------------------------------------------------------------------
# Multi-view fusion
# ---------------------------------------------------------------------------
# WHY THIS EXISTS. On real hardware the gripper hangs in front of the lens and
# occludes the far pair of tags from every hover the arm can reach: one still
# sees 2 of the 4 tags, never more. A 2-tag homography is over-determined but
# poorly conditioned across the thin direction of the pair (see MIN_TAGS), so a
# single still is not something to descend on.
#
# The fix is to take several stills with the WRIST ROTATED between them, so a
# different pair of tags is occluded each time, and combine the results.
#
# THE KEY DESIGN CHOICE, and the reason this is structured as "solve each still
# independently, then fuse" rather than "pool all the correspondences into one
# big fit": pooling would require knowing the camera pose of each still relative
# to the others, i.e. trusting the arm's encoders about how far the wrist
# actually turned. As of 2026-07-30 that is known to be exactly what this robot
# cannot be trusted about -- the servo gears have enough wear that the encoder
# and the link disagree by several degrees, invisibly (see APRIL_TAGS.md, ROOT
# CAUSE). Each still here is self-contained: its homography comes only from tags
# visible in that one frame, so the fused answer never depends on the wrist
# angle being what the encoder claims. The wrist rotation only has to CHANGE the
# occlusion; it does not have to be known.
#
# The second payoff is free and arguably worth more than the averaging: the
# SPREAD across stills is an independent, end-to-end estimate of the real error,
# measured on the actual mat under the actual lighting. Nothing else in this
# system produces an honest error bar.

MATCH_RADIUS_M = 0.012          # two views' detections are the same block if
                                # their zone positions agree within this. 12 mm
                                # is well under the 30 mm block so two distinct
                                # blocks can never merge, and well over the
                                # single-view error a 2-tag fit is expected to
                                # have.
MIN_VIEWS_PER_BLOCK = 2         # a block seen in only one still is reported but
                                # flagged: one view has no cross-check at all.


class FusedDetection:
    """A block's pose agreed across several stills, with its spread."""

    def __init__(self, zx, zy, zyaw, width, length, shape, symmetry,
                 n_views, spread_m, spread_yaw_rad, views):
        self.zx = zx
        self.zy = zy
        self.zyaw = zyaw
        self.width = width
        self.length = length
        self.shape = shape
        self.symmetry = symmetry
        self.n_views = n_views
        self.spread_m = spread_m            # max deviation from the fused centre
        self.spread_yaw_rad = spread_yaw_rad
        self.views = views                  # the contributing Detections

    @property
    def trustworthy(self):
        return self.n_views >= MIN_VIEWS_PER_BLOCK

    def world_pose(self, zone):
        x, y = zone.zone_to_world(self.zx, self.zy)
        return x, y, zone.zone_yaw_to_world(self.zyaw)

    def __repr__(self):
        return ("FusedDetection(zone=(%.4f, %.4f) yaw=%.1fdeg %s %.1fx%.1fmm "
                "views=%d spread=%.1fmm/%.1fdeg)"
                % (self.zx, self.zy, math.degrees(self.zyaw), self.shape,
                   self.width * 1000.0, self.length * 1000.0, self.n_views,
                   self.spread_m * 1000.0, math.degrees(self.spread_yaw_rad)))


class FusedResult:
    def __init__(self):
        self.success = False
        self.message = ""
        self.blocks = []
        self.views = []             # every ZoneResult, good or bad
        self.good_views = 0
        self.tag_ids_union = []
        self.camera_spread_m = 0.0  # how far apart the per-still camera
                                    # positions landed; see analyze_multi


def _fold_yaw(yaw, symmetry):
    """Fold a yaw into the canonical wedge for its symmetry order.

    A square block's 0 and 90 degrees are the same physical pose, so averaging
    them raw would give 45 -- a pose the block is never in. Folding first is what
    makes a circular mean meaningful here.
    """
    if not symmetry:                     # 0 = continuous (a circle): yaw is
        return 0.0                       # meaningless, do not average noise
    period = math.pi * 2.0 / symmetry
    return yaw % period


def _circular_mean(angles, period):
    """Mean of angles that wrap at `period`, via unit vectors.

    Plain averaging breaks across the wrap point -- two readings either side of
    it average to the opposite of the truth, which for a 4-fold block is the one
    error large enough to make the gripper miss.
    """
    scale = 2.0 * math.pi / period
    s = sum(math.sin(a * scale) for a in angles)
    c = sum(math.cos(a * scale) for a in angles)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return angles[0]
    return (math.atan2(s, c) / scale) % period


def fuse_detections(per_view_blocks, match_radius_m=MATCH_RADIUS_M):
    """Group detections that refer to the same physical block across stills.

    per_view_blocks: [[Detection, ...], ...], one list per still.
    Greedy nearest-cluster assignment in zone coordinates -- adequate because
    match_radius_m is far below the block pitch, so clusters cannot overlap.
    """
    clusters = []
    for view_blocks in per_view_blocks:
        for det in view_blocks:
            for cluster in clusters:
                if math.hypot(det.zx - cluster[0].zx,
                              det.zy - cluster[0].zy) <= match_radius_m:
                    cluster.append(det)
                    break
            else:
                clusters.append([det])

    fused = []
    for cluster in clusters:
        zx = float(np.median([d.zx for d in cluster]))
        zy = float(np.median([d.zy for d in cluster]))
        width = float(np.median([d.width for d in cluster]))
        length = float(np.median([d.length for d in cluster]))
        # Shape and symmetry by majority: a single still misreading a square as a
        # rectangle must not decide the grasp for all of them.
        shapes = [d.shape for d in cluster]
        shape = max(set(shapes), key=shapes.count)
        syms = [d.symmetry for d in cluster]
        symmetry = max(set(syms), key=syms.count)

        if symmetry:
            period = math.pi * 2.0 / symmetry
            folded = [_fold_yaw(d.zyaw, symmetry) for d in cluster]
            zyaw = _circular_mean(folded, period)
            # Spread measured the same wrapped way it was averaged.
            devs = []
            for a in folded:
                d_ = abs(a - zyaw) % period
                devs.append(min(d_, period - d_))
            spread_yaw = max(devs) if devs else 0.0
        else:
            zyaw = 0.0
            spread_yaw = 0.0

        spread = max(math.hypot(d.zx - zx, d.zy - zy) for d in cluster)
        fused.append(FusedDetection(zx, zy, zyaw, width, length, shape,
                                    symmetry, len(cluster), spread,
                                    spread_yaw, list(cluster)))
    fused.sort(key=lambda f: -f.n_views)
    return fused


def analyze_multi(images, zone, method="canny",
                  max_rms_px=MAX_HOMOGRAPHY_RMS_PX):
    """analyze() over several stills of the same zone, fused into one answer.

    The stills should be taken with the wrist rotated between them so a
    different pair of tags is occluded in each. Their camera poses do NOT need to
    be known, and deliberately are not used -- see the note above.

    Views that fail are kept in .views with their messages rather than dropped
    silently: "3 of 4 stills saw no tags" is a lighting or framing diagnosis, and
    it must not look the same as "all 4 agreed".
    """
    result = FusedResult()
    per_view = []
    cams = []
    ids = set()
    for image in images:
        view = analyze(image, zone, method=method, max_rms_px=max_rms_px)
        result.views.append(view)
        if not view.success:
            continue
        result.good_views += 1
        ids.update(view.tag_ids)
        per_view.append(view.blocks)
        cams.append((view.camera_zx, view.camera_zy))

    result.tag_ids_union = sorted(ids)

    if result.good_views == 0:
        msgs = "; ".join(v.message for v in result.views) or "no stills supplied"
        result.message = "no still produced a usable homography (%s)" % msgs
        return result

    # How far apart the stills thought the CAMERA was. The arm does move the
    # wrist between stills, so this is not expected to be zero -- it is a sanity
    # bound, and a wild value means a still was fitted against a misdetected tag.
    if len(cams) > 1:
        mx = float(np.median([c[0] for c in cams]))
        my = float(np.median([c[1] for c in cams]))
        result.camera_spread_m = max(math.hypot(c[0] - mx, c[1] - my)
                                     for c in cams)

    result.blocks = fuse_detections(per_view)
    result.success = True
    result.message = "%d/%d stills usable, tags %s, %d block(s)" % (
        result.good_views, len(result.views), result.tag_ids_union,
        len(result.blocks))
    return result
