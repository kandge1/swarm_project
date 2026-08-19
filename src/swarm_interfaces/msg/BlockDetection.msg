# One object found inside a zone.
#
# Reported in TWO frames on purpose:
#
#   zx/zy/zyaw  ZONE-LOCAL. This is the raw measurement -- it comes straight out
#               of the tag homography and depends on nothing but the tags being
#               visible. Not on the arm's position, not on the camera pose, not
#               on the camera intrinsics. Log this one.
#
#   x/y/yaw     WORLD. Derived by composing the zone-local measurement with the
#               zone pose the CALLER supplied in the request. Its accuracy is
#               therefore the accuracy of that surveyed zone pose -- the vision
#               contributes no error here beyond what zone-local already has.
#
# The distinction matters when a grasp misses: a bad zx/zy is a vision problem,
# a good zx/zy with a bad grasp is a zone-survey or arm problem. See
# APRIL_TAGS.md, "The central design decision".

float64 zx          # zone-local m, +X along the tag square's first edge
float64 zy          # zone-local m
float64 zyaw        # zone-local rad, the footprint's MAJOR axis

float64 x           # world m, footprint centre
float64 y           # world m
float64 yaw         # world rad, the footprint's MAJOR axis

float64 width       # m, SHORT footprint dimension
float64 length      # m, LONG footprint dimension

string shape        # "square" | "rect" | "circle" | "unknown"

# Order of the block's rotational symmetry about the vertical, i.e. how many
# distinct yaws are indistinguishable from above:
#   1 = none (yaw is unique)   2 = 180 deg (a rectangle)
#   4 = 90 deg (a square)      0 = continuous (a circle -- yaw is meaningless)
# The caller reduces the grasp yaw modulo (360/symmetry) to pick the wrist
# rotation nearest its current one. Do not skip this: an unreduced yaw can
# demand a wrist rotation past joint6output_to_joint6's -2.4434 rad limit.
uint8 symmetry

float64 fill_ratio  # contour area / minAreaRect area. ~1.0 rectangle, ~0.79 circle
float64 area_px     # contour area in pixels, for triage when a detection looks wrong
