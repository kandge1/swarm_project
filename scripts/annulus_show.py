#!/usr/bin/env python3
"""
annulus_viz.py -- publish the confirmed annular-sector work zone to RViz as a
translucent green volume, plus an opaque outline tracing the exact boundary
that annulus_test.py --execute completed (66/66 vertices, all four edges).

WHAT GETS PUBLISHED (MarkerArray on --topic, default /reachability_zone):
  id 0  TRIANGLE_LIST   translucent green volume, extruded from z-lo to z-hi
  id 1  LINE_STRIP      bright, opaque green outline of the traced boundary
                         at the trace plane (uses annulus_test.generate_boundary
                         directly, so it's the real executed path, not a
                         redrawn approximation)
  id 2  TEXT_VIEW_FACING label showing r / yaw range, floating above the zone

DEFAULTS
--------
r-inner/r-outer/yaw-min/yaw-max default to the values confirmed by the
--sweep-rz ground-truth run and then executed cleanly end to end:
    r    : 0.15 .. 0.24 m
    yaw  : -80 .. +80 deg
z-lo/z-hi default to the trace plane and hover height from annulus_test.py
(TRACE_Z=0.14, TRACE_Z+HOVER_DZ=0.20) so the volume reads as "the slab of
space the arm can work within," not just a flat sheet.

These are CLI-overridable because the confirmed numbers are empirical
(from your sweep on your robot) and will drift if the URDF, grasp geometry,
or ceilings change -- don't let this script's defaults silently go stale.

USAGE
-----
    python3 annulus_viz.py
    python3 annulus_viz.py --r-inner 0.15 --r-outer 0.24 --alpha 0.35
    python3 annulus_viz.py --topic /reachability_zone --rate 1.0

Then in RViz: Add -> By display type -> MarkerArray -> set Topic to the
value above (default /reachability_zone). Fixed Frame must match --frame
(default 'world', same as PLANNING_FRAME in pick_place.py).

Requires annulus_test.py (and therefore pick_place.py) importable from the
same directory, purely for PLANNING_FRAME and generate_boundary() -- no
MoveIt / planning scene is touched here, this is visualization only.
"""

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from annulus_test import generate_boundary  # noqa: E402
from pick_place import PLANNING_FRAME  # noqa: E402


# ---------------------------------------------------------------------------
# Confirmed-by-sweep defaults. See module docstring.
# ---------------------------------------------------------------------------

R_INNER_DEFAULT = 0.15
R_OUTER_DEFAULT = 0.24
YAW_MIN_DEFAULT = -80.0   # degrees
YAW_MAX_DEFAULT = +80.0   # degrees
Z_LO_DEFAULT = 0.14       # trace plane, matches annulus_test.TRACE_Z
Z_HI_DEFAULT = 0.20       # hover height, matches TRACE_Z + HOVER_DZ

VIZ_ARC_STEP = 0.005      # fine subdivision for a smooth rendered curve,
                          # independent of the coarser step actually used
                          # during physical execution


# ---------------------------------------------------------------------------
# Pure geometry -- no ROS types, so this is unit-testable without rclpy.
# ---------------------------------------------------------------------------

def _pt(r, yaw, z):
    return (r * math.cos(yaw), r * math.sin(yaw), z)


def annulus_prism_triangles(r_inner, r_outer, yaw_min, yaw_max, z_lo, z_hi,
                            arc_step=VIZ_ARC_STEP):
    """Closed, watertight triangular mesh of an annular-sector prism.

    Returns a flat list of (x, y, z) tuples, length a multiple of 3 -- every
    consecutive triple is one triangle. Six surfaces are covered: top, bottom,
    outer wall, inner wall, and the two flat end-caps at yaw_min / yaw_max.

    Winding is chosen for outward-facing normals; if some faces render dark
    from certain angles in RViz that's a backface-culling artifact of one
    surface's winding, not a hole in the geometry -- the watertight-ness is
    what the self-test below checks, not per-face winding direction.
    """
    span = abs(yaw_max - yaw_min) * max(r_inner, r_outer)
    n_div = max(2, int(math.ceil(span / arc_step)) + 1)
    angles = [yaw_min + i * (yaw_max - yaw_min) / (n_div - 1) for i in range(n_div)]

    tris = []

    for i in range(n_div - 1):
        a0, a1 = angles[i], angles[i + 1]

        it0, it1 = _pt(r_inner, a0, z_hi), _pt(r_inner, a1, z_hi)
        ot0, ot1 = _pt(r_outer, a0, z_hi), _pt(r_outer, a1, z_hi)
        ib0, ib1 = _pt(r_inner, a0, z_lo), _pt(r_inner, a1, z_lo)
        ob0, ob1 = _pt(r_outer, a0, z_lo), _pt(r_outer, a1, z_lo)

        # top (z_hi), viewed from above: CCW = outward-up normal
        tris += [it0, ot0, ot1, it0, ot1, it1]
        # bottom (z_lo), viewed from below: reversed winding
        tris += [ib0, ob1, ob0, ib0, ib1, ob1]
        # outer wall, normal points +r
        tris += [ot0, ob0, ob1, ot0, ob1, ot1]
        # inner wall, normal points -r
        tris += [it0, ib1, ib0, it0, it1, ib1]

    # end caps
    a0 = angles[0]
    it0, ot0 = _pt(r_inner, a0, z_hi), _pt(r_outer, a0, z_hi)
    ib0, ob0 = _pt(r_inner, a0, z_lo), _pt(r_outer, a0, z_lo)
    tris += [it0, ib0, ob0, it0, ob0, ot0]

    aN = angles[-1]
    itN, otN = _pt(r_inner, aN, z_hi), _pt(r_outer, aN, z_hi)
    ibN, obN = _pt(r_inner, aN, z_lo), _pt(r_outer, aN, z_lo)
    tris += [itN, obN, ibN, itN, otN, obN]

    return tris


def _selftest_prism(r_inner=0.15, r_outer=0.24, yaw_min=math.radians(-80),
                    yaw_max=math.radians(80), z_lo=0.14, z_hi=0.20):
    """Sanity-check the mesh: right triangle count, watertight bounds,
    no degenerate (zero-area) triangles. Raises AssertionError on failure."""
    tris = annulus_prism_triangles(r_inner, r_outer, yaw_min, yaw_max, z_lo, z_hi)
    assert len(tris) % 3 == 0, "triangle list length must be a multiple of 3"
    n_tri = len(tris) // 3

    span = abs(yaw_max - yaw_min) * max(r_inner, r_outer)
    n_div = max(2, int(math.ceil(span / VIZ_ARC_STEP)) + 1)
    expected = 8 * (n_div - 1) + 4
    assert n_tri == expected, f"triangle count {n_tri} != expected {expected}"

    rs = [math.hypot(p[0], p[1]) for p in tris]
    zs = [p[2] for p in tris]
    assert min(rs) >= r_inner - 1e-9, "a vertex fell inside r_inner"
    assert max(rs) <= r_outer + 1e-9, "a vertex fell outside r_outer"
    assert abs(min(zs) - z_lo) < 1e-9, "z_lo bound not touched"
    assert abs(max(zs) - z_hi) < 1e-9, "z_hi bound not touched"

    def area(p0, p1, p2):
        ux, uy, uz = p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]
        vx, vy, vz = p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]
        cx, cy, cz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
        return 0.5 * math.sqrt(cx*cx + cy*cy + cz*cz)

    degenerate = 0
    for i in range(0, len(tris), 3):
        if area(tris[i], tris[i+1], tris[i+2]) < 1e-12:
            degenerate += 1
    assert degenerate == 0, f"{degenerate} degenerate triangles"

    return n_tri, n_div


# ---------------------------------------------------------------------------
# ROS glue
# ---------------------------------------------------------------------------

def _to_points(tris):
    return [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in tris]


def build_marker_array(frame_id, r_inner, r_outer, yaw_min, yaw_max,
                       z_lo, z_hi, alpha, color, stamp, lifetime_sec):
    marker_array = MarkerArray()
    lifetime = Duration(sec=int(lifetime_sec),
                        nanosec=int((lifetime_sec % 1) * 1e9))

    # ---- id 0: translucent volume ----
    vol = Marker()
    vol.header.frame_id = frame_id
    vol.header.stamp = stamp
    vol.ns = "reachability_zone"
    vol.id = 0
    vol.type = Marker.TRIANGLE_LIST
    vol.action = Marker.ADD
    vol.pose.orientation.w = 1.0
    vol.scale.x = vol.scale.y = vol.scale.z = 1.0
    vol.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    vol.lifetime = lifetime
    tris = annulus_prism_triangles(r_inner, r_outer, yaw_min, yaw_max, z_lo, z_hi)
    vol.points = _to_points(tris)
    marker_array.markers.append(vol)

    # ---- id 1: outline of the ACTUAL executed boundary ----
    # Reuses annulus_test.generate_boundary directly -- these are the same
    # vertices annulus_test.py --execute traced, just resampled finer here
    # purely for a smooth on-screen line.
    boundary_pts, _edges = generate_boundary(
        r_inner=r_inner, r_outer=r_outer,
        yaw_min=yaw_min, yaw_max=yaw_max,
        z=z_lo, arc_step=VIZ_ARC_STEP, radial_step=VIZ_ARC_STEP)

    outline = Marker()
    outline.header.frame_id = frame_id
    outline.header.stamp = stamp
    outline.ns = "reachability_zone"
    outline.id = 1
    outline.type = Marker.LINE_STRIP
    outline.action = Marker.ADD
    outline.pose.orientation.w = 1.0
    outline.scale.x = 0.003  # line width, meters
    outline.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    outline.lifetime = lifetime
    outline.points = [Point(x=p["x"], y=p["y"], z=p["z"]) for p in boundary_pts]
    marker_array.markers.append(outline)

    # ---- id 2: floating label ----
    label = Marker()
    label.header.frame_id = frame_id
    label.header.stamp = stamp
    label.ns = "reachability_zone"
    label.id = 2
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    mid_yaw = (yaw_min + yaw_max) / 2.0
    mid_r = (r_inner + r_outer) / 2.0
    label.pose.position.x = mid_r * math.cos(mid_yaw)
    label.pose.position.y = mid_r * math.sin(mid_yaw)
    label.pose.position.z = z_hi + 0.05
    label.pose.orientation.w = 1.0
    label.scale.z = 0.02  # text height, meters
    label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
    label.lifetime = lifetime
    label.text = (f"reachable: r {r_inner:.2f}-{r_outer:.2f} m, "
                 f"yaw {math.degrees(yaw_min):+.0f}..{math.degrees(yaw_max):+.0f} deg")
    marker_array.markers.append(label)

    return marker_array


class ReachabilityZoneViz(Node):
    def __init__(self, args):
        super().__init__("reachability_zone_viz")
        self.args = args
        self.pub = self.create_publisher(MarkerArray, args.topic, 10)
        period = 1.0 / args.rate if args.rate > 0 else 1.0
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"Publishing reachability zone on '{args.topic}' in frame "
            f"'{args.frame}' at {args.rate:.1f} Hz. "
            f"r={args.r_inner:.3f}-{args.r_outer:.3f} m, "
            f"yaw={args.yaw_min:+.1f}..{args.yaw_max:+.1f} deg, "
            f"z={args.z_lo:.3f}-{args.z_hi:.3f} m")

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        lifetime_sec = max(2.0, 3.0 / self.args.rate)
        marker_array = build_marker_array(
            frame_id=self.args.frame,
            r_inner=self.args.r_inner, r_outer=self.args.r_outer,
            yaw_min=math.radians(self.args.yaw_min),
            yaw_max=math.radians(self.args.yaw_max),
            z_lo=self.args.z_lo, z_hi=self.args.z_hi,
            alpha=self.args.alpha,
            color=(self.args.color[0], self.args.color[1], self.args.color[2]),
            stamp=stamp, lifetime_sec=lifetime_sec)
        self.pub.publish(marker_array)


def main():
    parser = argparse.ArgumentParser(
        description="Publish the confirmed reachability zone to RViz.")
    parser.add_argument("--r-inner", type=float, default=R_INNER_DEFAULT)
    parser.add_argument("--r-outer", type=float, default=R_OUTER_DEFAULT)
    parser.add_argument("--yaw-min", type=float, default=YAW_MIN_DEFAULT,
                        help="degrees")
    parser.add_argument("--yaw-max", type=float, default=YAW_MAX_DEFAULT,
                        help="degrees")
    parser.add_argument("--z-lo", type=float, default=Z_LO_DEFAULT)
    parser.add_argument("--z-hi", type=float, default=Z_HI_DEFAULT)
    parser.add_argument("--frame", default=PLANNING_FRAME)
    parser.add_argument("--topic", default="/reachability_zone")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="Hz. Republished periodically since RViz's default"
                             " marker subscription QoS is volatile, not latched.")
    parser.add_argument("--alpha", type=float, default=0.4,
                        help="volume transparency, 0=invisible, 1=opaque")
    parser.add_argument("--color", type=float, nargs=3, default=[0.1, 0.9, 0.2],
                        metavar=("R", "G", "B"), help="0..1 each")
    parser.add_argument("--selftest", action="store_true",
                        help="run the offline mesh self-test and exit, no ROS")
    args = parser.parse_args()

    if args.selftest:
        n_tri, n_div = _selftest_prism(
            args.r_inner, args.r_outer,
            math.radians(args.yaw_min), math.radians(args.yaw_max),
            args.z_lo, args.z_hi)
        print(f"Self-test OK: {n_tri} triangles, {n_div} angular divisions, "
              f"watertight, no degenerate faces.")
        return

    rclpy.init()
    node = ReachabilityZoneViz(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.spin() already shuts the context down on KeyboardInterrupt in
        # recent rclpy, so a second rclpy.shutdown() here throws "rcl_shutdown
        # already called." That's harmless (Ctrl+C still stops the node
        # cleanly) but noisy -- swallow just that one error rather than
        # leaving a scary traceback on the totally normal case of hitting
        # Ctrl+C to stop a long-running viz node.
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()