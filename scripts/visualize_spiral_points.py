#!/usr/bin/env python3
"""
Visualize the spiral reach-test points RELATIVE TO THE ROBOT.

Two independent modes (use either or both):

  RViz markers (default):
      python3 viz_spiral_points.py
      python3 viz_spiral_points.py --frame base_link
      python3 viz_spiral_points.py --results /tmp/spiral_results.csv --labels
    Then in RViz:  Add -> MarkerArray, topic = /spiral_markers,
    and set the Fixed Frame to whatever you pass with --frame (default 'world').

  Matplotlib top-down PNG (no ROS/RViz needed):
      python3 viz_spiral_points.py --png /tmp/spiral_points.png
      python3 viz_spiral_points.py --png /tmp/spiral_points.png --results /tmp/spiral_results.csv

The spiral parameters below are duplicated from spiral_reach_test.py on purpose
so this stays a standalone, fast script (no moveit import).
"""

import argparse
import csv
import math
import os

# --- keep these in sync with spiral_reach_test.py ---
SPIRAL_R0 = 0.12
SPIRAL_R_MAX = 0.28
SPIRAL_GROWTH_PER_TURN = 0.05
SPIRAL_ARC_STEP = 0.05
GRASP_Z = 0.10
# ----------------------------------------------------


def generate_spiral(r0=SPIRAL_R0, r_max=SPIRAL_R_MAX,
                    growth_per_turn=SPIRAL_GROWTH_PER_TURN,
                    arc_step=SPIRAL_ARC_STEP, z=GRASP_Z):
    b = growth_per_turn / (2.0 * math.pi)
    points, theta, idx = [], 0.0, 0
    while True:
        r = r0 + b * theta
        if r > r_max:
            break
        points.append({
            "index": idx,
            "x": r * math.cos(theta),
            "y": r * math.sin(theta),
            "z": z,
            "r": r,
            "theta_deg": math.degrees(theta) % 360.0,
        })
        theta += arc_step / math.sqrt(r * r + b * b)
        idx += 1
    return points


def load_results(path):
    """Return {index: bool success} if the CSV exists, else {}."""
    if not path or not os.path.exists(path):
        return {}
    out = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                out[int(row["index"])] = str(row["success"]).strip().lower() == "true"
            except (KeyError, ValueError):
                pass
    return out


# ---------------------------------------------------------------------------
# Matplotlib (no ROS required)
# ---------------------------------------------------------------------------

def plot_matplotlib(points, results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    # reach rings
    for radius, style in ((SPIRAL_R0, ":"), (SPIRAL_R_MAX, "--")):
        circle = plt.Circle((0, 0), radius, fill=False, color="0.6", linestyle=style)
        ax.add_patch(circle)

    # spiral path
    ax.plot([p["x"] for p in points], [p["y"] for p in points],
            "-", color="0.85", linewidth=0.8, zorder=0)

    # robot base
    ax.plot(0, 0, "ks", markersize=10, label="base")
    ax.annotate("+x (front)", (SPIRAL_R_MAX * 0.72, 0.0), color="0.4")

    for p in points:
        if results:
            color = "tab:green" if results.get(p["index"]) else "tab:red"
        else:
            color = "tab:blue"
        ax.plot(p["x"], p["y"], "o", color=color, markersize=6)
        ax.annotate(str(p["index"]), (p["x"], p["y"]),
                    fontsize=6, color="0.3",
                    xytext=(3, 3), textcoords="offset points")

    ax.set_aspect("equal")
    lim = SPIRAL_R_MAX * 1.15
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("x (m)  [robot forward]")
    ax.set_ylabel("y (m)  [robot left]")
    title = "Spiral test points (top-down, base at origin)"
    if results:
        n_ok = sum(1 for p in points if results.get(p["index"]))
        title += f"  --  green=reachable, red=fail  ({n_ok}/{len(points)})"
    ax.set_title(title)
    ax.grid(alpha=0.3)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"Wrote {path}")


# ---------------------------------------------------------------------------
# RViz markers
# ---------------------------------------------------------------------------

def publish_rviz(points, results, frame, show_labels):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA

    def color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
        return c

    def pt(x, y, z):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        return p

    class SpiralViz(Node):
        def __init__(self):
            super().__init__("spiral_viz")
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched
            self.pub = self.create_publisher(MarkerArray, "/spiral_markers", qos)
            self.msg = self.build()
            self.pub.publish(self.msg)
            # republish periodically so late-joining RViz still sees it
            self.create_timer(2.0, lambda: self.pub.publish(self.msg))
            self.get_logger().info(
                f"Publishing {len(points)} points on /spiral_markers "
                f"in frame '{frame}'. Add a MarkerArray display in RViz.")

        def build(self):
            arr = MarkerArray()
            now = self.get_clock().now().to_msg()

            def base_marker(mid, mtype):
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = now
                m.ns = "spiral"
                m.id = mid
                m.type = mtype
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                return m

            # spiral path line
            line = base_marker(11, Marker.LINE_STRIP)
            line.scale.x = 0.002
            line.color = color(0.7, 0.7, 0.7, 0.9)
            line.points = [pt(p["x"], p["y"], p["z"]) for p in points]
            arr.markers.append(line)

            # points as a single SPHERE_LIST
            spheres = base_marker(10, Marker.SPHERE_LIST)
            spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.012
            for p in points:
                spheres.points.append(pt(p["x"], p["y"], p["z"]))
                if results:
                    spheres.colors.append(
                        color(0.1, 0.8, 0.1) if results.get(p["index"])
                        else color(0.85, 0.1, 0.1))
                else:
                    # gradient by radius (blue inner -> cyan outer)
                    t = (p["r"] - SPIRAL_R0) / max(1e-6, SPIRAL_R_MAX - SPIRAL_R0)
                    spheres.colors.append(color(0.1, 0.3 + 0.5 * t, 1.0 - 0.4 * t))
            arr.markers.append(spheres)

            # reach rings at R0 and R_MAX
            for mid, radius, col in ((12, SPIRAL_R0, color(0.5, 0.5, 0.5, 0.5)),
                                     (13, SPIRAL_R_MAX, color(0.9, 0.6, 0.2, 0.7))):
                ring = base_marker(mid, Marker.LINE_STRIP)
                ring.scale.x = 0.002
                ring.color = col
                ring.points = [pt(radius * math.cos(a), radius * math.sin(a), GRASP_Z)
                               for a in [i * 2 * math.pi / 72 for i in range(73)]]
                arr.markers.append(ring)

            # base marker at origin
            base = base_marker(14, Marker.SPHERE)
            base.scale.x = base.scale.y = base.scale.z = 0.03
            base.color = color(0.1, 0.1, 0.1)
            arr.markers.append(base)

            # +x arrow (robot forward)
            arrow = base_marker(15, Marker.ARROW)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.01, 0.01
            arrow.color = color(0.2, 0.2, 0.9)
            arr.markers.append(arrow)

            # optional index labels
            if show_labels:
                for p in points:
                    t = base_marker(1000 + p["index"], Marker.TEXT_VIEW_FACING)
                    t.pose.position = pt(p["x"], p["y"], p["z"] + 0.01)
                    t.scale.z = 0.01
                    t.color = color(0.2, 0.2, 0.2)
                    t.text = str(p["index"])
                    arr.markers.append(t)

            return arr

    rclpy.init()
    node = SpiralViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="world",
                    help="TF frame to publish markers in (default: world)")
    ap.add_argument("--results", default="",
                    help="optional spiral_results.csv to colour points green/red")
    ap.add_argument("--labels", action="store_true",
                    help="show index labels in RViz")
    ap.add_argument("--png", default="",
                    help="also (or instead) write a matplotlib top-down PNG here")
    ap.add_argument("--no-rviz", action="store_true",
                    help="skip RViz publishing (use with --png for a pure plot)")
    args = ap.parse_args()

    points = generate_spiral()
    results = load_results(args.results)
    print(f"Generated {len(points)} spiral points "
          f"(r={SPIRAL_R0:.2f}..{SPIRAL_R_MAX:.2f} m)"
          + (f", loaded {len(results)} results" if results else ""))

    if args.png:
        plot_matplotlib(points, results, args.png)

    if not args.no_rviz:
        publish_rviz(points, results, args.frame, args.labels)


if __name__ == "__main__":
    main()