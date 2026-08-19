#!/usr/bin/env python3
"""Spawn 3 AprilTag marker squares (front/left/right) and a ball into the
running Gazebo world.

Uses the same mechanism gazebo.launch.py uses to spawn the robot itself:
the `ros_gz_sim create` CLI tool, which calls the world's gz-transport
/world/<world>/create service. Each object is a self-contained SDF <model>
string with its world pose baked into <pose>, so nothing depends on where
`create`'s own -x/-y/-z flags would otherwise place it.

The table itself is no longer spawned by this script - it's now a static
<model> baked directly into camera_world.sdf so the robot can spawn
already sitting on it (see gazebo.launch.py). TABLE_TOP_Z below must stay
in sync by hand with that table's height (top surface at z=0.02).

Requires Gazebo already running (see myscript.txt, Terminal 1) with a world
named "empty" - true for both the stock empty.sdf and this project's
camera_world.sdf, since the latter kept the same <world name="empty">.
"""
import argparse
import pathlib
import subprocess

ASSETS_DIR = pathlib.Path(__file__).resolve().parent / "assets"
# One distinct tag ID per corner (0-3) so each corner is individually
# identifiable for pose estimation, rather than 4 copies of the same tag.
TAG_TEXTURES = [ASSETS_DIR / f"tag36h11_0000{i}.png" for i in range(4)]

INCH = 0.0254

# Top surface height of the table baked into camera_world.sdf, to match the
# work-surface height annulus_test.py already assumes (its z range is
# 0.13-0.24m measured from the arm base at z=0).
TABLE_TOP_Z = 0.02

# 4 AprilTag markers (1in x 1in each), one at each corner of a 10cm x 10cm
# square, resting on the table. One square in front of the robot (+X) and
# one each to the left/right (+-Y, standard +X-forward/+Y-left convention),
# all the same 0.15m from the origin. Only 4 distinct tag textures exist
# (assets/tag36h11_0000{0-3}.png), so each square reuses the same 4 IDs -
# tags are unique per-corner within a square but repeat across squares.
MARKER_SIZE = 1 * INCH
MARKER_THICKNESS = 0.001
MARKER_Z = TABLE_TOP_Z + MARKER_THICKNESS / 2
SQUARE_SIZE = 0.05
# MUST track pick_place.ZONE_RADIUS_M. The "left"/"right" square centres below
# ARE PICK_XYZ / PLACE_XYZ, and pick_place.py is the same script in sim and on
# hardware -- let these drift apart and the sim spawns blocks where the arm does
# not aim, which looks like a grasp bug rather than a world-file bug.
#
# Duplicated as a literal rather than imported on purpose: this script only needs
# argparse/pathlib/subprocess and shells out to `ros2 run ros_gz_sim create`,
# whereas importing pick_place would drag in rclpy and moveit_msgs. Cheap to keep
# in sync by hand, expensive to make this file depend on a planning stack.
#
# 0.250 -> 0.2286 (9 in) on 2026-08-02, when the physical mats moved in for reach.
SQUARE_DISTANCE = 9 * INCH   # 0.2286
_half = SQUARE_SIZE / 2
SQUARE_CENTERS = {
    "front": (SQUARE_DISTANCE, 0.0),
    "left": (0.0, SQUARE_DISTANCE),
    "right": (0.0, -SQUARE_DISTANCE),
}
SQUARE_CORNERS = {
    name: [
        (cx + sx * _half, cy + sy * _half)
        for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    ]
    for name, (cx, cy) in SQUARE_CENTERS.items()
}

# Ball: sits at the center of the front square, resting directly on the table.
BALL_RADIUS = 0.02
BALL_MASS = 0.05
BALL_X, BALL_Y = SQUARE_CENTERS["right"]
BALL_Z = TABLE_TOP_Z + BALL_RADIUS

CUBE_SIZE_1 = 0.02
CUBE_MASS_1 = 0.05
CUBE_X_1, CUBE_Y_1 = SQUARE_CENTERS["left"]
CUBE_Z_1 = TABLE_TOP_Z + CUBE_SIZE_1 / 2


CUBE_SIZE_2 = 0.02
CUBE_MASS_2 = 0.05
CUBE_X_2, CUBE_Y_2 = SQUARE_CENTERS["right"]
CUBE_Z_2 = TABLE_TOP_Z + CUBE_SIZE_2 / 2

def marker_sdf(texture: pathlib.Path) -> str:
    return f"""
<sdf version="1.9">
  <model name="apriltag_marker">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>{MARKER_SIZE} {MARKER_SIZE} {MARKER_THICKNESS}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{MARKER_SIZE} {MARKER_SIZE} {MARKER_THICKNESS}</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>file://{texture}</albedo_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def cube_sdf(name: str, size: float, mass: float, color: tuple) -> str:
    # Solid cube inertia: I = 1/6 * m * size^2 (same about all 3 axes)
    i = mass * size ** 2 / 6
    r, g, b = color
    return f"""
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def ball_sdf() -> str:
    # Solid sphere inertia: I = 2/5 * m * r^2
    i = 0.4 * BALL_MASS * BALL_RADIUS ** 2
    return f"""
<sdf version="1.9">
  <model name="ball">
    <link name="link">
      <inertial>
        <mass>{BALL_MASS}</mass>
        <inertia>
          <ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>{BALL_RADIUS}</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>{BALL_RADIUS}</radius></sphere></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn(world: str, name: str, sdf: str, x: float, y: float, z: float):
    # `create`'s -x/-y/-z flags (default 0) override whatever <pose> is
    # embedded in -string, rather than composing with it - so the model's
    # own <pose> must stay identity and the real position goes here.
    print(f"Spawning '{name}'...")
    result = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", world, "-name", name, "-string", sdf,
         "-x", str(x), "-y", str(y), "-z", str(z)],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"  FAILED: {result.stderr.strip()}")
        return False
    print("  OK")
    return True


def print_coordinate_summary(spawned_blocks):
    """Print the world-frame centers of everything spawned, so pick_place.py's
    --pick-position/--place-position can be filled in without re-deriving
    them by hand. These are the coordinates baked into this script (not
    measured from the robot's camera), matching pick_place.py's "known
    start/end poses, no camera" approach for now."""
    print("\n" + "=" * 68)
    print("COORDINATE SUMMARY (world frame, meters)")
    print("=" * 68)

    print("\nAprilTag squares (center of the 4-corner square):")
    for square_name, (cx, cy) in SQUARE_CENTERS.items():
        print(f"  {square_name:6s}  center=({cx:+.3f}, {cy:+.3f}, {MARKER_Z:.3f})")

    print("\nBlocks (center Z is what pick_place.py's --pick-position expects; "
          "top Z is the resting surface a block stacked on top of this one "
          "would use for --place-position):")
    for record in spawned_blocks:
        top_z = record["z"] + record["size"] / 2.0
        print(f"  {record['name']:8s}  center=({record['x']:+.3f}, {record['y']:+.3f}, "
              f"{record['z']:.3f})  top_z={top_z:.3f}  on '{record['square']}' square")

    empty_squares = [name for name in SQUARE_CENTERS
                     if name not in {b["square"] for b in spawned_blocks}]
    if spawned_blocks and empty_squares:
        pick = spawned_blocks[0]
        place_x, place_y = SQUARE_CENTERS[empty_squares[0]]
        print("\nExample pick_place.py invocation "
              f"(pick '{pick['name']}', place on empty '{empty_squares[0]}' square, "
              f"resting directly on the table):")
        print(f"  python3 pick_place.py "
              f"--pick-position {pick['x']:.3f} {pick['y']:.3f} {pick['z']:.3f} "
              f"--place-position {place_x:.3f} {place_y:.3f} {TABLE_TOP_Z:.3f}")
        print(f"\nTo stack a second block on top of '{pick['name']}' instead, use its "
              f"top_z as the place surface:")
        print(f"  python3 pick_place.py "
              f"--pick-position <other block center xyz> "
              f"--place-position {pick['x']:.3f} {pick['y']:.3f} "
              f"{pick['z'] + pick['size'] / 2.0:.3f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="empty", help="Gazebo world name")
    args = parser.parse_args()

    for square_name, corners in SQUARE_CORNERS.items():
        for i, (x, y) in enumerate(corners):
            spawn(args.world, f"apriltag_marker_{square_name}_{i}", marker_sdf(TAG_TEXTURES[i]), x, y, MARKER_Z)
    # spawn(args.world, "ball", ball_sdf(), BALL_X, BALL_Y, BALL_Z)

    blocks_to_spawn = [
        {"name": "cube_1", "square": "left", "size": CUBE_SIZE_1, "mass": CUBE_MASS_1,
         "color": (0.1, 0.3, 0.9), "x": CUBE_X_1, "y": CUBE_Y_1, "z": CUBE_Z_1},
        {"name": "cube_2", "square": "right", "size": CUBE_SIZE_2, "mass": CUBE_MASS_2,
         "color": (0.15, 0.7, 0.2), "x": CUBE_X_2, "y": CUBE_Y_2, "z": CUBE_Z_2},
    ]
    spawned_blocks = []
    for block in blocks_to_spawn:
        sdf = cube_sdf(block["name"], block["size"], block["mass"], block["color"])
        if spawn(args.world, block["name"], sdf, block["x"], block["y"], block["z"]):
            spawned_blocks.append(block)

    print_coordinate_summary(spawned_blocks)


if __name__ == "__main__":
    main()
