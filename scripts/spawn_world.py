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
SQUARE_SIZE = 0.10
SQUARE_DISTANCE = 0.25
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

CUBE_SIZE_1 = 0.04
CUBE_MASS_1 = 0.05
CUBE_X_1, CUBE_Y_1 = SQUARE_CENTERS["left"]
CUBE_Z_1 = TABLE_TOP_Z + CUBE_SIZE_1 / 2


CUBE_SIZE_2 = 0.04
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
    else:
        print("  OK")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="empty", help="Gazebo world name")
    args = parser.parse_args()

    for square_name, corners in SQUARE_CORNERS.items():
        for i, (x, y) in enumerate(corners):
            spawn(args.world, f"apriltag_marker_{square_name}_{i}", marker_sdf(TAG_TEXTURES[i]), x, y, MARKER_Z)
    # spawn(args.world, "ball", ball_sdf(), BALL_X, BALL_Y, BALL_Z)
    spawn(args.world, "cube_1", cube_sdf("cube_1", CUBE_SIZE_1, CUBE_MASS_1, (0.1, 0.3, 0.9)), CUBE_X_1, CUBE_Y_1, CUBE_Z_1)
    spawn(args.world, "cube_2", cube_sdf("cube_2", CUBE_SIZE_2, CUBE_MASS_2, (0.15, 0.7, 0.2)), CUBE_X_2, CUBE_Y_2, CUBE_Z_2)


if __name__ == "__main__":
    main()
