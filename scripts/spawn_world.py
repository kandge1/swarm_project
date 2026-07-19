#!/usr/bin/env python3
"""Spawn a table, an AprilTag marker, and a ball into the running Gazebo world.

Uses the same mechanism gazebo.launch.py uses to spawn the robot itself:
the `ros_gz_sim create` CLI tool, which calls the world's gz-transport
/world/<world>/create service. Each object is a self-contained SDF <model>
string with its world pose baked into <pose>, so nothing depends on where
`create`'s own -x/-y/-z flags would otherwise place it.

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

# Table: 4ft x 4ft, centered at the origin, top surface flush with the
# ground (z=0) to match the work-surface height annulus_test.py already
# assumes (its z range is 0.13-0.24m measured from the arm base at z=0).
TABLE_SIZE = 40 * INCH
TABLE_THICKNESS = 0.02
TABLE_TOP_Z = 0.02

# 4 AprilTag markers (1in x 1in each), one at each corner of a 10cm x 10cm
# square whose center is 15cm from the origin along +X, resting on the table.
MARKER_SIZE = 1 * INCH
MARKER_THICKNESS = 0.001
MARKER_Z = TABLE_TOP_Z + MARKER_THICKNESS / 2
SQUARE_SIZE = 0.10
SQUARE_CENTER_X = 0.15
SQUARE_CENTER_Y = 0.0
_half = SQUARE_SIZE / 2
MARKER_CORNERS = [
    (SQUARE_CENTER_X + sx * _half, SQUARE_CENTER_Y + sy * _half)
    for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]
]

# Ball: sits at the center of the square, resting directly on the table.
BALL_RADIUS = 0.02
BALL_MASS = 0.05
BALL_X = SQUARE_CENTER_X
BALL_Y = SQUARE_CENTER_Y
BALL_Z = TABLE_TOP_Z + BALL_RADIUS


def table_sdf() -> str:
    return f"""
<sdf version="1.9">
  <model name="table">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>{TABLE_SIZE} {TABLE_SIZE} {TABLE_THICKNESS}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{TABLE_SIZE} {TABLE_SIZE} {TABLE_THICKNESS}</size></box>
        </geometry>
        <material>
          <ambient>0.55 0.35 0.2 1</ambient>
          <diffuse>0.55 0.35 0.2 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


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

    table_z = TABLE_TOP_Z - TABLE_THICKNESS / 2
    spawn(args.world, "table", table_sdf(), 0, 0, table_z)
    for i, (x, y) in enumerate(MARKER_CORNERS):
        spawn(args.world, f"apriltag_marker_{i}", marker_sdf(TAG_TEXTURES[i]), x, y, MARKER_Z)
    spawn(args.world, "ball", ball_sdf(), BALL_X, BALL_Y, BALL_Z)


if __name__ == "__main__":
    main()
