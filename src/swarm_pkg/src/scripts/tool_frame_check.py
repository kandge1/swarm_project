#!/usr/bin/env python3
"""
tool_frame_check.py -- print where the URDF thinks the camera and the gripper
point, in plain world directions, at a given joint configuration.

WHY THIS EXISTS: the URDF and the physical robot disagree about the tool
assembly's orientation, and "the camera is pointing the wrong way in RViz" is
not a number you can correct a transform with. This turns it into one.

It computes forward kinematics straight from the URDF's joint origins -- no
ROS, no MoveIt, no running robot -- and reports, for each frame of interest,
which way its axes point in the world frame, in words ("+X world, i.e. AWAY
from the base"). Run it, look at the real arm in the same pose, and any
disagreement is then a specific rotation rather than an impression.

  # the pose the real arm calls "home" (joint6output compensated -45 deg)
  ./tool_frame_check.py --home

  # all joints at zero, where the mount's own offset is visible
  ./tool_frame_check.py --zeros

  # anything else
  ./tool_frame_check.py --joints 0 0 0 0 0 -45

WHAT TO COMPARE. For each pose, check on the real robot:
  1. which way the CAMERA lens faces
  2. which way the GRIPPER's notch (the servo side) faces
and compare against the "camera lens (optical +Z)" and "gripper notch" lines
this prints. If both are flipped together, the error is a rotation about the
tool's own axis, and the fix is a single term in
`joint6output_to_camera_flange`'s rpy. If only one is flipped, the error is
below the flange and the fix is in `camera_flange_to_gripper_base` or
`camera_flange_to_camera_link` instead -- which is exactly the distinction
that cannot be made by eye from a description.
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..",
    "mycobot_description", "urdf", "mycobot_280_pi",
    "mycobot_280_pi_camera_flange_plus_gripper_unchanged_transforms.urdf",
)

# The pose pick_place.py calls home. joint6output is -45 deg because the
# physical mount sits 45 deg askew at zero -- that compensation is the thing
# under investigation here, so it is spelled out rather than imported.
HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, -45.0]

ARM_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Frames worth reporting, and which of their local axes is the "business" one.
FRAMES_OF_INTEREST = [
    ("joint6_flange", "Z", "flange approach axis (what IK aims)"),
    ("camera_flange", "Z", "camera flange"),
    ("wrist_camera_optical_frame", "Z", "camera lens (optical +Z = view direction)"),
    ("gripper_base", "Y", "gripper notch / servo side"),
    ("gripper_base", "Z", "gripper approach (fingers close along X)"),
]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return matmul(rz, matmul(ry, rx))


def axis_angle_to_matrix(axis, angle):
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / norm, y / norm, z / norm
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}
    children = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0").split()
                                  if origin is not None else "0 0 0".split())]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0").split()
                                  if origin is not None else "0 0 0".split())]
        axis_el = j.find("axis")
        axis = ([float(v) for v in axis_el.get("xyz").split()]
                if axis_el is not None else [0.0, 0.0, 1.0])
        rec = {
            "name": j.get("name"), "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz, "rpy": rpy, "axis": axis,
        }
        joints[j.get("name")] = rec
        children[rec["child"]] = rec
    return joints, children


def chain_to(children, link):
    """Joint records from the root down to `link`, root-first."""
    chain = []
    while link in children:
        rec = children[link]
        chain.append(rec)
        link = rec["parent"]
    return list(reversed(chain))


def frame_rotation(children, link, values):
    """World rotation of `link`, given {joint_name: radians}."""
    rotation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for rec in chain_to(children, link):
        rotation = matmul(rotation, rpy_to_matrix(*rec["rpy"]))
        if rec["type"] in ("revolute", "continuous"):
            angle = values.get(rec["name"], 0.0)
            rotation = matmul(rotation, axis_angle_to_matrix(rec["axis"], angle))
    return rotation


def frame_transform(children, link, values, relative_to=None):
    """(rotation, translation) of `link`, given {joint_name: radians}.

    relative_to=None measures from the URDF root; pass a link name to measure
    from that link instead. Unlike frame_rotation this carries translation too,
    which is what any "where is the lens relative to the flange" question needs.
    """
    chain = chain_to(children, link)
    if relative_to is not None:
        base = chain_to(children, relative_to)
        if [rec["name"] for rec in base] != [rec["name"] for rec in chain[:len(base)]]:
            raise ValueError("%s is not an ancestor of %s" % (relative_to, link))
        chain = chain[len(base):]

    rotation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    translation = [0.0, 0.0, 0.0]
    for rec in chain:
        offset = rec["xyz"]
        translation = [translation[i] + sum(rotation[i][k] * offset[k] for k in range(3))
                       for i in range(3)]
        rotation = matmul(rotation, rpy_to_matrix(*rec["rpy"]))
        if rec["type"] in ("revolute", "continuous"):
            angle = values.get(rec["name"], 0.0)
            rotation = matmul(rotation, axis_angle_to_matrix(rec["axis"], angle))
    return rotation, translation


def flange_to_camera(urdf_path=DEFAULT_URDF):
    """(offset, view_axis) from joint6_flange to the camera lens, in the
    FLANGE's own frame.

    Every joint between joint6_flange and wrist_camera_optical_frame is fixed,
    so both are constants -- they do not depend on the arm's configuration, and
    computing them once is enough.

    This is what turns "point the camera at the zone centre" into a flange
    target: the lens sits to one side of the flange, so commanding the flange to
    the zone centre points the CAMERA somewhere else. Note this only affects
    FRAMING. The block's measured position comes from the tag homography and is
    unaffected by getting this wrong -- a bad offset loses the tags out of
    frame, it does not bias the answer. See APRIL_TAGS.md.
    """
    _, children = parse_urdf(urdf_path)
    rotation, offset = frame_transform(
        children, "wrist_camera_optical_frame", {}, relative_to="joint6_flange")
    # Optical +Z is the view direction (ROS convention), i.e. the third column.
    view_axis = [rotation[i][2] for i in range(3)]
    return offset, view_axis


def describe(vector):
    """A world direction in words, so it can be checked against the real arm."""
    x, y, z = vector
    names = []
    for value, positive, negative in ((x, "+X (away from base, front)",
                                       "-X (toward base, back)"),
                                      (y, "+Y (left)", "-Y (right)"),
                                      (z, "+Z (UP)", "-Z (DOWN)")):
        if abs(value) >= 0.15:
            names.append("{} {:.2f}".format(positive if value > 0 else negative,
                                            abs(value)))
    dominant = max(range(3), key=lambda i: abs(vector[i]))
    label = ["X", "Y", "Z"][dominant]
    sign = "+" if vector[dominant] > 0 else "-"
    plain = {"+Z": "UP", "-Z": "DOWN", "+X": "FORWARD (away from base)",
             "-X": "BACK (toward base)", "+Y": "LEFT", "-Y": "RIGHT"}[sign + label]
    return "{:<28} [{}]".format("mostly " + plain, ", ".join(names) or "mixed")


def report(children, values, title):
    print("=" * 78)
    print(title)
    print("  joints (deg): {}".format(
        [round(math.degrees(values[n]), 1) for n in ARM_JOINTS]))
    print("=" * 78)
    for link, axis_name, label in FRAMES_OF_INTEREST:
        rotation = frame_rotation(children, link, values)
        if rotation is None:
            print("  {:<44} <link not found>".format(label))
            continue
        col = {"X": 0, "Y": 1, "Z": 2}[axis_name]
        vector = [rotation[0][col], rotation[1][col], rotation[2][col]]
        print("  {:<44} {} {}".format(label, axis_name + ":", describe(vector)))
    print()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--home", action="store_true",
                       help="the pose pick_place.py calls home (joint6output -45 deg)")
    group.add_argument("--zeros", action="store_true",
                       help="all six joints at zero")
    group.add_argument("--joints", nargs=6, type=float, metavar="DEG",
                       help="six joint angles in degrees")
    parser.add_argument("--urdf", default=DEFAULT_URDF,
                        help="URDF to read (default: the one the moveit config loads)")
    args = parser.parse_args()

    if not os.path.exists(args.urdf):
        print("URDF not found: {}".format(args.urdf), file=sys.stderr)
        return 1
    _, children = parse_urdf(args.urdf)

    if args.zeros:
        degrees, title = [0.0] * 6, "ALL JOINTS ZERO"
    elif args.home:
        degrees, title = list(HOME_DEG), "HOME POSE (joint6output = -45 deg)"
    else:
        degrees, title = list(args.joints), "CUSTOM POSE"

    values = {n: math.radians(d) for n, d in zip(ARM_JOINTS, degrees)}
    report(children, values, title)

    print("Now look at the REAL arm in this pose and answer two questions:")
    print("  1. which way does the CAMERA LENS face?")
    print("  2. which way does the GRIPPER NOTCH (servo side) face?")
    print()
    print("If BOTH disagree with the lines above, the error is a rotation about")
    print("the tool's own axis -> fix joint6output_to_camera_flange's rpy.")
    print("If only ONE disagrees, the error is below the flange -> fix")
    print("camera_flange_to_camera_link or camera_flange_to_gripper_base.")
    print("Either way the correction is then a specific angle, not a guess.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
