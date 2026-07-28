#!/usr/bin/env python3
"""
Direct pymycobot workspace and forward-kinematics experiment for myCobot 280 Pi.

This script does NOT use ROS, /joint_states, ros2_control, MoveIt, Gazebo, or
joint_state_publisher. It communicates directly with the physical robot using:

    from pymycobot.mycobot280 import MyCobot280

It performs two related tasks:

1. Theoretical five-angle workspace grid (offline, no movement)
   It evaluates all 5^6 = 15,625 joint combinations using URDF-based forward
   kinematics. This gives a coarse approximation of the robot's workspace.

2. Physical five-angle single-joint sweeps (30 movements)
   It moves one joint through five angles while the other five remain at a
   fixed pose. After every movement it reads the real angles with get_angles(),
   computes each joint's error, and calculates commanded/measured FK positions.

IMPORTANT:
- Run this on the Raspberry Pi/controller that has /dev/serial0.
- Stop mycobot_bridge.py and every other program using /dev/serial0 first.
- Start with small angles and one joint. Keep the emergency stop accessible.
- Serial port and baud rate are already set to /dev/serial0 and 1000000.
- The custom URDF is found automatically from the ROS package/workspace.
- The 30 physical points validate cross-sections of the workspace; they are not
  the complete continuous workspace.
"""

import argparse
import csv
import itertools
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

# Official Elephant Robotics Python API used to communicate directly with
# the physical myCobot 280 controller over the serial port.
from pymycobot.mycobot280 import MyCobot280

# Friendly joint labels used in terminal output and CSV column names.
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
# Exact movable-joint names from the custom URDF. The order must match the
# six-angle list returned by pymycobot.get_angles().
URDF_JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
# Five default test values in degrees. Use smaller values for the first
# physical test if the robot is close to a table or other obstruction.
DEFAULT_ANGLES = [-20.0, -10.0, 0.0, 10.0, 20.0]

# Robot communication settings are fixed here, so they do not need to be
# supplied every time the script is run. These match the existing project.
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 1000000
DEFAULT_URDF_RELATIVE = os.path.join(
    "urdf", "mycobot_280_pi",
    "mycobot_280_pi_camera_flange_plus_gripper_unchanged_transforms.urdf",
)


@dataclass
class UrdfJoint:
    """Information needed from one URDF joint for forward kinematics."""
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: Optional[float]
    upper: Optional[float]


def vector(text: Optional[str], default: Tuple[float, float, float]) -> np.ndarray:
    """Convert a URDF string such as ``"0 0 0.1"`` into a NumPy vector."""
    return np.array([float(v) for v in text.split()], dtype=float) if text else np.array(default, dtype=float)


def rx(a):
    """Return a 3x3 rotation matrix for rotation about the X-axis."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def ry(a):
    """Return a 3x3 rotation matrix for rotation about the Y-axis."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rz(a):
    """Return a 3x3 rotation matrix for rotation about the Z-axis."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """Create the fixed 4x4 transform stored in a URDF joint origin."""
    t = np.eye(4)
    t[:3, :3] = rz(rpy[2]) @ ry(rpy[1]) @ rx(rpy[0])
    t[:3, 3] = xyz
    return t


def axis_transform(axis: np.ndarray, angle: float) -> np.ndarray:
    """Create a 4x4 rotation transform about any normalized joint axis."""
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, d = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    r = np.array([
        [c + x*x*d, x*y*d - z*s, x*z*d + y*s],
        [y*x*d + z*s, c + y*y*d, y*z*d - x*s],
        [z*x*d - y*s, z*y*d + x*s, c + z*z*d],
    ])
    t = np.eye(4)
    t[:3, :3] = r
    return t


class UrdfFK:
    """Minimal URDF parser and forward-kinematics calculator."""

    def __init__(self, path: str, base_link: str, end_link: str):
        self.path = path
        self.base_link = base_link
        self.end_link = end_link
        self.joints = self._read(path)
        self.chain = self._find_chain(base_link, end_link)

    def _read(self, path: str) -> List[UrdfJoint]:
        """Read all URDF joints and keep only fields required by FK."""
        root = ET.parse(path).getroot()
        result = []
        for e in root.findall("joint"):
            p, c = e.find("parent"), e.find("child")
            if p is None or c is None:
                continue
            o, a, lim = e.find("origin"), e.find("axis"), e.find("limit")
            result.append(UrdfJoint(
                name=e.get("name", ""), joint_type=e.get("type", "fixed"),
                parent=p.get("link"), child=c.get("link"),
                xyz=vector(o.get("xyz") if o is not None else None, (0, 0, 0)),
                rpy=vector(o.get("rpy") if o is not None else None, (0, 0, 0)),
                axis=vector(a.get("xyz") if a is not None else None, (1, 0, 0)),
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
            ))
        return result

    def _find_chain(self, base: str, end: str) -> List[UrdfJoint]:
        """Find the ordered joint path from the selected base to end link."""
        children: Dict[str, List[UrdfJoint]] = {}
        for joint in self.joints:
            children.setdefault(joint.parent, []).append(joint)

        def dfs(link, path):
            if link == end:
                return path
            for joint in children.get(link, []):
                found = dfs(joint.child, path + [joint])
                if found is not None:
                    return found
            return None

        chain = dfs(base, [])
        if chain is None:
            raise ValueError(f"No URDF chain from {base} to {end}")
        return chain

    def validate(self, degrees: List[float]) -> None:
        """Reject a six-angle pose if it exceeds any URDF joint limit."""
        values = dict(zip(URDF_JOINT_NAMES, [math.radians(v) for v in degrees]))
        errors = []
        for joint in self.chain:
            if joint.name not in values:
                continue
            value = values[joint.name]
            if joint.lower is not None and value < joint.lower - 1e-9:
                errors.append(f"{joint.name}: {math.degrees(value):.1f} < {math.degrees(joint.lower):.1f}")
            if joint.upper is not None and value > joint.upper + 1e-9:
                errors.append(f"{joint.name}: {math.degrees(value):.1f} > {math.degrees(joint.upper):.1f}")
        if errors:
            raise ValueError("Joint-limit violation: " + "; ".join(errors))

    def calculate(self, degrees: List[float]) -> np.ndarray:
        """Return the base-to-end-effector 4x4 FK transform for six angles."""
        values = dict(zip(URDF_JOINT_NAMES, [math.radians(v) for v in degrees]))
        t = np.eye(4)
        for joint in self.chain:
            t = t @ origin_transform(joint.xyz, joint.rpy)
            if joint.joint_type in ("revolute", "continuous"):
                t = t @ axis_transform(joint.axis, values.get(joint.name, 0.0))
        return t


def resolve_urdf() -> str:
    """Automatically locate the project's custom myCobot URDF.

    Search order:
    1. ROS 2's installed ``mycobot_description`` package, when available.
    2. Common source-workspace locations under the current user's home folder.
    3. A targeted recursive search inside likely ROS workspaces.

    The exact preferred filename is selected, so the user normally does not
    need to enter a URDF path.
    """
    preferred_name = os.path.basename(DEFAULT_URDF_RELATIVE)
    candidates = []

    # First try the ROS 2 package index. This works after the workspace has
    # been built and sourced, for example: source ~/colcon_ws/install/setup.bash
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory("mycobot_description")
        candidates.append(os.path.join(package_share, DEFAULT_URDF_RELATIVE))
    except Exception:
        # The script can still work without ament_index_python by searching the
        # common source-workspace paths below.
        pass

    home = os.path.expanduser("~")
    workspace_roots = [
        os.path.join(home, "colcon_ws"),
        os.path.join(home, "ros2_ws"),
        os.path.join(home, "galactic_ws"),
        os.path.join(home, "mycobot_ws"),
        os.getcwd(),
        os.path.dirname(os.path.abspath(__file__)),
    ]

    # Check the most likely project layouts directly before doing any recursive
    # search. This is fast and covers both mycobot_ros2/mycobot_description and
    # a standalone mycobot_description package.
    for root in workspace_roots:
        candidates.extend([
            os.path.join(root, "src", "mycobot_ros2", "mycobot_description", DEFAULT_URDF_RELATIVE),
            os.path.join(root, "src", "mycobot_description", DEFAULT_URDF_RELATIVE),
            os.path.join(root, "mycobot_ros2", "mycobot_description", DEFAULT_URDF_RELATIVE),
            os.path.join(root, "mycobot_description", DEFAULT_URDF_RELATIVE),
        ])

    checked = set()
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if candidate in checked:
            continue
        checked.add(candidate)
        if os.path.isfile(candidate):
            return candidate

    # Last fallback: search only likely ROS workspace roots for the exact custom
    # filename. Directories such as build/install/log are skipped where possible.
    for root in workspace_roots:
        if not os.path.isdir(root):
            continue
        for current_root, directories, files in os.walk(root):
            directories[:] = [d for d in directories if d not in {"build", "log", ".git", "__pycache__"}]
            if preferred_name in files:
                return os.path.join(current_root, preferred_name)

    searched = "\n  - ".join(os.path.abspath(p) for p in workspace_roots)
    raise FileNotFoundError(
        "Could not automatically find the custom URDF named:\n"
        f"  {preferred_name}\n"
        "Searched these workspace locations:\n  - " + searched +
        "\nBuild/source the workspace or place the URDF inside one of these workspaces."
    )



def xyz_mm(transform: np.ndarray) -> Tuple[float, float, float]:
    """Extract XYZ from a transform and convert metres to millimetres."""
    p = transform[:3, 3] * 1000.0
    return float(p[0]), float(p[1]), float(p[2])


def read_average_angles(robot: MyCobot280, samples: int, delay: float) -> List[float]:
    """Average several get_angles() replies to reduce small reading noise."""
    readings = []
    for _ in range(samples):
        value = robot.get_angles()
        if isinstance(value, (list, tuple)) and len(value) == 6:
            readings.append([float(v) for v in value])
        time.sleep(delay)
    if not readings:
        raise RuntimeError("get_angles() returned no valid six-angle reading")
    return np.mean(np.array(readings, dtype=float), axis=0).tolist()


def wait_until_reached(robot: MyCobot280, target: List[float], timeout: float, tolerance: float) -> List[float]:
    """Poll get_angles() until all joints are close to target or timeout expires."""
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        value = robot.get_angles()
        if isinstance(value, (list, tuple)) and len(value) == 6:
            latest = [float(v) for v in value]
            if max(abs(a - b) for a, b in zip(latest, target)) <= tolerance:
                return latest
        time.sleep(0.2)
    if latest is None:
        raise RuntimeError("Robot did not return angles while waiting for movement")
    return latest


def generate_theoretical_grid(fk: UrdfFK, angles: List[float], output_dir: str) -> str:
    """Evaluate every 5-angle combination: 5^6 = 15,625 FK points.

    This is an offline calculation only. It never sends movement commands to
    the physical robot.
    """
    path = os.path.join(output_dir, "theoretical_workspace_5x6.csv")
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(JOINT_NAMES + ["x_mm", "y_mm", "z_mm"])
        for pose in itertools.product(angles, repeat=6):
            pose = list(pose)
            fk.validate(pose)
            writer.writerow(pose + list(xyz_mm(fk.calculate(pose))))
    return path


def physical_sweep(robot: MyCobot280, fk: UrdfFK, args, output_dir: str) -> str:
    """Run the physical one-joint-at-a-time sweep and save all errors."""
    path = os.path.join(output_dir, "physical_workspace_and_joint_errors.csv")
    headers = ["test", "swept_joint", "requested_sweep_deg"]
    headers += [f"commanded_{j}_deg" for j in JOINT_NAMES]
    headers += [f"actual_{j}_deg" for j in JOINT_NAMES]
    headers += [f"error_{j}_deg" for j in JOINT_NAMES]
    headers += ["commanded_x_mm", "commanded_y_mm", "commanded_z_mm",
                "measured_x_mm", "measured_y_mm", "measured_z_mm",
                "position_error_mm", "mean_abs_joint_error_deg",
                "max_abs_joint_error_deg", "joint_rmse_deg"]

    with open(path, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=headers)
        writer.writeheader()
        test_number = 0

        # Each selected joint is swept separately. The other five joints stay
        # at args.fixed_degrees, producing a safe and understandable cross-section.
        for joint_number in args.joints:
            print(f"\n--- Sweeping Joint {joint_number} ---")
            for sweep_angle in args.angles:
                test_number += 1
                # Copy the fixed pose, then replace only the joint being swept.
                target = list(args.fixed_degrees)
                target[joint_number - 1] = sweep_angle
                fk.validate(target)

                print(f"Test {test_number}: command {target}")
                # send_angles() is the established pymycobot command that asks
                # the physical controller to move all six joints simultaneously.
                robot.send_angles(target, args.speed)
                wait_until_reached(robot, target, args.timeout, args.tolerance)
                time.sleep(args.settle_time)
                # get_angles() is called inside read_average_angles(). These are
                # the joint angles reported by the physical robot controller.
                actual = read_average_angles(robot, args.samples, args.sample_delay)

                # Signed joint error: reported physical angle minus command.
                # Example: commanded 45 deg, actual 44 deg -> error = -1 deg.
                errors = [a - c for a, c in zip(actual, target)]
                abs_errors = np.abs(np.array(errors))
                # Compute FK twice: once from the commanded angles and once from
                # the measured angles. Their difference shows how joint error
                # changes the calculated end-effector position.
                commanded_xyz = np.array(xyz_mm(fk.calculate(target)))
                measured_xyz = np.array(xyz_mm(fk.calculate(actual)))

                row = {
                    "test": test_number,
                    "swept_joint": joint_number,
                    "requested_sweep_deg": sweep_angle,
                    "commanded_x_mm": commanded_xyz[0],
                    "commanded_y_mm": commanded_xyz[1],
                    "commanded_z_mm": commanded_xyz[2],
                    "measured_x_mm": measured_xyz[0],
                    "measured_y_mm": measured_xyz[1],
                    "measured_z_mm": measured_xyz[2],
                    "position_error_mm": float(np.linalg.norm(measured_xyz - commanded_xyz)),
                    "mean_abs_joint_error_deg": float(np.mean(abs_errors)),
                    "max_abs_joint_error_deg": float(np.max(abs_errors)),
                    "joint_rmse_deg": float(math.sqrt(np.mean(np.square(errors)))),
                }
                for i, name in enumerate(JOINT_NAMES):
                    row[f"commanded_{name}_deg"] = target[i]
                    row[f"actual_{name}_deg"] = actual[i]
                    row[f"error_{name}_deg"] = errors[i]
                writer.writerow(row)
                file.flush()

                print("  actual:", [round(v, 3) for v in actual])
                print("  errors:", [round(v, 3) for v in errors])
                print("  measured FK XYZ mm:", [round(v, 2) for v in measured_xyz])

    return path


def parser() -> argparse.ArgumentParser:
    """Create all command-line options used by the experiment."""
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speed", type=int, default=20, help="pymycobot movement speed, 1-100")
    p.add_argument("--angles", type=float, nargs=5, default=DEFAULT_ANGLES)
    p.add_argument("--fixed-degrees", type=float, nargs=6, default=[0.0] * 6)
    p.add_argument("--joints", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    p.add_argument("--timeout", type=float, default=8.0)
    p.add_argument("--tolerance", type=float, default=2.0)
    p.add_argument("--settle-time", type=float, default=1.0)
    p.add_argument("--samples", type=int, default=5)
    p.add_argument("--sample-delay", type=float, default=0.1)
    p.add_argument("--base-link", default="g_base")
    p.add_argument("--end-link", default="gripper_base")
    p.add_argument("--output-dir", default=None)
    p.add_argument("--skip-theoretical-grid", action="store_true")
    p.add_argument("--dry-run", action="store_true", help="calculate and print the plan without moving the robot")
    return p


def main() -> int:
    """Validate options, generate FK data, and optionally move the robot."""
    args = parser().parse_args()
    if not 1 <= args.speed <= 100:
        raise ValueError("--speed must be between 1 and 100")
    if any(j < 1 or j > 6 for j in args.joints):
        raise ValueError("--joints must contain numbers from 1 to 6")

    urdf_path = resolve_urdf()
    fk = UrdfFK(urdf_path, args.base_link, args.end_link)
    output_dir = os.path.abspath(args.output_dir or os.path.expanduser(
        "~/mycobot_pymycobot_workspace/" + datetime.now().strftime("%Y%m%d_%H%M%S")
    ))
    os.makedirs(output_dir, exist_ok=True)

    print("URDF:", urdf_path)
    print(f"Serial connection: {SERIAL_PORT} at {BAUD_RATE} baud")
    print("Output:", output_dir)
    print("Five angles:", args.angles)

    if not args.skip_theoretical_grid:
        print("Generating 15,625-point theoretical five-angle FK grid...")
        print("Saved:", generate_theoretical_grid(fk, args.angles, output_dir))

    physical_pose_count = len(args.joints) * 5
    print(f"Physical plan: {physical_pose_count} poses; one joint changes at a time.")
    if args.dry_run:
        print("Dry run complete. No command was sent to the robot.")
        return 0

    print("WARNING: the physical robot will now move.")
    print("Confirm the workspace is clear and no ROS bridge is using /dev/serial0.")
    print("Press Ctrl+C now to cancel. Movement starts in 5 seconds...")
    time.sleep(5.0)

    print("Connecting directly with pymycobot...")
    # Open the direct serial connection. No ROS node or /joint_states topic is
    # involved in this version, so no other process may use the same serial port.
    robot = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1.0)
    # Confirm communication by requesting the robot controller's current angles.
    initial = robot.get_angles()
    if not isinstance(initial, (list, tuple)) or len(initial) != 6:
        raise RuntimeError("Connected, but get_angles() did not return six joint angles")
    print("Initial physical angles:", initial)
    print("Saved:", physical_sweep(robot, fk, args, output_dir))
    print("Experiment complete.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nStopped by user. The robot receives no further commands.", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
