#!/usr/bin/env python3
"""Bridge: mirrors ros2_control's mock-hardware /joint_states onto the real
mycobot 280pi over pymycobot, so pick_place.py's existing action-server-based
control flow (unchanged) drives the physical arm instead of the RViz mock.

Run this alongside demo.launch.py's existing mock-hardware setup (NOT
gazebo.launch.py) -- see the "RVIZ Everything" section of myscript.txt.
Neither demo.launch.py nor firefighter.ros2_control.xacro need any changes;
this just adds a third consumer of the same /joint_states topic that
controller_manager and MoveIt already use.

Adapted from mycobot_280pi's own slider_control_adaptive_gripper.py (which
does the same job for RViz's joint_state_publisher_gui sliders), but reads
/joint_states by JOINT NAME instead of fixed index, matching pick_place.py's
HOME_RADIANS ordering.

    source ~/swarm/swarm_project/source/ros2_ws/install/setup.bash
    python3 ~/swarm/swarm_project/scripts/real_arm_bridge.py
    # override the serial port/baud if yours differs from the mycobot 280pi
    # default (internal UART, not USB):
    python3 ~/swarm/swarm_project/scripts/real_arm_bridge.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200

KNOWN LIMITATIONS (mock hardware has no real feedback loop):
  - No real position feedback: MoveIt/pick_place.py believe every commanded
    move succeeded immediately, exactly as they already do against the
    RViz-only mock today -- there is no closed loop to the real encoders.
  - No real force feedback: gripper_controller's effort state interface
    stays at its mock initial_value (0.0) forever, so
    gripper_close_until_contact() in pick_place.py will never detect
    contact and always fully closes to GRIPPER_CLOSED (this is the
    documented "no effort readings received" fallback already built into
    that function). This relies on the adaptive gripper's own mechanical
    compliance, not software force detection -- test it closing on nothing,
    then on a block, before trusting the full pick-and-place sequence.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pymycobot
from packaging import version

MIN_REQUIRE_VERSION = "3.6.1"
if version.parse(pymycobot.__version__) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        f"pymycobot >= {MIN_REQUIRE_VERSION} required, found {pymycobot.__version__}. "
        "Upgrade with: pip3 install -U pymycobot"
    )
from pymycobot import MyCobot280  # noqa: E402

# Must match pick_place.py's HOME_RADIANS key order.
ARM_JOINT_ORDER = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"
# URDF joint limits for gripper_controller -- same mapping mycobot_280pi's
# own listen_real.py uses to turn a URDF radian value into the 0-100 range
# pymycobot's adaptive-gripper API expects. NOT pick_place.py's operational
# GRIPPER_OPEN/GRIPPER_CLOSED subset -- this must span the full URDF range.
GRIPPER_MIN = -0.74
GRIPPER_MAX = 0.15
GRIPPER_TYPE = 1  # adaptive gripper -- matches mycobot_280pi's real-hardware scripts

ARM_SEND_SPEED = 25     # 0-100, matches slider_control_adaptive_gripper.py
GRIPPER_SEND_SPEED = 80  # 0-100, matches listen_real.py


class RealArmBridge(Node):
    def __init__(self):
        super().__init__("real_arm_bridge")
        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 1000000)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info(f"Connecting to real mycobot on {port} @ {baud}...")
        self.mc = MyCobot280(port, baud)
        self.mc.set_fresh_mode(1)
        self.get_logger().info("Connected. Mirroring /joint_states to the real arm.")

        self._last_sent_degrees = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in ARM_JOINT_ORDER):
            return  # partial update (e.g. gripper-only tick); wait for a full one

        degrees = [round(math.degrees(positions[name]), 2) for name in ARM_JOINT_ORDER]
        if degrees != self._last_sent_degrees:
            self._last_sent_degrees = degrees
            self.mc.send_angles(degrees, ARM_SEND_SPEED, _async=True)

        if GRIPPER_JOINT in positions:
            g = positions[GRIPPER_JOINT]
            fraction = (g - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
            gripper_value = int(round(max(0.0, min(1.0, fraction)) * 100))
            self.mc.set_gripper_value(gripper_value, GRIPPER_SEND_SPEED, gripper_type=GRIPPER_TYPE)


def main():
    rclpy.init()
    node = RealArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
