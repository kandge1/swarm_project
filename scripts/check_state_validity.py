#!/usr/bin/env python3
"""
Diagnostic: ask MoveIt directly whether the robot's CURRENT state is
self-collision-free, and if not, list every colliding link pair by name.
No guessing from RViz colors -- this is the ground truth the planner and
IK service are actually using. Run with demo.launch.py already up.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

GROUP_NAME = "arm_group"


class StateValidityProbe(Node):
    def __init__(self):
        super().__init__("state_validity_probe")
        self.client = self.create_client(GetStateValidity, "/check_state_validity")
        self._latest_joint_state = None
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg

    def wait_for_joint_state(self, timeout_sec=5.0):
        end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self._latest_joint_state is None and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_joint_state

    def check(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/check_state_validity service not available")
            return None

        joint_state = self.wait_for_joint_state()
        if joint_state is None:
            self.get_logger().error("Never received a /joint_states message")
            return None

        req = GetStateValidity.Request()
        req.group_name = GROUP_NAME
        req.robot_state = RobotState()
        req.robot_state.joint_state = joint_state
        req.robot_state.is_diff = False

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    probe = StateValidityProbe()

    print("\nChecking CURRENT robot state for self-collision "
          "(no target pose involved -- this is just 'is the robot ok right now')\n")

    response = probe.check()
    if response is None:
        print("Could not complete the check -- see errors above.")
        rclpy.shutdown()
        return

    if response.valid:
        print("Current state is VALID -- no self-collision detected.")
        print("If IK is still failing everywhere, the problem is elsewhere "
              "(e.g. workspace reach), not a standing self-collision.")
    else:
        print("Current state is INVALID -- self-collision detected.")
        print(f"\n{len(response.contacts)} contact(s):\n")
        for contact in response.contacts:
            print(f"  {contact.contact_body_1}  <-->  {contact.contact_body_2}")
        print("\nThese pairs need to either be fixed geometrically or added "
              "to firefighter.srdf's <disable_collisions> list if they're "
              "expected/adjacent contact, not a real problem.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()