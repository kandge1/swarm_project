#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class DiffDriveKinematics(Node):
    def __init__(self):
        super().__init__("diff_drive_kinematics")

        # Robot geometry
        self.declare_parameter("wheel_radius", 0.14)   # meters
        self.declare_parameter("wheel_base", 0.413)    # meters

        # Topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_vel_topic", "/wheelVel")

        # Joint names
        self.declare_parameter("left_wheel_joint", "joint_wheel_left")
        self.declare_parameter("right_wheel_joint", "joint_wheel_right")

        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_base").value)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        wheel_vel_topic = self.get_parameter("wheel_vel_topic").value
        self.left_joint = self.get_parameter("left_wheel_joint").value
        self.right_joint = self.get_parameter("right_wheel_joint").value

        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.wheel_pub = self.create_publisher(JointState, wheel_vel_topic, 10)

        self.get_logger().info("Diff Drive Kinematics Started")
        self.get_logger().info(f"Listening on {cmd_vel_topic}, publishing to {wheel_vel_topic}")

    def cmd_vel_callback(self, msg: Twist):

        v = msg.linear.x
        w = msg.angular.z

        # Convert (v, w) to wheel angular velocities (rad/s)
        w_left = (2.0 * v - w * self.L) / (2.0 * self.R)
        w_right = (2.0 * v + w * self.L) / (2.0 * self.R)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.velocity = [w_left, w_right]

        self.wheel_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
