#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(x, y, z, w):
    """Return yaw angle from quaternion (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Generate quaternion (x,y,z,w) from yaw angle."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def wrap_angle(angle):
    """Wrap angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SetpointController(Node):
    def __init__(self):
        super().__init__("setpoint_controller")

        # Control gains
        self.declare_parameter("k_rho", 1.0)
        self.declare_parameter("k_alpha", 2.0)
        self.declare_parameter("k_beta", -0.5)

        # Thresholds for stopping motion
        self.declare_parameter("rho_stop", 0.02)
        self.declare_parameter("yaw_stop", 0.2)

        # Maximum speed limits
        self.declare_parameter("v_max", 0.8)
        self.declare_parameter("w_max", 1.5)

        # Topic and frame settings
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_parent_frame", "map")
        self.declare_parameter("goal_child_frame", "goal")
        self.declare_parameter("goal_topic", "/goal_pose")

        # Load parameters
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_alpha = float(self.get_parameter("k_alpha").value)
        self.k_beta = float(self.get_parameter("k_beta").value)

        self.rho_stop = float(self.get_parameter("rho_stop").value)
        self.yaw_stop = float(self.get_parameter("yaw_stop").value)

        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)

        odom_topic = self.get_parameter("odom_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.goal_parent_frame = self.get_parameter("goal_parent_frame").value
        self.goal_child_frame = self.get_parameter("goal_child_frame").value
        goal_topic = self.get_parameter("goal_topic").value

        # Internal state variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.odom_received = False

        # Goal state: start "idle" (no active goal)
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.th_goal = 0.0
        self.goal_reached = True

        # Logging timer
        self.last_log_time = self.get_clock().now()
        self.log_period = 1.0

        # ROS2 publishers/subscribers
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 50 Hz control loop
        self.timer = self.create_timer(1.0 / 50.0, self.control_loop)

        self.get_logger().info("Setpoint controller started.")
        self.get_logger().info("Waiting for first goal on topic '%s'..." % goal_topic)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.th = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def goal_callback(self, msg: PoseStamped):
        """Receive a new goal from RViz."""
        self.goal_parent_frame = msg.header.frame_id if msg.header.frame_id else self.goal_parent_frame
        self.x_goal = float(msg.pose.position.x)
        self.y_goal = float(msg.pose.position.y)

        q = msg.pose.orientation
        self.th_goal = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # New goal: mark as "not reached" so control loop activates
        self.goal_reached = False

        self.get_logger().info(f"New goal received in '{self.goal_parent_frame}': x={self.x_goal:.3f}, y={self.y_goal:.3f}, th={self.th_goal:.3f} rad")

        self.publish_goal_tf()

    def control_loop(self):
        # Wait until we have odometry AND an active (not-yet-reached) goal
        if not self.odom_received or self.goal_reached:
            self.publish_cmd_vel(0.0, 0.0)
            return

        # Compute error between current pose and goal pose
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y

        rho = math.sqrt(dx * dx + dy * dy)                     # Distance to goal
        alpha = wrap_angle(math.atan2(dy, dx) - self.th)       # Heading error to goal
        beta = wrap_angle(self.th_goal - self.th - alpha)      # Orientation correction term
        dtheta = wrap_angle(self.th_goal - self.th)            # Final yaw error

        # Check if we are at the setpoint
        if (rho < self.rho_stop) and (abs(dtheta) < self.yaw_stop):
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(f"Setpoint reached: pose=[x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f} rad], goal=[x={self.x_goal:.3f}, y={self.y_goal:.3f}, th={self.th_goal:.3f} rad]")

            self.publish_cmd_vel(0.0, 0.0) # Stop the robot
            return
        else:
            # Robot is outside the setpoint region again
            self.goal_reached = False

        # Compute linear and angular velocity commands
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        v_min = 0.05
        w_min = 0.25

        # Enforce a minimum linear speed if still far from goal
        if rho > self.rho_stop and abs(v) < v_min:
            v = math.copysign(v_min, v)

        # Enforce a minimum angular speed if heading error is large
        if abs(dtheta) > self.yaw_stop and abs(w) < w_min:
            w = math.copysign(w_min, w)

        # Clamp speeds to maximum limits
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        # Log current error
        self.log_error()

        # Send velocity command to the robot
        self.publish_cmd_vel(v, w)

        # Keep goal transform updated for visualization
        self.publish_goal_tf()

    def publish_cmd_vel(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_goal_tf(self):
        """Send a TF transform for the goal pose."""
        # If goal is "reached"/inactive (including before first goal), skip TF
        if self.goal_reached:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.goal_parent_frame
        t.child_frame_id = self.goal_child_frame

        t.transform.translation.x = float(self.x_goal)
        t.transform.translation.y = float(self.y_goal)
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(self.th_goal)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def log_error(self):
        """Log error once per second and refresh TF."""
        now = self.get_clock().now()
        dt = (now - self.last_log_time).nanoseconds * 1e-9

        if dt < self.log_period:
            return

        self.last_log_time = now

        # If there is no odom or no active goal, nothing to log
        if not self.odom_received or self.goal_reached:
            return

        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        dtheta = wrap_angle(self.th_goal - self.th)
        rho = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(f"Error [dx, dy, dtheta] = [{dx:.3f}, {dy:.3f}, {dtheta:.3f}] (rho={rho:.3f})")

        # Keep the goal TF alive / updated
        self.publish_goal_tf()


def main(args=None):
    rclpy.init(args=args)
    node = SetpointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
