#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pygame


class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        # ROS publisher
        self.pub = self.create_publisher(JointState, '/wheelVel', 10)

        # Params
        self.declare_parameter('wheel_speed', 5.0)
        self.declare_parameter('turn_speed', 2.5)
        self.declare_parameter('rate', 30.0)

        self.wheel_speed = self.get_parameter('wheel_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.rate = self.get_parameter('rate').value

        # Pygame init
        pygame.init()
        self.screen = pygame.display.set_mode((300, 200))
        pygame.display.set_caption("ROS2 WASD Teleop")

        self.clock = pygame.time.Clock()

        self.get_logger().info(
            "Pygame teleop started.\n"
            "Focus this window and use:\n"
            "  W/S = forward/back\n"
            "  A/D = turn left/right\n"
            "  ESC or close window = quit"
        )

    def step(self):
        """Handle events, compute wheel speeds from key states, publish."""
        # Handle window events (close, etc)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("Window closed, shutting down.")
                rclpy.shutdown()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.get_logger().info("ESC pressed, shutting down.")
                rclpy.shutdown()
                return

        # Get current key states
        keys = pygame.key.get_pressed()

        lin = 0.0
        ang = 0.0

        # Forward/back
        if keys[pygame.K_w]:
            lin += 1.0
        if keys[pygame.K_s]:
            lin -= 1.0

        # Turn
        if keys[pygame.K_a]:
            ang += 1.0    # left
        if keys[pygame.K_d]:
            ang -= 1.0    # right

        # Compute wheel speeds
        left = lin * self.wheel_speed - ang * self.turn_speed
        right = lin * self.wheel_speed + ang * self.turn_speed

        # Publish
        msg = JointState()
        msg.name = ['joint_wheel_left', 'joint_wheel_right']
        msg.velocity = [float(left), float(right)]
        self.pub.publish(msg)

        # Limit rate
        self.clock.tick(self.rate)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()

    try:
        while rclpy.ok():
            node.step()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        # stop robot
        msg = JointState()
        msg.name = ['joint_wheel_left', 'joint_wheel_right']
        msg.velocity = [0.0, 0.0]
        node.pub.publish(msg)

        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
