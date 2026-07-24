#!/usr/bin/env python3
"""Subscribe to the wrist camera, run image processing, republish the result.

Works unmodified against Gazebo (wrist_camera sensor plugin) or the real
mycobot 280pi (v4l2_camera_node) - both publish sensor_msgs/Image on
/wrist_camera/image_raw, so this node doesn't need to know which one it's
talking to.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraView(Node):
    def __init__(self):
        super().__init__("camera_view")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/wrist_camera/image_raw", self.on_image, 10
        )
        self.pub = self.create_publisher(Image, "/wrist_camera/processed", 10)

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Placeholder processing - replace with real detection/tracking.
        processed = cv2.Canny(frame, 100, 200)
        processed = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        cv2.imshow("wrist_camera raw", frame)
        cv2.imshow("wrist_camera processed", processed)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
