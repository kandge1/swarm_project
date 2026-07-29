#!/usr/bin/env python3
"""Pi-side DetectBlock service: one still, one answer.

RUNS ON THE ROBOT (the Pi), not on mars. It holds the wrist camera subscription
and never sends an image anywhere -- the DDS link silently drops anything over
~1400 bytes (PROJECT_CONTEXT.md, MTU fragmentation), so a cross-machine image
stream is not slow, it is impossible. The reply is a few hundred bytes.

    # on the Pi, alongside real_robot_hardware.launch.py
    ros2 launch mycobot_280pi_camera_moveit2 camera.launch.py
    python3 block_detector_node.py

    # from either machine, with the arm parked at a hover
    ros2 service call /detect_block swarm_interfaces/srv/DetectBlock \\
        "{zone: pickup, zone_x: 0.0, zone_y: 0.25, zone_z: 0.0, zone_yaw: 0.0}"

All the actual vision lives in zone_vision.py, which imports no ROS at all. This
file is only the plumbing: grab a fresh frame, call analyze(), fill in the
response. Keeping the split that way is what lets the same code be tuned against
saved stills on mars -- see zone_view.py.

CALL THIS ONLY WHILE THE ARM IS STATIONARY. Two reasons, both documented in
PROJECT_CONTEXT.md: the serial link is half-duplex so reads fail while the arm
moves, and this node shares a Pi with the 100 Hz ros2_control loop.
"""
import os
import sys
import threading
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from swarm_interfaces.msg import BlockDetection
from swarm_interfaces.srv import DetectBlock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402


class BlockDetector(Node):
    def __init__(self):
        super().__init__("block_detector")

        self.declare_parameter("image_topic", "/wrist_camera/image_raw")
        self.declare_parameter("method", "canny")
        self.declare_parameter("tag_size", zv.DEFAULT_TAG_SIZE)
        self.declare_parameter("default_zone_size", zv.DEFAULT_ZONE_SIZE)
        self.declare_parameter("max_homography_rms_px", zv.MAX_HOMOGRAPHY_RMS_PX)
        # Frames to throw away after a request arrives before keeping one. The
        # arm has just finished moving and v4l2's auto-exposure and auto-white-
        # balance are still chasing the new scene; the first frame after a move
        # is routinely darker or blurrier than the third.
        self.declare_parameter("warmup_frames", 3)
        self.declare_parameter("frame_timeout_sec", 4.0)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self._frame_seq = 0          # bumped per frame; how freshness is judged

        # Separate callback groups + a MultiThreadedExecutor, so the image
        # subscription keeps firing while a service callback is waiting for a
        # fresh frame. On a single-threaded executor the service handler would
        # block the very callback it is waiting on and deadlock until timeout.
        self._image_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        topic = self.get_parameter("image_topic").value
        self._sub = self.create_subscription(
            Image, topic, self._on_image, 1, callback_group=self._image_group)
        self._service = self.create_service(
            DetectBlock, "detect_block", self._on_request,
            callback_group=self._service_group)

        self.get_logger().info(
            "block_detector up: listening on %s, serving /detect_block" % topic)
        self.get_logger().info(
            "tag_size=%.4f m, default zone_size=%.4f m, method=%s"
            % (self.get_parameter("tag_size").value,
               self.get_parameter("default_zone_size").value,
               self.get_parameter("method").value))

    # -- camera ------------------------------------------------------------
    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn("cv_bridge failed: %s" % exc)
            return
        with self._lock:
            self._frame = frame
            self._frame_seq += 1

    def _grab_fresh_frame(self):
        """(frame, error). Waits for a frame captured AFTER this call started.

        Never returns whatever happens to be cached: that frame may predate the
        move that just finished, which would place the block using a picture of
        where it used to be relative to the camera. A stale frame is the one
        failure here that produces a confident wrong answer rather than an
        error, so the sequence check is not optional.
        """
        warmup = int(self.get_parameter("warmup_frames").value)
        timeout = float(self.get_parameter("frame_timeout_sec").value)

        with self._lock:
            start_seq = self._frame_seq
        needed = start_seq + max(1, warmup)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frame_seq >= needed and self._frame is not None:
                    return self._frame.copy(), None
            time.sleep(0.02)

        with self._lock:
            seen = self._frame_seq - start_seq
        topic = self.get_parameter("image_topic").value
        if self._frame is None and seen == 0:
            return None, ("no images on %s at all in %.1f s -- is camera.launch.py "
                          "running on the Pi?" % (topic, timeout))
        return None, ("only %d of %d fresh frames on %s in %.1f s; camera is "
                      "publishing but too slowly to trust" % (seen, warmup, topic, timeout))

    # -- service -----------------------------------------------------------
    def _on_request(self, request, response):
        started = time.monotonic()
        response.success = False
        response.blocks = []
        response.tag_ids = []

        zone_size = request.zone_size or float(
            self.get_parameter("default_zone_size").value)
        try:
            zone = zv.zone_for(
                request.zone,
                world_x=request.zone_x, world_y=request.zone_y,
                world_yaw=request.zone_yaw, world_z=request.zone_z,
                zone_size=zone_size,
                tag_size=float(self.get_parameter("tag_size").value))
        except ValueError as exc:
            response.message = str(exc)
            self.get_logger().warn(response.message)
            return response

        frame, error = self._grab_fresh_frame()
        if frame is None:
            response.message = error
            self.get_logger().warn(error)
            return response

        result = zv.analyze(
            frame, zone,
            method=str(self.get_parameter("method").value),
            max_rms_px=float(self.get_parameter("max_homography_rms_px").value))

        response.success = result.success
        response.message = result.message
        response.tags_seen = result.tags_seen
        response.tag_ids = [int(t) for t in result.tag_ids]
        response.homography_rms = float(result.homography_rms)
        response.scale_px_per_m = float(result.scale_px_per_m)
        response.camera_zx = float(result.camera_zx)
        response.camera_zy = float(result.camera_zy)

        for block in result.blocks:
            wx, wy, wyaw = block.world_pose(zone)
            entry = BlockDetection()
            entry.zx, entry.zy, entry.zyaw = block.zx, block.zy, block.zyaw
            entry.x, entry.y, entry.yaw = wx, wy, wyaw
            entry.width, entry.length = block.width, block.length
            entry.shape = block.shape
            entry.symmetry = int(block.symmetry)
            entry.fill_ratio = float(block.fill_ratio)
            entry.area_px = float(block.area_px)
            response.blocks.append(entry)

        if request.save_debug_image and request.debug_image_path:
            self._write_debug_image(frame, result, zone, request.debug_image_path)

        elapsed = time.monotonic() - started
        level = self.get_logger().info if result.success else self.get_logger().warn
        level("[%s] %s (%.0f ms)" % (request.zone, result.message, elapsed * 1000.0))
        for index, block in enumerate(result.blocks):
            self.get_logger().info(
                "  [%d] zone (%+.1f, %+.1f) mm yaw %+.1f deg  %.1fx%.1f mm %s sym=%d"
                % (index, block.zx * 1000, block.zy * 1000,
                   block.zyaw * 57.2958, block.width * 1000,
                   block.length * 1000, block.shape, block.symmetry))
        return response

    def _write_debug_image(self, frame, result, zone, path):
        """Best effort -- a debug image failing must never fail a detection."""
        try:
            import cv2
            import zone_view
            cv2.imwrite(path, zone_view.annotate(frame, result, zone))
            self.get_logger().info("wrote debug image %s" % path)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn("debug image failed (detection is unaffected): %s" % exc)


def main():
    rclpy.init()
    node = BlockDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy installs its own signal handler, so a SIGTERM (which is how this
        # gets stopped in practice) has already shut the context down by the
        # time this runs. Calling shutdown() again raises RCLError and buries
        # whatever the node was actually doing under a traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
