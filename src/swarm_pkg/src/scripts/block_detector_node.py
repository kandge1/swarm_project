#!/usr/bin/env python3
"""Pi-side DetectBlock service: one still, one answer.

RUNS ON THE ROBOT (the Pi), not on mars. It never sends an image anywhere -- the
DDS link silently drops anything over ~1400 bytes on the mars<->robot Wi-Fi hop
(PROJECT_CONTEXT.md, MTU fragmentation), so a cross-machine image stream is not
slow, it is impossible. The reply is a few hundred bytes.

    # on the Pi, alongside real_robot_hardware.launch.py
    python3 block_detector_node.py

    # from either machine, with the arm parked at a hover
    ros2 service call /detect_block swarm_interfaces/srv/DetectBlock \\
        "{zone: pickup, zone_x: 0.0, zone_y: 0.25, zone_z: 0.0, zone_yaw: 0.0}"

READS THE CAMERA DIRECTLY (cv2.VideoCapture), not via a v4l2_camera_node topic.
Changed 2026-07-31 -- it used to subscribe to /wrist_camera/image_raw, published
by a SEPARATE process (camera.launch.py's v4l2_camera_node). Both processes are
on the Pi, so the image never left the machine -- but ROS topic pub/sub still
goes through Cyclone DDS's RTPS layer regardless of locality, and that layer was
using the SAME MaxMessageSize=1400B tuned for the weak mars<->robot Wi-Fi hop. A
640x480 BGR8 frame is 921,600 bytes, so under that cap it fragmented into ~700
RTPS pieces PER FRAME, on loopback, for a hop that never touches Wi-Fi at all.
Measured cost: a detect call that decoded tags fine still took the better part
of a minute, with the node's own log showing repeated 'invalid data size' /
'string data is not null-terminated' RTPS deserialization errors in between.
Reading the device directly removes DDS from the image path completely, the
same way live_tag_view.py already does.

CONSEQUENCE: this node now OWNS THE CAMERA DEVICE. Do NOT also run
camera.launch.py's v4l2_camera_node or live_tag_view.py at the same time --
V4L2 only allows one reader, and the second one to start will fail to open the
device (same conflict live_tag_view.py's docstring already warns about, now
also applying between this node and that script).

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

import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from swarm_interfaces.msg import BlockDetection
from swarm_interfaces.srv import DetectBlock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402


class BlockDetector(Node):
    def __init__(self):
        super().__init__("block_detector")

        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
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

        self._lock = threading.Lock()
        self._frame = None
        self._frame_seq = 0          # bumped per frame; how freshness is judged

        device = self.get_parameter("video_device").value
        self._cap = cv2.VideoCapture(device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,
                      self.get_parameter("frame_width").value)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT,
                      self.get_parameter("frame_height").value)
        if not self._cap.isOpened():
            raise RuntimeError(
                "could not open %s -- is it already held by camera.launch.py's "
                "v4l2_camera_node or live_tag_view.py? Only one reader is "
                "allowed; stop the other one first, or check `ls /dev/video*`."
                % device)

        # Background thread, not a timer callback: cap.read() blocks on the
        # USB transfer, and blocking inside an executor callback would stall
        # every other callback on this node's executor along with it.
        self._capture_stop = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # Separate callback groups + a MultiThreadedExecutor, so a service
        # callback waiting on a fresh frame cannot block anything else this
        # node needs to do. The capture thread is independent of both --
        # ROS callback groups only govern ROS callbacks -- but keeping this
        # matches the original single-threaded-executor deadlock this design
        # was already built to avoid.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._service = self.create_service(
            DetectBlock, "detect_block", self._on_request,
            callback_group=self._service_group)

        self.get_logger().info(
            "block_detector up: reading %s directly, serving /detect_block"
            % device)
        self.get_logger().info(
            "tag_size=%.4f m, default zone_size=%.4f m, method=%s"
            % (self.get_parameter("tag_size").value,
               self.get_parameter("default_zone_size").value,
               self.get_parameter("method").value))

    # -- camera ------------------------------------------------------------
    def _capture_loop(self):
        """Runs on its own thread for the node's whole lifetime, continuously
        pulling frames from the device. Same _frame/_frame_seq contract
        _on_image used to fill from a DDS callback -- _grab_fresh_frame()
        does not know or care which one is writing them."""
        fail_streak = 0
        while not self._capture_stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                fail_streak += 1
                if fail_streak % 30 == 1:      # log occasionally, not per-frame
                    self.get_logger().warn(
                        "cap.read() failed (%d in a row) -- camera unplugged, "
                        "or /dev/video0 renumbered?" % fail_streak)
                time.sleep(0.05)
                continue
            fail_streak = 0
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
        device = self.get_parameter("video_device").value
        if self._frame is None and seen == 0:
            return None, ("no frames read from %s at all in %.1f s -- check "
                          "`ls /dev/video*` and that nothing else has the "
                          "device open" % (device, timeout))
        return None, ("only %d of %d fresh frames from %s in %.1f s; camera is "
                      "reading but too slowly to trust" % (seen, warmup, device, timeout))

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

        # When no tags of THIS zone were found, say what the frame actually
        # contained. Without this, "saw 0 of zone's 4 tags" is the same message
        # whether the mat is out of frame, the wrong zone is under the camera,
        # the lens is out of focus, or the exposure is blown -- four different
        # problems with four different fixes, and telling them apart otherwise
        # means copying an image off the Pi mid-session.
        #
        # Deliberately computed only on the failure path: detect_all_tags is a
        # second full detector pass, which is not worth paying for on every
        # successful call.
        if not result.tag_ids:
            gray = frame if frame.ndim == 2 else cv2.cvtColor(
                frame, cv2.COLOR_BGR2GRAY)
            others = zv.detect_all_tags(gray)
            # Laplacian variance is the standard cheap focus metric: sharp edges
            # produce large second derivatives, a defocused frame does not.
            focus = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            mean = float(gray.mean())
            lo, hi = float(gray.min()), float(gray.max())
            diag = (" | frame %dx%d, mean %.0f (range %.0f-%.0f), focus %.0f"
                    % (gray.shape[1], gray.shape[0], mean, lo, hi, focus))
            if others:
                diag += (" | %d AprilTag(s) of OTHER ids visible: %s -- the "
                         "camera CAN see tags, so this is a zone/id mismatch, "
                         "not an image problem"
                         % (len(others), sorted(t for t, _ in others)))
            else:
                diag += " | NO AprilTag of any id anywhere in the frame"
                if focus < 100:
                    diag += " -- focus metric is very low, suspect defocus/blur"
                if mean < 40:
                    diag += " -- frame is very dark, suspect exposure"
                elif mean > 215:
                    diag += " -- frame is very bright, suspect glare/overexposure"
            response.message += diag
            self.get_logger().warn("diagnostics:%s" % diag)
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

    def close_camera(self):
        """Stop the capture thread and release the device. Must happen before
        the process exits, or the next reader (camera.launch.py,
        live_tag_view.py, a re-run of this node) inherits a busy device."""
        self._capture_stop.set()
        self._capture_thread.join(timeout=2.0)
        self._cap.release()


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
        node.close_camera()
        node.destroy_node()
        # rclpy installs its own signal handler, so a SIGTERM (which is how this
        # gets stopped in practice) has already shut the context down by the
        # time this runs. Calling shutdown() again raises RCLError and buries
        # whatever the node was actually doing under a traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
