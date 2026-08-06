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
        "{zone: pickup, zone_x: 0.0, zone_y: 0.2286, zone_z: 0.0, zone_yaw: 0.0}"

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

from std_msgs.msg import Float64MultiArray
from swarm_interfaces.msg import BlockDetection
from swarm_interfaces.srv import DetectBlock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import block_coordinates as bc  # noqa: E402
import detection_wire as wire  # noqa: E402
import zone_vision as zv  # noqa: E402

# Pixels per tag module, the number that says whether to believe a decode.
# Measured, not folklore -- block_tags_selftest.test_px_per_module_floor renders
# a tag foreshortened like a block side face, blurs and noises it, and counts
# decodes:
#
#     2.5 px/module    0% decoded
#     3.0 px/module   10% decoded     <- MIN: a sharp-image floor only
#     4.0 px/module   95% decoded     <- GOOD: survives realistic blur
#
# So MIN is the point below which nothing works even in ideal conditions, and
# GOOD is the point where the tag stops being the limiting factor. A tag between
# them decodes intermittently, which presents as a flaky vision bug rather than
# a sizing problem -- which is the reason to report the number per tag at all.
TAG_PX_PER_MODULE_MIN = 3.0
TAG_PX_PER_MODULE_GOOD = 4.0

_WINDOW_NAME = "detect_block -- what the detector sees"


class _WireView(object):
    """A zone_vision block plus its world pose, in the shape detection_wire
    expects. zone_vision blocks carry only zone-local coordinates -- the world
    pose needs the ZoneSpec -- so the two are joined here rather than teaching
    detection_wire about zones, which would give it a second job and a reason
    to import something."""

    __slots__ = ("zx", "zy", "zyaw", "x", "y", "yaw", "width", "length",
                 "shape", "symmetry")

    def __init__(self, block, world_x, world_y, world_yaw):
        self.zx, self.zy, self.zyaw = block.zx, block.zy, block.zyaw
        self.x, self.y, self.yaw = world_x, world_y, world_yaw
        self.width, self.length = block.width, block.length
        self.shape = block.shape
        self.symmetry = int(block.symmetry)


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
        # Printed size of a BLOCK tag, which is not the zone tag size. Block
        # tags come off print_block_tags.py at 25.4 mm -- 1 inch, chosen
        # 2026-08-04 over the 22.5 mm the face arithmetic allows, for the extra
        # 0.5 px/module. Only used to turn a tag's pixel size into a px/m scale
        # for the height estimate; detection does not care.
        #
        # This is the MEASURED size of the sheet actually stuck on the blocks,
        # not the nominal one -- three prints in a row came out 8-13% small
        # before the print chain was understood (print_block_tags.
        # DEFAULT_PRINT_CORRECTION). Measure the ruler on each new sheet. It was
        # 0.024 until 2026-08-04, the pre-1.25-module quiet zone value, which
        # never matched anything printed.
        self.declare_parameter("block_tag_size", 0.0254)
        # Lens height above the mat at the pose the still was taken from. The
        # height estimate is linear in it, so a wrong value scales the answer
        # rather than breaking it. 0.2235 is the surveyed detection hover.
        self.declare_parameter("lens_height_hint", 0.2235)
        # Show every analysed frame in a window, annotated exactly as the debug
        # image is. Off by default because the Pi is normally headless and the
        # window costs ~15 ms a call.
        #
        # This is a DIAGNOSTIC OF LAST RESORT and it earns its place: on
        # 2026-08-05 an explore sweep reported the pickup mat at two bearings
        # 180 deg apart, which one mat cannot do. No amount of reading numbers
        # settles that -- seeing the frame does, immediately.
        # Every analysed frame to disk, annotated, so "what was the camera
        # actually looking at" stops being a question that needs a person
        # standing next to the robot. Off by default because it costs disk and
        # a few ms; set it and it just starts filling up.
        #
        # This is not the same as save_debug_image on the request: that writes
        # ONE frame to a path the caller names, which only helps when you
        # already know which call went wrong. The failures worth chasing here
        # are the ones you notice afterwards -- 2026-08-05, twenty consecutive
        # stills decoded zero block tags with nothing visibly wrong with the
        # tag, and no image of any of them existed.
        self.declare_parameter("frame_dump_dir", "")
        # Newest N kept, oldest pruned. 640x480 PNG is ~200-400 kB, so 200
        # frames is under 100 MB -- bounded enough to leave on during a whole
        # session on a Pi's SD card.
        self.declare_parameter("frame_dump_keep", 200)
        # Also write the UNANNOTATED frame. Costs double the disk and is what
        # you need to re-run detection offline with different parameters --
        # the annotated one answers "what did it find", the raw one answers
        # "what could it have found".
        self.declare_parameter("frame_dump_raw", False)
        self.declare_parameter("show_window", False)
        self.declare_parameter("window_scale", 1.0)

        # Say so at startup. A parameter that silently does nothing when it is
        # misspelled, or when `ros2 param set` failed and nobody noticed, is
        # indistinguishable from a parameter that is working and finding
        # nothing -- and the whole point of the dump is to be believed.
        dump_dir = str(self.get_parameter("frame_dump_dir").value).strip()
        if dump_dir:
            self.get_logger().info(
                "frame dump ON -> %s (keeping newest %d, raw frames %s)"
                % (dump_dir, int(self.get_parameter("frame_dump_keep").value),
                   "on" if self.get_parameter("frame_dump_raw").value else "off"))
        else:
            self.get_logger().info(
                "frame dump off; start with --ros-args -p "
                "frame_dump_dir:=/tmp/frames to record what the camera sees")

        self._lock = threading.Lock()
        self._frame = None
        self._frame_seq = 0          # bumped per frame; how freshness is judged
        # None = not tried yet, "open", or "unavailable" (never retried).
        self._window_state = None

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

        # Every detection also goes out as a flat float array, ALWAYS, not only
        # when the service struggles. Two reasons it is unconditional: a
        # fallback exercised for the first time in the middle of a failure is
        # not a fallback, and a caller that prefers the topic should not have to
        # ask the robot to switch modes. Costs one small publish per request.
        # See detection_wire.py for why this path can carry what the service
        # reply cannot.
        self._wire_pub = self.create_publisher(
            Float64MultiArray, wire.TOPIC, 1)

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

        # Block face tags, found whatever the zone detection did -- a frame can
        # show block tags and no zone tags at all, and that is a useful answer
        # rather than a failure.
        #
        # THIS IS NOT IN THE SERVICE RESPONSE, deliberately. The obvious move is
        # a BlockTag[] field on DetectBlock.srv; do not. APRIL_TAGS_DEV.md's
        # OPEN BUG is that populating a nested message containing a STRING makes
        # rcl_send_response fail on this machine ('string data is not
        # null-terminated', serdata.cpp:354), and a BlockTag carrying a face
        # name is exactly that shape -- on an interface whose build is already
        # the prime suspect, needing a coordinated rebuild on both machines. The
        # question this exists to answer, are the tags legible at the survey
        # pose, is answered entirely by this node's log and the debug image,
        # neither of which crosses DDS. If mars ever does need them, add
        # PARALLEL PRIMITIVE ARRAYS (uint16[] ids, float64[] px, ...), never a
        # nested message with a string -- that document's own fallback.
        gray_frame = frame if frame.ndim == 2 else cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY)
        block_tags = zv.find_block_tags(gray_frame, result.H_px_to_zone)

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
            gray = gray_frame
            others = zv.detect_all_tags(gray)
            # Laplacian variance is the standard cheap focus metric: sharp edges
            # produce large second derivatives, a defocused frame does not.
            focus = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            mean = float(gray.mean())
            lo, hi = float(gray.min()), float(gray.max())
            diag = (" | frame %dx%d, mean %.0f (range %.0f-%.0f), focus %.0f"
                    % (gray.shape[1], gray.shape[0], mean, lo, hi, focus))
            if others:
                # Naming them matters now that block tags are expected in shot:
                # a frame showing only ids 8-19 is the angled survey pose
                # working exactly as intended, not a zone mismatch.
                diag += (" | %d AprilTag(s) of OTHER ids visible: %s -- the "
                         "camera CAN see tags, so this is a zone/id mismatch, "
                         "not an image problem"
                         % (len(others),
                            ", ".join(bc.label(t) for t in sorted(others))))
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

        self._publish_wire(result, zone, block_tags)
        self._show_window(frame, result, zone, block_tags)
        self._dump_frame(frame, result, zone, block_tags)

        if request.save_debug_image and request.debug_image_path:
            self._write_debug_image(frame, result, zone,
                                    request.debug_image_path, block_tags)

        elapsed = time.monotonic() - started
        # TWO call sites, not one variable-severity call. Found 2026-07-31 on
        # hardware: every request after the first was silently abandoned mid-
        # callback -- the debug image (logged the line before this) always
        # wrote fine, but the result line right after it, and therefore the
        # service RESPONSE, never arrived, so mars sat out its full timeout
        # waiting for an answer the Pi had already computed and then dropped.
        # Root cause matches a known rclpy/rcutils limitation: severity is
        # cached PER CALL SITE, and `level = self.get_logger().info if ... else
        # .warn; level(...)` makes ONE call site's effective severity flip
        # between invocations depending on that call's result -- which is
        # exactly what triggered the unretrieved exception seen in between,
        # word for word: "Logger severity cannot be changed between calls."
        # Splitting into two fixed call sites, each always the same severity,
        # removes the only mechanism that could trip it.
        summary = "[%s] %s (%.0f ms)" % (request.zone, result.message,
                                         elapsed * 1000.0)
        if result.success:
            self.get_logger().info(summary)
        else:
            self.get_logger().warn(summary)
        for index, block in enumerate(result.blocks):
            self.get_logger().info(
                "  [%d] zone (%+.1f, %+.1f) mm yaw %+.1f deg  %.1fx%.1f mm %s sym=%d"
                % (index, block.zx * 1000, block.zy * 1000,
                   block.zyaw * 57.2958, block.width * 1000,
                   block.length * 1000, block.shape, block.symmetry))
        self._log_block_tags(block_tags, result)
        return response

    # -- block tags ----------------------------------------------------------
    def _log_block_tags(self, block_tags, result):
        if not block_tags:
            self.get_logger().info("  no block tags (ids %d-%d) in frame"
                                   % (bc.BLOCK_TAG_ID_MIN, bc.BLOCK_TAG_ID_MAX))
            return
        for face, _corners, px, per_module, zone_xy in block_tags:
            verdict = ("ok" if per_module >= TAG_PX_PER_MODULE_GOOD
                       else "MARGINAL" if per_module >= TAG_PX_PER_MODULE_MIN
                       else "TOO SMALL -- decoded, but do not rely on it")
            extra = ""
            if zone_xy is not None:
                extra = "  zone (%+.1f, %+.1f) mm" % (zone_xy[0] * 1000,
                                                      zone_xy[1] * 1000)
                # Scale relative to the mat plane is a height readout that needs
                # no intrinsics -- there are none in this repo. Only valid for a
                # mat-parallel tag, which is why it is inside this branch.
                if result.scale_px_per_m > 0:
                    tag_scale = px / float(self.get_parameter("block_tag_size").value)
                    height = bc.height_from_scale(
                        tag_scale, result.scale_px_per_m,
                        float(self.get_parameter("lens_height_hint").value))
                    extra += "  implies h %+.1f mm" % (height * 1000)
            self.get_logger().info(
                "  block tag id %-2d %-14s %5.1f px (%.1f px/module, %s)%s"
                % (face.tag_id, face.label, px, per_module, verdict, extra))

    def _show_window(self, frame, result, zone, block_tags):
        """Live annotated view of the frame that was just analysed.

        Best effort and never able to fail a detection: this is a diagnostic,
        and a diagnostic that can break the thing it observes is worse than no
        diagnostic.

        The display is checked UP FRONT rather than caught, because with no
        display cv2.namedWindow does not raise -- Qt fails to load its xcb
        plugin and calls abort(), which no try/except can survive. Verified
        while building block_tag_probe.py. On a headless Pi this would
        otherwise take the detector node down on the first request.
        """
        if not bool(self.get_parameter("show_window").value):
            return
        if self._window_state == "unavailable":
            return
        if self._window_state is None:
            if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
                self._window_state = "unavailable"
                self.get_logger().warn(
                    "show_window is set but neither DISPLAY nor "
                    "WAYLAND_DISPLAY is -- no window can be opened. Reconnect "
                    "with `ssh -X`, or use save_debug_image. Detection is "
                    "unaffected.")
                return
            try:
                cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
                self._window_state = "open"
            except cv2.error as exc:                   # noqa: BLE001
                self._window_state = "unavailable"
                self.get_logger().warn(
                    "a display is set but OpenCV cannot open a window (%s) -- "
                    "this build may lack GUI support. Detection is unaffected."
                    % str(exc).strip().splitlines()[-1][:100])
                return
        try:
            import zone_view
            canvas = zone_view.annotate(frame, result, zone,
                                        block_tags=block_tags)
            scale = float(self.get_parameter("window_scale").value)
            if scale and abs(scale - 1.0) > 1e-6:
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale,
                                    interpolation=cv2.INTER_NEAREST)
            cv2.imshow(_WINDOW_NAME, canvas)
            # Pumps the GUI event loop. Without it the window paints once and
            # then freezes, which looks exactly like the detector having hung.
            cv2.waitKey(1)
        except Exception as exc:                       # noqa: BLE001
            self._window_state = "unavailable"
            self.get_logger().warn(
                "window update failed, disabling it (detection is "
                "unaffected): %s" % exc)

    def _publish_wire(self, result, zone, block_tags=()):
        """Best effort -- the topic must never be able to fail a detection.

        Published BEFORE the service response is built, so it goes out even if
        the reply is the thing that cannot be sent. That ordering is the entire
        value of this path: when rcl_send_response fails, mars still has the
        answer.
        """
        try:
            blocks = []
            for block in result.blocks:
                wx, wy, wyaw = block.world_pose(zone)
                blocks.append(_WireView(block, wx, wy, wyaw))
            data, dropped = wire.encode(
                result.success, [int(t) for t in result.tag_ids],
                float(result.homography_rms), float(result.scale_px_per_m),
                float(result.camera_zx), float(result.camera_zy), blocks,
                block_tags=block_tags)
            size = wire.encoded_bytes(data)
            if size > 1300:
                self.get_logger().warn(
                    "detection payload %d B is close to the ~1400 B DDS limit; "
                    "lower detection_wire.MAX_BLOCKS" % size)
            message = Float64MultiArray()
            message.data = data
            self._wire_pub.publish(message)
            if dropped:
                self.get_logger().warn(
                    "%d block(s) past detection_wire.MAX_BLOCKS were not "
                    "published (smallest first)" % dropped)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(
                "could not publish the detection topic (the service reply is "
                "unaffected): %s" % exc)

    # -- frame dump ----------------------------------------------------------
    # Prefix on every file this writes. Pruning only ever deletes files that
    # start with it, so pointing frame_dump_dir at a directory holding anything
    # else cannot lose that other thing.
    _DUMP_PREFIX = "frame_"

    def _dump_frame(self, frame, result, zone, block_tags):
        """Write this frame to frame_dump_dir. Best effort, always.

        The filename carries the answer, so a failing frame can be found
        without opening any of them:

            frame_20260805-213412-042_zone4_blocktags0_focus331.png

        zone<N> is how many of the four zone tags decoded, blocktags<N> how many
        block face tags did. A run of blocktags0 with a healthy focus number is
        a different problem from a run with focus 30.
        """
        directory = str(self.get_parameter("frame_dump_dir").value).strip()
        if not directory:
            return
        try:
            import zone_view
            if not os.path.isdir(directory):
                os.makedirs(directory)
            gray = frame if frame.ndim == 2 else cv2.cvtColor(
                frame, cv2.COLOR_BGR2GRAY)
            focus = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            now = time.time()
            stamp = "%s-%03d" % (time.strftime("%Y%m%d-%H%M%S",
                                               time.localtime(now)),
                                 int((now % 1.0) * 1000))
            base = "%s%s_zone%d_blocktags%d_focus%d" % (
                self._DUMP_PREFIX, stamp, len(result.tag_ids),
                len(block_tags or []), round(focus))
            cv2.imwrite(os.path.join(directory, base + ".png"),
                        zone_view.annotate(frame, result, zone,
                                           block_tags=block_tags))
            if bool(self.get_parameter("frame_dump_raw").value):
                cv2.imwrite(os.path.join(directory, base + "_raw.png"), frame)
            self._prune_dump(directory)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(
                "frame dump failed, disabling it for this frame only "
                "(detection is unaffected): %s" % exc)

    def _prune_dump(self, directory):
        """Keep the newest frame_dump_keep files, delete the rest.

        Sorted by NAME, not mtime: the names begin with a zero-padded timestamp
        so they sort chronologically, and name order is stable where mtime on a
        Pi's SD card with a drifting clock is not. A raw frame sorts next to its
        annotated twin, so a pair is pruned together.
        """
        keep = int(self.get_parameter("frame_dump_keep").value)
        if keep <= 0:
            return
        names = sorted(n for n in os.listdir(directory)
                       if n.startswith(self._DUMP_PREFIX) and n.endswith(".png"))
        for name in names[:max(0, len(names) - keep)]:
            try:
                os.remove(os.path.join(directory, name))
            except OSError:
                pass

    def _write_debug_image(self, frame, result, zone, path, block_tags=None):
        """Best effort -- a debug image failing must never fail a detection."""
        try:
            import cv2
            import zone_view
            cv2.imwrite(path, zone_view.annotate(frame, result, zone,
                                                 block_tags=block_tags))
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
        if self._window_state == "open":
            try:
                cv2.destroyAllWindows()
            except Exception:                          # noqa: BLE001
                pass


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
