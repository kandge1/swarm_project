#!/usr/bin/env python3
"""Pack a zone detection into a flat float array, and unpack it again.

PURE PYTHON. No ROS, no OpenCV -- so it imports on both machines, and the
encoding can be unit-tested offline instead of on a robot. The ROS parts are
thirty lines in block_detector_node.py (publish) and DetectionSubscriber below
(receive).

------------------------------------------------------------------------------
WHY THIS EXISTS ALONGSIDE /detect_block
------------------------------------------------------------------------------
The service reply carries `BlockDetection[] blocks` -- a nested message
containing a STRING (`shape`). APRIL_TAGS_DEV.md's OPEN BUG is that populating
it makes rcl_send_response fail on the robot:

    'string data is not null-terminated, at rmw-cyclonedds-cpp/src/serdata.cpp:354'

The empty-list case serialises as a 4-byte zero and never enters that
typesupport at all, which is exactly why a call over an empty zone returns and
a call over a full one does not.

This path has no nested message and no string. It is a `std_msgs/
Float64MultiArray` -- a STANDARD message, already built on both machines -- so:

  * no change to swarm_interfaces, and therefore no rebuild on either machine,
    on an interface whose build was the prime suspect for the bug in the first
    place;
  * nothing but float64 on the wire, so the failing code path is not reachable;
  * a topic rather than a service, so a slow or absent subscriber cannot hang
    the detector the way an unretrieved service response can.

It is NOT a replacement for the service. `/detect_block` remains the request/
response path and is the right shape for "take a still NOW and tell me". This
is the escape hatch for the one field that cannot cross.

------------------------------------------------------------------------------
SIZE, WHICH IS A HARD CONSTRAINT HERE
------------------------------------------------------------------------------
The mars<->robot DDS link silently drops anything over ~1400 bytes. A float64
is 8 bytes on the wire, so the budget is ~170 doubles including CDR overhead
and the MultiArrayLayout. MAX_BLOCKS caps the payload well inside that; blocks
past the cap are dropped, largest-first order meaning the ones dropped are the
least interesting. encoded_bytes() reports the real figure so a caller can
assert rather than hope.

------------------------------------------------------------------------------
LAYOUT
------------------------------------------------------------------------------
    [0] schema version        [5] camera_zx
    [1] success (1/0)         [6] camera_zy
    [2] tags_seen             [7] n_tags
    [3] homography_rms        [8] n_blocks
    [4] scale_px_per_m        [9] fields per block
    [10 .. 10+n_tags-1]       tag ids
    then n_blocks x FIELDS_PER_BLOCK:
        zx, zy, zyaw, x, y, yaw, width, length, shape_code, symmetry

`shape` travels as a CODE, not a string -- see SHAPE_CODES. That is the whole
point of the exercise, so do not add a string field here later.
"""

SCHEMA_VERSION = 1
HEADER_LEN = 10
FIELDS_PER_BLOCK = 10

TOPIC = "block_detections"

# Keeps the payload inside the ~1400 byte DDS limit with room to spare:
# 10 + 4 + 10*10 = 114 doubles = 912 bytes before overhead.
MAX_BLOCKS = 10

SHAPE_CODES = {"unknown": 0, "square": 1, "rect": 2, "circle": 3}
SHAPE_NAMES = {code: name for name, code in SHAPE_CODES.items()}


class WireBlock(object):
    """One decoded block. Field-for-field the useful part of BlockDetection."""

    __slots__ = ("zx", "zy", "zyaw", "x", "y", "yaw", "width", "length",
                 "shape", "symmetry")

    def __init__(self, zx, zy, zyaw, x, y, yaw, width, length, shape, symmetry):
        self.zx, self.zy, self.zyaw = zx, zy, zyaw
        self.x, self.y, self.yaw = x, y, yaw
        self.width, self.length = width, length
        self.shape = shape
        self.symmetry = symmetry

    def __repr__(self):
        return ("<block zone(%+.1f, %+.1f) mm %.1fx%.1f mm %s sym=%d>"
                % (self.zx * 1000, self.zy * 1000, self.width * 1000,
                   self.length * 1000, self.shape, self.symmetry))


class WireDetection(object):
    """A decoded detection. Mirrors the DetectBlock response, minus the image."""

    __slots__ = ("success", "tags_seen", "tag_ids", "homography_rms",
                 "scale_px_per_m", "camera_zx", "camera_zy", "blocks",
                 "truncated")

    def __init__(self, success, tags_seen, tag_ids, homography_rms,
                 scale_px_per_m, camera_zx, camera_zy, blocks, truncated=0):
        self.success = success
        self.tags_seen = tags_seen
        self.tag_ids = tag_ids
        self.homography_rms = homography_rms
        self.scale_px_per_m = scale_px_per_m
        self.camera_zx = camera_zx
        self.camera_zy = camera_zy
        self.blocks = blocks
        self.truncated = truncated

    def __repr__(self):
        return ("<detection %s tags=%s rms=%.2f blocks=%d>"
                % ("OK" if self.success else "FAIL", self.tag_ids,
                   self.homography_rms, len(self.blocks)))


def encode(success, tag_ids, homography_rms, scale_px_per_m, camera_zx,
           camera_zy, blocks, max_blocks=MAX_BLOCKS):
    """-> [float]. `blocks` is any sequence of objects with the BlockDetection
    attribute names, so a zone_vision block and a ROS BlockDetection both work
    without a conversion step in between."""
    tag_ids = [int(t) for t in tag_ids]
    kept = list(blocks)[:max_blocks]
    truncated = len(list(blocks)) - len(kept)

    data = [float(SCHEMA_VERSION), 1.0 if success else 0.0,
            float(len(tag_ids)), float(homography_rms), float(scale_px_per_m),
            float(camera_zx), float(camera_zy), float(len(tag_ids)),
            float(len(kept)), float(FIELDS_PER_BLOCK)]
    data.extend(float(t) for t in tag_ids)
    for block in kept:
        shape = getattr(block, "shape", "unknown")
        data.extend([
            float(block.zx), float(block.zy), float(block.zyaw),
            float(getattr(block, "x", 0.0)), float(getattr(block, "y", 0.0)),
            float(getattr(block, "yaw", 0.0)),
            float(block.width), float(block.length),
            float(SHAPE_CODES.get(shape, 0)),
            float(getattr(block, "symmetry", 0)),
        ])
    return data, truncated


def decode(data):
    """-> WireDetection. Raises ValueError on anything malformed.

    Strict on purpose. A truncated or mis-versioned array that decoded to
    "0 blocks" would be indistinguishable from an empty zone, and an empty zone
    is a legitimate, actionable answer -- Stage 2 asks exactly that question
    before releasing a block. Silence and emptiness must not look alike.
    """
    data = list(data)
    if len(data) < HEADER_LEN:
        raise ValueError("array of %d floats is shorter than the %d-float header"
                         % (len(data), HEADER_LEN))
    version = int(round(data[0]))
    if version != SCHEMA_VERSION:
        raise ValueError("schema version %d, expected %d -- one machine is "
                         "running an older detection_wire.py"
                         % (version, SCHEMA_VERSION))
    n_tags = int(round(data[7]))
    n_blocks = int(round(data[8]))
    fields = int(round(data[9]))
    if fields != FIELDS_PER_BLOCK:
        raise ValueError("%d fields per block, expected %d"
                         % (fields, FIELDS_PER_BLOCK))
    expected = HEADER_LEN + n_tags + n_blocks * fields
    if len(data) != expected:
        raise ValueError("array is %d floats, header describes %d "
                         "(%d tags, %d blocks)"
                         % (len(data), expected, n_tags, n_blocks))

    tag_ids = [int(round(v)) for v in data[HEADER_LEN:HEADER_LEN + n_tags]]
    blocks = []
    base = HEADER_LEN + n_tags
    for index in range(n_blocks):
        f = data[base + index * fields: base + (index + 1) * fields]
        blocks.append(WireBlock(
            zx=f[0], zy=f[1], zyaw=f[2], x=f[3], y=f[4], yaw=f[5],
            width=f[6], length=f[7],
            shape=SHAPE_NAMES.get(int(round(f[8])), "unknown"),
            symmetry=int(round(f[9]))))

    return WireDetection(
        success=bool(round(data[1])), tags_seen=int(round(data[2])),
        tag_ids=tag_ids, homography_rms=data[3], scale_px_per_m=data[4],
        camera_zx=data[5], camera_zy=data[6], blocks=blocks)


def encoded_bytes(data):
    """Payload size on the wire, for asserting against the ~1400 byte DDS cap.

    8 bytes a double, plus CDR's 4-byte sequence length and the
    MultiArrayLayout (an empty dim[] plus data_offset). Approximate but on the
    conservative side, and the point is to be nowhere near the limit rather
    than to predict it exactly.
    """
    return len(data) * 8 + 4 + 16


class DetectionSubscriber(object):
    """Mars-side receiver. rclpy is imported lazily so this module stays
    importable, and testable, with no ROS on the path."""

    def __init__(self, node, topic=TOPIC, depth=1):
        from std_msgs.msg import Float64MultiArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.node = node
        self.latest = None
        self.error = None
        # Depth 1 and KEEP_LAST: a detection is only interesting while the arm
        # is where it was taken from, so an old one queued behind a new one is
        # worse than no history at all.
        qos = QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.sub = node.create_subscription(Float64MultiArray, topic,
                                            self._on_message, qos)

    def _on_message(self, msg):
        try:
            self.latest = decode(msg.data)
            self.error = None
        except ValueError as exc:
            self.latest = None
            self.error = str(exc)

    def wait(self, timeout_sec=10.0):
        """Block until a detection arrives. -> WireDetection or None."""
        import time

        import rclpy
        self.latest = None
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest is not None:
                return self.latest
            if self.error is not None:
                raise ValueError(self.error)
        return None


def _selftest():
    class B(object):
        def __init__(self, **kw):
            self.__dict__.update(kw)

    blocks = [B(zx=0.012, zy=-0.005, zyaw=0.3, x=0.012, y=0.2236, yaw=0.3,
                width=0.030, length=0.031, shape="square", symmetry=4),
              B(zx=-0.020, zy=0.018, zyaw=-1.1, x=-0.02, y=0.2466, yaw=-1.1,
                width=0.029, length=0.058, shape="rect", symmetry=2)]
    data, dropped = encode(True, [0, 1, 2, 3], 0.61, 2468.0, 0.0215, 0.0043,
                           blocks)
    got = decode(data)
    assert dropped == 0
    assert got.success and got.tag_ids == [0, 1, 2, 3] and got.tags_seen == 4
    assert abs(got.homography_rms - 0.61) < 1e-12
    assert abs(got.camera_zx - 0.0215) < 1e-12
    assert len(got.blocks) == 2
    assert got.blocks[0].shape == "square" and got.blocks[0].symmetry == 4
    assert got.blocks[1].shape == "rect"
    assert abs(got.blocks[1].length - 0.058) < 1e-12
    print("  round trip           OK  (%d floats, ~%d bytes)"
          % (len(data), encoded_bytes(data)))

    empty, _ = encode(True, [], 0.0, 0.0, 0.0, 0.0, [])
    assert decode(empty).blocks == []
    print("  empty zone           OK  (%d floats) -- and it is a SUCCESS, not a "
          "failure" % len(empty))

    full, dropped = encode(True, [0, 1, 2, 3], 0.5, 2400.0, 0.0, 0.0,
                           blocks * 12)
    assert len(decode(full).blocks) == MAX_BLOCKS and dropped == 24 - MAX_BLOCKS
    size = encoded_bytes(full)
    assert size < 1400, size
    print("  %d blocks (the cap)  OK  (~%d bytes, inside the ~1400 B DDS limit)"
          % (MAX_BLOCKS, size))

    for bad, why in ((data[:5], "truncated"),
                     ([99.0] + data[1:], "wrong schema version"),
                     (data + [0.0], "trailing junk")):
        try:
            decode(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("decode accepted %s input" % why)
    print("  malformed input      OK  (rejected, not silently read as empty)")
    print("\n0 failure(s)")


if __name__ == "__main__":
    _selftest()
