#!/usr/bin/env python3
"""Live camera view: see the wrist camera identifying AprilTags in real time.

STANDALONE -- no ROS, no rclpy. Opens the camera directly with
cv2.VideoCapture so it can run in a spare terminal next to pick_place.py or
tag_pick_place.py with nothing else needing to be running first.

Two windows:
  "tags"  -- live feed with every detected AprilTag boxed and labelled: its
             ID, and if it's one of ours, which zone and which corner.
  "edges" -- Canny edge detection on the same frame. This is the same
             first-pass signal zone_vision.py's segmentation is built on, so
             it's a live preview of what the real detector works with.

    python3 live_tag_view.py                # /dev/video0
    python3 live_tag_view.py --device 2      # /dev/video2
    q or Ctrl-C to quit.

IMPORTANT -- device contention: a V4L2 device can only be held open by one
process at a time. If camera.launch.py's v4l2_camera_node is already running
against this camera, cv2.VideoCapture will fail to open it here. Stop that
node first, or point --device at a different camera if you have more than
one attached.

ON UNDERSIZED PRINTS: this script only identifies tag IDs and draws edges --
it computes no physical position, so a printer that ignored "Actual Size"
does not affect anything shown here (AprilTag decoding doesn't care about
absolute size). It WILL matter once zone_vision.py's homography is used for
real measurements -- see APRIL_TAGS.md's calibration ruler for checking and
fixing print scale before that.
"""
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zone_vision as zv  # noqa: E402

BOX_COLOR = (0, 255, 0)
KNOWN_TEXT_COLOR = (0, 255, 255)
UNKNOWN_TEXT_COLOR = (0, 0, 255)
HEADER_COLOR = (255, 255, 255)


def corner_label(signs):
    sx, sy = signs
    return "%sX,%sY" % ("+" if sx > 0 else "-", "+" if sy > 0 else "-")


def annotate_tags(frame_bgr, gray):
    """Box + label every detected tag; return the annotated BGR frame."""
    canvas = frame_bgr.copy()
    tags = zv.detect_all_tags(gray)

    seen_by_zone = {"pickup": [], "place": []}
    for tag_id, corners in sorted(tags.items()):
        pts = corners.astype(int)
        info = zv.describe_tag_id(tag_id)
        if info is not None:
            zone_name, signs = info
            seen_by_zone[zone_name].append(tag_id)
            label = "ID %d: %s zone, %s corner" % (tag_id, zone_name, corner_label(signs))
            color = KNOWN_TEXT_COLOR
        else:
            label = "ID %d: not part of any zone" % tag_id
            color = UNKNOWN_TEXT_COLOR

        cv2.polylines(canvas, [pts], True, BOX_COLOR, 2)
        anchor = tuple(pts[0])
        cv2.putText(canvas, label, (anchor[0], max(15, anchor[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    header = "pickup seen: %-14s place seen: %s" % (
        sorted(seen_by_zone["pickup"]) or "none", sorted(seen_by_zone["place"]) or "none")
    cv2.putText(canvas, header, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                HEADER_COLOR, 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=0,
                        help="V4L2 device index, i.e. /dev/video<N> (default: %(default)s)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--canny-lo", type=int, default=100,
                        help="Canny low threshold (default: %(default)s)")
    parser.add_argument("--canny-hi", type=int, default=200,
                        help="Canny high threshold (default: %(default)s)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print("[live_tag_view] could not open /dev/video%d -- is it already held "
              "by camera.launch.py's v4l2_camera_node? Stop that first, or check "
              "`ls /dev/video*` for the right index." % args.device, file=sys.stderr)
        return 1

    print("[live_tag_view] streaming /dev/video%d at %dx%d -- press q to quit"
          % (args.device, args.width, args.height))
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[live_tag_view] frame read failed", file=sys.stderr)
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("tags", annotate_tags(frame, gray))
            cv2.imshow("edges", cv2.Canny(gray, args.canny_lo, args.canny_hi))

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
