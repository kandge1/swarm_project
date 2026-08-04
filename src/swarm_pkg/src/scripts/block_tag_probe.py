#!/usr/bin/env python3
"""Are the block tags big enough to read from here? Live answer, on the robot.

RUNS ON THE ROBOT (the Pi). No ROS, no rclpy, no DDS -- it opens /dev/video0
with cv2.VideoCapture the way live_tag_view.py does, so it works regardless of
THE OPEN BUG (the /detect_block service timing out). That is the point: the
legibility question does not need the service, and waiting for the service to
be fixed before answering it would be waiting for nothing.

    # on the robot, in a spare terminal
    python3 block_tag_probe.py                 # live, one line per update
    python3 block_tag_probe.py --once          # one frame, full report, exit
    python3 block_tag_probe.py --save /tmp/probe.png

DEVICE CONTENTION -- V4L2 allows ONE reader. block_detector_node.py owns the
camera whenever it is running, and so does live_tag_view.py. Stop the detector
node (Ctrl-C in its terminal) before running this, or neither will open the
device.

WHAT IT PRINTS, and why each column is there:

  px          the tag's mean edge length in the image
  px/module   px / 8, since a 36h11 tag is 8 modules across its black square.
              THIS is the number that decides legibility, not px: a big tag far
              away and a small tag close up decode the same way.
  verdict     measured offline (block_tags_selftest.test_px_per_module_floor)
              against foreshortened, blurred, noised renders --
                  < 3.0   dead, 0% decode
                3.0-4.0   intermittent; presents as a flaky vision bug
                  >= 4.0  reliable, ~95% under realistic blur
  focus       Laplacian variance, the standard cheap focus metric. Below ~100
              the frame is soft, and blur kills a marginal tag outright. The
              lens has a focus floor around 0.220 m, so moving CLOSER to win
              pixels eventually loses more to defocus than it gains.

A tag that is DECODED is by definition legible in that frame -- the verdict
column says whether it will keep being decoded once the arm has moved, the
lighting has changed and the block is at a different angle. Judge on the
verdict, not on the fact that something appeared once.

TYPICAL USE, with the arm driven from mars:

    mars:   python3 joint_trajectory_test.py --degrees 107 84 -138 0 0 135
    robot:  watch this print, then try the next pose

See STACKED_BLOCKS_GUIDE.md for the pose sweep and what each one is worth.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import block_coordinates as bc  # noqa: E402
import zone_vision as zv  # noqa: E402

PX_PER_MODULE_MIN = 3.0
PX_PER_MODULE_GOOD = 4.0
FOCUS_SOFT = 100.0


def verdict(px_per_module):
    if px_per_module >= PX_PER_MODULE_GOOD:
        return "ok"
    if px_per_module >= PX_PER_MODULE_MIN:
        return "INTERMITTENT"
    return "DEAD"


def open_camera(device, width, height):
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(
            "could not open %s.\n"
            "V4L2 allows one reader -- is block_detector_node.py or "
            "live_tag_view.py already running? Stop it first.\n"
            "Check with:  ls /dev/video*   and   fuser -v /dev/video0"
            % device)
    return cap


def grab(cap, warmup):
    """Discard `warmup` frames, then return one. The first frames after the arm
    stops are darker and blurrier than the third -- v4l2's auto-exposure and
    auto-white-balance are still chasing the new scene."""
    frame = None
    for _ in range(max(1, warmup)):
        ok, frame = cap.read()
        if not ok:
            return None
    return frame


def analyse(frame):
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    block_tags = zv.find_block_tags(gray)
    zone_ids = sorted(t for t, _ in zv.detect_all_tags_list(gray)
                      if not bc.is_block_tag(t))
    return gray, block_tags, zone_ids, {
        "focus": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "mean": float(gray.mean()),
    }


def live_line(elapsed, block_tags, zone_ids, stats):
    if block_tags:
        seen = "  ".join("%s %.0fpx/%.1f" % (t.face.label, t.px, t.px_per_module)
                         for t in block_tags)
    else:
        seen = "-- no block tags --"
    return ("[%6.1fs] focus %5.0f  zone %-12s  %s"
            % (elapsed, stats["focus"],
               ",".join(str(z) for z in zone_ids) or "none", seen))


def full_report(block_tags, zone_ids, stats, tag_size_mm, lens_height_m):
    out = []
    out.append("-" * 74)
    out.append("frame: focus %.0f%s, mean brightness %.0f"
               % (stats["focus"],
                  "  <-- SOFT, blur will kill a marginal tag"
                  if stats["focus"] < FOCUS_SOFT else "",
                  stats["mean"]))
    out.append("zone tags visible: %s"
               % (", ".join(str(z) for z in zone_ids) or "none"))
    out.append("")
    if not block_tags:
        out.append("NO BLOCK TAGS (ids %d-%d) IN FRAME."
                   % (bc.BLOCK_TAG_ID_MIN, bc.BLOCK_TAG_ID_MAX))
        out.append("Either none are pointing at the lens, or they are too small")
        out.append("to decode at all -- move closer and see which.")
        out.append("-" * 74)
        return "\n".join(out)

    out.append("  id  face          px   px/module  verdict")
    optimistic = False
    for tag in block_tags:
        flag = ""
        if tag.px < zv.TAG_PX_MEASUREMENT_FLOOR:
            flag = "  (*)"
            optimistic = True
        out.append("  %-3d %-13s %5.1f    %5.2f     %s%s"
                   % (tag.face.tag_id, tag.face.label, tag.px,
                      tag.px_per_module, verdict(tag.px_per_module), flag))
    if optimistic:
        out.append("")
        out.append("  (*) below %.0f px the measurement itself reads HIGH -- up to +14%%,"
                   % zv.TAG_PX_MEASUREMENT_FLOOR)
        out.append("      because subpixel corner refinement pushes corners outward on a")
        out.append("      tag this small. Treat these as an UPPER BOUND. A flagged tag")
        out.append("      reading 3.0 px/module may really be 2.6, which is dead, not")
        out.append("      intermittent. Settle it by moving closer, not by believing it.")

    # Tag scale is a height gauge needing no intrinsics -- but only for a
    # mat-parallel face, and only against a known mat-plane scale. Without a
    # zone homography here there is no mat-plane reference, so this reports the
    # implied lens DISTANCE instead, which needs neither.
    out.append("")
    out.append("implied lens distance (from tag scale, %.1f mm tags):"
               % tag_size_mm)
    for tag in block_tags:
        scale_px_per_m = tag.px / (tag_size_mm / 1000.0)
        # px/m * distance = 551, anchored on the project's own survey.
        distance = 551.0 / scale_px_per_m
        note = ""
        if tag.face.kind == "side":
            note = "   (foreshortened, so this reads FARTHER than it is)"
        out.append("  %-13s %.3f m%s" % (tag.face.label, distance, note))
    out.append("-" * 74)
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--once", action="store_true",
                        help="one frame, full report, exit")
    parser.add_argument("--save", metavar="PATH",
                        help="write an annotated frame (implies a full report)")
    parser.add_argument("--warmup", type=int, default=3,
                        help="frames to discard before keeping one "
                             "(default %(default)s)")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between live updates (default %(default)s)")
    parser.add_argument("--tag-size", type=float, default=19.6,
                        help="PRINTED block tag size in MM, for the implied "
                             "distance (default %(default)s -- the undersized "
                             "first print; use 22.5 once reprinted)")
    parser.add_argument("--lens-height", type=float, default=0.2235,
                        help="lens height above the mat, m (default %(default)s)")
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height)
    started = time.monotonic()
    try:
        if args.once or args.save:
            frame = grab(cap, args.warmup)
            if frame is None:
                raise SystemExit("no frame read from %s" % args.device)
            gray, block_tags, zone_ids, stats = analyse(frame)
            print(full_report(block_tags, zone_ids, stats, args.tag_size,
                              args.lens_height))
            if args.save:
                canvas = frame.copy() if frame.ndim == 3 else cv2.cvtColor(
                    frame, cv2.COLOR_GRAY2BGR)
                for tag in block_tags:
                    pts = tag.corners.astype(np.int32)
                    cv2.polylines(canvas, [pts], True, (255, 0, 255), 2)
                    top = pts[pts[:, 1].argmin()]
                    cv2.putText(canvas, "%s %.1f" % (tag.face.label,
                                                     tag.px_per_module),
                                (top[0], max(12, top[1] - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255),
                                1, cv2.LINE_AA)
                cv2.imwrite(args.save, canvas)
                print("wrote %s" % args.save)
            return

        print("live probe on %s -- Ctrl-C to stop" % args.device)
        print("verdict thresholds: >=%.1f px/module ok, >=%.1f intermittent, "
              "below that dead\n" % (PX_PER_MODULE_GOOD, PX_PER_MODULE_MIN))
        while True:
            frame = grab(cap, 1)
            if frame is None:
                print("cap.read() failed")
                time.sleep(0.2)
                continue
            _gray, block_tags, zone_ids, stats = analyse(frame)
            print(live_line(time.monotonic() - started, block_tags, zone_ids,
                            stats))
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
