#!/usr/bin/env python3
"""Are the block tags big enough to read from here? Live answer, on the robot.

RUNS ON THE ROBOT (the Pi). No ROS, no rclpy, no DDS -- it opens /dev/video0
with cv2.VideoCapture the way live_tag_view.py does, so it works regardless of
THE OPEN BUG (the /detect_block service timing out). That is the point: the
legibility question does not need the service, and waiting for the service to
be fixed before answering it would be waiting for nothing.

    # on the robot, in a spare terminal
    python3 block_tag_probe.py --show          # live window, boxes and names
    python3 block_tag_probe.py                 # text only (headless)
    python3 block_tag_probe.py --once          # one frame, full report, exit
    python3 block_tag_probe.py --save /tmp/probe.png

--show needs a display: run it over `ssh -X`. Without one it says so and drops
to the text feed, which carries the same numbers. In the window: q quits, s
saves a snapshot, and r resets the decode rates -- PRESS r AFTER EVERY ARM MOVE,
or the rolling window is still averaging in the pose you just left.

THE DECODE RATE IS THE ANSWER, not the pixel count. The window's main panel is
the fraction of the last N frames in which each tag decoded, and that is the
question being asked -- "will this tag be read from this pose" -- measured by
counting rather than predicted from a size. It matters because the size
measurement is unreliable at exactly the sizes worth judging: on a small tag the
corner refinement often locks onto the outer edge of the white QUIET ZONE rather
than the black square, and a 24 px tag has been measured reporting 32.2 px, a
+34% over-read in the flattering direction. Those readings are marked '?'.

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
import collections
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


class DecodeRate(object):
    """How often each tag decodes, over a rolling window of frames.

    THIS IS THE PRIMARY INSTRUMENT, not px/module, and the reason is that the
    pixel measurement cannot be trusted at exactly the sizes being judged. On a
    small tag OpenCV's corner refinement frequently locks onto the outer edge of
    the WHITE QUIET ZONE rather than the black square -- measured: a 24 px tag
    reported as 32.2 px, which is the 34 px quiet-zone square, a +34% over-read
    that makes a dead tag look fine (see zone_vision.tag_pixel_size).

    A decode rate has none of that problem. It is the exact question -- "will
    this tag be read from this pose" -- answered by counting, and a tag that
    decodes in 40 of 40 frames is legible whatever any pixel measurement says.
    Hold the arm still and watch the percentages settle.
    """

    def __init__(self, window):
        self.window = window
        self.frames = collections.deque(maxlen=window)

    def update(self, block_tags):
        self.frames.append(frozenset(t.face.tag_id for t in block_tags))

    def rates(self):
        """[(tag_id, rate 0-1, hits, total)], best first. Only ids seen at
        least once -- a tag that has never appeared is not evidence of a poor
        rate, it is evidence of nothing."""
        total = len(self.frames)
        if not total:
            return []
        counts = collections.Counter()
        for seen in self.frames:
            counts.update(seen)
        out = [(tag_id, hits / float(total), hits, total)
               for tag_id, hits in counts.items()]
        out.sort(key=lambda row: (-row[1], row[0]))
        return out

    def rate_for(self, tag_id):
        for other, rate, _hits, _total in self.rates():
            if other == tag_id:
                return rate
        return None


def display_available():
    """Is there a display to open a window on?

    CHECKED UP FRONT BECAUSE THE FAILURE CANNOT BE CAUGHT. With no display,
    cv2.namedWindow does not raise -- Qt fails to load its xcb platform plugin
    and calls abort(), and the process core-dumps out from under any
    try/except. Verified. So `except cv2.error` around the window call is not a
    fallback, it is decoration, and on a headless Pi over plain ssh -- the
    normal way this script gets run -- it would take the probe down with it.

    An env check is crude, but it is the thing that can actually be tested
    before the abort happens.
    """
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


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
    # Corners, not just ids: the viewer draws them. Kept as a list so two tags
    # sharing an id both survive -- see zone_vision.detect_all_tags_list.
    zone_tags = [(t, c) for t, c in zv.detect_all_tags_list(gray)
                 if not bc.is_block_tag(t)]
    zone_ids = sorted(t for t, _ in zone_tags)
    return gray, block_tags, zone_tags, zone_ids, {
        "focus": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "mean": float(gray.mean()),
    }


# BGR. Block tags are coloured BY VERDICT, so the thing you are trying to judge
# is readable across the room without reading any numbers: green means this pose
# works, red means it does not.
COLOR_OK = (0, 220, 0)
COLOR_INTERMITTENT = (0, 190, 255)
COLOR_DEAD = (0, 0, 255)
COLOR_ZONE = (170, 170, 170)        # grey: context, not the subject
COLOR_HEADER = (255, 255, 255)


def verdict_color(px_per_module):
    if px_per_module >= PX_PER_MODULE_GOOD:
        return COLOR_OK
    if px_per_module >= PX_PER_MODULE_MIN:
        return COLOR_INTERMITTENT
    return COLOR_DEAD


def _label(canvas, text, origin, color, scale=0.45):
    """Text on a filled box, kept inside the frame.

    Plain putText over a camera frame of a black-and-white tag is frequently
    unreadable, which defeats the point of a viewer. The clamping matters as
    much: a tag near the right edge is exactly the one whose label would run off
    the frame, and a tag near the edge is a normal thing to be looking at.
    """
    height, width = canvas.shape[:2]
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = min(max(2, int(origin[0])), max(2, width - w - 4))
    y = min(max(h + 3, int(origin[1])), height - base - 2)
    cv2.rectangle(canvas, (x - 2, y - h - 3), (x + w + 2, y + base), (0, 0, 0), -1)
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1,
                cv2.LINE_AA)


def rate_color(rate):
    if rate >= 0.90:
        return COLOR_OK
    if rate >= 0.50:
        return COLOR_INTERMITTENT
    return COLOR_DEAD


def annotate(frame, block_tags, zone_tags, stats, decode_rate=None):
    """Live view: every tag boxed and named, coloured by whether it is legible."""
    canvas = frame.copy() if frame.ndim == 3 else cv2.cvtColor(
        frame, cv2.COLOR_GRAY2BGR)

    for tag_id, corners in zone_tags:
        pts = corners.astype(np.int32)
        cv2.polylines(canvas, [pts], True, COLOR_ZONE, 1)
        _label(canvas, bc.label(tag_id), pts[pts[:, 1].argmin()] + (0, -5),
               COLOR_ZONE, 0.40)

    for tag in block_tags:
        pts = tag.corners.astype(np.int32)
        # Colour by DECODE RATE when there is one, falling back to px/module
        # only for the first few frames. The rate is the trustworthy signal.
        rate = decode_rate.rate_for(tag.face.tag_id) if decode_rate else None
        color = rate_color(rate) if rate is not None else verdict_color(
            tag.px_per_module)
        cv2.polylines(canvas, [pts], True, color, 2)
        # A corner dot marks the tag's own "up", so a tag stuck on rotated is
        # visible as such rather than showing up later as a 90 deg yaw error.
        cv2.circle(canvas, tuple(pts[0]), 3, color, -1)
        top = pts[pts[:, 1].argmin()]
        suffix = "?" if tag.px < zv.TAG_PX_MEASUREMENT_FLOOR else ""
        text = "%s  %.0f%%" % (tag.face.label,
                               100 * rate) if rate is not None else tag.face.label
        _label(canvas, "%s  (%.1f px/mod%s)" % (text, tag.px_per_module, suffix),
               (top[0], top[1] - 6), color)

    focus_color = COLOR_HEADER if stats["focus"] >= FOCUS_SOFT else COLOR_DEAD
    _label(canvas, "focus %.0f%s   mean %.0f   zone %d   block %d"
           % (stats["focus"], "" if stats["focus"] >= FOCUS_SOFT else " SOFT",
              stats["mean"], len(zone_tags), len(block_tags)),
           (6, 16), focus_color, 0.48)

    # The decode-rate panel. Deliberately the most prominent thing after the
    # boxes: it is what decides whether this pose is usable.
    y = 38
    if decode_rate is not None and decode_rate.rates():
        _label(canvas, "DECODE RATE over last %d frames:" % len(decode_rate.frames),
               (6, y), COLOR_HEADER, 0.44)
        y += 17
        for tag_id, rate, hits, total in decode_rate.rates():
            _label(canvas, "  %-13s %3.0f%%  (%d/%d)"
                   % (bc.label(tag_id), 100 * rate, hits, total),
                   (6, y), rate_color(rate), 0.44)
            y += 16
        y += 3

    _label(canvas, "hold the arm STILL and let the rates settle",
           (6, y), COLOR_HEADER, 0.38)
    _label(canvas, "(px/mod is unreliable below %.0f px -- marked ?)"
           % zv.TAG_PX_MEASUREMENT_FLOOR, (6, y + 15), COLOR_HEADER, 0.38)
    _label(canvas, "q quit    s save snapshot    r reset rates",
           (6, y + 30), COLOR_HEADER, 0.38)
    return canvas


def live_line(elapsed, block_tags, zone_ids, stats, decode_rate=None):
    if decode_rate is not None and decode_rate.rates():
        seen = "  ".join("%s %.0f%%" % (bc.label(i), 100 * r)
                         for i, r, _h, _t in decode_rate.rates())
    elif block_tags:
        seen = "  ".join("%s %.0fpx" % (t.face.label, t.px) for t in block_tags)
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
    parser.add_argument("--show", action="store_true",
                        help="live window with every tag boxed and named, "
                             "coloured by legibility. Needs a display: run over "
                             "`ssh -X`, or use --save when headless.")
    parser.add_argument("--snapshot-prefix", default="/tmp/block_tags",
                        help="where `s` writes snapshots (default %(default)s)")
    parser.add_argument("--once", action="store_true",
                        help="one frame, full report, exit")
    parser.add_argument("--save", metavar="PATH",
                        help="write an annotated frame (implies a full report)")
    parser.add_argument("--warmup", type=int, default=3,
                        help="frames to discard before keeping one "
                             "(default %(default)s)")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="seconds between live TEXT updates; the window "
                             "refreshes every frame (default %(default)s)")
    parser.add_argument("--rate-window", type=int, default=40,
                        help="frames in the rolling decode-rate window "
                             "(default %(default)s)")
    parser.add_argument("--tag-size", type=float, default=19.6,
                        help="PRINTED block tag size in MM, for the implied "
                             "distance (default %(default)s -- the undersized "
                             "first print; use 22.5 once reprinted)")
    parser.add_argument("--lens-height", type=float, default=0.2235,
                        help="lens height above the mat, m (default %(default)s)")
    args = parser.parse_args()

    cap = open_camera(args.device, args.width, args.height)
    started = time.monotonic()
    snapshots = 0
    try:
        if args.once or args.save:
            frame = grab(cap, args.warmup)
            if frame is None:
                raise SystemExit("no frame read from %s" % args.device)
            _gray, block_tags, zone_tags, zone_ids, stats = analyse(frame)
            print(full_report(block_tags, zone_ids, stats, args.tag_size,
                              args.lens_height))
            if args.save:
                cv2.imwrite(args.save,
                            annotate(frame, block_tags, zone_tags, stats))
                print("wrote %s" % args.save)
            return

        show = args.show
        if show and not display_available():
            # Never reached by an exception handler -- see display_available().
            print("--show asked for, but neither DISPLAY nor WAYLAND_DISPLAY is\n"
                  "set, so there is nowhere to put a window. Reconnect with\n"
                  "`ssh -X` (or `ssh -Y`), or use --save to write an annotated\n"
                  "frame. Falling back to the text feed; the numbers are the\n"
                  "same either way.\n")
            show = False
        if show:
            try:
                cv2.namedWindow("block tags", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("block tags", args.width, args.height)
            except cv2.error as exc:
                # Reached when a display EXISTS but this OpenCV cannot draw on
                # it -- built without GUI support, which is a real possibility
                # for a distro python3-opencv on the Pi.
                print("a display is set but OpenCV cannot open a window (%s).\n"
                      "This build may lack GUI support; use --save instead.\n"
                      "Falling back to text.\n"
                      % str(exc).strip().splitlines()[-1][:100])
                show = False

        print("live probe on %s -- %s to stop"
              % (args.device, "q in the window, or Ctrl-C" if show else "Ctrl-C"))
        print("verdict thresholds: >=%.1f px/module ok, >=%.1f intermittent, "
              "below that dead\n" % (PX_PER_MODULE_GOOD, PX_PER_MODULE_MIN))
        last_print = 0.0
        tracker = DecodeRate(args.rate_window)
        while True:
            frame = grab(cap, 1)
            if frame is None:
                print("cap.read() failed")
                time.sleep(0.2)
                continue
            _gray, block_tags, zone_tags, zone_ids, stats = analyse(frame)
            tracker.update(block_tags)
            now = time.monotonic()

            if show:
                canvas = annotate(frame, block_tags, zone_tags, stats, tracker)
                cv2.imshow("block tags", canvas)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    path = "%s_%02d.png" % (args.snapshot_prefix, snapshots)
                    cv2.imwrite(path, canvas)
                    print("saved %s" % path)
                    snapshots += 1
                if key == ord("r"):
                    # After moving the arm the old frames describe the old pose,
                    # and a stale window is worse than no window.
                    tracker = DecodeRate(args.rate_window)
                    print("decode rates reset")

            # The text feed keeps running behind the window: it is the thing
            # that can be scrolled back through afterwards, and it is all there
            # is when running headless.
            if now - last_print >= args.interval:
                print(live_line(now - started, block_tags, zone_ids, stats,
                                tracker))
                last_print = now
            if not show:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
