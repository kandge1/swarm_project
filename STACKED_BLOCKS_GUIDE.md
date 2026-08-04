# Block tags — printing, sticking, and what the detector does with them

Stage 1–2 groundwork for **THE STACKED-BLOCK PLAN** in `APRIL_TAGS_DEV.md`.
That document owns the plan, the reach envelope and every calibration constant;
this one covers only the block tags themselves and is not a second copy of it.

Written 2026-08-04.

---

## The scheme

Six tags per block — TOP, BOTTOM and **four distinct SIDE ids**. Zone tags own
0–7, so blocks take 8–19:

| | TOP | BOTTOM | SIDE0 | SIDE1 | SIDE2 | SIDE3 |
|---|---|---|---|---|---|---|
| **cube** | 8 | 9 | 10 | 11 | 12 | 13 |
| **cuboid** | 14 | 15 | 16 | 17 | 18 | 19 |

Defined once, in `src/swarm_pkg/src/scripts/block_coordinates.py`. Nothing else
hardcodes it.

**Why four side ids rather than one shared "side" tag**, which was the first
design and is the more obvious economy:

- **It hands over block yaw for free**, and yaw is a *known-broken* signal here.
  `APRIL_TAGS_DEV.md` records that a 30 mm square block classified as `circle`
  in two stills of three, symmetry then read 0, and `zone_vision.py:942` forced
  the yaw to 0.0 during fusion — so the jaws were driven at the block's 52 mm
  diagonal. Reading *"that is SIDE2"* off a decoded id needs no contour, no
  fill ratio and no shape classification, and an integer cannot be quietly
  averaged away downstream.
- **Duplicate ids are silently dropped.** `zone_vision.detect_all_tags()`
  returns a dict, so two tags sharing an id collapse to one — verified, not
  assumed. Harmless for one block, a confident wrong answer the moment stage 0
  puts several in the zone. The new `detect_all_tags_list()` exists for this.
- **Occlusion.** The gripper already hides half the zone tags from every
  reachable hover; a neighbouring block hides more. Any surviving side face
  still identifies the block *and* fixes its yaw.

### Which face is SIDE0

Stand the block with its TOP tag upward and that tag's arrow pointing **away
from you**. SIDE0 is the far face; then counter-clockwise **seen from above** —
SIDE1 left, SIDE2 near, SIDE3 right. (Block frame: SIDE0 faces +Y, SIDE1 −X,
SIDE2 −Y, SIDE3 +X. Counter-clockwise-from-above matches
`zone_vision.ZONE_CORNER_SIGNS`.)

A wrong side index is **silent** — the tag still decodes and the yaw it implies
is 90° out. Check the four read 0,1,2,3 counter-clockwise before the glue dries.

---

## Printing

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
python3 print_block_tags.py --out-dir ../../../../print_sheets
```

Writes `block_tags_cube_A4.png` and `block_tags_cuboid_A4.png` — six tags each,
with cut lines, centre cross-hairs, an orientation arrow, per-face placement
hints and a 150 mm ruler.

Print **to fit the page, not at 100%** — the default `--print-correction`
(16/14) pre-scales the page to cancel the lab printer's fixed shrink, the same
trick `print_tag_sheet.py` uses. Then **measure the ruler**. If it is not
150 mm the print is scaled; that is survivable, but the true size must be
*known*, because the tag-scale height estimate divides by it.

`--report` prints the pixel budget without drawing anything, and
`--face-size` / `--tag-size` re-derive it for a different block.

### Sizing: two measured constraints, not guesses

**A 36h11 tag is 8 modules across the black square** (6×6 data + a one-module
black border). `print_tag_sheet.py:67` says `tag_size/10`; that comment is
wrong. The script asserts the real number against OpenCV at run time.

**The quiet zone must be ≥ 1.25 modules, not the spec's 1.0.** Swept against
background grey level, the failure is a cliff rather than a gradient: at
*exactly* 1.00 module a 48 px-or-larger tag fails against **any** non-white
background and succeeds against white. 1.25 decodes at every background and
size tried. Covered by `block_tags_selftest.test_quiet_zone_floor`.

Together those give **22.5 mm on a 30 mm face**, rounded down to 0.5 mm so it
can be confirmed with a ruler.

---

## The pixel budget — and the finding that matters

`APRIL_TAGS_DEV.md` flagged tag legibility at the angled survey pose as the
experiment that decides stage 2's architecture, and estimated side tags were
not viable. **That is now confirmed offline, and quantified.**

Rendering a tag foreshortened like a side face, blurred and noised:

| px/module | decode rate |
|---|---|
| 2.5 | 0% |
| **3.0** | **10%** ← an ideal-conditions floor only |
| **4.0** | **95%** ← survives realistic blur |

Against the project's own `px/m × distance = 551` invariant, at the agreed
angled pose `107 49 -103 0 0 135` (0.305 m from the zone centre):

| | apparent | px/module | |
|---|---|---|---|
| block TOP (×0.81) | 32.9 px | **4.1** | fine |
| block SIDE (×0.59) | 24.0 px | **3.0** | **10% decode rate** |

**Conclusion: the angled pose can read TOP tags and cannot read SIDE tags.**
For side tags the lens has to come in to **~0.23 m**. Options, unchanged from
`APRIL_TAGS_DEV.md`: pull the vantage closer and pan over more views, or use a
coarser dictionary for block tags only (a 4×4 ArUco family is 6 modules instead
of 8, cutting the requirement by 25%), or use bigger blocks.

This does not block stage 1 — a survey that reads TOP tags still gives block
identity, position and the stacking signal. It constrains stage 2's design.

---

## What the detector now does

`block_detector_node.py` reports every block tag in the frame on **every**
`/detect_block` call, in its own log and on the debug image:

```
  block tag id 8  cube TOP    32.9 px (4.1 px/module, ok)  zone (+12.0, -5.1) mm  implies h +29.2 mm
  block tag id 12 cube SIDE2  24.0 px (3.0 px/module, MARGINAL)
```

Independent of the zone result: a frame can show block tags and **no** zone
tags — the angled pose looks *across* the mat, not down at it — and that is a
useful answer rather than a failure.

The vision itself lives in `zone_vision.find_block_tags()`, not in the node,
keeping the node the plumbing its own docstring promises and making the whole
path testable offline against saved stills.

`zone_xy` is filled in for TOP/BOTTOM faces only. A side tag stands
perpendicular to the mat, so projecting it through a mat-plane homography would
return a plausible-looking position that means nothing. Even for a top tag it
is a *raised* point projected onto the mat and carries the parallax offset
already documented under "Known systematic errors" — good for identifying a
block, not for aiming at one.

### `DetectBlock.srv` is deliberately unchanged

The obvious move is a `BlockTag[]` field. **Do not.** THE OPEN BUG is that
populating a nested message containing a **string** makes `rcl_send_response`
fail on the robot (`string data is not null-terminated`, `serdata.cpp:354`) — a
`BlockTag` carrying a face name is exactly that shape, on an interface whose
build is already the prime suspect, and it would need a coordinated rebuild on
both machines.

The legibility question is answered entirely by the robot's log and the debug
image, neither of which crosses DDS. If mars ever does need these, add
**parallel primitive arrays** (`uint16[] ids`, `float64[] px`, …) — never a
nested message with a string. That is `APRIL_TAGS_DEV.md`'s own fallback.

---

## Height from tag scale — no intrinsics needed

A mat-parallel tag raised `h` above the mat is magnified by `d/(d−h)`, so

```
h = d × (1 − mat_scale / tag_scale)
```

`block_coordinates.height_from_scale()`. Verified end-to-end on a synthetic
scene: a tag rendered at the magnification a 30 mm block produces read back at
**29.2 mm**. There are no camera intrinsics anywhere in this repo, so
`solvePnP` is unavailable — this needs none.

Two caveats it enforces rather than hides: it is **TOP/BOTTOM only**
(`BlockFace.is_mat_parallel`), and it takes a `view_tilt_deg` because raising a
tag shortens the optical-axis range by `h·cos(tilt)` — a naive reading at the
36° pose under-reports by ~19%. A negative result is returned **signed, not
clamped**: it cannot happen physically, so it is evidence the lens-height input
is wrong, and swallowing it would hide the one number that says so.

---

## Offline tests

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
python3 block_tags_selftest.py      # the block tag scheme, ~2 s
python3 zone_vision_selftest.py     # the zone homography, unchanged, ~1 s
python3 block_coordinates.py        # prints the scheme, self-checks
python3 print_block_tags.py --report
```

Both selftests expect `0 failure(s)`.

---

## THE VISIBILITY TEST — run this next

Answers one question: **at what pose are the block tags actually readable?**
It does not use `/detect_block`, so THE OPEN BUG does not block it.

### Setup

One tagged block at the pickup zone centre. Arm terminals 1 and 2 as usual.
**Stop `block_detector_node.py`** — V4L2 allows one reader and the probe needs
the camera.

```bash
# robot, terminal 4 (in place of block_detector_node)
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_tag_probe.py
```

It prints a line twice a second. Leave it running and drive the arm from mars:

```bash
# mars, terminal 5
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
python3 joint_trajectory_test.py --degrees 107 49 -103 0 0 135
```

Then walk in through the sweep below, watching the probe. `--once` gives a full
report at a pose; `--save /tmp/probe.png` writes an annotated frame.

### The sweep

All at tool tilt 36° with `J1=107 J4=0 J5=0 J6=135`, so only the two pitch
joints change and the view angle stays constant — the only variable is
distance. Joint angles are driven directly, no IK.

| pose | flange→zone | fingertip z | clears a 3-stack | TOP px/module (19.6 mm) | (22.5 mm) |
|---|---|---|---|---|---|
| `107 95 -149 0 0 135` | 0.253 | 82.7 mm | **−3 mm — collides** | 4.3 | 5.0 |
| `107 90 -144 0 0 135` | 0.260 | 92.3 mm | +6 mm | 4.2 | 4.8 |
| `107 84 -138 0 0 135` | 0.269 | 103.8 mm | +18 mm | 4.1 | 4.7 |
| `107 75 -129 0 0 135` | 0.281 | 120.9 mm | +35 mm | 3.9 | 4.5 |
| `107 66 -120 0 0 135` | 0.291 | 137.2 mm | +51 mm | 3.8 | 4.3 |
| `107 49 -103 0 0 135` | 0.307 | 164.7 mm | +79 mm | 3.6 | 4.1 |

The last row is the pose `APRIL_TAGS_DEV.md` agreed on. **It is the worst one in
the table for legibility**, and the top rows buy real pixels — but the gripper
comes down as the arm reaches in, and the top row would strike a 3-stack.

**`107 84 -138 0 0 135` is the recommendation**: 4.1 px/module on TOP tags even
with your undersized print, and 18 mm of clearance over the tallest stack the
plan allows. Testing today with a single block on the mat, clearance is 78 mm
at every row, so start at the top and work down.

### What to expect

**TOP tags should decode. SIDE tags will not.** The lens cannot get close
enough — and that conclusion is now boxed in from both sides:

- side tags need the lens within ~0.23 m for a 22.5 mm tag, nearer still for
  19.6 mm
- the lens has a **focus floor at 0.220 m**, and blur kills a marginal tag
  outright

There is no pose that satisfies both. At the closest *focusable* distance in
the table (0.253 m) side tags reach only 3.2 px/module at 19.6 mm, 3.6 at
22.5 mm — neither is the 4.0 that survives blur. **Side tags at a 36° view are
not achievable with 30 mm blocks and this camera.** Stage 2 has to lean on TOP
tag visibility plus tag scale, which is what `APRIL_TAGS_DEV.md` already
preferred on other grounds.

Watch the `focus` column. Below ~100 the frame is soft and any marginal reading
is worthless.

### Reading the numbers honestly

`tag_pixel_size` **over-reads by up to 14% below 28 px** — subpixel corner
refinement pushes corners outward on a small tag — so the probe flags those
with `(*)`. The bias flatters exactly the marginal cases. A flagged 3.0
px/module may really be 2.6, which is dead rather than intermittent.

---

## Not done

- **Nothing has run on hardware.** Every number above is arithmetic or
  synthetic. The tags have not been printed, stuck on, or looked at.
- **Stage 1 pan/survey and stage 2 topology are not written.** Use
  `joint_trajectory_test.py --degrees 107 49 -103 0 0 135` to drive the arm to
  the survey pose meanwhile; it bypasses IK, which is where the fragile things
  in this project live.
- **The cuboid's dimensions are assumed to be a 30 mm square cross-section.**
  If its square face is not 30 mm, regenerate with
  `--block cuboid --face-size <metres>`.
- **`/detect_block` still times out** — THE OPEN BUG. The block-tag work does
  not depend on it (the log and debug image are robot-side), but stage 0 does.
