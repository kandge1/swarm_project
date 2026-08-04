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

Writes `block_tags_cube_LETTER` and `block_tags_cuboid_LETTER` as **both `.pdf`
and `.png`** — six **25.4 mm (1 in)** tags each, with cut lines, centre
cross-hairs, an orientation arrow, a per-face placement hint and a **150 mm
calibration ruler**. `--paper a4` and `--fit-face` (22.5 mm tags) are the other
two configurations.

**Print the PDF, at 100% / Actual Size**, on the paper named in the filename —
never "fit to page" or "shrink to fit". Then **measure the ruler**, not a tag.

### The print chain, measured — read this before touching `--print-correction`

Four prints, all of the same nominal 22.5 mm tag:

| what was printed | printed tag | ratio |
|---|---|---|
| PNG, `--print-correction` 16/14 | 19.6 mm | 0.871 |
| PNG, `--print-correction` 1.0 | 19.58 mm | 0.870 |
| PDF on A4 geometry, Actual Size | 21.27 mm | 0.945 |
| PDF, content-scaled | 21 mm | |

**Rows 1 and 2 are the finding.** Two corrections 14% apart produced the same
physical tag, so the correction was not over-cancelling a shrink — it had no
effect on physical size *at all*. It scaled `px_per_mm`, which scales the canvas
along with the content, so the tag kept the same **fraction** of the page either
way, and a fraction of a page is exactly what survives a printer mapping an
image onto paper. `print_zone_tags.py` measured the same no-op independently: a
1.0926× pre-scale moved its printed tag from 23 mm to 23 mm.

Underneath it, **a `cv2.imwrite` PNG carries no `pHYs` chunk** and so never
states how many millimetres it is meant to be. "Actual Size" has nothing to be
actual against, and the dialog can only fit pixels to paper whatever it is set
to. What landed 2026-08-04:

1. **PDF output** with a real page box (verified by reading `/MediaBox`), so
   Actual Size is well defined. The PNG carries 300 dpi metadata now too.
2. **`--print-correction` scales content inside a page that keeps the paper's
   size**, so it changes the tag's fraction of the page and survives the
   mapping — the same mechanism as `print_zone_tags.CONTENT_SCALE`.
3. **The page is authored on Letter**, like the zone sheets, because that is
   what these printers feed. A page-size mismatch is a second scale error
   stacked on the first, and under a non-aspect-preserving fit it is what made
   the zone sheet come out *non-square* before.
4. **A 150 mm ruler on the sheet**, and `--measured-ruler` to close the loop.

### Recalibrating

Measure the ruler on the printed sheet. If it is not 150 mm, re-run with the
same paper and dialog settings, adding what you measured:

```bash
python3 print_block_tags.py --out-dir ../../../../print_sheets \
    --measured-ruler 154.42
```

That rescales the correction by 150/measured for you. `--print-correction`
itself takes a **plain number** — it cannot evaluate `1.0925*150/154.42`.

**Measure the ruler, not a tag.** A ruler that reads to 1 mm is 0.7% over a
150 mm baseline and 4% over a 25 mm tag — and four rounds of this loop were
spent measuring tags. The ruler scales with the content, which is exactly what
makes it a valid instrument.

### The correction is 1.0612, and it is NOT the zone sheets' number

Sharing one constant across both sheets was tried and this measurement ruled it
out. A block sheet drawn at `print_zone_tags.CONTENT_SCALE` (1.0925) printed its
ruler at **154.42 mm — 3% over**. This chain needs 1.0925 × 150/154.42 = **1.0612**.

Corroborated independently: the implied shrink 1/1.0612 = **0.942** matches the
**0.945** measured off the A4 PDF two prints earlier. Two papers, two
measurements, one number — so the residual is the printer's own printable-area
inset rather than a paper-size fit, and it is deterministic enough to cancel.

`CONTENT_SCALE` is not wrong; it is fitted to a **different chain** — a PNG
through a campus printer that applies fit-to-page unconditionally. **If the zone
mat was printed through the chain used here, its 101.6 mm square is ~3%
oversized**, and `zone_vision` takes `DEFAULT_ZONE_SIZE` as ground truth, so that
error would pass silently into every position it reports. Worth measuring the mat
before trusting millimetres out of the detector.

The correction is a property of printer + paper, so once found it stays put
until the tray changes.

If you choose to live with an off-size print instead, the true size must be
*known*, because the tag-scale height estimate divides by it:
`block_detector_node.py`'s `block_tag_size` must be set to what you measured,
not to the nominal 25.4.

`--report` prints the pixel budget without drawing anything, and
`--face-size` / `--tag-size` re-derive it for a different block.

### Sizing: the constraint that was traded away, on purpose

**A 36h11 tag is 8 modules across the black square** (6×6 data + a one-module
black border). `print_tag_sheet.py:67` says `tag_size/10`; that comment is
wrong. The script asserts the real number against OpenCV at run time.

**The quiet zone wants ≥ 1.25 modules, not the spec's 1.0.** Swept against
background grey level, the failure is a cliff rather than a gradient: at
*exactly* 1.00 module a 48 px-or-larger tag fails against **any** non-white
background and succeeds against white. 1.25 decodes at every background and
size tried. Covered by `block_tags_selftest.test_quiet_zone_floor`.

That arithmetic gives **22.5 mm on a 30 mm face** — and **the default is 25.4 mm
anyway**, decided 2026-08-04:

| | quiet zone | TOP px/module | SIDE px/module |
|---|---|---|---|
| 22.5 mm (`--fit-face`) | 1.25 modules | 4.1 | 3.0 |
| **25.4 mm (default)** | **0.72 modules** | **4.6** | **3.4** |

0.72 modules is **under the cliff**. The bet is that the white does not have to
stop at the sticker edge — a light-coloured block face carries the rest of the
quiet zone itself — and it is **untested**. If block tags decode on the sheet
and not on a block, this is the first thing to suspect, and `--fit-face` is the
retreat at a cost of 0.5 px/module.

The cut square never exceeds the face: past that point the quiet zone is
squeezed rather than the square grown, so cutting on the printed line always
gives a sticker that lies flat on a 30 mm face.

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
angled pose `107 49 -103 0 0 135` (0.305 m from the zone centre), at both tag
sizes:

| | 22.5 mm | | **25.4 mm** | |
|---|---|---|---|---|
| block TOP (×0.81) | 32.9 px | 4.1 | **37.2 px** | **4.6** fine |
| block SIDE (×0.59) | 24.0 px | 3.0 | **27.1 px** | **3.4** marginal |

**Conclusion: the angled pose can read TOP tags and cannot read SIDE tags —
at either size.** Going to 1 in moves the distance at which side tags become
comfortable from 0.229 m to 0.258 m; it does not make that pose work.

For side tags the lens has to come in to **~0.26 m**. Options, unchanged from
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
  block tag id 8  cube TOP    37.2 px (4.6 px/module, ok)  zone (+12.0, -5.1) mm  implies h +29.2 mm
  block tag id 12 cube SIDE2  27.1 px (3.4 px/module, MARGINAL)
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

### THE POSE IS `107 49 -103 0 0 135`

Settled on hardware 2026-08-04. Earlier drafts of this file proposed folding the
arm in closer (`107 95 -149`, `107 84 -138`) to win pixels. **Those poses drive
the camera into the arm's own links** — found by trying it.

The analysis behind them was wrong in a specific, worth-recording way: it
computed fingertip height and checked clearance over the *blocks*, and never
checked link-against-link **self-collision**. Driving joint angles directly is
what makes this survey robust — it bypasses IK, where everything fragile in this
project lives — but it also bypasses MoveIt's collision model, so nothing was
checking. A high `J2` with a strongly negative `J3` folds the forearm back over
the shoulder, and the flange position alone says nothing about that.

If a closer vantage is ever wanted, screen candidates through
`check_state_validity.py` (which does consult the collision model) before
putting them in a table.

### Setup

One tagged block at the pickup zone centre. Arm terminals 1 and 2 as usual.
**Stop `block_detector_node.py`** — V4L2 allows one reader and the probe needs
the camera.

```bash
# mars: drive the arm there first, no IK involved
python3 joint_trajectory_test.py --degrees 107 49 -103 0 0 135

# robot, terminal 4 (in place of block_detector_node)
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_tag_probe.py --show
```

`--show` opens a live window with every tag boxed and named, coloured by
whether it is actually decoding. Needs a display — run it over `ssh -X`. Without
one it says so and falls back to the text feed, which carries the same numbers.

| key | |
|---|---|
| `q` | quit |
| `s` | save an annotated snapshot |
| `r` | reset the decode rates — **do this after every arm move**, or the window is still averaging in the old pose |

`--once` prints a full report for a single frame; `--save PATH` writes one
annotated frame and exits, which is the headless option.

### Read the DECODE RATE, not the pixel count

The window's main panel is the fraction of the last 40 frames in which each tag
decoded. **That is the primary instrument**, because the pixel measurement is
not trustworthy at the sizes being judged:

> On a small tag OpenCV's corner refinement frequently locks onto the outer
> edge of the **white quiet zone** instead of the black square. Measured: a
> 24 px tag reported as 32.2 px — which is exactly the 34 px quiet-zone square.
> A **+34% over-read**, in the direction that makes a dead tag look fine.

Readings below 28 px are marked `?` for this reason. A decode rate has none of
that problem: it is the exact question — *will this tag be read from this pose*
— answered by counting. Hold the arm still and let the percentages settle.

### What to expect at this pose

| | 19.6 mm (your first print) | 22.5 mm (reprinted) |
|---|---|---|
| TOP | 3.6 px/module — intermittent | **4.1 — ok** |
| SIDE | 2.6 — dead | 3.0 — dead |

**Reprinting matters now.** Having settled on the far pose, tag size is the only
lever left, and 19.6 → 22.5 mm moves TOP tags out of the intermittent band. With
`--print-correction` now defaulting to 1.0 a reprint should come out right.

If TOP tags still will not hold a high decode rate at 22.5 mm, the next lever is
the one `APRIL_TAGS_DEV.md` already suggests: **a coarser dictionary for block
tags only.** A 4×4 ArUco family is 6 modules across instead of 8, so the same
22.5 mm tag at the same pose gives 5.5 px/module instead of 4.1 — a 33%
improvement for a one-line change, and the id space needed here is tiny. Zone
tags stay 36h11.

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
