# Open-loop grasp calibration, 2026-08-07 → 2026-08-11

**Result: open-loop grasp error went from ~20-27 mm (2026-08-06) to 5.8 mm, then
to a measured RMS of ~0.6 mm, worst pose 1.0 mm.** The 1-2 mm target is met and
verified with a caliper at three reaches and two bearings.

This file is the record of how, what was wrong on the way, and what is still
open. It supersedes `src/swarm_pkg/testing/saturday_plan.txt` and
`src/swarm_pkg/testing/sunday_plan.txt`, which are kept as the contemporaneous
notes.

---

## 1. The final model

Three lateral terms, in two different frames. **The frames are the whole story
and getting them confused is what cost four days.**

```python
# pick_place.py -- POSE frame, resolved against the bearing to (x, y)
JAW_RADIAL_OFFSET_M          = -0.0199
JAW_TANGENTIAL_OFFSET_M      =  0.00368     # base, at zero reach
JAW_TANGENTIAL_PER_M_REACH   = -0.05930     # per metre of hypot(x, y)
JAW_TANGENTIAL_REACH_RANGE_M = (0.121, 0.222)

# pick_place.py -- TOOL frame, rotates with the commanded wrist yaw
JAW_PERP_OFFSET_M            = -0.004       # perpendicular to jaw closing axis

# explore.py -- the survey's radial bias
ORIGIN_RADIAL_BIAS_M         = -0.0272
```

`compensate_for_tip_swing` applies the first three against `r_hat`/`t_hat` and
the fourth against the jaw-perpendicular `p_hat`.

### Measured residual, 2026-08-11 evening

| pose | wrist yaw | bearing | r (mm) | residual |
|---|---|---|---|---|
| L | 90 | +89.9 | 174 | 0.0 mm |
| O | 90 | −1.0 | 170 | 0.0 mm |
| O | 0 | −1.0 | 174 | −1.0 mm |
| N | 0 | +0.1 | 126 | 0.0 mm |
| O | 0 | −1.1 | 177 | 0.0 mm |
| H | 0 | −2.0 | 225 | +1.0 mm |

**mean |residual| 0.33 mm, RMS 0.58 mm, worst 1.0 mm.** Confirmed afterwards at
N (~2 mm, inside the dead band), H (perfect) and L (inside the dead band).

### Working envelope

Reach **121-222 mm (4.8-8.7 in)**, the span the tangential line was fitted over.
Independently confirmed at the bench: **4 in fails and 10 in fails**, bracketing
the fitted span from both sides without being told where it was. Outside the
span `compensate_for_tip_swing` prints a one-time extrapolation warning, because
the tangential term carries a 3.4 deg angular component and extrapolates badly.

---

## 2. The three errors that actually mattered

### 2.1 J1 lost motion was never live on the pick path

`move_arm_to` defaults `unidirectional=False`, and **not one call site in
`tag_pick_place.py` ever passed it**. Only `pick_place.py`'s legacy
fixed-coordinate flow and `zone_calibrate.py` did — which is why the survey that
*measured* `J1_RESIDUAL_BIAS_DEG = 1.10` saw it work while the code that picks
blocks never got it.

Extracted from transcripts, 73 moves: `+ve` commands land −0.89°, `−ve` land
+0.97°, |err| 0.94° (sd 0.12), **identical for a 0.9° move and a 22° one** — so
lost motion, not tracking error. Three round trips to the same commanded point
from opposite sides landed 6.3 / 6.0 / 6.9 mm apart.

Fixed by passing `unidirectional=True` on both pre-grasp parks. Deliberately
**not** on the descent: `compensate_for_tip_swing`'s lateral terms have no
z-dependence, so hover and grasp share a bit-identical commanded flange XY
(verified: difference `(0.0e+00, 0.0e+00)`). J1 never turns during the descent
and inherits the park's flank. After the fix, same-point spread **0.33 mm**.

**The dead band, in task units.** Full backlash 1.88°, so its arc length grows
with reach:

| r (mm) | full band | first-nudge loss |
|---|---|---|
| 126 | 4.1 mm | 2.6 mm |
| 176 | 5.8 mm | 3.6 mm |
| 229 | 7.5 mm | 4.7 mm |

Measured directly at N: typed `0 5` three times, delivered **+2.42 / +5.38 /
+5.76 mm**. The dead band is paid **once**, on the first correction, then
delivery is full. Confirmed at H: 10 mm typed to fix a 5 mm error, so 5.0 mm
lost against 4.7 mm predicted — 0.3 mm agreement, 103 mm away from where the
1.17° figure was first measured.

Also confirmed: the radial axis has **no** meaningful lost motion (typed +2.0,
delivered +1.94), so the dead band is J1/tangential only.

**Operational rule: nudge in ONE step, never several small ones.** Each reversal
donates up to a full backlash. Creeping up in 5s is what produced a +3.86 mm
overshoot at N.

### 2.2 `JAW_TANGENTIAL_OFFSET_M` is a line in reach, not a constant

Fitted from caliper readings with the jaw axis pinned by `--force-grasp-yaw 90`:

```
r = 121 mm   tangential residual +6.0 mm
r = 173 mm                       +3.0 mm    (two runs, both -3 -3 exactly)
r = 222 mm                        0.0 mm
```

Equal reach steps (52, 49 mm) gave equal error steps (−3, −3). Least squares
`13.18 − 59.14·r` mm, residuals **−0.03 / +0.07 / −0.04 mm**, zero crossing at
222.8 mm. A constant could not fit this at any value — **the "4 mm of
irreducible scatter" reported earlier was mostly this line sampled at scattered
radii.** It was signal, not noise.

### 2.3 The last 4 mm is TOOL-fixed, and no pose-frame constant can express it

Six runs across two bearings 91° apart, two wrist yaws, and 101 mm of reach:

| pos | yaw | bearing | r | correction | radial | tang | jaw | **perp** |
|---|---|---|---|---|---|---|---|---|
| L | 90 | +89.9 | 174 | (−4.0, +0.0) | −0.01 | +4.00 | −0.00 | **+4.00** |
| O | 90 | −1.0 | 170 | (−4.0, +0.0) | −4.00 | −0.07 | −0.00 | **+4.00** |
| O | 0 | −1.0 | 174 | (+0.0, +3.0) | −0.05 | +3.00 | +0.00 | **+3.00** |
| N | 0 | +0.1 | 126 | (+0.0, +4.0) | +0.01 | +4.00 | +0.00 | **+4.00** |
| O | 0 | −1.1 | 177 | (+0.0, +4.0) | −0.08 | +4.00 | +0.00 | **+4.00** |
| H | 0 | −2.0 | 225 | (+0.0, +5.0) | −0.17 | +5.00 | +0.00 | **+5.00** |

In the **pose frame** it is noise: radial swings −4..0, tangential 0..+4, no
pattern. In the **tool frame** the jaw-axis component is zero everywhere and the
perpendicular is **+4.00 mm, sd 0.63**.

The clincher: **rotating the wrist 90° rotated the residual 90° in the world.**
`(0,+4)` became `(−4,0)`, and `R(+90)·(0,+4)` is exactly `(−4,0)`.

`JAW_RADIAL/TANGENTIAL_OFFSET_M` are resolved against the *bearing*, so they
cannot express a tool-fixed offset at any value. **This is why those two
constants kept moving all week: fitted at one wrist yaw they look right, and a
run at another yaw contradicts them.** Hence `JAW_PERP_OFFSET_M`.

---

## 3. Instrument and tooling fixes (these mattered as much as the constants)

- **The nudge is a control action; the caliper is a measurement.** They are not
  the same number. At N the caliper read 9.70 mm while the operator typed 15 mm
  of nudge, because the first 5 mm delivered only 2.42 and the third overshot.
  Fitting the nudge column fits the dead band.
- **`m dx dy`** at the confirm prompt records a caliper reading and moves
  nothing. First entry lands in the row as `open_loop_offset`. Sign convention
  is **pinned**: give `m` the same numbers you would type as a nudge (the
  correction). **Any fit on `open_loop_offset` must negate it.**
- **`nudge_steps`** records every nudge separately with `at_park`, so the dead
  band and the real error stay separable. `nudge_total` is a sum and a sum
  cannot show that the first correction did nothing.
- **`--force-grasp-yaw DEG`** pins the wrist so the jaw axis lies on a known
  world axis. Without it a caliper gap difference measures an unknown
  projection. **Calibration only** — with it on, the arm will not orient to the
  block, which is the flag working, not a bug.
- **Footprint warning** when the measured block is >4 mm off its 30 mm nominal.
  The block size is free ground truth on the whole vision chain and was being
  discarded every run.
- **`calibration.py --fit --zone pickup|place|both`**, defaulting to pickup.
  Place rows are **rank deficient alone** (the place zone never moves, radius
  constant at 229 mm) and pooled they dragged the survey fit's `kr` to
  −157 ± 32 mm/m, "measured" at 4.9 se and pure artifact.
- **Refusal** when a single-view detection lands outside the usable area. Both
  halves used to be warnings, and both fired on the same run: 1 view, 52.4 mm
  from the surveyed centre, 2.3× outside the 23.1 mm usable area. It was
  accepted, the arm drove there, and the operator hand-dragged the jaws 60 mm.
- **`report_reached` yaw bug**: it recomputed the tip-swing compensation without
  the yaw, misreporting by 1.2-2.5 mm at 40-90° yaws. The same bug was in
  `zone_calibrate.py`, the instrument that fitted `JAW_RADIAL_OFFSET_M` and
  `J1_RESIDUAL_BIAS_DEG`.

---

## 4. Mistakes made during this calibration, and what they teach

Recorded because each one nearly became a wrong constant.

1. **A sign inversion in the tangential line, shipped and caught at the bench.**
   Residuals were built from *corrections* and added to the offset as though a
   correction and an offset pushed the same way. They oppose. **H hid it** — its
   correction was already 0.0, so its required offset is identical on either
   sign convention and it stayed perfect right through the error. *A term that
   reproduces one pose exactly can still be inverted; only a pose with a
   non-zero residual tests the sign.*
2. **`m` re-parked the arm while printing "moves NOTHING".** `continue` in the
   confirm loop jumps to the top, which re-parks. Two "identical" readings at N
   were of poses 2.6 mm apart. Fixed with an inner prompt loop. *Verify the
   control flow of an instrument, not just its output.*
3. **"The tool frame is ruled out"** — asserted from one pose read by eye, and
   wrong. Two yaws and two bearings later it is the dominant term. *One pose
   cannot separate two frames that coincide there.*
4. **The truth column was the nominal inch grid all along.** Every position's
   `truth_world` is a round number of inches, so it recorded *where the mat was
   meant to go*. At H it claimed 8.3 mm of Y error while the caliper read zero.
   Position K read +20 mm tangential twice because its mat sat one full inch out
   in Y — **the survey was right and the tape was wrong.** *Truth must be
   measured, not asserted (Lesson 5), and the caliper is immune to it because it
   reads jaws against the physical block.*
5. **Repeatability quoted as accuracy.** "Survey error 0.3-0.6 mm" was the
   *repeat* of a fixed pose. Against tape the same survey is RMS 2.2-2.5 mm.
   *Internal agreement is not accuracy (Lesson 4).*
6. **A `--confirm` flag that does not exist** was put in a bench protocol.
   `--yes` is `dest="confirm", action="store_false"`; confirm is the default.
7. **Placeholders in a shell command.** `--truth-block-zone <zx> <zy>` was
   handed over literally and bash rejected it, and
   `--no-truth-block-on-centre` was attached to `tag_pick_place.py` when it is
   an `explore_pick_place.py` flag. See §6 for the commands that actually run.

---

## 5. Still open

1. **Q2, the in-zone sweep.** Untouched. Top-face parallax is 6.5 mm at the zone
   edge, corrected in code but **never tested off-centre since the correction
   landed**. This is half the original scope.
2. **The radial axis is not fitted.** At `--force-grasp-yaw 90` radial is
   perpendicular to the jaws so the gaps are blind to it; those readings were
   eyeballed. Latest caliper-grade radial values are ~0 at all three reaches, so
   it may need nothing — but that is not established.
3. **Pickup mat tag 1 is not being detected** (1 sighting in 17). Trust radius
   is tag-count dependent — `{4: 2.0, 3: 1.0}` half-diagonals, i.e. **144 mm
   with four tags, 72 mm with three**. With the pickup mat adjacent to the place
   mat the fine pass centred on the place zone, leaving pickup 111-124 mm off
   image centre, and all 11 of its sightings were rejected: `no pickup zone, so
   there is nothing to pick`. **Reprint or clean tag 1.**
4. **Two separate tag squares carry the place ids 4-7**, 6.4 mm apart. Check for
   a stray or doubled place mat; not a code artifact.
5. **The survey's tangential error is real and pose-organised**, ~2.3 mm beyond
   what isotropic taping error explains. Radial sd is 1.08/1.13 mm in two
   bearing groups while tangential is 3.56/2.60 — the noise stays in the
   tangential channel when the frames swap. Not yaw (dYaw sd 0.47°, worth
   0.18 mm). It does not currently limit the grasp, because the caliper measures
   the whole chain and reads ~0.6 mm RMS, but it will limit any fully autonomous
   no-nudge run.
6. **`J1_RESIDUAL_BIAS_DEG` 1.10 → 0.85.** Over-corrects by +0.25°, now
   confirmed at two radii (predicted 0.55 mm at r=126, observed 0.57 mm). Worth
   ~0.9 mm of tangential. Not applied: one change at a time, and it was not
   needed to reach target.
7. **The robot was physically replaced on 2026-08-11.** The dead band (1.88°)
   and sag were measured on the previous unit. The tangential line still lands
   at O afterwards, so it appears to transfer — but if a nudge ever
   under-delivers unexpectedly, re-measure the dead band first, not last.
8. **`choose()` view-set stability.** At O, 9 views vs 6 moved the origin
   3.66 mm. Cheapest fix is a warning on low view count, not a change to the
   gates — changing gates mid-calibration makes rows un-poolable.

---

## 6. Commands that actually run

Calibration measurement at one position, block on the zone centre:

```bash
python3 explore_pick_place.py --any-block --skip-pick --position N \
    --force-grasp-yaw 0 --note my_note
```

At the park: read the gaps, `m 0 4` to record, then `0 4` (no `m`) to move.
`--skip-pick` returns before any descent, so the block never moves and repeats
are free.

Survey only, no gripper motion at all:

```bash
python3 explore_pick_place.py --survey-only --position G --note my_note
```

In-zone (Q2) cell, driving `tag_pick_place` directly against a surveyed origin
so the survey term is held fixed. **`--no-truth-block-on-centre` is an
`explore_pick_place.py` flag and must NOT appear here.** Substitute real
numbers — no `<>` placeholders:

```bash
# from the survey above: pickup origin (+0.2059, -0.0315), zone yaw +88.8
python3 tag_pick_place.py --zone-origin 0.2059 -0.0315 0.050 --zone-yaw 88.8 \
    --skip-pick --truth-block-zone 0 -20 --note wed_q2
```

`--truth-block-zone` is **MILLIMETRES**; `--truth-block-world` is **METRES**.
Usable range is 23.1 mm and the check is on `max(|zx|, |zy|)`, so the ±20 mm
corners of a 3×3 factorial are legal.

Picking a real block at an arbitrary angle — **no `--force-grasp-yaw`**, or the
wrist will not orient to the block:

```bash
python3 explore_pick_place.py --any-block --note real_pick
```

Off-robot checks, all of which must print `0 failure(s)`:

```bash
python3 -m py_compile tag_pick_place.py pick_place.py zone_vision.py \
    zone_calibrate.py explore.py explore_pick_place.py calibration.py
python3 zone_vision_selftest.py
python3 explore.py --selftest
python3 calibration.py --selftest
python3 calibration.py --fit --channel survey --zone pickup
```

---

## 7. Reading the calibration history

`src/swarm_pkg/src/logs/calibration_history.jsonl`, one JSON object per run.

- `open_loop_offset` — the caliper reading before any nudge. **The only
  admissible error measurement.** Carries the sign of the *correction*; negate
  it for a fit.
- `nudge` / `nudge_steps` — what the operator did. Contaminated by the dead
  band. Not an error measurement.
- `constants` — full provenance including `JAW_TANGENTIAL_PER_M_REACH` and
  `JAW_PERP_OFFSET_M`. **A row missing these cannot be compared with one taken
  at a different reach or grasp yaw.**
- `invalid` — a string reason. `calibration.load()` drops these loudly with line
  numbers. Rows marked so far: two stale-truth rows (176 mm, 319 mm), two K rows
  from a one-inch mat misplacement, and one single-view detection 52 mm outside
  the usable area.
- `truth_world` — **the nominal inch grid unless independently measured.** Do
  not fit against it.
