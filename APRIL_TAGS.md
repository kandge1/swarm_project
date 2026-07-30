# AprilTag-guided pick and place

Branch: `feature/april_tags`. Started 2026-07-28.

Companion to `PROJECT_CONTEXT.md` (what the system is), `WORKFLOW.md` (how to run it) and
`TESTS.md` (what the hardware actually does). This file is the goal document for the
AprilTag feature: the stages, the decisions behind them, and the measured numbers as they
come in.

Same convention as the other three: record **why**, and record **measured numbers rather
than impressions**. A constant with no measurement behind it should say so out loud.

---

## The goal

Today `pick_place.py` runs a fixed sequence against hardcoded `PICK_XYZ` / `PLACE_XYZ`, at
one fixed grasp orientation (`GRIPPER_YAW_DEG = -45.0`). Every block has to be set down by
hand at the exact spot the constants describe, rotated to match the jaws.

The goal is to remove that. A block placed **anywhere, at any rotation** inside a ~4in x 4in
"pickup zone" should be located, identified, approached at the correct yaw, grasped by a
face the gripper can actually span, and placed in a "place zone".

Zones are marked by four 1in AprilTags (36h11) whose **centres** sit on the vertices of a
**6in** square — bigger than the ~4in working area itself. See
[Usable area](#usable-area-the-tag-square-is-not-the-working-square) for why: a tag centred
on a 4in square's own vertices would eat into the corners of the working area it's meant to
mark. Decided 2026-07-29.

---

## Status

| Stage | What it adds | Status |
|---|---|---|
| — | Mat layout: 6in tag square around a ~4in working area | **Decided 2026-07-29** |
| 0a | Characterization: dead-zone floor + repeatability scatter | **DONE 2026-07-29** — Test 1 passed (GO); Test 6 now largely redundant, repeatability fell out of Test 1 |
| 0b | Calibration: tag lib, FOV, jaw mm/rad, camera framing offset | Camera offset **done** (from URDF); the other three need the robot |
| — | Grasp tilt, encoder side: 4.19°/5.05° → **0.50°/0.19°** | **DONE 2026-07-29**, verified |
| — | Grasp tilt, **actual block**: still 5–10° | **ROOT CAUSE FOUND 2026-07-30** — mechanical play downstream of the encoders; needs re-fit against measured tilt, or vision |
| 1 | Pickup zone hardcoded, square blocks, random position + rotation | **Code complete, unverified on hardware** |
| 1b | Multi-view tag fusion (2-of-4 tag occlusion) | **DONE 2026-07-30**, synthetic tests pass |
| 2 | Place zone also tag-located | Not started |
| 3 | 24-block database: shape/size ID, per-block grasp orientation | Not started |
| 4 | Place anywhere in the place zone (beside / on top) | Not started |

**Nothing in Stage 1's code has run on the robot yet.** Everything below the synthetic tests
is unverified against real optics, real lighting and real arm behaviour. The two correction
thresholds in `tag_pick_place.py` are explicitly marked placeholders in the source until
Test 1's results are in.

Measured constants land in [Measurements](#measurements) below as each is taken. Until a
row there has a number, treat the corresponding code constant as a guess.

---

## The central design decision

**The four tags give a homography from image pixels to the zone plane in millimetres.**

Because the tag centres are at known world coordinates, block position comes out of that
homography directly in world coordinates. The camera pose, the camera intrinsics, and the
accuracy of the hover move all **cancel out** — they affect *framing* (whether the tags are
in shot) but not the answer.

This matters because of a constraint documented at length in `PROJECT_CONTEXT.md`: the
arm's absolute positioning is not trustworthy. `IK_POS_TOLERANCE = 0.02`, the residual
grasp tilt was **wrongly believed** mechanical (see [the tilt](#the-grasp-tilt--encoder-side-solved-root-cause-is-mechanical-play-2026-07-30) — the flange-side error is fixed, but mechanical play past the encoders remains and is invisible to `/joint_states`), and `_send_goal_and_wait`
infers success from `/joint_states` because `arm_group_controller` has no `constraints:`
block to report tracking failure itself. Any design that computes block position from
*where the arm thinks it is* inherits every bit of that error.

So the tags are used from Stage 1 onward, even though Stage 1's zone position is hardcoded.
The hardcoded pose only aims the hover; it never enters the position calculation.

The second constraint is `PROJECT_CONTEXT.md`'s networking finding: the DDS link silently
drops anything over ~1400 bytes (the MTU fragmentation root cause, Session 3c). A camera
stream cross-machine is not merely slow, it is impossible. **All vision runs on the Pi.**

---

## Usable area: the tag square is not the working square

Found 2026-07-28 while building `zone_vision_selftest.py`, before any hardware time.

A tag centred **on** a vertex reaches `tag_size/2` **inward** from that vertex. A block
whose corner gets that far out is sitting on top of the tag and occluding it. So the area a
block can actually occupy is not the tag square — it is the tag square shrunk by half a tag
on every side, and then by half the block:

```
usable_half_extent = zone_size/2 - tag_size/2 - block_size/2
```

For the current numbers — 4in tag square, 1in tags, 1.18in block:

```
50.8 mm - 12.7 mm - 15.0 mm = 23.1 mm  =>  a 46.2 mm (1.82 in) usable square
```

That is less than half the 4in the zone appears to offer, and it gets worse for the larger
Stage 3 blocks: a 3in block does not fit at all (`50.8 - 12.7 - 38.1 < 0`).

This was not a code bug. The first synthetic sweep placed blocks at ±30 mm — physically
impossible positions where the block covers a tag — and the resulting truncated contours
showed up as 26–37 mm measurements for a 30 mm block with `fill_ratio` collapsing to ~0.11.
Chasing that as a segmentation problem would have been wasted effort.

**The fix is a mat layout change, not a code change.** `zone_size` is already a parameter,
and the tag square and the working area do not have to be the same square. Putting the tag
centres on a **6in** square around a 4in working area gives:

```
76.2 mm - 12.7 mm - 15.0 mm = 48.5 mm  =>  a 97 mm (3.8 in) usable square
```

i.e. essentially the whole intended 4in working area, at the cost of a slightly larger
printed mat and a slightly wider field of view (see the FOV go/no-go in Stage 0b).

**Decided 2026-07-29: 6in.** `zone_vision.DEFAULT_ZONE_SIZE = 0.1524` (was 0.1016). Print the
tag square at 6in on a side; the ~4in "working area" the user places blocks in is now a
nominal region well inside it, not the tag square itself. The ±23 mm limit above no longer
applies — the new usable half-extent is ±48.5 mm, i.e. essentially the whole 4in area.

Related: an occluded tag is survivable. The homography is fitted from all four corners of
every visible tag, so three tags give 12 correspondences and still over-determine it. Two
is refused — the points span too thin a band to condition the fit.

---

## Mat geometry: where the tags physically go

Origin is the **bottom of the robot base** — the URDF has `world -> g_base` at `xyz="0 0 0"`,
so world origin and the base's mounting plane are the same point. +X and +Y are world axes;
the numbers below assume `zone_yaw = 0`, i.e. the mat squared to the robot.

Zone centres come from `pick_place.py`'s hand-tuned `PICK_XYZ` / `PLACE_XYZ`:

| zone | centre (mm from base origin) |
|---|---|
| pickup | `(0, +250)` |
| place | `(0, −250)` |

Tag **centres** on the corners of a 152.4 mm (6.00 in) square:

| zone | tag | zone-local (mm) | world from base (mm) | radius |
|---|---|---|---|---|
| pickup | 0 | (−76.2, −76.2) | **(−76.2, +173.8)** | 189.8 |
| pickup | 1 | (+76.2, −76.2) | **(+76.2, +173.8)** | 189.8 |
| pickup | 2 | (+76.2, +76.2) | **(+76.2, +326.2)** | 335.0 |
| pickup | 3 | (−76.2, +76.2) | **(−76.2, +326.2)** | 335.0 |
| place | 4 | (−76.2, −76.2) | **(−76.2, −326.2)** | 335.0 |
| place | 5 | (+76.2, −76.2) | **(+76.2, −326.2)** | 335.0 |
| place | 6 | (+76.2, +76.2) | **(+76.2, −173.8)** | 189.8 |
| place | 7 | (−76.2, +76.2) | **(−76.2, −173.8)** | 189.8 |

Ordering is `ZONE_CORNER_SIGNS`, counter-clockwise from the −X−Y corner. Note it is applied
per-zone, so tag 4 sits at the place zone's −X−Y corner, which is its FAR side from the robot.

Two things to notice before printing:

- **The far tags sit at 335 mm radius**, well outside the 249 mm the arm actually works at.
  They only need to be *seen*, not reached, so this is fine — but it is the number that decides
  the FOV go/no-go (Stage 0b.2), and it grew when the tag square went 4in → 6in.
- **Placement accuracy is yours, not the printer's.** The zone size is set by where you put the
  tag centres with a ruler. `zone_size` is a parameter; if you end up with 150 mm instead of
  152.4, measure it and pass the real number rather than forcing the placement.

### Printing: `print_tag_sheet.py`

`print_zone_tags.py` renders each zone as a finished mat, which is only useful if the printer
honours "Actual Size" — and a mat printed at 96% is silently wrong in the worst possible way,
because the tags still decode perfectly and only the GEOMETRY is off. Nothing downstream can
detect that.

`print_tag_sheet.py` puts all 8 tags on one A4 as individual cut-outs and removes the
dependency on print scale entirely:

```
python3 print_tag_sheet.py --out print_sheets/all_tags_A4.png
```

- tags cut out and positioned **by hand with a ruler**, so zone size is a measurement, not a
  print artifact
- whatever scale the printer applied, **measure a tag and pass `--tag-size`**. A uniformly
  scaled tag is not a defect, it is a different `tag_size` — which is already a parameter.
- 4 mm quiet zone around each tag (the detector needs it; a tag cut flush to its black edge
  gets much harder to find against a dark mat)
- centre cross-hairs to measure to, since it is the tag **centre** that goes on the corner
- an up-arrow on every tag: all 8 must share one "up" = zone +Y, which
  `TAG_CORNER_OFFSETS` assumes
- a 150 mm ruler to check what actually came out

Verified by round-trip: the rendered sheet was fed back through `zone_vision._aruco_detect`
and all 8 ids decode, each measuring 25.39 mm against a 25.40 nominal.

**Getting an id onto the wrong corner is the one mistake the residual will not catch** — a
mirrored or rotated zone frame is still a perfect fit. Check it against a block at a known
corner.

---

## Architecture

```
      mars (Jazzy)                              robot Pi (Galactic)
  ┌──────────────────────┐                 ┌──────────────────────────────┐
  │ tag_pick_place.py    │  DetectBlock    │ block_detector_node.py        │
  │  (orchestrator)      │ ──────────────► │   holds /wrist_camera sub     │
  │  imports pick_place  │ ◄────────────── │   grabs ONE fresh frame       │
  │                      │   ~200 B reply  │   calls zone_vision.py        │
  │ move_group (IK/OMPL) │                 │ v4l2_camera_node              │
  └──────────────────────┘                 │ ros2_control + mycobot_bridge │
                                           └──────────────────────────────┘
```

Images never cross the network. The service reply is a handful of floats, comfortably under
the MTU cap. Nothing about the split-compute launch topology from `WORKFLOW.md` changes.

Stills are taken only when the arm is parked at a hover — never during motion. That is both
a CPU concession (the Pi is also running the 100 Hz `ros2_control` loop and the serial
bridge) and a correctness one (`PROJECT_CONTEXT.md` Session 5: the serial link is
half-duplex and reads fail while the arm moves).

### Files

| Path | Runs on | Role |
|---|---|---|
| `src/swarm_interfaces/` | both | `srv/DetectBlock.srv`. Must be built on both machines from identical source. |
| `src/swarm_pkg/src/scripts/zone_vision.py` | Pi + mars | Pure OpenCV, **no ROS**. Tag detect → homography → block segmentation → footprint + yaw. The whole algorithm lives here so it can run against saved stills. |
| `src/swarm_pkg/src/scripts/block_detector_node.py` | Pi | Thin ROS wrapper: service server, one-frame grab, calls `zone_vision`, returns world coords. |
| `src/swarm_pkg/src/scripts/zone_view.py` | either | Developer viewer. `--show` opens a window (mars), `--write out.png` saves annotated frames (Pi, headless). Not a compute offload — see below. |
| `src/swarm_pkg/src/scripts/blocks.yaml` | Pi | Stage 3 block database. |
| `src/swarm_pkg/src/scripts/tag_pick_place.py` | mars | Orchestrator. Imports from `pick_place.py`, the same pattern `reset_arm.py:29` and `annulus_test.py:55` already use. |

**On `zone_view.py` not being an offload:** the Pi does all the real work, including the
homography. `cv2.findHomography` on four point pairs is an 8x8 linear solve — microseconds.
The per-frame cost is tag detection and contour finding on one 640x480 frame, tens of
milliseconds. `zone_view.py` exists so a *developer* can try a threshold against 20 saved
frames in seconds instead of re-running the robot each time, and so you can look at an
overlay. It runs the same `zone_vision.py` the Pi runs in production. If the Pi ever needs
longer, hovering longer is free.

---

## No control system in this feature

The correction loop is **measure, correct once, verify**. No gains, no integrator, no state
estimator, no model of the arm. Every measurement is absolute (from the tags), so errors do
not accumulate and there is nothing to stabilise. The PID / disturbance-observer work stays
on its own branch; this feature is what gives it a measurement to observe.

But two numbers from `TESTS.md` are load-bearing, and without them the loop's stopping
conditions are guesswork:

- **Test 1, dead-zone / correctability.** Answers whether commanding a *different* value
  moves the joint at all. If the arm is not correctable by biasing, detect→correct→re-detect
  cannot converge and Stage 1's design has to change. A genuine precondition. It also gives
  the **minimum correction worth commanding**.
- **Test 6, repeatability.** Gives the **scatter floor**, which is exactly the "close
  enough, stop correcting" tolerance. A threshold set below the arm's own scatter produces a
  loop that never terminates.

Tests 3, 4 and read-failure-rate are skipped. They exist to support phase-margin math for a
continuous controller; a discrete correct-and-verify step does not need them.

**One thing worth noting:** `TESTS.md` says that if the residual turns out unpredictable run
to run (stiction, backlash), the fallback is "dither, dead-zone inversion, or **external
metrology**". The AprilTag zone *is* external metrology. Vision measures the
gripper-to-block error directly in world space, so gravity droop, joint dead-zone and
kinematic error do not have to be modelled — only out-measured. Even a bad Test 1 result is
survivable here in a way it would not be for a joint-space controller.

---

## Test 1 results (2026-07-29) — GO, with one caveat

108 trials, 6 decorrelated postures, joints 0/1/2, both directions, 3 repeats, ±35 deg.
Raw data: `src/swarm_pkg/testing/test1_full.csv`.

**The decorrelation worked.** `corr(gravity_arm, inertia_lever)` came out **+0.03 and +0.00**,
against +0.6 to +1.0 for the old three-posture set. The two regressors are finally separable,
which was the entire reason for rebuilding the posture set.

**The residual is not gravity droop.** Joint 0's gravity moment arm is 0.0000 in every
posture — gravity cannot load it — and its residual is **flat at 0.98 deg across a 4.4×
span of inertia lever** (fit slope −0.13 deg/m, total span 0.13 deg). A constant offset with
the gravity term provably absent. It is also *directional*: J0 undershoots by +0.93 deg going
positive and −1.10 deg going negative — undershoot in the direction of travel, both ways.
That is Coulomb friction, and a feedforward of `sign(Δθ) × 1.0 deg` would cancel most of it.
Handing that to the controller branch.

J1 and J2 do show gravity terms, but with *opposite* signs (corr −0.79 and +0.85) and J1
overshoots where J0/J2 undershoot. Not over-reading that yet: `|residual|` conflates over-
and undershoot, and each per-joint fit has only 6 points.

**Correctability — the k=1 column:**

| joint | median k=1 ratio | stuck at k=1 | at k=2 | at k=3 |
|---|---|---|---|---|
| **0** | **0.07** | 3/21 (14%) | 0/21 | 0/21 |
| 1 | 1.00 | 10/19 (53%) | 2/19 (11%) | 1/18 (6%) |
| 2 | 1.00 | 7/12 (58%) | 5/12 (42%) | 3/12 (25%) |

J0 is excellent — biasing by `e` cuts the error to 7% of itself. J1/J2 ignore a 1× bias about
half the time but respond at 2×. Only 4 of 52 staircases never moved at all. A ratio of
exactly 1.00 means the joint settled at the **bit-identical** encoder value: the bias did
nothing. That is the dead band, and it is what sets `CORRECTION_DEADZONE_M`.

**Repeatable but inaccurate.** Same-direction repeatability is 0.045 deg — *below* the
0.088 deg readback quantum, with 36% of cells returning bit-identical residuals across all
three repeats. Those two facts are not in tension: the arm lands in the same place every
time, and that place is the wrong one by ~1 deg. It also means Test 6 is now largely
redundant — this run already measured repeatability as a by-product.

**The caveat: J0 backlash, 1.83 deg = 8.0 mm at r = 0.25 m.** Larger than the dead zone
itself, and J0 is the joint that swings the gripper laterally across the zone. Any correction
that reverses direction spends its first 8 mm taking up slack and does nothing visible.

### What this changed

`CORRECTION_CONVERGED_M` 3 mm → **2 mm**, `CORRECTION_DEADZONE_M` 1 mm → **5 mm**. Both
placeholders were wrong in the same direction: 3 mm sits *below* the dead band, so the loop
would have commanded corrections the arm physically cannot execute, burned both retries and
aborted — precisely the failure Test 1 exists to predict.

### Open, from this

- **Unidirectional final approach** to defeat J0 backlash. Deliberately not implemented yet:
  it changes how every hover is planned and deserves a measured before/after on real frames.
- **Is 5 mm good enough to grasp?** Vision knows the block to well under a millimetre; the
  arm gets there to ~5 mm. Whether that grasps depends on jaw clearance around a 30 mm block,
  which is Stage 0b.3 and still unmeasured. **That measurement is now the deciding one for
  Stage 1.**

---

## The grasp tilt — encoder-side solved; ROOT CAUSE is mechanical play (2026-07-30)

> **Status: half done, and the half that is done is not the half that was complained about.**
> The FLANGE is now vertical to 0.2-0.5°, verified. But the held BLOCK is still visibly
> tilted, and everything below the last encoder is invisible to `/joint_states` — so none of
> the joint-space work could ever have addressed it. See
> [the block tilt](#the-block-tilt-what-joint_states-cannot-see) for where that stands.
>
> | | grasp | place | jaw offset | block top face |
> |---|---|---|---|---|
> | original | 4.19° | 5.05° | 4.1 / 4.9 mm | 2.2 / 2.6 mm |
> | radial-only pre-comp | 2.02° | 2.32° | 2.0 / 2.3 mm | 1.1 / 1.2 mm |
> | **+ tangential + payload** | **0.50°** | **0.19°** | **0.49 / 0.18 mm** | **0.26 / 0.10 mm** |
>
> **8.3× and 27× better than where it started, and 2–5× inside the ~1 mm target.**
>
> The end goal is an assembly system building structures out of blocks, so the standard is ~1 mm
> and ~1°, not "looks straight". At the 0.056 m flange-to-jaw lever, 1° of residual tilt = 1.0 mm
> of jaw offset, and a 30 mm block tilted 1° has its top face 0.5 mm out of level — which
> compounds per course when stacking. That is why this was worth the effort.
>
> Route to get here, worth reading before revisiting: the cause is four pitch joints
> undershooting ([Evidence](#evidence)); the obvious fix of nudging them afterwards is
> *provably impossible* ([Fix 2](#fix-2--the-biased-settle-re-send-tried-failed-disabled)) and
> dangerous — it drove the arm into the table; what works is pre-compensating in task space
> during a trajectory, on **both** horizontal axes, with a payload term
> ([Fix 3](#fix-3--sag-pre-compensation-in-task-space-current-approach)).

The long-standing "the gripper is always visibly tilted when it grasps" complaint.
Previously concluded to be **mechanical and unfixable** — sag under the camera+gripper mass,
or a URDF/mount mismatch — on the reasoning that it was visible even at all-joints-zero and
that tightening `IK_ORI_XY_TOLERANCE` (0.10 → 0.04) changed nothing. **That conclusion was
wrong**, and Test 1's data plus a URDF FK check is what overturned it.

### Evidence

A grasp was stopped mid-run and `/joint_states` captured. Feeding those exact values through
the URDF forward kinematics:

- flange Z axis = `(+0.0002, −0.0646, −0.9979)` → **3.70° off straight down**, tilted almost
  purely toward −Y (leaning back toward the base)
- flange position 10.2 mm **below** the commanded z

An exact straight-down solution *does* exist at that position — solved numerically to a
residual of 1.7×10⁻¹¹. So it is neither a singularity nor kinematically forced. Comparing
that exact solution against what the arm actually did:

| joint | commanded | achieved | error | tilts? |
|---|---|---|---|---|
| joint2_to_joint1 | +104.743° | +103.790° | −0.953° | no (vertical axis) |
| joint3_to_joint2 | −42.486° | −44.120° | **−1.634°** | **yes, 1:1** |
| joint4_to_joint3 | −59.157° | −59.670° | **−0.513°** | **yes, 1:1** |
| joint5_to_joint4 | +11.643° | +10.190° | **−1.453°** | **yes, 1:1** |
| joint6_to_joint5 | −0.000° | +0.870° | **+0.870°** | **yes, 1:1** |
| joint6output_to_joint6 | +149.743° | +147.300° | −2.443° | no (vertical axis) |

RMS joint error **1.45°** — exactly the ~1° per-joint residual Test 1 measured.

`joint3_to_joint2`, `joint4_to_joint3`, `joint5_to_joint4` and `joint6_to_joint5` all have
**horizontal** axes under the downward grasp, so each contributes **exactly 1.000° of tilt
per 1° of joint error** (verified by perturbation). The other two are vertical and contribute
**exactly 0.000°**. So the four pitch errors *add*, and:

```
tilt from ONLY the four pitch errors : 3.704 deg
tilt from ONLY the vertical-axis errors: 0.000 deg
tilt actually observed               : 3.704 deg
```

Matching to three decimals. **The tilt is four undershooting joints stacking up.** Not
mechanical, not the URDF, not an IK tolerance.

This also explains why the earlier investigation went astray: tightening
`IK_ORI_XY_TOLERANCE` correctly changed nothing, because the IK solution was never the
problem — the solver asked for straight down and the servos didn't deliver it. And the error
is consistent run to run precisely because Test 1 measured repeatability at 0.045°: a
deterministic undershoot produces a deterministic tilt, which is exactly why it "never goes
away and is always the same angle."

### Fix 1 — the rounded quaternion (kept)

**`GRASP_QX/QY` were rounded to 4 decimals.** `0.7071 ≠ 1/√2`, and the resulting quaternion
asked the flange for a pose **0.5019° off vertical**. Half a degree of the tilt was baked into
the target before any solver or servo was involved. Now `±math.sqrt(0.5)` exactly → target
tilt 0.0000°, confirmed on hardware (`commanded tilt 0.001°`). Free, and it stays.

### Fix 2 — the biased settle re-send: TRIED, FAILED, DISABLED

Two hardware runs. The idea was to re-send `command + gain·residual` instead of the identical
command. It does not work, and the second run was **physically dangerous** — the arm struck
the table. Both outcomes fall out of one line of algebra that should have been written down
*before* burning robot time on it:

> A joint commanded to `c` settles at `c − d` for a deterministic undershoot `d`. Biasing
> commands `c + g·d`, which settles at `c + (g−1)·d`. Settle measures the residual against the
> original `c`, so **`residual_next = (1 − g)·residual`**, which converges only for `0 < g < 2`.

| gain | behaviour |
|---|---|
| `g = 1` | residual → 0. **Deadbeat. 1.0 was the exactly-correct gain, not a weak one.** |
| `g = 2` | residual → `−d`. Marginal: sustained oscillation, no decay. |
| `g > 2` | divergent. |

**Run 1, `g = 1.0`** — the deadbeat value — and the arm did not move a single count. Bias
`[+1.03, +1.89, +0.51, +1.89, +0.79, +3.06]°`, final residual
`[+1.03, +1.90, +0.51, +1.89, +0.79, +3.06]°`. Identical. Gain 1.0 asks for a delta of `d`
(~1–2°), which is **below the servo dead band**, so the joint ignores it. Tilt 4.442 → 4.372°.

**Run 2, `g = 2.0` plus a 1.5× escalation multiplier** = effective gain 3.0, past the boundary
and into divergence. The log shows it exactly — `joint3_to_joint2` went `+3.79, −4.01, +4.01,
−4.01, +4.01 …` for **14 attempts**, sign-flipping every time, pinned at the 4° cap, residual
*growing* `0.0362 → 0.0374 → 0.0729 → 0.0581 → 0.0791 → 0.0853`. A 4° pitch swing at 0.28 m
reach is ~20 mm of vertical travel: that is what hit the table on the place descent. Tilt
4.372 → 4.189°, i.e. nothing.

**The two failures together close the door.** Breaking the dead band needs `g` well above 1;
staying stable needs `g` below 2; and the dead band for these joints is *larger* than the `d`
being corrected. No gain satisfies both. A stationary joint cannot be nudged by less than its
own dead band — exactly the case `TESTS.md:63` anticipated ("dither, dead-zone inversion, or
external metrology"). `SETTLE_BIAS_ENABLED = False`; do not re-enable without new evidence
that the dead band itself has changed.

Two incidental bugs the same logs exposed, both fixed: the bias was computed and logged during
*gripper-only* settles where the arm was never re-sent (misleading log, no motion — it cost
real debugging time), and `loop_rate` collapses to **1.1–1.8 Hz** during settles with
`get_angles() returned -1` alongside, so the loop was biasing against stale readings.

### Fix 3 — sag pre-compensation in task space (current approach)

The dead band only arms when a joint is **stationary**. During a trajectory the joints are
already moving, so instead of correcting after the fact, aim past: command a grasp orientation
tilted *outward* by the sag, and let the sag bring it to vertical. `SAG_PRECOMP_DEG = 4.6` in
`pick_place.py`, applied via `sag_precomp_quat()`.

What makes one scalar the right model rather than six joint offsets — four measured poses:

| pose | tilt | lean · radial | lean · tangential | reach |
|---|---|---|---|---|
| grasp, run 1 | 4.372° | −0.903 | −0.429 | 0.248 m |
| grasp, run 2 | 4.189° | −0.909 | −0.416 | 0.249 m |
| place, run 1 | 5.087° | −0.952 | −0.305 | 0.249 m |
| place, run 2 | 5.049° | −0.921 | −0.389 | 0.249 m |

The lean is 0.90–0.95 radial and **negative (inward, toward the base) in every case**. Grasp
and place sit ~180° apart in base yaw (+104.7° vs −74.1°), so in the *world* frame these tilts
point in opposite directions — but in the arm's own radial frame they are the same direction
and nearly the same size. That is the signature of a pose-frame-constant gravity sag, and it
is why a single scalar, rotated into place per target, is the correct model.

Applied to `make_grasp_pose`, `move_arm_to` and `cartesian_move_to`'s path constraint —
**all three**, deliberately. Hover uses one and the descent the other, and if their
orientations disagree the "straight down" Cartesian descent has to rotate the wrist while
translating, which is the sideways nudge that descent exists to avoid.

Verified before hardware: commanded tilt exactly 4.6000° with `lean · radial = +1.0000`
(purely outward) at both poses; quaternion stays unit; identity when `SAG_PRECOMP_DEG = 0`.
IK reaches both pre-compensated targets with residual ~1e-11, all joints inside limits.

**Predicted: 4.189° → 0.41° at the grasp pose, 5.049° → 0.45° at the place pose.**

#### Hardware result: radial worked, and exposed the other half of the problem

Three runs (run 2 aborted early, unrelated — relaunching both launch files cleared it).
Measured tilt **4.19° → 2.02°/2.25°** at the grasp and **5.05° → 2.32°/2.20°** at the place.
Real, repeatable, and not enough. Decomposing into the two horizontal components explains why:

| pose | tilt | radial | tangential |
|---|---|---|---|
| grasp, before | 4.19° | −3.81 | −1.74 |
| place, before | 5.05° | −4.65 | −1.96 |
| grasp, run 1 | 2.02° | **+1.54** | −1.30 |
| grasp, run 3 | 2.25° | **+1.62** | −1.56 |
| place, run 1 | 2.32° | **−0.57** | −2.25 |
| place, run 3 | 2.20° | **−0.51** | −2.14 |

1. **Radial went from −3.8/−4.7 to roughly zero.** The mechanism is sound: pre-compensation
   during a trajectory does defeat the dead band, exactly as the Fix-2 algebra predicted it
   would.
2. **Tangential was never corrected and is now the dominant residual** (−1.7 → −1.3, −2.0 →
   −2.2). It is negative in all eight measurements, so it is every bit as systematic as radial
   was. Correcting one axis of a two-axis error just leaves the other one standing.
3. **This is why place looks worse than pick, and why it reads as the tilt "amplifying" through
   the sequence.** It does not amplify. Place has the larger tangential term (−2.2 vs −1.3) and
   always did — compare the two "before" rows, where the same gap is already present.

The radial residuals also disagree in a physically meaningful way rather than randomly: grasp
**overshot** (+1.54, wants less) while place **undershot** (−0.57, wants more), and the place
descent is the one **holding a block**. More end mass → more sag. So the model gains a payload
term rather than two unrelated per-pose constants.

Fitted for zero residual using the measured radial response (1.16× at grasp, 0.89× at place —
a commanded degree does not land as a degree):

```
SAG_PRECOMP_RADIAL_DEG             = 3.27   # empty
SAG_PRECOMP_TANGENTIAL_DEG         = 1.30   # empty
SAG_PRECOMP_PAYLOAD_RADIAL_DEG     = 1.97   # added while holding
SAG_PRECOMP_PAYLOAD_TANGENTIAL_DEG = 0.95   # added while holding
```

`holding_block` is threaded through `make_grasp_pose`, `move_arm_to` and `cartesian_move_to`,
set True from the grasp close until the release. Verified in isolation: the composed quaternion
reproduces the requested radial/tangential lean to 0.006°, stays unit-norm, and returns
identity both when disabled and on the base axis.

#### Unrelated bug the same session surfaced: the aborting run was not a fluke

One of the three runs aborted at "Move to pre-grasp" with
`joint6output_to_joint6: -0.0712 rad` against `ARM_SETTLE_TOLERANCE = 0.07` — **failing by
0.0012 rad = 0.07°**, with every other joint inside 0.022. Re-running worked, which is exactly
what makes it look random. It is not: `joint6output_to_joint6` carries the largest dead-band
residual of the six (+3.06° in earlier logs, 4.08° here), so it sits on the threshold and will
keep landing either side of it. Test 1's `extended` posture failed the same way, by 0.07°
against a 3.0° gate.

Raising the global tolerance would be wrong — 0.07 rad is already ~1 cm at the fingertips, and
the pitch joints are precisely where that error becomes the grasp tilt. But `joint6output` is
**vertical-axis**, contributing exactly 0.000° of tilt; its error is gripper *yaw*, not lean. So
it earns a looser gate on a real distinction rather than a convenient one:
`ARM_SETTLE_TOLERANCE_PER_JOINT = {"joint6output_to_joint6": 0.09}`, applied only where it
loosens (so the gripper's explicit 0.05 is never silently widened). The timeout message now
names the override, because reporting a flat "tolerance 0.07" while a joint is gated at 0.09 is
what made this read as a fluke in the first place.

#### Hardware result: solved

Measured joint states, full clean run, no `bias[...]` lines (bridge correction stays off):

```
grasp: [103.71, -42.27, -63.98, 16.61, -0.35, 147.30]
place: [ -74.09, -41.92, -58.62, 10.37, +0.08, -28.74]
```

| pose | tilt | radial | tangential | jaw offset |
|---|---|---|---|---|
| grasp | **0.502°** | +0.438 | −0.245 | 0.49 mm |
| place | **0.188°** | −0.185 | +0.034 | 0.18 mm |

Both axes collapsed together, which is the confirmation that the two-axis model was the right
shape: radial went +1.54 → +0.44 at the grasp and −0.57 → −0.19 at the place, tangential −1.30 →
−0.25 and −2.25 → +0.03. The payload term also proved out — place, the loaded descent, is now
the *better* of the two poses, having been the worse one at every earlier stage.

**Deliberately not tuned further.** The residuals (0.44° radial at the grasp being the largest)
are only ~3× the run-to-run scatter measured earlier (±0.11° tilt, ±0.04° radial, ±0.13°
tangential across the two radial-only runs), and they come from a single run. Fitting four
constants tighter against n=1 at that signal-to-noise is how you make it worse. The target is
met with margin; stop here.

If a future pass does want the last fraction of a degree, the direction is:
`SAG_PRECOMP_RADIAL_DEG` 3.27 → ~2.9, `SAG_PRECOMP_TANGENTIAL_DEG` 1.30 → ~1.55,
`SAG_PRECOMP_PAYLOAD_RADIAL_DEG` 1.97 → ~2.6 — and it needs 3+ runs per pose first.

### ROOT CAUSE: mechanical play downstream of the encoders (2026-07-30)

**The encoders cannot see the error.** Established by direct experiment, and it supersedes the
speculation in the section below.

Test: with the arm at the all-zeros home pose, read `/joint_states` while (1) letting it sag
under its own weight, (2) physically holding it upright with all joint notches aligned, and
(3) releasing it again. Photographs show J2 (`joint3_to_joint2`) visibly misaligned by several
degrees when sagging. What the encoders reported:

| transition | flange position | flange rotation |
|---|---|---|
| sagging → held upright | **4.49 mm** | **1.49°** |
| upright → released | 3.80 mm | 1.12° |

Per-joint, straightening the whole arm by hand moved the encoders a **total of 1.39° across all
four pitch joints**, J2 itself by only **0.51°** — far less than the photo shows. The encoder is
on the motor side of the joint; the deflection is in the gearing/coupling between it and the
link, so the encoder is structurally blind to it.

Not a resolution problem: every reported value is an exact multiple of the 0.0879° readback
quantum. The sensor is fine. It is measuring the wrong side of the slop.

**Consequences, which retire several earlier lines of work:**

- FK-from-`/joint_states` tilt (0.19–0.50° after the sag fix) and the visibly tilted held block
  (5–10°) are *both correct*. They measure opposite sides of the play.
- Every joint-space correction — the settle bias, `SAG_PRECOMP_*`, tightening
  `IK_ORI_XY_TOLERANCE` — operates on a signal that cannot observe the dominant error. This is
  why the tilt survived all of them, and why the original complaint that it "never goes away"
  was accurate all along.
- `PICK_XYZ` / `PLACE_XYZ` were tuned by hand against the real robot, so the deflection is
  already absorbed into the **position** constants. Nobody ever did the equivalent for
  **orientation**. That asymmetry is the entire remaining gap.

**What still works.** The deflection is repeatable — reading 3 returns to within 0.35°/joint of
reading 1, consistent with Test 1's 0.045° repeatability. A repeatable error is calibratable.
So `SAG_PRECOMP_*` is the right mechanism (pose-dependent, payload-aware); it was simply fitted
against the FK number instead of against reality. **Re-fit it against measured physical block
tilt** — level app on the block's top face at the pick pose and at the place pose, holding.

**Check J2 mechanically first.** One joint with several degrees of play while the others have
"a tiny bit" suggests a loose grub screw or worn gear rather than design compliance. Calibrating
around a loose fastener bakes in a number that moves the next time it shifts.

**And the reason the AprilTag work matters more than it looked.** The wrist camera is mounted on
the **link** side, downstream of the play, so it observes the *true* orientation. It is the only
sensor on this robot that can see this error at all. Vision-based correction is therefore not a
convenience for arbitrary poses — it is the only route to millimetre accuracy on hardware with
this much slop.

---

### The block tilt: what /joint_states cannot see

Found 2026-07-29, immediately after the sag fix was confirmed. **The flange is vertical to
0.2–0.5° and the held block is still visibly tilted, worse at the place pose.** Both
measurements are right; they measure different things, and the gap between them is everything
downstream of the last encoder — which no joint-space method can observe, let alone correct.

**A theory that was wrong, recorded so nobody re-derives it.** The gripper is angular: every
finger joint is revolute about one axis, and that axis is *horizontal* in world at both poses
(`[-0.025, +0.9997, -0.001]` at the grasp, horizontal component 1.0000). So pad swing during
closure would tilt the block degree for degree — 21.1° of swing between the logged open
(`0.135`) and contact (`-0.2325`). Compelling, and false. The mimic tags make it a
**parallelogram linkage**:

```
gripper_left3_to_gripper_left1   mimic gripper_controller x-1.0
```

`gripper_left3` turns +θ, the pad turns −θ, netting zero. Confirmed by FK: pad rotation
**0.000° at every point** in the travel. The pads translate; they never rotate.

**What that check did turn up, and the thing to test first.** At the logged contact point the
two pad links sit ~69 mm apart, against a 30 mm block. And `effort` reads `0.000` on every line
of every log, so `gripper_close_until_contact` has nothing but jaw *position lag* to work with —
`CONTACT: jaw trailing its command by 0.0675 rad` is as consistent with the linkage binding as
with the block. If contact fires early the block is held **slack**, and a loose block swinging
and re-settling during transit explains a tilt that is worse at the place pose than at the pick
pose. A rigid error would be identical at both. (Related: the jaw mm/rad calibration, Stage
0b.3, is still unmeasured — it would settle this outright.)

**The other candidate** is a genuine mount offset. The URDF reaches the gripper through two
hand-authored right angles:

```
joint6output_to_camera_flange   rpy = "1.5708 1.5708 0"
camera_flange_to_gripper_base   rpy = "0 1.5708 1.5708"
```

If the physical mount does not match, "flange vertical" and "jaws vertical" differ by a
constant — which is precisely the original complaint that the tilt "is always the same, always
very consistently that angle, and never goes away."

**The distinguishing test takes five seconds:** grip a block, then try to move it by hand.
Shifts → slack grip, fix the contact detection. Rock solid and still tilted → mount offset, and
`GRIPPER_MOUNT_TILT_X_DEG` / `GRIPPER_MOUNT_TILT_Y_DEG` in `pick_place.py` correct it (both
default 0.0; tool-frame post-multiply, verified unit-norm and bit-identical to prior behaviour
when disabled).

**Frame note, because it is the subtle part.** A mount error is constant in the *tool* frame, so
the correction must rotate with the gripper. `SAG_PRECOMP_*` is a world-frame effect and
pre-multiplies. Apply a tool-frame error in the radial frame and it cancels at one pose and
doubles at the pose 180° opposite — and grasp and place here *are* ~180° apart, which is one
candidate explanation for the pick/place asymmetry on its own.

---

### Where this model still does not apply — Stage 0b.4

The tilt is solved **at the two hardcoded pick and place poses**. That is exactly what
`pick_place.py` needs and it is verified there. But the four constants are fitted to two
positions, both at 0.249 m reach, with one block mass, and reach/height dependence is
unmeasured. Stage 1 grasps at arbitrary positions inside the zone and Stage 3 introduces 24
blocks of differing mass — three directions this model has no data in.

So: done for today's goal, and **re-check the tilt once Stage 1 is grasping off-centre**. If it
degrades away from the fitted poses, do not chase it with more constants; go to option 2 below.

Two ways forward, and the second is much better:

1. **Calibration map.** Sweep tilt over a grid of (reach, height, payload), fit radial and
   tangential as functions of those. Straightforward, but it is a lot of robot time, it goes
   stale whenever the tool or a servo changes, and it is still open-loop — nothing detects when
   it has drifted.

2. **Measure the tilt in situ from the tags.** The wrist camera is *rigid to the flange*, so
   the flange's orientation relative to the zone plane is recoverable from the AprilTag
   homography that Stage 1 already computes at every hover. A camera perfectly perpendicular to
   the mat images the tag square as a square; any tilt turns it into a trapezoid, and the
   asymmetry gives both the direction and the magnitude. So the arm can measure its own tilt,
   at the actual pose, on every pick — no model, no map, and self-correcting if anything
   changes.

   This is `TESTS.md:63`'s "external metrology" applied to orientation rather than position,
   and it is the same move that already makes the position loop trustworthy: `camera_in_zone()`
   converges on where the camera *actually* went rather than where the encoders claim.

   **What it needs:** camera intrinsics, to decompose a homography into rotation. Currently
   uncalibrated — but `camera.launch.py:14-18` already has a `camera_info_url` argument sitting
   unused, and a one-off checkerboard calibration populates it. That is the single highest-value
   unblock for the assembly goal, because it converts tilt from something modelled into
   something measured.

   Ordering note: this supersedes the constants above rather than extending them, so do not
   invest in the calibration map first.

---

## Stage 0 — characterization and calibration

Before any of the feature's own motion. Most of it needs no robot.

### 0a. Characterization

Run on the Pi. **Stop `mycobot_bridge.py` first** — it holds `/dev/ttyAMA0` exclusively and
two processes on that port produce garbage that looks like a hardware fault.

```bash
python3 src/mycobot_hardware/scripts/serial_rate_probe.py --deadzone-sweep --dry-run
python3 src/mycobot_hardware/scripts/serial_rate_probe.py --deadzone-sweep --out /tmp/deadzone_sweep.csv
```

Read the k=1 column for go/no-go. Then write `--repeatability` (a thin wrapper around the
existing `settle()` / `read_angles()`, sketched in `TESTS.md`) and run it for the scatter
number.

### 0b. Calibration

1. **AprilTag detector available on the Pi?**
   ```bash
   python3 -c "import cv2; print(cv2.__version__); cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)"
   ```
   The Pi has `python3-opencv` 4.2.x from `pi_setup/install_pi_galactic.sh`. Debian's build
   normally includes contrib/aruco, and 36h11 matches the existing
   `scripts/assets/tag36h11_0000{0..3}.png` textures. If it is missing, add
   `pupil-apriltags` to `pi_setup/requirements.txt`; both code paths sit behind one function
   in `zone_vision.py`. Settle this before writing detection code.

2. **Field of view — a genuine go/no-go, and a stricter one now.** `hover_z()` clamps to
   `MAX_HOVER_Z = 0.205` (`pick_place.py:118,142`), so hover height is *not* a free
   variable. At ~0.20 m the camera must see a ~7in span (the 6in tag square plus the 1in
   tags straddling its vertices) for all four tags to be in frame — up from ~5in before the
   6in decision, precisely because that decision trades FOV margin for usable working area.
   Park at hover, capture a still, count tags. If four do not fit at `MAX_HOVER_Z`, this is
   the moment to know it, not a moment to guess through: fallback options in order of
   preference are (a) accept **three** tags — a homography needs four point pairs, but three
   tag centres plus the known square geometry determine the fourth — or (b) a wider-FOV lens
   on the wrist camera, since hover height itself is not adjustable.

3. **Gripper jaw opening in millimetres.** `GRIPPER_OPEN = 0.15`, `GRIPPER_CLOSED = -0.60`,
   jaw span 0.75 rad (`pick_place.py:167,211`) — but **nothing currently maps radians to
   millimetres of jaw gap**, and Stage 3's entire "2 inch maximum" rule depends on that map.
   Measure the physical gap at three commanded values with calipers, fit a linear map, store
   as `JAW_RAD_TO_M`. Confirm the Stage 1 block (1.18in = 30.0 mm) sits well inside range.

4. **Camera framing offset.** `camera_flange_to_camera_link` puts the lens 40 mm laterally
   and 8.5 mm axially off the flange origin (`mycobot_280_pi_camera_flange_plus_gripper_unchanged_transforms.urdf:289`).
   The *flange* hover target must be offset so the *camera* looks at the zone centre. This
   is framing only — it does not enter the position calculation — but get it roughly right
   or tags fall out of frame. Derive it from the URDF chain rather than guessing;
   `tool_frame_check.py` already does this kind of pure-Python FK.

5. **Still-image corpus.** ~20 frames on the Pi: block at zone centre, at each corner, at
   0/15/30/45 deg, plus two deliberately bad frames (one tag occluded, one with glare). Copy
   to mars. All threshold tuning happens against this corpus, not against the robot. The two
   bad frames must come back `success=false` with a useful message — never a wrong answer.

---

## The 2-of-4 tag occlusion, solved (2026-07-30)

**The gripper hides half the tags.** It hangs in front of the lens, so every hover the arm can
reach sees 2 of the 4 tags, never more. `MIN_TAGS = 3` refused every real frame — Stage 1 could
not have run at all on hardware.

**Two tags is not the degraded case it looked like.** Each tag gives 4 corners and each corner 2
equations: 16 equations for a homography's 8 DOF. Genuinely over-determined, so `homography_rms`
still carries information. (One tag would be 8 equations for 8 DOF — exact fit, zero residual by
construction, useless as a health check. That is why the floor is 2, not 1.) `MIN_TAGS = 2`.

What two tags actually cost is **conditioning**, and the measured numbers corrected two wrong
intuitions:

| constellation | spread ratio |
|---|---|
| all four | 1.0000 |
| any three | 0.5890 |
| adjacent pair | 0.1644 |
| **diagonal pair** | **0.1170** |
| single tag | 1.0000 |

- A **diagonal** pair is *worse* conditioned than an adjacent one, which is backwards from
  "diagonal spans the zone better". For a homography what matters is general position, and two
  diagonal tags put all 8 corners in a thin band along the diagonal. An initial `0.12` floor
  would have silently rejected every diagonal pair — a case wrist rotation actively produces.
  Floor is `0.08`.
- A **single tag** scores 1.0000, because the metric measures the constellation's *shape* and
  four corners of one tag are a perfect square. It is scale-blind. `MIN_TAGS` excludes that case,
  not this.

Anything above the floor is *kept*, with quality expressed through `homography_rms` and the
multi-view spread. Hard-rejecting weak-but-usable views in exchange for a cleaner-looking single
answer is how a system ends up confident and wrong.

### The acquisition plan, and the one design decision that matters

Four stills at 90° of **wrist** yaw (`joint6output_to_joint6`, not base rotation — J0 has 1.83°
of measured backlash, 8.0 mm at r = 0.25 m, which would move the camera further than the thing
being measured). Four offsets guarantee every tag appears in at least one still whichever pair
the gripper starts out hiding. Stops early once 3 views are usable.

**Each still is solved independently, then the results are fused** — *not* pooled into one big
fit. Pooling correspondences across stills would require knowing how far the wrist actually
turned, and as of the root-cause finding above, that is precisely what this robot cannot be
trusted about. Solving each still from only its own visible tags means **the fused answer never
depends on the wrist angle being what the encoder claims.** The rotation only has to *change the
occlusion*; it does not have to be known. That is what makes this work on worn gears.

The second payoff is free and arguably worth more: **the spread across views is an independent,
end-to-end error bar**, measured on the real mat under real lighting. Nothing else in the system
produces one.

### Verified (synthetic, `zone_vision_selftest.py`, 0 failures)

- Four stills each seeing a different pair: **0.09 mm fused vs 0.17 mm for the worst single
  view**, spread 0.23 mm — and the test asserts the spread actually bounds the error, since a
  confidence number that does not is worse than none.
- Mars-side path against mock service responses: **0.05 mm fused** from views scattered up to
  1.48 mm.
- **Yaw wrap:** views at 88°, 2°, 0.5°, 89° on a 4-fold block fuse to **89.87°**. Naive averaging
  gives **44.9°** — the worst possible answer, putting the jaws on the corners instead of the
  faces. This is the one fusion bug that would be catastrophic rather than noisy, so it has a
  dedicated test with no rendering involved.
- Two distinct blocks do not merge into one cluster.

No `.srv` change was needed: the response already carries the zone-local fields fusion reads, and
fusion runs on mars. That avoids a rebuild of `swarm_interfaces` on both machines from identical
source, which is a documented pain point.

---

## Stage 1 — pickup zone hardcoded, square blocks

Square blocks, 1.18in (30.0 mm) thick. Zone world pose hardcoded.

1. `go_home`, open gripper.
2. Hover over the zone centre (flange target offset by the camera offset from 0b.4).
3. `DetectBlock("pickup")`. On the Pi: grab one fresh frame (discard stale ones — the arm
   was just moving), detect the four tags, build the homography from the known 6in tag
   square, reject on high `homography_rms`, mask to the zone interior, find the largest non-tag
   contour, `cv2.minAreaRect` → centre + angle + side lengths, map back through the
   homography into world metres.
4. Reduce yaw by the block's symmetry. A square is 4-fold, so `yaw mod 90 deg`; pick the
   representative nearest zero to minimise wrist travel and stay clear of
   `joint6output_to_joint6`'s -2.4434 rad limit that `_is_near_joint_limit` guards.
5. Move to the block's XY at the block's yaw.
6. **Correct on the camera position, not the block position.** Second still. Stopping
   conditions come from 0a:
   - residual **below the repeatability scatter** → converged, descend;
   - residual **below the dead-zone floor** but above scatter → the arm physically cannot
     make a correction this small. Descend anyway and log it; retrying is guaranteed to do
     nothing. This is precisely the failure mode Test 1 exists to predict.
   - otherwise correct once more; abort after two failed corrections rather than descending
     onto a mislocated block.

   Log every (commanded correction, measured result) pair — that is the dataset the
   disturbance-observer branch needs, and it comes free here.

   > **Correction, made during implementation 2026-07-28.** An earlier draft of this
   > document said to re-detect the *block* to verify the move. That verifies nothing. The
   > block's zone-local position is derived from the tags, so it is the same answer no
   > matter where the arm is standing — move and re-detect and you have re-measured the
   > block, not the move.
   >
   > What *does* measure the arm is where the **camera** ended up: the zone-local point
   > under the image centre, recovered by mapping that pixel back through the homography
   > (`zone_vision.camera_in_zone()`). That is an external observation of the arm's
   > position which owes nothing to its encoders, and is therefore blind to exactly the
   > gravity droop and dead-zone effects that make the encoders untrustworthy. `TESTS.md`
   > names "external metrology" as the fallback if residuals turn out not to be plainly
   > correctable — this is it, and it arrives for free with the tags.
   >
   > Two caveats, both systematic and both constant at a given pose: the image centre is
   > used as the principal point (uncalibrated, can be a few percent off), and the optical
   > axis is assumed perpendicular to the mat (the URDF says 0.5° off; the arm's mechanical
   > tilt adds more, and 3° at 0.20 m is ~10 mm). So the absolute value is uncalibrated
   > while **differences between hovers are trustworthy** — and a correction step only
   > needs the difference.
7. Cartesian descend to `block_centre_z + GRASP_OFFSET_Z`, holding the block's yaw.
8. `gripper_close_until_contact`, retreat, then place via the existing hardcoded
   `PLACE_XYZ` path, unchanged.

**Done when** a square block dropped at a random spot and random rotation inside the zone is
picked reliably with no hand-alignment.

---

## Stage 2 — place zone also tag-located

Second detection at the place zone. Same service call, but the question is "where is the
zone centre, really" rather than "where is the block" — the homography gives the zone's true
world pose, correcting the hardcoded estimate. Release at the corrected centre, reusing the
existing `lz = place_surface_z + block_size/2 + GRASP_OFFSET_Z` with the block size measured
in Stage 1 instead of `--block-size`.

Also check the place zone is *empty* before releasing — same contour pass, inverted
question.

---

## Stage 3 — the 24-block database

`blocks.yaml`, one entry per block:

```yaml
- id: cuboid_1x3
  footprint: [0.0254, 0.0762]   # m, short side first
  height: 0.0254
  shape: rect
  symmetry: 2
  grasp_axis: short             # which footprint dimension the jaws span
  grasp_height_frac: 0.5        # how far up the block to grip
```

**Footprint matching needs a tolerance of at least 3–4 mm.** The synthetic sweep already
shows 2.9 mm (otsu) to 3.8 mm (canny) worst-case dimension error under ideal lighting, and
real frames will not beat that. Blocks whose footprints differ by less than ~5 mm are not
reliably distinguishable, which is a constraint on the block set as much as on the code —
and it compounds with the height ambiguity below.

Match the detected footprint to the DB within a tolerance. Grasp yaw = block yaw, rotated
90 deg if the jaws must span the minor axis. Grasp z from `height * grasp_height_frac`.
**Reject outright if the graspable dimension exceeds the calibrated jaw span** — this is the
1in-vs-3in rule, and it needs 0b.3 to be a real number.

### Open question, flagged before it costs a rebuild

A top-down view **cannot** distinguish two blocks with the same footprint but different
heights — a 1x1x1 cube and a 1x1x3 pillar standing upright look identical. Three ways out:

1. make footprints unique across the 24 blocks by construction;
2. add a second view from an oblique angle;
3. have the caller declare which block it is, and use vision only for pose.

This has to be decided **before** populating the DB, because it determines whether the DB
can be keyed on footprint at all.

---

## Stage 4 — place anywhere in the place zone

Detect existing blocks in the place zone, then ask the user "on top" or "beside".

"Beside" needs a free-space search in zone coordinates: a small occupancy grid over the zone
plane, place at the centroid of the largest free region that fits the held block's footprint
plus a margin.

"On top" needs the supporting block's height from the DB added to the release z, and is the
point at which `PROJECT_CONTEXT.md`'s open item 6 — telling MoveIt the block is in the
gripper, so place-move collision checking accounts for it — stops being optional.

---

## Measurements

Filled in as taken. An empty row means the corresponding constant in the code is still a
guess.

| Quantity | Value | How measured | Date |
|---|---|---|---|
| Synthetic position accuracy (worst of 36) | **0.27–0.30 mm** | `zone_vision_selftest.py` | 2026-07-28 |
| Synthetic yaw accuracy (worst of 36) | **1.5–2.0 deg** | `zone_vision_selftest.py` | 2026-07-28 |
| Synthetic dimension accuracy (worst of 36) | **2.9 mm otsu / 3.8 mm canny** | `zone_vision_selftest.py` | 2026-07-28 |
| Synthetic homography RMS | **0.27 px** | `zone_vision_selftest.py` | 2026-07-28 |
| Synthetic camera-position accuracy | **0.13 mm** | `zone_vision_selftest.py` | 2026-07-28 |
| Flange → lens offset (from URDF) | **40.0 mm** lateral, 18.5 mm axial | `tool_frame_check.flange_to_camera()` | 2026-07-28 |
| Camera optical axis vs vertical at grasp pose (URDF) | **0.5 deg** | `tool_frame_check.flange_to_camera()` | 2026-07-28 |
| **Dead band, J0 / J1 / J2** (median bias before a joint moves) | **0.99 / 1.36 / 1.08 deg** = 4.3 / 5.9 / 4.7 mm @ r=0.25 m | Test 1 `--deadzone-sweep`, 108 trials | 2026-07-29 |
| Dead band, worst case seen | **2.78 deg** = 12.1 mm | Test 1 | 2026-07-29 |
| **Repeatability, same direction** | **0.045 deg** = 0.20 mm (below the 0.088 deg quantum) | Test 1, spread across 3 repeats | 2026-07-29 |
| **Backlash J0** (lost motion on reversal) | **1.83 deg** median, 2.01 max = 8.0 mm @ r=0.25 m | Test 1 backlash trials | 2026-07-29 |
| Backlash J1 / J2 | 0.53 / 0.80 deg | Test 1 | 2026-07-29 |
| J0 residual across 4.4x lever span (gravity-free control) | **flat, 0.98 deg**, span 0.13 | Test 1 | 2026-07-29 |
| Jaw gap at `GRIPPER_OPEN = 0.15` rad | — | calipers | — |
| Jaw gap at `GRIPPER_CLOSED = -0.60` rad | — | calipers | — |
| `JAW_RAD_TO_M` slope | — | linear fit, 3 points | — |
| Tags visible at `MAX_HOVER_Z = 0.205` | — | count in captured still | — |
| Homography RMS, typical good frame | — | `zone_view.py` on corpus | — |
| Homography RMS, reject threshold | — | worst acceptable frame in corpus | — |
| Block position error vs ruler ground truth | — | 3 corpus frames | — |
| Detection round-trip time on the Pi | — | service call timing | — |

---

## Risks

- **All four tags must stay visible.** A block near a corner can occlude one. Mitigation:
  the three-tag fallback, and reject-and-report rather than guess.
- **No lens undistortion.** Homography-only calibration accepts residual distortion, worst
  at frame edges. If Stage 1 residuals are poor *specifically at the zone corners*, that is
  the distortion signature; the fix is a one-time intrinsic calibration fed through
  `camera.launch.py`'s already-present but unused `camera_info_url` argument.
- **Pi CPU contention** with the 100 Hz control loop and serial bridge. Detection is gated
  to stationary hover by design; measure the round trip and confirm it does not perturb
  motion.
- **Custom interface across distros.** `swarm_interfaces` must be built on both machines
  from identical `.srv` source. If cross-distro type matching misbehaves, fall back to
  `std_srvs/Trigger` with a JSON string payload — ugly but distro-proof. Do not spend long
  debugging this.

---

## The trap, restated

`arm_group_controller` reports "Goal reached, success!" from elapsed time alone — there is
no `constraints:` block in `ros2_controllers.yaml`. **ROS logs are not evidence of physical
motion on this setup.** Confirm every grasp visually. This has bitten the project
repeatedly, and a vision feature is exactly the place where a plausible-looking log and a
motionless arm are easiest to confuse.
