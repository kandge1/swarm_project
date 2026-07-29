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
| 1 | Pickup zone hardcoded, square blocks, random position + rotation | **Code complete, unverified on hardware** |
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
grasp tilt was **wrongly believed** mechanical (see [the tilt](#the-grasp-tilt-solved-2026-07-29)), and `_send_goal_and_wait`
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

## The grasp tilt, solved (2026-07-29)

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

### Fixes applied

1. **`GRASP_QX/QY` were rounded to 4 decimals.** `0.7071 ≠ 1/√2`, and the resulting
   quaternion asked the flange for a pose **0.5019° off vertical**. Half a degree of the tilt
   was baked into the target before any solver or servo was involved. Now `±math.sqrt(0.5)`
   exactly → target tilt 0.0000°. Free.
2. **The settle re-send is now biased** (`SETTLE_BIAS_*` in `mycobot_bridge.py`). It used to
   re-send the *identical* command, which Test 1 proved can never work. It now aims at
   `command + gain·residual`, gain escalating 1.0 → 1.5 → 2.0 per attempt, capped at 4° and
   clamped to joint limits.
3. **A per-joint bias threshold** (`SETTLE_BIAS_MIN_RAD = 0.004`), *not* `SETTLE_TOLERANCE_RAD`.
   This distinction is the fix. The tolerance is 0.03 rad = 1.72°, larger than the individual
   errors causing the tilt — gating on it would have biased only `joint6output` (0.0426 rad,
   and vertical, so zero tilt) while skipping all four pitch joints. The correction would have
   looked active and fixed nothing. Caught in simulation before hardware time.
4. **`settle_pause()` after each descent.** The descent was declared converged at 0.0426 rad
   (inside `ARM_SETTLE_TOLERANCE` = 0.07), pick_place slept 0.5 s, the gripper close changed
   the command, and the bridge's 1.0 s quiet-period timer reset — so **the arm's settle never
   ran once before the grasp**. A correction mechanism that was enabled and never got a turn.

Simulated end to end against the measured residuals: one biased attempt takes tilt
3.704° → 0.000° and z 0.1448 → 0.1550 m, then stops.

### Not yet verified on hardware

That simulation assumes each servo undershoots its *new* target by the same signed residual —
the ideal Coulomb-friction model. Test 1 supports it for J0 (median `|err|/|e|` = 0.07 at
k=1) but the pitch joints had median 1.00 at k=1 and mostly moved only by k=2, which is
exactly why the gain escalates. **Expect the real improvement to be partial on the first
attempt.** What to check on the next run:

- `[mycobot_bridge] TIMING settle re-send ... bias[...]` lines appear, naming the pitch joints
  (needs `--log-timing`). If they say `(no bias)`, the residuals were under
  `SETTLE_BIAS_MIN_RAD` and nothing needed doing.
- Stop the arm in the grasp pose again, capture `/joint_states`, re-run the FK check. The
  number to beat is 3.70°.
- If the tilt persists, the next lever is `SETTLE_TOLERANCE_RAD` (0.03 → ~0.015). Its current
  value was chosen on the basis that small errors were uncorrectable, which was true for
  *unbiased* re-sends and is no longer the governing assumption. Left alone for now because
  it changes when settling fires at all, and that interacts with the hard-won smooth-motion
  and serial-flooding work.

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
