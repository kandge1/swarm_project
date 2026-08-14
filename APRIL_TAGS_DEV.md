# APRIL_TAGS_DEV.md — working state, terminals, and the open bug

Companion to `APRIL_TAGS.md`. That file is the **design**: why the homography
approach was chosen, the staging, the long-form justifications. This file is the
**bench state**: which machine runs what, what currently works, what is broken
right now, and the exact commands to reproduce and fix it.

Written 2026-08-02, appended to since. If you are a fresh session picking this
up, **start at the bottom, not the top** — the sections are chronological and
the early ones describe a state that no longer exists ("Where we are" and "THE
OPEN BUG" are both from 2026-08-02 and are history now).

**Current state, 2026-08-11:** open-loop grasp error is **0.58 mm RMS, 1.0 mm
worst**, verified with a caliper across three reaches, two bearings and two
wrist yaws. Read `CALIBRATION_2026-08-11.md` first, then the
"2026-08-07 → 2026-08-11" section at the end of this file.

**Picking up new work?** The open list is at the very end of this file, under
that same section. Item 1 is the in-zone sweep, which is half the scope that was
agreed on 2026-08-07 and has never been run. "THE STACKED-BLOCK PLAN" is still
the agreed longer-term direction.

---

## Vocabulary, because this caused real confusion

There are **two machines**, not three:

| Name | What it is | ROS distro |
|---|---|---|
| **mars** | The desktop workstation. Planning, IK, orchestration. | Jazzy |
| **the robot** | The Raspberry Pi *inside* the myCobot 280 Pi. Motor control, camera, vision. | Galactic |

"The Pi", "the robot", and "the arm" are all the **same machine**. There is no
separate control box. The camera is mounted on the wrist and is read directly by
a process running on that same Pi.

Nothing about the images crosses the network. The DDS link silently drops
anything over ~1400 bytes, so a 640x480 frame (921,600 B) is not merely slow, it
is impossible. Vision runs where the camera is.

---

## Terminal map (matches `WORKFLOW.md` numbering)

| # | Machine | What runs there |
|---|---|---|
| **1** | robot | `real_robot_hardware.launch.py` — bridge, `ros2_control`, controller spawners |
| **2** | mars | `real_robot_planning.launch.py` — `move_group` + RViz |
| **3** | mars | verification (`ros2 control list_controllers`, `ros2 topic hz /joint_states`), and `pick_place.py` |
| **4** | robot | `block_detector_node.py` — the vision service |
| **5** | mars | `tag_pick_place.py` — the orchestrator, and `ros2 service call /detect_block` |

For AprilTag work you need **1, 2, 4, 5**. Terminal 3 is optional and only for
sanity checks or running the old fixed-coordinate `pick_place.py`.

**Do NOT launch `camera.launch.py`.** Since 2026-07-31 `block_detector_node.py`
opens `/dev/video0` itself with `cv2.VideoCapture`. V4L2 allows one reader, so
running both means one of them fails to open the device.

### Terminal 1 — robot
```bash
cd ~/swarm_project
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot_hardware.launch.py
```

### Terminal 2 — mars
```bash
cd ~/swarm/swarm_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_280pi_camera_moveit2 real_robot_planning.launch.py
```

### Terminal 4 — robot
```bash
source ~/swarm_project/install/setup.bash
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_detector_node.py
```

### Terminal 5 — mars
```bash
source ~/swarm/swarm_project/install/setup.bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts

# dry run: survey, then park the jaws over the block at grasp height. No descent.
python3 tag_pick_place.py --dry-run --debug-image /tmp/zone.png --log /tmp/corrections.csv

# the real thing
python3 tag_pick_place.py --debug-image /tmp/zone.png --log /tmp/corrections.csv
```

All defaults are now correct — `--zone-origin` is no longer needed. Debug images
are written **on the robot** at `/tmp/zone_view0.png` … `_view3.png`.

---

## THE OPEN BUG (blocking, as of 2026-08-02)

**Symptom:** every `/detect_block` call times out after 45 s. The robot's own
log shows the detection *succeeded* — tags decoded, rms 0.5 px, blocks found —
and then:

```
'string data is not null-terminated, at rmw-cyclonedds-cpp/src/serdata.cpp:354'
'invalid data size,                  at rmw-cyclonedds-cpp/src/serdata.cpp:354'
The following exception was never retrieved: failed to send response:
    error not set, at rcl/service.c:287
```

**The isolating experiment (already run, 4 times):** park the arm at home so the
camera sees no tags and call the service — **it returns normally**. Hold a sheet
of AprilTags in front of the camera and call it — **it never returns**.

**What that proves.** `block_detector_node.py:184` sets `response.blocks = []`
on entry, and every early-return path leaves it empty. An empty sequence
serializes as a 4-byte zero length and **never touches `BlockDetection`'s
typesupport at all**. Populate it and the serializer enters `BlockDetection` for
the first time — which is exactly where `serdata.cpp:354` complains about a
non-null-terminated string. The only string in `BlockDetection` is `shape`.

**Ruled out, with evidence:**

- *Message content.* The exact reply was rebuilt from the run's own numbers and
  serialized on mars: **196 bytes**, no error, with plain floats, numpy floats
  and numpy strings alike.
- *MTU / fragmentation.* 196 B against a 1400 B limit.
- *Interface drift.* `DetectBlock.srv` and `BlockDetection.msg` are byte-identical
  between `src/` and `install/` on mars, and unchanged in git since 2026-07-30.
- *Vision.* The four debug stills from the failing run are the best of the
  project: rms 0.51–0.65 px, all four tags framed in the centred view.

**Conclusion.** `rcl_send_response` serializes locally, before anything goes on
the wire, and it is failing on the robot. So the robot's Python message module
and its C typesupport disagree with each other — a partial or stale
`swarm_interfaces` build. `DetectBlock.srv` changed in commit `6fd84b3` after
`BlockDetection.msg` was created, which is the window where half-regenerated
code gets stranded.

### Confirm it — 5 seconds, on the robot, no arm needed

```bash
python3 -c "
from swarm_interfaces.srv import DetectBlock
from swarm_interfaces.msg import BlockDetection
from rclpy.serialization import serialize_message
r = DetectBlock.Response(); r.success = True; r.message = 'x'
print('empty blocks[] :', len(serialize_message(r)), 'bytes')
b = BlockDetection(); b.shape = 'circle'; b.symmetry = 0
r.blocks.append(b); r.tag_ids = [0,1,2,3]
print('1 block        :', len(serialize_message(r)), 'bytes')
"
```

On mars this prints `56` and `164`. **If the second line throws on the robot,
the diagnosis is confirmed.**

### Fix — Terminal 4 on the robot

```bash
# stop the detector first (Ctrl-C in Terminal 4)
cd ~/swarm_project
rm -rf build/swarm_interfaces install/swarm_interfaces
colcon build --packages-select swarm_interfaces
# NEW shell -- the old one has stale paths cached
source ~/swarm_project/install/setup.bash
python3 ~/swarm_project/src/swarm_pkg/src/scripts/block_detector_node.py
```

The `rm -rf` is the part that matters. `colcon` reuses stale generated code, and
that is the failure mode itself.

Also worth checking for two installs shadowing each other:

```bash
python3 -c "import swarm_interfaces; print(swarm_interfaces.__file__)"
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -i swarm
```

### If a clean rebuild does not fix it

Then it is a real nested-sequence bug in Galactic's `rmw_cyclonedds` 0.22.6, and
the fix is to stop putting a nested message with a string on the wire. Flatten
`BlockDetection[] blocks` into parallel primitive arrays in `DetectBlock.srv`:

```
float64[] block_zx      float64[] block_zy      float64[] block_zyaw
float64[] block_width   float64[] block_length
uint8[]   block_symmetry
uint8[]   block_shape_code   # 0 unknown, 1 square, 2 rect, 3 circle
```

Smaller and less invasive than the `std_srvs/Trigger` + JSON fallback that
`APRIL_TAGS.md` lists under Risks. Requires a rebuild on **both** machines.

---

## Where we are

### Working, verified on hardware 2026-08-02

| | before | now |
|---|---|---|
| homography rms | 7.15 / 7.21 px (rejected) | **0.48 – 0.83 px** |
| tags in the centred still | 2 of 4, two clipped off-frame | **4 of 4** |
| block selection | picked a 9×23 mm tape sliver | rejects all slivers, confirms across 3 views |
| grasp-hover | "Planning FAILED", ran away out of reach | converges on 13 real IK seeds |

The last dry run parked the jaws over the block correctly and reported
`STAGE 1 COMPLETE`. Arm residuals at that pose: **J0 +1.21°, i.e. 4.8 mm
tangential at r = 0.2271**; every other joint under 2°.

Jaw orientation was verified square to the block, not diagonal:

```
commanded jaw yaw  -0.09 deg from world +X
achieved  jaw yaw  +2.20 deg
block yaw          +0.0 deg fused (-2.3 best single view)
```

`joint6output = 2.6264` looks alarming against the historical `1.828`, but
`pick_place.py:704` records the hardware-verified anchor: at `j6out 1.828` the
jaw sits at **+45°**, at `1.828 + 0.7854` at **0°**. The old 1.828 was the
pre-fix state where the jaw really was 45° off.

### The one thing never tested

**No descent has ever run.** Every session so far used `--dry-run`. The first
non-dry run is still ahead.

---

## Constants that matter, and where they live

### `pick_place.py`

| Constant | Value | Note |
|---|---|---|
| `ZONE_RADIUS_M` | `0.2286` | 9 in exactly. Moved in from 0.250 on 2026-08-02 to buy reach margin. |
| `PICK_XYZ` | `(0, +0.2286, 0.065)` | Z is a block **CENTRE**, hand-tuned on hardware. |
| `PLACE_XYZ` | `(0, -0.2286, 0.075)` | Z is a resting **SURFACE**, hand-tuned. |
| `GRASP_OFFSET_Z` | `0.09` | Flange to fingertip. Pure gripper geometry. |
| `MAX_HOVER_Z` | `0.205` | Reach ceiling, measured at radius 0.250 — now conservative, not re-measured. |

### `tag_pick_place.py` (runs on mars)

| Constant | Value | Note |
|---|---|---|
| `GRASP_FLANGE_Z` | `0.155` | `= PICK_XYZ.z + GRASP_OFFSET_Z`. **Not** computed from mat + thickness. |
| `DETECT_HOVER_Z` | `0.240` | Lens ends up ~0.2235 above the mat; focus floor is 0.220. |
| `DETECT_HOVER_PULLIN_M` | `0.041` | = the lens's lateral offset at wrist yaw 180. Centres the lens on the zone. |
| `MIN_FLANGE_RADIUS_M` | `0.150` | Self-collision floor. 0.1529 worked, 0.1209 collided. |
| `MAX_FLANGE_RADIUS_M` | `0.245` | Stops the correction runaway (see below). |
| `DETECT_WRIST_YAW_DEG` | `180.0` | Swings the lens to the far side of the flange so the pose is reachable. |
| `CAMERA_MOUNT_FLIPPED` | `True` | Camera is physically 180° from the URDF. Corrected in code, not in the URDF. |
| `MULTIVIEW_YAW_OFFSETS_DEG` | `(180, 150, 210, 120)` | Clustered, **not** 0/90/180/270 — see below. |
| `DETECT_ORI_XY_TOLERANCE` | `0.15` rad | Detection only; grasp/descent keep the tight 0.10. |
| `CORRECTION_CONVERGED_M` | `0.002` | |
| `CORRECTION_DEADZONE_M` | `0.005` | Below this the arm physically cannot correct. |
| `MULTIVIEW_MAX_SPREAD_M` | `0.006` | Cross-view agreement gate. |
| `MULTIVIEW_MAX_SPREAD_YAW_DEG` | `20.0` | Same, for yaw, skipped when symmetry == 0. |
| `DETECT_SERVICE_TIMEOUT` | `45.0` s | |

### `zone_vision.py` (runs on the robot)

| Constant | Value | Note |
|---|---|---|
| `DEFAULT_ZONE_SIZE` | `0.1016` | 4 in between **tag centres**. Was 6 in. |
| `DEFAULT_TAG_SIZE` | `0.0254` | 1 in printed tag. |
| `MAX_BLOCK_LENGTH_M` | `0.060` | Rejects grid-line slivers. Detector over-reads size, hence 60 not 40. |

---

## Decisions worth not re-litigating

**Why the survey yaws are 180/150/210/120 and not 0/90/180/270.** We *are*
rotating only `joint6output`; the flange stays parked at (0, 0.1876) for all
four stills. But the lens sits ~40 mm off the wrist axis, so the wrist rotation
swings it around a 40 mm circle rather than spinning the view in place:

```
yaw   0 -> lens 79.9 mm off zone centre      yaw 180 -> lens  0.4 mm
yaw  90 -> lens 56.2 mm                      yaw 270 -> lens 56.9 mm
```

The zone is 101.6 mm across, so at 80 mm off centre the camera is not looking at
the mat. Measured on hardware 2026-07-31 and the debug stills matched exactly.

Moving the flange in a compensating circle to keep the lens centred at every yaw
would need, at yaw 0, a flange radius of **0.2675** — past reach. The other
three (0.2309 / 0.1876 / 0.2312) are fine. So true 90° spacing is unavailable.

It costs nothing: the design never needed the angles to be 90° apart or even
*known*. `zone_vision.analyze_multi` solves each still independently precisely
so it never has to trust how far the worn wrist gearing actually turned. The
angles only have to **change which tags the gripper occludes**, and 120→210
still rotates the occlusion shadow a full 90°.

**Why the grasp height is not computed.** `PICK_XYZ.z` (block centre, 0.065) and
`PLACE_XYZ.z` (resting surface, 0.075) on the same flat mat would, as pure
geometry, require a −20 mm thick block. They differ because pick and place sit
at opposite ends of the workspace and the arm droops differently at each. Both
are right; neither is derivable. Stage 1 measures X, Y and yaw — what a top-down
camera can actually see — and takes Z from the configuration that already picks
this block off this mat.

**Why `--verify` is off by default.** The survey commands the flange where the
lens should land exactly on the zone centre, and the detector then measures the
camera at zone (+21.5, +4.3) mm. Something is ~22 mm out, but a single run cannot
say whether it is the **arm** (correction helps) or the **camera model** —
`zone_vision.camera_in_zone`'s own docstring flags an uncalibrated principal
point and a non-perpendicular optical axis. Guessing is a coin flip on making
the grasp worse. `--dry-run` now parks the jaws over the block so one photo
settles it.

**Why `MAX_FLANGE_RADIUS_M` exists.** Commanding an unreachable target does not
fail cleanly: IK finds nothing, OMPL satisfies its 4 cm position sphere by
parking the arm **short**, and a short pose reads as an error pointing outward —
so each correction makes the next reading worse. Hardware 2026-08-02:
0.2271 → 0.2770 → planning FAILED. Confirmed independently by the homography
scale: 2891 px/m against the survey's 2466 puts the lens at 0.1906 m instead of
0.2235, i.e. **33 mm low and below the focus floor**.

**`px/m` is a free height gauge.** It is the homography scale at the tag plane,
so `lens height = f / (px per m)`. Anchoring `f` on the survey still gives
551 px, and a 640×480 sensor at ~60° HFOV predicts 554. Use it to find out where
the arm actually went without trusting a single encoder.

---

## Known systematic errors, unfixed

**Parallax inflates the block.** The homography solves the **tag plane**; the
block's top face is 30 mm above it, so it is magnified by `h/(h−t)` =
0.2235/0.1935 = **1.155**. A 30 mm block therefore reads ~34.7 mm, and it
measures 36.5 × 37.3 — parallax accounts for ~4.7 mm of the 6.5 mm over-read,
the rest looks like the shadow edge visible in the debug stills.

The same effect displaces the **centre** away from the lens's ground point by
`0.155 × (distance from it)`. The lens moves 20–30 mm between stills, which
predicts a few mm of view-dependent scatter; measured cross-view spread is
**3.9–4.0 mm**. It is a bias, not noise, so fusing more views will not average it
out. `--block-thickness` is a parameter and `h` comes free from `px/m`, so it is
correctable — do it after a grasp lands, not before.

Note that converging the camera **over** the block kills this for free: a point
directly under the lens projects to its true position whatever its height. That
is why `--verify` targets the camera *at* the block.

**"Circle" vs "square" is unstable, and `reduce_yaw`'s guard is inert.** The
30 mm square block classified as `circle` in two of three stills and `square` in
the third — its `fill_ratio` sits right on `CIRCLE_FILL_MAX = 0.86`. Symmetry 0
then means "any yaw grasps it", and forcing yaw 0 on a square sitting at 45°
drives the jaws at its 52 mm diagonal.

`reduce_yaw` in `tag_pick_place.py` was changed to fold symmetry 0 mod 90 rather
than return 0.0 — correct, but **inert**, because `zone_vision.py:942` already
returns `0.0` for symmetry 0 during fusion, so the yaw is destroyed upstream
before `reduce_yaw` sees it. That is why the log reads `yaw +0.0 deg`. The real
fix belongs in `zone_vision` and needs a robot-side restart.

**J0 backlash.** 1.83° = 8.0 mm at r = 0.25 (7.3 mm at 0.2286), larger than the
dead zone itself. Any correction that reverses direction spends its first
~7 mm taking up slack. The standard fix is a unidirectional final approach; not
implemented.

**Sag pre-compensation has expired.** `SAG_PRECOMP_*` was fitted at 0.249 m
reach and no grasp happens there any more. Direction is known — less reach, less
droop, so it now over-corrects — magnitude is a fraction of a degree, inside the
scatter of the original fit.

**Correction, 2026-08-02: the droop is much larger than "a fraction of a
degree", and the reason it looked small was a bug in the measurement.**

`serial_rate_probe.py`'s `POSTURES` table stored gravity moment arms as
*unsigned* magnitudes. Five of the six postures put the arm on one side of the
shoulder axis (J2 from 0 to −90°); `extended` puts it on the other (J2 = +45°),
so gravity loads it the opposite way. Taking `abs()` folded the two halves
together and regressed the residual against a quantity that is not physical.

Re-analysed with the sign restored, and with the residual split into the part
that reverses with travel direction (friction/dead zone) and the part that does
not (gravity):

| | residual vs **unsigned** arm | residual vs **signed** arm |
|---|---|---|
| J2 shoulder | R² 0.00 / 0.03 | **R² 0.77 / 0.93** |
| J3 elbow | R² 0.09 / 0.10 | **R² 0.87 / 0.88** |

(test1_full.csv 2026-07-29 / test2_full.csv 2026-08-02.) Gravity droop is the
**largest modelable error on the pitch joints**, roughly −5°per metre of moment
arm, and it is clean enough to feed forward. The old `abs()` is also what made
`report_sweep` print "They DISAGREE … do not use this slope as a feedforward"
on every run it ever did.

Both files are fixed: `POSTURES` now carries signed arms for all six joints,
and `report_sweep` splits symmetric from antisymmetric before fitting. The
self-check passes — J1, whose gravity arm is 0 by construction, comes out with a
symmetric term of −0.11° and an antisymmetric term of +0.90°, which is half its
1.67° measured backlash, exactly as it must be.

**Joint 3 (`joint5_to_joint4`) was the missing term**, and it turned out to be
the largest one. It carries a gravity arm of ±0.119 m — as much as joint 2 — and
had never been measured, because the old table stopped at three columns and the
default `--sweep-joints` stopped with it. `test3_full.csv` (2026-08-02, six
postures × 3 repeats × both directions, 144/144 trials usable) closes it:

| joint | | droop coefficient | R² |
|---|---|---|---|
| 1 | `joint3_to_joint2` shoulder | −5.47 °/m | 0.93 |
| 2 | `joint4_to_joint3` elbow | −4.59 °/m | 0.90 |
| 3 | `joint5_to_joint4` | **−13.16 °/m** | **0.96** |

At the IK solution actually chosen for the grasp (`pick_place.py:517`) that
predicts **1.25 + 0.77 + 0.90 = 2.92° of flange tilt and 8.9 mm of jaw
displacement, 8.6 mm of it straight DOWN.** Measured tilt at that pose before
any correction was 4.19°, so the model accounts for **70%** of it.

**The other 30% is not droop.** Joints 4 and 5 have ~zero gravity arm at the
grasp pose too, so there is no unmeasured pitch joint left to blame. The
remaining ~1.3° is a fixed mount/URDF offset — a constant, not a load effect.

### What is now implemented

- `mycobot_bridge.py` — `GRAVITY_FF_ENABLED`, the `JOINT_FF_BIAS_DEG` the
  `SETTLE_BIAS` comment promised and nobody ever wrote. Applies the measured
  bias to **every streamed setpoint**, which is the whole trick: during a
  trajectory the joint is already moving, so the dead band never arms and the
  error can be aimed past instead of corrected after. Bias capped at 3.0°
  (worst case anywhere in the tested envelope is 1.77°) and clamped to joint
  limits. Its embedded FK reproduces the sweep table to 5×10⁻⁵ m.
- `pick_place.py` — `SAG_PRECOMP_*` **zeroed**, old values preserved in the
  comment for a one-line revert. Leaving both live would over-correct by ~3°.
  Zeroed rather than scaled to 30% because two corrections for one effect
  cannot be tuned simultaneously, and the leftover is a different *shape* than
  this radial+tangential+payload model.

### Verified on hardware 2026-08-03 — it works, and it over-corrects

First clean A/B, `ff_verify.py` at the grasp pose, bridge banner confirming
`DISABLED` then `ENABLED`:

| joint | ff-off | ff-on | change | has a coefficient? |
|---|---|---|---|---|
| j0 `joint2_to_joint1` | +0.77 | +0.95 | +0.18 | no |
| **j1 `joint3_to_joint2`** | +0.94 | **−0.65** | **−1.59** | **yes** |
| **j2 `joint4_to_joint3`** | +0.75 | **−0.83** | **−1.58** | **yes** |
| **j3 `joint5_to_joint4`** | +0.83 | **+0.04** | **−0.79** | **yes** |
| j4 `joint6_to_joint5` | −0.35 | −0.08 | +0.27 | no |
| j5 `joint6output` | +1.12 | +1.21 | +0.09 | no |

The three joints with coefficients moved 1.59 / 1.58 / 0.79°; the three without
moved 0.18 / 0.27 / 0.09°. **A 7× separation, exactly the predicted signature** —
the effect is real and specific, not drift.

The model predicted the droop well: at ff-off, j2 measured +0.75 against +0.77
predicted and j3 +0.83 against +0.90. **j3 is essentially perfect after
correction, +0.83° → +0.04°** — it still fell 0.94° short of its *biased*
command, and the +0.90 bias cancelled that almost exactly, which is the
mechanism working as designed.

| | ff-off | ff-on |
|---|---|---|
| flange tilt | 2.54° | **1.44°** |
| jaw position error | 8.2 mm | **6.4 mm** |
| jaw dz | **−7.1 mm** | **+5.1 mm** |

**j1 and j2 over-correct** — their errors flipped sign rather than going to
zero, and dz overshot through zero from 7.1 mm low to 5.1 mm high.

**Do not retune from this run.** It is n=1, and the leftover residuals
(±0.65–0.83°) sit inside the 0.4–0.9° antisymmetric friction band that no
feedforward can address. The per-joint scale factors it implies are incoherent
(~0.5× for j2, ~1.0× for j3, on two joints that should behave alike), which is
what n=1 noise looks like. `ff_verify.py --repeats 3` now returns to home
between runs and reports mean vs spread, and only flags a joint for tuning when
its mean error exceeds its own spread.

This run did **not** exercise the `write_command` fix — home→grasp is 2.2 rad,
which cleared the old epsilon anyway. That remains unverified.

### The leftover tilt is the MOUNT, not sag (2026-08-03)

The arm still visibly slouched after the feedforward landed, so the grasp pose
was sampled directly from `/joint_states`, once at rest and once with the
gripper held physically vertical by hand.

**At rest the joints were already at their commanded target** — `joint3_to_joint2`
off by **−0.12°**, `joint4_to_joint3` by **−0.12°**. The joint-space droop is
gone. And the URDF puts the flange **0.17° from straight down** at those exact
values. The arm is doing what it was told; the tool is not where the URDF says.

Holding it physically vertical took **+1.05° on `joint3_to_joint2`** and
**+0.53° on `joint4_to_joint3`**, everything else under 0.2° — a **1.68°
tool-frame rotation** from the commanded orientation.

That is a fixed geometric offset, and it must **not** go into
`GRAVITY_FF_COEFFS`: those are scaled by the gravity moment arm, so a constant
folded in there would be right at this one pose and wrong everywhere else. It
belongs in `GRIPPER_MOUNT_TILT_*`, which post-multiplies in the tool frame and
therefore rotates with the gripper — so it generalises across the workspace
even though it was measured at one pose. Now set to **X −0.37, Y −1.48**
(the −0.71° yaw component is dropped; rotation about the approach axis only
spins the jaws and `GRIPPER_YAW_DEG` already owns that).

It looked worse than before because `SAG_PRECOMP_*` had been empirically
cancelling this constant, and zeroing it left the offset uncorrected.

**The let-go test, answered 2026-08-03.** Held vertical and released, the arm
**droops back** every time. Held there, the servo neither resists (no whine)
nor drives. So the joint has ~1° of free play, the servo exerts no torque
anywhere inside it, and gravity parks the arm at the bottom — consistently, in
the same direction, because gravity does not change direction.

That is *why feedforward is the only tool that works here*: the servo sees no
error inside the window, so no amount of feedback or integrator gain can act on
it, but the resting position is deterministic and can therefore be aimed past.

**Critically, the encoder SEES the play** — it moved +1.05° when the arm was
lifted by hand. If the play sat between the encoder and the output link the
reading could not have changed. So it is upstream of the encoder, and
`/joint_states` reads the true joint angle. That is what makes it a valid
instrument for calibrating any of this.

**And that is why the play does not explain the tilt.** At rest the encoder
reads −0.12° from target and the URDF puts the flange 0.17° from vertical
there, yet it is visibly tilted. 0.12° of joint error cannot make 1.68° of tool
tilt, so ~1.5° is unaccounted for *in encoder space* — either the physical
mount not matching the URDF's two hand-authored right angles, or J2's encoder
zero being off. Indistinguishable from this data, and both fixed by the same
tool-frame correction.

**Play is not backlash.** test3 measured J2's backlash at **0.44°, the smallest
of the pitch joints** (J3 0.79, J4 0.61), against 1.05° of measured free play.
The excess is elastic compliance, which is load-dependent — and J2 shows it
worst because it carries the largest gravity moment arm (0.24 m at the grasp).
That part is already modelled; the FF predicted +0.77° on J3 against +0.75°
measured.

**Still open:** the same pose measured **−0.65 / −0.83°** during the ff-on A/B
versus **−0.12 / −0.12°** here — **~0.6° of run-to-run scatter on the same
joints**, which is the free-play window showing up as noise and the reason
`--repeats` exists before any coefficient is touched.

### It is NOT a pure mount offset — it is pose-dependent (2026-08-03)

`--hover-only` with `GRIPPER_MOUNT_TILT_* = (−0.37, −1.48)` live, then the same
two-echo hand measurement at the hover. Only **one** joint moved:
`joint3_to_joint2` by **+0.79°**; four of the six were bit-identical. The
remaining correction is 0.80° (local X −0.63, Y −0.48), which would put the
constants at **X −1.00, Y −1.96**.

**Do not just apply that.** The required URDF tilt for a physically vertical
tool measures:

| pose | required |
|---|---|
| grasp | **~1.68°** |
| hover | **2.86°** |

A rigid mount error is a fixed rotation in the TOOL frame, so it would have to
be the *same magnitude at both*. It is not — they differ by 1.2°. So part of
this is pose-dependent, i.e. load-dependent compliance the gravity model
under-predicts at the hover, and `GRIPPER_MOUNT_TILT_*` is the wrong home for
that part.

Fitting at the hover would over-correct the **grasp** by ~0.5°, and the grasp is
where blocks are actually picked. `--grasp-only` was added to park at the grasp
pose with the jaws open so the measurement can be repeated where it matters.

**Correction, after a third measurement:** the pose-dependence above was mostly
an artifact of the *first* sample. Total required measured 1.68° (nothing
applied, judging the full 2.6° tilt by eye), 2.86° at the hover and 2.60° at the
grasp — and the two taken *with* a correction live agree to 0.26°. Eyeballing
"vertical" is far harder at 2.6° than at 0.8°. A single tool-frame constant is
appropriate after all.

### Converged, 2026-08-03 — `GRIPPER_MOUNT_TILT_* = (−1.96, −2.42)`

| iteration | applied | residual tool rotation |
|---|---|---|
| 1 | 1.53° | 1.85° |
| 2 | 3.11° | **0.63°** (0.54° of it actual tilt; 0.34° is yaw about the tool axis) |

**`joint3_to_joint2` went from needing +1.58° to needing +0.17°.** The joint that
dominated every earlier measurement is done.

**Stopped deliberately.** 0.54° is inside the ~0.6° run-to-run scatter J2's free
play produces at a single pose, so a fourth pass would fit the play rather than
the offset. It is 0.5 mm of jaw offset against ~7 mm of J1 backlash — not the
limiting error anywhere.

**Not understood, recorded rather than hidden:** the URDF tilt at which the tool
reads physically vertical grew from 2.60° to 4.53° between iterations 2 and 3 —
the apparent offset moved when the command moved, which a genuinely fixed mount
error would not do. The physical residual shrank as intended, so nothing is
blocked, but these constants are an empirical fit, not a measured geometric
constant. Re-fit rather than extrapolate if the tool assembly is disturbed.

**J1 backlash is the largest single error**: 1.75° = 7.6 mm at 250 mm reach,
bigger than the droop's Cartesian effect. It is not correctable by feedforward
*or* feedback — on a reversal the joint does not move at all (43/62 reversing
corrections stalled in test3), so there is no error signal to act on and no
deterministic offset to aim past.

**Unidirectional final approach implemented 2026-08-03**, `pick_place.py`:
`j1_unidirectional_approach()` backs J1 off 3° and re-commands the planned
target, so the last thing J1 does is always travel in `+J1_APPROACH_DIR` and
always rests against the same flank of its own slack. The lost motion becomes a
*constant* offset instead of a ±1.75° coin flip — and a constant is
calibratable.

Enabled via `unidirectional=True` on the two **pre-grasp / pre-place** moves
only. Those are where J1 makes its large swing; the Cartesian descents that
follow are near-vertical, so J1 barely turns and inherits its approach direction
from them. Putting it on a descent would inject a 3° base rotation into a move
whose entire job is not to move sideways.

It runs inside `move_arm_to` because that is the only place the *commanded*
joint target is known — re-approaching the achieved position instead would bake
in whatever backlash offset the arrival happened to leave.

**Best-effort by design:** the back-off leg is itself a reversal and is free to
stall, which is the very effect being worked around. If it does, the re-approach
is a no-op and the arm is where it would have been anyway, so a failure logs and
continues rather than aborting the pick.

**Depends on the `write_command` fix** — both legs are 3° joint moves, exactly
the class the bridge was silently discarding. Verify that first.

### All three verified on hardware 2026-08-03

**1. `write_command` fix — CONFIRMED.** `ff_verify --from-below` runs its final
approach at 0.00070 rad/cycle, **0.70× the old epsilon** — the exact case that
stalled for 60 s on 2026-08-02. It completed.

**2. The unidirectional premise — VALIDATED, by an independent route.** In
`--repeats 3` every repeat returns home first, so all three approach identically.
`joint2_to_joint1`'s spread collapsed to **0.18°** around a consistent mean of
**+0.89°**, and `joint6output`'s was **0.00** (bit-identical across all three).
That is the ±1.75° coin flip becoming a *constant* — exactly what the
unidirectional approach exists to produce, and a constant is calibratable.

The `--from-below` run is the counter-example that proves it: approaching from
below reverses every joint at the end, and **every single error came out
positive** (+0.95 / +1.38 / +1.11 / +0.74 / +0.70 / +0.95) — uniformly ~1° short.
Consistent, but consistently offset.

**3. Spread table** (`--repeats 3`, ff-on):

| joint | mean | spread | |
|---|---|---|---|
| `joint2_to_joint1` | +0.89 | 0.18 | consistent — J1 backlash as a constant |
| `joint3_to_joint2` | −0.41 | 0.09 | mean ≫ spread, real |
| `joint4_to_joint3` | −0.21 | 0.27 | noise |
| `joint5_to_joint4` | +0.04 | 0.18 | noise |
| `joint6_to_joint5` | −0.38 | 0.62 | noise |
| `joint6output` | +0.95 | 0.00 | consistent |

`joint3_to_joint2`'s −0.41° is a real signal, **but do not act on it.**
`GRIPPER_MOUNT_TILT_*` was fitted *on top of* the current feedforward, so
changing `GRAVITY_FF_COEFFS` now invalidates that fit and both would have to be
re-measured together. The two corrections are coupled from here on.

**J2 and J3 are done.** At the grasp they read bit-identical between resting and
being held physically vertical.

### Iteration 3 diverged — reverted, and this is the stopping point

Adding J4's apparently-actionable +0.62° took the constants to (−2.50, −2.73),
3.70°, and made things **worse**:

| applied | residual |
|---|---|
| 1.53° | 1.85° |
| **3.11°** | **0.63°** ← best |
| 3.70° | 1.43° — and J4 itself went +0.62° → **+2.02°** |

**Why the gate was wrong**, since the mistake is easy to repeat: J4's residual
was measured while IK **re-solves the orientation every run**, while
`ff_verify`'s 0.18° spread was measured with **fixed joint angles** commanded
directly. Those are not the same noise. Each change to the commanded orientation
makes IK redistribute the tilt across the wrist, so a residual measured at one
orientation does not predict the residual at another — J4 simply absorbed more
of the larger tilt. The iteration is only valid for steps small enough that IK
returns essentially the same solution, and 0.6° was already past that.

It was also heading for a cliff: the next step implied 5.66° against
`IK_ORI_XY_TOLERANCE`'s 5.73°, where IK may discard the request silently.

**Reverted to (−1.96, −2.42), 3.11°.** Residual 0.63°, of which 0.34° is yaw
about the tool axis and tilts nothing — so **0.54° of real tilt = 0.5 mm** of jaw
offset, against ~7 mm of J1 backlash still present. Not the limiting error
anywhere. **Stop here**; further iteration fits how IK happened to distribute
the wrist on a given run.

---

## FIRST COMPLETE PICK AND PLACE (2026-08-03)

Full `pick_place.py`, no flags. Every step succeeded — descent at
`fraction=1.00`, gripper contact detected, transit, place, release. Two real
defects surfaced.

### The mount tilt was displacing the grasp by 4.9 mm — FIXED

The jaws sit `GRASP_OFFSET_Z` = 0.09 m below the flange. `GRIPPER_MOUNT_TILT_*`
rotates the tool to hang them vertical, but **IK aims the FLANGE at (x, y, z)** —
so the tip swings on that 0.09 m lever and nothing moves the target back.
3.11° × 0.09 m = **4.9 mm**, and the measured jaw-tip error at the grasp was
**radial +4.9 mm**. The prediction to the millimetre.

Symptom on hardware: the block gripped off-centre, toward the near face.

Fixed by `mount_tilt_tip_offset()` / `compensate_for_tip_swing()`, applied in
both `make_grasp_pose` and `move_arm_to` — both, because if only one
compensated the "straight down" Cartesian descent would have to translate
sideways to reconcile them, which is the exact nudge that descent avoids. Inert
when the tilt is zero.

The **tangential** −5.3 mm is *not* from the tilt (which contributes +0.5 mm
there). That is J1 backlash, 7.6 mm, now a constant thanks to the unidirectional
approach but not yet calibrated out.

### The pick/place tilt asymmetry is PAYLOAD, not frame

The tilt corrected at the pick reappears at the place. It is **not** the
tool-frame-vs-world-frame trap `GRIPPER_MOUNT_TILT_*`'s comment warns about:
pick and place differ by only **3.04° of flange rotation**, essentially a pure
roll, and the correction lands in the same world direction at both (+83.8° vs
+87.9°).

What differs is the **block in the jaws**. Measured flange tilt: **3.12° at the
grasp, 3.94° at the place** — 0.82° more droop under load.
`SAG_PRECOMP_PAYLOAD_*` was fitted for exactly this and is currently zeroed.

The bridge's gravity feedforward cannot fix it — it has no idea whether a block
is held. `pick_place.py` does (`holding_block`), so the payload term belongs
there.

`--place-only` added: runs the grasp and parks at the place pose **still holding
the block**, so the two-echo hand measurement can be taken loaded. Every other
mode arrives empty and measures nothing about the payload.

### The grasp height was 54 mm too high — a simulator constant on hardware

The jaws closed on the block's **top inward corner** instead of straddling its
middle. The cause was not sag, not the URDF, and not the arm:

```
PICK_XYZ = (0, ZONE_RADIUS_M, 0.030 + SPAWN_HEIGHT_CORRECTION)   # = 0.065
SPAWN_HEIGHT_CORRECTION = 0.035   # "old spawn_z (0.055) - new spawn_z (0.02)"
```

`SPAWN_HEIGHT_CORRECTION` is **Gazebo bookkeeping** — the delta between two
`ros_gz_sim create -z` spawn heights. It has no meaning on hardware, where there
is no spawn offset. And the 0.030 it was added to was itself
`TABLE_TOP_Z + DEFAULT_BLOCK_SIZE/2` for a **20 mm simulated cube on a simulated
table**. Two simulator numbers stacked into a hardware target.

**The arm was never wrong.** With the sag held out by hand and the block centred
in the jaws, its centre measured **63.5 mm** above the mat against the **65 mm**
commanded — accurate to **1.5 mm**. It was being told the wrong height.

**Real geometry, measured 2026-08-03:**

| | |
|---|---|
| `MAT_SURFACE_Z` | **−0.004** — the URDF has no mesh for the 4 mm plate the robot stands on, so model z=0 floats 4 mm above the mat the blocks sit on |
| `BLOCK_HEIGHT_M` | **0.030** (1.18 in), not the 20 mm Gazebo cube |
| `PICK_XYZ.z` | `MAT_SURFACE_Z + BLOCK_HEIGHT_M/2` = **+0.011** (was 0.065) |
| `PLACE_XYZ.z` | `MAT_SURFACE_Z` = **−0.004** (was 0.075) — a *surface*, not a centre |

Both flange targets now land at **0.101 m**, fingertips **15 mm above the mat**,
straddling the block's mid-height. `DEFAULT_BLOCK_SIZE` now shares
`BLOCK_HEIGHT_M` so the pick and place conventions cannot drift apart again.

`SPAWN_HEIGHT_CORRECTION` is left defined — `annulus_test.py` still uses it and
is still correct there, because that script targets Gazebo.

**This also explains why earlier picks "worked":** grabbing a 30 mm block by its
top corner still lifts it. It was never a good grasp, just a lucky one.

### …and then `GRASP_OFFSET_Z` turned out to be 22 mm short

Stepping the corrected height in with `--pick-position` drove the gripper into
the mat at every rung — the shallowest (block centre 0.040, predicted 44 mm of
clearance) came within a couple of mm, and the two below it pressed in.

Measured from `/joint_states` with the tips on the mat: **the flange sits
114.0 mm above the mat**, so flange-to-fingertip is **~0.112 m**, against the
0.090 configured. FK independently puts `gripper_base` alone 50 mm below the
flange, and the fingers are visibly longer than the remaining 40 mm.

`GRASP_OFFSET_Z` → **0.112** (±2 mm, biased toward the safe side — a larger
offset holds the arm higher).

**Both errors pushed the same way**, which is why the first symptom looked like
a pure height problem: the target was 54 mm too high, the offset 22 mm too
short, and the jaws landed on the block's top corner.

### A floor, because nothing in the stack has one

`clamp_flange_z()` refuses any target that would bring the fingertips closer
than `MIN_TIP_CLEARANCE_M` = 5 mm to the mat, applied in both `make_grasp_pose`
and `move_arm_to`.

**MoveIt could never have caught this.** It plans against the URDF's kinematic
tree, which contains no mat, no table and no floor — a Cartesian descent into
the bench is a perfectly valid plan and reports `fraction=1.00`, which is
exactly what the logs showed while the gripper was being pressed into the mat.

The floor is derived (`MAT_SURFACE_Z + GRASP_OFFSET_Z + clearance`), so it
tracks automatically if either constant is re-measured. It does not make a
miscalibrated offset correct — it makes the failure a bad grasp instead of a
damaged gripper.

### The tool frame, finally measured rather than inferred (2026-08-03)

`report_reached()` prints the FK flange and gripper positions after every
Cartesian descent, which turned "the arm is somewhere else and we cannot tell
why" into two numbers. **The arm was never the problem** — flange radius came
out **within 0.5 mm** of commanded. Everything wrong was in the flange→jaw map.

Parked at the grasp pose, FK vs a ruler:

| | FK | measured | error |
|---|---|---|---|
| flange height above mat | 151.3 mm | — | — |
| fingertip height above mat | — | **4 mm** | `GRASP_OFFSET_Z` = **0.147**, not 0.112 |
| flange radius | 223.0 mm | — | — |
| jaw centreline radius | 226.5 mm (`gripper_base`) | **203.2 mm** | jaws hang **19.8 mm inward** |

`GRASP_OFFSET_Z` has now been 0.090 → 0.112 → **0.147**. The first two were
inferred from "the tips looked about here"; this one is subtraction against the
FK the report prints. Both earlier values were *short*, which is the direction
that drives the gripper into the bench.

`JAW_LATERAL_OFFSET = (-0.0003, -0.0198)` is new. **The URDF's tool chain is
~23 mm wrong laterally** — it places `gripper_base` 3.5 mm *outward* of the
flange where the jaws physically hang 19.8 mm *inward*. Same class of error as
the already-known 180° camera flip.

**World-frame, not radial**, because the jaws hold a fixed world yaw
(`GRIPPER_YAW_DEG`) so the tool frame has a fixed world orientation. That makes
the pick harder and the place easier. **This frame is inferred from one pose** —
measure the jaws at the place pose to settle it: 19.8 mm inward there in *world*
terms confirms it; 19.8 mm inward *radially* means the constant is the wrong
shape.

Net reach, after the mount-tilt compensation partly cancels it: pick flange
**0.2405** (inside `MAX_FLANGE_RADIUS_M` 0.245 with 4.5 mm margin), place
**0.2167**. No re-taping required.

---

## SLOW MOVES WERE NEVER SENT TO THE ARM AT ALL (found and fixed 2026-08-02)

Found on the first `ff_verify.py` run. An 8° move produced **zero motion for
60 s** while the goal was accepted normally. The robot-side log gave it away in
one line: **exactly one `TIMING dt=` entry for the entire 6 s trajectory.**

`write_command()` compared each incoming setpoint against the **previously
received** one and then overwrote it on the next line. So the test was "did the
setpoint move more than `COMMAND_CHANGE_EPSILON_RAD` = 0.001 rad *since the last
control cycle*" — and because the reference moved along with it, the difference
could never accumulate.

At ros2_control's 100 Hz, over a 6 s trajectory:

| move | per cycle | vs 0.001 epsilon | |
|---|---|---|---|
| pre-move, 141° | 0.00410 rad | 4.1× | forwarded |
| main move, 8° | 0.00023 rad | **0.2×** | **never sent** |
| a 5 cm grasp descent, ~6°/4 s | 0.00026 rad | **0.3×** | **never sent** |

The slowest move that could get through at all was **~34° in 6 s**. Anything
gentler was dropped in full — `command_dirty` never set, the loop never wrote,
the arm sat still while `state.command` walked all the way to the target.

It broke the end-of-trajectory detector too, for the same reason:
`command_changed_monotonic` is only stamped when `changed` is true, so it went
stale mid-ramp and `_serial_settle_if_needed` started correcting **1 s into a
6 s move**. That is the settle-vs-trajectory race visible in the log.

**This is the likeliest reason no grasp descent has ever completed.** A descent
is precisely the shape that was being silently discarded.

**Fixed** by comparing against the last command actually *sent*
(`state.last_sent`), so deltas accumulate. Verified by replaying JTC's setpoint
stream through the gate: the 8° move goes 0 → 120 forwarded commands, the
descent 0 → 100, **large moves are unchanged at 600** (so no regression on the
moves that already worked), and a held target still sends nothing — the epsilon
keeps doing its original job of not re-spamming an unchanged target.

**Not yet confirmed on hardware.**

---

## REACH IS A CURVE, NOT A NUMBER (found and fixed 2026-08-03)

**The pickup zone does not need to move.** An earlier version of this section
said it had to come in 1.17 in. That was wrong, and the way it was wrong is the
useful part.

### What went wrong

`zone_calibrate.py`'s pre-flight screened every waypoint against
`tag_pick_place.MAX_FLANGE_RADIUS_M = 0.245` and **rejected failures before IK
was ever called.** Two compounding errors:

1. **0.245 is the limit at ONE height.** Its own comment says where it came
   from — "the measured ceiling at `DETECT_HOVER_Z`'s predecessor (0.280)". It
   is true at a flange z of about **0.220**. At the grasp height, z ≈ 0.151, the
   real limit is **0.2793** — 34 mm further out.
2. **The screen was a gate, not advice.** Corners the arm can plainly reach were
   thrown out by a constant. The solver never got asked. "The solver must
   improve if it is throwing those corners out" was the right instinct aimed at
   the wrong component: the solver was never consulted.

### The envelope, derived and validated

The tool's approach direction depends **only** on the sum
`joint3_to_joint2 + joint4_to_joint3 + joint5_to_joint4`, and points straight
down when that sum is exactly **−π/2** (verified numerically). So the
vertical-tool workspace is a 2-DOF sweep, not 3 — pick two pitch joints inside
their URDF limits and the third is determined. Swept at 0.25°, 1.1M samples,
max `hypot(x, y)` per 3 mm band of flange z. Lives in `pick_place.py` as
`FLANGE_REACH_ENVELOPE` / `max_flange_radius(z)` / `max_flange_z(radius)`.

**It reproduces a hardware measurement.** `reach_probe.py` found on the real arm
that at radius 0.250, z = 0.210 is reachable and z = 0.215 is not. The table
gives `rmax(0.210) = 0.2523` and `rmax(0.215) = 0.2490` — the same boundary,
from the URDF alone, inside the 5 mm probe step.

| flange z | max radius |
|---|---|
| 0.150 (grasp) | **0.2793** |
| 0.190 (hover) | 0.2639 |
| 0.220 | 0.2454 ← where `MAX_FLANGE_RADIUS_M` is true |
| 0.250 | 0.2183 |

### What the zone actually looks like at 9 in

| square | far corner flange r | at grasp z | at hover |
|---|---|---|---|
| `tags` 4.00 in | 0.2956 | −16.5 mm | out |
| `span` 3.00 in | 0.2811 | −2.0 mm | out |
| **`blocks` 1.82 in** | 0.2645 | **+14.6 mm** | **+3.5 mm** |

**The block-centre square — the only one that has to be reachable — clears at
every corner.** The 3 in clear span misses by 2 mm, which is a knife edge and
does not matter because a block's centre cannot sit on that square's corner
anyway. The 4 in tag square is genuinely out by 16.5 mm, and the tags are
fiducials the arm never has to reach.

### The hover was the real constraint

Note the far corner's grasp margin (+14.6 mm) against its hover margin. The
envelope **shrinks with height**, so a hover 40 mm above a reachable grasp can
sit outside it. The descent's *starting point* fails, not the grasp — and it
fails as an IK miss at the hover, which reads like a reach problem at the block
and is not.

`hover_z(target_z, radius=None)` now clamps to `max_flange_z(radius)`. It only
ever **lowers** the hover, and omitting `radius` keeps the old behaviour. At the
far corner the descent shortens 40 mm → **31 mm**; the default pick and place
hovers are **unchanged at 0.198**, confirmed.

`hover_z_for(x, y, target_z, block_yaw_deg)` is the form callers should use — it
takes the *compensated* flange radius, ~12 mm further out than the jaw target.
Wired into `pick_place.main()` (4 sites) and `tag_pick_place.py`, where it
matters most because the grasp position comes from vision and can be anywhere in
the zone. There it is recomputed **after** `--verify` moves the target, since a
correction that pushes the block outward also moves the hover ceiling.

`MIN_USEFUL_DESCENT_M = 0.010` warns when clamping leaves essentially no
vertical approach — the jaws would come in from the side and could knock the
block. Warned, not enforced: refusing a target the arm can otherwise reach would
be worse.

### The envelope is NOT simply an upper bound — and that opens a gap

The table is computed with the tool **exactly vertical**. `solve_ik_state` gets
`IK_ORI_XY_TOLERANCE` = 0.10 rad (5.73°) of tilt, and re-running the same sweep
across that band shows it buys real reach:

| flange z | exact | with ±5.73° | gain |
|---|---|---|---|
| 0.1508 (grasp) | 0.2792 | 0.2852 | **+6.0 mm** |
| 0.1908 (hover) | 0.2637 | 0.2714 | **+7.7 mm** |

So for a **hover**, reached by `solve_ik_state`, the table is conservative by
6–8 mm. For a **grasp** it is not: `cartesian_move_to` re-imposes the exact
downward quaternion through `make_grasp_pose`.

**That leaves a band the IK pre-flight cannot catch.** A target 0–6 mm outside
exact vertical solves at the hover and then the descent has nothing to follow —
the pre-flight passes, the arm gets there, and the Cartesian path falls short.
It reads like a planner failure and is really a reach one.

`ORI_TOLERANCE_REACH_BONUS_M = 0.006`, and the pre-flight now prints a
**DESCENT RISK** line for anything in that band. It fires on the 3 in `span`
square's far corners (+2.0 mm over exact vertical) and on nothing in the `blocks`
square — which is exactly the distinction that matters.

### The rule this leaves

**The envelope is advisory. IK is the authority for the hover; exact-vertical is
the authority for the grasp.** Neither knows about `joint6output`'s limit (the
wrist absorbing the fixed world grasp yaw — see `YAW_RETRIES_DEG`),
self-collision, or convergence. The pre-flight prints the radius map as ADVISORY,
flags the descent-risk band, and gives every point to the solver regardless.

`tag_pick_place.py` still uses the flat 0.245, correctly: it clamps the vision
correction loop at the detection hover, one specific height, where one number is
fine.

---

## `zone_calibrate.py` — the zone survey

Drives the jaws to the zone centre and each of the four tag vertices, and
records where they actually ended up. Mars-side, no vision, no
`DetectBlock`. Every calibration constant in `pick_place.py` was fitted at
**one** point, the zone centre, and nothing had ever checked it 2 in away.

```bash
python3 zone_calibrate.py --dry-run                    # reach map, arm untouched
python3 zone_calibrate.py --interactive                # survey + hand measurements
python3 zone_calibrate.py --square span                # the 3 in clear area instead
python3 zone_calibrate.py --repeats 3 --out zone3.csv  # repeatability
```

**The two column families answer different questions, and mixing them wastes a
session:**

- **FK columns** (`fk_*`, `err_*`) come from `/joint_states`. They see droop,
  backlash and dead zone — the residual a feedforward removes. This is the
  column set worth turning into a lookup table.
- **Hand columns** (`meas_*`, from `--interactive`) are the *only* thing that
  can catch a wrong geometry constant. FK reports the model's opinion of where
  the tool is, and the model is what is under test. A wrong `GRASP_OFFSET_Z` is
  invisible to FK by construction.

FK error large → control problem. FK error small but the tape disagrees →
geometry constant is wrong.

The pre-flight screens twice, on radius then on real IK, and **skips** what
fails rather than attempting it: `move_arm_to`'s OMPL fallback will "succeed" at
an unreachable target by parking short and tilted inside its 4 cm position
sphere — the runaway `MAX_FLANGE_RADIUS_M` documents — which looks like a
completed move and poisons every number measured at it.

`summarise()` splits the residual the same way the droop analysis did: a
**constant** offset across the zone belongs in `DESCENT_BIAS_Z` /
`JAW_LATERAL_OFFSET`; error that **swings** with position is pose-dependent and
no single constant fixes it. Anything under 1 mm is reported as negligible
rather than modelled, since that is under both the servo dead zone and what a
tape measure resolves.

---

## `DESCENT_BIAS_Z` (2026-08-03)

The arm stops **high** of its commanded flange z even with the bridge's gravity
feedforward live. Two independent readings agreed to about a millimetre:
`report_reached` measured **+3.7 mm** of flange `dz`, and the jaws visibly
closed **~5 mm** above the block's midline.

`DESCENT_BIAS_Z = -0.007` (−0.005 first, then −0.002 more after the jaws still
read ~2 mm high by eye). Deliberately **not** folded into `GRASP_OFFSET_Z`,
which is a rigid tool dimension measured by subtraction and has no business
absorbing a control error. This is the residual free play the feedforward cannot
reach — joints 2 and 3 give up their last degree and the tool rides up.

Margins are now thin in both directions:

| | |
|---|---|
| commanded flange z at the pick | 0.1508 |
| `clamp_flange_z` floor | 0.1480 — **2.8 mm** |
| flange radius | 0.2405 vs 0.245 — **4.5 mm** |

**Any further increase hits the clamp**, and the clamp is not the thing to
relax. If the grasp still lands high past this point, `GRASP_OFFSET_Z` is what
is wrong.

---

## FIRST ZONE SURVEY — RESULTS (2026-08-03)

Five waypoints of the `blocks` square at the 9 in pickup centre, jaws open,
digital caliper on the fingertip height and two scales + a protractor on the
radius. `src/swarm_pkg/testing/zone_calibration.csv`.

### What the arm did vs what the tool did — the split the survey exists for

| | FK (arm vs its own command) | hand (tool model vs reality) |
|---|---|---|
| height | **+0.87 mm** | **+12.6 mm** |
| radius | −0.3 to −0.5 mm | **−7.3 mm** |
| tangential | **+4.61 mm** | — |

**The arm is fine.** It lands within ~1 mm of its commanded flange z and within
0.5 mm of the commanded radius. Everything large is in the *model* — which is
exactly the distinction FK cannot make on its own, and the reason the
`--interactive` columns exist.

### Four constants changed

**`GRASP_OFFSET_Z` 0.147 → 0.1345.** Tips sat higher than predicted at all five
points (+14.0, +16.6, +13.8, +9.3, +9.0 mm; mean +12.6). Cross-checked the other
way — achieved flange z minus measured tip height, per point — gives mean 0.1345.
Two reductions of the same data agreeing to 0.1 mm. This is the first value
measured *at the fingertips* rather than inferred; the history is 0.090 → 0.112 →
0.147 → **0.1345**.

**`DESCENT_BIAS_Z` −0.007 → −0.001.** It had been doing `GRASP_OFFSET_Z`'s job.
Both constants lower the commanded flange, so by eye a tool-model error and a
control error are the same thing — and the eye is what set −0.005 and then
−0.007. The survey measured them separately: the arm's residual is +0.87 mm, and
that is this constant's entire justification. The other 12.6 mm went where it
belongs. Sum check: the two need to total 0.1338 m to put the tips on the block
midline; 0.1345 − 0.001 = 0.1335. Floor margin *improves*, 2.8 → **8.8 mm**.

**`JAW_LATERAL_OFFSET[1]` −0.0198 → −0.0271.** Jaws short of target at every
point (−6.3, −6.8, −6.8, −8.3, −8.3; mean −7.3) while FK's radius error was
−0.3 to −0.5 mm. **Frame still unresolved** — every waypoint sits within ~6° of
+Y, so world-frame and radial are indistinguishable here. The place zone is the
measurement that separates them.

**`J1_RESIDUAL_BIAS_DEG` = +1.10, new.** The achieved bearing sits below the
commanded bearing by +1.02, +1.23, +0.97, +1.10, +1.16° — J1 under-travels and
stops short. `J1_UNIDIRECTIONAL_ENABLED` already guarantees it always rests on
the same flank of its slack; making it constant was the hard part, subtracting it
is arithmetic.

**As an angle, not a distance.** The same samples read 3.69–5.35 mm (36% spread)
but 0.97–1.23° (24%). The error lives at the joint, so a Cartesian correction
would only be right at the radius it was fitted at — and the zone spans
200–265 mm. Applied only inside `j1_unidirectional_approach`, because the sign
depends on the approach direction that function is what guarantees.

### What is genuinely pose-dependent

The height error is **not** uniform: near waypoints read ~+15 mm, far ones ~+9 —
7.6 mm of structure, systematic, with the arm hanging *lower* when extended.
That is post-encoder compliance: the encoder reports the commanded angle while
the link sits below it, so FK is blind to it by construction. No constant removes
it. After the mean correction the residual is roughly ±4 mm across the zone,
which a 30 mm block tolerates — this is the term a lookup table would carry.

### VERIFIED ON HARDWARE — the re-run

Same five waypoints with the four corrections live
(`zone_calibration.csv` at the repo root):

| | before | after |
|---|---|---|
| J1 tangential | **+4.61 mm** (spread 1.66) | **−0.69 mm** (spread **0.37**) |
| jaw radius vs target | **−7.29 mm** | **−1.58 mm** |
| tip height vs 15 mm target | +6.20 mm (spread 7.00) | −2.30 mm (spread 6.00) |

The J1 angular bias is the standout: the residual dropped 6.7× and its *spread*
dropped 4.5×, which is the part that says the model was the right shape. A
Cartesian correction could not have done that across a 200–272 mm span of radius.

### The whole remaining error is at the far corners

| | centre + near | far |
|---|---|---|
| tip height vs target | **0.00 mm** (spread 0.00) | **−5.75 mm** (spread 0.50) |

Three waypoints landed on the block midline exactly; the two far ones sit
5.75 mm low, and repeatably so. **The constants are done — what is left is one
pose-dependent term**, and it is now isolated and measured rather than inferred.

It is post-encoder compliance, and the encoders prove it. At the far corners FK
reports the flange *closer* to its commanded z than at the centre (+0.25 mm vs
+0.63 mm) while the tips are physically 5.75 mm lower. The joint is where it says
it is; the link is not. The arm is much straighter out there
(j2 −1.01 rad against −0.74 at the centre), so the moment arm is longest exactly
where the deflection appears.

This is the lookup table's first real data point: **flat to ~248 mm of flange
radius, −5.75 mm by 272 mm.** Two clusters is not yet a curve — `--repeats 3` and
intermediate radii would give it shape.

### Two bugs in `summarise()` that the data exposed

1. **It compared fingertip measurements against `fk_grip_*`** — the
   `gripper_base` link, ~85 mm above the fingertips — and reported the gap
   between two different points as an error ("mean −84.5 mm"). Now compared
   against the predicted fingertip, `flange z − GRASP_OFFSET_Z`.
2. **It called the `dx` residual `JAW_LATERAL_OFFSET[0]`.** It is a J1 tracking
   error, not a tool offset. Now reported as an angle, with the right knob named.

Also `HOVER_REACH_CLEARANCE_M = 0.003`: clamping the hover exactly *onto* the
envelope left the far corners +0.5 mm, and the Cartesian retreat back up to it
re-imposes exact vertical. 3 mm of radius costs ~8 mm of hover height and buys a
descent that can be reversed.

**The constants are verified; the compliance term is not corrected.** Next
measurement is the PLACE zone (`--zone place`), which settles the
`JAW_LATERAL_OFFSET` frame question: at the place centre the jaws read **9.00 in
if world-frame, 7.15 in if radial** — 47 mm apart. That survey is also
comfortably inside the envelope, every waypoint clearing by 33–94 mm with no
hover clamping at all.

---

## THE CORRECTIONS WERE BEARING-BLIND (found and fixed 2026-08-04)

The place-zone survey came back with the gripper visibly sagging and the jaws
~39 mm short of target at every waypoint. Both symptoms are one root cause, and
it is the trap `GRIPPER_MOUNT_TILT_X_DEG`'s own comment warns about.

**The jaws hold a fixed WORLD yaw, so a tool-frame correction is world-fixed.
Gravity sag is not — the arm droops outward at every bearing.** The pickup zone
is at +Y and the place zone at −Y, so a correction fitted at one lands backwards
at the other. It cancelled the sag at the pick and *added* to it at the place.

### The lateral offset: radial, not world-frame

Open since it was first fitted, and the place survey settles it. Flange-to-jaw
across ten waypoints:

| read as | pick | place |
|---|---|---|
| **radial** | −20.8 mm | **−16.9 mm** — same sign |
| world +Y | −20.0 mm | **+15.9 mm** — flips |

A world vector cannot change sign between two poses; a radial one must, in world
terms. So `JAW_LATERAL_OFFSET` (x, y) becomes `JAW_RADIAL_OFFSET_M` = −0.0271
and `JAW_TANGENTIAL_OFFSET_M` = 0.0003, resolved against the bearing.

Modelling it as world +Y pushed the place flange ~20 mm *inward* when it needed
~24 mm outward; with the ~19 mm inboard hang, that is the 39 mm shortfall.

### The tilt: two effects, applied as one

Decomposed against the bearing, the 3.11° in `GRIPPER_MOUNT_TILT_*` reads
**+3.10° radial at the pick and −3.10° at the place.** With `A` applied, `m` the
genuine tool-frame mount error (flips) and `s` the radial sag (does not), the
pick being correct gives `m + s = 3.10`, and the place residual is then `2s`.

From the held-upright measurement at five place waypoints — **4.57°, spread
2.11** — that gives **s = 2.29°** into `SAG_PRECOMP_RADIAL_DEG` (world-frame,
bearing-aware, and already built for exactly this) and **m = 0.81°** left in
`GRIPPER_MOUNT_TILT_*`, scaled to (−0.51, −0.63).

**The pick is deliberately unchanged**: 2.29 + 0.81 = 3.10 radial, exactly what
the verified configuration commanded. Only the place moves, −3.10 → +1.48, a
swing of 4.58° against the 4.57° measured. Verified: pick flange radii identical
to 0.1 mm at all five waypoints.

### `mount_tilt_tip_offset` → `tool_tip_offset`

Once `SAG_PRECOMP_*` carries part of the tilt, a tip-swing compensation that
looks only at `GRIPPER_MOUNT_TILT_*` sees half the rotation and under-compensates
by the rest. `tool_tip_offset(x, y, ...)` takes the swing from `grasp_quat_for`
itself, so there is one definition of the commanded orientation and the
compensation cannot drift out of step with it. `holding_block` is now threaded
into both, or the tip swing is computed against a different orientation than the
one commanded.

### The place zone is now the TIGHTER of the two

Fixing the sign costs reach. The radial push now goes outward at both zones, and
because the place tilt is smaller (+1.48° vs +3.10°) it swings the tip out less,
so it needs *more* lateral push — 23.6 mm against 19.6. Place far corners land at
flange radius 0.2764, 4.3 mm inside the envelope, and the hover clamps to a 4 mm
descent, tripping `MIN_USEFUL_DESCENT_M`. Blocks at the far corners of the place
zone will be set down with almost no vertical approach.

### Confidence

**The direction and the bearing dependence are beyond doubt** — a sign flip
across ten measurements in two zones is not noise. The *magnitude* of `s` is
weaker: the radius data implies a smaller physical tilt at the place (~1.7°) than
the held-upright reading (4.57°), probably because "upright" by feel overshoots,
or the radius was taken at the gripper body rather than the fingertips. `s` could
be 20–50% high. Re-survey the place zone.

---

## THE STACKED-BLOCK PLAN (agreed 2026-08-04) — START HERE FOR NEW WORK

The pick side is calibrated and verified. This is the next build. Stages 0, 1
and 3 are sound as written; stage 2 needs its Z signal chosen deliberately, and
there are two hard constraints below that shape the whole design.

### The plan

0. **Stop calibrating the place zone.** Place is now a 4 in x 4 in box that
   blocks are TOSSED into. Tolerance goes from ±2 mm to ±25 mm, which retires
   the entire place-calibration problem. Verify PICK once with blocks all around
   the pickup area, localise them with the existing code, pick them up with the
   existing path.
1. **Pan and look.** From rest `0 0 0 0 0 -45`, pan left toward +Y over the
   family `[90, d, 2d, 0, 0, -45]`. The chosen angled view is
   **`107 49 -103 0 0 135`**.
2. **Work out the +Z topology** — which blocks sit on which — from tags on the
   blocks, the zone tags, and edge information. Blocks carry a TOP tag and a
   SIDE tag; which tags are visible and where they sit relative to each other
   gives the stacking order.
3. **Go top-down** for precise X-Y, combine with the Z from stage 2, and pick.

**Constraint: stacks are at most 3 tall.**

### BUILT 2026-08-12 — `stack_blocks.py`, and two corrections to this plan

`stack_blocks.py` does stage 0's pick plus a **precise** place: survey both
zones, pick two blocks by name, stack them at the place zone centre with the
near face square to the robot. It is a driver over `tag_pick_place`'s
primitives, not a fork of `run_stage1` — see its module docstring for why.
`--selftest` runs 61 offline checks, including that a `kind: "place"` row is
inert to every existing `calibration.py` statistic. `WORKFLOW.md` has the
commands.

**Correction 1: placing at level 2 is blocked by `MAX_HOVER_Z`, not by reach.**
A level-2 release wants flange z **0.2055** against `MAX_HOVER_Z` **0.205**, so
`hover_z_for` clamps the pre-place hover *below* the release point and the
descent inverts — the arm would rise into the block it is placing. The flange
itself can reach 0.2055 at the zone radius (the envelope allows ~0.2358 at
r = 0.2316), so this is a hover-ceiling limit and it is **a different constraint
from CONSTRAINT 1 below**, which is about *picking* from level 2. This one bites
first. Two blocks (levels 0 and 1) is the default ceiling.

**Correction 2: stage 0's "stop calibrating the place zone" survives stacking,
and the reason is worth keeping.** ±25 mm was declared acceptable for a block
tossed into a box; a stack needs the second block within ~10 mm of the first,
which looks like it reopens the problem. It does not, because the stack is a
**relative** measurement: block 2 is commanded to the same world XY as block 1,
so the place zone's survey error, the jaw model at that bearing and J1's lost
motion there are all **common to both releases** and displace the whole stack
together rather than tipping it. Same-direction repeatability is 0.20 mm
(TESTS.md Test 1), so the common part cancels to well under a millimetre.

What does **not** cancel, and is therefore the real floor:

- the **per-block grasp residual** — the two blocks sit at different points in
  the pickup zone, so each is grasped with its own error and sits slightly
  differently in the jaws (~1 mm, from the 2026-08-11 six-run residual);
- the **level-0 vs level-1 droop difference**. The two releases are at flange z
  0.1455 and 0.1755 — different arm configurations, different sag.
  `DESCENT_BIAS_Z` and the −5.75 mm far-corner compliance term were both
  measured at level 0. **UNMEASURED**, and the one term that could exceed the
  first.

`stack_blocks.py`'s place park exists to measure it: `m dx dy` at the park
writes `place_open_loop_offset` on a `kind: "place"` row. No such number exists
anywhere in the history yet.

### OPEN: BEARING AND ORIENTATION — what still does not work off the +Y axis

**The pickup zone has been at bearing +90° for the life of the project.** Every
constant, gate and pose was tuned there, and moving the mat off that axis broke
things in four separate places. Two are fixed, two are open. **Test at bearing 0
until the open ones are closed.**

Evidence throughout: `logs.txt` 2026-08-12, eight runs.

#### FIXED — the fine arc inherited the coarse pitch

`EXPLORE_PITCH_DEG = -71` aims the optical axis at **7.72 in**, not the 9 in its
comment claimed. Fine as a *coarse* compromise across a 5–10 in bench; fatal when
the fine pass inherited it. The axis landed 33–41 mm short of the mat centre at
every J1, the image centre sat 80–89 mm off, the 3-tag trust radius is 72 mm, so
**every 3-tag sighting was rejected** → one surviving still → cannot solve zone
yaw → `no pickup zone`, arm never moves.

The fine arc now takes its own pitch from the coarse pass's measured radius
(`explore.refine_pitch` / `pitch_for_radius`). Framing error 18–69 mm → **under
0.2 mm** at any bench radius.

#### FIXED — the survey's wrist yaws were absolute world angles

`survey_flange_for_yaw` places the flange at `zone centre − lens offset`, and the
offset direction came from an absolute angle. So whether a still pulled the
flange *in* or shoved it *out past the mat* depended on the mat's bearing — right
only at the bearing it was tuned at. Worst-of-5 flange radius:

| mat bearing | before | after |
|---|---|---|
| 0° (N, O, H) | — | **bit-identical** |
| +90° (standard pickup) | 0.2640 | 0.2115 (−52 mm) |
| −41° | 0.2478 | 0.2200 (−28 mm) |
| −135° | 0.2711 | **0.2152 (−56 mm)** |

At −135° all five stills were refused (`[multiview] NO usable view`) while a
still at 0.2087 reached fine the same day. Framing is now bearing-invariant.

#### OPEN 1 — the fine arc is aimed at the wrong BEARING

Runs 5 and 6 still failed after the pitch fix, with framing radially correct
(−8 mm). From the sighting dump: the camera track never passes closer than
**80.6 mm** to the mat centre, and *both* zone components move together across
the arc. That is a **tangential/bearing** aiming error — the fine arc is centred
on the wrong J1 — and it is untouched by the pitch correction.

`bearing = J1 − 6.9°` is the documented relation. The coarse anchor is chosen on
tag count, so its J1 can be a frame-edge glimpse rather than the mat's true
direction. Worth testing: centre the fine arc on the tag-count-weighted mean J1,
or on the anchor's own solved bearing, rather than on the anchor's J1.

#### OPEN 2 — `refine_pitch` trusts a coarse radius that can be 53 mm wrong

My own fix, and it needs a second pass. It takes the radius from the coarse
**anchor's** origin — the estimate explore's own comment says not to trust ("at
coarse spacing the origin is expected to be poor"). Runs 6 and 8 report a place
coarse radius of **0.287 m against a true 0.234** — 53 mm out — and the fine arc
was aimed there. It survived only because the place mat shows 4 tags and gets the
144 mm gate.

Fix: take the **median** radius over the coarse sightings rather than one
anchor's, and clamp the correction to a sane delta. The credibility band is
currently 0.05–0.40 m, which is far too wide to catch this.

#### OPEN 3 — no per-zone yaw override

`--zone-yaw` fixes **both** zones. On this bench the two mats sit at different
yaws (place +89°, an angled pickup −73 to −93°), so it cannot rescue one without
corrupting the other. A single surviving 4-tag view is rejected only because one
view cannot solve yaw — a `--pickup-zone-yaw` / `--place-zone-yaw` split would
make a measured yaw an escape hatch. ~10 lines, deliberately not done before a
hardware test.

#### Also fixed: a log message that lied

`[ik] All seeds exhausted … falling back to constraint sampling` is printed by
`solve_ik_state`, which does not know what the caller will do — and
`detect_multiview` passes `allow_constraint_sampling=False`, so those stills were
**skipped**, not sampled. Every skipped survey still logged a fallback that never
happened. Cost an hour of chasing a pose that was never commanded.

#### Diagnosing the next one: `--dump-sightings`

```bash
python3 stack_blocks.py --survey-only --dump-sightings /tmp/s1.json
```

Written **before** the gate, so a survey that rejects everything still leaves its
evidence; `explore.load_sightings()` rebuilds real `Sighting` objects so
`choose()` / `fit_zone()` run on them unchanged. Added because "is there a good
solution in this data the gate threw away?" was unanswerable from printed text —
an attempt to parse it recovered **none** of a known-good run's sightings, so no
conclusion could be drawn either way. OPEN 1 above was diagnosed from the dump in
minutes.

---

### NEIGHBOURING BLOCKS — the clearance rule, and why 90° is mandatory

Built 2026-08-12 (`tag_pick_place.grasp_clearance` / `choose_jaw_axis` /
`merged_contour_reason`). Before it, **nothing looked at what was beside the
block being grasped** — `select_block` ranks on measurement agreement alone — so
two blocks in one zone meant the open jaw came down on the neighbour.

**The detection merges before the collision happens, and that is the worse bug.**

| situation | merged footprint | outcome |
|---|---|---|
| blocks **touching** | 30 × **60** mm | `MAX_BLOCK_LENGTH_M = 60` → **accepted as one block** |
| blocks 2 mm apart | 30 × 62 mm | rejected → "zone is empty" |

The touching case lands exactly on the length cap and passes as one fat block
whose centroid sits **in the seam**. The arm then descends into the gap with the
jaws straddling nothing — a successful-looking detection of a thing that is not
there. Not hypothetical: single blocks on this bench have read **34 × 44 mm**,
so the segmentation already over-reads by up to 14 mm.

Two signals, strongest first:

1. **Two different block classes' TOP tags matching one contour.** Definitive, no
   threshold. `identify_blocks` already computed this and already said "the
   contour is not one block … neither is safe to grasp" — then dropped only the
   *identity*, leaving it a fine candidate for `--any-block`. Now recorded in
   `LAST_IDENTITY_CONFLICTS` and refused.
2. **Footprint ≥ `MERGED_FOOTPRINT_M` (50 mm).** The fallback, and the only
   signal the colour path will have. The window between "one block, badly
   measured" (44 mm observed) and "two blocks, merged" (60 mm) is **16 mm wide**,
   so this threshold has 6 mm either side. Tight, and stated rather than hidden.

**The geometry, and the asymmetry the whole rule turns on.** Each finger is a
plate normal to the closing axis: along that axis it is only its *thickness*,
across it its *width*. So a neighbour on the closing axis blocks the grasp and the
same neighbour across it does not. For 30 mm blocks:

| | minimum centre separation |
|---|---|
| **along** the closing axis | **51.3 mm** |
| **across** it | **20.8 mm** |
| usable box for a block centre | **46.2 mm** across |

**51.3 mm does not fit in a 46.2 mm box.** Two 30 mm blocks can never both be
graspable along the same axis inside one 4-inch zone — so the 90° rotation is not
an optimisation, it is **mandatory**. Across the axis, 20.8 mm is below the 30 mm
at which two blocks physically touch, so any separated pair passes. The rule
reduces to one sentence: **put the jaw axis perpendicular to the line joining the
two blocks.**

Symmetry decides whether that is available at all, because **a jaw axis is a line
and repeats every 180°, not 360°**:

- **symmetry 4** — base and base+90 are *different* axes, both valid grasps. Two
  chances to dodge a neighbour. This is the case that works.
- **symmetry 2** — base+180 is the *same* axis. **No alternative orientation
  exists**; the jaws must span the short face, so the grasp is reachable or it is
  not. Most of `block_database/` is non-cubic, so this is step 2's problem.
- **symmetry 0** — `reduce_yaw` folds it to 4, so it gets both axes.

The neighbour is modelled as a **disc of its half-diagonal**, not its rectangle,
because the neighbour's *yaw* is the least trustworthy number available about it
(a 30 mm square has classified `circle`, symmetry 0, yaw discarded, in two stills
of three). Yaw-free and errs outward.

**THE THREE INPUTS ARE UNMEASURED**, and every run says so
(`JAW_GEOMETRY_MEASURED = False`). The repo records a 0.75 rad jaw span and an
inferred "~5 mm margin over a 30 mm block", and nowhere records the aperture,
finger thickness, or finger width. With the gripper at `GRIPPER_OPEN`, measure:

```
JAW_APERTURE_OPEN_M      inner face to inner face
JAW_FINGER_THICKNESS_M   one finger, ALONG the closing axis
JAW_FINGER_WIDTH_M       one finger, ACROSS it
```

Then set `JAW_GEOMETRY_MEASURED = True`. Until then a *pass* is provisional and
the 51.3 / 20.8 mm figures move with the assumption.

**Free strategy that composes with all of it:** pick the **most isolated block
first**. Removing it makes room for the next, which is why `stack_blocks` passes
the shrinking candidate list to each pick. Not yet used to *order* the picks —
the order comes from `--stack` — which is the obvious next improvement.

Not yet handled: **height**. Both blocks are 30 mm today, so any footprint
overlap is a collision. A shorter neighbour the fingers could pass over needs the
fingertip depth below the block's top face, which is the fourth unmeasured
number.

### FIXED — fusion could produce a `square` carrying symmetry 0

Found 2026-08-12 when the **first real `stack_blocks` run refused at LEVEL 0**,
reporting "its footprint measured symmetry 0, not 4". The clearance check never
ran; nothing was too close together. The block was a 30 mm cube with a decoded
TOP tag, correctly identified as `orange_cube`.

`fuse_detections` took **two independent majority votes** over one cluster:

```python
shape    = max(set(shapes), key=shapes.count)
symmetry = max(set(syms),   key=syms.count)
```

`_classify` only ever emits the pairs `(unknown,1) (circle,0) (square,4)
(rect,2)`, so per view the two agree by construction — but two separate votes
over a non-unanimous cluster need not, and `max(set(...))` breaks a tie by set
iteration order, which differs between a set of strings and a set of small ints.

**Proof it happened, straight from the log line**, with no need to know the
cluster membership:

```
[multiview] [0] zone (+22.5, -16.1) mm  yaw +0.0 deg  23.9 x 30.0 mm  square  views=3 spread=3.3 mm/0.0 deg
```

Shape `square` — and `zyaw` / `spread_yaw` are forced to `0.0` **only** in the
`if symmetry:` else-branch, so that same detection carried symmetry 0.
Self-contradictory.

The cost was not only the refusal. Its per-view yaws were **−5.8, −95.0, +85.3,
−95.0, +82.3**; folded mod 90 that is **−5.8, −5.0, −4.7, −5.0, −7.7** — agreeing
to 3°. A perfectly good yaw was discarded for want of a consistent symmetry,
which is the exact failure `promote_tagged_tops_to_square` was written to prevent
one level up. Re-running the real cluster through the fix:

| | shape | symmetry | yaw | spread |
|---|---|---|---|---|
| before | `square` | **0** | forced 0.0 | forced 0.0 |
| after | `square` | **4** | +84.8° | **0.6°** |

Fixed by voting **once**: symmetry is now derived from the fused shape through
`zone_vision.SHAPE_SYMMETRY`, the single definition of the mapping, asserted
against `_classify` in `zone_vision_selftest.test_shape_symmetry_consistency`
along with the hardware cluster itself. A shape and its symmetry can no longer
disagree.

### OPEN — the footprint classifier is the recurring disease, and this was symptom four

The fusion bug above was the trigger; the underlying cause is that **the
segmentation does not measure this block reliably**. One 30 mm cube, five stills
of the same run:

```
26.0 x 26.2  square  sym=4
27.2 x 34.1  unknown sym=1
34.7 x 37.4  circle  sym=0
35.2 x 37.6  circle  sym=0
27.0 x 27.6  square  sym=4
```

Fused: **23.9 × 30.0 mm** for a 30 × 30 block. That is a 6 mm under-read on one
axis, and the shape verdict changes three ways across stills of one scene.

This is the fourth distinct failure traced to it:

1. `circle` in 2 of 3 stills → symmetry 0 → yaw discarded → jaws driven at the
   44 mm diagonal. Patched by `promote_tagged_tops_to_square` (per view).
2. 34 × 44 mm footprints displacing the centroid ~7 mm, matching the +5 to
   +7 mm `measured_zone` seen with the block taped on the zone centre.
3. `unknown sym=1` on 216 of 481 detections in one session's log.
4. The inconsistent fused symmetry above.

Every fix so far has been a patch downstream of it. The patches are individually
justified and they are accumulating, which is the signal that the real work is in
`find_blocks` / `_classify` — thresholds `SQUARE_ASPECT_TOL 0.88`,
`CIRCLE_FILL_MAX 0.86`, `RECT_FILL_MIN 0.80`, `SQUARE_TOP_ASPECT_MIN 0.80` are
all being asked to separate classes that a 6 mm measurement error smears
together.

Two knock-ons worth knowing before that work starts:

- **`MERGED_FOOTPRINT_M` has only 6 mm either side.** The window between "one
  block, badly measured" (44 mm observed) and "two blocks, merged" (60 mm) is
  16 mm wide. Improving the footprint accuracy widens it; degrading it closes it.
- **`SQUARE_TOP_ASPECT_MIN = 0.80` nearly blocked the tag-based rescue too.** The
  fused 23.9 × 30.0 is aspect **0.797** — under the gate by 0.003. So even
  `promote_tagged_tops_to_square` applied post-fusion would have refused this
  block. The tag path is not the safety net it looks like while the footprint is
  this noisy.

`zone_view.py` runs the same `analyze()` offline against saved stills and can
draw the contour it found (`--method canny|otsu`, `--show`, `--write`,
`--summary`). No frames corpus exists on disk yet; capturing one with
`--debug-image` is the prerequisite for fixing this properly rather than
threshold-twiddling.

### CONSTRAINT 1 — stacking costs reach, and 3 tall is exactly the ceiling

The reach envelope shrinks with height (`FLANGE_REACH_ENVELOPE`) and a stacked
block is a higher grasp. Margins inside the envelope, pickup zone:

| stack level | block centre | flange z | at 9 in centre | at far corner |
|---|---|---|---|---|
| 0 (on the mat) | 15 mm | 0.1443 | +32.3 mm | **+8.2 mm** |
| 1 | 45 mm | 0.1743 | +22.6 mm | **−1.5 mm** |
| 2 (top of a 3-stack) | 75 mm | 0.2043 | **+7.5 mm** | −16.7 mm |
| 3 | 105 mm | 0.2343 | **−14.6 mm** | −38.8 mm |

**The 3-tall constraint is not arbitrary — it is the hardware limit.** A 4-stack
cannot be picked anywhere in the zone. And the ceiling is tighter than "3": the
top of a 3-stack clears by only 7.5 mm **at the zone centre**, and is out of
reach at the far corners. From level 1 upward the far corners are already gone.

Design consequence: **tall stacks must live near the zone centre.** If the demo
needs 3-stacks anywhere in the zone, the mat has to come inward — the one place
the earlier "move the zone in" idea genuinely applies.

Also: `DESCENT_BIAS_Z` and the −5.75 mm far-corner compliance term were both
measured at level 0. Higher grasps put the arm in a different configuration with
different droop, so **stacked picks need their own verification pass.** Do not
assume the level-0 calibration transfers.

### CONSTRAINT 2 — tag legibility at the angled view

`107 49 -103 0 0 135` is inside every joint limit and geometrically sensible:

```
flange        (+0.0373, +0.0954, +0.2735)   r 0.1025   277.5 mm above the mat
tool tilt     36.0 deg from vertical  (pitch sum -54)
tool axis meets the mat at r 0.2891 = 11.38 in, i.e. 63 mm past the zone centre
flange -> zone centre                 0.305 m
```

A high, pulled-in vantage looking down and outward at 36°. Both the top and side
faces of a block are visible (foreshortened x0.81 and x0.59), which is what
makes the two-tag scheme possible at all. Pointing 63 mm past the centre is
probably fine or even deliberate — the camera has a FOV and is offset from the
tool axis — but it has not been checked against a real still.

**The problem is scale.** The project's own numbers give the invariant
`px/m x distance = 551` (2466 px/m at 0.2235 m; 2891 at 0.1906). At 0.305 m that
is **1807 px/m**. `DICT_APRILTAG_36h11` is 8 modules across the black square, and
a 30 mm block face fits at most a ~22 mm tag once `QUIET_ZONE_MM = 4` is
respected:

| tag | apparent | px across | px/module |
|---|---|---|---|
| zone tag, 25.4 mm flat | 20.6 mm | 37 | 4.6 |
| block TOP tag, 22 mm | 17.8 mm | 32 | **4.0** |
| block SIDE tag, 22 mm | 12.9 mm | 23 | **2.9** |

The side tag is **not viable** at this pose and the top tag is marginal. For a
~30 px side tag the camera needs to be about **0.24 m** from the block, not
0.305 m.

Options, in the order worth trying:
- **Pull the vantage closer** and pan over more views to cover the zone.
- **A coarser dictionary for the block tags only.** The id space needed is tiny;
  a 4x4 ArUco family needs 6 modules instead of 8, cutting the pixel requirement
  by 25%. The zone tags stay 36h11.
- Bigger blocks.

**Test this before building on it**: print block tags, put a block in the zone,
run the pose, and count detections. It is a 20-minute experiment that decides
the architecture of stage 2.

### Stage 2 — prefer tag evidence over edge continuity

The two-tag idea (top tag hidden => something is on top of it) is much stronger
than the edge heuristic, and it should be primary:

- **Occlusion is not only caused by stacking.** Two blocks side by side occlude
  each other at a 36° view, so "broken edges" does not cleanly mean "underneath".
- **The edge path is already known to be unreliable at exactly this geometry.**
  See `MAX_BLOCK_LENGTH_M`: measured block size over-reads on real stills, with
  shadow at the tilted camera angle as the likely cause, unfixed. Stage 2 would
  be leaning on edge detection in the one condition it is documented to fail.

**A third signal, free and needing no new calibration: tag SCALE.** A tag on a
raised block appears larger than the mat-plane homography predicts, by
`d / (d - h)`. At a 0.22 m lens height a 30 mm block reads **~16% larger** —
about 8 px on a 50 px tag. This is the same reasoning `tag_pick_place.py`
already uses in reverse to detect that the lens is too low. Inverted, it is a
direct height readout, and it cross-checks the tag-visibility logic.

**`solvePnP` is NOT available.** There are no camera intrinsics anywhere in the
repo — `zone_vision.py` is built on a plane homography specifically so it never
needs them, and uses the image centre as the principal point. Per-tag 6DOF pose
would require a camera-calibration campaign first. The scale trick needs none.

### Stage 3 — the two-view split is load-bearing, not an optimisation

A tag above the mat plane projects to the WRONG mat-plane position. The error is
`h x tan(theta)`:

- at the 36° angled view, a 30 mm block lands **22 mm** off
- top-down, parallax is ~0 on-axis and a few mm at the zone edge

So the angled view genuinely cannot give X-Y for a raised block, and the
top-down view genuinely can. Better still: once stage 2 gives `h`, the residual
off-axis parallax at the top view is analytically correctable, so the two stages
close on each other. Keep them separate.

### Stage 1 — the pan family, and one correction

`[90, d, 2d, 0, 0, -45]` is a clean one-parameter sweep: the pitch sum is `3d`,
so the tool tilt is exactly `3d + 90`.

| d | flange r | flange z | tool tilt |
|---|---|---|---|
| −10 | 0.157 | 0.371 | 60° |
| −20 | 0.217 | 0.287 | 30° |
| −25 | 0.231 | 0.238 | 15° |
| **−30** | 0.233 | 0.189 | **0° (top-down)** |

Driving joint angles directly sidesteps IK, which is where everything fragile in
this project has lived. Worth keeping for the whole survey.

**J1 leads the bearing by ~15.7°**, so `J1 = 90` aims at bearing 74°, not 90°.
The chosen view already corrects for this (`J1 = 107` → bearing ~91°). Any other
hand-written pose must too.

### Where to start

1. **Stage 0 first, it is nearly free.** Blocks at several spots in the pickup
   zone, `tag_pick_place.py --dry-run`, then a real pick. This exercises the
   whole verified calibration through the vision path and is the baseline
   everything else is measured against. It goes through `/detect_block`, so THE
   OPEN BUG has to be cleared first.
2. **The tag-legibility experiment** above, because it decides stage 2's design.
3. Only then build the pan/survey.

---

## Offline tests (no robot, no mars launch needed)

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
python3 zone_vision_selftest.py     # synthetic geometry regression, ~1 s, expect "0 failure(s)"
python3 -m py_compile tag_pick_place.py pick_place.py zone_vision.py block_detector_node.py zone_calibrate.py
```

`zone_vision_selftest.py` catches corner-order, homography and classification
bugs against synthetic ground truth. Run it after any `zone_vision.py` change.

---

## Ground rules

- **The user does all commits.** Leave finished work as uncommitted working-tree
  changes and say what changed. Do not `git commit` or `git push`.
- **ROS logs are not evidence of motion.** `arm_group_controller` reports
  "Goal reached, success!" from elapsed time alone — there is no `constraints:`
  block in `ros2_controllers.yaml`. Confirm every grasp visually.
- **Long robot output goes in `src/swarm_pkg/src/logs/logs.txt`**, not pasted
  into chat.
- **Never drop the orientation constraint to make IK converge.** It was tried on
  hardware: OMPL swung the base 180° and reached back over the arm toward the
  electrical panel. `look_at_quat` exists to give the *right* constraint, not
  fewer of them.

---

## Immediate next steps

These belong to THE OPEN BUG (the `/detect_block` serialization failure), not to
the stacked-block plan. Stage 0 of that plan needs step 3 below working first,
since it goes through `/detect_block`.

1. Run the 5-second serialization check on the robot. Confirm or refute.
2. Clean-rebuild `swarm_interfaces` on the robot; restart Terminal 4.
3. `ros2 service call /detect_block ...` from Terminal 5 with the zone in view —
   it should return a populated reply.
4. `tag_pick_place.py --dry-run` from Terminal 5. Look at the jaws.
5. Drop `--dry-run`. **First descent ever attempted.** Watch it.


## Shift in plan 08/04/2026

Superseded in place by **"THE STACKED-BLOCK PLAN"** above, which carries the same
four stages and the 3-tall constraint verbatim, plus the reach and tag-legibility
numbers that constrain them. Kept as a heading only so the date is findable.


# 2026-08-06 — THE DAY THE OPEN-LOOP ERROR WENT FROM 26 mm TO 5 mm

The longest single session in the project. Seven distinct defects found and
fixed, the survey error measured for the first time ever, and `explore.py` and
`tag_pick_place.py` combined into one run. This section is the whole history in
the order it happened, because the order is the lesson: almost every fix was
blocked by the one before it, and two of them were *undone and redone* when a
later measurement showed the first fit had been made against a false truth.

Read "THE FIVE LESSONS" at the end if you read nothing else.


## Where the day started

`tag_pick_place.py` could survey a zone and find a block, but the grasp needed
20–27 mm of hand-typed nudge every time and nobody could say why. Twenty-four
calibration runs existed, all at one pose, and `calibration.py` had been
reporting `REPEATABLE AT ONE POSE` for days — correctly refusing to attribute
the error, because at one pose the camera model and the arm are indistinguishable.


## 1. Blur and framing (fixed, and it held)

Nine consecutive stills had sat within 2.3 mm of the measured 0.220 m focus
floor, five of them below it. Raising `DETECT_HOVER_Z` 0.240 → 0.255 and
computing a per-wrist-yaw survey flange (`survey_flange_for_yaw`) moved every
still to 0.2366–0.2426 m and put all four zone tags in the first frame of every
run.

Repeatability with the block untouched, five consecutive runs:

    radial   -0.6, -0.4, -0.5, -0.5, -0.5
    lateral  +3.5, +3.6, +3.6, +3.4, +3.5

**0.08 mm standard deviation.** The vision was never the problem after this
point, and knowing that is what made everything below attributable.

Residual, unfixed: the lens offset is yaw-dependent (−21 mm at yaw 90, −27 at
60, −31 at 120), and only one bias is learned, from the first usable still. The
yaw-60 still loses two tags and is correctly rejected by `MULTIVIEW_MIN_TAGS`
in every run. It costs a view, not accuracy.


## 2. The zone sheet was 180° out (fixed)

Stage 2a moved the block ±20 mm in four directions and every axis read back
inverted. `ZONE_CORNER_SIGNS` puts tags 1 and 2 at zone +x, and with
`--zone-yaw -179.1` zone +x points at the robot — so **tags 1 and 2 belong
nearest the robot** and 0 and 3 were there instead.

A 90° error swaps the axes; a 180° error negates both. Both were negated, which
is what identified it. `zone_vision.py:135` had already warned that the
homography residual cannot catch this: *"a mirrored fit is still a perfect fit."*


## 3. Top-face parallax (fixed, +14 % on every off-centre reading)

Eight placements at a taped 20 mm from the zone centre read back:

    22.4  22.5  23.3  23.3  22.7  22.6  22.9  22.9   ->  mean 22.82 mm

The homography maps the **mat plane**. A block's top face floats 30 mm above it,
so it projects outward from the camera's nadir by

    h / (h - t) = 0.2396 / (0.2396 - 0.030) = 1.1431

Predicted reading for a true 20 mm offset: **22.86 mm**. Measured: **22.82 mm**.
Agreement to 0.04 mm across both axes and all four directions.

`correct_top_face_parallax` now scales each still about its own nadir before
fusion. Verified: 22.86 mm reads back as 20.00, a 34.3 mm apparent footprint as
30.0.

**Why twenty-four runs never saw it:** the correction is identically zero at the
nadir, and the lens re-centring puts the nadir on the zone centre. It only
appears once the block is off-centre — 2.9 mm at 20 mm out, 6.5 mm at the zone
edge.

Cost: the per-still nadir wanders ~9.5 mm between stills, which adds ~1.2 mm of
view spread. Real, and much smaller than the effect it removes.


## 4. The block tags had no quiet zone at all (fixed)

Block tags decoded intermittently — 19 hits one session, 0 the next, at the same
distance and the same `rms`. Not focus: focus degrades monotonically, and this
flipped run to run.

The tags were being **cut flush to the black border**, removing all 3.52 mm of
white per side. AprilTag finds a tag by locating a black quad against a lighter
background; orange plastic is a mid-tone, so edge contrast collapsed and
decoding became a coin flip on the lighting.

`print_block_tags.py` prints tag + quiet zone + a 3 mm handling margin, and its
own code commented *"the arrow is drawn OUTSIDE the cut line"* — but **no cut
line was ever drawn.** The dashed rectangle is the cell (35.53 mm); the tag's
black border is 22.5 mm; the correct cut is the 29.53 mm square between them,
and nothing marked it.

Now drawn, stroked 0.25 mm outside the quiet-zone boundary so it cannot eat the
white it protects. Console output states all three sizes explicitly.


## 5. The jaw offset, calibrated three times in one day

This is the important one, and it is important because **twice we fitted it
against a truth that was an assumption.**

**Morning.** `JAW_RADIAL_OFFSET_M = -0.0271`, measured 2026-08-04 across ten
waypoints at bearings ±90°. At the pick zone's new bearing of 0° it was 26 mm
wrong: it pushed the flange 21.5 mm outboard when the truth wanted ~1 mm inboard.
Two confirmed grasps fitted **-0.0013**, and both grasps reproduced to 0.1 mm.

**Evening.** Eight grasps with the block itself taped from the base — five at
bearing +90°, three at bearing 0°:

    +Y   radial -0.0203 (sd 0.0012)   tangential +0.0001 (sd 0.0029)
    +X   radial -0.0192 (sd 0.0016)   tangential -0.0088 (sd 0.0000)

**The radial term agrees to 1.1 mm across a 90° change of bearing.** The frame
question — radial vs world-fixed vs tool-fixed, open since 2026-08-03 — is
settled, and the answer is **RADIAL**, which is what the August 4 survey said.

Final: `JAW_RADIAL_OFFSET_M = -0.0199`, `JAW_TANGENTIAL_OFFSET_M = -0.0033`.

Note where that sits: **much nearer the original -0.0271 than the -0.0013 that
replaced it.** The August 4 ruler survey was closer to right than the fit that
overturned it, because it measured against a ruler and the fit did not.

The `-0.0013` was fitted against runs that passed `--truth-block-world` as *"the
sheet is at 9 inches"* — an assumption about where a hand-placed piece of paper
landed. Taping the **block** instead of the **sheet** fixed it in one session.

Tangential is not settled: +X wants -0.0088, +Y wants 0.0000, and a radial model
cannot have both. The mean leaves 4.5 mm typical / 7.5 mm worst across all eight,
against 19.9 mm for the constants it replaced. `--jaw-tangential-offset` exists
for a zone used repeatedly.


## 6. Block yaw was being discarded, silently (fixed)

Two runs aborted at the hover because the wrist never turned to meet a rotated
block. `zone_vision.fuse_detections`:

    if symmetry:
        ... fold the yaws ...
    else:
        zyaw = 0.0          # <-- the yaw is DISCARDED, not averaged badly

A 30 mm cube's rounded corners put its fill ratio right on `CIRCLE_FILL_MAX`, so
it classifies as `circle` in some stills and `square` in others. Majority vote
hands fusion `symmetry 0`, the yaw is thrown away, grasp yaw comes out 0, and
the jaws close on the block's 44 mm diagonal instead of its 31 mm face.

The per-still yaws in those runs were `-91.0, -90.0, +88.6, -2.0` degrees.
**Reduced mod 90 that is -1.0, 0.0, -1.4, -2.0.** The contour's yaw was never
the problem. Only the fold was missing.

`promote_tagged_tops_to_square` sets symmetry 4 when a decoded **TOP** tag sits
on a contour whose footprint is square (short/long ≥ 0.80). Deliberately *not*
keyed on the class name — a TOP tag proves the mat-parallel face, aspect ratio
proves that face is square, and four-fold symmetry follows from geometry rather
than from what the block is called.

Verified against the exact failing data: fused yaw `+0.0, symmetry 0` becomes
`+88.9, symmetry 4` → grasp yaw −1.1°. On hardware the next run picked a block
rotated 40° (`grasp yaw -40.3 deg, symmetry 4`).

`block_coordinates.py:29-36` predicted this failure verbatim, months earlier.


## 7. THE SURVEY ERROR — 28 mm, and why nothing could see it

`explore.py` produced the zone origin independently for the first time. Against
taped truth:

    zone     surveyed r   taped r    radial error   bearing
    pickup     232.5 mm   203.2 mm     +29.3 mm       -1 deg
    place      256.3 mm   228.6 mm     +27.7 mm      +90 deg

Two zones, **91° apart in bearing, 26 mm apart in radius, agreeing to 1.6 mm**,
tangential −2.1 and −1.4 mm — i.e. zero. One constant, applied radially.

**`ORIGIN_RADIAL_BIAS_M = -0.0285`.** Both zones then land within **0.8 mm** of
taped truth.

**Why the fit could not catch it.** `fit_zone`'s residual measures the eleven
views *against each other*. A bias every view shares is invisible to it. Both
zones fitted at 3.8 and 4.0 mm residual **while sitting 28 mm out.** This is the
cleanest demonstration the project has produced that *internal agreement is not
accuracy.*

Probable cause: `zone_origin_from` builds the origin out of `camera_zx/zy`, the
one quantity that depends on the principal-point assumption — the same reading
`tag_pick_place`'s lens re-centring measures at 20–22 mm every run and nulls
before it takes its stills. `explore` has no such reference and inherits it
whole. That is a hypothesis; the 28 mm is a measurement.

n=2. Both sign and frame are unambiguous and the two agree closely, but a third
bearing should be run before it is treated as settled. `--origin-radial-bias 0`
restores the raw fit.

**`survey_error` had read `0.0 ± 0.0` for the life of the project** because every
run passed the same number as both `--zone-origin` and `--truth-block-world` —
asserting the survey was right rather than checking it. This was the first
measurement of it.


## 8. Instrumentation defects found and fixed the same day

- **`--survey-only` was a silent no-op.** The flag parsed; the behaviour block's
  edit anchor no longer existed. Five runs were taken believing nothing was
  grasped; the history said `grasped: True`. Fixed and verified by AST-walking
  `run_stage1`.
- **`flange_fk` recorded the PLACE pose, not the grasp.** `LAST_FLANGE_FK` is
  overwritten by every later Cartesian move; `jaw_offset()` reported a 213 mm
  offset. `descend()` now snapshots at the grasp.
- **`LAST_FLANGE_FK` was only written by `cartesian_move_to`.** Under
  `--ik-descent` the grasp goes through `move_arm_to` and the snapshot would
  copy a stale pose — a *hover*, which is a plausible-looking 40 mm error that
  `jaw_offset()` would consume without complaint. `record_flange_fk()` is now
  called from both planners.
- **`verdict()` over-claimed "consistent across 4 distinct poses"** when all five
  runs were at one pose; the pose counter was counting rounding noise in
  `grasp_yaw_deg`. Fixed with `pose_spread()`.
- **Units bug**: metres printed into a `%+.1f mm` format in the arm line.
- **`save_calibration` recorded nothing without a `--truth-block-*`.** Two full
  pick-and-place runs produced **zero rows**. The nudge does not need a truth —
  it is the operator measuring the residual at the grasp — and it was going in
  the bin. Now every run records.


## 9. `explore_pick_place.py` — the combined run

    python3 explore_pick_place.py

One process: sweeps J1 detecting **both** zones at each stop, solves each
centre and yaw, hands the pickup zone to `tag_pick_place.run_stage1` and the
place zone's centre to its release step.

- `explore.sweep_both()` — one arm sweep, both zones per stop. Arm motion
  dominates (1.6 s move + 1.2 s settle), so a second detect at a stop already
  paid for is nearly free. Two sweeps would cost double for the same information.
- Coarse 10° pass finds each square, then a fine 2.5° arc over each. ~50 stops,
  3–4 minutes, against 15+ for a fine sweep everywhere.
- `--place-origin X Y` on `tag_pick_place` releases at the surveyed place centre.
  Z deliberately still comes from `PLACE_XYZ.z` — a hand-tuned release height for
  that pose, not geometry.
- Refuses to pick if the place zone was not found: it would end the run with the
  block held in the jaws.

**Result, second full run:** pickup surveyed 8.04 in against 8.00 taped (+0.4 mm),
place 8.97 against 9.00 (−0.8 mm), block at zone (−15.8, +16.0) mm rotated 40°,
found, picked, placed. Nudge **5.8 mm**, down from 27.


## THE FIVE LESSONS

**1. This is a modelling problem, not a control problem.** The `[reached]` line
says `flange within 5 mm of command: the ARM is fine` — the joint loops are
closed and converged. PID is already there and cannot see this error; MPC tracks
the same wrong target more elegantly; an observer needs a sensor observing the
quantity, and the encoders cannot see model error by construction. What was
wrong all day was the **map from joint angles to real-world position**, which is kinematic calibration. Conflating the two is how a month gets lost.

**2. Diverse data beats more data, and it is not close.**

    24 runs at one pose  ->  told us nothing about pose dependence
     2 runs at two bearings  ->  settled the jaw-offset frame
     2 zones at two bearings ->  settled the 28 mm survey bias to 1.6 mm

Samples only help along the axis you are trying to separate. Ten more runs at
one pose teach nothing.

**3. Two poses 180° apart are one measurement, not two.** At +Y and −Y a
world-fixed offset gives equal and opposite radial readings that average to
zero, and a radial offset gives identical ones. Pooling them destroys the signal
— and pooling is the natural thing to do. This is the mirror image of the
2026-08-04 error, where ±Y agreeing on −20.8 and −16.9 was read as proof of a
radial offset. `jaw_offset_report()` now buckets by bearing and never pools.

**4. Internal agreement is not accuracy.** Eleven views agreeing to 3.8 mm about
a centre 28 mm out. A residual measures the views against each other and is
structurally blind to any bias they share.

**5. Truth must be measured, not asserted.** Every calibration that had to be
redone was redone because its truth column was *"the sheet is at 9 inches"* —
a statement about intent, not a measurement. Taping the **block** instead of the
**sheet** settled in one session what three previous calibrations got wrong.
`--truth-block-world` is a measurement or it is worthless.


## Error budget, start of day to end

| term | morning | evening |
|---|---|---|
| vision, block vs zone centre | ~5 mm, +14 % scale on offsets | 0.08 mm repeatability, scale corrected |
| survey, zone centre vs world | unmeasured, actually 28 mm | 0.4–0.8 mm |
| jaw offset | 26 mm wrong at bearing 0 | 4.5 mm typical, 7.5 worst |
| block yaw | discarded ~half the time | held, symmetry 4 |
| **operator nudge to grasp** | **20–27 mm** | **5.8 mm** |


## Open, in priority order

1. **`ORIGIN_RADIAL_BIAS_M` is n=2.** The `--position A..K` sweep in
   `explore_pick_place.py` spans 160–229 mm of radius and −113° to +18° of
   bearing to test it. Bearings +20° to +90° are **not covered** by that set.
2. **Tangential jaw offset is bearing-dependent** (−0.0088 at +X, 0.0000 at +Y).
3. **Harvest camera-vs-FK pairs.** `camera at zone (x, y)` is an absolute
   position measurement of the lens, independent of the joint model. Every
   explore sweep already collects 11 per zone, for free, and `fit_zone` discards
   them after solving the origin. This is the cheapest calibration data
   available and none of it is kept.
4. **Per-still lens bias** — one bias learned from the first usable still is
   applied to stills whose true offset differs by 10 mm.
5. **Place is open-loop** at the surveyed centre, by design and by requirement.


# 2026-08-07 → 2026-08-11 — THE ERROR WENT FROM 5.8 mm TO 0.6 mm RMS

**Full write-up: `CALIBRATION_2026-08-11.md`.** This section is the short form
and the pointers; that file has the tables, the mistakes, and the commands.

## Result

| term | 2026-08-06 evening | 2026-08-11 evening |
|---|---|---|
| operator nudge to grasp | 5.8 mm | **0.33 mm mean, 0.58 mm RMS, 1.0 mm worst** |
| verified across | one pose | 3 reaches (126–225 mm), 2 bearings (91° apart), 2 wrist yaws |
| measured with | eyeball at a 40 mm hover | caliper at an 8 mm park, repeatable to 0.075 mm |

**The 1–2 mm target is met.** Working envelope is reach **121–222 mm
(4.8–8.7 in)**; 4 in and 10 in both fail at the bench, bracketing the fitted
span independently.

## The three things that were actually wrong

1. **J1 lost motion was never live on the pick path.** `move_arm_to` defaults
   `unidirectional=False` and no `tag_pick_place.py` call site passed it, so the
   validated `J1_RESIDUAL_BIAS_DEG = 1.10` never ran on the code that picks
   blocks — only `zone_calibrate.py`, which is why the survey that *measured* it
   saw it work. Full backlash **1.88°**, which is 4.1 mm of arc at r=126 and
   7.5 mm at r=229, and it is paid **once** on the first correction of each pose.
2. **`JAW_TANGENTIAL_OFFSET_M` is a line in reach, not a constant.** +6.0 / +3.0
   / 0.0 mm at r = 121 / 173 / 222, linear to 0.07 mm. The "4 mm of irreducible
   scatter" reported on 2026-08-06 was mostly this line sampled at scattered
   radii — signal, not noise.
3. **The last 4 mm is TOOL-fixed and no pose-frame constant can express it.**
   Rotating the wrist 90° rotated the residual 90° *in the world*: `(0,+4)`
   became `(−4,0)`. `JAW_RADIAL/TANGENTIAL_OFFSET_M` resolve against the
   **bearing**, so they can never hold it — which is exactly why they kept
   moving. New term `JAW_PERP_OFFSET_M = -0.004`, applied against the
   jaw-perpendicular `p_hat`.

## Constants as of 2026-08-11

```python
# pick_place.py -- POSE frame, resolved against the BEARING to (x, y)
JAW_RADIAL_OFFSET_M          = -0.0199
JAW_TANGENTIAL_OFFSET_M      =  0.00368     # base, at zero reach
JAW_TANGENTIAL_PER_M_REACH   = -0.05930     # per metre of hypot(x, y)
JAW_TANGENTIAL_REACH_RANGE_M = (0.121, 0.222)

# pick_place.py -- TOOL frame, rotates with the commanded WRIST YAW
JAW_PERP_OFFSET_M            = -0.004       # perpendicular to jaw closing axis

# explore.py
ORIGIN_RADIAL_BIAS_M         = -0.0272      # was -0.0285; purely radial, purely constant
```

`ORIGIN_RADIAL_BIAS_M` closed the question reopened three times in August: at
n=15 the constant is measured at 4.0 se while **every** shape term (`kx`, `ky`,
`kr`) is zero within its own standard error. The simple radial model was right.

## New tooling — read this before running a calibration

- **`m dx dy`** at the confirm prompt records a caliper reading and moves
  nothing. Lands in the row as `open_loop_offset`. **This, not `nudge`, is the
  error measurement** — a nudge is a control action and is contaminated by the
  dead band. Sign is pinned: same numbers you would type as a nudge, so **a fit
  must negate it**.
- **`--force-grasp-yaw DEG`** pins the wrist so the jaw axis lies on a world
  axis, which is what makes a caliper gap mean anything. **Calibration only** —
  with it on the arm will not orient to the block. If the arm hovers dead centre
  but does not rotate, this flag is on.
- **`nudge_steps`** records each nudge separately, so the dead band and the real
  error stay separable.
- **`calibration.py --fit --zone pickup`** is the default now. Place rows are
  rank deficient on their own (the place zone never moves; radius constant at
  229 mm) and pooling them put `kr` at −157 ± 32 mm/m — "measured" at 4.9 se and
  pure artifact.
- **Nudge in ONE step, never several small ones.** Each reversal donates up to a
  full backlash.

## Two operational traps found on 2026-08-11

- **A zone whose tag 1 is not detected caps its trust radius at 72 mm** instead
  of 144 mm — `MAX_CENTRE_OFFSET_HALF_DIAGONALS = {4: 2.0, 3: 1.0}`. With the
  pickup mat adjacent to the place mat, the fine pass centres on the place zone
  and leaves pickup 111–124 mm off image centre, so every sighting is rejected
  and the run dies with `no pickup zone, so there is nothing to pick`.
- **`--no-truth-block-on-centre` is an `explore_pick_place.py` flag**, not a
  `tag_pick_place.py` one. `--truth-block-zone` is **millimetres**;
  `--truth-block-world` is **metres**.

## Open, in priority order (supersedes the 2026-08-06 list)

1. **Q2, the in-zone sweep — untouched, and it is half the original scope.**
   Top-face parallax is 6.5 mm at the zone edge, corrected in code but never
   tested off-centre since the correction landed.
2. **Pickup mat tag 1** — reprint or clean it. Also: two separate tag squares
   currently carry the place ids 4–7, 6.4 mm apart. Check for a stray mat.
3. **The radial axis is not fitted.** Caliper-grade radial reads ~0 at all three
   reaches, so it may need nothing, but that is not established.
4. **The survey's tangential error is real and pose-organised**, ~2.3 mm beyond
   isotropic taping error. Radial sd 1.08/1.13 mm vs tangential 3.56/2.60 in two
   bearing groups — the noise stays tangential when the frames swap. Not yaw
   (dYaw sd 0.47°). Does not limit the grasp today; will limit any autonomous
   no-nudge run.
5. **`J1_RESIDUAL_BIAS_DEG` 1.10 → 0.85** — over-corrects by +0.25°, confirmed
   at two radii. Worth ~0.9 mm tangential. Held back deliberately: one change at
   a time, and it was not needed to hit target.
6. **The robot was physically replaced 2026-08-11.** The dead band and sag were
   measured on the previous unit. It appears to transfer, but if a nudge ever
   under-delivers unexpectedly, re-measure the dead band first.
7. **Harvest camera-vs-FK pairs** — still the cheapest unused calibration data
   in the project (carried over from 2026-08-06).

## A sixth lesson, earned twice this week

**A term that reproduces one pose exactly can still be inverted.** A sign error
in the tangential line was shipped and survived a verification run at H, because
H's residual was already zero and its required offset is identical on either sign
convention. Only a pose with a **non-zero** residual tests a sign. The same shape
of mistake produced "the tool frame is ruled out", asserted from one pose read by
eye and wrong — one pose cannot separate two frames that coincide there.


# 2026-08-12 — THE FIRST TWO-BLOCK STACK, AND WHAT IT COST

`stack_blocks.py` surveyed both zones, picked the orange cube, placed it at the
centre of the place zone, went back, picked the green cube and set it on top.
Level 0 landed dead centre; level 1 landed square on level 0. **Zero nudges on
either block** — `pick_nudge_steps` and `place_nudge_steps` are empty on both
rows, so the whole thing was open loop.

Rows **217 and 218** of `calibration_history.jsonl`, `kind: "place"`,
`grasped: true / released: true / stacked: true` on each.

## The numbers

| | level 0 (orange) | level 1 (green) |
|---|---|---|
| pick world | (0.18948, −0.02007) | (0.21859, +0.02563) |
| pick bearing / radius | −6.05° / 190.5 mm | +6.69° / 220.1 mm |
| pick in zone | (+23.6, −17.9) mm | (−22.7, +10.4) mm |
| views fused / spread | **2** / 1.63 mm | **2** / 1.84 mm |
| grasp yaw commanded | −6.21° | +29.33° |
| flange FK z at grasp | 0.14503 (**−0.47 mm**) | 0.14438 (**−1.12 mm**) |
| release flange z | 0.1455 | 0.1755 (+30.0 mm) |
| place XY commanded | (0.009491, 0.231756) | **identical, to the last digit** |

Three things worth keeping out of that table:

1. **The common-mode cancellation argument held.** Both levels were commanded to
   the same place XY because the place target is the surveyed origin, computed
   once. The two blocks were grasped 54.2 mm apart in the zone, at bearings 12.7°
   apart and radii 30 mm apart, so their grasp residuals were genuinely
   independent — and the stack still came out square. That is the prediction in
   `stack_blocks.py` design decision 4, tested rather than argued.
2. **Two views, not five.** `MULTIVIEW_YAW_OFFSETS_DEG` takes five stills;
   `fuse_detections` got two usable detections out of them on both blocks. It
   worked anyway, but the per-run vision noise budget assumes 3–5 views, and at
   n=2 there is no median to take and no outlier to reject. Same disease as
   below.
3. **The descent lands 0.5–1.1 mm short in z, and not by the same amount twice.**
   0.65 mm of difference between two grasps 30 mm apart in radius. Absorbed by the
   gripper's compliance today. It is the only hint on the ledger about level-0 vs
   level-1 droop, which is still unmeasured.

## What went wrong: the transit height, and it is a separate concept

Carrying block 1 to the place zone, **the carried block struck block 2 and moved
it**. Not a planning failure, not a clearance-check failure — the clearance check
is about the open jaws at the grasp and it passed correctly. Pure arithmetic:

```
retreat after grasp   = hover_z_for(...) = grasp_z + APPROACH_HEIGHT
                      = 0.1455 + 0.040          = 0.1855
carried block bottom  = 0.1855 - GRASP_OFFSET_Z - h/2   = 0.0360
a block on the mat    = MAT_SURFACE_Z + h               = 0.0260
                                             clearance  =  10.0 mm
```

and the recorded bearings put the sweep directly over it: block 1 at −6.1°,
block 2 at **+6.7°**, place zone at +87.7°, so 94° of J1 rotation passes over
block 2 at 10 mm. The sweep is an unconstrained OMPL free-space plan — nothing
holds z between the endpoints, and with no response adapters there is not even a
time profile to reason about. The endpoint heights are the only lever.

**`APPROACH_HEIGHT` was never a transit clearance.** It is sized for the *descent*
— long enough to arrive vertically, short enough to stay inside the reach
envelope. A 30 mm block eats three quarters of it. Two different jobs sharing one
constant, which is the shape of most of the bugs in this file.

### Fixed

`stack_blocks.py`: `TRANSIT_CLEARANCE_M = 0.025`, `transit_flange_z()`,
`traverse()`, `obstacle_top_z()`, `--transit-clearance-mm`. Every cross-zone move
now goes: **straight-up Cartesian lift** (vertical, so it cannot sweep through
anything) → **long planned sweep with both endpoints raised** → the existing
approach. The return leg is raised too: the jaws come back over the stack they
just built, and the release retreat leaves the arm at the place hover.

The clearance is computed from the load, not the flange:
`obstacle_top + clearance + h/2 + GRASP_OFFSET_Z`, which is the identity
**`transit over an n-block pile == release_flange_z(n) + clearance`** — asserted
in the selftest, so the two height derivations cannot drift apart silently. The
empty-gripper case deliberately reuses the loaded formula: the fingertips sit
~20 mm *higher* than a carried block's bottom face, so loaded is the conservative
one and the return leg needs no second number — and no dependence on
flange-to-fingertip, which is still unmeasured.

Default gives **25.0 mm**, up from 10.0. Clamped, not asserted: a failed lift
degrades to today's behaviour loudly rather than stopping a working run.

### The ceiling is tight, and it is the same ceiling as level 2

`MAX_HOVER_Z = 0.205` allows **29.5 mm** of transit clearance over a one-block
pile and **nothing at all** over a two-block one. So a three-high stack is
blocked twice over — the place hover inverts (design decision 3) *and* the arm
cannot fly the third block in. One measured ceiling, two symptoms. Raising it
means re-running `reach_probe.py`, not editing the constant.

### OPEN — the level arc, which would remove the failure mode rather than derate it

Raising both endpoints does not *guarantee* a level path, it only makes the dip
that would have to happen a much larger one. The guarantee is available and
cheap: **J1 alone rotates the flange on a horizontal circle at constant z and
constant radius**, so a pure J1 joint goal is an exactly level arc. Take the joint
state at the lifted pick pose, change only J1 to the destination bearing, send it
as a joint goal via `pp.make_joint_goal_constraints` — the machinery
`PoseMemory.replay` already uses — then fix the radius with a second move. Two
moves, both with a geometric guarantee, and a single-waypoint goal each, which is
what this JTC bridge actually executes.

Not done today because it puts a new motion primitive on the hardware, and the
height raise addresses what was actually knocked over.

## Open, in priority order — reordered 2026-08-12, and the reason matters

The 2026-08-11 list is a **calibration** list. Calibration is no longer the
bottleneck: the grasp is at 0.33 mm mean / 0.58 mm RMS and a fully open-loop
two-block stack just succeeded with no operator correction. **Perception is the
bottleneck**, and the next step (a wider block set, identified by colour and
shape, with no AprilTags) is entirely perception. So:

1. **The footprint classifier.** Fourth distinct failure traced to it this week.
   One 30 mm cube read 26×26 / 27×34 / 34×37 / 35×37 / 27×27 mm across five
   stills of one run and fused to 23.9×30.0. It has already produced: the
   `circle`/diagonal-grasp bug, the 34×44 centroid displacement, 216
   `unknown sym=1` detections in a session, and the fusion inconsistency fixed
   today. Every fix so far has been downstream of it. **Prerequisite: a frames
   corpus on disk** (`--debug-image`, then replay offline). None exists.
   Without this, nothing built on colour blobs can be trusted, because the
   AprilTag rescue path is what has been quietly covering for it.
2. **Measure the four jaw numbers** — aperture at `GRIPPER_OPEN`, finger
   thickness, finger width, fingertip depth below the block top face — then set
   `JAW_GEOMETRY_MEASURED = True`. Now blocking rather than merely untidy: the
   aperture decides *which blocks in the new set can be grasped at all* (see
   below), and it is a two-minute caliper job.
3. **Two views is not five, and the log already names the mechanism.** On the
   successful run, 4 of the 5 stills printed
   `only 2 tag(s); 3 needed to VOTE on a block position` and were dropped by
   `MULTIVIEW_MIN_TAGS = 3`. Separately, 3 of the 5 failed IK at
   `DETECT_HOVER_Z = 0.255` (flange radii 0.2046–0.2060 with the `look_at_quat`
   tilt) and fell back to 0.240, below the 220 mm focus floor — so those stills
   were soft as well as tag-poor. Note the two are independent: the height
   fallback still produced a homography; it was the **tag count** that
   disqualified the views.

   Why only two of four zone tags are visible per still is NOT in the log — the
   candidates are framing (the lens measured 14.2 mm off the zone centre) and
   gripper occlusion, and they need looking at, not guessing. Same instrument as
   (1): run a survey with `--debug-image` and look at the frames. Fixing this is
   the cheapest available reduction in fusion noise, because it is the difference
   between fusing 2 views and fusing 5, and at n=2 there is no median to take and
   no outlier to reject.

   Also: put a floor under `views` before a grasp. Today a `views=1` detection
   with `spread=0.0` is printed with a warning and is otherwise trusted.
4. **The level arc** (above). Cheap, and it converts a derated failure mode into
   an eliminated one.
5. **Pickup mat tag 1** — reprint or clean it; and two tag squares currently
   carry the place ids 4–7 6.4 mm apart, so check for a stray mat. Unchanged
   from 2026-08-11, still a five-minute job, still capping a zone's trust radius
   at 72 mm instead of 144 when it bites.
6. **`J1_RESIDUAL_BIAS_DEG` 1.10 → 0.85.** Over-corrects by +0.25°, confirmed at
   two radii, worth ~0.9 mm tangential. One constant, already measured.
7. **Q2, the in-zone sweep.** Top-face parallax is 6.5 mm at the zone edge,
   corrected in code, never tested off-centre since the correction landed. Was #1
   on the 2026-08-11 list. Demoted because both blocks today sat 22–29 mm off
   centre and were grasped without a nudge — which is weak evidence the
   correction works, not proof, but it is no longer the thing most likely to
   break the next run.
8. **The survey's tangential error**, ~2.3 mm beyond isotropic taping error and
   pose-organised (radial sd 1.08/1.13 mm vs tangential 3.56/2.60 in two bearing
   groups; not yaw, dYaw sd 0.47°). Does not limit the grasp today. Will limit
   any autonomous no-nudge run — and today's run *was* one, so this is rising.
9. **The radial axis is not fitted.** Caliper-grade radial reads ~0 at all three
   reaches, so it may need nothing; not established.
10. **Harvest camera-vs-FK pairs.** Still the cheapest unused calibration data in
    the project. Carried over from 2026-08-06, twice.
11. **The robot was physically replaced 2026-08-11.** A contingency, not a task:
    if a nudge ever under-delivers unexpectedly, re-measure the dead band first.


# 2026-08-12 — STEP 2 DESIGN: THE WIDER BLOCK SET WITHOUT APRILTAGS

Decisions and arithmetic settled before any of step 2 is built. `object_detection.py`
(contributed) is assessed at the end.

## The one inequality that governs the whole block set

`MAX_HOVER_Z = 0.205` is a measured ceiling (`reach_probe.py`, 2026-07-27: 0.215
is outside the workspace at *any* orientation). Everything vertical resolves
against it. Write `g` for the **grasp height above the block's own base** — the
height of the point in the block that the pads clamp — and `c` for the transit
clearance:

```
transit flange z  =  obstacle_top + c + g + GRASP_OFFSET_Z  <=  MAX_HOVER_Z
                  =  0.026        + c + g + 0.1345          <=  0.205

                             g + c  <=  44.5 mm
```

That is the budget, for a traverse over one 30 mm block. Today: `g = 15`,
`c = 25`, total 40. Before today's fix: `g = 15`, `c = 10`, total 25 — 19.5 mm of
the budget simply unspent.

Three consequences, and they decide the block set:

1. **Grip LOW. Every millimetre of grasp height is a millimetre of transit
   clearance given up.** This is the direct answer to "grasp at a fixed depth
   below the top face": *no* — grasp at a fixed height above the **base**, as low
   as the pads allow.
2. **A block gripped near its top cannot be transported at all.** The 61 mm
   pyramid gripped 15 mm below its apex has `g = 46`, so its transit flange z is
   `0.026 + 0.025 + 0.046 + 0.1345 = 0.2315` — 26.5 mm above the ceiling. The
   load hangs below the pads, and the ceiling is on the flange. Gripped 15 mm off
   its base instead, the same block transits at 0.2005.
3. The pre-grasp hover has its own, looser limit: `g <= 34.5 mm` before
   `hover_z_for` starts clamping the 40 mm approach. **Transit binds first**, at
   19.5 mm for `c = 25`. Nothing above `g = 34.5` has a vertical approach either.

For the current cube, "fixed depth below the top", "mid-height" and "as low as
the pads allow" all evaluate to 15 mm. That coincidence is why the question has
had no visible answer so far.

### What the rule cannot be until one number is measured

`g` is bounded below by the pads hitting the mat and above by the pads leaving the
side face, both set by the **vertical extent of the finger pad**, which is
unmeasured (`JAW_GEOMETRY_MEASURED = False`). The rule is:

```
g = clamp( G_MIN, G_MIN, H - PAD_HALF_HEIGHT )      # prefer the floor
```

with a refusal when `G_MIN > H - PAD_HALF_HEIGHT`, i.e. the block is shorter than
the pads can grip.

There is a hint in the constants worth chasing with the caliper. `GRASP_OFFSET_Z`
is 0.1345 flange-to-pad-centre, while `gripper_offset_probe.py` put the
fingertips 0.114 below the flange — so the tips sit **20.5 mm above the pad
centre**, which for a 30 mm cube gripped at its middle puts them 5.5 mm above the
block's top face. Those two numbers cannot both mean what their comments say. The
reading that makes them consistent is that the pad band is tall (~30 mm) and
today's cube is gripped across essentially its whole face — which would predict
that the **0.6 in (15.2 mm) slabs in this set are ungrippable**, and that is a
prediction a caliper settles in two minutes.

## Rest poses, not grasp heights

Yes to "valid poses per block" — but the table stores **rest poses** and *derives*
the grasp geometry. 18 blocks x up to 3 poses is ~40 hand-entered grasp heights,
i.e. ~40 chances to typo a number that drives the jaws at the mat.

Store per block: colour, shape family, the three dimensions.
Derive per rest pose: footprint W x D, standing height H, top-face outline and its
symmetry, the candidate jaw axes, `g`, the pre-grasp hover, the transit height.

Pose counts fall out of the dimensions: a cuboid with three distinct dimensions
has **3** rest-pose classes, with two equal **2**, a cube **1**. So yes — the
2.4 x 1.2 x 1.2 green brick has two: lying (footprint 61 x 30.5, H 30.5) and
standing on end (footprint 30.5 x 30.5, H 61). The 2.4 x 1.2 x 0.6 blue slab has
three.

### The aperture prunes the set harder than the vision does

`JAW_APERTURE_OPEN_M = 0.040` — **also unmeasured**. Against it, every dimension
in the set, in mm:

| in | 0.6 | 0.8 | 1.0 | 1.2 | 1.4 | 1.6 | 1.7 | 2.2 | 2.4 |
|---|---|---|---|---|---|---|---|---|---|
| mm | 15.2 | 20.3 | 25.4 | 30.5 | 35.6 | 40.6 | 43.2 | 55.9 | 61.0 |
| grasp? | yes | yes | yes | yes | tight | **no** | **no** | **no** | **no** |

So **every grasp in this set is across a 1.4 in dimension or smaller**, and the
grasp axis is forced for most blocks rather than chosen. Consequences worth having
before building anything:

- **The pink disc (2.2 in dia x 0.8 in) is ungraspable in its stable pose.** Lying
  flat, every horizontal chord through its centre is 55.9 mm. Its one graspable
  dimension, 20.3 mm, is vertical when it is lying down, and a disc on edge is not
  a stable rest pose.
- **The white sphere** is 30.5 mm and fits, but a sphere gives two tangent-point
  contacts and will squirt out of a parallel-jaw squeeze.
- **The cone, both pyramids and the frustum** taper, so the pads land on a slope
  and the clamp drives the block *up and out*. Grip as low as possible or not at
  all.
- **The red rhombic prism (1.6 in) and the green cylinder (1.7 in)** need the
  aperture measured before they are in or out.

**Measuring the jaw aperture is now the highest-value two minutes available**, and
that is why it moved to #2 on the open list. It decides the scope of the block set
before a line of the classifier is written.

## Telling a top face from a slanted side

The honest first answer: **from directly overhead you do not have to.** For a
right prism at zero offset from the lens nadir, no side wall is visible at all and
the silhouette *is* the top face. The problem is entirely one of the block being
off-axis.

### The size of it, derived

Lens height `h` above the mat, block height `H`, block centre offset `r` from the
nadir, footprint half-width `a`. The silhouette in the mat plane is the union of
the base outline, spanning `[r-a, r+a]`, and the top outline magnified about the
nadir by `m = h/(h-H)`, spanning `[(r-a)m, (r+a)m]`.

- For `r < a` — the block straddles the nadir — the magnified top contains the
  base, the union is the top face alone, and `correct_top_face_parallax` is
  **exact**.
- For `r > a` the union's near edge is the *base* at `r-a` and its far edge is the
  *magnified top* at `(r+a)m`. Scaling the whole contour by `1/m` restores the far
  edge exactly and pulls the near edge inward to `(r-a)/m`, leaving

```
        footprint error  =  (r - a) * H / h            radial, biased LARGE
        centroid error   =  half of that, inward
```

At `h = 0.240`, `H = 30`, `a = 15`: **+4.4 mm at r = 50 mm, +10.6 mm at
r = 100 mm, zero at r <= 15 mm.** That is the right size and the right shape to
account for the same 30.5 mm cube reading 34.7 x 37.4 and 35.2 x 37.6 in two
stills of one run.

`correct_top_face_parallax`'s docstring says "everything the contour finder
measures — position AND footprint — is magnified about the nadir", and applies one
`factor` to `zx`, `zy`, `width` and `length`. **That is true only of a contour
lying wholly in the top-face plane.** For an off-nadir prism the contour is the
union of two planes and is not a uniform magnification of anything. Not a wrong
correction — a correction whose assumption expires with offset.

**It does not explain everything, and saying so is the point.** The same run also
produced 26.0 x 26.2 and a fused 23.9 x 30.0, i.e. readings 4–7 mm *small*, and
this mechanism has one sign. So there are two mechanisms with opposite signs
(under-covering segmentation is the obvious second candidate: a shaded edge
clipped by the threshold), and averaging them is why the fused number lands
near-ish while the spread is 9 mm.

**The test that separates them needs no hardware.** From a frames corpus, plot
footprint error against the block's offset from the image centre, per axis. The
parallax residual must lie on `(r - a)·H/h` and must be **radial**; anything that
does not is the other bug. This is the single strongest reason the corpus is #1 on
the open list.

### So, in order of what to actually do

1. **Centre the block, then measure it.** A third survey pass that centres the
   *individual block* — we already have coarse (find the zone) and fine (centre
   the zone) — drives `r` below `a`, where the correction is exact and no side
   wall is visible. `tag_pick_place.py:2353` already notes "converging the camera
   over the block also kills the parallax". Geometric, not algorithmic, and it
   fixes the per-still lens bias at the same time.
2. **Use multiview disagreement as a height sensor instead of discarding it as
   noise.** The shift of a feature between two views is `r*H/(h-H)`, so
   `H = shift*h/(r + shift)`. At `h = 230` and `r = 50–100`, one measurable
   millimetre of shift is 2–5 mm of height. That distinguishes a 15.2 mm slab from
   a 30.5 mm cube from a 61 mm brick — which is the depth information this rig was
   assumed not to have. `spread_m` is currently reported as a quality metric and
   thrown away.
3. **Intersection versus union.** Project each view's contour into the mat plane:
   the part that agrees across views is the base outline, the part that moves is
   side wall. For collision purposes the **base outline** is the number that
   matters, and it is the one that is currently not measured.
4. **Shading, as a supporting cue only.** A matte top face normal to a ceiling
   light is brighter than any slanted face, so a brightness split *within* one
   colour blob separates top from side. Cheap, and it is also why colour goes
   `unknown`: `detect_colour`-style ratios computed over top ∪ shaded side pull
   saturation and value out of every threshold box.
5. **Tapered blocks: stop asking.** For a cone, pyramid or frustum the top face is
   either a point or irrelevant — the grasp is on the sides and the collision
   envelope is the base outline. Concentric nested contours in one blob (a smaller
   top face inside a larger base) is itself the *signature* of a tapered block,
   and the area ratio gives the taper.

## `object_detection.py` — what to take and what cannot transfer

Genuinely useful, and the colour table is better targeted at this block set than
anything we have: red / orange / yellow / green / blue / purple / pink / white /
wood is exactly the set in the photo. Take:

- **The HSV range table as a starting point**, including the two-interval red that
  wraps both ends of hue — correct and easy to get wrong.
- **The idea of a confidence and an explicit `Unknown`** rather than forcing a
  label. `best_ratio < 0.12 -> Unknown` is the right instinct.
- **Averaging N readings with a reset when the classification changes** — the same
  discipline `fuse_detections` applies spatially.
- **The idea of subtracting a known background**, re-pointed: not a captured
  frame, but the *known mat colour* inside the known zone quad. That gives
  foreground segmentation independent of the block's own colour, which is what
  makes a wooden block on a wooden bench findable.

Cannot transfer as written:

- **Background subtraction against a static captured background is the core of the
  script and it is incompatible with a wrist camera.** `capture_background()`
  averages 30 frames of an empty scene; our camera moves to a new pose for every
  still, so the background is different every frame. This is not a porting detail
  — it is the whole segmentation stage. (It would work well for a *fixed*
  overhead camera, which is a real option worth considering separately.)
- **`pixels_per_inch` from a reference object assumes a fixed camera distance.** We
  already solve a homography onto the mat plane from the zone tags, which handles
  tilt and varying height and needs no reference object in frame. Strictly better;
  do not adopt the weaker instrument.
- **Argmax of per-box pixel counts over overlapping HSV boxes decides by box
  width, not by colour distance.** `Wood` (H 5–30, S 20–220) overlaps `Orange`
  (8–20) and half of `Yellow`; `Pink` (160–179) overlaps both `Purple` (130–165)
  and `Red`'s second interval (170–179); `Cyan` (80–100) overlaps `Green`
  (36–90). A wider box wins more pixels of a hue-spread blob regardless of where
  the blob's colour actually sits, and exact ties fall to dict order. Replace with
  **nearest prototype on the blob's median HSV using a circular hue metric** —
  simpler, order-independent, and it gives a real distance to threshold on.
- **It is a script, not a module.** Camera open, background capture and the main
  `while True` are at module level from line 769, so `import object_detection`
  opens the camera and blocks. Measurement state is module globals. Needs
  splitting into functions before any of it can be called.
- **`break` after the first contour** — it classifies one object per frame. We
  need all of them.
- **`classify_shape` counts `approxPolyDP` vertices** at `eps = 0.035 * perimeter`.
  On blobs this small the vertex count is unstable, and — the deeper problem — it
  classifies the **silhouette**, which off-nadir is top ∪ side. It would inherit
  the exact disease documented above.

**Verdict: harvest the colour table, the confidence/`Unknown` discipline and the
averaging; write the segmentation against the mat homography we already have.** A
fixed overhead camera would make the contributed approach work nearly as written,
and that is a real architectural option — it is just a different robot.


# 2026-08-12 EVENING — THE COLOUR PATH, BUILT FOR A DEMO

Scope was cut deliberately, on instruction: **all blocks rest on their least-tall
face, all are grasped at the cube's grasp height, locate the centre and pick.** No
per-block grasp heights, no rest-pose table, no shape classification for the
grasp. What follows is what that took and the four real defects it uncovered.

## The shape of it

Colour is a **segmentation method**, not a new pipeline. `method="colour"` in
`zone_vision`, and every other stage — tag homography, zone frame, parallax
correction, multiview fusion, `select_block`, the whole pick — is reused
untouched. The switch is one flag on `stack_blocks.py`.

**Segmentation is by SATURATION, and that is the real upgrade.** The mat is white
paper (near-zero S at any brightness), the blocks are painted (saturated). One
threshold, and it yields **whole regions** rather than an outline. Canny finds an
edge, needs a dilate and a close to join it up, and every dilation iteration is
added directly to the reported block size — `_segment`'s own comment puts two
iterations at ~1 mm of systematic oversize. Region segmentation has no such term.
It should be *more* accurate than the tag-era path, not less.

## Colour crosses on the WIRE, not the interface

`detection_wire.py` schema 2 already exists precisely because "identity has to
travel and the service response cannot carry it", and its docstring says
extending `DetectBlock.srv` "means a nested message and a rebuild on the Pi,
which is the exact path this file exists to avoid". So schema **3** adds
`colour_code` and `colour_score` per block: copy the file to the Pi and restart
the node. **No interface rebuild, on either machine.**

A code, not a string, and the reason is asymmetric cost:
`block_detector_node.py` records `rcl_send_response` failing here with "string
data is not null-terminated" on a nested message carrying a string. That
diagnosis is incomplete — `shape` is a string on this same wire and works — but
that failure takes the whole detection service down, whereas a bad index shows up
as a wrong colour *name*. **The cheaper failure wins.** `zone_vision.COLOUR_NAMES`
is the one ordered list; both ends import it, so there is no table to sync, and it
is **append-only** because an index is a wire value.

The join back onto the service response is **by index**, which is exact: both
lists are built from the same `result.blocks` in the same callback. It is
*checked* — a length mismatch means a stale free-running message, and pairing a
colour with the wrong contour is worse than having no colour, so the colours are
dropped and said so.

## Classification: harvested, then re-pointed

Taken from the contributed `object_detection.py`: the HSV table (well aimed at
this set), the explicit `Unknown`, the averaging discipline.

**Replaced: the decision rule.** That file took argmax over per-colour pixel
counts inside *overlapping* HSV boxes, which decides by **box width** rather than
by colour distance — `Wood` (H 5–30) contains `Orange` (8–20) and half of
`Yellow`, `Pink` (160–179) overlaps `Purple` and the upper half of `Red`, and
exact ties fall to dict order. A wider box wins more pixels of a hue-spread blob
regardless of where the blob's colour actually sits.

Now: **nearest prototype on the blob's median hue, circular metric.** Median
because a highlight or a shaded facet is an outlier, not a vote. Circular because
red wraps 0/179 and any linear distance gets red wrong. Achromatic classes
(white, wood) are decided on saturation/value **before** hue is consulted at all —
the hue of a near-grey pixel is numerically defined and physically meaningless,
which is how a white block becomes "pink".

Sampled from the **contour's own eroded pixels**, not its bounding box: the box of
a rotated block is up to 41 % mat, which drags the median saturation down and is
exactly how a coloured block reads "white".

## Four defects found on the way

### 1. `MAX_BLOCK_LENGTH_M = 0.060` rejected every 2.4 in block, silently

The set's longest dimension is **61.0 mm**. The filter rejected it on every
frame, as an *empty zone* — the green brick, the blue slab, the 61 mm bar, both
long prisms, the sheared slab. And `MAX_BLOCK_ASPECT = 4.0` rejected the purple
2.4 × 0.6 in bar at aspect **4.01**, by 0.3 %.

Found by rendering a 61 mm block into the new selftest, which detected nothing at
all. Raised to **0.075** (61.0 mm + 23 %, sized against the same over-read the
constant has always been sized against) and **4.5**. Now pinned against a stated
`BLOCK_SET_LONGEST_M` rather than left as a number whose provenance has to be
remembered.

### 2. A near-miss, recorded because it nearly shipped

I convinced myself from a synthetic `cv2.minAreaRect` probe that `zone_vision`'s
`zyaw` computed the **minor** axis while the field documented the major, and
rewrote the derivation to "fix" it. **`test_rectangle` failed at all four
orientations and it was wrong.** `side_a`/`side_b` in that branch are the
zone-space lengths of the two *box edges*, not `rect[1]` — my probe fed it
`rect[1]`, where the ordering is different. The original code is correct: `zyaw`
is the major axis, verified through the full render → warp → `findContours` →
`minAreaRect` path (at rendered 0° the edges measure 26.6 and 51.7 mm and the
51.7 one is chosen; at 30°, 52.4 and 27.1 and the 52.4 one is chosen).

Reverted, and the arithmetic plus the trap is now written into `find_blocks` so
the next person to "simplify" it to `min()`/`max()` on `rect[1]` reads why not.
**Lesson 6 again, from the other side**: a term that reproduces one case exactly
can still be inverted, and one synthetic case is not the pipeline.

### 3. The wrist-yaw convention is unverified and has never mattered

Verified: `zyaw` is the major axis, and the jaws must close across the **short**
side (it is the only side narrower than the aperture on most of this set).

**Not verified: whether `pick_place`'s `block_yaw_deg` names the CLOSING AXIS or
the BLOCK'S MAJOR AXIS.** They differ by exactly 90°, and for a 30 mm **cube** —
every block this project has ever grasped, on every run — `reduce_yaw(·, 4)` folds
90° away, so both conventions produce the identical wrist angle. **No run has ever
been able to distinguish them.** The first elongated block makes it decisive.

`GRASP_YAW_FROM_MAJOR_DEG = 0.0` — today's behaviour, byte for byte — with
`grasp_yaw_report()` printing the block's long-axis world angle against the
commanded wrist yaw, and saying so explicitly when the block is elongated enough
for the two to be distinguishable. **A five-second look at a `--dry-run --confirm`
park settles it.** The selftest pins the constant at 0 and asserts both halves:
that a cube cannot tell the conventions apart, and that a 2-fold block can.

**And note what does NOT catch this.** `grip_span_ok` checks the *block's*
geometry against the aperture, not the wrist — with the offset wrong, a
61 × 30.5 mm block passes the span check and the jaws still close on the long
axis. Only the operator at the park catches it. **Do not run an elongated block
unattended until this constant is settled.**

### 4. The wire budget is now 4 bytes from its own limit

Schema 3's two extra fields cost 160 B at `MAX_BLOCKS`, and the realistic worst
case (10 blocks + 6 block tags) is **~1396 B against the self-imposed 1400**. The
hard limit is the 1500 B MTU, so there is ~104 B of real headroom — but the next
field added here needs a cap reduction first. The selftest prints the remaining
margin and warns below 40 B, so this cannot be walked past quietly.

## What the colour method cannot do, asserted rather than discovered

**A white block on a white mat is not found.** No saturation step to threshold and
no reliable value step either. It is reported as an *absence* — which is the safe
direction, nothing is descended on — and the selftest asserts it so it stays a
known limitation instead of becoming a surprise. The honest fixes are a coloured
mat or a different segmentation, not a threshold nudged until one frame works.
Natural wood *is* rescued, by value: beech is distinctly darker than printer
paper.

## Still open after tonight

- **The four jaw numbers.** The aperture decides which blocks are in scope at all
  (1.6 in = 40.6 mm is already over a 40 mm guess), and the refusal that enforces
  it is currently enforcing an *estimate*. Still the highest-value two minutes in
  the project.
- **`GRASP_YAW_FROM_MAJOR_DEG`**, above. One observation.
- **Real HSV thresholds.** `COLOUR_HUES` and `COLOUR_SAT_MIN` are reasoned, not
  measured; the selftest swatches test the *arithmetic*, not the paint. They get
  settled against a frames corpus — which is still open #1 and now has a second
  customer.
- **Rest poses and per-block grasp heights**, deliberately cut tonight. The
  arithmetic is in the STEP 2 DESIGN section above and unchanged: `g + c <=
  44.5 mm`, grip low.


# 2026-08-12, RUNS 1 AND 2 OF THE COLOUR PATH — WHAT THE LOGS SAID

Two `--by-colour --survey-only` runs. **They failed for completely unrelated
reasons and only one of them was about colour.** Worth separating before anything
else, because the second one reads like a colour failure and is not.

## Run 1: the segmentation worked. Read these two numbers first

```
[detect]   [0] zone ( -1.4, +14.9) mm  yaw -81.0 deg  33.9 x 67.4 mm  rect sym=2
[multiview] [0] zone ( -0.2, +10.8) mm  yaw +10.9 deg  29.7 x 59.1 mm  rect  views=1
```

The green cuboid is **30.5 x 61.0 mm** by construction. The fused reading is
**29.7 x 59.1** — within **0.8 mm and 1.9 mm** — classified `rect sym=2`
correctly, and named **green, score 0.89**, on the first hardware run of a
segmentation method that did not exist that morning.

That is the saturation segmentation doing exactly what it was supposed to: whole
regions, no Canny outline to dilate and close, so none of the size over-read the
old path has. Worth holding onto, because the rest of this section is defects.

**And it confirms `MAX_BLOCK_LENGTH_M` mattered.** The first still measured that
block at **67.4 mm long**. Under the old 0.060 cap it would have been thrown
away, on that still, silently.

## Run 1's real defect: a red prism was named "pink"

```
[identify] contour 1 at zone (+20.1, -22.4) mm is pink (score 0.72, ...)
```

The score is diagnostic, which is the one good thing here. Distance =
`(1 - score) * COLOUR_MAX_HUE_DIST` puts the median hue **5.0 units from pink's
168 prototype**, i.e. ~163 or ~173. Then:

| hue | dist to red (0) | dist to pink (168) | nearest |
|---|---|---|---|
| 160 | 20.0 | 8.0 | pink |
| 170 | 10.0 | 2.0 | pink |
| 172 | 8.0 | 4.0 | pink |
| 175 | 5.0 | 7.0 | red |

**Pink's prototype sat 12 hue units from red's, inside red's own 18-unit
tolerance, so it captured every hue from 160 to 174** — which is exactly where
crimson paint lives. Red and pink are not separable by hue at this resolution and
never were. My table, inherited straight from the contributed one, and the flaw is
mine for keeping both as hue prototypes.

**Pink is physically a TINT of red** — red mixed with white — so it is now split
off red by **saturation**, in the same spirit as white and wood being decided
before hue is consulted at all. `COLOUR_PINK_MAX_SAT = 140`, `COLOUR_PINK_MIN_VAL
= 150`, biased towards red: the pink disc is ungraspable lying flat anyway, while
a red block misnamed pink is a block the operator asked for and did not get.

### Fixing that broke three more things, each a real hole

1. **A 4-unit hue gap at 159–161.** Removing pink left purple (140) as the last
   prototype before red wraps at 180 — a 40-unit span with an 18-unit reach either
   side. `COLOUR_MAX_HUE_DIST` 18 → **21**, which closes it exactly at 160.
2. **A 9-unit gap at 82–90.** Green (60) to blue (112) is 52 apart. No block in
   the set is cyan, but blue paint under warm light drifts precisely that way.
   `cyan` **appended** to `COLOUR_NAMES` (never inserted — an index is a wire
   value) with a prototype at 86.
3. **Pink was unreachable below saturation 90**, because red's own `s_min` was 90
   and the split happens after the prototype loop. Red's floor is now **60**,
   which is exactly `COLOUR_WHITE_MAX_SAT`, so red/pink picks up precisely where
   white leaves off with no band between them matching nothing.

The selftest now walks **all 180 hues** at high saturation and asserts none comes
back `unknown`. That is what found both gaps, and it is cheaper than finding them
on the bench.

### And then: red was named correctly and thrown away anyway

With pink gone, a saturated hue 163 scored **0.19** against
`COLOUR_MIN_SCORE = 0.45` — named red, then discarded for want of confidence,
which is the worst of the three outcomes. **Because red is a BAND, not a point**:
it straddles the 0/179 wrap and real reds spread from ~168 through 0 to ~6.

`COLOUR_HUE_BANDS = {"red": (168, 6)}` — distance 0 anywhere inside, growing from
the nearest edge outside. The band stops at 6 rather than 10 so it does not crowd
orange at 14; a hue of 12 should be an honest toss-up, not a confident red. Every
saturated hue from 160 to 179 now names red with score **0.62–1.00**, and the
selftest pins all twenty of them.

Pink is rescored against `COLOUR_PINK_HUE_REF = 172` for the same reason — its
selecting distance was measured to red.

| blob | verdict | score |
|---|---|---|
| H160 S220 V200 | red | 0.62 |
| H165 S220 V200 | red | 0.86 |
| H170 S220 V200 | red | 1.00 |
| H165 S083 V245 | **pink** | 0.86 |
| H175 S090 V100 | red | 1.00 (dark, so not a tint) |

### The instrument that made all of this possible, and it should have been there first

The only way to diagnose run 1 was to invert the score arithmetic to recover a
hue. `classify_colour` now returns the **median H/S/V** and
`block_detector_node.py` logs it per contour:

```
[0] ... colour red (0.86) HSV(165, 220, 200)
```

On the Pi, where the pixels are — deliberately **not** on the wire, which has 4
bytes of margin left. Every threshold above is still reasoned rather than
measured, and this is the line that turns them into one reading instead of a
sequence of nudges.

## Run 2 was not a colour failure at all

It never reached a block. The **pickup zone survey rejected all 11 sightings**:

```
[survey] pickup J1 -12.5 REJECTED: image centre is 93 mm from the zone centre;
         3 tags are trusted only to 72 mm out (1.0 half-diagonals ...)
[survey] pickup zone: 11 sighting(s), none of them usable.
[stack] no pickup zone, so there is nothing to pick.
```

**This is open item #5, verbatim, and it is now blocking rather than untidy.** One
missing pickup zone tag drops `MAX_CENTRE_OFFSET_HALF_DIAGONALS` from 4:2.0 to
3:1.0, i.e. the trust radius from 144 mm to **72 mm**, and every sighting landed
74–93 mm out. The same run also printed
`place zone: 2 SEPARATE tag squares carry these ids` — the stray mat, also #5.

**The fix is physical: clean or reprint pickup tag 1, and find the stray place
mat.** The trust gate is not being loosened to work around a dirty sticker; it is
the gate that stops a guessed origin sending the arm at a physical target.

For tonight there is now an override: **`--pickup-at X Y --zone-yaw DEG`**, which
skips the pickup survey. It requires `--zone-yaw` and refuses without it — the
zone frame has an origin *and* a rotation, and guessing the rotation swings every
block position about the given origin. Unlike `--place-at` this feeds a **grasp**,
so the tape measure lands on the jaws: tape it, do not estimate it, keep
`--confirm` on.

## Two usability defects, both mine

- **`--survey-only` refused instead of reporting.** Run 1's entire output about the
  pickup zone was `REFUSING: orange (need 1, found 0)` — when the news was that it
  had found a green cuboid, measured it to 2 mm and named it correctly. A survey
  is a diagnostic; a missing name is a finding, not a failure. It now always
  prints the tally and says the `--stack` list is incomplete as a note.
- **The default `--stack` was still `orange green`.** Now mode-dependent:
  `green blue` with `--by-colour` (the demo pair), `orange green` without.
  Not a shared default, because `blue` is not a `BLOCK_CLASS` and a shared list
  would fail to resolve a name before the arm homed. Caught by trying it.

## Why green + blue is a good demo pair, beyond the colours

Both are **1.2 in (30.5 mm) tall in their least-tall rest pose** — the green
2.4 x 1.2 x 1.2 in brick lying down, the blue 1.2 x 1.4 x 1.2 in prism standing.
The stack arithmetic has **one** height for every level (`stack_surface_z`), so a
single `--block-thickness` of 0.030 is correct for both to within 0.5 mm. That is
the only reason a stack of two *different* blocks works at all before the
rest-pose table exists. Green underneath because its 61 x 30.5 mm footprint is the
larger base, and both short sides (30.5, and 30.5 or 35.6) clear the aperture.

## Runs 3-5 — two bugs of mine, and the pickup tag is still the wall

Attempt 1 was Ctrl-C'd. Attempt 2 died on `/detect_block never appeared in 15s`
(the node was mid-restart). Attempt 3 ran the full sweep, and then:

### 1. `--pickup-at` crashed the exact case it was written for

```
AttributeError: 'NoneType' object has no attribute 'origin'
    pickup_origin, pickup_yaw = tuple(pickup.origin), pickup.yaw
```

I put the origin unpack **before** the `if pickup is None` guard, so when the
survey failed the traceback replaced the one message that says what to do next.
The guards now come first. Ordering bug, mine, in code added the same day to
handle this failure.

### 2. `--confirm` did not exist

Both of the last two commands died on `unrecognized arguments: --confirm` — the
two that were supposed to be the demo. `tag_pick_place` has `--confirm`; this file
only ever had the `--yes` that turns it **off**, so confirm was already the
default and the flag was never added. My run instructions said to pass it.

Now accepted as an explicit no-op (`--confirm`, default on; `--yes` still turns it
off, last one wins). **A documented flag the parser rejects is a defect in the
parser, not in the operator.**

`_selftest` now walks the module docstring, extracts every
`python3 stack_blocks.py ...` line and asserts the parser accepts it. That is the
general form of this bug and it is now impossible to ship again.

### 3. The pickup zone survey failed again, same cause

```
[survey] pickup J1 -12.5 REJECTED: image centre is 94 mm from the zone centre;
         3 tags are trusted only to 72 mm out
[survey] pickup run J1 +0.0..+7.5 (4 views) REJECTED: the camera moved only
         19 mm across the mat -- too little to solve the zone yaw (need 25 mm)
[survey] pickup zone: 11 sighting(s), none of them usable.
```

Seven sightings lost to the 3-tag trust radius, and the one usable run of four
views panned only 19 mm against the 25 mm the yaw solve needs. **Still open item
#5. Still a physical fix: clean or reprint the missing pickup zone tag.** With all
four tags the radius is 144 mm and every one of those seven sightings is inside
it.

The place zone survey did work, but its residual went **1.6 mm -> 4.6 mm over 11
views** — worth watching, and possibly the same tag population changing which
views contribute.

### 4. NEW: the colour method invents blocks on an empty mat

The place mat was empty. Five consecutive fine-pass stills reported:

```
7.9 x 9.4   8.6 x 11.3   7.4 x 9.5   3.1 x 7.0   2.9 x 9.6  mm
```

each named `purple` with scores 0.67-0.90, plus a `28.3 x 44.7 mm` blob named
`wood (1.00)`. Two separate holes:

**There was a ceiling on block size and no floor.** `MIN_BLOCK_AREA_FRAC` is 0.18%
of the zone = 18.6 mm², and a 3 x 7 mm sliver is ~20 mm² — it squeaks through.
`MIN_BLOCK_LENGTH_M = 0.015` added, on the long footprint side. The physical fact
that makes it safe: **the smallest footprint long side in the whole set is
30.5 mm** — even the 0.6 in bar presents 61 mm long — so 15 mm is half the true
minimum and still leaves room for the size under-read (a 30 mm block has measured
23.0). Every one of those five slivers is now rejected outright, asserted by
number in the selftest.

**And the operator supplied the piece I was missing: the mat is taped down with
blue tape.** That reframed it — a coloured object *is* in the scene. Checking the
positions settled it though: all five blobs sat **7-12 mm from a corner tag's
centre**, and a 25.4 mm tag spans +-12.7 mm, so they were **on the tags**, not on
the tape. A tag is nothing but maximum-contrast black/white edges, and this lens
fringes them — chromatic aberration puts a saturated purple-blue edge on one side
of every such transition. **The tag has no colour; its edges do.**

Which makes my reason for skipping `flatten_tags` on the colour path wrong: "a
printed black-on-white tag has no saturation, so the colour segmentation drops it
for free". True of the tag, false of its edges. The colour frame now gets the same
treatment as the grayscale one, filled with the mat's own median **BGR** instead
of its median grey — same function, same grown quad, and the same reason it paints
rather than punching holes (a hole bites a chunk out of a block that legitimately
overlaps a corner).

**Two fixes, and they are separable — worth being exact about which does what:**

- `MIN_BLOCK_LENGTH_M` kills the **observed** blobs. All five were under 12 mm.
  Symptom filter, and sufficient for them.
- Painting the tags out removes the **cause**, for every tag that decoded.

The regression test does *not* prove the second from the first, and says so. A
fringe wide enough to clear the 15 mm floor is wide enough to cover a tag's outer
border and stop it decoding — tried, and the frame then fails at the homography
instead. That is a reassuring structural argument on its own: **any fringe large
enough to be mistaken for a block is large enough to make its own zone survey fail
loudly rather than be grasped.** Written into the test rather than dressed up as a
proof, after I built a version that passed with the fix deleted — which is the only
way to find out whether a regression test tests anything.

**As for the tape: it is very probably already excluded.** The zone quad is defined
by the four tag *centres*, so its boundary runs through them and the mat physically
extends beyond — tape at the mat's edge is outside `search_mask`. And tape that did
intrude would be long: over `MAX_BLOCK_LENGTH_M` (75 mm) and rejected. No inset was
added, because insetting `search_mask` clips blocks that overhang the boundary and
that is the exact failure `build_masks` has two masks to avoid: a clipped block
comes back with the wrong size, wrong centre and wrong yaw, which is worse than a
rejected one.

**`wood` is the loosest class in the table and a shadow falls into it.** Any
saturation up to 110, any value up to 205, hue 5-32 — which is exactly where a
neutral grey's numerically-meaningless hue tends to land. Beech has a real if weak
hue; a grey shadow has essentially none. `COLOUR_WOOD_MIN_SAT = 35` added to both
the segmentation mask and the classifier, with the selftest asserting a near-grey
is not wood while real beech still is.

Both thresholds are still unmeasured. The HSV is now logged per contour, so they
get set from a reading of the actual block.

## Run 6 — it missed by 2 mm, and it is NOT the tag I said it was

Same refusal, and the `--pickup-at` crash is gone (the message printed instead of
a traceback). But the diagnosis I gave last time was wrong, and the log says so
plainly. **Tag 1 is in every single sighting.**

| tag | seen in |
|---|---|
| 0 | 11 of 11 |
| 1 | **11 of 11** |
| 2 | 9 of 11 |
| 3 | **3 of 11** |

I named tag 1 from the 2026-08-11 note without checking this run against it. It is
**tag 3** that is marginal here.

### The actual failure chain

```
[survey] pickup J1  +5.0 REJECTED: image centre is 74 mm from the zone centre;
         3 tags are trusted only to 72 mm out
[survey] pickup run J1 +2.5..+2.5 (1 views) REJECTED: the camera moved only 0 mm
```

1. Exactly **one** fine sighting saw all four tags (J1 +2.5) → trust radius 144 mm
   → **accepted**.
2. Every other sighting saw three → trust radius **72 mm** — and the closest the
   fine arc ever brought the camera to the zone centre was **74 mm**.
   **It missed by 2 mm**, ten times over.
3. The one accepted sighting is alone, and one view cannot solve the zone yaw
   (needs 25 mm of camera travel, had 0).
4. "11 sightings, none of them usable."

Either half alone is survivable. Both together leave nothing.

### Root cause: `refine_pitch` aimed the fine arc WORSE than not refining

```
[explore] pickup fine arc: coarse radius 0.1731 m; pitch -71.0 aims the axis at
          0.1961 (+23 mm off the mat centre) -> using pitch -74.3, axis 0.1730
```

The pickup mat sits at ~**0.209 m** (from the surveyed origin of earlier runs).
So:

| aim | axis lands at | error |
|---|---|---|
| unrefined, pitch −71 | 0.1961 | **13 mm** inside the mat |
| "refined", pitch −74.3 | 0.1730 | **36 mm** inside the mat |

**The refinement moved the aim 23 mm further from the truth**, because it trusted
one sighting's projected radius, which was 36 mm short. The place zone shows the
same defect in the other direction — coarse radius **0.2866** against a surveyed
**0.2348**, 52 mm long — and got away with it only because its four tags were
visible, giving it the 144 mm trust radius.

This is OPEN item 2 from 2026-08-12, which I wrote as "needs a median plus a
clamp" and did not build. It is no longer a nicety; it is the blocker.

### Fixed, and the guard needed fixing too

`refine_pitch` now takes **every** coarse sighting of the zone and uses the
**median** radius. The anchor is chosen for tag count, which says nothing about
whether its projected origin is any good.

The first version guarded on max-minus-min and **the selftest caught that it
defeats the median it protects**. The realistic pool from this run — 0.1731,
0.207, 0.208, 0.209, 0.211 — has a *range* of 38 mm, all of it the one bad view,
so a range-based guard refused to refine at all. Its **MAD is 1 mm**. So the guard
is a median absolute deviation against `COARSE_RADIUS_SPREAD_LIMIT_M = 0.015`, and
the estimator and its guard are now the same kind of statistic:

| pool | median | MAD | verdict | error vs truth |
|---|---|---|---|---|
| anchor alone (0.1731) | 0.1731 | — | refine | **36 mm** |
| realistic 5 views | 0.2080 | 1 mm | refine | **0.1 mm** |
| scattered 0.15–0.29 | 0.2100 | 40 mm | **keep −71** | 14 mm, known |

When it refuses it says what it is keeping and how far off that is, because the
unrefined pitch is a known quantity and a confidently wrong one is not.

### Tonight's unblock is `--zone-yaw`, not `--pickup-at`

`fit_zone` skips the baseline check entirely when the yaw is given, so the single
**four-tag** sighting at J1 +2.5 becomes a usable fit — a *measured* origin, not a
typed one. Strictly better than `--pickup-at`.

With one caveat now printed loudly: **a one-view fit has residual 0 by
construction.** `describe()` renders that as "residual 0.0 mm over 1 view(s)",
which reads as perfect and is arithmetic — one view cannot disagree with itself.
Lesson 4 again, and this path is reachable exactly when it matters. `survey_zone`
now warns that nothing cross-checks the origin and, with `--zone-yaw`, nothing
cross-checks the yaw either.

## Run 7 — the pickup zone is fixed. My flag advice broke the place zone.

```
[explore] pickup fine arc: coarse radius 0.2456 m (median of 6, MAD 1 mm);
          pitch -71.0 aims at 0.1961 (-50 mm off) -> using pitch -64.5, axis 0.2455
[survey]  pickup zone: origin (+0.2228, -0.0112)  r 0.2231 m  bearing -2.9 deg
          | zone yaw -93.0 deg  residual 4.1 mm over 11 view(s)
```

**All 11 pickup sightings usable, where the previous run had none.** The median +
MAD fix did what it was built for. Both new guards behaved correctly, including on
the zone that failed:

```
[explore] place fine arc: 6 coarse view(s) disagree about the radius -- MAD 36 mm
          over 0.1375..0.2859, past the 15 mm this will aim on. KEEPING pitch -71.0
```

That is the guard refusing to aim at noise, and it was right to.

### The new failure is `--zone-yaw`, and it is mine

```
[survey] place run J1 +77.5..+102.5 (11 views) REJECTED: views disagree by
         49.5 mm about where the mat is, past the 20 mm limit -- one mat cannot
         do that, so an input is wrong
```

It could not have been clearer that an input was wrong. **The input was the flag I
recommended.** `--zone-yaw` is a single value applied to *both* zones, and across
every survey in this project's history the two squares sit ~180° apart:

| zone | surveyed yaw |
|---|---|
| pickup | −91 to −93 (and sometimes +89) |
| place | **+88.0 to +89.0**, in 55 of 57 surveys |

So `--zone-yaw -93` was right for the pickup mat and **178.4° wrong** for the place
mat.

**A 180° error is the worst case here, not a harmless sign flip.**
`zone_origin_from` computes `origin = axis_hit − R(zone_yaw)·(camera_zx, camera_zy)`,
so a half turn points the correction the opposite way and each view's origin moves
by **twice** the camera offset. Offsets across that arc were 25–130 mm, which is
exactly the 49.5 mm of disagreement the residual gate rejected. The gate worked,
the message was honest, and it still took a table of 57 historical surveys to see
which input it meant.

### Fixed three ways

1. **`--pickup-yaw` / `--place-yaw`.** `explore.zone_yaw_for(args, zone)` resolves
   per-zone first, then the shared `--zone-yaw`, then None (solve it). Threaded
   through the coarse sweep too, so one mat's origins are no longer rotated by the
   other mat's yaw — that is what scattered the place coarse radii over
   0.1375–0.2859 m.
2. **`--zone-yaw` now warns** that it is being applied to both zones, names the two
   mats' typical yaws, and says a half turn is what breaks it.
3. **`fit_zone` checks a given yaw against the one the views imply**, whenever
   there is baseline to imply anything — two sums, and it turns "an input is wrong"
   into *which* input:

```
[explore] the 11 view(s) imply a zone yaw of +88.6 deg, but -93.0 deg was given
          -- 178.4 deg apart. The given one is being used, as asked ...
[explore]   178 deg is a HALF TURN: the two mats on this bench are ~180 deg
            apart, so this is what a single --zone-yaw applied to both looks
            like. Use --pickup-yaw / --place-yaw.
```

Threshold 10°, against a solved-yaw scatter of 0.47° sd across good runs — so it
cannot fire on ordinary noise.

### Next command

The place zone has never needed a fixed yaw; it solves cleanly on its own. Only
the pickup zone did, and with the pitch fix in it may not any more:

```bash
python3 stack_blocks.py --by-colour                       # try with nothing fixed
python3 stack_blocks.py --by-colour --pickup-yaw -93      # if pickup still fails
```

# Runs 8–9, 2026-08-13 — both zones surveyed clean; the guard refused a good measurement

Two `stack_blocks.py --by-colour` runs, **no yaw flags**. The per-zone yaw work is
done: both zones solved their own yaw, both runs, and neither needed telling.

```
run 8   pickup  origin (+0.2190, -0.0099)  r 0.2193  yaw -91.4  residual 3.3 mm over  9 views
        place   origin (+0.0131, +0.2287)  r 0.2291  yaw +88.1  residual 1.8 mm over  5 views
run 9   pickup  origin (+0.2194, -0.0079)  r 0.2195  yaw -91.4  residual 1.8 mm over  5 views
        place   origin (+0.0098, +0.2294)  r 0.2296  yaw +88.2  residual 3.8 mm over 11 views
```

Origins agree between runs to **0.4 mm and 2.0 mm** on the pickup zone, 3.3 mm and
0.7 mm on the place zone. The `--zone-yaw` recommendation that broke run 7 is gone
and nothing replaced it. The MAD guard also earned its keep on both runs, refusing
to aim a fine arc at coarse radii scattered over 84 mm and 78 mm MAD.

Then both runs died in the same place, on the same block, for a reason that was
mine.

## The defect: one nominal footprint for a block set with more than one shape

```
[stack] green: 29.7 x 60.3 mm rect, zone (-18.4, +0.9) mm, ... views=1
[stack] WARNING: measured length 60.3 mm against a nominal 30.0 mm.
[stack] REFUSING to grasp the green: its footprint is 60 mm long, past the 50 mm
        at which one 30 mm block becomes implausible -- two blocks touching read
        as one blob whose centroid sits in the seam between them.
```

**The green brick is 30.5 x 61.0 mm by construction.** 29.7 x 60.3 is right to
**0.8 mm and 0.7 mm** — one of the better footprint readings this rig has
produced. It was refused for being the size it is.

`merged_contour_reason` and two other footprint checks compared against
`BLOCK_NOMINAL_M = 0.030` alone. That is correct for the tag path, where every
tagged block on this bench is a 30 mm cube, and it is **structurally wrong for the
colour path, whose entire purpose is a block set with different shapes**. The
colour path shipped on 2026-08-12 without anyone noticing that it inherited a
single-shape assumption from the path it replaced. Two full hardware runs paid for
it.

### What no footprint test can do, and why the fix has to be a lookup

Two touching 30.5 mm cubes and one 61 x 30.5 mm brick are **the same rectangle**.
There is no threshold anywhere that separates them. So the guard cannot infer what
it is looking at from size — it has to be *told*, which is what the identity it
already has in hand is for.

### The fix

`COLOUR_FOOTPRINT_M`, a `(short side, long side)` nominal per colour, read off the
Amazon sheet:

| colour | block | footprint | merge threshold |
|---|---|---|---|
| green | 1.2 × 1.2 × 2.4 in brick, lying | 30.5 × 61.0 mm | 50.5 / 81.0 mm |
| blue | 1.2 in hexagonal prism, standing | 30.5 × 35.6 mm | 50.5 / 55.6 mm |
| *anything unlisted* | 30 mm cube | 30.0 × 30.0 mm | 50.0 / 50.0 mm |

Three consequences worth stating separately:

1. **`MERGED_FOOTPRINT_M = 0.050` became `MERGED_MARGIN_M = 0.020`.** The 50 mm
   was tuned on this bench — 44 mm is the worst observed single-block over-read,
   60 mm is two touching cubes, 50 splits it. But it is the **margin** that
   transfers to another shape, not the absolute length. 30 + 20 reproduces the
   tuned 50 mm exactly, and the brick gets 81 mm.
2. **Both axes are now tested, not just the longest.** On a cube this changes
   nothing (if `min >= T` then `max >= T`). On an elongated block it is the
   whole point: two green bricks touching along their long sides read **61 × 61**,
   whose longest side passes an 81 mm test. It is the *short* side, 61 against a
   nominal 30.5, that gives it away. On a cube the two merge geometries are the
   same rectangle; on a brick they are not.
3. **The fallback is the old behaviour exactly**, so nothing that used to pass
   starts failing. An unlisted label, or no label at all, is the 30 mm cube.

The footprint *warnings* in `stack_blocks.grasp_target` and
`tag_pick_place.report_reached` had the same bug in a quieter form: comparing both
axes against one nominal printed a **30 mm error on a measurement that was right
to 1 mm**, every run. Now short against short, long against long.

Nine selftests pin it, including the false positive reconstructed from the run's
own numbers (`_Blk(0.0297, 0.0603)`), both merge geometries for the brick, and an
assertion that every colour in `DEFAULT_STACK_COLOUR` has a footprint that clears
the jaw aperture and `zone_vision.MAX_BLOCK_LENGTH_M`.

## The next wall: 1 usable view of 5, and it is a framing problem

Run 9's multiview took five stills and fused **one**:

```
[multiview] still 1/5 at wrist yaw  +88 -> tags [0, 1, 2, 3]  4 tags, FUSED
[multiview] still 2/5 at wrist yaw  +58 -> tags [1, 2]        only 2 tag(s); 3 needed to VOTE
[multiview] still 3/5 at wrist yaw +118 -> tags [0, 3]        only 2 tag(s)
[multiview] still 4/5 at wrist yaw  +28 -> tags [1, 2]        only 2 tag(s)
[multiview] still 5/5 at wrist yaw +148 -> tags [0, 3]        only 2 tag(s)
[multiview] 1 usable view(s) ... <-- ONE VIEW ONLY, no cross-check
```

**This is not the pickup tag-3 problem.** Tag 3 decoded in three of the five
stills. Every still saw *some* tags cleanly at rms 0.47–1.04 px. The problem is
that each still framed only half the mat.

### Where the lens actually landed

`still 1` measured the lens 15.6 mm off the mat centre and re-centred the other
four by (+15.1, +3.9) mm — which is arithmetically right: (−4.3, +15.0) in the
zone frame rotated into the world by the −91.4° zone yaw is (+15.1, +3.9) to four
places. The correction was applied correctly and **the lens still landed 15–25 mm
off centre on every subsequent still**:

| still | wrist yaw | camera at zone | lens→mat |
|---|---|---|---|
| 1 | +88 | (+4.3, −15.0) | 0.2450 |
| 2 | +58 | (+9.0, −15.2) | 0.2411 |
| 3 | +118 | (−2.7, −22.9) | 0.2380 |
| 4 | +28 | (+14.0, −17.6) | 0.2261 |
| 5 | +148 | (−4.3, −24.6) | 0.2224 |

Zone X falls monotonically with wrist yaw and zone Y never once goes positive.
That is structure, not scatter. Fitting `c + kx·cos(yaw) + ky·sin(yaw)` per axis
over the five points:

| axis | constant | rotating amplitude | phase | fit rms |
|---|---|---|---|---|
| zone X | +6.0 mm | 11.1 mm | −15.2° | **0.6 mm** |
| zone Y | −26.2 mm | 10.7 mm | +60.5° | **1.4 mm** |

Two components, and they have different homes:

- **An ~11 mm rotating term.** Equal amplitudes (11.1, 10.7) and phases 75.7°
  apart is what a rigid vector rotating with the wrist looks like — i.e. **the
  modelled camera offset in the flange frame is wrong by about 11 mm**. A one-shot
  re-centring measured at a single wrist yaw *cannot* remove this, by
  construction: it fits a constant to something that rotates.
- **An ~26 mm constant in zone −Y.** At a −91.4° zone yaw, zone −Y is world −X at
  bearing −2°, i.e. **radially inward**. The lens lands ~26 mm short of the mat
  centre no matter what the wrist does.

### Why 26 mm of radial error has never hurt a grasp

Because it cancels. The grasp world position is
`survey_origin + R(zone_yaw)·(zone-local block)`, and the survey origin was itself
measured *through the same under-reaching arm*. Command the arm back to that
origin and it reproduces the same physical point. The bias is in commanded
coordinates on both sides and it subtracts out — which is why 2026-08-12 stacked
two blocks perfectly with this error present the whole time.

**The camera is the one thing in the loop that does not participate in the
cancellation.** It looks where the arm *actually is*, not where it was told to be,
so the residual shows up in the optics and nowhere else. It costs framing, not
accuracy. On a 102 mm mat with 25.4 mm tags, 26 mm of inward offset plus 11 mm of
wrist swing is enough to push the far tag pair out of frame — which is exactly the
4-of-5 failure above.

It also matches the number already in hand from run 7: the surveyed pickup radius
was **22 mm shorter** than the coarse median. Same sign, same magnitude, different
measurement.

**This is a hypothesis on five points with 2 df, not a result.** What confirms it
is the `flange_fk`-versus-commanded comparison already in the calibration plan,
and it should be confirmed before anything is written to a constant. What makes it
worth recording now is that it is the first *optical* measurement of the reach
error, and it decomposes cleanly into one term that belongs in the camera mount
and one that belongs in the arm.

## Also seen, both already understood

- `[tool] reach 230 mm is outside the 121-222 mm span the tangential term was
  measured over, so it is EXTRAPOLATED here (-9.94 mm)` at the place zone. The
  place radius is 0.2296 and the term carries a 3.4° angular component. Known,
  flagged by the code, unmeasured at this reach.
- `[pick_place] hover 0.215 exceeds the reachable ceiling 0.205 -- clamping` at
  level 1, shortening the descent to 29 mm. Expected — this is `MAX_HOVER_Z`
  doing its job and the reason stack level 2 is illegal.
- Two stills fell back to z 0.240 when IK found no solution at 0.255 at radius
  0.218, costing focus. The `[multiview] z 0.255 out of reach` path worked as
  designed.

## Jaw clearance on the demo pair is 1.5 mm — move the blue

With the guard fixed, the next check passes, but barely:

```
[clearance] green block at jaw axis +91.8 deg: CLEAR (1.5 mm of room, nearest neighbour #0)
[clearance] NOTE: jaw aperture 40 mm, finger 8 x 18 mm are NOT MEASURED
```

The blue prism sits 46 mm from the green's centre, mostly *along* the closing
axis. 1.5 mm of margin on top of three unmeasured finger dimensions is a coin
flip, and the green is symmetry 2 so the wrist cannot rotate 90° out of the way.

| blue moved further out | centres apart | margin |
|---|---|---|
| +0 mm | 46 mm | +1.5 mm |
| +5 mm | 51 mm | +5.7 mm |
| **+10 mm** | **56 mm** | **+10.1 mm** |
| +20 mm | 66 mm | +19.4 mm |

**10 mm buys 10 mm.** Cheapest fix on the bench tonight.

## And the yaw convention is finally decidable

`GRASP_YAW_FROM_MAJOR_DEG` has been unsettled since it was written, because
`reduce_yaw(·, 4)` folds 90° away on a cube and cubes are all this project had
grasped. The green brick is elongated and symmetry 2, so the fold no longer hides
it, and the code says so:

```
[grip] green: 29.7 mm short side x 60.3 mm long side. Its LONG axis lies at
       +0.4 deg in the world; the wrist is commanded to +0.4 deg.
[grip]   ELONGATED, so the two conventions are distinguishable here: the fingers
         must end up spanning the 29.7 mm side. LOOK AT THE JAWS at the park --
         if they are lined up to close on the 60.3 mm side instead, set
         GRASP_YAW_FROM_MAJOR_DEG to 90.
```

One look at the park settles a constant that has been a guess for a fortnight. It
is worth stopping the run there to look.

# Run 10, 2026-08-13 — `zone_yaw = 0.0`, and everything downstream of it

The merged-contour fix was never reached. The pickup survey threw away all eleven
of its sightings:

```
[explore] pickup fine arc: 6 coarse view(s) disagree about the radius -- MAD 84 mm
          over 0.0416..0.2803, past the 15 mm this will aim on. KEEPING pitch -71.0
[survey]  pickup J1 +5.0, +12.5, +27.5 REJECTED: only 2 tag(s); 3 needed ...
[survey]  pickup J1 +22.5 REJECTED: image centre is 74 mm from the zone centre;
          3 tags are trusted only to 72 mm out
[survey]  pickup J1 +25.0 REJECTED: image centre is 80 mm ...
[survey]  pickup run J1  +2.5..+2.5  (1 views) REJECTED: camera moved only  0 mm
[survey]  pickup run J1  +7.5..+10.0 (2 views) REJECTED: camera moved only  7 mm
[survey]  pickup run J1 +15.0..+20.0 (3 views) REJECTED: camera moved only 14 mm
[survey]  pickup zone: 11 sighting(s), none of them usable.
```

The place zone surveyed fine — origin (+0.0097, +0.2294), **identical to run 9's
(+0.0098, +0.2294) to 0.1 mm**. Nothing on the bench moved. The two coarse sweeps
are the same measurement twice: run 9's first coarse hit reads
`0.3308 m from the mat | camera at zone (+132.3, -118.9)`, run 10's reads
`0.3309 m | (+131.4, -119.4)`.

So this was not a hardware difference. Two defects in my code, one of them the
cause of a message that has appeared in every run for three days.

## Defect 1 — the coarse origins are built with an assumed zone yaw of ZERO

`sweep_both` computes each coarse sighting's origin as

```python
origin = zone_origin_from(joints, response.camera_zx, response.camera_zy,
                          zone_yaw_for(args, zone) or 0.0)
```

With no `--pickup-yaw`, `zone_yaw_for` correctly returns `None` — *solve it* — and
`or 0.0` turns that into a literal zero. **The pickup mat sits at −91.4°.**

`zone_origin_from` subtracts `R(zone_yaw) · (camera_zx, camera_zy)`. A 91° error in
that rotation, applied to coarse camera offsets of 58–178 mm, does this — the same
six views, the same six camera positions, only the yaw differing:

| zone yaw | coarse radii | median | MAD |
|---|---|---|---|
| **0.0** (assumed) | 0.0365 … 0.2837 | 0.1491 | **84.3 mm** |
| **−91.4** (solved) | 0.2445 … 0.2497 | 0.2481 | **1.6 mm** |
| +88.2 (the *place* mat's, for contrast) | 0.1455 … 0.2819 | 0.1802 | 32.4 mm |

Then the whole run follows, mechanically:

1. `refine_pitch`'s MAD guard refuses to refine — **correctly**. The radii it was
   handed really were garbage. The guard was the one thing working properly.
2. The fine arc runs at the default pitch −71°, which aims the optical axis at
   0.1961 m against a mat at 0.247 — **51 mm inside it**. The code said so:
   "a known +52 mm off their median rather than a guess."
3. Aimed 51 mm short, the camera never passes over the mat centre. Its offset
   walks 53.6 → 86.0 mm monotonically across the arc, against a 3-tag trust
   radius of 72 mm. Two views gated for that.
4. Aimed short also means aimed *far*: 0.35 m from the mat, where a 1 in tag is
   **39 px** across. Three views decoded only 2 tags. (At the multiview hover,
   0.245 m, the same tags are 57 px and run 9 got all four.)

Every rejection in the run traces to `or 0.0`.

### The fix — solve the yaw from the coarse sweep, which needs nothing new

`camera_zx/zy` and `joints` are both **yaw-free**; the yaw only enters when they
are combined. So `fit_zone` can solve it from the coarse sightings directly, and
the coarse pass pans the camera ~149 mm across the mat against
`MIN_YAW_BASELINE_M`'s 25 mm — the baseline is never the binding constraint.

`reseat_coarse_origins(sightings, yaw_fixed, label)` solves it, rewrites every
coarse `zone_origin` with the answer, and says so. A **given** yaw still wins; this
only fills in the case that used to assume zero. On run 10's replayed data it
solves **−89.2° with a 12.4 mm residual** against the fine survey's independently
found −91.4°, the radii tighten to a 2 mm MAD, and the pitch refines to −64.3°
aiming at 0.2471 — on the mat instead of 51 mm inside it.

## Defect 2 — an isolated rejection was read as a mat boundary

`split_runs` exists for a good reason: fitting across two *different* mats produces
a confident point in the empty space between them, and the 2026-08-05 coarse pass
did exactly that — ids 0–3 at J1 −135..−65, nothing for 90°, then ids 4–7 at
+45..+115. A gap in the sweep is the evidence.

But `choose` split the **survivors**, not the sweep. A gated sighting leaves a
2-step hole in that list, which is indistinguishable from the tags going out of
sight — so every isolated rejection became a fake mat boundary. Run 10's six good
views, with five rejections interleaved, were split into runs of **1, 2 and 3**,
with **0, 7 and 14 mm** of baseline, and all three were refused for under-running
the 25 mm needed to solve a yaw.

Pooled, those same six views span **52 mm** and fit to **5.3 mm residual** at
`origin (+0.2213, +0.0016) r 0.2213 m yaw −90.1` — which `gate_fit` accepts, and
which agrees with runs 8 and 9's surveyed 0.2193 and 0.2195 m. **Eleven usable
sightings were thrown away by bookkeeping.**

A rejected sighting is not a gap. The arm went there, the tags *were* seen, and
the measurement was judged untrustworthy. The mat did not move. So the grouping now
runs over **all** the sightings — where the real holes are — and keeps the usable
members of each group. The 90° hole still splits; a selftest asserts it.

## Both defects are pinned to this run's own numbers

Nineteen checks in `explore.py --selftest`, built from the logged camera positions
rather than from invented ones: that assuming 0° scatters the radii past the MAD
limit, that `refine_pitch` was right to refuse, that the solved yaw lands within 3°
of −91.4, that the same views then agree to a few mm, that the refined pitch aims
at the mat, that the unrefined one was ~50 mm off, that a given yaw is still
obeyed, that splitting survivors gives three unusable fragments while splitting the
sweep gives one good stretch, that the pooled fit clears both `MIN_YAW_BASELINE_M`
and `gate_fit`, and that the 2026-08-05 two-mat split survives.

## What is still fragile, and was not changed

**The coarse anchor is `argmax` over tag count, and one dropout moves it 15°.**
Run 9 anchored at J1 +0.0 (3 tags, 56.9 mm off centre); run 10 anchored at +15.0,
because tag 2 dropped out of the J1 0 still and left it with 2 — while +15 had 3
and an offset of 59.6 mm. **A 1.5 mm difference in framing lost to a single tag,
and the fine arc moved 15°.** The offsets over the coarse arc are 177.6, 132.9,
90.2, 58.0, 59.6, 92.3 — a clean V whose minimum interpolates to J1 **+6.8** in
run 10 and **+6.7** in run 9. The vertex is stable to 0.1° across two runs whose
argmax differed by 15°, so a parabolic centring would be strictly better.

Not changed tonight, because with the pitch now refined the arc is aimed at the mat
and a 15° error in its centre costs much less than it did here. Recorded as the
next thing to do to the search.

**And the standing ~53 mm camera offset is still unexplained** — see run 9's
decomposition into an ~11 mm term that rotates with the wrist and an ~26 mm
constant pointing radially inward. Defect 1 accounts for the *aim* being 51 mm
short; it does not account for the residual once the aim is right.

# 2026-08-13 — THE FIRST COLOUR STACK. Green brick, blue prism, no AprilTags on the blocks

```
[stack] stack complete: green -> blue, bottom first, at (0.0092, 0.2334).
```

Both fixes from run 10 landed, and the survey is now the strongest it has ever been
on this bench:

```
[explore] pickup coarse yaw solved at -91.4 deg over 6 view(s)   <- fine survey: -91.4
[explore] pickup fine arc: coarse radius 0.2450 m (median of 6, MAD 1 mm);
          pitch -71.0 aims at 0.1961 (-49 mm off) -> using pitch -64.5, axis 0.2451
[explore] place  coarse yaw solved at +87.9 deg over 6 view(s)   <- fine survey: +88.1
[explore] place  fine arc: coarse radius 0.2550 m (median of 6, MAD 2 mm)
                                        -> using pitch -63.3, axis 0.2550
[survey]  pickup residual 4.3 mm over 11 view(s)
[survey]  place  residual 4.2 mm over 11 view(s)
```

**Every one of the 22 sightings survived, both zones, both runs of the gate.** The
coarse yaw the sweep solved landed within 0.0° of the pickup fine survey and 0.2°
of the place one, and the MADs went from 84 and 78 mm to **1 and 2 mm**. That is
`reseat_coarse_origins` doing exactly what the replay predicted.

## 1. `GRASP_YAW_FROM_MAJOR_DEG` is 90. Measured, not reasoned

The constant that has been an open question since it was written:

```
[stack] green: 30.5 x 60.2 mm rect ... yaw +0.2 -> grasp yaw +0.2 (symmetry 2)
[grip]  Its LONG axis lies at +0.2 deg in the world; the wrist is commanded to +0.2.
[grip]  ELONGATED, so the two conventions are distinguishable here ...
[confirm] ENTER = go  'dx dy' = nudge  'm dx dy' = record  q = abort > 0 0 -90
```

The operator turned the wrist 90°. **That answers it: `block_yaw_deg` names the
block's MAJOR axis, so the closing axis needs 90° on top of it.** Elongated and
symmetry 2, so nothing folded the answer away — the first block this project has
ever grasped that could tell the two conventions apart.

Set to **90.0**, and `run_stage1` was missing the term entirely — `stack_blocks`
applied it and `tag_pick_place` did not, a disagreement that was invisible only
while the constant was zero. Both now carry it, and a selftest counts the call
sites in each file so the next one cannot be missed.

**Nothing already validated moves.** On a symmetry-4 block `reduce_yaw` has period
90°, so adding 90 lands on the identical wrist angle: every tagged cube, every
calibration row ever recorded, unchanged. Asserted at four yaws. And +90 and −90
name one closing axis (it is a line, period 180°), so the operator's −90 and the
constant's +90 are the same instruction — also asserted, at both symmetries.

## 2. The stack was digging into the block below. Two millimetres, two causes

> "it stacked the block and was probably trying to dig into the block below"

It was. Both parts are measurable and only one of them is fixable by arithmetic.

**a. The block is 30.5 mm tall and the step assumed 30.0.** Level 1's surface was
computed at `MAT_SURFACE_Z + 30.0` while the green's top face is at +30.5, so the
release aimed **0.5 mm inside it**. `COLOUR_HEIGHT_M` now carries the height beside
the footprint, and `--block-thickness` defaults to the **tallest block in the
stack** rather than to `BLOCK_HEIGHT_M`. Max, not per-level, because
`stack_surface_z` is linear in the level and everything downstream assumes that;
per-level heights are the honest version and are not built.

**b. The arm does not land where it is sent in z, and the error CHANGES SIGN.**
From the run's own `[reached]` lines at the two place descents:

| | commanded | reached | dz |
|---|---|---|---|
| level 0 | 0.1443 | 0.1471 | **+2.8 mm** (high) |
| level 1 | 0.1743 | 0.1729 | **−1.4 mm** (low) |

**A 4.2 mm spread between two descents to the same XY minutes apart.** No constant
corrects that, because it is not a constant. So it gets clearance, not a
correction: `PLACE_DROP_M = 0.003` releases the block 3 mm above the computed
surface and lets it drop. A 30 mm block dropped 4 mm onto a flat top lands flat; a
block pressed 3 mm into the one below either tips the stack or stalls the arm
holding it there.

Three design points worth keeping:

- **The drop raises the RELEASE, not the surface.** Level *n*+1 still sits exactly
  one block height above level *n*, so the step stays honest and the drop does not
  compound up the stack.
- **Level 0 gets no drop.** The mat cannot be dented, so there is nothing to dig
  into — and level 0's placement is the one the stack's straightness is measured
  against. Both levels target the same XY so a systematic error displaces the stack
  instead of tipping it (design decision 4); a drop that lets the bottom block
  bounce a millimetre is the one thing that breaks that cancellation. Set it down;
  drop only onto blocks.
- **`PLACE_DROP_M` is not `PLACE_CLEARANCE_M`.** The latter already existed and is
  how far above the release the block is *parked for inspection* (8 mm). I first
  wrote the new constant under the same name, which would have silently cut that
  park from 8 mm to 3 mm. Caught before it ran. Park high, release slightly high,
  land.

Net effect at level 1: release 0.1755 → **0.1793**, of which 0.5 mm is the true
block height, 0.25 mm the half-height, and 3.0 mm the drop. The descent shortens
from 30 to 26 mm, still well over `MIN_USEFUL_DESCENT_M`, and a selftest asserts
the blue's underside now ends up **above** the green's top face by more than the
1.4 mm that descent undershot by.

## 3. The two after-the-fact prompts are gone

Removed on request: *"did the jaws actually close on the X block?"* and *"is the
block sitting squarely on level N?"*. The operator is watching and will Ctrl-C, so
the prompts only sat between them and the stop. The **parks** stay — they are where
nudges and `m` readings happen.

What that gives up, recorded so nobody re-derives it as a surprise:

- **Nothing else in the loop knows whether a block is in the jaws.** The gripper's
  `CONTACT` detection is the closest thing (`jaw trailing its command by 0.0750 rad
  (>= 0.06)` in this run) and it fires on the fingertips touching *anything*,
  including each other on a missed block.
- **Nothing measures whether a level went down square.** The camera never looks at
  the stack, only at the pickup zone. A crooked level 0 makes level 1 land on a
  slope and the run will not notice.

Both fields now record as **`None`, not `False`** — *nobody looked* is a different
claim from *the operator said it failed*, and `bool(pick_pose.get("grasped"))`
would have relabelled every future grasp in `calibration_history.jsonl` as a
confirmed failure. Caught while wiring it.

## Still open after this run

- **The blue needed no nudge and the green needed 90°**, which is the difference
  between symmetry 0 (yaw discarded, any wrist grabs a hexagon) and symmetry 2.
  With the constant now at 90 the green should need no nudge either — that is the
  next thing this run has not yet tested.
- **The jaw clearance was 1.0 mm** at the green (`CLEAR (1.0 mm of room)`) and the
  operator moved the blue by hand to make the grasp. Still on unmeasured finger
  dimensions. Caliper them.
- **The ~53 mm standing camera offset** and its decomposition into ~11 mm rotating
  and ~26 mm radial-inward — untouched, and now the largest thing between this rig
  and an unattended run.
- **Level-0-vs-level-1 droop** is still unmeasured, and the ±2.8/−1.4 mm z spread
  above is the first direct evidence of how big it is.

# 2026-08-13, next run — the yaw fix works. Stopped on 0.2 mm of jaw clearance

Both new constants did what they were meant to, visibly:

```
[stack] step 0.0305 m from the tallest block in the stack (green 0.0305, blue 0.0305),
        not the 0.0300 default -- a step short of a block's real height puts the
        next release INSIDE it.
[stack] level 1: surface z 0.0265, release flange z 0.1793, hover 0.2050 (26 mm
        descent) -- releasing 3.0 mm high, so the block DROPS that far
```

And the survey held up again: coarse yaw solved at **−91.4** and **+87.8** against
fine surveys of −91.3 and +88.2, MADs of **2 mm** on both zones, 9 of 11 and 10 of
11 sightings usable.

## The yaw convention is confirmed correct on the second look

```
[stack] green: 32.3 x 61.2 mm rect ... yaw +177.0 -> grasp yaw -3.0 (symmetry 2)
[grip]  Its LONG axis lies at +87.0 deg in the world; the wrist is commanded to -3.0.
```

**+87.0 and −3.0 are 90° apart.** The fingers would have spanned the 32.3 mm side,
which is the whole point, and no nudge was needed. `GRASP_YAW_FROM_MAJOR_DEG = 90`
is right.

`grasp_yaw_report` was still printing the *old* advice underneath it — "if they are
lined up to close on the 61.2 mm side instead, set GRASP_YAW_FROM_MAJOR_DEG to 90"
— asking for a constant that is already set. It now **checks** instead of asking:
the angle between the two lines has to come out near 90, and if it does not it says
so loudly and names the constant as the likely cause. Quiet when correct.

## The run stopped on jaw clearance, and "move them apart" was wrong advice

```
[clearance] green block at jaw axis +88.3 deg: BLOCKED (0.2 mm of overlap)
[clearance] This block's footprint is 2-fold, so base+180 is the SAME jaw axis --
            there is no alternative orientation to try. Move the blocks apart.
```

0.2 mm, on finger dimensions nobody has measured. The blocks were 48.5 mm apart —
the previous successful run had 1.0 mm of room at the same sort of spacing, so this
is not a new failure so much as the same coin landing the other way.

**But "move the blocks apart" is not enough information, and following it in the
obvious direction makes things worse.** The fingers are 8 mm thick along the
closing axis and 18 mm wide across it, so the two directions cost completely
differently. With the green brick as the target:

| blue's position relative to the green | distance | margin |
|---|---|---|
| 40 mm **along** the closing axis (beside the brick) | 40 mm | **−11.3 mm** blocked |
| 34 mm **across** it (off the brick's end) | 34 mm | **+8.7 mm** clear |

**A neighbour 6 mm nearer is 20 mm better, because it is in the other direction.**
Distance is the wrong variable and it is the one an operator reaches for.

Worse, moving the blue *further along* is not even available: it was already 22 mm
from the zone centre against a usable box of ±23.3 mm
(`DEFAULT_ZONE_SIZE − DEFAULT_TAG_SIZE − BLOCK_HEIGHT_M`), so every extra
millimetre in that direction pushes it off the mat. Sliding it 4 mm would have
cleared the jaws and put it outside the surveyed area.

So the refusal now decomposes the blocker into the jaw frame and names the
direction:

```
[clearance] neighbour #0 sits 38 mm ALONG the closing axis and 30 mm across it.
            The fingers are 8 mm thick along that axis and 18 mm wide across, so
            ALONG is the expensive direction.
[clearance] MOVE IT OFF THE END of the target instead of beside it -- i.e. increase
            the 30 mm and let the 38 mm shrink. Sliding it further along the
            closing axis buys much less per mm.
```

### The underlying squeeze, stated plainly

A 4 in mat with 1 in corner tags leaves a **46.6 mm box** for a block centre, and
the green brick is **61 mm long**. Two blocks that need ~50 mm of separation in the
expensive direction cannot both live in that box; on the diagonal there is 66 mm,
which is why some layouts work and near-identical ones do not. **The demo pair is
at the geometric limit of this mat**, and the fix is either a bigger pickup mat, or
placing the blue deliberately off the brick's end every run.

## And a 3 mm lie in the run's own summary, mine

```
[stack]   level 0  green  release flange z 0.1488      <- the summary
[stack] level 0: surface z -0.0040, release flange z 0.1458   <- what it actually used
```

The pre-flight summary applied `PLACE_DROP_M` at every level while
`check_stack_geometry` correctly withholds it at level 0 — and the summary's own
label said "no drop" while showing a number 3 mm higher. Nothing moved wrong, but
two printed release heights disagreeing by 3 mm in one run is exactly the kind of
thing that costs an hour later. The summary now takes the drop per level, prints
"set down, no drop -- it lands on the mat" for level 0, and a selftest asserts the
two paths agree at both levels.

## Next

```bash
python3 stack_blocks.py --by-colour
```

**Put the blue off one END of the green brick, in line with its length, not
alongside it.** ~32-34 mm from the brick's centre in that direction gives 7-9 mm of
margin — more than the 1.0 mm the successful run had, at less separation than the
48.5 mm that just failed.

Still open, unchanged: the jaw dimensions are guesses and this refusal is computed
from them; the ~53 mm standing camera offset; level-0-vs-level-1 droop.

# 2026-08-13 — the green brick is retired. Red trapezoid + blue frustum

The green brick is out, on the operator's call, and the reason is geometric rather
than a defect: at 61 mm long it needs ~50 mm of jaw clearance from its neighbour in
a mat whose usable box for a block centre is **46.6 mm** across. It refused on
clearance about as often as it grasped. Its rows stay in the tables; only the
default pair changed.

## The run itself: best detection yet, and it failed only on the default `--stack`

```
[multiview] 3 usable view(s), tags seen across all of them: [0, 1, 2, 3]
[multiview]   [0] zone (+14.4, -19.5) mm  32.6 x 33.0 mm  square  views=3 spread=1.4 mm
[multiview]   [1] zone (-24.4, +15.4) mm  28.6 x 31.2 mm  circle  views=3 spread=1.7 mm
[identify] contour 0 is red  (score 1.00, 100% of 3 view(s) agree)
[identify] contour 1 is blue (score 0.60, 100% of 3 view(s) agree)
[stack] REFUSING: green (need 1, found 0) not found in the pickup zone.
```

**3 usable views**, up from 1 and 2 in the previous two runs, with spreads of 1.4
and 1.7 mm and red at a perfect colour score. The perception is not the problem
any more. The run cost a full two-minute sweep to discover that the default
`--stack` named a block that was not on the mat — **while the two lines above the
refusal said exactly what was.** The refusal now says so:

```
[stack] The mat is holding red x1, blue x1. Did you mean:
[stack]     python3 stack_blocks.py --by-colour --stack red blue
[stack] Check the order -- the FIRST name goes on the bottom ...
```

## The tables are BENCH STATE, and that just bit

`COLOUR_FOOTPRINT_M` and `COLOUR_HEIGHT_M` are keyed by **colour**, and the set has
two reds, two blues and two greens. An entry is only correct while that particular
block is the one on the mat. When the pair changed, the `green` row stopped
describing anything present and `blue` needed re-labelling — the block I had
catalogued as a "hexagonal prism" is the **frustum**, same 1.2 × 1.4 in base and
1.2 in height, wrong name. Written into the table as a caveat: **a stale row is
worse than a missing one**, because a missing one falls back to the cube and says
so out loud.

Added `red`: 1 in trapezoid, base 30.5 × 35.6 mm, height **25.4 mm**.

## That 25.4 mm forced per-level heights, which I had explicitly not built

`max(heights)` — the first version — never digs in, but it over-shoots every level
whose supporting block is shorter than the tallest. Red 25.4 under blue 30.5 puts
level 1 **5.1 mm too high**, so the frustum is dropped 5.1 + 3.0 = **8.1 mm** onto a
trapezoid's small top face. Safe from the arm's point of view; not from the stack's.

`stack_surface_z` now sums the heights *below* a level rather than multiplying, and
`height_at` accepts a scalar or a sequence. **A scalar reproduces the old
arithmetic exactly** — asserted at three heights × four levels, because every
constant in the file was tuned against `level * h`.

### And I got the second term wrong for one commit. Worth recording

The formula reads

```
release = surface(level) + <something> + GRASP_OFFSET_Z + clearance
```

and I filled `<something>` with `height_at(level)/2`, reading it as "half the held
block's height". **It is not.** The pick descends to a fixed `--grasp-z`, so the
fingertips always close `GRIP_HEIGHT_ABOVE_BASE_M` above whatever base is under
them — 15 mm, on a 25.4 mm trapezoid exactly as on a 30 mm cube. To put that base
back on the surface, the release undoes the grip height it was *picked* at. The
carried block's own height does not enter at all.

Using the half-height instead gave level 0 a release of **0.1432 against
GRASP_FLANGE_Z 0.1455** — 2.3 mm below it, pressing the red into the mat.

It equals `BLOCK_HEIGHT_M / 2` today, which is a **coincidence** of the pick
gripping a 30 mm cube at its centre, and a misleading one. It is now derived from
the constants that encode it —
`GRASP_FLANGE_Z − MAT_SURFACE_Z − GRASP_OFFSET_Z` — so it cannot drift from them.

**What let it through was a weak test.** `release_flange_z(0) == GRASP_FLANGE_Z`
was asserted only at the default height, where the coincidence holds. It is now
asserted across `None, 0.030, 0.0305, 0.0254, 0.020` and two sequences, plus a
check that the carried block's height does not move the release at all. Same class
of mistake as Lesson 4: an identity checked at one point is not an identity.

## The resulting stack

```
level 0  red   release 0.1455  = GRASP_FLANGE_Z exactly, base lands on the mat
level 1  blue  release 0.1739  base lands 3.0 mm above the red's top face
```

Level 1 is **5.1 mm lower** than it was with the green underneath, which buys back
descent against the `MAX_HOVER_Z` clamp: 31 mm instead of 26.

## Taper is now recorded, and the risk is the PLACE not the pick

`COLOUR_TAPERED` lists red, blue, yellow, purple. Both demo blocks slope.

The grasp direction is the safe one and it is worth saying why: the jaws close
15 mm up, where a taper is **narrower** than the footprint the camera measured, so
the error is toward a looser grip rather than a block too wide for the aperture.

**Stacking ONTO a taper is the real exposure** — a tapered block's top face is the
small end, so the block landing on it has less bearing area than its own footprint
suggests and less margin for the place error. `place_block` says so at the release
of any level landing on a tapered block, which on this pair is level 1.

## Next

```bash
python3 stack_blocks.py --by-colour
```

The default is now `red blue`, so no flag is needed. Watch the level-1 release: it
lands on the trapezoid's small top face, which is the weakest point in this pair.

Unchanged and still open: the jaw dimensions are guesses; the ~53 mm standing
camera offset; level-0-vs-level-1 droop, now with the extra wrinkle that the two
levels are 25.4 mm apart rather than 30.

## The crash after the grasp — `block_height / 2.0` on a list

Symptom: the arm picked a block and returned home. Cause:

```
TypeError: unsupported operand type(s) for /: 'list' and 'float'
```

Turning `args.block_thickness` into the per-level `args.block_heights` touched ten
call sites, and one consumer — `transit_flange_z` — still divided by it. It is
called from `traverse()`, on the **lift after the grasp**, so the run got all the
way to a block in the jaws before raising.

**It was the same conceptual error as the release-height one, in a second place.**
`block_height / 2.0` there was meant as the carried block's half-height, and what
has to clear the obstacle is the block's **base** — which sits
`GRIP_HEIGHT_ABOVE_BASE_M` below the fingertips, because that is where the pick
closed on it. A property of `--grasp-z`, not of the block. So the fix is the same
substitution, and it has two consequences beyond removing the crash:

- **The clearance is now exact for a block of any height.** The old form
  under-cleared a short one: carrying the 25.4 mm red, it computed the base 12.7 mm
  below the fingertips when the real figure is 15.0, so the transit sat **2.3 mm
  lower than it claimed** over whatever it was crossing.
- **The function stops caring** whether it is handed a scalar or a sequence, which
  is what makes the crash impossible rather than fixed.

The documented identity survives and is now exact at every height rather than only
at 30 mm:

```
transit over an n-block pile == release_flange_z(n) + clearance
```

### The test that was missing

One general lesson, worth more than the fix: **changing a scalar to a sequence has
to be checked at every consumer, not at the ones you remember.** The selftest now
drives all six functions that receive `args.block_heights` with both forms:

```
  height_at / stack_surface_z / release_flange_z / obstacle_top_z /
  transit_flange_z / check_stack_geometry
        accepts a per-level SEQUENCE      ok
        still accepts a scalar            ok
```

Verified by putting the bug back: `transit_flange_z accepts a per-level SEQUENCE
FAIL TypeError: unsupported operand type(s) for /: 'list' and 'float'` — the exact
runtime error, caught offline. And the transit identity, asserted at 0.030, 0.0254
and `[0.0254, 0.0305]`, fails on the old half-height form at the two non-default
heights, which is the 2.3 mm above.

# 2026-08-13 — FIRST AUTONOMOUS COLOUR STACK, and what limits its repeatability

```
[stack] stack complete: red -> blue, bottom first, at (0.0107, 0.2337).
```

No nudges. Both zones surveyed, both blocks identified, both grasped, both placed,
operator approving commands only. Everything below is about making it repeat.

## What is already solid, and should be protected rather than touched

| | evidence |
|---|---|
| **Colour identity** | red **1.00** in every still of every run; blue 0.57–0.71. Zero misidentifications across all colour runs. |
| **Coarse yaw** | solved −91.4 / +87.8 against fine surveys of −91.4 / +88.1. Within 0.3°, every run since the fix. |
| **Fine-arc aim** | MAD 2 mm on both zones, every run. |
| **Pickup survey** | **radial spread 0.17 mm** over four surveys. |
| **Per-block geometry** | red 32.3 × 32.6 against a 30.5 × 35.6 base; step 25.4 then 30.5 mm, both correct. |
| **The arm's XY tracking** | every `[reached]` line: `dx` −1.2…0.0, `dy` −0.7…+1.3 mm. |

## Limit 1 — the Z landing error, ±5 mm, and nothing was measuring it

Downward moves from this run's own `[reached]` lines, mm against command:

```
+0.5  +2.1  +1.0  +4.6  +0.7  +0.2  -7.7  -2.7
```

At the two releases specifically:

| | asked | flange reached | error | block's base vs surface |
|---|---|---|---|---|
| level 0 | 0.1455 | 0.1471 | **+1.6 mm** | +1.6 mm — dropped onto the mat |
| level 1 | 0.1739 | 0.1700 | **−3.9 mm** | **−0.9 mm — INTO the red** |

**The blue was pressed 0.9 mm into the red despite the 3 mm drop.** The run before
was the same shape: +2.8 then −1.4. Level 0 lands high, level 1 lands low, ~5 mm
apart, twice.

A bigger drop is not the fix — it trades digging in for a harder landing, and
`PLACE_DROP_M` was already carrying the whole ±5 mm on its own. **The fix is to
look**, and the number was already on stdout being thrown away, fifteen readings a
run since 2026-08-06, while the arithmetic upstream assumed the arm goes where it
is sent.

`release_z_gap()` now reads `pp.LAST_FLANGE_FK` the instant the descent finishes,
computes where the held block's base actually is against the surface, prints it,
and **lifts by the shortfall before opening the jaws** if it is negative:

```
[stack] descent landed at flange 0.1700, asked for 0.1739 (-3.9 mm). The block's
        base is 0.0205 against a surface of 0.0214: -0.9 mm.
[stack] NEGATIVE -- the block is 0.9 mm INTO level 0. That is what tips a stack.
[stack] lifting 0.9 mm before releasing, so the block is set down rather than
        pressed in.
```

Closed loop, no instrument, no operator, no calibration. And the three numbers now
go into `calibration_history.jsonl` as `place_release_z_achieved`,
`place_release_z_error_mm`, `place_release_gap_mm` — so ten more runs is a fit
rather than ten more scrollbacks.

Both levels are reconstructed in the selftest from the logged flange heights,
including the −0.9 mm.

## Limit 2 — half the survey stills never reach the vote, and the cause is upstream

Across every survey in these logs: **10 of 20 stills fused.** Why the other ten
failed:

```
 2  skipped outright -- no reachable height at z 0.255 OR 0.240
 8  had to drop to z 0.240 -- softer frame, tag modules ~39 px instead of ~57
 7  decoded fewer than 3 tags, so could not VOTE (MULTIVIEW_MIN_TAGS)
 1  produced no homography at all
```

Those are one causal chain, not four problems. And the thing at the top of it is
**the lens re-centring correction**:

| offset | modelled flange r | flange actually flown | pushed out by | IK at z 0.255 |
|---|---|---|---|---|
| +90 | 0.1822 | 0.1828 | +0.6 mm | **yes** |
| +60 | 0.1886 | 0.2031 | +14.5 mm | no → fell back to 0.240 |
| +120 | 0.1886 | 0.2016 | +13.0 mm | no |
| +30 | 0.2052 | 0.2199 | +14.7 mm | no |
| +150 | 0.2050 | 0.2176 | +12.6 mm | no |

**Every still that lost `DETECT_HOVER_Z` lost it to the correction**, not to its own
geometry. Uncorrected, all five sit at 0.1822–0.2052.

And the correction is *structurally* unable to do its job: it is learned from **one
still at one wrist yaw** and applied as a fixed world translation, while the lens
residual has a component that **rotates with the wrist** — ~11 mm amplitude, fitted
over five stills to 0.6 and 1.4 mm rms (see run 9's decomposition). A constant
cannot remove a rotating term, and is wrong by up to ~22 mm at the other yaws.

So it plausibly costs more than it buys, and that is now testable in one run rather
than arguable: **`--no-lens-recentre`**. Run both ways, compare
`[multiview] N usable view(s)`. Default unchanged — this is good arithmetic with no
measurement behind it yet.

## Limit 3 — the place survey is 5× less repeatable than the pickup, and it is J1

Four surveys, decomposed into the axes the arm actually controls:

| zone | J1 | radial spread | tangential spread | tangential as J1 |
|---|---|---|---|---|
| pickup | ≈ 0° | **0.17 mm** | 1.50 mm | 0.39° |
| place | ≈ +90° | 1.78 mm | **8.28 mm** | **2.04°** |

**The error is almost purely tangential at both zones, and tangential is J1.** The
pickup zone is essentially perfect. The place zone is 2° of J1.

Two things follow, and the second matters more than the first:

- **Within a run this does not tip the stack.** Both levels are commanded to the
  same surveyed place XY, so a survey error is common-mode and displaces the stack
  as a unit — design decision 4, and it is why 8 mm of run-to-run spread has never
  shown up as a leaning stack.
- **Between runs it moves where the stack lands** by up to 8 mm. If the stack has
  to land in the same place twice, this is the number to attack.

**It is not yet attributable.** The four values cluster (+4.2, −4.1, −2.9, +2.7 mm)
rather than scattering, which is as consistent with the mat being nudged between
sessions as with J1 hysteresis. **The decisive experiment costs four minutes:**

```bash
python3 stack_blocks.py --by-colour --survey-only     # twice, touching nothing
```

Agree to ~1 mm → the 8 mm is the bench, and the fix is tape. Disagree by 8 mm →
the fix is in code, and `j1_unidirectional_approach` / `J1_RESIDUAL_BIAS_DEG` is
where to look, since 2° is about two lost-motion widths.

## Also fixed — a warning that fired on a good measurement every run

```
[stack] WARNING: measured long side 31.5 mm against a nominal 35.6 mm
```

**A tapered block's nominal footprint is a range, not a number.** The camera sees a
silhouette between the base and the smaller, optically magnified top face, and the
parallax correction then scales the whole thing by a factor derived for a
straight-sided block — so a frustum reads *under* its base every time. Tapered
blocks now get a one-sided band: over the base still warns, under it is expected.
A warning that cries wolf every run is how the real ones get missed.

## The order to attack these in

1. **`--survey-only` twice** (4 min, no code). Splits limit 3 into "bench" or
   "code" and nothing else can.
2. **`--no-lens-recentre` once** (2 min, no code). Doubling the usable views to 4–5
   would improve every position this rig reports, and the arithmetic says it should.
3. **Caliper the three jaw numbers** (2 min). The clearance gate passed at
   +8.5 mm here and refused at −0.2 mm the run before, all computed from guesses.
4. **Ten runs of the new z fields**, then fit the level-dependent z error and
   retire `PLACE_DROP_M` in favour of a correction.

Nothing on that list needs new hardware and only the last needs new code.
