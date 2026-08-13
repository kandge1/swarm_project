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
