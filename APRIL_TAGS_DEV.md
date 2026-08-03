# APRIL_TAGS_DEV.md — working state, terminals, and the open bug

Companion to `APRIL_TAGS.md`. That file is the **design**: why the homography
approach was chosen, the staging, the long-form justifications. This file is the
**bench state**: which machine runs what, what currently works, what is broken
right now, and the exact commands to reproduce and fix it.

Written 2026-08-02. If you are a fresh session picking this up, read "Where we
are" and "THE OPEN BUG" first — everything else is reference.

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

## Offline tests (no robot, no mars launch needed)

```bash
cd ~/swarm/swarm_project/src/swarm_pkg/src/scripts
python3 zone_vision_selftest.py     # synthetic geometry regression, ~1 s, expect "0 failure(s)"
python3 -m py_compile tag_pick_place.py pick_place.py zone_vision.py block_detector_node.py
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

1. Run the 5-second serialization check on the robot. Confirm or refute.
2. Clean-rebuild `swarm_interfaces` on the robot; restart Terminal 4.
3. `ros2 service call /detect_block ...` from Terminal 5 with the zone in view —
   it should return a populated reply.
4. `tag_pick_place.py --dry-run` from Terminal 5. Look at the jaws.
5. Drop `--dry-run`. **First descent ever attempted.** Watch it.
