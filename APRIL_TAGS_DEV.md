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

### Not yet verified on hardware

**None of this has moved the arm.** The next run must confirm the tilt actually
improves, and measure what is left. If the remainder is a pose-independent
constant, add it as a fixed offset; if it still varies with reach, the
coefficients are wrong rather than incomplete.

**J1 backlash is untouched and is now the largest single error**: 1.75° =
7.6 mm at 250 mm reach, bigger than the droop's Cartesian effect. It is not
correctable by feedforward *or* feedback — on a reversal the joint does not move
at all (43/62 reversing corrections stalled in test3), so there is no error
signal. The fix is a unidirectional final approach, still not implemented.

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
