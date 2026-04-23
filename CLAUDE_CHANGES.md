# CLAUDE_CHANGES

Human-readable log of changes Claude has applied. Newest entries at the top.

## 2026-04-23 — third-person camera reframing + place-target marker for pick-and-place

Goal: ready the scene for a fine-tuning dataset where the task is "pick the {red,blue} ball and place it on the green marker." Camera previously framed the arm base dead-centre (optical axis hit world `(0.141, 0, 0)`), so the balls were ~10° off-centre and the arm filled the top half of the frame. Also, no place-target existed in the world — the earlier cup/pot/sponge layout had been reverted to bare balls.

`src/arm_description/worlds/ball_world.sdf`:
- **Added `place_target`** — green visual-only disc (radius 5 cm, height 1 cm, RGB ≈ 0.15/0.85/0.25) at world `(0.35, 0.00, 0.085)`. `static=true`, collision omitted so a released ball lands on the table at the same XY rather than bouncing off a raised surface. Placement success is a 2-D test (ball XY within marker radius) regardless of Z.
- **Camera pitch `0.73 → 0.81`** on `third_person_camera`. Optical axis now hits ground at `(0.225, 0, 0)`, right between the two balls (x=0.22). Red/blue balls moved from ~10° off-axis to ~4°.
- **Camera HFOV `1.0 → 1.3`** (rad) on the sensor. Widens horizontal coverage from ±28.6° to ±37.2° half-FOV, so the marker + both balls + full arm stay in frame with margin for wrist/gripper jitter during manipulation.

Verified by grabbing a single frame from `/third_person/image_raw` after relaunch: both balls visible in the upper-middle, green marker visible in the lower-middle, arm silhouette in the top third not occluding any manipuland. Framing is symmetric left/right (camera y=0) so red- and blue-pick demos will have mirror-image distributions.

## 2026-04-23 — one-shot launch: `vla/launch/vla_full.launch.py`

Bundles the Gazebo + VLA bring-up into a single command so the two-terminal dance isn't necessary for normal runs. Reduces the "I forgot to export PYTHONPATH" failure mode that cost us a full launch cycle earlier.

- **New file**: `src/vla/launch/vla_full.launch.py`. Actions, in order:
  1. `SetEnvironmentVariable` prepends `venv_site_packages` (default `/home/peyton-peyton/vla_arm/.venv/lib/python3.12/site-packages`) to `PYTHONPATH`, so the child node processes inherit it. This is what makes `json_numpy` importable in the VLA nodes without manual `source .venv/bin/activate`.
  2. `IncludeLaunchDescription(arm_description/arm_gazebo.launch.py)` — starts at t=0. Forwards `render_engine` arg.
  3. `TimerAction(period=vla_start_delay)` → `IncludeLaunchDescription(vla/vla.launch.py)` — default 15 s delay, which safely clears the 6 s internal spawner timer inside `arm_gazebo.launch.py` plus whatever gz_sim takes to come up.
- **Declared args** (all overrideable): `venv_site_packages`, `vla_start_delay`, `render_engine`.
- **Usage**: `ros2 launch vla vla_full.launch.py`. No more manual `PYTHONPATH` exports.
- `src/vla/CMakeLists.txt` already installs `launch/*` via `install(DIRECTORY launch …)` — the new file is picked up automatically.
- Updated `run_commands.txt` to promote `vla_full.launch.py` as the primary path and keep the individual launches as a debug-only fallback.

Build passes. Not yet exercised end-to-end against a live sim.

## 2026-04-23 — reverted Bridge V2 scene dressing

Rolled back `ball_world.sdf` to the pre-dressing state (simple spheres, flat brown table, single directional sun, no backsplash/clutter/fill light, camera at `(0.70, 0, 0.50)` with pitch `0.73`). World name restored to `balls_world`. Kept the 4 ms physics step and the third-person camera. Also reverted `vla_action_client.py` default instruction + `vla.launch.py` phase schedule back to "blue ball" (from "blue cup").

Reason: the Bridge-like scene (extra fill light + more objects + complex materials) pushed `gz sim server` to 100% CPU (single-threaded), tanking RTF from ~1.05 to ~0.36. Lighter-weight partial-rollback didn't recover RTF either, and the visual authenticity gain wasn't worth the RTF cost given the embodiment gap (SO-101 vs WidowX) still dominates.

Build passes. SDF XML parses.

## 2026-04-23 — Bridge V2-style scene dressing (`ball_world.sdf`)

Goal: reduce the OOD gap between what OpenVLA-bridge_orig was trained on (toy-kitchen tabletop scenes with everyday objects) and the current scene (two abstract spheres on a flat brown table), so its language-conditioned actions have a chance to land on the right region.

- **World renamed** `balls_world → bridge_like_world`.
- **Lighting**: added `<scene>` block with ambient `0.5` grey and background `0.75`; warmed the sun's diffuse to `(0.8, 0.78, 0.72)` with a slightly more lateral direction; added a point `fill_light` at `(0.35, 0.30, 0.75)` to lift shadows.
- **Table**: widened to `0.40 × 0.55 × 0.08` and recolored to a lighter wood tone (`diffuse 0.78 0.60 0.42`).
- **Backsplash**: new `backsplash` model — a thin `0.02 × 1.20 × 0.55` panel at world `(-0.22, 0, 0.275)`, cream color, sits behind the arm from the camera's POV (Bridge V2 training frames always have a painted/tiled wall in the background).
- **Counter edge**: thin dark strip `counter_edge` between the table back and the backsplash, for visual separation.
- **Primary/target objects**: `red_ball` → `red_cup`, `blue_ball` → `blue_cup` — both 0.025 m radius × 0.07 m tall cylinders (mug-shaped) at `(0.25, ±0.09, 0.115)`. Blue is the task target; red is a distractor.
- **Clutter**: added a static black `pot` cylinder (`r=0.04, h=0.09`) at `(0.38, 0.18, 0.125)` and a yellow `sponge` box at `(0.40, -0.18, 0.11)` — Bridge scenes always have several non-target props on the counter.
- **Camera**: moved from `(0.70, 0, 0.50)` with pitch `0.73` to `(0.85, 0, 0.55)` with pitch `0.644` — slightly further back and shallower angle, so all objects + arm fit comfortably in frame at the ~45° overhead angle typical of Bridge V2 training data. 224×224 @ 5 Hz unchanged.
- **Removed** the floating `test_ball` that was clearly a debugging artifact.

Also updated `vla_action_client.py` default + `vla.launch.py` phase schedule to target "the blue cup" instead of "the blue ball".

Caveats unchanged from the OOD analysis: SO-101 robot is still visible in frame (Bridge was trained with WidowX), and `bridge_orig` action magnitudes are still calibrated to WidowX's reach, not SO-101's. This scene change should improve language grounding and object recognition but will not close the embodiment gap.

Build verified (`colcon build --symlink-install`). SDF XML parses.

## 2026-04-23 — add `run_commands.txt` (bring-up runbook)

Verified the VLA pipeline end-to-end against the live OpenVLA server (ogre2 default, RTF ~1.05, controllers active, client → action_to_ee → vla_ik → joint motion all flowing). Persisted the bring-up sequence as `run_commands.txt` in the repo root so the recovery path is reproducible without re-deriving it.

Runbook covers: (a) the "TERM 0" nuke-and-reset needed to recover from ghost controller_managers after a crashed sim, (b) building, (c) the SSH tunnel, (d) Gazebo launch (with the note that `arm_controller` FATAL on spawn is cosmetic — gz_ros2_control auto-activates from `controllers.yaml`), (e) the VLA launch with `PYTHONPATH=<venv>/site-packages:$PYTHONPATH` (activating the venv after sourcing ROS doesn't stick; only the prepended-PYTHONPATH approach worked reliably), (f) liveness/hz verification commands, and (g) notes on the OOD-domain gotcha (targets wandering around `(0.45, -0.05, 0.22)` while blue ball is at `(0.22, +0.10, 0.11)` — bridge_orig is out of distribution for SO-101 + sphere scene).

## 2026-04-23 — restore `ogre2` as default render engine in `arm_gazebo.launch.py`

`src/arm_description/launch/arm_gazebo.launch.py`: flipped `render_engine` `default_value` from `'ogre'` back to `'ogre2'` and dropped the "ogre2 was slower, likely software-GL" note from the description.

The stale note was a red herring. RTF-investigation diagnostics showed the NVIDIA driver (590.48.01) exposes a working NVIDIA EGL/GL stack (`GL_VENDOR=NVIDIA Corporation`, `GL_RENDERER=NVIDIA GeForce RTX 2060/PCIe/SSE2` via EGL pbuffer probe). The previously observed "ogre2 slowness" was caused by multiple concurrent `gz sim` instances + the VLA pipeline competing for CPU, not by a software-GL fallback. With a single clean ogre2 instance, RTF measured ~0.97 (vs ~0.23 with the zombie-duplicates-plus-ogre baseline) and the GPU moved from P5 → P0 at ~42 W draw.

Ogre2 still emits `libEGL warning: egl: failed to create dri2 screen` messages on startup — non-fatal; Ogre2 falls back to a working EGL surface path. Not addressed here.

## 2026-04-22 — scripted instruction curriculum for blue-ball pick

`src/vla/vla/vla_action_client.py` now sends a time-varying instruction instead of a single static string, so the prompt can guide OpenVLA through the sub-tasks of a pick.

- New parameters: `phase_durations: double[]`, `phase_instructions: string[]`, `loop_phases: bool`. If either list is empty (or they're different lengths, which logs a warning), the node falls back to the single `instruction` parameter.
- The schedule timer starts on the first received camera frame, not node start, so it doesn't elapse while Gazebo is still coming up. Elapsed-time arithmetic uses `self.get_clock()` so it respects `use_sim_time`.
- Phase transitions are logged (`Phase N: "..."`) and the active instruction is embedded in the `/openvla/status` message for offline inspection.
- Default schedule in `vla.launch.py` for the blue-ball pick: 12 s "move the arm above the blue ball" → 8 s "lower the gripper onto the blue ball" → 4 s "close the gripper on the blue ball" → 6 s "lift the blue ball". The last phase persists indefinitely after its window (loop is off). Fallback static `instruction` is now `"pick up the blue ball"`.

Build verified via `colcon build --packages-select vla --symlink-install`.

## 2026-04-22 — align pipeline with OpenVLA `bridge_orig` conventions

Audit against SimplerEnv's OpenVLA policy adapter (canonical reference for `unnorm_key='bridge_orig'`) revealed three real semantic mismatches in the pipeline. Fixed all three plus the upstream camera-view domain gap. Euler convention and workspace reachability were re-checked and found already correct, so not touched.

- **Third-person fixed camera added to the world.** `src/arm_description/worlds/ball_world.sdf` now contains a static `third_person_camera` model at world `(0.70, 0.00, 0.50)` with rpy `(0, 0.73, π)`, publishing a 224×224 @ 5 Hz RGB stream on `/third_person/image_raw`. Matches Bridge V2's over-the-shoulder tabletop viewpoint (which is what OpenVLA-7B was actually trained on); the wrist camera is still present on `/camera/image_raw` for other uses. Square aspect ratio was chosen to avoid the 4:3→1:1 squash the model's preprocessor would otherwise apply.
- **Bridge plumbing for the new camera.** `src/arm_description/launch/arm_gazebo.launch.py` gained a `third_person_bridge` `ros_gz_bridge` node for `/third_person/image_raw`, added to the `LaunchDescription`.
- **VLA client now consumes the third-person view.** `src/vla/launch/vla.launch.py` default `camera_topic` switched from `/camera/image_raw` to `/third_person/image_raw`. This is the single biggest expected quality lift — OpenVLA was essentially blind to the previous wrist-cam frames.
- **Gripper convention fix.** OpenVLA bridge_orig returns gripper in ~[0, 1] with `>0.5 ⇒ OPEN` (per `simpler_env/policies/openvla/openvla_model.py`, `2*(open_gripper > 0.5) - 1`). Previously the pipeline treated `>0.0` as "closed" and piped the raw value straight into the gripper joint as a position target. Now:
  - `src/vla/vla/vla_action_client.py`: `gripper_threshold` default `0.0 → 0.5`, status label `gripper_closed → gripper_open`.
  - `src/vla/vla/vla_ik.py` `gripper_cb`: binarizes at threshold and maps to `gripper_open_position` (default `1.745`) or `gripper_closed_position` (default `-0.174`). The old `gripper_min/max` clamp and `default_gripper` param are gone.
  - `src/vla/launch/vla.launch.py` VLA-IK params updated: `gripper_open_position`, `gripper_closed_position`, `gripper_threshold`.
- **IK now honors target orientation.** Previously `vla_ik.py` read only `pose.position` and held `wrist_flex=0.5`, `wrist_roll=0.0` constant, discarding the 3 rotation components OpenVLA predicted. Rewrote as full 5-DOF numerical IK:
  - `fk_pose(q5)` returns `(position, 3x3 rotation)` of `gripper_frame_link`; chain unchanged.
  - New `quat_to_mat` and `rot_log` helpers (axis-angle log map with near-π branch).
  - `solve_ik(target_pos, target_rot)` minimizes the stacked residual `[pos_err; orientation_weight * rot_log(R_target^T @ R_current)]` over all 5 joints, bounded by URDF limits. `orientation_weight=0.5` default, settable via param; `0.0` falls back to position-only IK.
  - Warm start is now 5-vector `[pan, lift, elbow, wrist_flex, wrist_roll]` with `wrist_flex_seed`/`wrist_roll_seed` params for the initial guess.
  - Deadband extended to detect orientation changes (quaternion dot-product distance) in addition to position.
  - Residual reporting now splits position (m) and rotation (rad) components.
- **What was audited and left alone.**
  - `action_to_ee.py`'s `quat_from_euler` is aerospace ZYX (`R = Rz(yaw)·Ry(pitch)·Rx(roll)`), which is mathematically identical to `transforms3d` `sxyz` extrinsic XYZ that SimplerEnv uses with `euler2axangle`. No change.
  - Embodiment mismatch (bridge_orig un-normalizes to WidowX scale, not SO-101) is not fixable without either fine-tuning or custom normalization stats; the existing `cmds.txt` workaround `translation_scale:=0.01` remains a viable knob. Flagged, not fixed.
  - Workspace reachability: shoulder at world `(0.039, 0, 0.062)`, balls at `(0.22, ±0.10, 0.11)`, distance ≈ 0.21 m vs arm reach ≈ 0.4 m. Already reachable since the 2026-04-21 table/ball changes. The CLAUDE.md "green ball is out of reach" gotcha is stale and applies only to the pre-table world.

### Build/launch

`colcon build --symlink-install` passes. To run end-to-end:

```bash
ros2 launch arm_description arm_gazebo.launch.py   # world, bridges (incl. /third_person/image_raw)
ros2 launch vla vla.launch.py                      # client → action_to_ee → full-DOF IK
```

Assumes the OpenVLA SSH tunnel on port 8000 is active (`cmds.txt`).

### Known next steps / risks

- Third-person camera pose `(0.70, 0, 0.50)` with pitch `0.73 rad` is a reasonable initial framing but was not visually verified against live Gazebo — may need tweaking so the arm + table + balls all sit comfortably in frame.
- Orientation-aware IK with `orientation_weight=0.5` is a starting point. If the arm struggles to reach desired positions because orientation constraints dominate, lower the weight; if orientation drifts, raise it.
- Gripper open/closed joint-angle mapping (`1.745` / `-0.174`) was inferred from URDF limits; if the physical sign is flipped (i.e. `-0.174` is open in practice), swap the two params in `vla.launch.py`.
- WidowX-vs-SO-101 action scale mismatch is still present; expect to need `translation_scale ≈ 0.01–0.1` in the `action_to_ee` node once real OpenVLA output starts flowing.

## 2026-04-21 — numerical FK/IK, gripper wiring, red-ball grab tuning (paused)

User asked me to fix the "IK is too simplified" limitation from the previous entry, then iterate on the grab until the arm actually grasps a ball. Paused mid-tuning.

### What works now

- **`src/vla/vla/vla_ik.py` — full numerical FK/IK.** Replaced the analytic 3-link IK with scipy.optimize.least_squares targeting `gripper_frame_link` directly. FK hardcodes the URDF joint chain (base_link → shoulder_pan → shoulder_lift → elbow_flex → wrist_flex → wrist_roll → gripper_frame_link). Verified FK matches observed rest-pose EE at base_link `(0.391, 0, 0.226)` exactly. Solver uses joint-limit bounds from the URDF (`shoulder_pan_min/max` etc. now params) and warm-starts from the previous solution. Old `shoulder_offset_x/z` and `upper/lower_arm_length` params are gone.
- **`src/vla/vla/vla_ik.py` — pose_cb is now the single trajectory publisher.** `gripper_cb` used to publish its own "gripper-only update" trajectory, which raced with `pose_cb` and produced chaotic flapping (trajectories flipping between gripper=-0.17 and 0.8 every few ms). Now `gripper_cb` just updates `self.current_gripper`; `pose_cb` publishes whenever *either* the target position *or* the gripper has changed past threshold. Added `last_published_gripper` for the deadband.
- **`src/vla/vla/fake_vla_action_client.py` — approach-from-behind keyframe + configurable gripper values.** Added `grab_approach_offset_x` (hover/descend land at `target + (offset_x, 0, 0)`) and a new `enclose` phase that sweeps from `behind` to `touch` with jaws still open. Added `grab_gripper_open`/`grab_gripper_closed` params (now `-0.17` and `0.8` — URDF range is `[-0.174, 1.745]`; open=most-negative).
- **`src/vla/launch/vla_fake.launch.py` — `use_sim_time: True` on all three VLA nodes.** Critical for this workspace because sim RTF is ~30% (confirmed via `gz topic -t /stats`). Without this, the fake's tick timer runs at wall-clock rate while the controller runs at sim-clock rate, so the scripted trajectory advances ~3.3× faster than the arm can execute — arm lags massively behind target and never reaches the ball. Also moved starting-pose gripper from `0.0` to `-0.17` (open).
- **`src/arm_description/worlds/ball_world.sdf` — lower table, repositioned balls, green removed per user.** Table now `0.30 × 0.40 × 0.08` at center `(0.25, 0, 0.04)` — top surface at world `z=0.08`. Balls at `z=0.11`. Currently only red `(0.22, -0.10, 0.11)` and blue `(0.22, 0.10, 0.11)` remain; `green_ball` was deleted to simplify target selection. The pre-existing `test_ball` floating at `(0.40, 0, 0.65)` is untouched.

### What's still open

User's three directional feedback comments, in order, during this session:
1. *"arm is nudging ball because balls are too high up"* — addressed by lowering balls to `z=0.11` (from `z=0.18`).
2. *"arm needs to move behind ball before grabbing, still pushing it by accident"* — addressed by the `enclose` phase + `grab_approach_offset_x=0.04`.
3. *"needs to approach ball from upper angle"* — **NOT YET ADDRESSED.** Current trajectory is mostly diagonal from the starting pose. User wants a more top-down descent; haven't tried raising the hover height or bumping `wrist_flex` (currently `0.5`) toward `1.0`–`1.2` so the gripper points more straight down.

Current result state (last confirmed run, red-ball target, with `use_sim_time` enabled, waited ~60 s wall = ~18 s sim): red and blue balls untouched at their original positions. Arm trajectory was smooth and residual=0 throughout. Did not observe the last few seconds of the sequence before the user paused. Don't know whether the arm grabbed, missed, or finished mid-sequence.

### Resume points

- Verify the last run's endpoint (EE pose, joint_states, ball positions) once sim is back up.
- Try `wrist_flex: 1.0` (more downward-pointing gripper) and `grab_hover_height: 0.20` (hover farther above so descent is more vertical).
- If still missing: target the ball not at its center but slightly above (e.g. ball_z + 0.015) so gripper_frame_link sits between the jaws with room to close.
- Fallback diagnostic: publish a hand-precomputed multi-waypoint `JointTrajectory` directly to `/arm_controller/joint_trajectory` to confirm the gripper geometry can actually enclose a 3 cm ball at this approach orientation. If it can't, the fix is a wrist angle / target offset issue, not a pipeline one.
- The ~30% RTF itself is worth investigating — probably the wrist camera under `ogre` (CPU) render engine. `render_engine:=ogre2` should help but needs a working GPU/EGL setup.

## 2026-04-21 — table in world + TF-relative grab + starting-pose + sim verification

Raised the balls onto a table, made the fake grab trajectory work from any starting pose (TF-relative), wired a starting-pose command into the fake launch, and ran the sim end-to-end to verify.

- **`src/arm_description/worlds/ball_world.sdf`** — added a static `table` box at world `(0.25, 0, 0.10)`, size `0.30 × 0.40 × 0.10` (top at `z=0.15`). Moved balls onto the tabletop at `z=0.18`: green at `(0.25, 0, 0.18)`, red at `(0.22, -0.10, 0.18)`, blue at `(0.22, 0.10, 0.18)`. Positions chosen so each ball is within ~0.23 m of the shoulder pivot (inside the simplified IK's 0.255 m reach envelope).
- **`src/vla/vla/fake_vla_action_client.py`** — grab mode now does a TF lookup (`base_link` → `gripper_frame_link`) every tick. The first keyframe is seeded with the current EE pose on first successful lookup, and the trajectory clock is reset so the approach ramps *from where the arm is*. Each tick emits `delta = scripted_target(t) − cur_ee_pos`, magnitude-clamped to `grab_max_step` (default 0.04 m). Added tf2 dependency, new params `grab_base_frame`, `grab_ee_frame`, `grab_max_step`. Made the file executable (fixed a silent-failure mode — `--symlink-install` preserves source permissions, and `ros2 launch` requires the script bit).
- **`src/vla/vla/vla_ik.py`** — added a `/vla_gripper` subscriber (`Float32`). When the gripper value changes by more than `gripper_change_threshold` (default 0.02 rad), republishes the last joint trajectory with only the gripper joint updated. Without this, the fake's gripper commands during the grip phase had no effect on the physical joint. New params: `gripper_topic`, `gripper_min`, `gripper_max`, `gripper_change_threshold`.
- **`src/vla/launch/vla_fake.launch.py`** — now a two-phase launch: (1) an `ExecuteProcess` publishes a one-shot JointTrajectory to drive the arm to the "viewing" pose `[0, -1, 1, 0.5, 0, 0]` from `cmds.txt`; (2) a `TimerAction` (4 s delay) brings up the fake client, action_to_ee, and vla_ik. Also passes `gripper_topic` to vla_ik.

### Verification run

Launched `arm_gazebo.launch.py` then `vla_fake.launch.py`. Captured from logs:

- Grab trajectory seeded at EE `(0.323, 0, 0.184)` — starting-pose command is working; arm is bent down, not at the default extended-forward rest pose.
- `action_to_ee` published 30+ EE targets; deltas progress smoothly from `(-0.007, 0, -0.009)` to the clamped `(-0.030, 0, -0.027)` as the arm descends toward the ball.
- `vla_ik` published joint trajectories every tick during motion, then switched to a "gripper-only update" at t ≈ 8 s when the fake's grip phase began — confirming the `/vla_gripper` pathway works.
- Final `joint_states`: `gripper = 1.0` rad (closed), `wrist_flex = 0.5`, other joints near 0 (arm extended roughly along +x).
- Final green-ball pose: `(0.2416, −0.0025, 0.180)` with non-zero RPY `(0.10, −0.30, 0.51)` — displaced by ~9 mm and rotated. Red and blue untouched. The arm *made contact* with the green ball but only brushed it rather than grasping it.

### Known limitation surfaced during testing

The analytic 3-DOF IK in `vla_ik.py` is a poor match for the real URDF. At the sim's rest pose (all joints 0), TF reports EE at base_link `(0.391, 0, 0.226)` — the fixed offsets in `gripper_link` + `gripper_frame_joint` + `wrist_roll` add ~0.09 m beyond the IK's `upper + lower = 0.255 m` reach model. Consequence: the IK targets the *wrist*, the gripper is ~9 cm further out, and the fake's "put gripper_frame at ball" command actually parks the wrist short of the ball and the gripper jaws past it. Not a pipeline bug — the fake is emitting correct intent — but precise grasping needs either (a) offsetting the fake's target inward by the EE-to-wrist distance, or (b) replacing the IK with something that matches this URDF (real 6-DOF IK or an MoveIt-based solver).

## 2026-04-21 — grab-green-ball mode + remap bugfix

Extended the fake client with a scripted grab trajectory toward the green ball, and fixed the blocking issues in `action_to_ee.py` that made 3D delta control impossible. Verified by offline replay: max per-tick step 0.0375 m (under 0.05 clamp), cumulative target reaches lift position, gripper flips to 1.0 at t=9s.

- **`src/vla/vla/action_to_ee.py`** — removed the "better debug mapping guess" that overwrote `dx/dy/dz` with `(0.2*raw_dx, -1.0*raw_dx, 1.0*raw_dy)` and dropped `raw_dz` entirely. Translation deltas are now identity pass-through in `base_link` frame. Also removed the dead `delta_world = rotate_vector_by_quat(...)` computation (previously flagged in CLAUDE.md gotchas). Rotation deltas unchanged.
- **`src/vla/vla/fake_vla_action_client.py`** — added a `grab` mode with a 5-phase keyframe trajectory (approach → descend → grip → lift → hold) and linear interpolation between keyframes. Emits `delta = target(t) - target(t-dt)` each tick, so cumulative deltas in `action_to_ee` reach the keyframe targets. Target, hover height, lift height, and phase durations are all ROS params. Defaults target the green ball at base_link `(0.45, 0, -0.17)`.
- **`src/vla/launch/vla_fake.launch.py`** — switched default mode to `grab` at 2 Hz with params tuned for the green ball. Total sequence ~15.5 s.
- **`CLAUDE.md`** — replaced the two remap-related gotchas with a single one documenting the new identity-pass-through convention and the path to re-introduce EE-frame deltas. Added a new gotcha: green ball (and presumably red/blue) is out of reach from the default spawn because the arm is 0.2 m above ground and the balls are 0.03 m — suggested fixes are lowering spawn z or raising balls onto a table.

Heads up: the grab will *not* actually grasp the ball as the world/spawn are currently configured. The fake node emits the right sequence, but the IK reach clamp will bottom out ~`0.26, 0, -0.06` in base_link — EE will hover well above the ball. This is geometry, not a code bug.

## 2026-04-21 — fake VLA action client for offline sim testing

Added a drop-in stub so the Gazebo pipeline can be exercised without the OpenVLA SSH tunnel up. Same wire contract as `vla_action_client.py` (7D `Float32MultiArray` on `/vla_action`, matching `deploy.py`'s `[dx, dy, dz, drx, dry, drz, gripper]`), so `action_to_ee` + `vla_ik` are unmodified.

- **`src/vla/vla/fake_vla_action_client.py`** (new) — timer-driven publisher. Param `mode` switches between `sine` (default, Lissajous oscillating deltas so integrated motion stays bounded), `constant` (fixed delta from params), and `zero` (useful for verifying the IK deadband). Defaults to `translation_amplitude=0.02 m`, `rotation_amplitude=0.0`, `publish_rate_hz=1.0`, gripper held at 0.0.
- **`src/vla/launch/vla_fake.launch.py`** (new) — mirror of `vla.launch.py` but swaps in the fake client. Run with `ros2 launch vla vla_fake.launch.py` after starting `arm_gazebo.launch.py`.
- **`src/vla/CMakeLists.txt`** — added `fake_vla_action_client.py` to `install(PROGRAMS ...)`.

## 2026-04-21 — post-pull evaluation fixes

Applied the fixes identified in the `01c97f6` repo evaluation. Verified by `colcon build --packages-select vla --symlink-install` and an `ast.parse` sweep of the vla Python files.

- **`src/vla/launch/vla.launch.py`** — fixed `package='your_package'` → `package='vla'` on the third Node entry (the IK node). Launch no longer fails with a package-not-found error.
- **`src/vla/vla/action_to_ee.py`** — defined `droll, dpitch, dyaw` from `data[3:6]` with `rotation_scale` and `max_rotation_step` clamping, matching the translation path. `action_cb` no longer raises `NameError` on the first `/vla_action` message.
- **`src/vla/vla/ik.py`** — deleted. File was a syntactically broken stub (`self.declare_parameter('shoulder_x'. 0.5)` — period instead of comma, statement outside `__init__`). Not in `CMakeLists.txt`, not imported anywhere. `vla_ik.py` is the real IK node.
- **`src/vla/config/vla_params.yaml`** — deleted. Orphaned since the launch file switched to inline parameter dicts; still listed the removed `action_bridge` node, the non-existent `joint_trajectory_controller` topic, and the wrong joint names (`joint1..joint6`). Config directory removed.
- **`src/vla/CMakeLists.txt`** — dropped `config` from `install(DIRECTORY launch config ...)` since the directory no longer exists.
- **`src/vla/package.xml`** — added `geometry_msgs` and `tf2_ros` to `<exec_depend>` (both imported by `action_to_ee.py`; previously only transitively available).
- **`CLAUDE.md`** — removed the now-resolved Gotchas (wrong package name, undefined euler vars, stub `ik.py`, orphaned `vla_params.yaml`) and the "Currently broken" markers in the Key-files list. Added a gotcha about `delta_world` being computed-but-unused in `action_to_ee.py` (noticed during review; left alone because fixing it is a frame-convention decision, not a bugfix).
- **`README.md`** — synced to match the post-fix tree. Dropped the deleted `config/vla_params.yaml` line from the repo-layout block; no warning callouts needed since the launch file and `action_to_ee.py` run as-is now.

Left alone: the "better debug mapping guess" translation remap in `action_to_ee.py:127-129` — experimental and already flagged in the gotchas; not a correctness bug.
