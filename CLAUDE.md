# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is right now

A ROS 2 Jazzy workspace for driving a simulated **SO-101** 6-DOF arm with
**SmolVLA**. Fine-tune `lerobot/smolvla_base` on Gazebo-collected
pick-and-place demos, then run the resulting policy closed-loop in the
sim. The expert is a hand-tuned **joint-keyframe** scripted policy (no
IK, no MoveIt in the runtime path); the recorder captures
image + joint-state + action tuples into a LeRobot v2 dataset; the
fine-tuned policy emits absolute joint positions directly.

## Workspace layout

ROS 2 **Jazzy** colcon workspace with two packages under `src/`:

- `arm_description` — SO-101 6-DOF arm URDF (xacro), Gazebo Sim (gz) world
  with two graspable cubes + a green place marker + a third-person camera,
  `ros2_control` config, and the base launch files (Gazebo + RViz).
- `vla` — Python ROS 2 nodes. Contains the scripted keyframe expert
  (`pick_and_place_moveit.py` — name is historical, no MoveIt in it),
  the demo recorder (`demo_recorder.py`), and the SmolVLA inference
  node (`smolvla_inference_node.py`).

All three use `ament_cmake` (not `ament_python`); `vla` installs its Python
modules via `ament_python_install_package` and individual scripts via
`install(PROGRAMS ...)` — editing `vla/vla/*.py` only survives `colcon build`
if it's listed there.

## Data flow (SmolVLA pipeline — primary)

Demo collection (complete: 200 successful episodes →
`data/demos_lerobot_final/`, collected with the **pre-2026-04-24
IK-based expert**):

```
arm_description (Gazebo + arm + controllers)
        │
        ▼
pick_and_place_moveit.py   hand-tuned 11-step joint-keyframe sequence
        │                  (home → pre-approach → above → grasp → close →
        │                   lift → above_marker → place → release →
        │                   retreat → home). Reads actual ball XY via
        │                  `gz model -p` before grasp and linearly
        │                  corrects shoulder_pan + shoulder_lift against
        │                  canonical keyframes. Friction grasp — no
        │                  DetachableJoint calls.
        ▼
joint_trajectory actions  → /arm_controller/follow_joint_trajectory (5 joints) +
                            /gripper_controller/follow_joint_trajectory (1 joint)
        │
        ▼
demo_recorder.py   subscribes to /third_person/image_raw, /joint_states,
        │          and the two joint_trajectory topics; writes synchronized
        │          tuples at 10 Hz into data/demos_raw_*/episode_NNNN/.
        ▼
scripts/convert_to_lerobot.py   → data/demos_lerobot_final/
                                  (parquet/episode + h264 video/episode +
                                   meta/{info,episodes,tasks,stats}.json)
```

Training + inference (both implemented; first closed-loop eval failed,
diagnosis in `NEXT_STEPS.md`):

```
data/demos_lerobot_final/  ──►  python train_relaxed.py
                                --policy.path=lerobot/smolvla_base
                                (~3 h on a single A100, batch 64, 30k steps)
                                ──►  outputs/.../checkpoints/030000/
                                         pretrained_model
                                                │
                                                ▼
                                  smolvla_inference_node
                                  loads SmolVLAPolicy.from_pretrained(...);
                                  subscribes to /third_person/image_raw +
                                  /joint_states; calls policy.select_action
                                  on a 10 Hz timer; publishes 6-D joints
                                  split across
                                  /arm_controller/joint_trajectory (5) +
                                  /gripper_controller/joint_trajectory (1).
```

There is no IK or EE-pose math in either the expert or the SmolVLA
inference path — both emit joint positions directly.

## Common commands

All commands assume `cd /home/peyton/vla_arm` and that ROS 2 Jazzy is
installed at `/opt/ros/jazzy`. Build before everything:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit launch / config / urdf / world files
without rebuilding. Re-build after editing Python nodes or
`CMakeLists.txt`. To build a single package:

```bash
colcon build --packages-select vla --symlink-install
```

### Sim launchers

```bash
# Gazebo + arm + controllers — use this for the keyframe expert, demo
# collection, and the SmolVLA inference node.
ros2 launch arm_description arm_gazebo.launch.py

# Arm + RViz with joint sliders (no physics)
ros2 launch arm_description arm_rviz2.launch.py
```

### Clean teardown (mandatory before every test — see clean-slate rule)

```bash
bash scripts/nuke_sim.sh
```

### Demo collection (SmolVLA path)

```bash
# Terminal 1 — launch the sim:
ros2 launch arm_description arm_gazebo.launch.py

# Terminal 2 — recorder (one episode dir per episode):
ros2 run vla demo_recorder.py --ros-args \
    -p output_dir:=/home/peyton/vla_arm/data/demos_raw \
    -p record_rate_hz:=10.0

# Terminal 3 — keyframe expert (logs to /tmp/collect_progress.log via tee):
ros2 run vla pick_and_place_moveit.py --trials 25 --randomize --seed 0

# Or batch wrapper that restarts the sim every 25 trials and stops at
# --target successes:
bash scripts/collect_200_demos.sh --target 200 --batch-size 25 \
    --output /home/peyton/vla_arm/data/demos_raw_final
```

### Filter + convert demos to LeRobot v2

```bash
python3 scripts/filter_successful_demos.py \
    --progress-log /tmp/collect_progress.log \
    --batches-root /home/peyton/vla_arm/data/demos_raw_final \
    --output /home/peyton/vla_arm/data/demos_successful

python3 scripts/convert_to_lerobot.py \
    --input-dir /home/peyton/vla_arm/data/demos_successful \
    --output-dir /home/peyton/vla_arm/data/demos_lerobot_final \
    --fps 10
```

The canonical artifact is **`data/demos_lerobot_final/`** — 200 episodes,
44,202 frames, ~15 MB, LeRobot v2 schema. Collected with the pre-2026-04-24
IK-based expert; a fresh collection with the new keyframe expert is likely
to have a better blue/red split and higher success rate but has not been
run yet.

### SmolVLA finetune (on UCF nobel)

Compute target is UCF nobel (`pe606840@nobel.ece.ucf.edu`). Training
runs inside tmux session `smolvla` via a wrapper script that patches
four LeRobot 0.3.3 / dataset incompatibilities (timestamp tolerance,
video decoder tolerance, parquet/mp4 off-by-one, missing image stats).
**Do not invoke `python -m lerobot.scripts.train` directly — use
`train_relaxed.py`.** Full runbook in `NEXT_STEPS.md`.

Short version:

```bash
# on nobel, inside ~/vla_smolvla with .venv active, inside tmux:
CUDA_VISIBLE_DEVICES=0 python train_relaxed.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.root=/home/pe606840/vla_smolvla/data/demos_lerobot_final \
    --dataset.repo_id=local/so101_pickplace_sim \
    --dataset.use_imagenet_stats=false \
    --batch_size=64 --steps=30000 --eval_freq=-1 --save_freq=5000 \
    --output_dir=outputs/train/smolvla_so101 \
    --policy.push_to_hub=false --wandb.enable=false
```

`lerobot[smolvla]==0.3.3` — pin that version. 0.4.x restructured the
dataset format to v3.0 and breaks on our v2.0 dataset.

### Diagnostics

```bash
xacro src/arm_description/description/so101.urdf.xacro -o /tmp/so101.urdf
gz topic -e -t /stats -n 5            # real-time factor (sim must be up)
ros2 control list_controllers          # arm_controller / gripper_controller / jsb
gz model -m blue_ball -p              # cube pose; sanity-check between resets
ros2 run tf2_ros tf2_echo base_link tcp_jaw_link
```

There are no tests, linter configs, or CI in this repo.

## Mandatory rule — clean slate before every sim test

Before running anything against Gazebo (the expert policy, the recorder,
the SmolVLA inference node), verify the sim is a **clean slate** — not
just "fresh launch", a full tear-down + state check.

Why: state leaks across restarts in two specific ways we've hit:

1. **Ghost controller_manager / move_group / gz processes on DDS.**
   `ros2 launch` can die but `controller_manager`, `move_group`,
   `parameter_bridge`, `robot_state_publisher`, `gz sim server`,
   `ruby.*gz` get reparented to PID 1 and keep holding state. A new
   Gazebo starts on top of that, and `/joint_states` reports whatever the
   ghosts were commanding, not the URDF zero pose. `pkill -f 'ros2
   launch'` alone misses orphans — you must `kill -9 <PIDs>` after `ps
   -ef`. `scripts/nuke_sim.sh` does exactly this plus a DDS daemon flush.

2. **Gazebo world state mismatches visual state.** `gz model -p` may
   return the SDF default pose when the live runtime state is something
   else.

How to apply, every time:

1. `bash scripts/nuke_sim.sh` (TERM 0 reset + explicit `kill -9` + DDS
   daemon stop/start).
2. Relaunch the sim.
3. **Verify** after startup: home joints ≈ `[0,0,0,0,0,0]` (tol ~0.05
   rad); target cube pose from `gz model -m <ball> -p` matches SDF
   default. If either is off, reset again — don't run the test on stale
   state.

Memory entry `feedback_sim_clean_slate.md` mirrors this rule.

## World state (already set up — change with care)

Defaults from `src/arm_description/worlds/ball_world.sdf`:

```
red_ball       3 cm cube visual + collision, mass 50 g, μ=2.0
                 at (0.22, -0.08, 0.10)
blue_ball      same at (0.22, +0.08, 0.10)
place_target   green disc r=5 cm, visual-only (no collision), static
                 at (0.28, 0, 0.085)
table          30×40×8 cm at (0.25, 0, 0.04)
third_person_camera  at (0.70, 0, 0.50), RPY (0, 0.81, π), HFOV 1.3,
                     224×224 @ 5 Hz, publishes /third_person/image_raw
```

- The models are named `red_ball`/`blue_ball` for historical reasons
  (they used to be spheres); both visual and collision are now 3 cm cubes
  — flat faces pinch reliably between the flat jaws. Demo videos show
  cubes, not balls, despite the filename.
- Success criterion (used during demo collection): cube XY within 5 cm of
  marker (0.28, 0). Z not checked.

## Gotchas / inconsistencies (will bite)

1. **Spawn `z=0.2` is still the default.** `arm_gazebo.launch.py` spawns
   the arm 20 cm off the floor. The URDF compensates with a
   `world→base_link` joint of `xyz="0 0 -0.2"`, so **base_link is
   actually at world z=0**. Ball XYZ from `gz model -p` is directly
   usable as a base-frame target — no `ARM_BASE_Z` offset needed.

2. **Two controllers, not one.** The arm is split across two
   `joint_trajectory_controller` instances: `arm_controller` (5 arm
   joints) and `gripper_controller` (just `gripper`). The SmolVLA
   inference node will need to fan its 6-D output across both topics.

3. **`tcp_jaw_link` is the planning frame, not `gripper_frame_link`.**
   `tcp_jaw_link` was added as a fixed child of `gripper_link` at
   `(0.012, 0.010, -0.035)` — the midpoint between the static finger and
   the moving jaw. The current keyframe expert doesn't use IK, but if
   you regenerate any MoveIt config in the future, re-add this frame as
   the SRDF tip_link.

4. **DetachableJoint plugins are idle.** Two
   `gz-sim-detachable-joint-system` plugins live in `so101.urdf.xacro`
   with `/attach_*`/`/detach_*` bridged into ROS, but the current
   keyframe expert does not publish to them — friction grasp alone holds
   the 3 cm cube. The bridge and plugins remain wired up but unused.
   If you re-enable them, two historical constraints:
   - `<attach_topic>` is the correct tag in gz-sim 8 (`<topic>` silently
     does nothing).
   - Reset balls via `gz service .../set_pose`, **not** remove + create.
     The plugin binds to the model entity ID at init; a respawn
     invalidates that pointer.

5. **Keyframes are tuned for canonical ball pose (0.22, ±0.08).** The
   expert reads actual ball XY just before grasp and applies a linear
   `shoulder_pan` correction (scale ≈ `canonical_pan / canonical_y`,
   clamped to ±0.12 rad) plus a `shoulder_lift` correction
   (scale ≈ 6.0 rad/m, full strength on `grasp`, half on `above`).
   Accurate within ~±2 cm of canonical; `--randomize` is capped at ±1 cm
   for that reason. Outside that envelope the linear scaling overshoots
   and you'll close jaws on empty space.

6. **Red-ball placement skew.** The *old* IK-based expert showed ~6–10 cm
   systematic drift past the marker on red, producing a 143 blue / 57 red
   split in `data/demos_lerobot_final/`. The keyframe expert mirrors
   blue keyframes for red by flipping `shoulder_pan` + `wrist_roll`, so
   color symmetry is structural now — a fresh collection is expected to
   be balanced, but hasn't been re-run.

7. **`render_engine` default is `ogre2`** (GPU). Fall back to `ogre` only
   if you hit EGL/driver issues. In the devcontainer there's no GPU
   passthrough, so Gazebo falls back to `llvmpipe` and RTF drops hard.

8. **Apt deps**: `ros-jazzy-joint-trajectory-controller`,
   `ros-jazzy-ros2controlcli`, `ros-jazzy-ros-gz`,
   `ros-jazzy-gz-ros2-control`, `ros-jazzy-cv-bridge`, `ffmpeg`. MoveIt
   is no longer required — the keyframe expert and SmolVLA inference
   node both bypass `move_group`.

## Key files to read before editing

### Robot + sim

- `src/arm_description/description/so101.urdf.xacro` — links/joints, the
  `<ros2_control>` block, the `gz_ros2_control` plugin, the two
  (currently idle) DetachableJoint plugins, and `tcp_jaw_link`.
- `src/arm_description/worlds/ball_world.sdf` — physics
  `<max_step_size>` (4 ms), the two cubes, the green marker, and the
  third-person camera.
- `src/arm_description/config/controllers.yaml` — `joint_state_broadcaster`,
  `arm_controller` (5 joints), `gripper_controller` (1 joint).
- `src/arm_description/launch/arm_gazebo.launch.py` — xacro → urdf → rsp
  → gazebo → bridges (`/clock`, `/third_person/image_raw`,
  `/attach_*`/`/detach_*`) → spawn → controller spawners (with a 6 s
  timer before controllers).

### VLA / demo pipeline

- `src/vla/vla/pick_and_place_moveit.py` — the scripted keyframe expert.
  `BLUE_KEYFRAMES` / `RED_KEYFRAMES` dicts, per-episode XY correction
  before grasp, 11-step sequence. Publishes `/episode/record` (Bool) +
  `/episode/target` (String) so the recorder brackets episodes. Despite
  the filename, this node **does not talk to MoveIt** and does not
  touch `/attach_*`/`/detach_*`. `JOINT_NAMES_ARM` is defined inline.
- `src/vla/vla/demo_recorder.py` — synchronized 10 Hz recorder, writes
  per-episode `frames.npz` + `meta.json` + sampled jpegs.
- `src/vla/vla/smolvla_inference_node.py` — closed-loop policy node.
  Loads `SmolVLAPolicy.from_pretrained` from `checkpoints/smolvla_so101/`,
  subscribes to `/third_person/image_raw` + `/joint_states`, runs
  `policy.select_action` on a 10 Hz timer, fans the 6-D output across
  `/arm_controller/joint_trajectory` (5 joints) +
  `/gripper_controller/joint_trajectory` (1 joint). Must be invoked
  with `.venv/bin/python` (the script's `#!/usr/bin/env python3`
  shebang otherwise picks `/usr/bin/python3` which has no torch/lerobot).
- `src/vla/CMakeLists.txt` — lists which scripts get installed; editing
  any `vla/vla/*.py` only survives `colcon build` if it's in
  `install(PROGRAMS ...)`.

### Demo collection scripts

- `scripts/collect_200_demos.sh` — batch wrapper; restarts the sim every
  N trials, runs the expert, parses successes from
  `/tmp/collect_progress.log`. Uses `set -eo` (not `-u`) for
  ROS-source tolerance. Launches `arm_gazebo.launch.py headless:=true`.
- `scripts/filter_successful_demos.py` — parses progress log,
  hard-links successful episodes into a flat tree.
- `scripts/convert_to_lerobot.py` — raw episode dirs → LeRobot v2
  parquet + h264 video + meta. Known off-by-one between parquet rows
  and mp4 frames in some episodes; `train_relaxed.py` clamps around it
  (see `NEXT_STEPS.md`).
- `scripts/nuke_sim.sh` — TERM 0 reset (`pkill` + `kill -9` survivors +
  DDS daemon stop/start).
- `scripts/joint_sliders.py` — Tk GUI for tuning keyframes. How the
  current `BLUE_KEYFRAMES` / `RED_KEYFRAMES` tables were captured.

### Handoffs + status

- `NEXT_STEPS.md` — the active handoff. Covers the failed 2026-04-25
  closed-loop eval, the time-horizon-mismatch hypothesis, and a four-
  step recovery plan (bump `command_horizon_sec`, lower
  `n_action_steps`, characterize across trials, escalate to data fixes
  if needed). Also captures the .venv/numpy/shebang invocation gotchas.
- `CLAUDE_CHANGES.md` — dated log of every file change Claude has made,
  newest at top.

## Current status (April 2026)

- ✅ Gazebo + URDF + ros2_control end-to-end on bare-metal Linux with
  RTX 2060 (RTF ≈ 1.0).
- ✅ Keyframe expert + recorder + LeRobot converter implemented.
  Keyframe expert is ~100% on canonical cubes, still reliable with
  ±1 cm randomization via the linear XY correction.
- ✅ 200 successful demos in `data/demos_lerobot_final/` (collected
  with the *old* IK-based expert; 143 blue / 57 red split, 49% overall
  success rate at collection time). A re-collection with the keyframe
  expert would likely be balanced and higher-yield, but hasn't been run.
- ✅ SmolVLA 30k-step fine-tune complete on UCF nobel A100 via
  `train_relaxed.py`. Checkpoint pulled to
  `checkpoints/smolvla_so101/`.
- ✅ SmolVLA inference node written (`src/vla/vla/smolvla_inference_node.py`)
  and registered in `src/vla/CMakeLists.txt`.
- ❌ First closed-loop eval (2026-04-25, blue task) failed: arm moved
  to a half-pose, gripper never closed, cube never moved. Leading
  hypothesis is a training/inference time-horizon mismatch — see
  `NEXT_STEPS.md` for the four-step recovery plan.

## Claude-specific conventions in this repo

- `CLAUDE_CHANGES.md` — human-readable log of changes Claude has applied,
  newest entries at the top. Append a dated entry whenever you finish a
  task that modifies files.
- `.claude/hooks/remind_changelog.sh` is registered as a `Stop` hook in
  `.claude/settings.json`. If `git status` is dirty and
  `CLAUDE_CHANGES.md` isn't among the dirty files, the hook blocks the
  stop and feeds a reminder back. Keep the hook silent by editing
  `CLAUDE_CHANGES.md` as part of any change-making session.
