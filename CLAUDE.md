# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is right now

A ROS 2 Jazzy workspace for driving a simulated **SO-101** 6-DOF arm with a
vision-language-action policy. The project has **two pipelines** living side
by side:

1. **SmolVLA pipeline (primary, current focus).** Finetune
   `lerobot/smolvla_base` on Gazebo-collected pick-and-place demos. The
   expert is a MoveIt2-driven scripted policy; the recorder captures
   image + joint-state + action tuples into a LeRobot v2 dataset; the
   finetuned policy is meant to run locally and emit absolute joint
   positions directly.
2. **OpenVLA pipeline (legacy, still wired but not the focus).** Streams a
   wrist (or third-person) camera frame to a remote OpenVLA-7B
   `/act` HTTP server, integrates the returned 7D EE-frame deltas onto
   the current EE pose, and analytically solves IK to joint commands.
   Useful as a comparison baseline.

Treat the SmolVLA path as the active direction. Touch the OpenVLA nodes
only when explicitly asked — they predate the MoveIt2 work and use a
different action representation.

## Workspace layout

ROS 2 **Jazzy** colcon workspace with three packages under `src/`:

- `arm_description` — SO-101 6-DOF arm URDF (xacro), Gazebo Sim (gz) world
  with two graspable balls + a green place marker + a third-person camera,
  `ros2_control` config, and the base launch files (Gazebo + RViz).
- `arm_moveit_config` — MoveIt2 configuration for SO-101: SRDF, KDL/TRAC-IK,
  OMPL + Pilz planning configs, controller bindings, and a one-shot launcher
  that brings up Gazebo + arm + controllers + `move_group` together.
- `vla` — Python ROS 2 nodes. Contains the legacy OpenVLA client trio
  (`vla_action_client.py` → `action_to_ee.py` → `vla_ik.py`), the SmolVLA-era
  scripted expert (`pick_and_place_moveit.py`), the demo recorder
  (`demo_recorder.py`), kinematic helpers (`kinematics_so101.py`), and
  small spike / test utilities (`grasp_spike_local.py`,
  `fake_vla_action_client.py`).

All three use `ament_cmake` (not `ament_python`); `vla` installs its Python
modules via `ament_python_install_package` and individual scripts via
`install(PROGRAMS ...)` — editing `vla/vla/*.py` only survives `colcon build`
if it's listed there.

## Data flow (SmolVLA pipeline — primary)

Demo collection (already complete: 200 successful episodes →
`data/demos_lerobot_final/`):

```
arm_moveit_config (move_group + Gazebo + arm + controllers, one launch)
        │
        ▼
pick_and_place_moveit.py   reads ground-truth ball poses; plans home →
        │                  above → cartesian descend → close → /attach_<color>
        │                  → lift → above marker → descend → /detach → home
        │                  via /compute_cartesian_path + /compute_ik
        │                  (falls back to local IK when fraction < 1.0).
        ▼
joint_trajectory commands → /arm_controller (5 joints) +
                            /gripper_controller (1 joint)
                          → DetachableJoint plugin handles "magic grasp"
                            via /attach_{blue,red} bridged into gz.
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

Training + inference (NOT yet executed; design only):

```
data/demos_lerobot_final/  ──►  python -m lerobot.scripts.train
                                --policy.type=smolvla
                                --policy.pretrained_path=lerobot/smolvla_base
                                (~4 h on a single A100, batch 64, 30k steps)
                                ──►  outputs/.../checkpoints/030000/
                                         pretrained_model
                                                │
                                                ▼
                                  smolvla_inference_node (TODO)
                                  loads SmolVLAPolicy.from_pretrained(...);
                                  subscribes to /third_person/image_raw +
                                  /joint_states; calls policy.select_action;
                                  publishes 6-D joints split across
                                  /arm_controller/joint_trajectory (5) +
                                  /gripper_controller/joint_trajectory (1).
```

There is no IK or EE-pose math in the SmolVLA inference path — the policy
directly emits joint positions. That's why this pipeline is shorter than
the OpenVLA one.

## Data flow (OpenVLA pipeline — legacy)

```
Gazebo  ── /third_person/image_raw (or /camera/image_raw if wrist cam re-added)
    │
    ▼
vla_action_client.py ── HTTP /act ─► OpenVLA server (remote, SSH-tunneled)
        │                            returns [dx,dy,dz, drx,dry,drz, gripper]
        │ /vla_action (Float32MultiArray, len 7)
        ▼
action_to_ee.py     looks up TF base_link ← gripper_frame_link;
        │           scales+clamps deltas; integrates onto current pose
        │ /vla_target_pose (PoseStamped) + /vla_gripper (Float32)
        ▼
vla_ik.py           analytic 3-DOF IK (pan, lift, elbow); wrist + gripper
        │           held at parameter values; deadband suppresses no-op
        │           publishes
        ▼
/arm_controller/joint_trajectory  → gz_ros2_control → joints in Gazebo
```

`vla_action_client.py` originally read a wrist camera (`/camera/image_raw`)
that was removed when `camera.xacro` was deleted. To re-run the OpenVLA
loop you'll either need to re-add a wrist camera or repoint the client at
`/third_person/image_raw`.

## Common commands

All commands assume `cd /home/peyton-peyton/vla_arm` and that ROS 2 Jazzy
is installed at `/opt/ros/jazzy`. Build before everything:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit launch / config / urdf / world files
without rebuilding. Re-build after editing Python nodes or
`CMakeLists.txt`. To build a single package:

```bash
colcon build --packages-select arm_moveit_config --symlink-install
```

### Sim launchers

```bash
# Gazebo + arm + controllers + move_group, one-shot (use this for demo collection)
ros2 launch arm_moveit_config arm_gazebo_moveit.launch.py

# Gazebo + arm + controllers only (no MoveIt — faster spin-up, used by the
# legacy OpenVLA pipeline and by grasp_spike_local.py)
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
# Terminal 1 — launch the integrated stack:
ros2 launch arm_moveit_config arm_gazebo_moveit.launch.py

# Terminal 2 — recorder (one episode dir per episode):
ros2 run vla demo_recorder.py --ros-args \
    -p output_dir:=/workspace/data/demos_raw -p record_rate_hz:=10.0

# Terminal 3 — expert policy (writes to /tmp/collect_progress.log via tee):
ros2 run vla pick_and_place_moveit.py --trials 25 --randomize --seed 0

# Or batch wrapper that restarts the sim every 25 trials and stops at
# --target successes:
bash scripts/collect_200_demos.sh --target 200 --batch-size 25 \
    --output /workspace/data/demos_raw_final
```

### Filter + convert demos to LeRobot v2

```bash
python3 scripts/filter_successful_demos.py \
    --progress-log /tmp/collect_progress.log \
    --batches-root /workspace/data/demos_raw_final \
    --output /workspace/data/demos_successful

python3 scripts/convert_to_lerobot.py \
    --input-dir /workspace/data/demos_successful \
    --output-dir /workspace/data/demos_lerobot_final \
    --fps 10
```

The canonical artifact is **`data/demos_lerobot_final/`** — 200 episodes,
44,202 frames, ~15 MB, LeRobot v2 schema.

### SmolVLA finetune (NOT yet executed)

See `SMOLVLA_INSTALL.md` for the install + training recipe end to end. The
short version:

```bash
python3 -m venv ~/smolvla_env && source ~/smolvla_env/bin/activate
pip install "lerobot[smolvla]"
python -m lerobot.scripts.train \
    --policy.type=smolvla \
    --policy.pretrained_path=lerobot/smolvla_base \
    --dataset.root=/path/to/demos_lerobot_final \
    --dataset.repo_id=local/so101_pickplace_sim \
    --batch_size=64 --steps=30000 --eval_freq=-1 --save_freq=5000 \
    --output_dir=outputs/smolvla_so101_full --policy.push_to_hub=false
```

Compute is unconfirmed — the natural target is the UCF box that already
hosts the OpenVLA inference server, but check with the user before
scheduling 4 h of A100 time.

### Legacy OpenVLA pipeline

```bash
ros2 launch arm_description arm_gazebo.launch.py    # no MoveIt needed
ros2 launch vla vla.launch.py                        # client → action_to_ee → vla_ik
```

Requires the OpenVLA server reachable via the SSH tunnel in `cmds.txt`.

### Diagnostics

```bash
xacro src/arm_description/description/so101.urdf.xacro -o /tmp/so101.urdf
gz topic -e -t /stats -n 5            # real-time factor (sim must be up)
ros2 control list_controllers          # arm_controller / gripper_controller / jsb
gz model -m blue_ball -p              # ball pose; sanity-check between resets
ros2 run tf2_ros tf2_echo base_link tcp_jaw_link
```

There are no tests, linter configs, or CI in this repo.

## Mandatory rule — clean slate before every sim test

Before running anything against Gazebo (the expert policy, the recorder,
grasp spikes, IK checks, an inference node when one exists), verify the
sim is a **clean slate** — not just "fresh launch", a full tear-down +
state check.

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
   else, especially after several DetachableJoint attach/detach cycles.

How to apply, every time:

1. `bash scripts/nuke_sim.sh` (TERM 0 reset + explicit `kill -9` + DDS
   daemon stop/start).
2. Relaunch the sim.
3. **Verify** after startup: home joints ≈ `[0,0,0,0,0,0]` (tol ~0.05
   rad); target object pose from `gz model -m <ball> -p` matches SDF
   default. If either is off, reset again — don't run the test on stale
   state.

Memory entry `feedback_sim_clean_slate.md` mirrors this rule.

## World state (already set up — change with care)

Defaults from `src/arm_description/worlds/ball_world.sdf`:

```
red_ball       sphere visual r=3 cm, BOX collision 5 cm cube, mass 50 g
                 at (0.16, -0.08, 0.11)
blue_ball      same shape/mass at (0.16, +0.08, 0.11)
place_target   green disc r=5 cm, visual-only (no collision), static
                 at (0.22, 0, 0.085)
table          40×55×8 cm at (0.25, 0, 0.04)
third_person_camera  at (0.70, 0, 0.50), RPY (0, 0.81, π), HFOV 1.3,
                     224×224 @ 5 Hz, publishes /third_person/image_raw
```

- Balls were repositioned from earlier (0.22, ±0.10, 0.11) → (0.16, ±0.08,
  0.11) to drop arm-reach usage from 82 % → 60 %.
- Sphere VISUAL + cube COLLISION is intentional: spheres squirt out of the
  flat jaws; cube faces pinch reliably. Visual unchanged so demo videos
  still show a sphere.
- Success criterion (used during demo collection): ball XY within 5 cm of
  marker (0.22, 0). Z not checked.

## Gotchas / inconsistencies (will bite)

1. **Spawn `z=0.2` is still the default.** `arm_gazebo.launch.py` spawns
   the arm 20 cm off the floor. Combined with the new ball positions
   (0.16, ±0.08, 0.11) the arm reach is comfortable, but if you change
   the spawn or revert ball positions you may re-introduce the
   "vla_ik.py reach clamp kicks in and the arm just fully extends"
   failure mode the old gotcha called out.

2. **Two controllers, not one.** The arm is split across two
   `joint_trajectory_controller` instances: `arm_controller` (5 arm
   joints) and `gripper_controller` (just `gripper`). The SmolVLA
   inference node will need to fan its 6-D output across both topics. The
   legacy `vla_ik.py` only publishes to `arm_controller`.

3. **`tcp_jaw_link` is the planning frame, not `gripper_frame_link`.**
   `tcp_jaw_link` was added as a fixed child of `gripper_link` at
   `(0.012, 0.010, -0.035)` — the midpoint between the static finger and
   the moving jaw. SRDF tip_link is `tcp_jaw_link`; `kinematics_so101.py`
   CHAIN ends at `tcp_jaw_link`; `pick_and_place_moveit.py` calls
   `compute_ik`/`compute_cartesian_path` with `link_name=tcp_jaw_link`.
   If you regenerate the MoveIt config from scratch via Setup Assistant,
   re-add this frame.

4. **DetachableJoint = sim-cheat grasp.** Two
   `gz-sim-detachable-joint-system` plugins live in `so101.urdf.xacro`,
   one per ball, with `<attach_topic>/<detach_topic>` bridged to ROS as
   `std_msgs/Empty`. The expert publishes `/attach_<color>` after
   gripper-close to fix the ball to the gripper. This is a sim trick —
   real-hardware transfer needs a real friction grasp. Two follow-on
   constraints:
   - `<attach_topic>` is the correct tag in gz-sim 8 (the older `<topic>`
     tag silently does nothing).
   - Resetting balls between trials must use `gz service .../set_pose`,
     **not** remove + create. The plugin binds to the model entity ID at
     plugin init; a re-spawn invalidates that pointer and attaches stop
     working.

5. **MoveIt cartesian paths often complete only partially.**
   `compute_cartesian_path` returns `fraction < 1.0` on ~90 % of descent
   requests with KDL/TRAC-IK on this arm. `pick_and_place_moveit.cartesian_move`
   falls back to a local pos-only IK joint goal at the endpoint. The
   recorded trajectory is then joint-space interpolated, not a true
   straight line — still acceptable for behaviour cloning.

6. **Red-ball placement has ~6–10 cm systematic drift past the marker.**
   Blue side is well-behaved. The `data/demos_lerobot_final/` dataset is
   skewed 143 blue / 57 red because of this. Earlier attempts to pin
   `wrist_roll = π/2` symmetrically per ball side made things worse (arm
   knocked balls during approach). If you need balanced colors, fix the
   IK first.

7. **Wrist camera (`/camera/image_raw`) no longer exists.**
   `description/camera.xacro` was deleted; the world now publishes only
   `/third_person/image_raw` (224×224 @ 5 Hz). The legacy
   `vla_action_client.py` reads `/camera/image_raw` by default — re-point
   it or re-add the wrist camera if you want to revive the OpenVLA loop.

8. **`render_engine` default is `ogre2`** (GPU). Fall back to `ogre` only
   if you hit EGL/driver issues. In the devcontainer there's no GPU
   passthrough, so Gazebo falls back to `llvmpipe` and RTF drops hard.

9. **`ros-jazzy-joint-trajectory-controller` and the MoveIt2 stack must
   be installed** apt-side. See `run_commands_moveit.txt` for the full
   apt list (`ros-jazzy-moveit`, `ros-jazzy-moveit-py`,
   `ros-jazzy-trac-ik-kinematics-plugin`, `ros-jazzy-ros2controlcli`,
   `ffmpeg`).

## Key files to read before editing

### Robot + sim

- `src/arm_description/description/so101.urdf.xacro` — links/joints, the
  `<ros2_control>` block, the `gz_ros2_control` plugin, the two
  DetachableJoint plugins, and `tcp_jaw_link`.
- `src/arm_description/worlds/ball_world.sdf` — physics
  `<max_step_size>` (4 ms), the two balls (sphere visual / cube
  collision), the green marker, and the third-person camera.
- `src/arm_description/config/controllers.yaml` — `joint_state_broadcaster`,
  `arm_controller` (5 joints), `gripper_controller` (1 joint).
- `src/arm_description/launch/arm_gazebo.launch.py` — xacro → urdf → rsp
  → gazebo → bridges (`/clock`, `/third_person/image_raw`,
  `/attach_*`/`/detach_*`) → spawn → controller spawners (with a 6 s
  timer before controllers).

### MoveIt

- `src/arm_moveit_config/config/so101.srdf` — planning groups (`arm`,
  `gripper`), `home` named state, end-effector chain ending at
  `tcp_jaw_link`.
- `src/arm_moveit_config/config/{kinematics,ompl_planning,pilz_*,joint_limits,moveit_controllers}.yaml`.
- `src/arm_moveit_config/launch/arm_gazebo_moveit.launch.py` — combined
  one-shot launcher (Gazebo first, `move_group` after a 10 s timer).

### VLA / demo pipeline

- `src/vla/vla/pick_and_place_moveit.py` — the scripted expert. Uses
  `/compute_ik` + `/compute_cartesian_path` services + a local pos-only
  IK fallback. Publishes `/episode/record` (Bool) + `/episode/target`
  (String) so the recorder brackets episodes; toggles
  `/attach_<color>` / `/detach_<color>` around the grasp.
- `src/vla/vla/demo_recorder.py` — synchronized 10 Hz recorder, writes
  per-episode `frames.npz` + `meta.json` + sampled jpegs.
- `src/vla/vla/kinematics_so101.py` — FK/IK helpers. CHAIN terminates at
  `tcp_jaw_link`.
- `src/vla/vla/grasp_spike_local.py` — standalone local-IK grasp test (no
  MoveIt). Useful for quickly debugging grip geometry.
- `src/vla/vla/{vla_action_client,action_to_ee,vla_ik}.py` — the legacy
  OpenVLA trio.
- `src/vla/launch/vla.launch.py` — launches the OpenVLA trio with inline
  parameter dicts.
- `src/vla/CMakeLists.txt` — lists which scripts get installed; editing
  any `vla/vla/*.py` only survives `colcon build` if it's in
  `install(PROGRAMS ...)`.

### Demo collection scripts

- `scripts/collect_200_demos.sh` — batch wrapper; restarts the sim every
  N trials, runs the expert, parses successes from
  `/tmp/collect_progress.log`. Tolerates ROS's unbound-variable
  sourcing (uses `set -eo`, not `-u`).
- `scripts/filter_successful_demos.py` — parses progress log,
  hard-links successful episodes into a flat tree.
- `scripts/convert_to_lerobot.py` — raw episode dirs → LeRobot v2
  parquet + h264 video + meta.
- `scripts/nuke_sim.sh` — TERM 0 reset (`pkill` + `kill -9` survivors +
  DDS daemon stop/start).

### Handoffs + status

- `HANDOFF_moveit_smolvla.md` — the demo-collection-phase handoff
  (assumes the devcontainer; explains the MoveIt2 + scripted-expert
  scope).
- `HANDOFF_smolvla_finetune.md` — the finetune-phase handoff (what's
  done, what the dataset looks like, the training command, open
  questions).
- `SMOLVLA_INSTALL.md` — install + training recipe end to end, plus an
  air-gapped variant.
- `QA_LOG.md` — open questions from earlier handoffs and the answers.
- `CLAUDE_CHANGES.md` — dated log of every file change Claude has made,
  newest at top.

## Current status (April 2026)

- ✅ Gazebo + URDF + ros2_control + MoveIt2 + DetachableJoint grasp work
  end to end.
- ✅ Scripted expert + recorder + LeRobot converter implemented.
- ✅ 200 successful demos collected → `data/demos_lerobot_final/`
  (49 % success rate over 408 attempts, 143 blue / 57 red split).
- ⏳ SmolVLA training — recipe documented, not executed. Compute target
  unconfirmed.
- ⏳ SmolVLA inference node — designed in `SMOLVLA_INSTALL.md` and
  `HANDOFF_smolvla_finetune.md`, not yet written. Will subscribe to
  `/third_person/image_raw` + `/joint_states`, call
  `policy.select_action`, and fan the 6-D output across `arm_controller`
  + `gripper_controller`.
- 🟡 OpenVLA pipeline still present but stale: it reads a
  `/camera/image_raw` topic that no longer exists since `camera.xacro`
  was removed.

## Claude-specific conventions in this repo

- `CLAUDE_CHANGES.md` — human-readable log of changes Claude has applied,
  newest entries at the top. Append a dated entry whenever you finish a
  task that modifies files.
- `.claude/hooks/remind_changelog.sh` is registered as a `Stop` hook in
  `.claude/settings.json`. If `git status` is dirty and
  `CLAUDE_CHANGES.md` isn't among the dirty files, the hook blocks the
  stop and feeds a reminder back. Keep the hook silent by editing
  `CLAUDE_CHANGES.md` as part of any change-making session.
