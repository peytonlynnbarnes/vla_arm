# Handoff: SO-ARM101 + SmolVLA — Demo Collection Phase

You're picking up this project in a **dev container** (see `.devcontainer/`) with the repo bind-mounted at `/workspace`. A previous Claude session on the host did the scoping research and a partial gripper-physics spike; this document is the self-contained briefing so you can resume without that conversation.

## Goal

Fine-tune **SmolVLA** (`lerobot/smolvla_base`) on **200 Gazebo-generated demos** of a **multi-color pick-and-place task** on **SO-ARM101**, for eventual real-hardware transfer.

Task: "pick the {red,blue} ball and place it on the green marker."

## Prime directive

**Do NOT fine-tune OpenVLA-7B.** Earlier research confirmed SmolVLA is pretrained on 487 SO-100 community datasets, consumes LeRobot datasets natively (no RLDS conversion), and fine-tunes in ~4 h on a single A100. It's the community-default for this embodiment. OpenVLA-7B has no SO-101 prior. An OpenVLA `/act` server may be reachable at `localhost:8000` via the host's SSH tunnel to UCF (`ssh -L 8000:localhost:8000 pe606840@nobel.ece.ucf.edu`); `--network=host` makes it visible from inside the container. Leave it alone unless you want comparison.

## Dev-container specifics (READ FIRST — several project assumptions change)

The Dockerfile is at `.devcontainer/Dockerfile`. Key facts:

- **Working dir** is `/workspace` (bind-mount from host repo). All file paths in this doc are container-relative.
- **ROS 2 Jazzy auto-sourced** on every `bash -c` via `BASH_ENV=/etc/profile.d/ros.sh`, which also sources `/workspace/install/setup.bash` if it exists. You do NOT need to `source /opt/ros/jazzy/setup.bash` manually.
- **`json-numpy` is pip-installed system-wide** (`--break-system-packages`). **Don't use the host's `.venv/`** — it's there in the bind mount but its shebangs point to the host's Python. Just use `python3`.
- **`python3-opencv` is apt-installed**. `cv_bridge` works.
- **MoveIt2 is NOT in the base image.** You need `sudo apt update && sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-trac-ik-kinematics-plugin`. This only persists until the container exits — add to `.devcontainer/Dockerfile` and rebuild (`./.devcontainer/run.sh` auto-rebuilds) to make it permanent.
- **No GPU passthrough.** Container has no `--gpus all` and no `/dev/dri` / X11 mounts. Gazebo will fall back to **CPU rendering (llvmpipe)**, so RTF will drop from ~1.0 on the host to maybe 0.1–0.3 in the container. The sim still works; demo collection just takes longer. If that becomes unworkable, either (a) add `--gpus all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` to `run.sh` and install nvidia-container-toolkit, or (b) run Gazebo on the host and talk to it from the container (but `--network=host` + `ROS_DOMAIN_ID` alignment is needed).
- **Network**: `--network=host` means localhost services on the host (the UCF tunnel) ARE reachable.
- **Sudo NOPASSWD** for user `dev` — `sudo apt install ...` works without prompts.
- **Branch protection**: `.devcontainer/run.sh` refuses to start on `main`/`master`/`jazzy` branches without `CLAUDE_ALLOW_PROTECTED=1`. The host session was on `jazzy`. Either start from a worktree (`git worktree add ../vla_arm-claude -b claude/playground`) or set the env var.
- **Claude home** in container: `~/.claude-vla-arm` on host bind-mounted to `/home/dev/.claude`. This is **separate from the host's Claude home**, so the memory file the previous session saved (`~/.claude/projects/-home-peyton-peyton-vla-arm/memory/`) is NOT visible here. Its critical content is reproduced below under "Mandatory rule — clean slate before every sim test."

## Must-read files before touching anything

- `CLAUDE.md` — project conventions, repo layout, build commands
- `run_commands.txt` — bring-up runbook; contains the mandatory TERM 0 nuke-and-reset
- `CLAUDE_CHANGES.md` — chronological log, newest at top. **Append a dated entry for every file change you make** or the stop-hook blocks you from ending the session
- `src/vla/vla/vla_ik.py` — working FK chain (`CHAIN`, `fk_pose`) and analytic 5-DOF IK. Useful kinematic reference even once MoveIt2 is in place
- `src/arm_description/description/so101.urdf.xacro` — URDF + `<ros2_control>` block
- `src/arm_description/worlds/ball_world.sdf` — world + objects + camera
- `src/arm_description/config/controllers.yaml` — `arm_controller` (joint_trajectory_controller, position interface) + `joint_state_broadcaster`

## Mandatory rule — clean slate before every sim test

**Reproducing the memory file from the host session verbatim** (won't be visible in this container's `~/.claude`):

> Before running any test against Gazebo (gripper spikes, IK checks, recorder dry-runs, etc.), make sure the sim is a **verified clean slate**. Not just "a fresh launch command" — a full tear-down and state verification.
>
> Why: sim state leaks across restarts in two specific ways we've hit:
> 1. **Ghost controller_manager on DDS** — `ros2 launch` can die but its `controller_manager` and friends get reparented to PID 1 and keep holding joint positions. A new Gazebo starts *on top* of that state, and `/joint_states` reports whatever the old controllers were commanding, not the URDF zero pose. `pkill -f 'ros2 launch'` alone doesn't catch these orphans — you have to `kill -9` them by explicit PID after `ps -ef`.
> 2. **Gazebo world state mismatches visual state** — `gz model -p` may return the SDF default pose when the live runtime state is something else. You may *think* the ball is at its spawn point because the query agrees with the SDF, while visually it's on the floor.
>
> How to apply, every time before a test:
> 1. Run the full `run_commands.txt` TERM 0 pkill sequence, then `kill -9 <explicit PIDs>` for any survivors reparented to PID 1 (check with `ps -ef | grep -E "gz sim|parameter_bridge|robot_state_publisher|ruby.*gz|controller_manager"`).
> 2. `ros2 daemon stop && sleep 2 && ros2 daemon start` to flush DDS.
> 3. Relaunch Gazebo fresh.
> 4. **Verify** after startup: home joints ≈ `[0,0,0,0,0,0]` (tol ~0.05 rad); target object's `gz model -p` pose matches the SDF default. If either is off, reset again — don't run the test on stale state.

Consider re-saving this to `~/.claude/projects/-workspace/memory/feedback_sim_clean_slate.md` (with a matching `MEMORY.md` index entry) so future container sessions inherit it.

## Current world state (already set up, no changes needed)

```
red_ball      sphere r=3cm mass=50g  at (0.16, -0.08, 0.11)
blue_ball     sphere r=3cm mass=50g  at (0.16, +0.08, 0.11)
place_target  green disc r=5cm visual-only static  at (0.22, 0, 0.085)
table         40x55x8 cm at (0.25, 0, 0.04)
third_person_camera at (0.70, 0, 0.50), RPY (0, 0.81, π), HFOV 1.3, 224×224 @ 5 Hz
  publishes /third_person/image_raw
```

Shoulder is at world (0.04, 0, 0.06), reach ~0.25 m. Balls at 60% of max reach — comfortable kinematics. Success criterion is **2-D**: ball XY within marker radius (5 cm), regardless of Z.

SDF is symlinked source→install; editing `src/arm_description/worlds/ball_world.sdf` takes effect on next Gazebo launch with no rebuild. URDF and `controllers.yaml` are NOT reloadable — Gazebo restart required.

## Task list (set these up with TaskCreate at the start of the session)

1. **[IN_PROGRESS] Gripper physics spike via MoveIt2** — reliably lift blue_ball ≥10 cm using `compute_cartesian_path` or Pilz LIN. Success: ≥80% lifts across 10 trials from default position.
2. **[PENDING] Scripted multi-color pick-and-place expert policy** — reads ground-truth ball poses from `gz model -p`, picks target color per episode prompt, uses MoveIt2 to do approach → cartesian descend → grasp → transport → cartesian descend-to-place → release → home. Outputs success bool + recorded trajectory.
3. **[PENDING] Demo recorder + episode reset + randomization** — ROS node subscribing to `/third_person/image_raw`, `/joint_states`, `/arm_controller/joint_trajectory`; writes synchronized tuples per episode. Between episodes: teleport balls via `gz service`, reset arm to home. **Include domain-randomization hooks** (ball XY jitter, lighting intensity, camera pose jitter, table texture) — real SO-101 hardware is the eventual target and sim-only policies don't transfer without randomization.
4. **[PENDING] Collect 200 successful demos** — overnight job. Expect 40–70% success rate, so 300–500 attempts. Success filter = ball XY within marker radius AND gripper released.
5. **[PENDING] Convert raw demos → LeRobot dataset format** — LeRobot v2 parquet/video schema: `observation.images.third_person`, `observation.state` (6-D joints), `action` (6-D commanded joints), `timestamp`, `episode_index`, `frame_index`. Verify `LeRobotDataset.from_parquet(...)` loads cleanly. **No RLDS** — SmolVLA consumes LeRobot format directly.

## Why MoveIt2 (what killed the analytic-IK spike)

- The SO-101 has 5 arm DOF. Analytic per-waypoint IK is accurate (`pos_err < 5 mm`) but the `joint_trajectory_controller` splines BETWEEN waypoints. Between "approach above ball" and "descend to ball," shoulder_lift + elbow_flex + wrist_flex all change simultaneously. The gripper tip traces a **curve**, not a vertical line, and brushes the ball sideways on the way down (ball moves ~2 cm) before the jaws can close around it.
- Orientation-constrained IK saturated joint limits during descent. Joint-space regularization produced smooth motion but didn't fix the Cartesian-path issue.
- **Fix:** MoveIt2's `compute_cartesian_path()` interpolates in Cartesian space and solves IK per micro-step, producing a true straight-line descent. Pilz's LIN planner is an even cleaner alternative for deterministic straight-line motions.

## MoveIt2 setup plan

1. Install (one-time, ephemeral until Dockerfile update):
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-trac-ik-kinematics-plugin
   ```
   Then add the same lines to `.devcontainer/Dockerfile` and rebuild to make permanent.

2. Create `src/arm_moveit_config/` package. Either use MoveIt Setup Assistant (`ros2 launch moveit_setup_assistant setup_assistant.launch.py`) or hand-write:
   - **SRDF** with 2 planning groups: `arm` (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll) and `gripper` (gripper). Define a `home` state at all zeros.
   - **kinematics.yaml** — KDL is fine to start; switch to TRAC-IK (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`) if KDL struggles near reach limits.
   - **ompl_planning.yaml** — RRTConnect default for transport.
   - **pilz_industrial_motion_planner_config.yaml** — enable LIN/PTP for deterministic descent/ascent (cleaner than `compute_cartesian_path` for pure straight-line moves).
   - **joint_limits.yaml** — import URDF velocity/accel limits.

3. Launch: a new `arm_moveit.launch.py` that brings up `move_group` alongside the existing `arm_gazebo.launch.py`. Ensure `use_sim_time:=true` everywhere.

4. Python interface: `moveit_py`. Pattern:
   ```python
   from moveit.planning import MoveItPy, PlanRequestParameters
   moveit = MoveItPy(node_name="grasp_spike_moveit")
   arm = moveit.get_planning_component("arm")
   arm.set_start_state_to_current_state()
   # for Cartesian descent:
   # arm.plan_cartesian_path(waypoints, max_step=0.01, jump_threshold=0.0)
   # or set goal + plan/execute with Pilz LIN pipeline
   ```

## Spike-level smoke test (first milestone — resume here)

Target: `src/vla/vla/grasp_spike_moveit.py` (or similar). Steps:

1. `ros2 launch arm_description arm_gazebo.launch.py` + MoveIt2 launch
2. Verify clean slate (home joints ≈ 0, `gz model -m blue_ball -p` matches SDF)
3. Home arm via `arm.plan_to_named("home")` (or joint values `[0]*5`)
4. Plan + execute to "above blue_ball": xyz=(0.16, 0.08, 0.26), orientation = "gripper pointing down"
   - Derive the exact target quaternion by running `fk_pose` at a sensible test-config first; the "pointing down" convention in `gripper_frame_link` is not obvious from the URDF alone (previous session confirmed this — the naive `[[1,0,0],[0,-1,0],[0,0,-1]]` saturated joint limits)
5. Cartesian or Pilz LIN descent from above-ball → at-ball (xyz=(0.16, 0.08, 0.11)), orientation locked, max_step=0.01 m, jump_threshold=0.0
6. Close gripper to ~-0.15 rad (range is [-0.174, 1.745]; grip joint named `gripper`)
7. Cartesian/LIN ascent to z=0.31
8. Sample `gz model -m blue_ball -p`. Success if Z ≥ 0.20 (rose ≥ 10 cm).
9. Repeat 10× for success rate.

## Earlier iteration notes (saves you re-discovering)

- `open=1.2, close=-0.15` values for the gripper joint worked geometrically (jaws spanned the 6cm-diameter ball) — what failed was trajectory, not grip position.
- Balls were moved from their earlier (0.22, ±0.10, 0.11) positions to **(0.16, ±0.08, 0.11)** specifically to drop arm-reach usage from 82% → 60% and give IK kinematic slack. Don't move them back unless you're intentionally testing reach limits.
- The camera was retargeted: pitch `0.73 → 0.81` and HFOV `1.0 → 1.3` so (a) balls + marker + arm are all in frame with margin, (b) optical axis hits the ball row centre. Don't revert unless you need the earlier framing.
- `controllers.yaml` declares `update_rate: 100` (Hz), but the live physics step ran at 1 ms for a while (stale) and should now be 4 ms from `ball_world.sdf`'s `<max_step_size>0.004</max_step_size>`. If you see controllers complaining `Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.004 s)` — that's informational, not an error.

## Project-specific pitfalls (will bite)

1. **Zombie controllers after a crashed sim.** `pkill -f 'ros2 launch'` misses orphans reparented to PID 1: `gz sim server`, `gz sim gui`, `ruby.*gz`, `parameter_bridge`, `robot_state_publisher`, `controller_manager`. Always grep `ps -ef | grep -E ...` and `kill -9 <explicit PIDs>` after pkill. Then `ros2 daemon stop && sleep 2 && ros2 daemon start`. See run_commands.txt TERM 0 for the canonical sequence.

2. **`arm_controller` "FATAL" on spawn is cosmetic.** `controllers.yaml` auto-loads both controllers from `gz_ros2_control` plugin init; the spawner's redundant `load_controller` call hits "already loaded" and logs FATAL, but the controller IS active. Verify with `ros2 control list_controllers` — both should show `active`.

3. **Gazebo ogre2 EGL warnings (`libEGL warning: egl: failed to create dri2 screen`)** print ~10× during startup. On the host this was non-fatal — Ogre2 fell back to a working surface. In this GPU-less container, it'll print MORE and rendering drops to llvmpipe. Still works, just slow.

4. **CLAUDE_CHANGES.md stop-hook.** If git status is dirty and CLAUDE_CHANGES.md isn't among the dirty files, your session will be blocked from stopping. Append a dated entry at the TOP (newest first) describing what files changed and why. There's a `.claude/hooks/remind_changelog.sh` that enforces this.

5. **SmolVLA fine-tune compute.** The RTX 2060 on the host has only 6 GB — insufficient for SmolVLA training. The UCF GPU that hosts OpenVLA inference is the natural target (see `cmds.txt`), but confirm with the user before scheduling any training jobs.

## Recommended first 30 minutes

1. `cat CLAUDE.md run_commands.txt HANDOFF_moveit_smolvla.md` (top sections), skim `CLAUDE_CHANGES.md`.
2. `apt list --installed 2>/dev/null | grep -E "ros-jazzy-moveit|trac-ik"` — install missing pieces via `sudo apt install`.
3. Create the 5 tasks in TaskCreate (per list above), mark #1 `in_progress`.
4. TERM 0 full reset per memory rule, `ros2 launch arm_description arm_gazebo.launch.py`, verify clean slate (home joints ≈ 0; balls at SDF positions; controllers active).
5. Scaffold `src/arm_moveit_config/` (SRDF, kinematics, OMPL, Pilz, launch). Test `move_group` starts cleanly.
6. Write `grasp_spike_moveit.py`, iterate until ≥80% lift success across 10 trials.
7. Update CLAUDE_CHANGES.md with a dated entry. Mark Task 1 complete. Proceed to Task 2.

## Open questions to confirm with the user

1. Add MoveIt2 packages to `.devcontainer/Dockerfile` permanently, or install ephemerally per session?
2. GPU passthrough: worth adding `--gpus all` + X11 mounts to `.devcontainer/run.sh` now, or live with CPU rendering during demo collection?
3. OMPL/RRTConnect for transport + Pilz LIN for descent/ascent — acceptable plan?
4. Episode ball-position distribution for Task #3: uniform over table half? ±3 cm jitter around default? Colors and marker position — fixed or randomized?
5. Dataset publishing: local-only, or push to HuggingFace hub?
6. SmolVLA fine-tune compute target: the UCF A100 that hosts OpenVLA, or a different box?
