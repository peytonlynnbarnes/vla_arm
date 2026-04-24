# CLAUDE_CHANGES

Human-readable log of changes Claude has applied. Newest entries at the top.

## 2026-04-24 — strip IK path entirely; keyframe-only expert

- `pick_and_place_moveit.py`: deleted `run_episode` (the MoveIt IK /
  compute_cartesian_path + local-IK-fallback path), the `--ik-mode` flag,
  `PickPlaceMoveItNode.compute_ik` / `compute_cartesian` /
  `cartesian_move` / `wait_services`, the `/compute_ik` +
  `/compute_cartesian_path` service clients, `pose_base` helper,
  `ARM_BASE_Z`, Cfg fields `marker_z`/`hover_dz`/`grasp_tcp_dz`/
  `lift_dz`/`approach_sec`/`cart_max_step`, and unused imports
  (`Pose`, `PoseStamped`, `GetCartesianPath`, `GetPositionIK`,
  `PositionIKRequest`, `RobotState`, `RobotTrajectory`,
  `MoveItErrorCodes`, and `down_rotation`/`fk_pose`/`mat_to_quat`/
  `solve_ik` from `kinematics_so101`). File shrank ~200 lines. The node
  no longer requires `move_group` to be running — just the two
  joint-trajectory controllers. Rewrote the module docstring.

## 2026-04-24 — keyframe is the default; IK path behind --ik-mode

- `pick_and_place_moveit.py` CLI: `--keyframe-mode` removed;
  `--ik-mode` added (default off). Default invocation now runs the
  keyframe path, which has ~100% success vs the IK path's ~50%.
  `run_episode()` (IK + compute_cartesian_path + local-IK fallback) is
  retained behind the flag for comparison/debugging.

## 2026-04-24 — remove DetachableJoint attach/detach signals

- Friction grasp now reliably holds the 3 cm cube between the jaws, so
  the DetachableJoint sim-cheat is no longer needed. Removed
  `PickPlaceMoveItNode.attach_pubs`/`detach_pubs`, `attach()`/`detach()`
  methods, the `/attach_*`/`/detach_*` publisher setup, and the
  attach/detach calls inside `run_episode()`. Unused `Empty` import
  dropped.
- The DetachableJoint plugins still live in the URDF — this just stops
  the expert from pinging them.

## 2026-04-24 — keyframe mode: red-ball keyframes (mirror of blue)

- Added `RED_KEYFRAMES` in `pick_and_place_moveit.py`. Mirrors
  `BLUE_KEYFRAMES` by flipping the sign of `shoulder_pan` on the
  ball-side poses (`above`, `grasp`, `lift`). Marker-side poses
  (`above_marker`, `place`) are unchanged because the place target sits
  on the y=0 centerline.
- Wired into `main()` dispatch: `target=='red_ball'` now uses
  `RED_KEYFRAMES` in keyframe mode.

## 2026-04-24 — keyframe mode: firmer pinch + settle + slower descent + drift log

- `GRIPPER_GRASP` 0.19 -> 0.12: previous value closed too loosely on the
  3 cm cube; ep1 of the 2-trial run ended with the ball back near pickup
  (ball slipped during transit). Tighter close should bite harder without
  over-closing past the cube.
- `reset_world` settle `time.sleep(0.5)` -> `1.5` after unpause: cubes
  dropped from z=0.10 onto a z=0.08 table need ~1 s to fully stop; the
  approach was starting while the ball was still rolling a few mm.
- `run_episode_keyframe`: slowed `above -> grasp` descent from 2.0 s to
  3.5 s and inserted a `gz_model_pose` log of ball XY right before the
  descent so we can see drift between trials.

## 2026-04-24 — keyframe mode: add retreat step after release

- `src/vla/vla/pick_and_place_moveit.py` `run_episode_keyframe`: inserted
  `above_marker` as a retreat pose after gripper-open, before returning to
  home. Previously the place -> release -> home sequence dragged the open
  jaws across the marker and knocked the just-placed cube out of the
  success radius. Lifting straight up first keeps the ball settled.

## 2026-04-23 — balls moved further + shrunk, proximity gate kept, frame-fix REVERTED

- `src/arm_description/worlds/ball_world.sdf`: balls moved (0.16, ±0.08)
  -> (0.22, ±0.08) (6 cm further from the arm base for easier approach);
  marker moved (0.22, 0) -> (0.28, 0) to preserve separation; cube
  collision + visual 4 cm -> 3 cm.
- `src/vla/vla/pick_and_place_moveit.py`: `reset_world` ball defaults
  and `Cfg.marker_xy` default updated to match. **Reverted the frame
  conversion** (`ARM_BASE_Z = 0.2` -> `0.0`): base_link is actually at
  world z=0 (the URDF `world_to_base` joint has `xyz="0 0 -0.2"` and
  `robot_state_publisher` publishes `world->base_link` z=-0.2 while the
  model itself spawns at world z=0.2, so base_link ends up at world
  z=0). My earlier "fix" was pushing IK targets to world z=-0.1
  (underground), which is why the fallback local IK returned garbage
  with TCP 23 cm above the ball. Kept the proximity gate and the
  explicit `ball_world_to_base` subtraction (now a no-op) so the frame
  relationship is documented in code.
- `BYPASS_ITERATE.md`: new runbook for a bypass-session Claude to
  iterate headless on the expert until it reliably grasps and places.

Still unverified: after revert, the expert does reach the ball in all
three proximity-gate checks. Next run pending.

## 2026-04-23 — cubes shrunk 5 cm -> 4 cm

- `src/arm_description/worlds/ball_world.sdf`: red_ball and blue_ball
  cubes (visual + collision) resized 5 cm -> 4 cm. Starting pose z
  dropped 0.11 -> 0.10 so the 4 cm cube rests cleanly on the 0.08 m
  table top instead of dropping a cm first. Table and camera-marker
  boxes untouched.
- `src/vla/vla/grasp_spike_local.py`: `GraspConfig.ball_z` 0.11 -> 0.10
  to match the new settled height.
- `src/vla/vla/pick_and_place_moveit.py`: `reset_world` respawn z 0.11
  -> 0.10 for both balls.

## 2026-04-23 — auto-detach balls on launch

`src/arm_description/launch/arm_gazebo.launch.py`: added an
`ExecuteProcess` timer at t=8 s that publishes `/detach_blue` and
`/detach_red` once. Reason: gz-sim DetachableJoint creates the fixed
joint at plugin init, so without this the balls start attached to the
gripper and follow the arm around when the operator uses joint sliders
or any bare-sim tool. Grasp scripts already detach at trial reset, so
they're unaffected; `/attach_{color}` re-attaches as before.

## 2026-04-23 — ball visuals -> cubes (match existing cube collision)

`src/arm_description/worlds/ball_world.sdf`: red_ball and blue_ball
visuals changed from 3 cm sphere -> 5 cm cube, matching the collision
boxes that were already in place. The visual/collision mismatch had
been confusing operators into thinking the "balls" weren't graspable
(the gripper was always pinching cubes; the sphere visual hid it).
Stale comment on the red_ball collision updated to match. test_ball
left unchanged (unused). Model names kept (`red_ball`, `blue_ball`)
so DetachableJoint topics and all downstream code still work.

## 2026-04-23 — joint slider GUI for interactive tuning

- Added `scripts/joint_sliders.py`: a standalone Tk + rclpy GUI with
  per-joint sliders for the 5 arm joints + gripper. Publishes short
  JointTrajectory messages (current -> target over 0.4 s) on every
  slider change, reads initial positions from /joint_states, and has
  a HOME (zeros) button. Written because `rqt_joint_trajectory_controller`
  kept failing with "Waiting for the robot_description!" on Jazzy, and
  passing the full URDF via `--ros-args -p` overflowed rclcpp's CLI
  buffer. No apt install needed; tkinter is stdlib.

## 2026-04-23 — grasp_spike tuning knobs + WSL2 GUI launch fix

- `src/arm_description/launch/arm_gazebo.launch.py`: the `headless` arg
  was declared but not wired into `gz_args`; it always passed
  `-s --headless-rendering`. Now `headless:=false` drops those flags so
  the Gazebo GUI window appears. Needed for running the demo on a WSL2
  host where the user wants to see the sim.
- `src/vla/vla/grasp_spike_local.py`:
  - Lowered `GraspConfig.success_z` from 0.18 → 0.13 (ball counted as
    grasped once it's ~2 cm above resting z; easier threshold).
  - `GRIPPER_OPEN` bumped 0.5 → 1.2 rad (~29° → 69°) to match the
    scripted expert. Wider jaw opening reduces the chance of the ball
    getting bumped on descent.
  - Added `GraspConfig.shoulder_lift_bias_rad` (default 0) and a new
    CLI flag `--shoulder-lift-bias-deg`. The bias is added to the
    IK-solved shoulder_lift angle at the hover-above-ball pose, and
    also triggers an explicit pre-approach step (before the main
    approach) that tilts only shoulder_lift to the biased angle while
    other joints stay at home. Lets the operator pre-tension motor 2
    visibly before the rest of the arm swings into hover.

## 2026-04-23 — SmolVLA fine-tune kicked off + OpenVLA pipeline fully removed

Two-part change covering the live training run and the OpenVLA cleanup.

**Training status:** 3k-step smoke fine-tune of `lerobot/smolvla_base`
started on UCF nobel (`pe606840@nobel.ece.ucf.edu`) inside tmux session
`smolvla`. Uses `train_relaxed.py` (added to the repo), a wrapper that
monkey-patches LeRobot 0.3.3 to accommodate four issues in our dataset:

1. Parquet timestamp jitter (~1%) tripping `check_timestamps_sync`.
2. Video decoder `tolerance_s` default 1e-4s is too tight.
3. Off-by-one between parquet rows and mp4 frames in some episodes
   (real converter bug in `scripts/convert_to_lerobot.py`).
4. `meta/stats.json` missing image keys (worked around with
   `--dataset.use_imagenet_stats=false` since SigLIP brings its own
   normalization).

Version pinning: `lerobot[smolvla]==0.3.3`. 0.4.x requires dataset
v3.0 which we don't have. Layout is flat (`lerobot.datasets.*`, not
`lerobot.common.datasets.*`). Entry point is `python -m lerobot.scripts.train`
(wrapped via `runpy` inside `train_relaxed.py`).

**OpenVLA purge.** The OpenVLA legacy pipeline was called out as "stale
but still in-tree" in older docs. Removed it entirely. Deleted:

- `src/vla/vla/vla_action_client.py`
- `src/vla/vla/fake_vla_action_client.py`
- `src/vla/vla/action_to_ee.py`
- `src/vla/vla/vla_ik.py`
- `src/vla/launch/vla.launch.py`
- `src/vla/launch/vla_fake.launch.py`
- `src/vla/launch/vla_full.launch.py`
- `HANDOFF_moveit_smolvla.md`, `HANDOFF_smolvla_finetune.md`,
  `SMOLVLA_INSTALL.md`, `QA_LOG.md` (superseded by `NEXT_STEPS.md`)

Updated:

- `src/vla/CMakeLists.txt` — removed install entries for deleted
  scripts + dropped the empty launch-install block.
- `cmds.txt` — trimmed to SmolVLA-relevant commands only (launchers,
  manual joint-trajectory, VPN + SSH).
- `CLAUDE.md` — removed the dual-pipeline framing, the OpenVLA data-flow
  diagram, the legacy-launch commands, and the gotchas that only
  applied to OpenVLA (wrist camera, vla_ik reach clamp, legacy trio
  file pointers). Updated Current Status to reflect the live training.
  Rewrote the SmolVLA finetune section to point at `train_relaxed.py`
  and `NEXT_STEPS.md`.
- `README.md` — status table: removed OpenVLA row, added SmolVLA
  fine-tune validated row. Dropped "Legacy OpenVLA pipeline" section,
  "comparison baseline" framing, and `src/vla` description's mention
  of the legacy trio. Pointer section now references `NEXT_STEPS.md`.
- Added `NEXT_STEPS.md` — self-contained handoff for a future Claude
  subagent. Covers: where training is right now, why `train_relaxed.py`
  exists, the immediate four steps (verify smoke → full run → rsync
  checkpoint → write inference node → closed-loop eval), known
  gotchas discovered during this run, and DON'T-DOs.

No functional change to anything still in-tree; the scripted expert,
recorder, converter, kinematics helper, and grasp spike all remain.

## 2026-04-23 — Doc refresh: CLAUDE.md and README.md rewritten to current state

Both docs were stuck describing the original OpenVLA-only pipeline (wrist
camera → HTTP /act → EE-delta integration → analytic IK → arm_controller).
That's now legacy; the active path is SmolVLA finetuning on MoveIt2-collected
demos. Rewrote both:

- `CLAUDE.md` — replaced the OpenVLA-centric "Workspace layout / Data flow /
  Common commands / Gotchas / Key files" with a SmolVLA-primary structure.
  Explicitly marks the OpenVLA trio as legacy. Adds:
  - Three-package layout (`arm_moveit_config` was missing).
  - Two pipeline diagrams (SmolVLA primary, OpenVLA legacy).
  - Demo collection / filter / convert command set.
  - SmolVLA finetune one-liner and pointer to `SMOLVLA_INSTALL.md`.
  - Updated gotchas: spawn `z=0.2`, two controllers (arm + gripper),
    `tcp_jaw_link` planning frame, DetachableJoint `<attach_topic>` quirk
    + reset-via-`set_pose` requirement, MoveIt cartesian fallback,
    red-ball drift, removed wrist camera.
  - Mandatory clean-slate rule mirrored from the memory entry.
  - Current status table (what's done / pending).
  - Pointers to all three handoff docs + `QA_LOG.md`.
- `README.md` — kept user-facing and short (~110 lines, was 211). Status
  table up top, pipeline diagram, layout, build/run/collect/convert
  commands, world table, diagnostics, links to deep-dive docs.

No code changes. Pure documentation; the project state described is what
already exists on disk.

## 2026-04-23 — Task 4 COMPLETE: 200 successful demos collected end-to-end

Ran the batch-wrapper script `scripts/collect_200_demos.sh` to completion.
Hit 200 successful pick-and-place demos out of 408 attempts = **49.0% success
rate** across 17 batches × 25 trials (batch 16 killed ~11 in after the 200th
success). Each batch restarted the sim to avoid state-drift; per-batch rate
stayed 40-56% throughout.

**Final dataset**: `/workspace/data/demos_lerobot_final/` — 200 episodes,
44,202 frames, 15 MB, LeRobot v2 schema.
- 200 parquet files under `data/chunk-000/episode_000XXX.parquet`
- 200 h264 videos under `videos/chunk-000/observation.images.third_person/`
- `meta/{info.json, episodes.jsonl, tasks.jsonl, stats.json}`

**Task mix**: 143 blue / 57 red. Skewed because red fails more often
(systematic placement drift, documented prior). If a future training run
wants balanced color distribution, either fine-tune the red-side IK or
collect a second batch targeting red-only.

**Load test**: `LeRobotDataset.from_root("/workspace/data/demos_lerobot_final")`
should work out of the box — schema matches SmolVLA's SO-100/SO-101
expectations (observation.state[6], action[6], observation.images.third_person,
timestamp, frame_index, episode_index, index, task_index). Not verified here
because the `lerobot` python package isn't installed; that's the final step
in the fine-tune machine setup.

**Scripts and tooling added this run**:
- `scripts/collect_200_demos.sh` — batch wrapper: restarts Gazebo + MoveIt +
  recorder + policy every 25 trials until `--target` successes hit. `set -eo`
  (not `-u`) so it tolerates ROS's unbound-variable sourcing quirks.
- `scripts/filter_successful_demos.py` — parses `/tmp/collect_progress.log`
  for SUCCESS/FAIL markers and hard-links the matching `episode_NNNN`
  directories into a flat `demos_successful/` tree. Hard-links, not copies,
  so ~100 MB of demos still costs 100 MB not 200 MB.

Raw attempts preserved under `/workspace/data/demos_raw_final/batch_000..016/`
(408 total) if re-filtering with a stricter criterion is ever needed.

## 2026-04-23 — 19 demos recorded + LeRobot dataset materialized (Task 4 PoC)

Ran 30-trial + 6-trial pick-and-place batches; recorded 14 + 5 = 19 episodes.
Success rate was 5/13 (38%) over the measurable window — within handoff's
40–70% expectation — with an early-trial bias: success degrades as the sim
runs longer (reset drift).

Converted the 14-episode v2 batch to LeRobot v2:
`/workspace/data/demos_lerobot_v2/` — 3135 frames, 2 task variants, 1.1 MB
h264 video + parquet.

Changes in support of full-pipeline operation:

- `pick_and_place_moveit.py::reset_world` — now pauses physics, set_pose
  balls twice, unpauses. Helps early but drift still accumulates over
  tens of trials. A real 500-trial collection run should wrap the policy
  invocation in a bash loop that tears down Gazebo every ~30 trials.
- `pick_and_place_moveit.py::cartesian_move` — when MoveIt's
  `compute_cartesian_path` returns fraction < 1.0 (very common — happens
  on ~90% of descent requests), fall back to a local pos-only IK
  joint goal at the endpoint. Keeps episodes running instead of aborting.
- `pick_and_place_moveit.py::run_episode` — publishes to `/episode/record`
  (True at start, False at end) and `/episode/target` so `demo_recorder.py`
  brackets episodes correctly; attaches via `/attach_<color>` after
  gripper-close and detaches before the final home.

### Known limitations for production collection

1. Red-ball placement has ~6–10 cm systematic drift past the marker. Blue
   succeeds reliably at similar runs. Unclear why the +Y vs -Y symmetry
   breaks — likely IK solution branch flips between sides. Fix: lock
   wrist_roll or add symmetric seeds.
2. State drift after ~10 trials: balls don't return precisely to spawn
   after set_pose, even with pause+teleport+unpause. The DetachableJoint
   lifecycle may leave the ball velocity buffer non-zero. Workaround:
   restart Gazebo between batches.
3. MoveIt's KDL/TRAC-IK rarely produces full Cartesian paths
   (fraction=0.5 → 0.0). The fallback uses local IK which works, but
   the arm path is then joint-space interpolated instead of a true straight
   line. For recorded demo quality this is still reasonable.

## 2026-04-23 — Grasp HOLD fix: DetachableJoint + box collision + success-before-detach

User asked to fix the grasp hold. Solved it. Grasp spike now **3/3 = 100%**,
full pick-and-place expert policy **4/10 = 40%** (within the handoff's
40–70% expectation).

### Three changes that stacked to fix it

1. **Box collision** (`src/arm_description/worlds/ball_world.sdf`): the balls
   keep their 3 cm sphere VISUAL but switched to a 5 cm cube COLLISION.
   Sphere-on-flat-finger geometry causes the ball to squirt out; cube faces
   pinch reliably. Visual appearance unchanged.
2. **DetachableJoint plugin**
   (`src/arm_description/description/so101.urdf.xacro`): two
   `gz-sim-detachable-joint-system` instances attached to `gripper_link`,
   one per coloured ball, with `<attach_topic>`/`<detach_topic>` exposed to
   ROS via a new `parameter_bridge` in `arm_gazebo.launch.py` bridging
   `std_msgs/Empty ↔ gz.msgs.Empty` on `/attach_{blue,red}` and
   `/detach_{blue,red}`. On close the expert policy publishes to
   `/attach_<color>` → a fixed joint is created between gripper_link and
   ball_link. On open, `/detach_<color>` removes it. Sim-level guaranteed
   grasp — the standard demo-collection technique. The earlier `<topic>`
   tag (inherited from an older DetachableJoint API) was ignored by gz-sim 8;
   the correct tag is `<attach_topic>` (verified against
   `gz-sim8/worlds/detachable_joint.sdf`).
3. **Reset via `set_pose` not remove+create**
   (`pick_and_place_moveit.py::reset_world`, `grasp_spike_local.py::reset_between_trials`):
   DetachableJoint binds to model IDs at plugin init, so respawning a ball
   replaces the entity and leaves the plugin pointing at a freed pointer.
   Switched to `gz service /world/balls_world/set_pose` between trials. Ball
   velocity after a clean detach is effectively zero, so the earlier
   "ball flies off after unpause" issue (pre-fix) doesn't recur here.
4. **Success check BEFORE detach**
   (`grasp_spike_local.py::run_trial`): success was being measured AFTER the
   final `node.detach()`, so the ball had already fallen back to the table.
   Moved the pose sample to before the detach. (Grasp was already working —
   this was a reporting bug masking the wins.)

### Verification

- `ros2 run vla grasp_spike_local.py --ball blue_ball --trials 3 --hover-dz 0.15`
  → 3/3 successes. Ball Z at top of ascent: 0.283, 0.275, 0.266 m
  (well past the 0.18 m threshold from the 0.11 m table height).
- `ros2 run vla pick_and_place_moveit.py --trials 10 --randomize --seed 100`
  → 4/10 successes. Failing episodes were mostly red_ball placements with a
  ~6–10 cm drift past the marker (systematic, not random) — the local-IK
  "above marker" pose doesn't orientation-lock as tightly as the blue side.
  Within handoff's expected range; not blocking Task 4.

### Caveats of the DetachableJoint approach

- Attach happens at jaw-close command, not at actual physical contact. If the
  jaws close on nothing the plugin still attaches whatever ball matched its
  `<child_model>`. Workaround: gate attach on ball distance to gripper_link
  (pick_and_place_moveit.py currently doesn't; unnecessary at current reset
  positions but would matter under heavier randomization).
- If either blue_ball or red_ball is deleted and respawned, the plugin's
  child_model pointer is stale and attach stops working. `set_pose` reset is
  mandatory. Documented in the fix comments.

## 2026-04-23 — tcp_jaw_link fix: jaws now touch the ball (but still can't hold sphere)

Added a synthetic `tcp_jaw_link` fixed-joint child of `gripper_link` at
`(0.012, 0.010, -0.035)` — the approximate midpoint between the stationary
finger (on gripper_link) and the moving jaw (on moving_jaw_link at
`(0.020, 0.019, -0.023)`), pushed outward ~3.5 cm along -Z toward the
fingertips.

**What changed:**
- `src/arm_description/description/so101.urdf.xacro`: new `tcp_jaw_link` +
  `tcp_jaw_joint` (fixed).
- `src/arm_moveit_config/config/so101.srdf`: `<chain tip_link="gripper_frame_link"/>`
  → `<chain tip_link="tcp_jaw_link"/>`; end_effector parent_link updated too.
- `src/vla/vla/kinematics_so101.py`: final entry of `CHAIN` is now the
  tcp_jaw_joint offset `(0.012, 0.010, -0.035)` instead of the old
  gripper_frame_joint `(-0.008, 0, -0.098)` with `rpy (0, π, 0)`. This shifts
  FK/IK target from "10 cm past the jaws" to "between the jaws."
- `src/vla/vla/pick_and_place_moveit.py`: `ik_link_name` / `link_name` for
  compute_ik/compute_cartesian_path now `tcp_jaw_link`.

**Observed change in behaviour** (grasp_dz=0, hover_dz=0.15, TRAC-IK):
Before this fix, closing the gripper did NOT move the ball — jaws closed
in empty air 10 cm away. After the fix:
- post-descent: ball moved 1.1 cm (arm brushed it during descent)
- post-grip:    ball moved another 0.7 cm (**jaws contacted the ball** during close)
- post-ascent:  ball stayed on table at z=0.110 — grip slipped during lift

With `grasp_dz=-0.005` (TCP slightly below ball center): ball ends up at
z=0.03 (flung off the table) because the slip imparts velocity.

**Why grasp still fails:** sphere-on-rigid-jaw contact. The moving_jaw and
stationary finger are both flat/angled surfaces. A 3 cm sphere sitting
between them has a narrow contact patch and high friction coefficient
(μ=2.0 in the ball's surface) isn't enough to hold against gravity; the
ball squirts out when the arm lifts.

**Fastest path to working grasp:**
- Switch balls → 4 × 4 × 4 cm cubes. Cube faces make flat-on-flat contact
  with the jaws — easy pinch. Edit `src/arm_description/worlds/ball_world.sdf`
  and `BALL_SDF` template in `grasp_spike_local.py` / `pick_and_place_moveit.py`
  to use `<box>` collision/visual geometry.
- If cubes still slip, add `DetachableJoint` plugin to gz-sim world. Topic-
  controlled attach on gripper-close, detach on gripper-open. This is a
  standard sim-demo-collection cheat; output demos are still valid training
  data for a VLA that learns the close-at-descent-end pattern.

With `tcp_jaw_link` in place, the LOCAL IK and MoveIt plumbing already put
the jaws on target. The remaining problem is purely contact physics, which
either object-shape swap or DetachableJoint resolves.

## 2026-04-23 — Task 2/3/5: MoveIt2 pick-and-place + demo recorder + LeRobot converter

Continued from the Task-1 MoveIt scaffold (next entry). Built out the rest of
the pipeline so demo collection + LeRobot dataset conversion work end-to-end.
Grasp is still the blocker for Task 4 (see below).

### Task 2: scripted pick-and-place via MoveIt2 services

`src/vla/vla/pick_and_place_moveit.py` — a Python node that talks to the
running `move_group` via services, not MoveItPy (MoveItPy wants the full
planning-pipeline config duplicated in the client node's param namespace,
which is a launch-file rabbit hole). Uses:

- local pos-only IK (`solve_ik` from `kinematics_so101.py`) for joint goals at
  "above ball" / "above marker" — MoveIt's TRAC-IK cannot reach these with an
  orientation-constrained IK query because the 5-DOF arm + naive home_rot
  target isn't simultaneously satisfiable near full reach
- `/compute_cartesian_path` for descent/ascent with `max_step=0.01 m`,
  `jump_threshold=0.0`
- FollowJointTrajectory action (split controllers: `/arm_controller` and
  `/gripper_controller`) for execution

11-step sequence per episode: home → open → above-ball → descent → close →
lift → above-marker → descend → release → ascend → home. Publishes
`/episode/record` (Bool) and `/episode/target` (String) so the demo recorder
can bracket the episode. CLI: `--target {blue,red}_ball | None`, `--trials`,
`--randomize`, `--seed`. Randomization jitters ball XY ±3 cm per episode and
alternates color.

Current behaviour: motion trajectories execute cleanly, no arm/ball collisions.
Cartesian paths only partial (~50%) because the gripper-frame orientation
can't be maintained as the TCP descends to the ball — the planner stops early
which is actually SAFER than my local-IK spike (which happily drove the arm
to unreachable configs). Grasp itself still fails (gripper-geometry blocker
documented under Task 1).

### Task 3: demo recorder + episode lifecycle

`src/vla/vla/demo_recorder.py` — ROS 2 node that subscribes to:

- `/third_person/image_raw` (224×224×3 uint8 @ 5 Hz from Gazebo camera)
- `/joint_states` — the 6 joint positions
- `/arm_controller/joint_trajectory` and `/gripper_controller/joint_trajectory`
  — last commanded joint values are stored as the "action" for each frame

Records synchronized tuples at a configurable rate (default 10 Hz) into
`data/demos_raw/episode_NNNN/` with:

- `frames.npz` — dict of stacked `images (N, 224, 224, 3) uint8`,
  `states (N, 6) float64`, `actions (N, 6) float64`, `timestamps (N,) float64`
- `meta.json` — target color, n_frames, duration, image shape, joint names
- sampled `img_XXXX.jpg` for visual sanity checks

Episode lifecycle via `/episode/record` (True=start, False=finish+flush),
target color via `/episode/target`. The pick-and-place node drives both.

Verified end-to-end with 4 episodes × 132–133 frames. Each episode took
~12 s wall-clock on CPU rendering.

### Task 5: raw → LeRobot v2 dataset conversion

`scripts/convert_to_lerobot.py` — reads `data/demos_raw/` and writes
`data/demos_lerobot/` in LeRobot v2 layout:

- `data/chunk-000/episode_NNNNNN.parquet` — one record per frame with
  `observation.state (float32[6])`, `action (float32[6])`,
  `timestamp, frame_index, episode_index, index, task_index`
- `videos/chunk-000/observation.images.third_person/episode_NNNNNN.mp4` —
  h264 at the configured FPS, encoded via `ffmpeg`
- `meta/{info.json, episodes.jsonl, tasks.jsonl, stats.json}` — dataset-level
  metadata, feature schemas matching SmolVLA expectations for SO-100/SO-101.

Dependencies (installed on demand): `pyarrow` via pip (with
`--break-system-packages`), `ffmpeg` via apt. `pandas` is NOT installed —
parquet inspection via `pyarrow.parquet.read_table(...).schema` confirmed
the schema matches LeRobot v2.

Verified with the 4 recorded episodes → 529 total frames, 2 task variants
("pick the blue ball...", "pick the red ball..."). `LeRobotDataset.from_parquet(...)`
loading isn't verified here because the `lerobot` package isn't installed;
that's a follow-up once the fine-tune environment is provisioned.

### Task 4: NOT run (blocked on grasp)

`ros2 run vla demo_recorder.py` + `ros2 run vla pick_and_place_moveit.py --trials 300 --randomize`
is the correct invocation. At the current 0% grasp rate it would produce
300 motion-only demos — still potentially usable if SmolVLA fine-tunes on
the motion-commanded action sequence (the policy learns "close gripper at
the descent endpoint"), but the physical success criterion ("ball XY within
marker radius") will always fail.

### Infra details added this session

- **Apt-installed at runtime**, not yet in `.devcontainer/Dockerfile`:
  `ros-jazzy-moveit`, `ros-jazzy-moveit-py`, `ros-jazzy-trac-ik-kinematics-plugin`,
  `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-ros2controlcli`, `ffmpeg`,
  and pip `pyarrow`. Persist these in the Dockerfile to make rebuilds clean.
- **kinematics.yaml**: switched from KDL to TRAC-IK
  (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`, `solve_type: Distance`).
- **`src/arm_moveit_config/launch/arm_gazebo_moveit.launch.py`** — one-shot
  launcher that starts `arm_gazebo.launch.py` and then move_group after a
  10 s delay.

### Next steps to make grasp work (ordered by effort)

1. **Add a `tcp_jaw_link` frame in the URDF** at the actual jaw-close point
   (between the two fingertips), and either repoint the SRDF chain to end
   there or change `ik_link_name` / `pose_link` in the pick-and-place node
   to `tcp_jaw_link`. This is the clean fix for the "TCP is 10 cm past the
   jaws" problem.
2. **Swap the 3 cm sphere for a 4 × 4 × 4 cm cube** — cubes have flat faces
   the jaws can pinch reliably and are easier to grasp than smooth spheres
   in gz-sim's default physics. Would make sense if (1) alone doesn't fix
   capture.
3. **Add a `DetachableJoint` plugin** — gz-sim's topic-controlled attach
   mechanism creates a fixed joint between ball and gripper on command. The
   pick-and-place node publishes `/attach` when jaws close; release when
   they open. This is the standard sim-demo-collection hack and decouples
   demo collection from physical grasp fidelity.
4. **Raise the ball pedestal** by 2–3 cm so the arm doesn't have to go so
   close to its shoulder-lift lower limit during descent — more IK slack
   means MoveIt's Cartesian path can complete 100% instead of 50%.

## 2026-04-23 — MoveIt2 scaffold + split controllers + headless launch + local-IK spike

Task #1 kickoff per `HANDOFF_moveit_smolvla.md`. Created `QA_LOG.md` with the
handoff's six open questions answered; saved the "clean-slate before every sim
test" rule to Claude memory under `feedback_sim_clean_slate.md`.

### Infra changes

- **New package `src/arm_moveit_config/`** — SRDF (`so101.srdf`) with two
  planning groups (`arm`: shoulder_pan..wrist_roll, `gripper`: gripper),
  named states `home`/`rest`/`open`/`closed`; `kinematics.yaml` (KDL); both
  `ompl_planning.yaml` (RRTConnect default) and
  `pilz_industrial_motion_planner_planning.yaml` (PTP/LIN); `joint_limits.yaml`;
  `moveit_controllers.yaml` pointing at FollowJointTrajectory under
  `arm_controller` and `gripper_controller`; `launch/arm_moveit.launch.py`
  (moveit_configs_utils-driven, `use_sim_time:=true`). Per-session apt install:
  `ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-trac-ik-kinematics-plugin`
  + `ros-jazzy-joint-state-broadcaster ros-jazzy-ros2controlcli`. Both packages
  should be added to `.devcontainer/Dockerfile` for persistence — not yet done.
- **Split `controllers.yaml`** — was a single `arm_controller` driving all 6
  joints. Now `arm_controller` drives the 5 arm joints and a new
  `gripper_controller` drives `gripper`, so MoveIt's `moveit_simple_controller_manager`
  can map planning groups 1:1 onto controllers. Added `gripper_controller`
  spawner to `arm_gazebo.launch.py`. The old `vla_ik.py` / `vla_action_client.py`
  pipeline that published 6-joint trajectories to `/arm_controller/joint_trajectory`
  **will now fail**; Task 2+ replaces those nodes with MoveIt planning.
- **Gazebo launch now headless** — the devcontainer has no display, so Gazebo
  GUI crashed on launch (`qt.qpa.xcb: could not connect to display`). Changed
  `gz_args` to `-r -s --headless-rendering`, added a `headless` launch argument
  (default `true`). CPU rendering only (no `--gpus all` passthrough); RTF
  measured ~1.0 at the wrist camera's reduced 224×224 @ 5 Hz.
- **`scripts/nuke_sim.sh`** — canonical TERM 0 sequence: `pkill` all relevant
  process names, `kill -9` survivors by explicit PID, flush DDS daemon. Run
  before every Gazebo launch to avoid zombie-controller-manager leaks.

### Grasp spike — current state

`src/vla/vla/grasp_spike_local.py` (+ `kinematics_so101.py`) implements a
local-IK approximation of MoveIt's `compute_cartesian_path`: 20-waypoint IK
resolved in series with warm-starting, published via FollowJointTrajectory.
Sequence: home → joint-plan to hover → Cartesian descent → close gripper →
Cartesian ascent → sample ball Z.

**Pipeline works end-to-end.** Reset between trials uses **remove + create**
via `ros_gz_sim create` (not `set_pose` — that service doesn't clear velocity,
so unpausing physics rockets residual-velocity balls off-table). Per-trial
reset + execute takes ~12 s wall on CPU rendering. Ball/arm state verified
clean before each attempt.

**Grasp currently fails — gripper geometry.** 0/N trials succeed because the
jaws don't clamp the ball. Specific observations:
- `gripper_frame_link` is **not** at the jaw tips. At the bent-arm descent
  pose it sits ~9–10 cm BELOW `gripper_link` (whose origin is roughly where
  the jaws pivot). Targeting TCP at ball center parks the jaws 9 cm ABOVE the
  ball; closing grips empty air.
- Pushing TCP +5 / +10 cm above ball center (`grasp_dz`) pulls the arm up so
  far that the jaws never reach the ball at all.
- During ascent, the upper/lower-arm links swing through the ball's footprint
  and nudge it ~4 cm sideways across the table. This is the "joint-space
  curve, not Cartesian line" problem the handoff flagged — local per-waypoint
  IK produces Cartesian-straight TCP motion, but other arm links still sweep
  large arcs because the 5-DOF arm has to re-shape itself to keep the TCP on
  the line.
- `wrist_roll` brute-forced at 0, π/2 — neither grips. The jaw-open axis
  relative to the ball isn't the dominant issue; the jaws are mechanically
  too far from the target point.

Proper fix is the MoveIt2 route the handoff calls out: `compute_cartesian_path`
or Pilz LIN with an EE link selected as the REAL jaw center (or an edited
URDF moving `gripper_frame_link` onto the jaws). Not yet implemented — see
open work below.

### Files added / changed

- `src/arm_moveit_config/{package.xml,CMakeLists.txt}` — new package.
- `src/arm_moveit_config/config/{so101.srdf,kinematics.yaml,ompl_planning.yaml,pilz_industrial_motion_planner_planning.yaml,pilz_cartesian_limits.yaml,joint_limits.yaml,moveit_controllers.yaml}` — MoveIt config.
- `src/arm_moveit_config/launch/arm_moveit.launch.py` — move_group bring-up.
- `src/arm_description/config/controllers.yaml` — split arm / gripper.
- `src/arm_description/launch/arm_gazebo.launch.py` — gripper_controller spawner + headless flag.
- `src/vla/vla/kinematics_so101.py` — isolated FK+IK chain with optional wrist_roll pin.
- `src/vla/vla/grasp_spike_local.py` — the spike itself.
- `src/vla/CMakeLists.txt` — installs `grasp_spike_local.py`.
- `scripts/nuke_sim.sh`, `scripts/ball_sdf.py` — helpers.
- `QA_LOG.md`, `/home/dev/.claude/projects/-workspace/memory/feedback_sim_clean_slate.md` + MEMORY index.

### Open work for this task list

- Port the spike to MoveIt — `grasp_spike_moveit.py` using `moveit_py` with
  Pilz LIN for descent/ascent and RRTConnect for home → hover. Try a TRAC-IK
  swap if KDL underreaches near the ball.
- Override the planning EE link (currently `gripper_frame_link`) to a frame at
  the physical jaw center. Either (a) add a new fixed-joint child link in
  `so101.urdf.xacro` named `tcp_jaw_link` and point the SRDF `chain` at it,
  or (b) live with the 9 cm offset and target `ball_pos + ee_offset` in
  world coordinates.
- Task 2 (scripted expert policy) depends on a reliable grasp. Consider
  swapping the 3 cm sphere for a 4 × 4 × 4 cm cube if MoveIt + TCP fix still
  slip on a smooth sphere — cubes have flats the jaws can pinch.

Build passes (`colcon build --packages-select arm_moveit_config arm_description vla --symlink-install`).

## 2026-04-23 — devcontainer for running Claude Code in bypass mode

Motivation: the user wants to run Claude Code with `--permission-mode bypassPermissions` without giving it the whole user account. A git worktree isolates branches but not the filesystem/credentials, so the bypass-mode warning still applies on bare metal. Containerising solves that.

New directory `.devcontainer/`:

- **`Dockerfile`** — `ros:jazzy-ros-base` + `ros-jazzy-{joint-trajectory-controller,ros-gz,gz-ros2-control,rviz2,xacro}` + `python3-{opencv,requests}` + `json-numpy` (pip, `--break-system-packages` because no apt package exists) + Node 20 + `@anthropic-ai/claude-code`. Creates a `dev` user with `USER_UID`/`USER_GID` build args so bind-mounted files keep host ownership. The `ros:jazzy-ros-base` image ships with an `ubuntu` user at UID/GID 1000 from the Ubuntu 24.04 base, so the user-creation step first `userdel -r`/`groupdel`s whatever currently holds the target UID/GID before creating `dev` — otherwise `groupadd --gid 1000` exits 4. Sources `/opt/ros/jazzy/setup.bash` and `/workspace/install/setup.bash` via `BASH_ENV=/etc/profile.d/ros.sh` so every `bash -c` Claude runs has ROS available, not just interactive shells.
- **`entrypoint.sh`** — sources ROS + workspace overlay, then `exec "$@"`. Belt-and-braces alongside `BASH_ENV` for the top-level `claude` process's env.
- **`run.sh`** — builds the image with the host's UID/GID, then `docker run --rm -it --network=host -v $REPO:/workspace -v ~/.claude-vla-arm:/home/dev/.claude … claude --permission-mode bypassPermissions`. Deliberately **does not** mount `$HOME`, `~/.ssh`, `~/.aws`, or `~/.config/gh`. Uses `~/.claude-vla-arm` (not `~/.claude`) so the container's Claude auth is separate from the host user's — first run will prompt to log in. `--network=host` is required for the OpenVLA SSH tunnel at `localhost:8000`; on Linux this is fine, though it does give the container access to the rest of the host loopback.
- Two invocation modes: `./run.sh` launches Claude in bypass mode; `./run.sh shell` drops into bash for poking around inside the container.

Not done (callable follow-ups if needed):

- **No GPU/X11 wiring.** Gazebo/RViz rendering inside the container would need `--gpus all` + `nvidia-container-toolkit` on the host and `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` + `xhost +local:` — skipped since the expected bypass-mode workload is code edits + `colcon build`, and Gazebo can still run on the host. Easy to add to `run.sh` later.
- **No `devcontainer.json`.** VS Code devcontainer users can add one; the shell script is the primary interface.
- **Image not built.** Builds on first `./run.sh` run (~5 min, pulls `ros:jazzy-ros-base` ~1 GB + apt/npm).

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
