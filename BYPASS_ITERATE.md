# BYPASS_ITERATE.md — Headless sim-iteration runbook

Goal: keep launching the sim and running the pick-and-place expert (headless)
until it reliably **grasps a ball between the gripper jaws** and **releases
it on top of the green disc**. Optimize the expert — do not touch the world
SDF, URDF, or controllers unless a defect there is unambiguously the blocker.

This is written for a bypass-permissions Claude session. You can run any bash,
edit any file, colcon build, pkill, etc. without approvals. Do not ask for
confirmation — just proceed.

## The primary context you MUST read first

1. `CLAUDE.md` — project overview, workspace layout, launch commands, the
   **clean-slate rule** (mandatory before every sim test), gotchas.
2. `CLAUDE_CHANGES.md` — dated log of prior edits. Append new entries as you
   make changes (`.claude/hooks/remind_changelog.sh` will block Stop if the
   log isn't touched in a dirty session).
3. `src/vla/vla/pick_and_place_moveit.py` — the expert you are iterating on.
4. `src/vla/vla/kinematics_so101.py` — local FK/IK used for fallbacks and
   the proximity gate.
5. `src/arm_description/worlds/ball_world.sdf` — ball positions, cube sizes,
   marker location. Read but do not edit unless necessary.

## Known ground truth (as of 2026-04-23, after the frame-fix reversion)

- **Arm base_link is at Gazebo world z=0** (despite the arm model being
  spawned at world z=0.2 — the URDF has `world→base_link` with
  `xyz="0 0 -0.2"`, and `robot_state_publisher` publishes a static
  `world→base_link` TF with `z=-0.2`). Verify with
  `ros2 run tf2_ros tf2_echo world base_link` or
  `ros2 topic echo --once /tf_static`. **No frame conversion is needed** for
  ball / marker XYZ from `gz model -p` when used with `solve_ik`,
  `fk_pose`, or `pose_base` — they are already in base_link frame because
  base_link ≡ world (translationally) at z=0.
- **Current ball positions**: blue (0.22, +0.08, 0.10), red (0.22, -0.08, 0.10)
  (base-frame = world-frame). Cubes are 3×3×3 cm (visual + collision).
- **Marker position**: (0.28, 0.00, 0.085), radius 5 cm, visual-only.
- **Gripper**: open = 1.2 rad, closed = -0.15 rad (published on
  `/gripper_controller/joint_trajectory`).
- **Proximity gate**: before `node.attach()` the expert now logs
  `TCP-ball dist at attach candidate: X.X cm`; if `>3 cm` it aborts. This is
  your primary success signal — do NOT weaken this threshold, it is how we
  detect "welded-from-10-cm-away" bogus grasps.
- **Success criterion** (reported by `pick_and_place_moveit.py` at exit):
  ball XY within 5 cm of marker after release. Z not checked.

## The loop (one iteration)

For each iteration, in order:

```bash
# 1. Clean slate — mandatory.
bash scripts/nuke_sim.sh

# 2. Rebuild anything you edited (vla is the usual target).
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vla --symlink-install
source install/setup.bash

# 3. Launch sim HEADLESS (fast; no GUI). Run in background; redirect logs.
#    headless:=true is the default but pass it explicitly to be defensive.
nohup ros2 launch arm_moveit_config arm_gazebo_moveit.launch.py \
    headless:=true > /tmp/sim_launch.log 2>&1 < /dev/null &
disown

# 4. Wait for readiness. Poll, do not sleep blind:
until grep -q "You can start planning now" /tmp/sim_launch.log 2>/dev/null; do
    sleep 1
done
# Confirm controllers are actually active (move_group ready != controllers ready):
source install/setup.bash
until ros2 control list_controllers 2>/dev/null \
        | grep -q "arm_controller.*active" \
    && ros2 control list_controllers 2>/dev/null \
        | grep -q "gripper_controller.*active"; do
    sleep 1
done

# 5. Run the expert — small trial count while iterating, bigger once green.
nohup ros2 run vla pick_and_place_moveit.py --trials 5 --randomize --seed 0 \
    > /tmp/expert_run.log 2>&1 < /dev/null &
disown

# 6. Wait for it to finish, then parse results.
wait  # if the shell sees the background job; otherwise:
until ! pgrep -f pick_and_place_moveit >/dev/null; do sleep 2; done

# 7. Grade the run. The expert prints one line per episode like:
#      RESULT: 3/5
#    And per episode:
#      TCP-ball dist at attach candidate: X.X cm
#      TCP too far from ball (...); aborting    # proximity gate trigger
#      SUCCESS  or  FAIL placement_xy_err=...
grep -E "RESULT|TCP-ball dist|TCP too far|SUCCESS|FAIL|home failed|cart descent failed|aborting" /tmp/expert_run.log
```

## Success definition (when to stop iterating)

Run a **20-trial** batch (`--trials 20 --randomize --seed 0`) and verify:
- `RESULT: N/20` with **N ≥ 17** (≥85% success).
- **All** successful episodes show a `TCP-ball dist at attach candidate`
  line < **1.5 cm** (ball actually inside the jaws, not brushed).
- No `TCP too far from ball (...)` aborts on the success path.
- Spot-check one episode with `headless:=false` at the end for visual
  confirmation (optional but recommended to write a short note in the
  changelog that you did).

If you can't reach 17/20 after ~10 iterations, stop and write a plain-English
diagnosis at the top of `CLAUDE_CHANGES.md` describing: (a) what the
dominant failure mode is, (b) what you tried, (c) what the owner would
need to decide. Do NOT keep spinning.

## What to tune in the expert

Roughly in order of likely payoff:

1. **`q_above` IK seed and orientation**. `above_ball_pos` currently solves
   with `orientation_weight=0.0` — the resulting `home_rot` at approach may
   not have the gripper aligned pointing down, so the jaws close around air.
   Try: solve with `down_rotation(yaw)` as the target rotation where yaw is
   aligned with the approach vector from base to ball XY. Add a
   `wrist_roll_lock` so the jaws stay perpendicular to the approach (this was
   the fix attempted for the red-ball drift earlier).
2. **`hover_dz` and `grasp_tcp_dz`** (in `Cfg`). Currently 0.15 and 0.0. If
   the proximity-gate distance is consistently >3 cm but the XY is close,
   you are hovering too high or descending too little — try `grasp_tcp_dz`
   slightly negative (e.g., -0.005) to drive the TCP into the cube for a
   tighter pinch, or raise `hover_dz` if the descent is colliding with the
   ball on the way in.
3. **`cart_max_step`** for `compute_cartesian`. If `compute_cartesian` is
   returning 0.00 fraction ("cartesian plan empty"), increase step size
   tolerance or fall back faster. The current fallback uses local IK with
   `orientation_weight=0.0`, which produces the "TCP ended 11 cm from ball"
   outcome — this is the top suspect.
4. **Shoulder-lift bias for pre-approach**. `grasp_spike_local.py` has a
   `shoulder_lift_bias_rad` that rolls the elbow backward 20–30° before
   descent. The expert does NOT have this yet. Port it over: after
   `q_above` is solved, nudge `q_above[1]` by `+0.35 rad` (about 20°) and
   re-execute the joint goal before the cartesian descent. Confirm in a
   visual run that this improves the approach geometry.
5. **Proximity gate threshold**. DO NOT raise it above 3 cm. If you are
   tempted, you are papering over a real miss — fix the upstream approach
   instead.

## Do-not-touch list

- `src/arm_description/worlds/ball_world.sdf` — positions/sizes are set to
  the user's current preference. Only touch if the expert absolutely cannot
  reach (and even then, ask in the changelog entry).
- `src/arm_description/description/so101.urdf.xacro` — especially the
  `<origin xyz="0 0 -0.2"/>` on `world_to_base` and the DetachableJoint
  plugins. Changing any of these invalidates the frame / grasp assumptions.
- `scripts/nuke_sim.sh` — works as is. Run it every iteration.
- `data/demos_lerobot_final/` — the fine-tune dataset. Treat as read-only.

## Changelog discipline

Every iteration that changes a file must add an entry to
`CLAUDE_CHANGES.md` (newest at top) with:
- Date (today's is filled in by the user; just use 2026-04-23).
- One sentence of WHY (not WHAT — the diff is the what).
- The grade of the run that motivated the change (e.g., "3/10 before, 8/10
  after").

Without this the Stop hook will loop you.

## If you get stuck

- "no joint state" at startup → DDS discovery timing. Nuke, run
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and relaunch.
- "home failed" repeatedly at 5s intervals → WSL2 clock jump
  (`time.time()` discontinuity). Ask the owner to keep Windows awake.
- Compute_cartesian returns 0.00 fraction every time → KDL/TRAC-IK does
  not like the orientation target. Check `at_ball`'s rotation matrix
  determinant = +1 and that the approach axis is well-defined.
- Balls moving with the arm right after launch → the t=8s auto-detach
  in `arm_gazebo.launch.py` didn't fire. Publish manually:
  `ros2 topic pub --once /detach_blue std_msgs/msg/Empty {}` (same for red).

## Target to beat

As of the last run (frame-fix reversion just applied, not yet verified):
- `RESULT: 0/3` — all three episodes aborted on the proximity gate with
  TCP-ball distances of ~11 cm after the frame-fix revert, ~26 cm before.
- Compute_cartesian returned 0.00 fraction every episode — local-IK
  fallback placed TCP too high and too far forward.

First milestone: get the proximity gate to trigger at <3 cm even once.
Second milestone: 3/3 with placement within 5 cm of the marker.
Third milestone: 17/20 on the randomized seed=0 batch.
