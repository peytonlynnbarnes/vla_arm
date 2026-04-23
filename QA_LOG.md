# Q&A Log

Claude-answered clarifying questions during the autonomous run starting 2026-04-23. The user will be unmonitored ~7 h, so open questions from `HANDOFF_moveit_smolvla.md` are answered here with rationale. Newest at top.

---

## 2026-04-23 — Post-grasp-fix update

**Grasp now works.** User asked to fix the grasp hold; see CLAUDE_CHANGES.md for
the detailed diff. Three things stacked: box collision (flat faces) + DetachableJoint
plugin (sim-level guaranteed attach on gripper-close) + success check before
detach. Grasp spike: 3/3 = 100%. Full pick-and-place expert policy: 4/10 =
40% on a 10-trial sweep (within the handoff's 40-70% expected range).

Task 4 (200 successful demos) now viable. At ~30 s/episode wall-clock + 40%
success, ~500 attempts = 4 hours for 200 successes.

---

## 2026-04-23 — End-of-session summary (user monitoring resume)

**Delivered:**
- Tasks 1, 2, 3, 5 — complete.
- Task 4 — blocked on grasp physics; full pipeline ready, just needs gripper geometry fix.

**What works:**
- MoveIt2 scaffolded, compiles, launches alongside Gazebo (headless).
- Split arm/gripper controllers wired to MoveIt via `moveit_simple_controller_manager`.
- Scripted pick-and-place expert policy runs 11-step MoveIt sequence per episode.
- Demo recorder captures (image, joint state, action, timestamp) at 10 Hz, writes per-episode `frames.npz` + `meta.json`.
- LeRobot v2 converter writes parquet + MP4 + meta directory; schema matches SmolVLA input.
- Reset between episodes: `gz world reset {model_only: true}` + ball remove+create.
- Nuke script recovers from zombie controllers.

**What doesn't:**
- Gripper jaws don't clamp the 3 cm sphere. `gripper_frame_link` (MoveIt's planning EE) sits ~10 cm beyond the actual jaws at the descent pose, so targeting TCP = ball center parks the jaws too high. Cartesian path only succeeds 50% because the home-rotation orientation isn't feasible mid-descent. Motion is safe (no wild arm swings), but the task always fails the "ball at marker" success criterion.

**Highest-leverage fix (when user returns):**
1. Add a `tcp_jaw_link` fixed-joint child of `gripper_link` in `so101.urdf.xacro` at the real jaw-close point. Repoint SRDF `chain tip_link` to it and change `ik_link_name` / `pose_link` in `pick_and_place_moveit.py`. This alone likely unblocks Task 4.
2. If (1) isn't enough, swap spheres for 4×4×4 cm cubes — flat surfaces grip reliably.
3. Last resort: `DetachableJoint` plugin (sim cheat) for guaranteed capture during demos.

**Sim leak check:** verified zero gz sim / controller_manager / move_group processes at end. Used explicit `bash /workspace/scripts/nuke_sim.sh` each restart.

**Container deps apt-installed at runtime — promote to Dockerfile:**
`ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-trac-ik-kinematics-plugin ros-jazzy-joint-state-broadcaster ros-jazzy-ros2controlcli ffmpeg` + `pip install pyarrow --break-system-packages`.

**Artifacts:**
- `/workspace/data/demos_raw/episode_{0000..0003}/` — 4 test episodes (132–133 frames each).
- `/workspace/data/demos_lerobot/` — 529 frames, 2 task variants, parquet+MP4+meta.

---

## 2026-04-23 — Handoff open questions

### Q1. Add MoveIt2 packages to `.devcontainer/Dockerfile` permanently, or install ephemerally per session?

**A: Permanent.** Rebuilding the image is a one-shot 5–10 min cost; re-installing MoveIt2 every session is ~3 min every time and loses state on restart. Going permanent also documents the dependency and keeps the container reproducible across machines. I'll add the apt lines to the Dockerfile, but I'll use ephemeral `sudo apt install` for this session to avoid losing runtime while rebuilding the image mid-task. Dockerfile update can land with the session.

### Q2. GPU passthrough now, or live with CPU rendering?

**A: Live with CPU.** Adding `--gpus all` requires nvidia-container-toolkit on the host (not in my control from inside the container), plus X11 mounts, plus `xhost` on the host session. All are host-side changes and therefore blockers; I can't test them without leaving the container. Demo collection at RTF ~0.2 is slow but not blocking — a ~10 s demo takes ~50 s wall-clock, 200 demos at 40% success = ~700 attempts × 50 s = ~10 h. That fits an overnight job. If later profiling shows CPU rendering is the dominant cost, the GPU route can be revisited; for this 7-h window, we stay on CPU.

### Q3. OMPL/RRTConnect for transport + Pilz LIN for descent/ascent — acceptable?

**A: Yes.** This is the standard MoveIt2 approach for pick-and-place. RRTConnect is fast for free-space transport (above-ball → above-marker) and Pilz LIN gives deterministic straight-line descents where collision with the target must be avoided. `compute_cartesian_path` is an alternative if Pilz gives trouble — I'll default to Pilz LIN and fall back to `compute_cartesian_path` if plan failures dominate.

### Q4. Episode ball-position distribution: uniform over table half? ±3 cm jitter? Colors/marker random?

**A: Defer to Task 3.** This is a Task 3 decision, not Task 1. Tentative plan I'll document when I get there:
- **Balls**: ±3 cm uniform jitter around (0.16, ±0.08, 0.11). Keeps kinematic slack intact (reach 60% max remains in range), avoids collisions between balls.
- **Colors**: fixed — the task is "pick the {red,blue}" so both always present.
- **Target color**: random per episode (50/50 red vs blue).
- **Marker**: fixed at (0.22, 0, 0.085). Randomizing the drop target across the table adds scope but not value for a pick-and-place policy whose success metric is "ball at marker XY."
- **Domain randomization hooks**: ball jitter, lighting intensity (±30%), camera pose pitch/yaw jitter (±2°), table diffuse color drawn from a small palette. These hooks are implemented but off by default so the first 200 demos can be clean.

### Q5. Dataset publishing: local-only or HuggingFace hub?

**A: Local-only for now.** Pushing to HF requires an HF token, a dataset namespace decision, and a license choice — all user calls, and none are in this handoff. Dataset is written to `data/demos_lerobot/` in repo with the LeRobot v2 schema. HF push is a one-liner (`dataset.push_to_hub`) when the user is ready.

### Q6. SmolVLA fine-tune compute target: UCF A100 or a different box?

**A: Flag as user-blocker, don't schedule.** The handoff explicitly says "confirm with the user before scheduling any training jobs." The UCF A100 is already running an OpenVLA inference server per `cmds.txt`; taking it over for a 4 h training job would stop that. This is a coordination call, not a Claude call. I'll prepare the training config and a `fine_tune.sh` script ready to run, but not actually start training.

---

## 2026-04-23 — Run-local decisions

### Q. Branch strategy inside the container?

**A: Work on current worktree branch; do not touch main.** `run.sh` already enforces branch protection. I'll use the current branch whatever it is. I won't create new branches without explicit instruction — too easy for the user to miss state on re-attach.

### Q. Can I push to remote / create PRs?

**A: No, absent explicit instruction.** `CLAUDE.md` requires explicit authorization. I'll commit locally if helpful for bisect/rollback but will not `git push`.

### Q. How to handle Gazebo session leaks?

**A: Aggressive teardown.** User specifically flagged this. Strategy:
- Before every sim launch: full TERM 0 kill sequence (pkill then kill -9 by PID) per `run_commands.txt`.
- After every test: same teardown.
- Launch Gazebo via `run_in_background` + explicit PID tracking.
- Register a shell-level trap so that any script I write exits cleanly.
- End of session: verify `ps -ef | grep -E 'gz sim|parameter_bridge|controller_manager|robot_state_publisher|ruby.*gz|move_group'` returns empty before stopping.
