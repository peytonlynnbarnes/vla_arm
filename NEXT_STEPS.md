# Next steps — diagnose & fix the SmolVLA closed-loop failure

You are picking this up after a complete first-pass closed-loop eval on the
re-fine-tuned SmolVLA checkpoint. The fine-tune ran clean on the new
2026-04-25 dataset (103 blue / 97 red, friction grasps), but the policy
**fails to grasp** in sim. Your job is to diagnose and fix that, not
to re-collect data or retrain from scratch yet — try the cheap knobs first.

## Where we are right now (2026-04-25)

- **Checkpoint pulled locally** at `/home/peyton/vla_arm/checkpoints/smolvla_so101/`
  (`config.json`, `model.safetensors`, `train_config.json`).
- **Inference node written and built**: `src/vla/vla/smolvla_inference_node.py`,
  registered in `src/vla/CMakeLists.txt`, installed at
  `install/vla/lib/vla/smolvla_inference_node.py`.
- **lerobot 0.3.3 installed** in `.venv` along with `torch 2.7.1+cu126`,
  `transformers 4.51.3`, etc.
- **numpy pinned to 1.26.4** in `.venv` — pip tried to upgrade to 2.4.4
  during the lerobot install, which would have broken every ROS Jazzy
  Python binding (`rclpy`, `cv_bridge`, etc.). Re-pinning fixed it. **Do
  not let any subsequent pip install bump numpy past 2.0.**
- **Closed-loop eval was attempted once on blue task.** Result: **FAIL.**
  See "What happened in the first eval" below.

## What happened in the first eval

Run was:

```bash
bash scripts/nuke_sim.sh
ros2 launch arm_description arm_gazebo.launch.py headless:=true   # bg
.venv/bin/python install/vla/lib/vla/smolvla_inference_node.py --ros-args \
    -p checkpoint_path:=/home/peyton/vla_arm/checkpoints/smolvla_so101 \
    -p task:='pick the blue ball and place it on the green marker' \
    -p control_rate_hz:=10.0
```

Observations:

- **RTF** (`gz topic -e -t /stats`): 0.98–1.07× — sim is healthy on the
  RTX 2060.
- **Inference startup**: ~62 s on first run (HuggingFace downloads
  `HuggingFaceTB/SmolVLM2-500M-Video-Instruct` + SigLIP weights). Cached
  after that.
- **Inference loop runs cleanly** at 10 Hz on cuda, no exceptions.
- **The arm moved**, ending at joint state ≈ (-0.16, -0.02, -0.22, 1.65,
  -0.81), which is closest to `BLUE_KEYFRAMES['above_marker']` =
  (0, 0.02, -0.42, 1.658, -0.48) — i.e., the *post-place transit* pose,
  not the *grasp* pose.
- **Gripper stayed at 1.19 rad (open)** the entire run. Never closed.
- **Latest commanded action ≈ current state** — the policy converged to a
  near-zero-delta steady output, just holding the half-pose.
- **Blue cube final position**: (0.219, 0.080, 0.095) — barely moved
  from initial (0.220, 0.080). Distance to marker (0.28, 0) = **0.101 m**
  vs. 0.05 m success threshold. Cube was never touched.

## Leading hypothesis: training/inference time-horizon mismatch

This is the most plausible cause of the "freeze in a half-pose with
gripper open" failure mode, and it is the cheapest thing to test:

`demo_recorder.py` records `action` as the **final position of the most
recent `JointTrajectory` message**, not the next-step joint position.
The keyframe expert sends ~2–3 s trajectories, so consecutive recorded
frames at 10 Hz have nearly-identical action labels (the destination
doesn't change while the controller interpolates).

The policy therefore learned: *"emit a position that's a ~2–3 s waypoint
ahead of the current joint state."* But the inference node publishes
each commanded action as a `JointTrajectory` with `time_from_start =
0.3 s`. The controller starts interpolating, then 100 ms later a new
trajectory pre-empts it before any meaningful displacement occurs. The
arm drifts in the policy's general direction at a rate way below the
demo distribution → the joint state never enters the regime where the
demo would have called for `grasp` → policy never closes the gripper.

Confirming evidence: the arm did drift a substantial way (toward an
intermediate pose), but the displacement-per-tick was much smaller than
demo trajectories. With `n_action_steps=50` and `chunk_size=50` (5 s of
buffered actions per inference call), the policy commits to a 5-second
plan up front; if the plan doesn't actually execute (because each step
is throttled to 0.3 s of motion), the second chunk replans from a
now-OOD state.

## Immediate work (try in this order, escalate only if cheap fixes don't move the needle)

### Step A. Bump `command_horizon_sec` (no retrain, no checkpoint edit)

`smolvla_inference_node.py` already exposes `command_horizon_sec` as a
ROS parameter. Default 0.3 s; bump to 2.5 s to roughly match what the
expert sent. Each inference tick still arrives every 100 ms and
pre-empts, but each pre-empted trajectory now has time to actually move
the arm before the next one arrives.

```bash
bash scripts/nuke_sim.sh
ros2 launch arm_description arm_gazebo.launch.py headless:=true &
.venv/bin/python install/vla/lib/vla/smolvla_inference_node.py --ros-args \
    -p task:='pick the blue ball and place it on the green marker' \
    -p command_horizon_sec:=2.5
```

Watch for: the arm actually descending toward the cube, gripper closing,
cube moving. If you see *any* of those, this hypothesis is confirmed and
proceed to Step B.

### Step B. Lower `n_action_steps` so the policy re-plans more often

`n_action_steps=50` at 10 Hz = a 5 s open-loop horizon. The world drifts
a lot in 5 s — the cube position post-grasp, the joint state mid-place,
etc. Drop to 10 (1 s) or 20 (2 s) so the policy re-observes after each
chunk. This lives in `config.json` (and `train_config.json`) inside the
checkpoint. Edit just `config.json`:

```python
import json
p = '/home/peyton/vla_arm/checkpoints/smolvla_so101/config.json'
c = json.load(open(p))
c['n_action_steps'] = 10  # was 50
json.dump(c, open(p, 'w'), indent=4)
```

`chunk_size` (the model's predicted output dim) stays 50 — only how many
of those 50 you actually execute before re-querying changes. Re-run the
eval as in Step A.

### Step C. If A+B don't fix it: characterize failure across trials

Don't keep eyeballing one trial. Write a small loop that, per color:

1. `nuke_sim.sh` → relaunch sim → spawn cubes at canonical XY (no
   randomization yet)
2. Set the `task` parameter to match the color
3. Run inference for ~30 s
4. Read `gz model -m {ball} -p`, compute `dist_to_marker`, log `SUCCESS
   if dist < 0.05`
5. Repeat 10× per color

Report: success rate per color, where in the sequence the policy
plateaus (joint state at end), and whether the gripper ever closes. The
keyframe expert is ~89% on canonical — you want to know how far below
that SmolVLA is, and whether failures are positional (jaws miss the
cube) vs. perceptual (arm goes to wrong cube) vs. control (drops
mid-transit).

The inference node currently takes `task` as a startup-only ROS
parameter. If you want the eval loop to flip colors without restarting
the node, add either (a) a `/task` `String` subscriber, or (b) make
`task` a re-settable parameter that recomputes on the next inference
tick. Option (a) is symmetric with how `pick_and_place_moveit.py`
publishes `/episode/target` for the recorder.

### Step D. Only after A–C: decide whether to retrain

If A+B unblock grasping, you are done with this round — write up
results and move on. If A+B do *not* fix it, the issue is deeper and
the next round is a retrain with one or more of:

- **`image_transforms.enable: true`** (free win — already configured in
  `train_config.json`, just flipped off). Brightness/contrast/hue/sat/
  sharpness jitter to broaden lighting robustness.
- **Wider `--randomize` envelope.** Currently capped at ±1 cm because
  the linear keyframe correction in
  `pick_and_place_moveit.run_episode_keyframe` overshoots beyond that.
  Calibrate a quadratic correction term against arm geometry, or have
  the expert reject seeds outside its capability and re-roll. ±3–5 cm
  XY would give the policy a much broader cube-position distribution to
  generalize from.
- **More episodes.** 200 is on the low end; 500–1000 is more typical
  for VLA fine-tuning.
- **Visual distractors.** Add a third non-target cube of a different
  color. Right now the policy can win the prompt-following objective by
  just "go to whichever cube" — with two non-targets it has to actually
  parse the prompt.
- **Re-recorded actions as next-step joint positions** (instead of
  trajectory-destination positions). This is the proper fix to the
  hypothesis above — change `demo_recorder._on_timer` to record
  `action[t] = state[t+1]`, requiring a one-frame look-ahead buffer or
  a post-process step over each saved episode. Then retrain.
- **Unfreeze the vision encoder** (`freeze_vision_encoder: false`) only
  after the data-side levers are exhausted. Big capacity bump but needs
  more data and more steps to avoid catastrophic forgetting; the sim
  render is OOD for SigLIP's pretraining so this *should* help, but
  it's the most expensive change to validate.

## How to invoke things correctly

The script's shebang is `#!/usr/bin/env python3` which resolves to
`/usr/bin/python3` (numpy 1.26.4, no torch, no lerobot), **not**
`.venv/bin/python3`. Always invoke the inference node with the venv
python explicitly:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/python install/vla/lib/vla/smolvla_inference_node.py --ros-args ...
```

`.venv` was created without `--system-site-packages` (`pyvenv.cfg` says
`include-system-site-packages = false`), but ROS Python imports work
from `.venv/bin/python` because sourcing `/opt/ros/jazzy/setup.bash`
prepends ROS site-packages to `PYTHONPATH`. Source ROS first.

For closed-loop eval, always use `arm_gazebo.launch.py` (no MoveIt
needed; the inference node doesn't call `move_group`):

```bash
ros2 launch arm_description arm_gazebo.launch.py headless:=true
```

`bash scripts/nuke_sim.sh` is **mandatory** before every sim run.
Verify after with `pgrep -af "gz sim|controller_manager|move_group"`
— if anything other than your own pgrep shows up, `kill -9` it before
relaunching. Ghost `gz sim` processes survive `pkill -f 'ros2 launch'`
and silently hold state across "fresh" launches.

## Known gotchas (still apply, do not regress these)

- **lerobot 0.3.3 is the pinned version.** 0.4.x restructured the
  dataset format to v3.0 and changed the SmolVLA batch interface
  (`OBS_LANGUAGE_TOKENS` directly instead of `task` strings).
- **numpy must stay < 2.0** in the same venv as ROS bindings.
- **Package layout in 0.3.3 is flat**: `lerobot.policies.smolvla.modeling_smolvla`,
  not `lerobot.common.policies.*`.
- **Train via `train_relaxed.py` only** (on nobel) — do not invoke
  `python -m lerobot.scripts.train` directly. The wrapper patches four
  LeRobot/dataset incompatibilities; same patches still apply.
- **Multi-GPU nobel auto-shards via accelerate.** Pin
  `CUDA_VISIBLE_DEVICES=0` or you hit layer-norm cross-device errors.
- **Don't load the old (2026-04-23) checkpoint by mistake.** It was
  trained on bad data (DetachableJoint magic-attach, 143/57 split).

## SSH / VPN reminder

```bash
openconnect-sso --server secure.vpn.ucf.edu -- '--script=vpn-slice nobel.ece.ucf.edu'
ssh pe606840@nobel.ece.ucf.edu
# active venv is ~/vla_smolvla/.venv, dataset at ~/vla_smolvla/data/demos_lerobot_final/
```

## Read next

- `CLAUDE.md` — project conventions, world geometry, every gotcha.
- `CLAUDE_CHANGES.md` — dated log, newest at top. The 2026-04-25
  entries explain the inference node design, the eval run, and this
  diagnosis.
- `src/vla/vla/smolvla_inference_node.py` — current node implementation.
  `command_horizon_sec` and `control_rate_hz` are the two main ROS
  params for tuning without a retrain.
- `src/vla/vla/demo_recorder.py` — `_on_arm_traj` / `_on_timer` is where
  the "action = trajectory destination" recording happens. Relevant if
  you escalate to Step D.
