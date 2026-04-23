# Next steps — SmolVLA fine-tune and deploy

You're picking up this project with the SmolVLA fine-tune **actively running on UCF's nobel server**. This doc is self-contained — read it top to bottom before doing anything.

## Where we are right now (2026-04-23)

**A 3k-step smoke-test fine-tune is running in a tmux session on nobel.** Full context:

- Host: `pe606840@nobel.ece.ucf.edu` (SSH over UCF VPN; see `cmds.txt` for tunnel + `openconnect-sso` command).
- Working dir: `~/vla_smolvla/` on nobel. Contains the dataset (`data/demos_lerobot_final/`), a venv (`.venv/`), and the training wrapper (`train_relaxed.py`).
- tmux session name: `smolvla`. Reattach with `tmux a -t smolvla`.
- GPU: `CUDA_VISIBLE_DEVICES=0` — nobel has multiple cards, we pin to one. `nvidia-smi` on nobel to pick a free card if the run needs to be redone.
- Config: LoRA-style partial fine-tune (vision encoder frozen + train_expert_only=True are SmolVLA defaults), 100M trainable params out of 450M, batch 64, lr 1e-4, ~0.35s/step → ~17 min for the smoke test.
- Checkpoints land in `~/vla_smolvla/outputs/train/smolvla_so101_smoke/checkpoints/{001000,002000,003000}/pretrained_model/`.

**Do not assume training is still running when you pick this up.** Reattach and check. If finished, look at `loss` trajectory in the output — should be trending down from ~0.1 toward ~0.05. If crashed, the last error is probably one of the ones already patched in `train_relaxed.py` — look there first.

## The canonical training command

Smoke test (3k steps, ~17 min):

```bash
CUDA_VISIBLE_DEVICES=0 python train_relaxed.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.root=/home/pe606840/vla_smolvla/data/demos_lerobot_final \
    --dataset.repo_id=local/so101_pickplace_sim \
    --dataset.use_imagenet_stats=false \
    --batch_size=64 \
    --steps=3000 \
    --eval_freq=-1 \
    --save_freq=1000 \
    --output_dir=outputs/train/smolvla_so101_smoke \
    --policy.push_to_hub=false \
    --wandb.enable=false
```

Full run (30k steps, ~3 h): same command with `--steps=30000 --save_freq=5000 --output_dir=outputs/train/smolvla_so101`.

## Why we need `train_relaxed.py` (do NOT run `python -m lerobot.scripts.train` directly)

Our dataset has four incompatibilities with vanilla LeRobot 0.3.3 that the wrapper patches around. Don't remove the wrapper without fixing the underlying issues.

1. **Parquet timestamp jitter** (~1% around 100 ms nominal). ROS recorder wasn't clocked precisely. LeRobot's `check_timestamps_sync` defaults to 1e-4s tolerance and raises on every frame. Wrapper replaces the check with a no-op.

2. **Video decoder tolerance is separate**, stored as `self.tolerance_s` on the dataset instance, default 1e-4s again. Same jitter trips it. Wrapper bumps it to 0.1s after `__init__`.

3. **Off-by-one between parquet rows and mp4 frames.** Our `scripts/convert_to_lerobot.py` sometimes produced (n+1)-row parquet / n-frame mp4. The loader errors on out-of-range frame indices. Wrapper clamps timestamps to the last available frame in `decode_video_frames_torchcodec`. **This is a real bug in the converter — fix before another dataset round.**

4. **`meta/stats.json` is missing image keys.** `make_dataset` unconditionally tries `dataset.meta.stats["observation.images.third_person"][stats_type] = ...` and KeyErrors if the image key isn't there. The `--dataset.use_imagenet_stats=false` flag sidesteps this by not entering that code path. SmolVLA's SigLIP vision encoder is pretrained with its own normalization, so this is safe.

If you see new errors of similar shape, extend `train_relaxed.py` (or paste them to a new Claude — it'll know what to patch).

## Your immediate work

### Step 1. Verify the smoke test completed cleanly

```bash
ssh pe606840@nobel.ece.ucf.edu
tmux a -t smolvla
# scroll up — look for `step:3000` log line
# if present: loss should be 0.02-0.05, training exited cleanly
# checkpoint should exist at outputs/train/smolvla_so101_smoke/checkpoints/003000/pretrained_model/
```

If the smoke run crashed, debug. If it's still running, just wait.

### Step 2. Kick off the full 30k-step run

Same tmux session (or a new one `tmux new -s smolvla_full`). ~3 hours on A100 at batch 64:

```bash
cd ~/vla_smolvla && source .venv/bin/activate
CUDA_VISIBLE_DEVICES=0 python train_relaxed.py \
    --policy.path=lerobot/smolvla_base \
    --dataset.root=/home/pe606840/vla_smolvla/data/demos_lerobot_final \
    --dataset.repo_id=local/so101_pickplace_sim \
    --dataset.use_imagenet_stats=false \
    --batch_size=64 \
    --steps=30000 \
    --eval_freq=-1 \
    --save_freq=5000 \
    --output_dir=outputs/train/smolvla_so101 \
    --policy.push_to_hub=false \
    --wandb.enable=false
```

Detach from tmux (`Ctrl+b d`), reconnect periodically. Done when loss plateaus (~0.01-0.02 typical for SmolVLA on in-distribution data).

### Step 3. Pull the checkpoint back

From local:

```bash
rsync -aP pe606840@nobel.ece.ucf.edu:~/vla_smolvla/outputs/train/smolvla_so101/checkpoints/030000/pretrained_model/ \
    /home/peyton/vla_arm/checkpoints/smolvla_so101/
```

### Step 4. Write the inference ROS node

Doesn't exist yet. Target: `src/vla/vla/smolvla_inference_node.py`. Must:

1. Subscribe to `/third_person/image_raw` (sensor_msgs/Image, 224×224 RGB) and `/joint_states` (sensor_msgs/JointState, needs re-ordering to `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]`).
2. Accept a task string parameter (`"pick the blue ball and place it on the green marker"` etc.).
3. Load the policy once at startup: `SmolVLAPolicy.from_pretrained("/path/to/checkpoints/.../pretrained_model")`. Import is from `lerobot.policies.smolvla.modeling_smolvla` in 0.3.3 (flat layout — no `lerobot.common.*`).
4. On each image callback (or timer at 10 Hz to match training rate), build a batch dict matching the training features, call `policy.select_action(batch)`, slice the 6-D output into 5-D arm + 1-D gripper trajectories.
5. Publish to `/arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory, joint names `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]`) and `/gripper_controller/joint_trajectory` (joint `[gripper]`). See how `src/vla/vla/pick_and_place_moveit.py` builds these messages — copy that shape.
6. Register in `src/vla/CMakeLists.txt` under `install(PROGRAMS ...)` or `colcon build` won't install it.

Install LeRobot locally too: `pip install "lerobot[smolvla]==0.3.3"` into the repo's `.venv/`. Make sure the ROS node uses that venv, not system python.

### Step 5. Closed-loop Gazebo eval

Follow the mandatory clean-slate rule (see CLAUDE.md):

```bash
bash scripts/nuke_sim.sh
ros2 launch arm_description arm_gazebo.launch.py   # no MoveIt needed for inference
# separate terminal:
ros2 run vla smolvla_inference_node.py --ros-args \
    -p checkpoint_path:=/home/peyton/vla_arm/checkpoints/smolvla_so101 \
    -p task:="pick the blue ball and place it on the green marker"
```

Success criterion (same as demo collection): ball XY within 5 cm of marker at (0.22, 0). Run 20+ trials per color, report success rate.

## Known gotchas discovered during this run

- **LeRobot 0.3.3 is the version to use.** 0.4.x restructured the dataset format to v3.0 and expects v3.0 datasets. Our dataset is v2.0. Only `v30/convert_dataset_v21_to_v30.py` ships — no v2.0→v2.1 converter in 0.3.3+. Sticking with 0.3.3 avoids the whole migration.
- **Package layout in 0.3.3 is flat**: `lerobot.datasets.*`, `lerobot.policies.*`, `lerobot.scripts.*`. The older docs referencing `lerobot.common.datasets.*` are wrong for this version.
- **Entry point is `python -m lerobot.scripts.train`**, not `lerobot-train` (that's a 0.4.x console script).
- **`--policy.path=lerobot/smolvla_base`** is how you load the pretrained checkpoint; `--policy.pretrained_path` is 0.4.x syntax.
- **`--env.type=none`** is 0.4.x — 0.3.3 rejects it. Just omit; `--eval_freq=-1` alone disables sim eval.
- **Multi-GPU nobel machines will auto-shard the model via accelerate.** Pin with `CUDA_VISIBLE_DEVICES=0` or you hit layer-norm cross-device errors.
- **Converter off-by-one** between parquet rows and mp4 frames in some episodes. Not catastrophic (wrapper clamps, max 1-frame loss per affected episode), but fix `scripts/convert_to_lerobot.py` before generating another dataset.
- **Image stats absent from `meta/stats.json`.** Same converter gap. Unblocked by `--dataset.use_imagenet_stats=false` since SigLIP has its own normalization, but should be computed properly next time.

## SSH / VPN reminder

From `cmds.txt`:

```bash
# VPN — in a terminal separate from SSH
openconnect-sso --server secure.vpn.ucf.edu -- '--script=vpn-slice nobel.ece.ucf.edu'

# SSH
ssh pe606840@nobel.ece.ucf.edu
```

## What NOT to do

- **Don't run `python -m lerobot.scripts.train` directly.** Use `train_relaxed.py`.
- **Don't upgrade LeRobot past 0.3.3** unless you're ready to migrate the dataset to v3.0 (non-trivial, restructures file layout).
- **Don't try full fine-tune (`train_expert_only=False`, vision encoder unfrozen) as a first attempt.** LoRA-style default converges fast and uses way less memory.
- **Don't skip the clean-slate rule during closed-loop eval.** Ghost controllers + DetachableJoint state leak across runs. `bash scripts/nuke_sim.sh` every time.

## Read next

- `CLAUDE.md` — full project conventions, every gotcha, every key file.
- `train_relaxed.py` on nobel (and mirrored in this repo at `/home/peyton/vla_arm/train_relaxed.py`) — the wrapper + inline comments explaining each patch.
- `CLAUDE_CHANGES.md` — dated log, newest at top. The 2026-04-23 entry explains this training run.
