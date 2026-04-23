# Handoff: SmolVLA fine-tuning on SO-101 pick-and-place demos

You're picking up a project where Claude-in-container has already collected
200 successful pick-and-place demos in LeRobot v2 format. Your job is the
fine-tune. You're on a **different machine** than where demos were collected;
see "Dataset transfer" below for the cross-device move.

## What's already done

1. **Gazebo sim + SO-101 URDF + MoveIt2 infra** — `/workspace/src/{arm_description,arm_moveit_config,vla}/`
2. **Scripted expert policy** — `src/vla/vla/pick_and_place_moveit.py` does
   home → hover → descend → close → lift → transport → place → release → home
   via MoveIt's `compute_cartesian_path` (with local-IK fallback) and a
   DetachableJoint sim plugin for reliable grasp.
3. **Demo recorder** — `src/vla/vla/demo_recorder.py` subscribes to
   `/third_person/image_raw`, `/joint_states`, and the two controllers'
   joint_trajectory topics, and writes synchronized tuples at 10 Hz.
4. **LeRobot v2 converter** — `scripts/convert_to_lerobot.py`.
5. **200 successful demos collected** — see next section.

Full history in `CLAUDE_CHANGES.md` (newest first) and open-questions in
`QA_LOG.md`.

## The dataset you're fine-tuning on

Primary artifact: **`data/demos_lerobot_final/`** (~15 MB on disk)

- **200 episodes**, **44,202 frames total** @ 10 Hz
- **Schema**: LeRobot v2 — parquet per episode + h264 video per episode +
  `meta/{info.json, episodes.jsonl, tasks.jsonl, stats.json}`
- **Features**:
  - `observation.images.third_person`: (224, 224, 3) uint8, encoded h264
  - `observation.state`: float32[6] — joint positions in order
    `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]`
  - `action`: float32[6] — commanded joint positions, same order
  - `timestamp, frame_index, episode_index, index, task_index`
- **Task mix**:
  - 143 × "pick the blue ball and place it on the green marker"
  - 57 × "pick the red ball and place it on the green marker"
  - Skewed because red-ball placement has a ~6-10 cm systematic IK drift
    past the marker; blue side is well-behaved. If you want balanced colors,
    fine-tune the red-side IK (`wrist_roll` pin per color) and collect a
    red-only top-up batch. For a first fine-tune, the skew is probably OK —
    SmolVLA has strong SO-100/SO-101 priors.
- **Success criterion used during collection**: ball XY within 5 cm of
  marker (0.22, 0.0) in base_link frame. Z was not checked.
- **Raw (unfiltered) attempts** are at `data/demos_raw_final/batch_000..016/`
  (408 attempts) — keep if you want to re-filter with a different criterion.

Verify load before anything else:

```python
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset.from_root("/path/to/demos_lerobot_final")
print(len(ds), ds.features.keys())
# should print 44202 and dict_keys(['observation.images.third_person',
#   'observation.state', 'action', 'timestamp', ...])
```

If that works, skip the troubleshooting section at the bottom.

## Dataset transfer (container → new machine)

You're on a container at `/workspace` whose `.git` points at a host path that
won't exist on the new machine. Three transfer options:

1. **Push to a remote and clone** — cleanest. Run the git ops on the HOST
   (not inside the container), since `git` is broken in the container:
   ```bash
   # host
   cd /home/peyton-peyton/vla_arm-claude
   git push -u origin HEAD
   # new machine
   git clone <remote-url>
   ```

2. **Bundle transfer**:
   ```bash
   # host
   cd /home/peyton-peyton/vla_arm-claude
   git bundle create /tmp/vla_arm.bundle --all
   scp /tmp/vla_arm.bundle new-machine:/tmp/
   # new machine
   git clone /tmp/vla_arm.bundle vla_arm && cd vla_arm && git checkout <branch>
   ```

3. **rsync** — simplest, excludes build artefacts:
   ```bash
   rsync -aP \
       --exclude='install/' --exclude='build/' --exclude='log/' \
       --exclude='data/demos_raw_final/' --exclude='.venv/' \
       /home/peyton-peyton/vla_arm/ new-machine:~/vla_arm/
   ```

Either way: you want `data/demos_lerobot_final/` on the training box.

## SmolVLA fine-tuning plan

### Prime directive

This is the **community-default path** for SO-100/SO-101 (487 pretraining
datasets, LeRobot-native consumption, ~4 h on a single A100). **Do NOT try
OpenVLA-7B** — it has no SO-101 prior and would require RLDS conversion. The
original handoff confirmed this; sticking with it.

### Compute target

The handoff flagged the **UCF A100** that historically hosted OpenVLA
inference (see `cmds.txt`, SSH tunnel target). Confirm with the user it's
free before scheduling 4 h of training. Alternatives: any A100/H100 box,
or a 24 GB 4090 (LoRA fine-tune fits, full fine-tune tight).

### Install

```bash
# create a python venv; system ROS env is NOT needed for training
python3 -m venv .venv_finetune
source .venv_finetune/bin/activate
pip install --upgrade pip
pip install "lerobot[smolvla]"     # pulls lerobot + smolvla deps
# or: pip install git+https://github.com/huggingface/lerobot.git
# sanity check:
python -c "from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print('ok')"
```

### Upload dataset to HuggingFace (optional but recommended)

LeRobot's training loop accepts a local path, but HF-hub makes reproducibility
and multi-machine runs easier:

```bash
python -c "
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset.from_root('/path/to/demos_lerobot_final')
ds.push_to_hub('your-hf-user/so101_pickplace_sim_200')
"
```

The handoff's Q5 was "push to HF or keep local?" I deferred the call. You
decide.

### Training command

LeRobot's training script, given this dataset:

```bash
python -m lerobot.scripts.train \
    --policy.type=smolvla \
    --policy.pretrained_path=lerobot/smolvla_base \
    --dataset.repo_id=your-hf-user/so101_pickplace_sim_200 \
    --env.type=none \
    --batch_size=64 \
    --steps=30000 \
    --eval_freq=-1 \
    --save_freq=5000 \
    --output_dir=outputs/train/smolvla_so101 \
    --policy.push_to_hub=false \
    --wandb.enable=false
```

If local-path loading instead of HF hub:

```bash
    --dataset.root=/path/to/demos_lerobot_final \
    --dataset.repo_id=local/so101_pickplace_sim
```

Expected runtime on an A100: ~4 h for ~30k steps. Adjust `--steps` based on
convergence; LeRobot logs `train/loss` you can curve-trace.

### Evaluation (closed-loop in Gazebo)

After training:

1. Copy the trained checkpoint back to the Gazebo machine.
2. Wrap the policy in an inference node that subscribes to
   `/third_person/image_raw` + `/joint_states` and publishes actions to
   `/arm_controller/joint_trajectory` + `/gripper_controller/joint_trajectory`.
   The existing `src/vla/vla/vla_action_client.py` is the closest analog
   — it does the HTTP-to-remote-VLA pattern. For a local-inference SmolVLA
   node, strip the HTTP, load the policy directly with
   `SmolVLAPolicy.from_pretrained('outputs/train/smolvla_so101')`, run
   inference in a callback.
3. Run against `arm_gazebo.launch.py` + the 200-demo distribution. Success
   criterion: ball XY within 5 cm of marker.

Expected sim-to-real gap: this is all sim training, so real-hardware transfer
is the next phase after sim evaluation. The handoff's ultimate goal is real
SO-101 hardware.

## Known issues + caveats

1. **Task imbalance** (143 blue / 57 red) — documented above. If model
   underperforms on red at eval time, collect a red-only top-up.
2. **Red-ball IK drift** during collection was systematic: ~7 cm past the
   marker in -Y. The fix suggested (but not implemented) is pinning
   `wrist_roll = 1.5708` mirrored per ball side in `pick_and_place_moveit.py`
   + warm-starting the above_marker IK from current state. My attempt at
   this made things worse — the wrist_roll pin at π/2 for both sides caused
   the arm to knock balls off the table during approach. Needs investigation.
3. **Demos use a DetachableJoint "magic grasp"** — gz-sim plugin that
   attaches ball-to-gripper on `/attach_<color>` topic publish. The jaws
   visibly pinch the ball in frames the recorder saves, but the grip is
   sim-cheat, not friction-based. Real-hardware transfer needs a real grasp
   — the VLA only learns the close-gripper timing from these demos, not
   the physical grasp stability. Budget time for closing this gap on
   real hardware.
4. **Balls have box collision with sphere visual** — the visual in frames
   is a sphere but physics treats it as a 5 cm cube. Doesn't affect training
   since the model sees only images.
5. **`tcp_jaw_link` frame** was added to `so101.urdf.xacro` specifically for
   MoveIt planning — it's the midpoint between the two jaws. If you rebuild
   MoveIt configs from scratch via MoveIt Setup Assistant, re-add it.

## Must-read files before touching anything

- `CLAUDE_CHANGES.md` — dated log of everything Claude did. Newest at top.
  The most recent 3 entries cover the grasp fix, the collection run, and
  this handoff.
- `QA_LOG.md` — open questions from the earlier handoff with my answers.
- `src/arm_description/description/so101.urdf.xacro` — robot + DetachableJoint
  plugins.
- `src/vla/vla/pick_and_place_moveit.py` — expert policy that generated demos.
- `src/vla/vla/demo_recorder.py` — recorder.
- `scripts/{collect_200_demos.sh,filter_successful_demos.py,convert_to_lerobot.py}`
  — collection pipeline.

## Your first 30 minutes

1. Clone the repo to the fine-tune box (see "Dataset transfer").
2. Sanity-check the dataset loads:
   `python -c "from lerobot.common.datasets.lerobot_dataset import LeRobotDataset; ds = LeRobotDataset.from_root('data/demos_lerobot_final'); print(len(ds))"`
3. Confirm UCF A100 (or whatever compute) is available with the user.
4. Install `lerobot[smolvla]` in a venv.
5. Kick off a short (3k-step) training run to verify the pipeline works end
   to end.
6. If the short run converges without errors, extend to the full ~30k steps.
7. Download checkpoint, port to the Gazebo machine, do closed-loop sim eval.

## Open questions for the user

1. Use the UCF A100, or a different box?
2. Push the dataset to HuggingFace Hub (`your-hf-user/so101_pickplace_sim_200`) or keep local-only?
3. Should the red-imbalance be corrected before training (collect ~86 more red demos to hit 143/143)?
4. Training hyperparameters: full fine-tune vs LoRA?
5. When ready for real-hardware eval, do you want Claude to help write the inference node that bridges trained-policy → `/arm_controller/joint_trajectory`?
