# SmolVLA installation guide

Install SmolVLA on a GPU server for fine-tuning on the SO-101 pick-and-place
demos (`data/demos_lerobot_final/`).

## Requirements

- Linux box with an NVIDIA GPU. Recommended: A100 (40/80 GB), H100, or
  24 GB RTX 4090. Full fine-tune of SmolVLA fits in ~16 GB; LoRA fits in
  ~10 GB.
- Python 3.10+ (3.12 is fine).
- `git`, `pip`, `huggingface-cli` (the latter ships with `huggingface_hub`
  which comes as a transitive dep of `lerobot`).
- Internet access on the GPU box (or pre-download weights on another
  machine — see "Air-gapped setup" below).

Verify prerequisites:

```bash
nvidia-smi                # should show your GPU + driver version
python3 --version         # >= 3.10
```

## 1. Create an isolated environment

```bash
python3 -m venv ~/smolvla_env
source ~/smolvla_env/bin/activate
pip install --upgrade pip setuptools wheel
```

Put the `source` line in your `~/.bashrc` if you want it auto-activated.

## 2. Install LeRobot with SmolVLA extras

```bash
pip install "lerobot[smolvla]"
```

This pulls LeRobot, SmolVLA dependencies (including `transformers`,
`torch`, `torchvision`, `peft`), plus the video codecs used by the dataset
format.

Alternative (latest `main`):

```bash
pip install "git+https://github.com/huggingface/lerobot.git#egg=lerobot[smolvla]"
```

Verify:

```bash
python -c "from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print('ok')"
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# expected: True on the second line
```

## 3. Log in to HuggingFace (if weights are gated)

`lerobot/smolvla_base` is public at time of writing, but log in just in case:

```bash
huggingface-cli login
# paste a read token from https://huggingface.co/settings/tokens
```

## 4. Pre-pull the pretrained weights (optional, recommended)

The training script will download weights lazily on first run. If you'd rather
cache them up front (or mirror them offline):

```bash
huggingface-cli download lerobot/smolvla_base
```

This writes to `~/.cache/huggingface/hub/models--lerobot--smolvla_base/`
(~500 MB).

## 5. Get the dataset onto the GPU box

From the demo-collection machine, copy the LeRobot dataset:

```bash
# option A: rsync directly (source is ~15 MB, finishes in seconds)
rsync -aP user@source-host:/path/to/vla_arm/data/demos_lerobot_final/ \
          ~/datasets/so101_pickplace_sim/

# option B: push to HuggingFace hub first, then pull
# (on source machine)
python -c "
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset.from_root('/path/to/demos_lerobot_final')
ds.push_to_hub('your-hf-user/so101_pickplace_sim_200')
"
# (on GPU box — no explicit download needed; training script pulls it)
```

Sanity-check the load:

```bash
python - <<'EOF'
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset.from_root('~/datasets/so101_pickplace_sim')
print(f'episodes: {ds.num_episodes}, frames: {len(ds)}')
print('features:', list(ds.features.keys()))
sample = ds[0]
print('sample image shape:', sample['observation.images.third_person'].shape)
print('sample state:', sample['observation.state'])
EOF
```

Expected:
- `episodes: 200, frames: 44202`
- features include `observation.images.third_person`, `observation.state`, `action`, `timestamp`, ...
- sample image shape `torch.Size([3, 224, 224])` (PyTorch CHW format, not the HWC uint8 on disk)

## 6. First training run

Short smoke test to confirm the pipeline works end-to-end (3k steps, ~30 min
on an A100):

```bash
python -m lerobot.scripts.train \
    --policy.type=smolvla \
    --policy.pretrained_path=lerobot/smolvla_base \
    --dataset.root=~/datasets/so101_pickplace_sim \
    --dataset.repo_id=local/so101_pickplace_sim \
    --env.type=none \
    --batch_size=32 \
    --steps=3000 \
    --eval_freq=-1 \
    --save_freq=1500 \
    --output_dir=outputs/smoke_test \
    --policy.push_to_hub=false \
    --wandb.enable=false
```

Watch the `train/loss` trace; it should decrease from ~1.5-2.0 to ~0.5 within
the first few hundred steps. If it's flat or diverging, stop and investigate
(batch size, dataset load, GPU memory).

## 7. Full fine-tune

Once the smoke test looks healthy:

```bash
python -m lerobot.scripts.train \
    --policy.type=smolvla \
    --policy.pretrained_path=lerobot/smolvla_base \
    --dataset.root=~/datasets/so101_pickplace_sim \
    --dataset.repo_id=local/so101_pickplace_sim \
    --env.type=none \
    --batch_size=64 \
    --steps=30000 \
    --eval_freq=-1 \
    --save_freq=5000 \
    --output_dir=outputs/smolvla_so101_full \
    --policy.push_to_hub=false \
    --wandb.enable=true                  # optional: requires `wandb login`
```

Expected runtime: ~4 h on a single A100 at batch size 64.

Checkpoints land under `outputs/smolvla_so101_full/checkpoints/`. The final
model is directly loadable with:

```python
from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
policy = SmolVLAPolicy.from_pretrained("outputs/smolvla_so101_full/checkpoints/030000/pretrained_model")
```

## 8. Port the trained policy back for closed-loop sim eval

On the Gazebo machine, create an inference node that:

1. `SmolVLAPolicy.from_pretrained(...)` at startup.
2. Subscribes to `/third_person/image_raw` and `/joint_states`.
3. Each tick: builds the `{observation.images.third_person, observation.state}`
   dict, calls `policy.select_action(...)`, gets a 6-DoF joint command.
4. Publishes `action[:5]` to `/arm_controller/joint_trajectory` and
   `action[5]` to `/gripper_controller/joint_trajectory`.

`src/vla/vla/vla_action_client.py` is the closest analog — same ROS 2 shape,
just with a local `select_action` call instead of the HTTP POST.

## Air-gapped setup

If the GPU box has no internet:

```bash
# on a machine WITH internet
pip download "lerobot[smolvla]" -d ~/wheelhouse       # downloads all wheels
huggingface-cli download lerobot/smolvla_base         # caches model
tar czf lerobot_cache.tar.gz \
    ~/wheelhouse \
    ~/.cache/huggingface/hub/models--lerobot--smolvla_base

scp lerobot_cache.tar.gz gpu-box:~/

# on the GPU box
tar xzf ~/lerobot_cache.tar.gz -C ~/
pip install --no-index --find-links ~/wheelhouse "lerobot[smolvla]"
# model is now in ~/.cache/huggingface/hub — LeRobot will find it
```

## Troubleshooting

**`ImportError: cannot import name 'SmolVLAPolicy'`** — old LeRobot version.
Upgrade to `lerobot >= 0.3.0` (check `pip show lerobot`).

**`torch.cuda.OutOfMemoryError`** — drop `--batch_size` (try 32, 16), or
enable LoRA with `--policy.use_lora=true` (saves ~6 GB).

**Dataset load fails with `ValueError: Unknown field 'observation.images.third_person'`** —
check `meta/info.json` has that feature key and the video directory exists
at `videos/chunk-000/observation.images.third_person/episode_NNNNNN.mp4`.
You should see 200 such files.

**Training loss stays flat above 1.5** — usually a data ordering issue. Print
a few samples via `ds[i]` and confirm the image and action arrays look right.

**`ffmpeg` decode errors on video read** — install `pip install av` or the
system `ffmpeg`.

## What comes after training

1. Evaluate closed-loop in Gazebo (next step).
2. If sim eval is ≥50% success, try on real SO-101 hardware. Expect a
   sim-to-real gap — the `DetachableJoint` magic grasp in sim doesn't
   translate to a real friction-based grasp. You may need a second data
   pass with real-arm teleop demos.
3. If sim eval is <30% success, look at: task-distribution skew
   (143 blue / 57 red), demo quality (some "successes" place the ball
   just inside the 5 cm marker tolerance), and whether the action space
   matches what SmolVLA expects (LeRobot's SO-100/SO-101 convention is
   6-DoF joint positions, which is what our demos provide).
