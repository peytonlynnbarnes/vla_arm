# vla_arm

A ROS 2 Jazzy workspace for driving a simulated **SO-101** 6-DOF arm with
**SmolVLA**. A scripted joint-keyframe expert collects pick-and-place
demos in Gazebo, the demos fine-tune `lerobot/smolvla_base`, and the
fine-tuned policy runs closed-loop against the same sim.

No IK and no MoveIt on the runtime path — both the expert and the
SmolVLA inference node emit absolute joint positions directly.

## Pipeline

```
arm_description (Gazebo + arm + ros2_control)
        │
        ▼
pick_and_place_moveit.py   hand-tuned 11-step joint-keyframe expert
        │                  (name is historical; no MoveIt). Reads ball
        │                  XY via `gz model -p` before grasp and applies
        │                  a linear XY correction to canonical keyframes.
        │                  Friction grasp on a 3 cm cube.
        ▼
demo_recorder.py           subscribes to /third_person/image_raw,
        │                  /joint_states, and the two
        │                  /…_controller/joint_trajectory topics; writes
        │                  synchronized 10 Hz tuples per episode.
        ▼
scripts/convert_to_lerobot.py   → data/demos_lerobot_final/
                                  (LeRobot v2: parquet/episode + h264
                                   video/episode + meta/{info,episodes,
                                   tasks,stats}.json)
        │
        ▼
SmolVLA fine-tune          train_relaxed.py wraps lerobot 0.3.3 to patch
(~3 h on UCF nobel A100)   four dataset/timestamp incompatibilities;
                           batch 64, 30k steps, freeze_vision_encoder=true.
        │
        ▼
checkpoints/smolvla_so101/ → smolvla_inference_node.py
                            (SmolVLAPolicy.from_pretrained → 10 Hz select_action
                             → /arm_controller + /gripper_controller traj topics)
```

## Status

| Stage | State |
| --- | --- |
| Gazebo + URDF + ros2_control on bare-metal Linux + RTX 2060 (RTF ≈ 1.0) | ✅ |
| Keyframe expert + recorder + LeRobot converter | ✅ implemented |
| 200 successful demos (`data/demos_lerobot_final/`, ~15 MB, 44k frames) | ✅ collected (old IK expert, 143/57 blue/red) |
| SmolVLA 30k-step fine-tune on nobel A100 | ✅ complete (checkpoint pulled) |
| SmolVLA inference node (`src/vla/vla/smolvla_inference_node.py`) | ✅ written + registered |
| Closed-loop eval | ❌ first attempt failed — see `NEXT_STEPS.md` |

## Repository layout

```
src/
  arm_description/      SO-101 URDF, Gazebo world, ros2_control config, base launchers
  vla/                  ROS nodes: keyframe expert, recorder, SmolVLA inference
scripts/                collect_200_demos.sh, filter_successful_demos.py,
                        convert_to_lerobot.py, nuke_sim.sh, joint_sliders.py
data/demos_lerobot_final/   canonical LeRobot v2 dataset (200 episodes, ~15 MB)
checkpoints/smolvla_so101/  fine-tuned SmolVLA (gitignored; pulled from nobel)
.venv/                  Python venv with torch + lerobot 0.3.3 (gitignored)
train_relaxed.py        nobel-side training wrapper (mirror)
CLAUDE.md               full project context for Claude Code agents
CLAUDE_CHANGES.md       dated change log
NEXT_STEPS.md           active handoff (failed eval + recovery plan)
```

## Requirements

- Ubuntu 24.04 with ROS 2 **Jazzy** at `/opt/ros/jazzy`
- Apt: `ros-jazzy-{ros-gz, gz-ros2-control, joint-state-broadcaster,
  joint-trajectory-controller, robot-state-publisher, xacro, rviz2,
  cv-bridge, ros2controlcli}`, `ffmpeg`
- NVIDIA GPU recommended for real-time Gazebo rendering (RTX 2060
  validated; devcontainer falls back to llvmpipe and RTF drops hard)
- For inference: `.venv/` with `lerobot[smolvla]==0.3.3` and torch
  with CUDA. **Pin numpy < 2.0** in this venv — numpy 2.x breaks
  every ROS Jazzy Python binding.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit URDF / launch / config / world files
in place. Re-build only after editing Python nodes or `CMakeLists.txt`.

## Run

**Always start with a clean slate.** Zombie controllers and `gz sim`
orphans persist across crashes:

```bash
bash scripts/nuke_sim.sh
```

### Sim

```bash
ros2 launch arm_description arm_gazebo.launch.py
```

Brings up Gazebo (`ball_world.sdf`), the SO-101 arm spawned at
`(0, 0, 0.2)` with a `world→base_link` joint that puts `base_link` at
world z=0, `arm_controller` (5 joints), `gripper_controller` (1 joint),
`joint_state_broadcaster`, the third-person camera bridge
(`/third_person/image_raw`, 224×224 @ 5 Hz), and the
`/attach_{blue,red}` / `/detach_{blue,red}` bridges (idle — friction
grasp).

### Collect demos

```bash
# Terminal 1 — recorder
ros2 run vla demo_recorder.py --ros-args \
    -p output_dir:=/home/peyton/vla_arm/data/demos_raw \
    -p record_rate_hz:=10.0

# Terminal 2 — keyframe expert (alternates colors with --randomize)
ros2 run vla pick_and_place_moveit.py --trials 25 --randomize --seed 0

# Or batch wrapper that restarts the sim every 25 trials and stops
# at --target successes:
bash scripts/collect_200_demos.sh --target 200
```

### Convert + train

```bash
python3 scripts/filter_successful_demos.py \
    --progress-log /tmp/collect_progress.log \
    --batches-root /home/peyton/vla_arm/data/demos_raw_final \
    --output /home/peyton/vla_arm/data/demos_successful

python3 scripts/convert_to_lerobot.py \
    --input-dir /home/peyton/vla_arm/data/demos_successful \
    --output-dir /home/peyton/vla_arm/data/demos_lerobot_final --fps 10
```

Training runs on UCF nobel (A100). See **`NEXT_STEPS.md`** for the
runbook: venv + `lerobot[smolvla]==0.3.3`, dataset rsync, and the
`train_relaxed.py` wrapper invocation (~3 h on A100 at batch 64 for
30k steps).

### Closed-loop inference

The script's `#!/usr/bin/env python3` shebang resolves to
`/usr/bin/python3` (no torch). Always invoke with `.venv/bin/python`:

```bash
bash scripts/nuke_sim.sh
ros2 launch arm_description arm_gazebo.launch.py headless:=true   # bg
.venv/bin/python install/vla/lib/vla/smolvla_inference_node.py --ros-args \
    -p checkpoint_path:=/home/peyton/vla_arm/checkpoints/smolvla_so101 \
    -p task:='pick the blue ball and place it on the green marker'
```

Only two task strings are valid (everything else is OOD):
- `pick the blue ball and place it on the green marker`
- `pick the red ball and place it on the green marker`

## World

| Object | Pose | Notes |
| --- | --- | --- |
| `red_ball`, `blue_ball` | `(0.22, ∓0.08, 0.10)` | 3 cm cube visual + collision (named "ball" historically), mass 50 g, μ=2.0 |
| `place_target` | `(0.28, 0, 0.085)` | Green disc r = 5 cm, visual-only (no collision) |
| `table` | `(0.25, 0, 0.04)` | 30×40×8 cm |
| `third_person_camera` | `(0.70, 0, 0.50)`, RPY `(0, 0.81, π)` | HFOV 1.3, 224×224 @ 5 Hz, `/third_person/image_raw` |

Success criterion (used both in demo collection and inference eval):
ball XY within 5 cm of marker (Z not checked).

## Diagnostics

```bash
gz topic -e -t /stats -n 5            # real-time factor (sim must be up)
ros2 control list_controllers          # arm/gripper controllers + jsb
gz model -m blue_ball -p              # ball pose; sanity-check resets
ros2 run tf2_ros tf2_echo base_link tcp_jaw_link
xacro src/arm_description/description/so101.urdf.xacro -o /tmp/so101.urdf
```

## Where to read more

- `CLAUDE.md` — full project context: every gotcha, every key file,
  exhaustive command list. Read this if you're modifying the code.
- `NEXT_STEPS.md` — active handoff: failed closed-loop eval, the
  time-horizon-mismatch hypothesis, four-step recovery plan.
- `CLAUDE_CHANGES.md` — dated log of every change.

## License

TODO.
