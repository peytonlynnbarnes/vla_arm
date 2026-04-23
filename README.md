# vla_arm

A ROS 2 Jazzy workspace for driving a simulated **SO-101** 6-DOF arm with a
vision-language-action policy. The pipeline fine-tunes **SmolVLA** on
Gazebo-collected pick-and-place demos and runs the resulting policy
closed-loop against the sim.

## Pipeline

```
arm_moveit_config (Gazebo + arm + controllers + move_group)
        │
        ▼
pick_and_place_moveit.py        scripted expert; uses MoveIt2's
        │                       /compute_cartesian_path + /compute_ik,
        │                       toggles a DetachableJoint for grasp
        ▼
demo_recorder.py                10 Hz sync of image + joint_state +
        │                       commanded action → episode dirs
        ▼
scripts/convert_to_lerobot.py   → data/demos_lerobot_final/  (LeRobot v2)
        │
        ▼
SmolVLA finetune                lerobot/smolvla_base + this dataset
(see NEXT_STEPS.md)
        │
        ▼
smolvla_inference_node          (TODO) /third_person/image_raw +
                                /joint_states → policy.select_action →
                                /arm_controller + /gripper_controller
                                joint_trajectory topics
```

## Status

| Stage | State |
| --- | --- |
| Gazebo + URDF + ros2_control + MoveIt2 + DetachableJoint grasp | ✅ working |
| Scripted expert + recorder + LeRobot converter | ✅ implemented |
| 200 successful demos (`data/demos_lerobot_final/`, 44k frames) | ✅ collected |
| SmolVLA fine-tune pipeline | ✅ validated on UCF A100 — 3k-step smoke run on `nobel`, see `NEXT_STEPS.md` |
| SmolVLA full 30k-step fine-tune | ⏳ next up after smoke verifies |
| SmolVLA inference node | ⏳ designed, not written |

## Repository layout

```
src/
  arm_description/      SO-101 URDF, Gazebo world, ros2_control config, base launchers
  arm_moveit_config/    SRDF, kinematics/OMPL/Pilz config, combined sim+MoveIt launcher
  vla/                  ROS nodes: scripted expert, recorder, kinematics helpers
scripts/                collect_200_demos.sh, filter_successful_demos.py,
                        convert_to_lerobot.py, nuke_sim.sh
data/demos_lerobot_final/   the canonical LeRobot v2 dataset (200 episodes, ~15 MB)
NEXT_STEPS.md           active handoff — smoke run, full fine-tune, inference node, eval
CLAUDE.md               internal notes for Claude Code agents
CLAUDE_CHANGES.md       dated change log
```

## Requirements

- Ubuntu 24.04, ROS 2 **Jazzy** at `/opt/ros/jazzy`
- `ros-jazzy-{ros-gz, gz-ros2-control, joint-state-broadcaster,
  joint-trajectory-controller, robot-state-publisher, xacro, rviz2,
  cv-bridge}`
- For demo collection: `ros-jazzy-{moveit, moveit-py,
  trac-ik-kinematics-plugin, ros2controlcli}`, `ffmpeg`, `pip install pyarrow`
- NVIDIA GPU recommended for real-time Gazebo rendering

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit URDF / launch / config / world files in
place. Re-build only after editing Python nodes or `CMakeLists.txt`.

## Run

**Always start with a clean slate.** Zombie controllers and
`gz sim` orphans persist across crashes:

```bash
bash scripts/nuke_sim.sh
```

### Sim only (no MoveIt)

```bash
ros2 launch arm_description arm_gazebo.launch.py
```

Brings up Gazebo (`ball_world.sdf`), the SO-101 arm spawned at `(0, 0,
0.2)`, `arm_controller` (5 joints), `gripper_controller` (1 joint),
`joint_state_broadcaster`, the third-person camera bridge
(`/third_person/image_raw`, 224×224 @ 5 Hz), and the
`/attach_{blue,red}` / `/detach_{blue,red}` bridges.

Useful args: `render_engine:=ogre` (CPU fallback), `world:=<path>`,
`x:=`, `y:=`, `z:=` (spawn pose).

### Sim + MoveIt (for demo collection)

```bash
ros2 launch arm_moveit_config arm_gazebo_moveit.launch.py
```

### Collect demos

```bash
# Terminal 1 — recorder
ros2 run vla demo_recorder.py --ros-args \
    -p output_dir:=/workspace/data/demos_raw -p record_rate_hz:=10.0

# Terminal 2 — expert
ros2 run vla pick_and_place_moveit.py --trials 25 --randomize --seed 0

# or batch wrapper that restarts the sim every 25 trials:
bash scripts/collect_200_demos.sh --target 200
```

### Convert + (eventually) train

```bash
python3 scripts/convert_to_lerobot.py \
    --input-dir /workspace/data/demos_successful \
    --output-dir /workspace/data/demos_lerobot_final --fps 10
```

Training runs on UCF nobel (A100). See **`NEXT_STEPS.md`** for the
runbook: venv + `lerobot[smolvla]==0.3.3`, dataset rsync, and the
`train_relaxed.py` wrapper invocation (~3 h on A100 at batch 64 for
30k steps).

## World

| Object | Pose | Notes |
| --- | --- | --- |
| `red_ball`, `blue_ball` | `(0.16, ∓0.08, 0.11)` | Sphere visual r = 3 cm; **box** collision 5 cm cube (jaws can pinch reliably) |
| `place_target` | `(0.22, 0, 0.085)` | Green disc r = 5 cm, visual-only |
| `table` | `(0.25, 0, 0.04)` | 40×55×8 cm |
| `third_person_camera` | `(0.70, 0, 0.50)`, RPY `(0, 0.81, π)` | HFOV 1.3, 224×224 @ 5 Hz, `/third_person/image_raw` |

Success criterion used during collection: ball XY within 5 cm of marker
(Z not checked).

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
- `NEXT_STEPS.md` — active handoff: smoke run verification, full
  fine-tune command, checkpoint rsync, inference-node spec, eval loop.
- `CLAUDE_CHANGES.md` — dated log of every change.

## License

TODO.
