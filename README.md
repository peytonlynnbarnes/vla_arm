# vla_arm

A ROS 2 Jazzy workspace that drives a simulated **SO-101** 6-DOF arm with
[OpenVLA](https://openvla.github.io). A wrist-mounted camera streams frames
to a remote OpenVLA inference server; the returned 7D action (EE-frame
deltas + gripper) is integrated onto the current end-effector pose, solved
to joint positions, and sent to the arm in Gazebo.

```
 Gazebo (SO-101 + wrist camera, ball_world)
        │  /camera/image_raw
        ▼
  vla_action_client.py  ── HTTP /act ──►  OpenVLA server  (remote, SSH-tunneled)
        │                                 returns [dx dy dz drx dry drz gripper]
        │  /vla_action (Float32MultiArray, len 7)
        ▼
  action_to_ee.py   TF base_link ← gripper_frame_link; scale + clamp deltas;
        │           integrate onto current EE pose
        │  /vla_target_pose (PoseStamped)   + /vla_gripper (Float32)
        ▼
  vla_ik.py         analytic 3-DOF IK (pan, lift, elbow); wrist/gripper held at params
        │
        │  /arm_controller/joint_trajectory
        ▼
  gz_ros2_control  → joints move in Gazebo
```

## Requirements

- Ubuntu 24.04 (Noble) or compatible, with an NVIDIA GPU recommended for
  real-time Gazebo rendering.
- **ROS 2 Jazzy** at `/opt/ros/jazzy` including:
  - `ros-jazzy-ros-gz` (Gazebo Harmonic integration)
  - `ros-jazzy-gz-ros2-control`
  - `ros-jazzy-joint-state-broadcaster`
  - `ros-jazzy-joint-trajectory-controller`
  - `ros-jazzy-robot-state-publisher`, `ros-jazzy-xacro`, `ros-jazzy-rviz2`
  - `ros-jazzy-cv-bridge`
- Python: `requests`, `numpy`, `json-numpy` (install into the workspace
  venv or system Python used by ROS).
- Access to an OpenVLA inference server (see **OpenVLA server** below).

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means URDF/xacro, launch, config, and world files are
edited in place — no rebuild needed after tweaking them. Re-run
`colcon build` after editing Python nodes or `CMakeLists.txt`.

## Running the simulation

```bash
ros2 launch arm_description arm_gazebo.launch.py
```

This brings up:

- Gazebo Sim (`ogre2` render engine by default) with `ball_world.sdf`
  (three colored balls on a ground plane).
- The SO-101 arm spawned at the origin via `ros_gz_sim create`.
- `robot_state_publisher` for TF.
- `parameter_bridge` nodes for `/clock` and `/camera/image_raw` between
  gz transport and ROS 2.
- `joint_state_broadcaster` and `arm_controller`
  (`JointTrajectoryController`) on the 6 joints: `shoulder_pan`,
  `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.

Useful launch arguments:

- `render_engine:=ogre` — fall back to the CPU renderer (slower, use only
  if `ogre2` has driver issues).
- `world:=<path>` — swap the SDF world.
- `x:=`, `y:=`, `z:=` — spawn pose.

To visualize the arm without physics:

```bash
ros2 launch arm_description arm_rviz2.launch.py
```

This opens RViz with the URDF and a `joint_state_publisher_gui` for
manual joint sliders.

Measure real-time factor while the sim is running:

```bash
gz topic -e -t /stats -n 5
```

## Running the VLA pipeline

Make sure the OpenVLA server is reachable (see below) and the sim is
running. The intended one-shot launcher is:

```bash
ros2 launch vla vla.launch.py
```

…which starts three nodes in series:

1. `vla_action_client.py` — subscribes to `/camera/image_raw`, POSTs each
   frame to OpenVLA with an instruction string (default:
   `"pick up the red ball"`), clamps the returned translation/rotation
   to safety limits, and publishes a 7D `Float32MultiArray` on
   `/vla_action` plus a human-readable status on `/openvla/status`.
2. `action_to_ee.py` — looks up the current end-effector pose via TF
   (`base_link` → `gripper_frame_link`), integrates the 7D delta onto
   it, and publishes the absolute target as `PoseStamped` on
   `/vla_target_pose` (plus a `Float32` gripper value on
   `/vla_gripper`).
3. `vla_ik.py` — analytic 3-DOF IK on `shoulder_pan`, `shoulder_lift`,
   `elbow_flex`. Wrist and gripper joints are driven from parameters.
   Output goes to `/arm_controller/joint_trajectory`. A deadband
   (`position_threshold`, `joint_threshold`) suppresses publishes when
   the target hasn't meaningfully changed.

The translation mapping inside `action_to_ee.py` is still experimental
(see the "better debug mapping guess" in the source), and the
`delta_world` TF-rotated delta it computes is currently unused — the
frame convention for translation is in flux and worth auditing before
trusting closed-loop behaviour. You can exercise the IK node on its
own with a hand-published `/vla_target_pose`:

```bash
ros2 run vla vla_ik.py
```

## OpenVLA server

The client POSTs JSON-encoded images to `http://127.0.0.1:8000/act`
(see `server_url` inside `vla.launch.py`). The repo assumes the real
server runs elsewhere and is reached over SSH port-forwarding. The
canonical command is in `cmds.txt`:

```bash
ssh -L 8000:localhost:8000 pe606840@nobel.ece.ucf.edu
# on the remote host:
python vla-scripts/deploy.py \
  --openvla_path openvla/openvla-7b \
  --host 0.0.0.0 \
  --port 8000
```

Payloads use the `json_numpy` protocol (`pip install json-numpy`) — the
client calls `json_numpy.patch()` so `requests.post(json=...)` handles
numpy arrays transparently. Requests time out after
`timeout_sec` (default 120 s); if the tunnel is down every request will
block for that long.

## Repository layout

```
vla_arm/
├── src/
│   ├── arm_description/            SO-101 URDF, Gazebo world, ros2_control
│   │   ├── description/            so101.urdf.xacro, camera.xacro
│   │   ├── worlds/ball_world.sdf   Three balls on a ground plane
│   │   ├── config/controllers.yaml joint_state_broadcaster + arm_controller
│   │   ├── launch/                 arm_gazebo, arm_rviz2, rsp
│   │   └── assets/                 STL/DAE meshes
│   └── vla/                        VLA client nodes
│       ├── vla/vla_action_client.py  camera → HTTP → /vla_action (Float32MultiArray)
│       ├── vla/action_to_ee.py       /vla_action + TF → /vla_target_pose (PoseStamped)
│       ├── vla/vla_ik.py             /vla_target_pose → /arm_controller/joint_trajectory
│       └── launch/vla.launch.py      launches the three nodes above with inline params
├── CLAUDE.md                       Internal notes for Claude Code agents
├── CLAUDE_CHANGES.md               Session-by-session change log
├── cmds.txt                        Handy one-liners (SSH tunnel, server start)
└── .claude/                        Claude Code settings + hooks
```

## Key parameters

All three VLA nodes are parameterized from inline dicts in
`src/vla/launch/vla.launch.py`; the interesting knobs:

`vla_action_client.py`
- `instruction` — the natural-language task handed to OpenVLA (default
  `"pick up the red ball"`).
- `server_url`, `timeout_sec`, `unnorm_key` — server config.
- `publish_rate_hz` — throttle; `0.0` publishes on every received
  frame when idle.
- `max_abs_translation`, `max_abs_rotation` — per-axis safety clamps
  on the returned action (metres / radians).

`action_to_ee.py`
- `base_frame`, `ee_frame` — TF frames used to look up the current EE
  pose (defaults `base_link` / `gripper_frame_link`).
- `translation_scale`, `rotation_scale` — gain on the incoming delta.
- `max_translation_step`, `max_rotation_step` — additional per-message
  clamps.
- `publish_gripper` — toggle `/vla_gripper` output.

`vla_ik.py`
- `upper_arm_length`, `lower_arm_length`, `shoulder_offset_{x,z}` —
  link lengths for the analytic IK.
- `wrist_flex`, `wrist_roll`, `gripper` — constant values for the
  three joints this IK doesn't solve for.
- `move_time_sec` — `time_from_start` on every published trajectory
  point.
- `position_threshold`, `joint_threshold` — deadband before a new
  trajectory is published.

## License

TODO.
