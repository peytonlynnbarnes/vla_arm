# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace layout

ROS 2 **Jazzy** colcon workspace with two packages under `src/`:

- `arm_description` — SO-101 6-DOF arm URDF (xacro), Gazebo Sim (gz) world, `ros2_control` config, and launch files for sim + RViz.
- `vla` — Python ROS 2 nodes that close the loop with an external OpenVLA inference server: subscribe to the wrist camera, POST frames to the VLA server, turn the returned end-effector pose into a `JointTrajectory`.

Both use `ament_cmake` (not `ament_python`); `vla` installs its Python modules via `ament_python_install_package` and individual scripts via `install(PROGRAMS ...)` — editing `vla/vla/*.py` only survives `colcon build` if it's listed there.

## Common commands

All commands assume `cd /home/peyton-peyton/vla_arm` and that ROS 2 Jazzy is installed at `/opt/ros/jazzy`.

```bash
# Build (symlink-install lets you edit launch/config/urdf files without rebuilding)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
# ...or just one package:
colcon build --packages-select arm_description --symlink-install

# Source the overlay (re-source after every build)
source install/setup.bash

# Launch the Gazebo sim (spawns arm in ball_world, loads gz_ros2_control, spawns controllers)
ros2 launch arm_description arm_gazebo.launch.py
# optional: render_engine:=ogre  (default ogre2 uses the GPU)

# Arm + RViz with joint sliders (no physics)
ros2 launch arm_description arm_rviz2.launch.py

# VLA client pipeline (requires OpenVLA server reachable at server_url)
ros2 launch vla vla.launch.py

# Quick manual render of the xacro for debugging
xacro src/arm_description/description/so101.urdf.xacro -o /tmp/so101.urdf

# Read sim RTF (must be run while sim is up)
gz topic -e -t /stats -n 5
```

There are no tests, linter configs, or CI in this repo.

## OpenVLA server dependency

The VLA nodes talk to an HTTP inference server, **not** a local model. The server is hosted remotely at UCF and exposed via SSH tunnel (see `cmds.txt`):

```bash
ssh -L 8000:localhost:8000 pe606840@nobel.ece.ucf.edu
# on the remote host, run vla-scripts/deploy.py ...
```

`server_url` defaults to `http://127.0.0.1:8000/act`. If the tunnel is down, every request in `vla_action_client.py` times out after `timeout_sec` (120s default). The wire format is `json_numpy` (`pip install json-numpy`) — the client calls `json_numpy.patch()` so `requests.post(json=...)` serializes numpy arrays transparently. The server is expected to return a 7-element action vector: `[dx, dy, dz, drx, dry, drz, gripper]` (EE-frame deltas + gripper), not an absolute pose.

## Data flow (big picture)

The pipeline is now three nodes in series. The VLA server returns EE-frame *deltas*, `action_to_ee.py` integrates those onto the current TF pose to produce an absolute target pose, and `vla_ik.py` solves joint positions for it.

```
Gazebo (gz topic /camera/image_raw)
    │ ros_gz_bridge (parameter_bridge)
    ▼
ROS /camera/image_raw ──► vla_action_client.py ──HTTP──► OpenVLA server
                                 │                        returns [dx,dy,dz, drx,dry,drz, gripper]
                                 │ /vla_action (Float32MultiArray, len 7)
                                 ▼
                          action_to_ee.py
                            │ looks up TF base_link ← gripper_frame_link
                            │ scales+clamps deltas, integrates onto current pose
                            │ publishes absolute target + gripper scalar
                            │ /vla_target_pose (PoseStamped)
                            │ /vla_gripper (Float32)
                            ▼
                          vla_ik.py  (class TargetPoseToJointTrajectoryIK)
                            │ analytic 3-DOF IK (pan, lift, elbow) — wrist/gripper held at params
                            │ deadband: skips publish if target & joints haven't moved past thresholds
                            ▼
                    /arm_controller/joint_trajectory (JointTrajectory)
                            │
                            ▼
               gz_ros2_control → joints in Gazebo
```

## Gotchas / inconsistencies

Things that will bite if you assume consistency — update them if they're wrong when you read this.

- **Translation deltas in `action_to_ee.py` are now interpreted in `base_link` frame** (identity pass-through from `/vla_action[0:3]` to `target_pos += (dx,dy,dz)`). The earlier "better debug mapping guess" (`dx=0.2*raw_dx; dy=-1.0*raw_dx; dz=1.0*raw_dy`, dropping `raw_dz` entirely) and the unused `delta_world = rotate_vector_by_quat(...)` path were both removed. If you want to switch back to EE-local-frame deltas (standard OpenVLA convention), re-introduce the quaternion rotation and feed `delta_world` into `target_pos`.
- **Green ball is out of reach from the default spawn.** `arm_gazebo.launch.py` spawns the arm at world `z=0.2`; shoulder pivot ends up at world `(0.0388, 0, 0.2624)`. Green ball is at world `(0.45, 0, 0.03)`, ~0.47 m away. Arm reach is `upper + lower = 0.255 m`, so `vla_ik.py`'s reach clamp will kick in and the arm will just fully extend toward the ball. Either lower the spawn (`z:=0.05`) or raise the balls onto a table to make grabs physically plausible.
- **Topic name change.** `vla_ik.py`'s default input topic is `/vla_target_pose` (was `/vla_output`). The old `center_{x,y,z}` / `scale_{x,y,z}` workspace-remap parameters were removed — it now consumes absolute base-frame poses directly. Added deadband params: `position_threshold` (0.01 m), `joint_threshold` (0.02 rad).
- **Camera resolution & rate were lowered** in `camera.xacro` from 1280×720 @ 30 Hz to 320×240 @ 5 Hz to keep RTF up with the wrist camera on. If you re-raise either, expect RTF to drop hard under `ogre`.
- **`ros-jazzy-joint-trajectory-controller` must be installed** apt-side or `arm_controller` fails to load (the YAML type `joint_trajectory_controller/JointTrajectoryController` comes from that package, not from `ros2_control` metapackage).
- **`render_engine` default is `ogre2`** (GPU). Fall back to `ogre` only if you hit EGL/driver issues — `ogre` is CPU-rendered and tanks RTF when the wrist camera is on.

## Key files to read before editing

- `src/arm_description/description/so101.urdf.xacro` — robot links/joints + the `<ros2_control>` block + the `gz_ros2_control` plugin declaration (points at `config/controllers.yaml`).
- `src/arm_description/description/camera.xacro` — wrist camera macro; `<visualize>` is off and resolution is 320×240 @ 5 Hz to keep RTF up.
- `src/arm_description/worlds/ball_world.sdf` — physics `<max_step_size>` (4ms) and the three colored balls the VLA is prompted to grab.
- `src/arm_description/config/controllers.yaml` — declares `joint_state_broadcaster` and `arm_controller` for `controller_manager`.
- `src/arm_description/launch/arm_gazebo.launch.py` — orchestrates xacro → urdf → rsp → gazebo → ros_gz bridges → spawn → controller spawners (with a 6s timer before controllers so `gz_ros2_control` has time to come up).
- `src/vla/vla/vla_action_client.py` — camera subscriber; posts frames to OpenVLA, publishes 7D `Float32MultiArray` on `/vla_action` plus a human-readable status string.
- `src/vla/vla/action_to_ee.py` — consumes `/vla_action`, looks up current EE pose via TF (`base_link` → `gripper_frame_link`), integrates deltas, publishes `/vla_target_pose` + `/vla_gripper`.
- `src/vla/vla/vla_ik.py` — analytic 3-DOF IK; subscribes to `/vla_target_pose`, publishes `/arm_controller/joint_trajectory`. The wrist/gripper joints are driven from parameters, not the incoming pose orientation.
- `src/vla/launch/vla.launch.py` — launches the three nodes above with inline parameter dicts.
- `src/vla/CMakeLists.txt` — lists which scripts get installed; editing any `vla/vla/*.py` only survives `colcon build` if it's in `install(PROGRAMS ...)`.

## Claude-specific conventions in this repo

- `CLAUDE_CHANGES.md` — human-readable log of changes Claude has applied, newest entries at the top. Append a dated entry whenever you finish a task that modifies files.
- `.claude/hooks/remind_changelog.sh` is registered as a `Stop` hook in `.claude/settings.json`. If `git status` is dirty and `CLAUDE_CHANGES.md` isn't among the dirty files, the hook blocks the stop and feeds a reminder back. Keep the hook silent by editing `CLAUDE_CHANGES.md` as part of any change-making session.
