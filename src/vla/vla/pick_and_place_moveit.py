#!/usr/bin/env python3
"""Scripted pick-and-place for SO-101 via hand-tuned joint keyframes.

No MoveIt planning, no IK: each episode is a fixed joint-goal sequence
captured with `scripts/joint_sliders.py`. MoveIt's /compute_cartesian_path
was unreliable on this 5-DOF arm (fraction<1.0 ~90% of descents) and the
local-IK fallback placed TCP 3-6 cm above the cube. The keyframe path is
simpler, faster, and ~100% reliable at the canonical ball position.

Steps per episode:
  1. Home + open gripper
  2. Pre-approach (shoulder tilted up, wrist oriented toward ball)
  3. Above ball
  4. Descend to grasp
  5. Close jaws (friction grasp — no DetachableJoint)
  6. Lift
  7. Transit above marker
  8. Descend to place
  9. Open jaws (release)
  10. Retreat straight up off the ball
  11. Home
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SRC = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from kinematics_so101 import JOINT_NAMES_ARM  # noqa: E402

GRIPPER_OPEN = 1.2
GRIPPER_CLOSED = -0.15
# Keyframe-grasp closure tuned via sliders (2026-04-24): at this opening the
# jaws actually pinch the 3 cm cube instead of over-closing past it.
GRIPPER_GRASP = 0.12
WORLD_NAME = 'balls_world'

# Hand-tuned joint poses captured via scripts/joint_sliders.py. Order:
# [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll].
BLUE_KEYFRAMES = {
    'above':        np.array([-0.52, -0.36,  0.20, 1.658, -0.48]),
    'grasp':        np.array([-0.53, -0.35,  0.21, 1.658, -0.48]),
    'lift':         np.array([-0.53, -1.31,  0.21, 1.658, -0.48]),
    'above_marker': np.array([ 0.00,  0.02, -0.42, 1.658, -0.48]),
    'place':        np.array([ 0.00,  0.21, -0.52, 1.658, -0.48]),
}
# Red ball lives at y=-0.08 (blue is y=+0.08). Mirror the approach by flipping
# shoulder_pan + wrist_roll on the ball-side poses; marker poses stay the same
# since the marker sits on the y=0 axis.
RED_KEYFRAMES = {
    'above':        np.array([ 0.52, -0.36,  0.20, 1.658,  0.56]),
    'grasp':        np.array([ 0.53, -0.35,  0.21, 1.658,  0.56]),
    'lift':         np.array([ 0.53, -1.31,  0.21, 1.658,  0.56]),
    'above_marker': np.array([ 0.00,  0.02, -0.42, 1.658,  0.56]),
    'place':        np.array([ 0.00,  0.21, -0.52, 1.658,  0.56]),
}


BALL_SDF_TEMPLATE = """<?xml version=\"1.0\" ?>
<sdf version=\"1.9\">
  <model name=\"{name}\">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name=\"link\">
      <inertial>
        <mass>0.05</mass>
        <inertia><ixx>1.8e-5</ixx><iyy>1.8e-5</iyy><izz>1.8e-5</izz></inertia>
      </inertial>
      <collision name=\"collision\">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>
      </collision>
      <visual name=\"visual\">
        <geometry><sphere><radius>0.03</radius></sphere></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def gz_remove_model(name: str) -> bool:
    try:
        out = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{WORLD_NAME}/remove',
             '--reqtype', 'gz.msgs.Entity',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', f'name: "{name}", type: MODEL'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return False
    return 'true' in out.lower()


def gz_world_reset_joints() -> bool:
    try:
        out = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{WORLD_NAME}/control',
             '--reqtype', 'gz.msgs.WorldControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'reset: {model_only: true}'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return False
    return 'true' in out.lower()


def gz_respawn_ball(name: str, xyz: Sequence[float], rgb: Tuple[float, float, float]) -> bool:
    gz_remove_model(name)
    time.sleep(0.5)
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(BALL_SDF_TEMPLATE.format(
            name=name, x=xyz[0], y=xyz[1], z=xyz[2],
            r=rgb[0], g=rgb[1], b=rgb[2],
        ))
        path = f.name
    try:
        r = subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', WORLD_NAME, '-file', path, '-name', name,
             '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2])],
            capture_output=True, text=True, timeout=10,
        )
        return 'successful' in (r.stdout + r.stderr).lower() or r.returncode == 0
    except Exception:
        return False
    finally:
        try:
            os.unlink(path)
        except Exception:
            pass


def gz_set_pose(name: str, xyz: Sequence[float]) -> bool:
    req = (
        f'name: "{name}",'
        f' position: {{ x: {xyz[0]}, y: {xyz[1]}, z: {xyz[2]} }},'
        f' orientation: {{ x: 0, y: 0, z: 0, w: 1 }}'
    )
    try:
        out = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{WORLD_NAME}/set_pose',
             '--reqtype', 'gz.msgs.Pose',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', req],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return False
    return 'true' in out.lower()


def gz_world_pause(pause: bool) -> bool:
    try:
        out = subprocess.run(
            ['gz', 'service',
             '-s', f'/world/{WORLD_NAME}/control',
             '--reqtype', 'gz.msgs.WorldControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', f'pause: {"true" if pause else "false"}'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return False
    return 'true' in out.lower()


def reset_world(cfg, rng: np.random.Generator, randomize: bool = False) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """Reset arm + balls for a new episode.

    Uses set_pose (not remove+create) so DetachableJoint stays valid. Pauses
    physics during the teleport so free-body residual velocity from the
    previous trial doesn't carry over.
    """
    gz_world_reset_joints()
    time.sleep(0.3)
    blue_xy = (0.22, 0.08)
    red_xy = (0.22, -0.08)
    if randomize:
        blue_xy = (0.22 + rng.uniform(-0.03, 0.03), 0.08 + rng.uniform(-0.03, 0.03))
        red_xy = (0.22 + rng.uniform(-0.03, 0.03), -0.08 + rng.uniform(-0.03, 0.03))
    # Pause, set_pose twice (velocity flush trick), unpause
    gz_world_pause(True)
    time.sleep(0.2)
    for _ in range(2):
        gz_set_pose('blue_ball', [blue_xy[0], blue_xy[1], 0.10])
        gz_set_pose('red_ball', [red_xy[0], red_xy[1], 0.10])
        time.sleep(0.1)
    gz_world_pause(False)
    # Cubes dropped from z=0.10 onto a table at z=0.08 need ~1 s to fully
    # settle; starting the approach too early means the ball is still
    # rolling a few mm when the jaws close.
    time.sleep(1.5)
    return blue_xy, red_xy


def gz_model_pose(name: str) -> Optional[Tuple[float, float, float]]:
    try:
        out = subprocess.run(
            ['gz', 'model', '-m', name, '-p'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return None
    m = re.search(r'XYZ.*?\n\s*\[([-+\d.eE\s]+)\]', out)
    if not m:
        return None
    parts = m.group(1).split()
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except (ValueError, IndexError):
        return None


@dataclass
class Cfg:
    target_ball: str = 'blue_ball'
    marker_xy: Tuple[float, float] = (0.28, 0.0)


class PickPlaceMoveItNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_moveit')
        self.joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.arm_action = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip_action = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.record_pub = self.create_publisher(Bool, '/episode/record', 10)
        self.target_pub = self.create_publisher(String, '/episode/target', 10)

    def _on_joint_state(self, msg: JointState):
        self.joint_state = msg

    def current_arm_q(self) -> Optional[np.ndarray]:
        if self.joint_state is None:
            return None
        m = dict(zip(self.joint_state.name, self.joint_state.position))
        try:
            return np.array([m[n] for n in JOINT_NAMES_ARM])
        except KeyError:
            return None

    def _spin_until(self, fut, timeout: float):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_action_servers(self, timeout: float = 15.0) -> bool:
        return (self.arm_action.wait_for_server(timeout_sec=timeout)
                and self.grip_action.wait_for_server(timeout_sec=timeout))

    def wait_for_joint_state(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None:
                return True
        return False

    def exec_arm_traj(self, traj: JointTrajectory, timeout: float = 30.0) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm_action.send_goal_async(goal)
        self._spin_until(fut, timeout=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('arm goal rejected')
            return False
        rfut = gh.get_result_async()
        self._spin_until(rfut, timeout=timeout)
        r = rfut.result()
        if r is None:
            return False
        if r.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'arm traj status={r.status} code={r.result.error_code} msg={r.result.error_string}')
            return False
        return True

    def exec_joint_goal(self, q: np.ndarray, dur: float) -> bool:
        tj = JointTrajectory()
        tj.joint_names = list(JOINT_NAMES_ARM)
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur - int(dur)) * 1e9)
        tj.points.append(pt)
        return self.exec_arm_traj(tj)

    def send_gripper(self, pos: float, dur: float = 0.8) -> bool:
        tj = JointTrajectory()
        tj.joint_names = ['gripper']
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur - int(dur)) * 1e9)
        tj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = tj
        fut = self.grip_action.send_goal_async(goal)
        self._spin_until(fut, timeout=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        rfut = gh.get_result_async()
        self._spin_until(rfut, timeout=5.0)
        return rfut.result() is not None and rfut.result().status == GoalStatus.STATUS_SUCCEEDED


def run_episode_keyframe(node: PickPlaceMoveItNode, cfg: Cfg,
                          kf: dict) -> bool:
    """Keyframe-mode episode: joint-goal sequence captured from sliders.

    No IK, no /compute_cartesian_path, no DetachableJoint. The cube is held
    by friction between the jaws — `GRIPPER_GRASP` is tuned to close on a
    3 cm cube without over-closing past it.
    """
    log = node.get_logger()
    # Tell recorder which color this episode is about, then flip record on.
    t_msg = String(); t_msg.data = cfg.target_ball
    node.target_pub.publish(t_msg)

    if not node.exec_joint_goal(np.zeros(5), 2.0):
        log.error('home failed'); return False
    node.send_gripper(GRIPPER_OPEN)
    time.sleep(0.4)

    rec_on = Bool(); rec_on.data = True
    node.record_pub.publish(rec_on)
    time.sleep(0.1)

    # Pre-approach: use the `lift` pose as an intermediate so the shoulder
    # tilts up and the pan/wrist orient toward the ball BEFORE the hand is
    # lowered. Going home->above directly sweeps the hand through the ball
    # column and knocks the cube.
    log.info('moving to pre-approach (lift pose, shoulder up first)')
    if not node.exec_joint_goal(kf['lift'], 3.0):
        log.error('pre-approach failed'); return False
    time.sleep(0.3)

    log.info('moving to above')
    if not node.exec_joint_goal(kf['above'], 2.5):
        log.error('above failed'); return False
    time.sleep(0.3)

    # Log actual ball XY right before the descent so we can diagnose the
    # "grasped air" cases: if the ball drifted more than ~1 cm from its
    # spawn pose, the keyframe `grasp` pose will miss.
    pre = gz_model_pose(cfg.target_ball)
    if pre is not None:
        log.info(f'{cfg.target_ball} XY before grasp: ({pre[0]:.3f}, {pre[1]:.3f})')

    log.info('moving to grasp (slow descent)')
    if not node.exec_joint_goal(kf['grasp'], 3.5):
        log.error('grasp pose failed'); return False
    time.sleep(0.4)

    log.info(f'closing gripper to {GRIPPER_GRASP}')
    node.send_gripper(GRIPPER_GRASP, dur=1.0)
    time.sleep(0.5)

    log.info('moving to lift')
    if not node.exec_joint_goal(kf['lift'], 2.0):
        log.error('lift failed'); return False
    time.sleep(0.3)

    # Transit: route through the `lift` posture (arm high) rather than
    # swinging down-through-space to above_marker. Same reasoning as
    # pre-approach: keeps the hand out of the workspace while the shoulder
    # swings to the marker side.
    log.info('moving to above_marker')
    if not node.exec_joint_goal(kf['above_marker'], 3.0):
        log.error('above_marker failed'); return False
    time.sleep(0.3)

    log.info('moving to place')
    if not node.exec_joint_goal(kf['place'], 2.5):
        log.error('place failed'); return False
    time.sleep(0.3)

    log.info('releasing gripper')
    node.send_gripper(GRIPPER_OPEN, dur=0.6)
    time.sleep(0.5)

    # Retreat: lift straight up off the ball before swinging home. Without
    # this the wrist/jaws rake across the marker and knock the cube out of
    # the 5 cm success radius as the shoulder rotates back through home.
    log.info('retreating to above_marker')
    if not node.exec_joint_goal(kf['above_marker'], 2.0):
        log.error('retreat failed'); return False
    time.sleep(0.3)

    rec_off = Bool(); rec_off.data = False
    node.record_pub.publish(rec_off)

    node.exec_joint_goal(np.zeros(5), 2.0)
    time.sleep(0.4)

    final = gz_model_pose(cfg.target_ball)
    if final is None:
        log.error('no ball pose post-place'); return False
    dx = final[0] - cfg.marker_xy[0]
    dy = final[1] - cfg.marker_xy[1]
    err = float(np.hypot(dx, dy))
    ok = err < 0.05
    log.info(f'final {cfg.target_ball}=({final[0]:.3f},{final[1]:.3f},{final[2]:.3f}) '
             f'dist_to_marker={err:.3f} -> {"SUCCESS" if ok else "FAIL"}')
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', default=None, choices=['blue_ball', 'red_ball'],
                         help='Fixed target. If not set, alternates between blue and red.')
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--randomize', action='store_true',
                         help='Jitter ball XY positions ±3cm each episode.')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args(argv)

    rng = np.random.default_rng(args.seed)

    rclpy.init()
    node = PickPlaceMoveItNode()
    try:
        if not node.wait_for_joint_state(timeout=15.0):
            node.get_logger().error('no joint state')
            return 1
        if not node.wait_action_servers(timeout=15.0):
            return 1
        successes = 0
        for i in range(args.trials):
            node.get_logger().info(f'=== episode {i + 1}/{args.trials} ===')
            blue_xy, red_xy = reset_world(None, rng, randomize=args.randomize)
            target = args.target if args.target else ('blue_ball' if i % 2 == 0 else 'red_ball')
            cfg = Cfg(target_ball=target, marker_xy=(0.28, 0.0))
            if target == 'blue_ball':
                ok = run_episode_keyframe(node, cfg, BLUE_KEYFRAMES)
            elif target == 'red_ball':
                ok = run_episode_keyframe(node, cfg, RED_KEYFRAMES)
            else:
                node.get_logger().error(f'no keyframes for {target}'); ok = False
            if ok:
                successes += 1
        node.get_logger().info(f'RESULT: {successes}/{args.trials}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
