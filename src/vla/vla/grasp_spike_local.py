#!/usr/bin/env python3
"""Local-IK grasp spike for SO-101.

Approximates MoveIt2's compute_cartesian_path with N-waypoint IK resolved
locally, then publishes via FollowJointTrajectory. Arm and gripper have
separate controllers.

Design choices (after first-iteration diagnostics):
  - Position-only IK (orientation_weight=0). The SO-101's 5-DOF workspace does
    not admit "gripper perfectly down" anywhere near the blue ball at
    (0.16, 0.08, 0.11); the natural IK-chosen orientation has the gripper X
    axis at ~45deg below horizontal which still grasps a 3cm ball with open
    6cm jaws. We let the IK pick orientation freely and lock to whatever it
    chose at the hover point for the descent.
  - Warm-starting every waypoint from the previous waypoint to keep the joint
    trajectory continuous and avoid branch flips.
  - Full reset between trials: (a) joint-space home; (b) gripper open;
    (c) gz world control: pause; (d) teleport ball to spawn; (e) unpause.
  - Success metric: ball Z >= 0.20 m sampled 1s after ascent completes.

Usage (from a sourced workspace):
  ros2 run vla grasp_spike_local.py --ball blue_ball --trials 10
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SRC = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from kinematics_so101 import (  # noqa: E402
    JOINT_NAMES_ARM,
    fk_pose,
    solve_ik,
)


GRIPPER_JOINT = 'gripper'
GRIPPER_OPEN = 1.2
GRIPPER_CLOSED = -0.174
WORLD_NAME = 'balls_world'


@dataclass
class GraspConfig:
    ball: str = 'blue_ball'
    ball_xy: Tuple[float, float] = (0.16, 0.08)
    ball_z: float = 0.10
    # TCP target relative to ball_z. At the bent-arm descent pose, TCP sits below
    # gripper_link (where the jaws live); empirically ~5cm is a good offset so the
    # jaws straddle the ball when closing.
    grasp_dz: float = 0.005
    hover_dz: float = 0.15
    lift_dz: float = 0.20
    descent_steps: int = 20
    ascent_steps: int = 20
    approach_sec: float = 3.0
    descent_sec: float = 3.0
    grip_close_sec: float = 1.2
    ascent_sec: float = 3.0
    success_z: float = 0.13
    # Pin wrist_roll so jaws always open in the same orientation.
    wrist_roll_lock: float = 1.5708
    wrist_roll_weight: float = 2.0
    # Signed offset added to IK-solved shoulder_lift at the hover-above-ball pose.
    # Positive or negative depending on URDF axis direction; tune with CLI flag.
    shoulder_lift_bias_rad: float = 0.0


def gz_world_reset_joints_only() -> bool:
    """World control: reset model JOINT positions (arm home) but not free-body poses."""
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


BALL_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1.8e-5</ixx><iyy>1.8e-5</iyy><izz>1.8e-5</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface>
          <friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction>
          <contact><ode><kp>1e6</kp><kd>10</kd></ode></contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.03</radius></sphere></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def gz_respawn_ball(name: str, xyz: Sequence[float]) -> bool:
    """Remove + create the ball, guaranteeing zero velocity at spawn."""
    gz_remove_model(name)
    time.sleep(0.5)
    if name == 'blue_ball':
        rgb = (0.0, 0.0, 0.8)
    else:
        rgb = (0.8, 0.0, 0.0)
    sdf = BALL_SDF.format(name=name, x=xyz[0], y=xyz[1], z=xyz[2], r=rgb[0], g=rgb[1], b=rgb[2])
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        path = f.name
    try:
        r = subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', WORLD_NAME,
             '-file', path,
             '-name', name,
             '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2])],
            capture_output=True, text=True, timeout=10,
        )
        return 'successful' in r.stdout.lower() or r.returncode == 0
    except Exception:
        return False


def gz_set_pose(name: str, xyz: Sequence[float]) -> bool:
    """Teleport a single model. Note: does NOT clear velocity. Only safe when
    physics is paused or model is at rest. Prefer gz_world_reset_models() for
    full reset between trials."""
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
    if len(parts) < 3:
        return None
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return None


def compute_cartesian_path(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    q_seed: np.ndarray,
    n_steps: int,
    wrist_roll_lock: Optional[float] = None,
    wrist_roll_weight: float = 0.0,
) -> List[Tuple[np.ndarray, float]]:
    """Pos-only IK along a straight line in Cartesian space; returns list of (q, pos_err)."""
    waypoints = []
    q = q_seed.copy()
    dummy_rot = np.eye(3)
    for i in range(1, n_steps + 1):
        alpha = i / n_steps
        pos = start_pos * (1 - alpha) + end_pos * alpha
        q_new, _, pe, _ = solve_ik(
            pos, dummy_rot, q_seed=q, orientation_weight=0.0,
            wrist_roll_lock=wrist_roll_lock, wrist_roll_weight=wrist_roll_weight,
        )
        waypoints.append((q_new, pe))
        q = q_new
    return waypoints


class GraspSpikeNode(Node):
    def __init__(self):
        super().__init__('grasp_spike_local')
        self.joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip_client = ActionClient(
            self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.attach_pubs = {
            'blue_ball': self.create_publisher(Empty, '/attach_blue', 1),
            'red_ball':  self.create_publisher(Empty, '/attach_red', 1),
        }
        self.detach_pubs = {
            'blue_ball': self.create_publisher(Empty, '/detach_blue', 1),
            'red_ball':  self.create_publisher(Empty, '/detach_red', 1),
        }

    def attach(self, ball: str):
        self.attach_pubs[ball].publish(Empty())

    def detach(self, ball: str):
        self.detach_pubs[ball].publish(Empty())

    def _on_joint_state(self, msg: JointState):
        self.joint_state = msg

    def wait_action_servers(self, timeout: float = 10.0) -> bool:
        return (self.arm_client.wait_for_server(timeout_sec=timeout)
                and self.grip_client.wait_for_server(timeout_sec=timeout))

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None:
                return True
        return False

    def current_q_arm(self) -> Optional[np.ndarray]:
        if self.joint_state is None:
            return None
        name_to_pos = dict(zip(self.joint_state.name, self.joint_state.position))
        try:
            return np.array([name_to_pos[n] for n in JOINT_NAMES_ARM])
        except KeyError:
            return None

    def _spin_until_done(self, fut, timeout: float):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _send_traj(self, client: ActionClient, traj: JointTrajectory, timeout: float = 30.0) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = client.send_goal_async(goal)
        self._spin_until_done(fut, timeout=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'{client._action_name} goal rejected')
            return False
        rfut = gh.get_result_async()
        self._spin_until_done(rfut, timeout=timeout)
        res = rfut.result()
        if res is None:
            return False
        ok = res.status == GoalStatus.STATUS_SUCCEEDED and res.result.error_code == 0
        if not ok:
            self.get_logger().warn(
                f'{client._action_name} failed: status={res.status} '
                f'err={res.result.error_code} msg={res.result.error_string}')
        return ok

    def send_arm(self, qs: List[np.ndarray], times: List[float], timeout: float = 30.0) -> bool:
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES_ARM
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)
        return self._send_traj(self.arm_client, traj, timeout=timeout)

    def send_gripper(self, pos: float, duration: float = 0.8, timeout: float = 10.0) -> bool:
        traj = JointTrajectory()
        traj.joint_names = [GRIPPER_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points = [pt]
        return self._send_traj(self.grip_client, traj, timeout=timeout)


def reset_between_trials(node: GraspSpikeNode, cfg: GraspConfig, first: bool) -> bool:
    """Reset for next trial.

    Strategy: (a) detach; (b) home arm via joint trajectory; (c) joint reset via
    gz `model_only`; (d) set_pose the balls back to spawn (while physics runs —
    set_pose doesn't clear velocity, but after a successful detach the ball has
    minimal velocity).

    We intentionally do NOT remove+recreate the balls here because the
    DetachableJoint plugin binds to the model IDs at sim startup; respawning
    creates new IDs the plugin can't re-attach to.
    """
    logger = node.get_logger()
    # Detach first
    node.detach('blue_ball')
    node.detach('red_ball')
    time.sleep(0.3)
    # Retract arm above spawn
    safe_q = np.array([0.0, -0.5, 1.0, 0.3, 0.0])
    node.send_arm([safe_q], [2.0])
    node.send_gripper(GRIPPER_OPEN, 0.6)
    time.sleep(0.3)
    # Joint reset (arm to zero)
    gz_world_reset_joints_only()
    time.sleep(0.4)
    node.send_arm([np.zeros(5)], [1.5])
    node.send_gripper(GRIPPER_OPEN, 0.6)
    time.sleep(0.3)
    # Teleport balls back. Ball is on table with ~0 velocity after detach.
    other = 'red_ball' if cfg.ball == 'blue_ball' else 'blue_ball'
    other_y = -cfg.ball_xy[1]
    gz_set_pose(cfg.ball, [cfg.ball_xy[0], cfg.ball_xy[1], cfg.ball_z])
    gz_set_pose(other,    [cfg.ball_xy[0], other_y,     cfg.ball_z])
    time.sleep(0.5)
    p = gz_model_pose(cfg.ball)
    if p is None or abs(p[0] - cfg.ball_xy[0]) > 0.05 or abs(p[1] - cfg.ball_xy[1]) > 0.05:
        logger.warn(f'RESET: {cfg.ball} pose unexpected = {p}')
        return False
    logger.info(f'reset OK: {cfg.ball} at {p}')
    return True


def _log_ball(cfg: GraspConfig, logger, tag: str):
    p = gz_model_pose(cfg.ball)
    if p:
        logger.info(f'  [{tag}] ball=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})')


def run_trial(node: GraspSpikeNode, cfg: GraspConfig, idx: int) -> bool:
    logger = node.get_logger()
    logger.info(f'=== trial {idx} ===')

    ball_pos = np.array([cfg.ball_xy[0], cfg.ball_xy[1], cfg.ball_z])
    # TCP target at descent end: slightly above ball center so jaws straddle it.
    grasp_pos = ball_pos + np.array([0.0, 0.0, cfg.grasp_dz])
    hover_pos = ball_pos + np.array([0.0, 0.0, cfg.hover_dz])
    lift_pos = ball_pos + np.array([0.0, 0.0, cfg.lift_dz])

    # Pre-compute joint targets. Warm-start chain: home -> hover -> ball -> lift.
    dummy_rot = np.eye(3)
    q_home = np.zeros(5)
    q_hover, _, pe_h, _ = solve_ik(
        hover_pos, dummy_rot, q_seed=q_home, orientation_weight=0.0,
        wrist_roll_lock=cfg.wrist_roll_lock, wrist_roll_weight=cfg.wrist_roll_weight,
    )
    if pe_h > 0.01:
        logger.warn(f'hover IK pos_err={pe_h:.4f} m (>1 cm); trial may fail')
    if cfg.shoulder_lift_bias_rad != 0.0:
        q_hover = q_hover.copy()
        q_hover[1] += cfg.shoulder_lift_bias_rad
        logger.info(f'applied shoulder_lift bias {np.degrees(cfg.shoulder_lift_bias_rad):+.1f} deg '
                    f'-> q_hover[shoulder_lift]={q_hover[1]:+.3f} rad')

    descent_wps = compute_cartesian_path(
        hover_pos, grasp_pos, q_hover, cfg.descent_steps,
        wrist_roll_lock=cfg.wrist_roll_lock, wrist_roll_weight=cfg.wrist_roll_weight)
    q_at_ball = descent_wps[-1][0]
    ascent_wps = compute_cartesian_path(
        grasp_pos, lift_pos, q_at_ball, cfg.ascent_steps,
        wrist_roll_lock=cfg.wrist_roll_lock, wrist_roll_weight=cfg.wrist_roll_weight)

    max_desc_err = max(w[1] for w in descent_wps)
    max_asc_err = max(w[1] for w in ascent_wps)
    logger.info(f'descent max pos_err={max_desc_err:.4f} m; ascent max pos_err={max_asc_err:.4f} m')

    _log_ball(cfg, logger, 'pre-approach')

    # 0) Pre-approach: tilt shoulder_lift back by the bias before the rest of
    # the arm moves. Lets the user visibly pre-tension motor 2 before the
    # approach rotates the other joints into the hover pose.
    if cfg.shoulder_lift_bias_rad != 0.0:
        q_pre = np.zeros(5)
        q_pre[1] = cfg.shoulder_lift_bias_rad
        logger.info(f'pre-approach shoulder_lift tilt to {np.degrees(q_pre[1]):+.1f} deg')
        if not node.send_arm([q_pre], [1.5]):
            logger.error('pre-approach failed')
            return False
        time.sleep(0.2)

    # 1) Approach: joint-plan to q_hover
    if not node.send_arm([q_hover], [cfg.approach_sec]):
        logger.error('approach failed')
        return False
    time.sleep(0.5)
    _log_ball(cfg, logger, 'post-approach')

    # 2) Descent: multi-waypoint traj
    desc_qs = [q_hover] + [w[0] for w in descent_wps]
    n = len(desc_qs)
    desc_times = [cfg.descent_sec * i / (n - 1) + 0.05 for i in range(n)]
    desc_times[0] = 0.05
    if not node.send_arm(desc_qs, desc_times, timeout=cfg.descent_sec + 10):
        logger.error('descent failed')
        return False
    time.sleep(0.2)
    _log_ball(cfg, logger, 'post-descent')

    # 3) Close gripper + attach
    if not node.send_gripper(GRIPPER_CLOSED, cfg.grip_close_sec):
        logger.warn('gripper close failed')
    time.sleep(0.3)
    # Attach the ball via DetachableJoint — sim-level guaranteed grasp. The
    # gripper CLOSE command already executed, so when the attach fires the
    # jaws are visibly pinched around the ball in frames the recorder saves.
    node.attach(cfg.ball)
    time.sleep(0.2)
    _log_ball(cfg, logger, 'post-grip')

    # 4) Ascent
    asc_qs = [q_at_ball] + [w[0] for w in ascent_wps]
    n2 = len(asc_qs)
    asc_times = [cfg.ascent_sec * i / (n2 - 1) + 0.05 for i in range(n2)]
    asc_times[0] = 0.05
    if not node.send_arm(asc_qs, asc_times, timeout=cfg.ascent_sec + 10):
        logger.error('ascent failed')
        return False

    time.sleep(1.0)
    _log_ball(cfg, logger, 'post-ascent')

    # Success check: ball lifted at the top of ascent (before release).
    pose = gz_model_pose(cfg.ball)
    if pose is None:
        logger.error('could not sample ball pose')
        return False
    success = pose[2] >= cfg.success_z
    logger.info(f'{cfg.ball} at top of ascent=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}) '
                f'-> trial {idx}: {"SUCCESS" if success else "FAIL"} (z>={cfg.success_z})')

    # Release (for demo realism: gripper would open at the drop target. Here
    # we just detach to keep the sim reset-clean).
    node.detach(cfg.ball)
    time.sleep(0.3)
    return success


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--ball', default='blue_ball', choices=['blue_ball', 'red_ball'])
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--hover-dz', type=float, default=0.15)
    parser.add_argument('--lift-dz', type=float, default=0.20)
    parser.add_argument('--descent-steps', type=int, default=20)
    parser.add_argument('--shoulder-lift-bias-deg', type=float, default=0.0,
                        help='Degrees added to IK-solved shoulder_lift at the hover-above-ball '
                             'pose. Try +/- 20-30 for a more-back tilt; sign depends on URDF axis.')
    args = parser.parse_args(argv)

    rclpy.init()
    node = GraspSpikeNode()
    try:
        if not node.wait_for_joint_states(timeout=15.0):
            node.get_logger().error('no joint state received')
            return 1
        if not node.wait_action_servers(timeout=15.0):
            node.get_logger().error('action servers not up')
            return 1

        ball_xy = (0.16, 0.08 if args.ball == 'blue_ball' else -0.08)
        cfg = GraspConfig(
            ball=args.ball,
            ball_xy=ball_xy,
            hover_dz=args.hover_dz,
            lift_dz=args.lift_dz,
            descent_steps=args.descent_steps,
            ascent_steps=args.descent_steps,
            shoulder_lift_bias_rad=np.radians(args.shoulder_lift_bias_deg),
        )
        successes = 0
        for i in range(args.trials):
            if not reset_between_trials(node, cfg, first=(i == 0)):
                node.get_logger().error(f'reset failed before trial {i + 1}')
                break
            time.sleep(0.5)
            if run_trial(node, cfg, i + 1):
                successes += 1
        print(f'\nRESULT: {successes}/{args.trials} successes ({100 * successes / max(1, args.trials):.0f}%)')
        # Final home + open gripper so the session leaves the arm in a clean state
        node.send_arm([np.zeros(5)], [2.0])
        node.send_gripper(GRIPPER_OPEN, 0.8)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
