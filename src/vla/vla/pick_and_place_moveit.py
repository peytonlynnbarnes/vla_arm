#!/usr/bin/env python3
"""Scripted pick-and-place via MoveIt2 services for SO-101.

Uses /compute_ik and /compute_cartesian_path services exposed by move_group,
plus direct FollowJointTrajectory action execution. This bypasses MoveItPy
(which requires the full planning-pipeline config duplicated in this node's
parameter namespace) and talks to the already-running move_group.

Steps per episode:
  1. Home
  2. Open gripper
  3. Move above target ball
  4. Cartesian descend to ball
  5. Close gripper
  6. Cartesian ascend (lift)
  7. Move above marker
  8. Cartesian descend to marker
  9. Open gripper (release)
  10. Cartesian ascend
  11. Home

NB: the actual gripper-to-ball grasp currently fails due to gripper-mesh
geometry — see CLAUDE_CHANGES.md. The motion trajectory still executes as
intended, which is what the demo collector records.
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
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SRC = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from kinematics_so101 import (  # noqa: E402
    JOINT_NAMES_ARM, fk_pose, mat_to_quat, solve_ik,
)

GRIPPER_OPEN = 1.2
GRIPPER_CLOSED = -0.15
WORLD_NAME = 'balls_world'


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
    blue_xy = (0.16, 0.08)
    red_xy = (0.16, -0.08)
    if randomize:
        blue_xy = (0.16 + rng.uniform(-0.03, 0.03), 0.08 + rng.uniform(-0.03, 0.03))
        red_xy = (0.16 + rng.uniform(-0.03, 0.03), -0.08 + rng.uniform(-0.03, 0.03))
    # Pause, set_pose twice (velocity flush trick), unpause
    gz_world_pause(True)
    time.sleep(0.2)
    for _ in range(2):
        gz_set_pose('blue_ball', [blue_xy[0], blue_xy[1], 0.11])
        gz_set_pose('red_ball', [red_xy[0], red_xy[1], 0.11])
        time.sleep(0.1)
    gz_world_pause(False)
    time.sleep(0.5)
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
    marker_xy: Tuple[float, float] = (0.22, 0.0)
    marker_z: float = 0.085
    hover_dz: float = 0.15
    grasp_tcp_dz: float = 0.0
    lift_dz: float = 0.20
    approach_sec: float = 3.0
    cart_max_step: float = 0.01


def pose_base(x: float, y: float, z: float, rot: np.ndarray) -> PoseStamped:
    qx, qy, qz, qw = mat_to_quat(rot)
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.x = float(qx)
    ps.pose.orientation.y = float(qy)
    ps.pose.orientation.z = float(qz)
    ps.pose.orientation.w = float(qw)
    return ps


class PickPlaceMoveItNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_moveit')
        self.joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.arm_action = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip_action = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.record_pub = self.create_publisher(Bool, '/episode/record', 10)
        self.target_pub = self.create_publisher(String, '/episode/target', 10)
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

    def wait_services(self, timeout: float = 15.0) -> bool:
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/compute_ik not available')
            return False
        if not self.cart_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/compute_cartesian_path not available')
            return False
        return True

    def wait_for_joint_state(self, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state is not None:
                return True
        return False

    def compute_ik(self, pose: PoseStamped, seed_q: np.ndarray) -> Optional[np.ndarray]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped = pose
        req.ik_request.ik_link_name = 'tcp_jaw_link'
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = False
        # seed
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(JOINT_NAMES_ARM)
        rs.joint_state.position = [float(x) for x in seed_q]
        req.ik_request.robot_state = rs
        fut = self.ik_client.call_async(req)
        self._spin_until(fut, timeout=5.0)
        res = fut.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            code = res.error_code.val if res else 'no response'
            self.get_logger().warn(f'compute_ik failed: {code}')
            return None
        # Extract the arm joint values from the full solution
        m = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        try:
            return np.array([m[n] for n in JOINT_NAMES_ARM])
        except KeyError:
            self.get_logger().warn(f'compute_ik missing joints: {list(m)}')
            return None

    def compute_cartesian(self, start_q: np.ndarray, waypoints: List[PoseStamped],
                           max_step: float) -> Optional[RobotTrajectory]:
        req = GetCartesianPath.Request()
        req.group_name = 'arm'
        req.link_name = 'tcp_jaw_link'
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        # start state
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(JOINT_NAMES_ARM)
        rs.joint_state.position = [float(x) for x in start_q]
        req.start_state = rs
        req.waypoints = [ps.pose for ps in waypoints]
        req.header.frame_id = 'base_link'
        fut = self.cart_client.call_async(req)
        self._spin_until(fut, timeout=10.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn('compute_cartesian: no response')
            return None
        if res.fraction < 0.95:
            self.get_logger().warn(f'compute_cartesian: only {res.fraction:.2f} of path planned')
        if res.fraction <= 0.0:
            return None
        return res.solution

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

    def cartesian_move(self, from_pose: PoseStamped, to_pose: PoseStamped,
                        cfg: Cfg) -> bool:
        cur_q = self.current_arm_q()
        if cur_q is None:
            return False
        traj_msg = self.compute_cartesian(cur_q, [from_pose, to_pose], cfg.cart_max_step)
        if traj_msg is None or len(traj_msg.joint_trajectory.points) == 0:
            # Fallback: direct joint-goal via local IK
            self.get_logger().info('cartesian plan empty; falling back to joint-goal')
            target_pos = np.array([to_pose.pose.position.x, to_pose.pose.position.y,
                                    to_pose.pose.position.z])
            q, _, pe, _ = solve_ik(
                target_pos, np.eye(3), q_seed=cur_q, orientation_weight=0.0,
            )
            if pe > 0.02:
                self.get_logger().warn(f'fallback IK pos_err={pe:.4f}')
                return False
            return self.exec_joint_goal(q, 3.0)
        return self.exec_arm_traj(traj_msg.joint_trajectory,
                                   timeout=max(10.0, 2.0 * len(traj_msg.joint_trajectory.points) * 0.05))


def run_episode(node: PickPlaceMoveItNode, cfg: Cfg) -> bool:
    log = node.get_logger()
    cur_q = node.current_arm_q()
    if cur_q is None:
        log.error('no joint state')
        return False
    # tell recorder the target color
    t_msg = String()
    t_msg.data = cfg.target_ball
    node.target_pub.publish(t_msg)

    home_rot = fk_pose(np.zeros(5))[1]
    ball_pos = gz_model_pose(cfg.target_ball)
    if ball_pos is None:
        log.error(f'no {cfg.target_ball} pose')
        return False
    bx, by, bz = ball_pos
    mx, my = cfg.marker_xy

    # (a) home + open gripper (PRE-recording: state setup)
    if not node.exec_joint_goal(np.zeros(5), 2.5):
        log.error('home failed')
        return False
    node.send_gripper(GRIPPER_OPEN)
    time.sleep(0.4)
    # Start recording the episode
    b = Bool(); b.data = True
    node.record_pub.publish(b)
    time.sleep(0.1)

    # (b) joint goal to 'above ball' using local pos-only IK.
    above_ball_pos = np.array([bx, by, bz + cfg.hover_dz])
    q_above, _, pe, _ = solve_ik(
        above_ball_pos, np.eye(3), q_seed=np.zeros(5),
        orientation_weight=0.0,
    )
    if pe > 0.02:
        log.warn(f'above_ball local IK pos_err={pe:.4f} m')
    if not node.exec_joint_goal(q_above, 3.0):
        return False
    above_ball = pose_base(bx, by, bz + cfg.hover_dz, fk_pose(q_above)[1])
    time.sleep(0.3)

    # (c) Cartesian descent
    at_ball = pose_base(bx, by, bz + cfg.grasp_tcp_dz, home_rot)
    if not node.cartesian_move(above_ball, at_ball, cfg):
        log.warn('cart descent failed; skipping')
        # Don't return False — we want to record the attempt
    time.sleep(0.3)

    # (d) Close gripper + attach (DetachableJoint sim-level grasp)
    node.send_gripper(GRIPPER_CLOSED, dur=1.0)
    time.sleep(0.3)
    node.attach(cfg.target_ball)
    time.sleep(0.2)

    # (e) Cartesian ascent
    lift_ball = pose_base(bx, by, bz + cfg.lift_dz, home_rot)
    node.cartesian_move(at_ball, lift_ball, cfg)
    time.sleep(0.3)

    # (f) above marker via local IK. Warm-start from the current arm state
    # (post-lift) so the seed is closer to reality than the pre-grasp q_above.
    above_marker_pos = np.array([mx, my, cfg.marker_z + cfg.hover_dz])
    cur_q = node.current_arm_q()
    seed = cur_q if cur_q is not None else q_above
    q_above_m, _, pe_m, _ = solve_ik(
        above_marker_pos, np.eye(3),
        q_seed=seed, orientation_weight=0.0,
    )
    if pe_m <= 0.02:
        node.exec_joint_goal(q_above_m, 3.0)
    else:
        log.warn(f'above_marker pe={pe_m}; skipped')
    above_marker = pose_base(mx, my, cfg.marker_z + cfg.hover_dz, fk_pose(q_above_m)[1])

    # (g) Cartesian descent to marker
    at_marker = pose_base(mx, my, cfg.marker_z + 0.03, home_rot)
    node.cartesian_move(above_marker, at_marker, cfg)

    # (h) Release + detach (jaws open, DetachableJoint released)
    node.send_gripper(GRIPPER_OPEN, dur=0.8)
    time.sleep(0.3)
    node.detach(cfg.target_ball)
    time.sleep(0.3)

    # (i) Ascent + home
    lift_m = pose_base(mx, my, cfg.marker_z + cfg.lift_dz, home_rot)
    node.cartesian_move(at_marker, lift_m, cfg)
    node.exec_joint_goal(np.zeros(5), 2.5)

    # Stop recording BEFORE success check so recorder flushes
    b = Bool(); b.data = False
    node.record_pub.publish(b)
    time.sleep(0.3)

    # Success criterion
    final = gz_model_pose(cfg.target_ball)
    if final is None:
        return False
    dx = final[0] - cfg.marker_xy[0]
    dy = final[1] - cfg.marker_xy[1]
    dist = (dx * dx + dy * dy) ** 0.5
    success = dist <= 0.05
    log.info(f'final {cfg.target_ball}=({final[0]:.3f},{final[1]:.3f},{final[2]:.3f}) '
             f'dist_to_marker={dist:.3f} -> {"SUCCESS" if success else "FAIL"}')
    return success


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
        if not node.wait_services(timeout=15.0):
            return 1
        if not node.wait_action_servers(timeout=15.0):
            return 1
        successes = 0
        for i in range(args.trials):
            node.get_logger().info(f'=== episode {i + 1}/{args.trials} ===')
            blue_xy, red_xy = reset_world(None, rng, randomize=args.randomize)
            target = args.target if args.target else ('blue_ball' if i % 2 == 0 else 'red_ball')
            ball_xy = blue_xy if target == 'blue_ball' else red_xy
            cfg = Cfg(target_ball=target, marker_xy=(0.22, 0.0))
            # Inject actual ball xy into marker config? No — we read ball pose from gz in run_episode.
            if run_episode(node, cfg):
                successes += 1
        node.get_logger().info(f'RESULT: {successes}/{args.trials}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
