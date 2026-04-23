#!/usr/bin/env python3
"""Demo recorder for pick-and-place episodes.

Subscribes to the simulation topics and writes synchronized (image, joint
state, last-commanded-action) tuples per episode. Each episode ends with a
human-readable manifest + numpy array + parquet-ready per-frame records.

Output layout (one directory per episode):
  {output_dir}/episode_{NNN}/
    frames.npz             # dict of stacked arrays
    meta.json              # target, success, success_dist, n_frames, timestamps
    img_{FFF}.jpg          # sampled frames (one per timestep) for visual checks

Episode lifecycle:
  - /episode/control service-ish: use a simple ROS2 std_srvs/Trigger-style
    start/stop. For now, the recorder is started externally and stopped by
    the expert policy via a /episode/stop topic. Start is implicit: the first
    frame after node init begins episode 0.

Data captured per sampled frame (at record_rate Hz, default 10):
  observation.images.third_person : (H, W, 3) uint8 JPEG decoded
  observation.state                : (6,) float64 — joint positions in CHAIN order (shoulder_pan..gripper)
  action                           : (6,) float64 — last commanded joint positions (mirrors observation.state when idle)
  timestamp                        : float64 (sim time, seconds from start of episode)

Mirrors LeRobot v2 record schema closely so the converter step (Task 5) is a
straight pass-through.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory


JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


@dataclass
class EpisodeBuffer:
    images: List[np.ndarray] = field(default_factory=list)
    states: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.declare_parameter('output_dir', '/workspace/data/demos_raw')
        self.declare_parameter('record_rate_hz', 10.0)
        self.declare_parameter('image_topic', '/third_person/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('arm_traj_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('grip_traj_topic', '/gripper_controller/joint_trajectory')
        self.declare_parameter('target_color', 'blue_ball')
        self.declare_parameter('save_images', True)

        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.record_rate_hz = float(self.get_parameter('record_rate_hz').value)
        self.target_color = self.get_parameter('target_color').value
        self.save_images = bool(self.get_parameter('save_images').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.lock = Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_joint_state: Optional[JointState] = None
        self.last_arm_cmd: Optional[np.ndarray] = None
        self.last_grip_cmd: Optional[float] = None
        self.recording = False
        self.episode_index = self._next_episode_index()
        self.episode_start: Optional[float] = None
        self.buf = EpisodeBuffer()

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, qos)
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value, self._on_joint_state,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            JointTrajectory, self.get_parameter('arm_traj_topic').value, self._on_arm_traj,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            JointTrajectory, self.get_parameter('grip_traj_topic').value, self._on_grip_traj,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Control channel: simple Bool topic. True = start episode, False = stop and flush.
        self.create_subscription(Bool, '/episode/record', self._on_record_control, 10)
        self.create_subscription(String, '/episode/target', self._on_target, 10)

        self.record_period = 1.0 / max(1e-3, self.record_rate_hz)
        self.create_timer(self.record_period, self._on_timer, clock=self.get_clock())
        self.get_logger().info(f'demo_recorder ready. output_dir={self.output_dir} rate={self.record_rate_hz} Hz')

    def _next_episode_index(self) -> int:
        existing = [p.name for p in self.output_dir.glob('episode_*') if p.is_dir()]
        idx = 0
        for e in existing:
            try:
                idx = max(idx, int(e.split('_', 1)[1]) + 1)
            except (ValueError, IndexError):
                pass
        return idx

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self.lock:
                self.latest_image = img
        except Exception as exc:
            self.get_logger().warn(f'img decode failed: {exc}')

    def _on_joint_state(self, msg: JointState):
        with self.lock:
            self.latest_joint_state = msg

    def _on_arm_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        last = msg.points[-1]
        # Match joint order: msg.joint_names → positions
        name_to_pos = dict(zip(msg.joint_names, last.positions))
        cmd = []
        for n in JOINT_NAMES[:5]:
            cmd.append(name_to_pos.get(n, float('nan')))
        with self.lock:
            self.last_arm_cmd = np.array(cmd, dtype=np.float64)

    def _on_grip_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        last = msg.points[-1]
        name_to_pos = dict(zip(msg.joint_names, last.positions))
        g = name_to_pos.get('gripper', float('nan'))
        with self.lock:
            self.last_grip_cmd = float(g)

    def _on_target(self, msg: String):
        self.target_color = msg.data

    def _on_record_control(self, msg: Bool):
        if msg.data and not self.recording:
            self._start_episode()
        elif not msg.data and self.recording:
            self._finish_episode(success_from_meta=None)

    def _start_episode(self):
        self.buf = EpisodeBuffer()
        self.episode_start = self.get_clock().now().nanoseconds / 1e9
        self.recording = True
        self.get_logger().info(f'Episode {self.episode_index} recording STARTED')

    def _finish_episode(self, success_from_meta: Optional[bool]):
        self.recording = False
        if not self.buf.images:
            self.get_logger().warn('episode had 0 frames; skip')
            return
        ep_dir = self.output_dir / f'episode_{self.episode_index:04d}'
        ep_dir.mkdir(parents=True, exist_ok=True)
        # Stack arrays
        images = np.stack(self.buf.images)   # (N, H, W, 3) uint8
        states = np.stack(self.buf.states)   # (N, 6)
        actions = np.stack(self.buf.actions)
        timestamps = np.array(self.buf.timestamps)
        # Save npz
        np.savez_compressed(
            ep_dir / 'frames.npz',
            images=images, states=states, actions=actions, timestamps=timestamps,
        )
        # Save meta
        meta = {
            'episode_index': self.episode_index,
            'target': self.target_color,
            'n_frames': int(images.shape[0]),
            'success': success_from_meta,
            'duration_sec': float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0,
            'image_shape': list(images.shape[1:]),
            'joint_names': JOINT_NAMES,
        }
        (ep_dir / 'meta.json').write_text(json.dumps(meta, indent=2))
        # Save sample images
        if self.save_images:
            for i, img in enumerate(self.buf.images):
                if i % max(1, len(self.buf.images) // 20) == 0:
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(str(ep_dir / f'img_{i:04d}.jpg'), bgr)
        self.get_logger().info(f'Episode {self.episode_index} saved: {images.shape[0]} frames -> {ep_dir}')
        self.episode_index += 1

    def _on_timer(self):
        if not self.recording:
            return
        with self.lock:
            img = self.latest_image
            js = self.latest_joint_state
            cmd_arm = self.last_arm_cmd
            cmd_grip = self.last_grip_cmd
        if img is None or js is None:
            return
        # Build (6,) state vector in JOINT_NAMES order from JointState
        name_to_pos = dict(zip(js.name, js.position))
        try:
            state = np.array([name_to_pos[n] for n in JOINT_NAMES], dtype=np.float64)
        except KeyError:
            return
        # Action = last commanded values; fall back to state for any joint with no command
        action = state.copy()
        if cmd_arm is not None:
            action[:5] = cmd_arm
        if cmd_grip is not None:
            action[5] = cmd_grip
        ts = self.get_clock().now().nanoseconds / 1e9 - (self.episode_start or 0.0)
        self.buf.images.append(img.copy())
        self.buf.states.append(state)
        self.buf.actions.append(action)
        self.buf.timestamps.append(ts)


def main(argv=None):
    rclpy.init(args=argv)
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recording:
            node._finish_episode(success_from_meta=None)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
