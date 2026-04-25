#!/usr/bin/env python3
"""SmolVLA closed-loop inference node for SO-101 in Gazebo.

Loads a fine-tuned SmolVLA checkpoint, subscribes to the third-person
camera + joint states, and publishes 6-D joint commands fanned across
the two ros2_control trajectory controllers (arm: 5 joints, gripper: 1).

Usage:
    ros2 run vla smolvla_inference_node.py --ros-args \
        -p checkpoint_path:=/home/peyton/vla_arm/checkpoints/smolvla_so101 \
        -p task:="pick the blue ball and place it on the green marker" \
        -p control_rate_hz:=10.0

The node uses the same Python venv that ROS 2 was sourced from. Install
LeRobot into it first:
    pip install "lerobot[smolvla]==0.3.3"
LeRobot 0.4.x changed the dataset format and import layout — do not upgrade.
"""

from __future__ import annotations

import sys
from pathlib import Path
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

# Joint order must match the dataset / training schema (demo_recorder.py).
JOINT_NAMES_FULL = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                    'wrist_flex', 'wrist_roll', 'gripper']
JOINT_NAMES_ARM = JOINT_NAMES_FULL[:5]
GRIPPER_NAME = JOINT_NAMES_FULL[5]


class SmolVLAInferenceNode(Node):
    def __init__(self):
        super().__init__('smolvla_inference_node')

        self.declare_parameter('checkpoint_path',
                               '/home/peyton/vla_arm/checkpoints/smolvla_so101')
        self.declare_parameter('task',
                               'pick the blue ball and place it on the green marker')
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('command_horizon_sec', 0.3)
        self.declare_parameter('image_topic', '/third_person/image_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('arm_traj_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('grip_traj_topic', '/gripper_controller/joint_trajectory')
        self.declare_parameter('device', 'cuda')

        ckpt = Path(self.get_parameter('checkpoint_path').value)
        self.task: str = self.get_parameter('task').value
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.cmd_horizon = float(self.get_parameter('command_horizon_sec').value)
        device_param = self.get_parameter('device').value
        self.device = torch.device(device_param if torch.cuda.is_available()
                                   or device_param == 'cpu' else 'cpu')

        if not ckpt.exists():
            raise FileNotFoundError(f'checkpoint dir not found: {ckpt}')
        self.get_logger().info(f'loading SmolVLA from {ckpt} on {self.device}')
        self.policy = SmolVLAPolicy.from_pretrained(str(ckpt))
        self.policy.to(self.device)
        self.policy.eval()
        # Clear any cached action chunks from a previous episode.
        if hasattr(self.policy, 'reset'):
            self.policy.reset()
        self.get_logger().info(f'task: "{self.task}"')

        self.bridge = CvBridge()
        self.lock = Lock()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_joint_state: Optional[JointState] = None

        img_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        js_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._on_image, img_qos)
        self.create_subscription(JointState,
                                 self.get_parameter('joint_states_topic').value,
                                 self._on_joint_state, js_qos)

        self.arm_pub = self.create_publisher(
            JointTrajectory, self.get_parameter('arm_traj_topic').value, 10)
        self.grip_pub = self.create_publisher(
            JointTrajectory, self.get_parameter('grip_traj_topic').value, 10)

        period = 1.0 / max(1e-3, self.rate_hz)
        self.create_timer(period, self._on_step)
        self.get_logger().info(f'inference loop @ {self.rate_hz} Hz, '
                               f'cmd horizon {self.cmd_horizon}s')

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(f'img decode failed: {exc}')
            return
        with self.lock:
            self.latest_image = img

    def _on_joint_state(self, msg: JointState):
        with self.lock:
            self.latest_joint_state = msg

    def _state_vector(self, js: JointState) -> Optional[np.ndarray]:
        name_to_pos = dict(zip(js.name, js.position))
        try:
            return np.array([name_to_pos[n] for n in JOINT_NAMES_FULL],
                            dtype=np.float32)
        except KeyError:
            return None

    def _build_batch(self, img: np.ndarray, state: np.ndarray) -> dict:
        # Image: (H, W, 3) uint8 RGB → (1, 3, H, W) float in [0, 1].
        # Policy's internal preprocessing (SigLIP + padding to 512) handles
        # resize/normalization; the dataset loader feeds raw [0,1] tensors.
        img_t = torch.from_numpy(img).to(self.device).float() / 255.0
        img_t = img_t.permute(2, 0, 1).unsqueeze(0).contiguous()
        state_t = torch.from_numpy(state).to(self.device).unsqueeze(0)
        return {
            'observation.images.third_person': img_t,
            'observation.state': state_t,
            'task': [self.task],
        }

    def _publish_action(self, action: np.ndarray):
        # action: (6,) in JOINT_NAMES_FULL order. Fan out to two controllers.
        sec = int(self.cmd_horizon)
        nsec = int((self.cmd_horizon - sec) * 1e9)

        arm = JointTrajectory()
        arm.joint_names = list(JOINT_NAMES_ARM)
        ap = JointTrajectoryPoint()
        ap.positions = [float(x) for x in action[:5]]
        ap.time_from_start.sec = sec
        ap.time_from_start.nanosec = nsec
        arm.points.append(ap)
        self.arm_pub.publish(arm)

        grip = JointTrajectory()
        grip.joint_names = [GRIPPER_NAME]
        gp = JointTrajectoryPoint()
        gp.positions = [float(action[5])]
        gp.time_from_start.sec = sec
        gp.time_from_start.nanosec = nsec
        grip.points.append(gp)
        self.grip_pub.publish(grip)

    @torch.no_grad()
    def _on_step(self):
        with self.lock:
            img = self.latest_image
            js = self.latest_joint_state
        if img is None or js is None:
            return
        state = self._state_vector(js)
        if state is None:
            return
        batch = self._build_batch(img, state)
        action_t = self.policy.select_action(batch)
        action = action_t.squeeze(0).detach().cpu().numpy()
        if action.shape[0] != 6:
            self.get_logger().error(
                f'unexpected action shape {action.shape}, expected (6,)')
            return
        self._publish_action(action)


def main(argv=None):
    rclpy.init(args=argv)
    node = SmolVLAInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
