#!/usr/bin/env python3

import requests
import json_numpy
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge

json_numpy.patch()


class VLAActionClient(Node):
    def __init__(self):
        super().__init__('vla_action_client')

        self.bridge = CvBridge()
        self.busy = False

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('action_topic', '/vla_action')
        self.declare_parameter('status_topic', '/openvla/status')
        self.declare_parameter('instruction', 'approach red object')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/act')
        self.declare_parameter('unnorm_key', 'bridge_orig')
        self.declare_parameter('timeout_sec', 120.0)

        self.declare_parameter('publish_rate_hz', 0.0)   # 0.0 = publish on every received frame when idle
        self.declare_parameter('max_abs_translation', 0.05)  # meters, safety clamp
        self.declare_parameter('max_abs_rotation', 0.5)      # radians, safety clamp
        self.declare_parameter('gripper_threshold', 0.0)     # for downstream open/close logic

        self.camera_topic = self.get_parameter('camera_topic').value
        self.action_topic = self.get_parameter('action_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.instruction = self.get_parameter('instruction').value
        self.server_url = self.get_parameter('server_url').value
        self.unnorm_key = self.get_parameter('unnorm_key').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_abs_translation = float(self.get_parameter('max_abs_translation').value)
        self.max_abs_rotation = float(self.get_parameter('max_abs_rotation').value)
        self.gripper_threshold = float(self.get_parameter('gripper_threshold').value)

        self.last_publish_time = None

        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_cb, 10
        )
        self.action_pub = self.create_publisher(
            Float32MultiArray, self.action_topic, 10
        )
        self.status_pub = self.create_publisher(
            String, self.status_topic, 10
        )

        self.get_logger().info(f'Listening on {self.camera_topic}')
        self.get_logger().info(f'Publishing 7D VLA actions to {self.action_topic}')
        self.get_logger().info(f'Instruction: {self.instruction}')
        self.get_logger().info(f'Server: {self.server_url}')
        self.get_logger().info(f'unnorm_key: {self.unnorm_key}')

    def image_cb(self, msg: Image):
        if self.busy:
            return

        # Optional throttling
        if self.publish_rate_hz > 0.0:
            now = self.get_clock().now()
            if self.last_publish_time is not None:
                dt = (now - self.last_publish_time).nanoseconds / 1e9
                if dt < 1.0 / self.publish_rate_hz:
                    return

        self.busy = True

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_rgb = frame_bgr[:, :, ::-1]  # BGR -> RGB

            resp = requests.post(
                self.server_url,
                json={
                    'image': frame_rgb,
                    'instruction': self.instruction,
                    'unnorm_key': self.unnorm_key,
                },
                timeout=self.timeout_sec
            )
            resp.raise_for_status()

            payload = resp.json()

            # deploy.py may return either {"action": ...} or a raw array depending on wrapper/client path
            if isinstance(payload, dict) and 'action' in payload:
                action_np = np.array(payload['action'], dtype=np.float32).flatten()
            else:
                action_np = np.array(payload, dtype=np.float32).flatten()

            if action_np.shape[0] < 7:
                raise ValueError(
                    f'Expected at least 7 action values, got {action_np.shape[0]}'
                )

            action_np = action_np[:7].copy()

            # Safety clamps
            action_np[0:3] = np.clip(
                action_np[0:3],
                -self.max_abs_translation,
                self.max_abs_translation
            )
            action_np[3:6] = np.clip(
                action_np[3:6],
                -self.max_abs_rotation,
                self.max_abs_rotation
            )

            dx, dy, dz = action_np[0:3]
            drot_x, drot_y, drot_z = action_np[3:6]
            gripper = action_np[6]

            msg_out = Float32MultiArray()
            msg_out.data = action_np.tolist()
            self.action_pub.publish(msg_out)

            status = String()
            status.data = (
                f'Published VLA action | '
                f'dpos=({dx:.5f}, {dy:.5f}, {dz:.5f}) | '
                f'drot=({drot_x:.5f}, {drot_y:.5f}, {drot_z:.5f}) | '
                f'gripper={gripper:.5f} | '
                f'gripper_closed={(gripper > self.gripper_threshold)}'
            )
            self.status_pub.publish(status)

            self.get_logger().info(
                f'VLA action: {action_np.tolist()} | '
                f'trans_norm={np.linalg.norm(action_np[0:3]):.6f} | '
                f'rot_norm={np.linalg.norm(action_np[3:6]):.6f}'
            )

            self.last_publish_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f'OpenVLA request failed: {e}')

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = VLAActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()