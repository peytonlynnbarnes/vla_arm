#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import requests
import json_numpy
import numpy as np

json_numpy.patch()


class OpenVLATestClient(Node):
    def __init__(self):
        super().__init__('openvla_test_client')

        self.declare_parameter('pose_topic', '/vla_output')
        self.declare_parameter('instruction', 'pick up the ball')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/act')
        self.declare_parameter('unnorm_key', 'bridge_orig')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('period_sec', 5.0)
        self.declare_parameter('image_height', 224)
        self.declare_parameter('image_width', 224)

        pose_topic = self.get_parameter('pose_topic').value
        self.instruction = self.get_parameter('instruction').value
        self.server_url = self.get_parameter('server_url').value
        self.unnorm_key = self.get_parameter('unnorm_key').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.period_sec = float(self.get_parameter('period_sec').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.image_width = int(self.get_parameter('image_width').value)

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.status_pub = self.create_publisher(String, '/openvla/status', 10)

        self.timer = self.create_timer(self.period_sec, self.timer_cb)
        self.busy = False

        self.get_logger().info('OpenVLA test client started')
        self.get_logger().info(f'Publishing EE pose to {pose_topic}')
        self.get_logger().info(f'Instruction: {self.instruction}')
        self.get_logger().info(f'Server: {self.server_url}')

    def make_test_image(self) -> np.ndarray:
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        h_mid = self.image_height // 2
        w_mid = self.image_width // 2
        img[h_mid-20:h_mid+20, w_mid-20:w_mid+20, 0] = 180
        img[h_mid-20:h_mid+20, w_mid-20:w_mid+20, 1] = 60
        img[h_mid-20:h_mid+20, w_mid-20:w_mid+20, 2] = 180
        return img

    def timer_cb(self):
        if self.busy:
            return
        self.busy = True

        try:
            frame = self.make_test_image()

            resp = requests.post(
                self.server_url,
                json={
                    'image': frame,
                    'instruction': self.instruction,
                    'unnorm_key': self.unnorm_key
                },
                timeout=self.timeout_sec
            )
            resp.raise_for_status()

            action = json_numpy.loads(resp.text)
            action_np = np.array(action, dtype=np.float32).flatten()

            self.get_logger().info(f'Raw action: {action_np.tolist()}')

            if len(action_np) < 7:
                raise ValueError(f'Expected at least 7 values, got {len(action_np)}')

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.frame_id

            pose_msg.pose.position.x = float(action_np[0])
            pose_msg.pose.position.y = float(action_np[1])
            pose_msg.pose.position.z = float(action_np[2])

            pose_msg.pose.orientation.x = float(action_np[3])
            pose_msg.pose.orientation.y = float(action_np[4])
            pose_msg.pose.orientation.z = float(action_np[5])
            pose_msg.pose.orientation.w = float(action_np[6])

            self.pose_pub.publish(pose_msg)

            status = String()
            status.data = f'Published EE pose: {action_np.tolist()}'
            self.status_pub.publish(status)

            self.get_logger().info('Published /vla_output')

        except Exception as e:
            self.get_logger().error(f'OpenVLA request failed: {e}')

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = OpenVLATestClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()