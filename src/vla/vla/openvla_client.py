#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import requests
import json_numpy
import numpy as np

json_numpy.patch()


class OpenVLAEEPoseClient(Node):
    def __init__(self):
        super().__init__('openvla_ee_pose_client')

        self.bridge = CvBridge()

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('pose_topic', '/vla_output')
        self.declare_parameter('instruction', 'pick up the ball')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/act')
        self.declare_parameter('unnorm_key', 'bridge_orig')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('frame_id', 'base_link')

        camera_topic = self.get_parameter('camera_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        self.instruction = self.get_parameter('instruction').value
        self.server_url = self.get_parameter('server_url').value
        self.unnorm_key = self.get_parameter('unnorm_key').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.sub = self.create_subscription(Image, camera_topic, self.image_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self.status_pub = self.create_publisher(String, '/openvla/status', 10)

        self.busy = False

        self.get_logger().info(f'Listening on {camera_topic}')
        self.get_logger().info(f'Publishing EE pose to {pose_topic}')
        self.get_logger().info(f'Instruction: {self.instruction}')
        self.get_logger().info(f'Server: {self.server_url}')

    def image_cb(self, msg: Image):
        if self.busy:
            return
        self.busy = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = frame[:, :, ::-1]  # BGR -> RGB

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

            action = resp.json()
            action_np = np.array(action, dtype=np.float32).flatten()

            if len(action_np) < 7:
                raise ValueError("OpenVLA output must have at least 7 values (xyz + quaternion)")

            # ---- Convert to PoseStamped ----
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

            self.get_logger().info(f'EE Pose: {action_np.tolist()}')

        except Exception as e:
            self.get_logger().error(f'OpenVLA request failed: {e}')

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = OpenVLAEEPoseClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()