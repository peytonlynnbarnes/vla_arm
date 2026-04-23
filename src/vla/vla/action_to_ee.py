#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / n, y / n, z / n, w / n]


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]


def quat_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return quat_normalize([x, y, z, w])


def rotate_vector_by_quat(v, q):
    vx, vy, vz = v
    qx, qy, qz, qw = quat_normalize(q)

    # R(q) * v
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rx = (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz
    ry = 2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz
    rz = 2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz
    return [rx, ry, rz]


class ActionToEE(Node):
    def __init__(self):
        super().__init__('action_to_ee')

        self.declare_parameter('action_topic', '/vla_action')
        self.declare_parameter('pose_topic', '/vla_target_pose')
        self.declare_parameter('gripper_topic', '/vla_gripper')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'gripper_frame_link')
        self.declare_parameter('translation_scale', 1.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('max_translation_step', 0.05)
        self.declare_parameter('max_rotation_step', 0.5)
        self.declare_parameter('publish_gripper', True)

        self.action_topic = self.get_parameter('action_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.gripper_topic = self.get_parameter('gripper_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.translation_scale = float(self.get_parameter('translation_scale').value)
        self.rotation_scale = float(self.get_parameter('rotation_scale').value)
        self.max_translation_step = float(self.get_parameter('max_translation_step').value)
        self.max_rotation_step = float(self.get_parameter('max_rotation_step').value)
        self.publish_gripper = bool(self.get_parameter('publish_gripper').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_sub = self.create_subscription(
            Float32MultiArray,
            self.action_topic,
            self.action_cb,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.gripper_pub = self.create_publisher(Float32, self.gripper_topic, 10)

        self.get_logger().info(f'Subscribed to {self.action_topic}')
        self.get_logger().info(f'Publishing target EE pose to {self.pose_topic}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info(f'EE frame: {self.ee_frame}')

    def action_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 7:
            self.get_logger().warn(f'Expected 7 action values, got {len(data)}')
            return

        dx = clamp(data[0] * self.translation_scale, -self.max_translation_step, self.max_translation_step)
        dy = clamp(data[1] * self.translation_scale, -self.max_translation_step, self.max_translation_step)
        dz = clamp(data[2] * self.translation_scale, -self.max_translation_step, self.max_translation_step)

        droll = clamp(data[3] * self.rotation_scale, -self.max_rotation_step, self.max_rotation_step)
        dpitch = clamp(data[4] * self.rotation_scale, -self.max_rotation_step, self.max_rotation_step)
        dyaw = clamp(data[5] * self.rotation_scale, -self.max_rotation_step, self.max_rotation_step)

        gripper = float(data[6])

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'Could not get TF {self.base_frame} -> {self.ee_frame}: {e}')
            return

        cur_pos = [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ]
        cur_quat = quat_normalize([
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ])

        # translation deltas are interpreted in base frame (identity pass-through)
        target_pos = [
            cur_pos[0] + dx,
            cur_pos[1] + dy,
            cur_pos[2] + dz,
        ]

        delta_quat = quat_from_euler(droll, dpitch, dyaw)
        target_quat = quat_normalize(quat_multiply(cur_quat, delta_quat))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame

        pose.pose.position.x = target_pos[0]
        pose.pose.position.y = target_pos[1]
        pose.pose.position.z = target_pos[2]

        pose.pose.orientation.x = target_quat[0]
        pose.pose.orientation.y = target_quat[1]
        pose.pose.orientation.z = target_quat[2]
        pose.pose.orientation.w = target_quat[3]

        self.pose_pub.publish(pose)

        if self.publish_gripper:
            g = Float32()
            g.data = gripper
            self.gripper_pub.publish(g)

        self.get_logger().info(
            f'Published EE target | '
            f'dpos=({dx:.4f}, {dy:.4f}, {dz:.4f}) | '
            f'drot=({droll:.4f}, {dpitch:.4f}, {dyaw:.4f}) | '
            f'target_pos=({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}) | '
            f'gripper={gripper:.4f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActionToEE()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()