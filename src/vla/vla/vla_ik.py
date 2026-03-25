#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class VLAtoJointStateIK(Node):
    def __init__(self):
        super().__init__('vla_to_jointstate_ik')

        self.declare_parameter('input_topic', '/vla_output')
        self.declare_parameter('joint_state_topic', '/joint_states')

        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper',
        ]

        # Approximate geometry for a first RViz-only IK pass
        self.declare_parameter('shoulder_offset_x', 0.0388)
        self.declare_parameter('shoulder_offset_z', 0.0624)
        self.declare_parameter('upper_arm_length', 0.120)
        self.declare_parameter('lower_arm_length', 0.135)

        # Remap tiny OpenVLA outputs into reachable absolute targets
        self.declare_parameter('center_x', 0.18)
        self.declare_parameter('center_y', 0.00)
        self.declare_parameter('center_z', 0.16)
        self.declare_parameter('scale_x', 4.0)
        self.declare_parameter('scale_y', 4.0)
        self.declare_parameter('scale_z', 4.0)

        self.declare_parameter('publish_rate', 20.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value

        self.shoulder_offset_x = float(self.get_parameter('shoulder_offset_x').value)
        self.shoulder_offset_z = float(self.get_parameter('shoulder_offset_z').value)
        self.upper_arm_length = float(self.get_parameter('upper_arm_length').value)
        self.lower_arm_length = float(self.get_parameter('lower_arm_length').value)

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.center_z = float(self.get_parameter('center_z').value)
        self.scale_x = float(self.get_parameter('scale_x').value)
        self.scale_y = float(self.get_parameter('scale_y').value)
        self.scale_z = float(self.get_parameter('scale_z').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_cb,
            10
        )
        self.pub = self.create_publisher(JointState, self.joint_state_topic, 10)

        self.current_positions = [0.0] * len(self.joint_names)
        self.has_target = False

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing joint states to {self.joint_state_topic}')
        self.get_logger().info(f'Joint names: {self.joint_names}')

    def remap_target(self, msg: PoseStamped):
        vx = msg.pose.position.x
        vy = msg.pose.position.y
        vz = msg.pose.position.z

        x = self.center_x + vx * self.scale_x
        y = self.center_y + vy * self.scale_y
        z = self.center_z + vz * self.scale_z

        return x, y, z

    def solve_ik(self, x: float, y: float, z: float):
        # Base yaw
        shoulder_pan = math.atan2(y, x)

        # Convert target into the shoulder pitch plane
        r_world = math.sqrt(x * x + y * y)
        r = r_world - self.shoulder_offset_x
        z_rel = z - self.shoulder_offset_z

        # Avoid degenerate cases
        r = max(r, 1e-5)

        d = math.sqrt(r * r + z_rel * z_rel)

        max_reach = self.upper_arm_length + self.lower_arm_length - 1e-6
        min_reach = abs(self.upper_arm_length - self.lower_arm_length) + 1e-6
        d = clamp(d, min_reach, max_reach)

        cos_elbow = (
            d * d
            - self.upper_arm_length * self.upper_arm_length
            - self.lower_arm_length * self.lower_arm_length
        ) / (2.0 * self.upper_arm_length * self.lower_arm_length)
        cos_elbow = clamp(cos_elbow, -1.0, 1.0)

        elbow_flex = math.acos(cos_elbow)

        k1 = self.upper_arm_length + self.lower_arm_length * math.cos(elbow_flex)
        k2 = self.lower_arm_length * math.sin(elbow_flex)

        shoulder_lift = math.atan2(z_rel, r) - math.atan2(k2, k1)

        return shoulder_pan, shoulder_lift, elbow_flex

    def pose_cb(self, msg: PoseStamped):
        try:
            x, y, z = self.remap_target(msg)
            shoulder_pan, shoulder_lift, elbow_flex = self.solve_ik(x, y, z)

            positions = [
                shoulder_pan,
                shoulder_lift,
                elbow_flex,
                0.0,   # wrist_flex
                0.0,   # wrist_roll
                0.2,   # gripper slightly open
            ]

            self.current_positions = positions
            self.has_target = True

            self.get_logger().info(
                f'target=({x:.3f}, {y:.3f}, {z:.3f}) '
                f'-> pan={shoulder_pan:.3f}, lift={shoulder_lift:.3f}, elbow={elbow_flex:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')

    def publish_joint_states(self):
        if not self.has_target:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLAtoJointStateIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()