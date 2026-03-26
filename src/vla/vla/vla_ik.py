#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class VLAtoJointTrajectoryIK(Node):
    def __init__(self):
        super().__init__('vla_to_joint_trajectory_ik')

        self.declare_parameter('input_topic', '/vla_output')
        self.declare_parameter('trajectory_topic', '/arm_controller/joint_trajectory')

        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper',
        ]

        self.declare_parameter('shoulder_offset_x', 0.0388)
        self.declare_parameter('shoulder_offset_z', 0.0624)
        self.declare_parameter('upper_arm_length', 0.120)
        self.declare_parameter('lower_arm_length', 0.135)

        self.declare_parameter('center_x', 0.18)
        self.declare_parameter('center_y', 0.00)
        self.declare_parameter('center_z', 0.16)
        self.declare_parameter('scale_x', 4.0)
        self.declare_parameter('scale_y', 4.0)
        self.declare_parameter('scale_z', 4.0)

        self.declare_parameter('wrist_flex', 0.5)
        self.declare_parameter('wrist_roll', 0.0)
        self.declare_parameter('gripper', 0.0)
        self.declare_parameter('move_time_sec', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value

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

        self.default_wrist_flex = float(self.get_parameter('wrist_flex').value)
        self.default_wrist_roll = float(self.get_parameter('wrist_roll').value)
        self.default_gripper = float(self.get_parameter('gripper').value)
        self.move_time_sec = float(self.get_parameter('move_time_sec').value)

        self.sub = self.create_subscription(PoseStamped, self.input_topic, self.pose_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing trajectories to {self.trajectory_topic}')

    def remap_target(self, msg: PoseStamped):
        vx = msg.pose.position.x
        vy = msg.pose.position.y
        vz = msg.pose.position.z

        x = self.center_x + vx * self.scale_x
        y = self.center_y + vy * self.scale_y
        z = self.center_z + vz * self.scale_z
        return x, y, z

    def solve_ik(self, x: float, y: float, z: float):
        shoulder_pan = math.atan2(y, x)

        r_world = math.sqrt(x * x + y * y)
        r = r_world - self.shoulder_offset_x
        z_rel = z - self.shoulder_offset_z
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
                self.default_wrist_flex,
                self.default_wrist_roll,
                self.default_gripper,
            ]

            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(self.move_time_sec)
            point.time_from_start.nanosec = int((self.move_time_sec - int(self.move_time_sec)) * 1e9)

            traj.points.append(point)
            self.pub.publish(traj)

            self.get_logger().info(f'Published trajectory: {positions}')

        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VLAtoJointTrajectoryIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()