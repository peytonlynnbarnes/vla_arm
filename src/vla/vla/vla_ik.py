#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class TargetPoseToJointTrajectoryIK(Node):
    def __init__(self):
        super().__init__('target_pose_to_joint_trajectory_ik')

        self.declare_parameter('input_topic', '/vla_target_pose')
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

        self.declare_parameter('wrist_flex', 0.5)
        self.declare_parameter('wrist_roll', 0.0)
        self.declare_parameter('gripper', 0.0)

        self.declare_parameter('move_time_sec', 2.0)
        self.declare_parameter('position_threshold', 0.01)
        self.declare_parameter('joint_threshold', 0.02)

        self.input_topic = self.get_parameter('input_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value

        self.shoulder_offset_x = float(self.get_parameter('shoulder_offset_x').value)
        self.shoulder_offset_z = float(self.get_parameter('shoulder_offset_z').value)
        self.upper_arm_length = float(self.get_parameter('upper_arm_length').value)
        self.lower_arm_length = float(self.get_parameter('lower_arm_length').value)

        self.default_wrist_flex = float(self.get_parameter('wrist_flex').value)
        self.default_wrist_roll = float(self.get_parameter('wrist_roll').value)
        self.default_gripper = float(self.get_parameter('gripper').value)

        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.position_threshold = float(self.get_parameter('position_threshold').value)
        self.joint_threshold = float(self.get_parameter('joint_threshold').value)

        self.last_target_xyz = None
        self.last_joint_positions = None

        self.sub = self.create_subscription(PoseStamped, self.input_topic, self.pose_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing trajectories to {self.trajectory_topic}')

    def solve_ik(self, x: float, y: float, z: float):
        shoulder_pan = math.atan2(y, x)

        r_world = math.sqrt(x * x + y * y)
        r = r_world - self.shoulder_offset_x
        z_rel = z - self.shoulder_offset_z
        r = max(r, 1e-6)

        d_raw = math.sqrt(r * r + z_rel * z_rel)

        max_reach = self.upper_arm_length + self.lower_arm_length - 1e-6
        min_reach = abs(self.upper_arm_length - self.lower_arm_length) + 1e-6
        d = clamp(d_raw, min_reach, max_reach)

        if abs(d - d_raw) > 1e-6:
            self.get_logger().warn(
                f'Target out of reach, clamped reach distance from {d_raw:.4f} to {d:.4f}'
            )

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

    def target_changed_enough(self, xyz):
        if self.last_target_xyz is None:
            return True
        return any(abs(a - b) > self.position_threshold for a, b in zip(xyz, self.last_target_xyz))

    def joints_changed_enough(self, joints):
        if self.last_joint_positions is None:
            return True
        return any(abs(a - b) > self.joint_threshold for a, b in zip(joints, self.last_joint_positions))

    def pose_cb(self, msg: PoseStamped):
        try:
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = float(msg.pose.position.z)

            xyz = [x, y, z]

            if not self.target_changed_enough(xyz):
                return

            shoulder_pan, shoulder_lift, elbow_flex = self.solve_ik(x, y, z)

            positions = [
                shoulder_pan,
                shoulder_lift,
                elbow_flex,
                self.default_wrist_flex,
                self.default_wrist_roll,
                self.default_gripper,
            ]

            if not self.joints_changed_enough(positions):
                self.last_target_xyz = xyz
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(self.move_time_sec)
            point.time_from_start.nanosec = int((self.move_time_sec - int(self.move_time_sec)) * 1e9)

            traj.points.append(point)
            self.pub.publish(traj)

            self.last_target_xyz = xyz
            self.last_joint_positions = positions[:]

            self.get_logger().info(
                f'Published trajectory from target pose '
                f'xyz=({x:.4f}, {y:.4f}, {z:.4f}) -> '
                f'joints={[round(v, 4) for v in positions]}'
            )

        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseToJointTrajectoryIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()