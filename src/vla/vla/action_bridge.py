#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ActionBridge(Node):
    def __init__(self):
        super().__init__('action_bridge')

        self.declare_parameter('input_topic', '/openvla/action')
        self.declare_parameter('trajectory_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter(
            'joint_names',
            [
                'joint1',
                'joint2',
                'joint3',
                'joint4',
                'joint5',
                'joint6',
            ]
        )
        self.declare_parameter('use_deltas', False)
        self.declare_parameter('default_move_time', 2.0)
        self.declare_parameter('current_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        input_topic = self.get_parameter('input_topic').value
        trajectory_topic = self.get_parameter('trajectory_topic').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.use_deltas = bool(self.get_parameter('use_deltas').value)
        self.default_move_time = float(self.get_parameter('default_move_time').value)
        self.current_positions = list(self.get_parameter('current_positions').value)

        self.sub = self.create_subscription(Float32MultiArray, input_topic, self.action_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)

        self.get_logger().info(f'Listening for VLA actions on {input_topic}')
        self.get_logger().info(f'Publishing trajectories to {trajectory_topic}')

    def action_cb(self, msg: Float32MultiArray):
        data = list(msg.data)

        if len(data) < len(self.joint_names):
            self.get_logger().error(
                f'Not enough action elements. Got {len(data)}, need at least {len(self.joint_names)}'
            )
            return

        action_joints = data[:len(self.joint_names)]

        if self.use_deltas:
            target_positions = [
                curr + delta for curr, delta in zip(self.current_positions, action_joints)
            ]
        else:
            target_positions = action_joints

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = Duration(
            sec=int(self.default_move_time),
            nanosec=int((self.default_move_time % 1.0) * 1e9)
        )

        traj.points.append(point)
        self.pub.publish(traj)

        self.current_positions = target_positions

        self.get_logger().info(f'Published trajectory: {target_positions}')


def main(args=None):
    rclpy.init(args=args)
    node = ActionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()