#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.optimize import least_squares
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Kinematic chain base_link -> gripper_frame_link (from so101.urdf.xacro).
# Each entry: (origin_xyz, origin_rpy, axis_or_None). Revolute joints have
# axis (0,0,1); the final gripper_frame_joint is fixed (axis=None).
# Solved joints in order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.
CHAIN = [
    ((0.0388353, -8.97657e-09, 0.0624),     (3.14159, 0.0, -3.14159),     (0, 0, 1)),
    ((-0.0303992, -0.0182778, -0.0542),     (-1.5708, -1.5708, 0.0),      (0, 0, 1)),
    ((-0.11257, -0.028, 0.0),               (0.0, 0.0, 1.5708),           (0, 0, 1)),
    ((-0.1349, 0.0052, 0.0),                (0.0, 0.0, -1.5708),          (0, 0, 1)),
    ((0.0, -0.0611, 0.0181),                (1.5708, 0.0486795, 3.14159), (0, 0, 1)),
    ((-0.0079, -0.000218121, -0.0981274),   (0.0, 3.14159, 0.0),          None),
]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _rpy_mat(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _axis_angle_mat(axis, theta):
    ax = np.array(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    c, s = math.cos(theta), math.sin(theta)
    v = 1 - c
    x, y, z = ax
    return np.array([
        [c + x * x * v,     x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v,     y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ])


def fk_pose(q5):
    """Full FK: returns (position (3,), rotation matrix (3,3)) of gripper_frame_link in base_link."""
    T = np.eye(4)
    idx = 0
    for xyz, rpy, axis in CHAIN:
        T_j = np.eye(4)
        T_j[:3, :3] = _rpy_mat(rpy)
        T_j[:3, 3] = xyz
        if axis is not None:
            T_r = np.eye(4)
            T_r[:3, :3] = _axis_angle_mat(axis, q5[idx])
            idx += 1
            T_j = T_j @ T_r
        T = T @ T_j
    return T[:3, 3], T[:3, :3]


def quat_to_mat(q):
    """ROS/Gazebo quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ])


def rot_log(R):
    """Rotation matrix -> axis-angle vector (theta * axis). Handles theta near 0 and pi."""
    cos_theta = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-6:
        return np.zeros(3)
    if math.pi - theta < 1e-3:
        # near pi: extract axis from the symmetric part
        diag = np.array([R[0, 0], R[1, 1], R[2, 2]])
        i = int(np.argmax(diag))
        j = (i + 1) % 3
        k = (i + 2) % 3
        denom = math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k]))
        axis = np.zeros(3)
        axis[i] = 0.5 * denom
        axis[j] = (R[j, i] + R[i, j]) / (2.0 * denom)
        axis[k] = (R[k, i] + R[i, k]) / (2.0 * denom)
        return axis * theta
    s = 2.0 * math.sin(theta)
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / s
    return axis * theta


class TargetPoseToJointTrajectoryIK(Node):
    def __init__(self):
        super().__init__('target_pose_to_joint_trajectory_ik')

        self.declare_parameter('input_topic', '/vla_target_pose')
        self.declare_parameter('gripper_topic', '/vla_gripper')
        self.declare_parameter('trajectory_topic', '/arm_controller/joint_trajectory')

        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper',
        ]

        # URDF joint limits for the 5 solved arm joints.
        self.declare_parameter('shoulder_pan_min', -1.91986)
        self.declare_parameter('shoulder_pan_max', 1.91986)
        self.declare_parameter('shoulder_lift_min', -1.74533)
        self.declare_parameter('shoulder_lift_max', 1.74533)
        self.declare_parameter('elbow_flex_min', -1.69)
        self.declare_parameter('elbow_flex_max', 1.69)
        self.declare_parameter('wrist_flex_min', -1.65806)
        self.declare_parameter('wrist_flex_max', 1.65806)
        self.declare_parameter('wrist_roll_min', -2.74385)
        self.declare_parameter('wrist_roll_max', 2.84121)

        # Initial-guess seed for wrist joints (used on first solve).
        self.declare_parameter('wrist_flex_seed', 0.5)
        self.declare_parameter('wrist_roll_seed', 0.0)

        # Gripper open/closed joint angles. OpenVLA bridge_orig returns
        # gripper in ~[0,1]; we binarize at `gripper_threshold` and map to
        # these joint positions.
        self.declare_parameter('gripper_open_position', 1.745)
        self.declare_parameter('gripper_closed_position', -0.174)
        self.declare_parameter('gripper_threshold', 0.5)
        self.declare_parameter('gripper_change_threshold', 0.02)

        # Relative weight of orientation error vs translation error in the
        # IK least-squares residual. Residual is [pos_err(m) * 1, rot_err(rad) * w].
        # w=0.0 disables orientation entirely (position-only IK).
        self.declare_parameter('orientation_weight', 0.5)

        self.declare_parameter('move_time_sec', 2.0)
        self.declare_parameter('position_threshold', 0.01)
        self.declare_parameter('joint_threshold', 0.02)

        # IK solver config.
        self.declare_parameter('ik_unreachable_threshold', 0.02)
        self.declare_parameter('ik_max_nfev', 80)

        self.input_topic = self.get_parameter('input_topic').value
        self.gripper_topic = self.get_parameter('gripper_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value

        self.q_lower = np.array([
            float(self.get_parameter('shoulder_pan_min').value),
            float(self.get_parameter('shoulder_lift_min').value),
            float(self.get_parameter('elbow_flex_min').value),
            float(self.get_parameter('wrist_flex_min').value),
            float(self.get_parameter('wrist_roll_min').value),
        ])
        self.q_upper = np.array([
            float(self.get_parameter('shoulder_pan_max').value),
            float(self.get_parameter('shoulder_lift_max').value),
            float(self.get_parameter('elbow_flex_max').value),
            float(self.get_parameter('wrist_flex_max').value),
            float(self.get_parameter('wrist_roll_max').value),
        ])

        wrist_flex_seed = float(self.get_parameter('wrist_flex_seed').value)
        wrist_roll_seed = float(self.get_parameter('wrist_roll_seed').value)

        self.gripper_open_position = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed_position = float(self.get_parameter('gripper_closed_position').value)
        self.gripper_threshold = float(self.get_parameter('gripper_threshold').value)
        self.gripper_change_threshold = float(self.get_parameter('gripper_change_threshold').value)

        self.orientation_weight = float(self.get_parameter('orientation_weight').value)

        self.move_time_sec = float(self.get_parameter('move_time_sec').value)
        self.position_threshold = float(self.get_parameter('position_threshold').value)
        self.joint_threshold = float(self.get_parameter('joint_threshold').value)

        self.ik_unreachable_threshold = float(self.get_parameter('ik_unreachable_threshold').value)
        self.ik_max_nfev = int(self.get_parameter('ik_max_nfev').value)

        self.last_target_xyz = None
        self.last_target_quat = None
        self.last_joint_positions = None
        # Warm start in joint order [pan, lift, elbow, wrist_flex, wrist_roll].
        self.last_ik_solution = np.array([0.0, 0.0, 0.0, wrist_flex_seed, wrist_roll_seed])

        # Start closed; update on each gripper message.
        self.current_gripper_target = self.gripper_closed_position
        self.last_published_gripper = self.gripper_closed_position

        self.sub = self.create_subscription(PoseStamped, self.input_topic, self.pose_cb, 10)
        self.gripper_sub = self.create_subscription(Float32, self.gripper_topic, self.gripper_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)

        self.get_logger().info(f'Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing trajectories to {self.trajectory_topic}')
        self.get_logger().info(
            f'Full 5-DOF IK; orientation_weight={self.orientation_weight}, '
            f'gripper threshold={self.gripper_threshold} '
            f'(open={self.gripper_open_position}, closed={self.gripper_closed_position})'
        )

    def solve_ik(self, target_pos, target_rot):
        target_pos = np.asarray(target_pos, dtype=float)
        target_rot = np.asarray(target_rot, dtype=float)
        w = self.orientation_weight

        def residual(q5):
            pos, rot = fk_pose(q5)
            pos_err = pos - target_pos
            if w <= 0.0:
                return pos_err
            R_err = target_rot.T @ rot
            rot_err = rot_log(R_err) * w
            return np.concatenate([pos_err, rot_err])

        x0 = np.clip(self.last_ik_solution, self.q_lower, self.q_upper)

        result = least_squares(
            residual, x0,
            bounds=(self.q_lower, self.q_upper),
            method='trf',
            max_nfev=self.ik_max_nfev,
        )

        res = result.fun
        pos_residual = float(np.linalg.norm(res[:3]))
        rot_residual = float(np.linalg.norm(res[3:])) / max(w, 1e-9) if res.size > 3 else 0.0

        if pos_residual > self.ik_unreachable_threshold:
            self.get_logger().warn(
                f'Target ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) '
                f'position residual {pos_residual:.4f} m (rot {rot_residual:.3f} rad)'
            )

        self.last_ik_solution = result.x
        return (
            float(result.x[0]),
            float(result.x[1]),
            float(result.x[2]),
            float(result.x[3]),
            float(result.x[4]),
            pos_residual,
            rot_residual,
        )

    def target_changed_enough(self, xyz, quat):
        if self.last_target_xyz is None or self.last_target_quat is None:
            return True
        if any(abs(a - b) > self.position_threshold for a, b in zip(xyz, self.last_target_xyz)):
            return True
        # Quaternion sign-flip-invariant distance: 1 - |dot|.
        d = abs(sum(a * b for a, b in zip(quat, self.last_target_quat)))
        return (1.0 - d) > 1e-4

    def joints_changed_enough(self, joints):
        if self.last_joint_positions is None:
            return True
        return any(abs(a - b) > self.joint_threshold for a, b in zip(joints, self.last_joint_positions))

    def pose_cb(self, msg: PoseStamped):
        try:
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = float(msg.pose.position.z)
            qx = float(msg.pose.orientation.x)
            qy = float(msg.pose.orientation.y)
            qz = float(msg.pose.orientation.z)
            qw = float(msg.pose.orientation.w)

            xyz = [x, y, z]
            quat = [qx, qy, qz, qw]
            target_rot = quat_to_mat(quat)

            gripper_changed = (
                abs(self.current_gripper_target - self.last_published_gripper)
                > self.gripper_change_threshold
            )
            target_changed = self.target_changed_enough(xyz, quat)
            if not target_changed and not gripper_changed:
                return

            pan, lift, elbow, wflex, wroll, pos_res, rot_res = self.solve_ik(xyz, target_rot)

            positions = [
                pan,
                lift,
                elbow,
                wflex,
                wroll,
                self.current_gripper_target,
            ]

            if not gripper_changed and not self.joints_changed_enough(positions):
                self.last_target_xyz = xyz
                self.last_target_quat = quat
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
            self.last_target_quat = quat
            self.last_joint_positions = positions[:]
            self.last_published_gripper = self.current_gripper_target

            self.get_logger().info(
                f'Trajectory | target_xyz=({x:.4f}, {y:.4f}, {z:.4f}) '
                f'target_quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) -> '
                f'q5=[{pan:.3f}, {lift:.3f}, {elbow:.3f}, {wflex:.3f}, {wroll:.3f}] '
                f'grip={self.current_gripper_target:.3f} '
                f'pos_res={pos_res:.4f} rot_res={rot_res:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')

    def gripper_cb(self, msg: Float32):
        # Binarize at threshold per OpenVLA bridge_orig convention:
        # value > threshold => open, else closed.
        self.current_gripper_target = (
            self.gripper_open_position
            if float(msg.data) > self.gripper_threshold
            else self.gripper_closed_position
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseToJointTrajectoryIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
