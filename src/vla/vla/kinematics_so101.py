"""Kinematic helpers for SO-101 — FK of base_link -> gripper_frame_link.

Copied/adapted from vla/vla_ik.py. Isolated here so scripted policies can use
the FK without pulling in rclpy/ROS state. Also provides a numerical IK solver
targeting both position and orientation of gripper_frame_link in base_link.
"""

from __future__ import annotations

import math
from typing import Optional  # noqa: F401

import numpy as np
from scipy.optimize import least_squares


# (origin_xyz, origin_rpy, axis_or_None). Revolute joints have axis (0,0,1);
# final tcp_jaw_joint is fixed (axis=None). Chain terminates at tcp_jaw_link,
# a synthetic frame placed between the two jaws (see so101.urdf.xacro).
# The earlier convention that ended at gripper_frame_link overshoots the
# physical jaws by ~10 cm and is wrong for grasp targeting.
CHAIN = [
    ((0.0388353, -8.97657e-09, 0.0624),     (3.14159, 0.0, -3.14159),     (0, 0, 1)),
    ((-0.0303992, -0.0182778, -0.0542),     (-1.5708, -1.5708, 0.0),      (0, 0, 1)),
    ((-0.11257, -0.028, 0.0),               (0.0, 0.0, 1.5708),           (0, 0, 1)),
    ((-0.1349, 0.0052, 0.0),                (0.0, 0.0, -1.5708),          (0, 0, 1)),
    ((0.0, -0.0611, 0.0181),                (1.5708, 0.0486795, 3.14159), (0, 0, 1)),
    ((0.012, 0.010, -0.035),                (0.0, 0.0, 0.0),              None),
]

# URDF joint limits (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)
JOINT_LOWER = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])
JOINT_UPPER = np.array([ 1.91986,  1.74533,  1.69,  1.65806,  2.84121])

JOINT_NAMES_ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


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
    """FK: joint values (5,) -> (position (3,), rotation (3,3)) of gripper_frame_link."""
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


def mat_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def rot_log(R):
    """Rotation matrix -> axis-angle vector (theta * axis)."""
    cos_theta = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-6:
        return np.zeros(3)
    if math.pi - theta < 1e-3:
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
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s
    return axis * theta


def solve_ik(target_pos, target_rot, q_seed=None, orientation_weight=0.5,
             wrist_roll_lock: Optional[float] = None, wrist_roll_weight: float = 0.0):
    """Solve 5-DOF IK to reach (target_pos, target_rot) with gripper_frame_link.

    - wrist_roll_lock: if not None, adds a soft residual pulling wrist_roll to this value
      (weighted by wrist_roll_weight). Useful to pin the jaw-open direction
      since position-only IK leaves wrist_roll otherwise arbitrary.
    """
    if q_seed is None:
        q_seed = np.zeros(5)

    def residual(q):
        pos, rot = fk_pose(q)
        e_pos = pos - np.asarray(target_pos)
        e_rot = rot_log(np.asarray(target_rot).T @ rot)
        out = [e_pos, orientation_weight * e_rot]
        if wrist_roll_lock is not None:
            out.append(np.array([wrist_roll_weight * (q[4] - wrist_roll_lock)]))
        return np.concatenate(out)

    try:
        res = least_squares(
            residual,
            np.clip(q_seed, JOINT_LOWER, JOINT_UPPER),
            bounds=(JOINT_LOWER, JOINT_UPPER),
            max_nfev=200,
            method='trf',
        )
        q = res.x
        pos, rot = fk_pose(q)
        pos_err = float(np.linalg.norm(pos - np.asarray(target_pos)))
        rot_err = float(np.linalg.norm(rot_log(np.asarray(target_rot).T @ rot)))
        return q, float(res.cost), pos_err, rot_err
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(f'IK failed: {exc}')


def down_rotation(yaw: float = 0.0):
    """Rotation matrix for 'gripper pointing down' with heading = yaw around base Z.

    Puts the gripper_frame_link's local +X axis along -Z_base (the approach
    direction). Y_gripper_base = (cos(yaw), sin(yaw), 0); Z = X × Y.
    Columns are [X_gripper, Y_gripper, Z_gripper] expressed in base frame.
    Determinant = +1 (verified for yaw=0).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [ 0.0,  c,    s],
        [ 0.0,  s,   -c],
        [-1.0,  0.0,  0.0],
    ])
