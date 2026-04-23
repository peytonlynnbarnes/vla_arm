#!/usr/bin/env python3

# Same pipeline as vla.launch.py but with fake_vla_action_client.py in place
# of the real one — no OpenVLA server needed. Useful for exercising
# action_to_ee + vla_ik + the Gazebo controller end-to-end.
#
# Flow on launch:
#   1. Publish a one-shot JointTrajectory to drive the arm to a "viewing" pose
#      (matches the command in cmds.txt). This pose has the arm bent so the
#      wrist camera looks down at the table and the EE is pre-positioned near
#      the balls.
#   2. After 4s (arm settle time), bring up the fake action client,
#      action_to_ee, and vla_ik.

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


STARTING_POSE_CMD = (
    "ros2 topic pub -t 1 /arm_controller/joint_trajectory "
    "trajectory_msgs/msg/JointTrajectory "
    "\"{joint_names: ['shoulder_pan','shoulder_lift','elbow_flex',"
    "'wrist_flex','wrist_roll','gripper'], "
    "points: [{positions: [0.0, -1.0, 1.0, 0.5, 0.0, -0.17], "
    "time_from_start: {sec: 2, nanosec: 0}}]}\""
)


def generate_launch_description():
    move_to_start = ExecuteProcess(
        cmd=['bash', '-lc', STARTING_POSE_CMD],
        output='screen',
    )

    fake_client = Node(
        package='vla',
        executable='fake_vla_action_client.py',
        name='fake_vla_action_client',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'action_topic': '/vla_action',
            'status_topic': '/openvla/status',
            'publish_rate_hz': 2.0,
            'mode': 'grab',
            # Red ball sits on the table top at world (0.22, -0.10, 0.11);
            # arm base is at world z=0.2, so target in base_link is (0.22, -0.10, -0.09).
            'grab_target_x': 0.22,
            'grab_target_y': -0.10,
            'grab_target_z': -0.09,
            'grab_hover_height': 0.10,
            'grab_lift_height': 0.15,
            'grab_approach_time_sec': 7.0,
            'grab_descend_time_sec': 5.0,
            'grab_grip_time_sec': 2.5,
            'grab_lift_time_sec': 4.0,
            'grab_hold_time_sec': 4.0,
            'grab_base_frame': 'base_link',
            'grab_ee_frame': 'gripper_frame_link',
            'grab_max_step': 0.04,
            'grab_gripper_open': -0.17,
            'grab_gripper_closed': 0.8,
            'grab_approach_offset_x': 0.04,
            'grab_enclose_time_sec': 2.0,
        }]
    )

    action_to_ee = Node(
        package='vla',
        executable='action_to_ee.py',
        name='action_to_ee',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'action_topic': '/vla_action',
            'pose_topic': '/vla_target_pose',
            'gripper_topic': '/vla_gripper',
            'base_frame': 'base_link',
            'ee_frame': 'gripper_frame_link',
            'translation_scale': 1.0,
            'rotation_scale': 1.0,
            'max_translation_step': 0.05,
            'max_rotation_step': 0.5,
            'publish_gripper': True,
        }]
    )

    vla_ik = Node(
        package='vla',
        executable='vla_ik.py',
        name='target_pose_to_joint_trajectory_ik',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/vla_target_pose',
            'gripper_topic': '/vla_gripper',
            'trajectory_topic': '/arm_controller/joint_trajectory',
            # FK is hardcoded from the URDF chain; the old 2-link `*_offset_*` /
            # `*_arm_length` params are gone. Fixed wrist angles are still params.
            'wrist_flex': 0.5,
            'wrist_roll': 0.0,
            'gripper': -0.17,
            'move_time_sec': 0.6,
            'position_threshold': 0.01,
            'joint_threshold': 0.02,
            'ik_unreachable_threshold': 0.02,
            'ik_max_nfev': 60,
        }]
    )

    delayed_nodes = TimerAction(
        period=4.0,
        actions=[fake_client, action_to_ee, vla_ik],
    )

    return LaunchDescription([
        move_to_start,
        delayed_nodes,
    ])
