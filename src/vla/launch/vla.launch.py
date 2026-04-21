#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla',
            executable='vla_action_client.py',
            name='vla_action_client',
            output='screen',
            parameters=[{
                'camera_topic': '/camera/image_raw',
                'action_topic': '/vla_action',
                'status_topic': '/openvla/status',
                'instruction': 'pick up the red ball',
                'server_url': 'http://127.0.0.1:8000/act',
                'unnorm_key': 'bridge_orig',
                'timeout_sec': 120.0,
                'publish_rate_hz': 1.0,
                'max_abs_translation': 0.05,
                'max_abs_rotation': 0.5,
                'gripper_threshold': 0.0,
            }]
        ),

        Node(
            package='vla',
            executable='action_to_ee.py',
            name='action_to_ee',
            output='screen',
            parameters=[{
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
        ),

        Node(
            package='your_package',
            executable='vla_ik.py',
            name='target_pose_to_joint_trajectory_ik',
            output='screen',
            parameters=[{
                'input_topic': '/vla_target_pose',
                'trajectory_topic': '/arm_controller/joint_trajectory',
                'shoulder_offset_x': 0.0388,
                'shoulder_offset_z': 0.0624,
                'upper_arm_length': 0.120,
                'lower_arm_length': 0.135,
                'wrist_flex': 0.5,
                'wrist_roll': 0.0,
                'gripper': 0.0,
                'move_time_sec': 2.0,
                'position_threshold': 0.01,
                'joint_threshold': 0.02,
            }]
        ),
    ])