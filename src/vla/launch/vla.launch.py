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
                'camera_topic': '/third_person/image_raw',
                'action_topic': '/vla_action',
                'status_topic': '/openvla/status',
                'instruction': 'pick up the blue ball',
                'server_url': 'http://127.0.0.1:8000/act',
                'unnorm_key': 'bridge_orig',
                'timeout_sec': 120.0,
                'publish_rate_hz': 1.0,
                'max_abs_translation': 0.05,
                'max_abs_rotation': 0.5,
                'gripper_threshold': 0.5,
                'phase_durations': [12.0, 8.0, 4.0, 6.0],
                'phase_instructions': [
                    'move the arm above the blue ball',
                    'lower the gripper onto the blue ball',
                    'close the gripper on the blue ball',
                    'lift the blue ball',
                ],
                'loop_phases': False,
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
            package='vla',
            executable='vla_ik.py',
            name='target_pose_to_joint_trajectory_ik',
            output='screen',
            parameters=[{
                'input_topic': '/vla_target_pose',
                'trajectory_topic': '/arm_controller/joint_trajectory',
                'wrist_flex_seed': 0.5,
                'wrist_roll_seed': 0.0,
                'gripper_open_position': 1.745,
                'gripper_closed_position': -0.174,
                'gripper_threshold': 0.5,
                'orientation_weight': 0.5,
                'move_time_sec': 2.0,
                'position_threshold': 0.01,
                'joint_threshold': 0.02,
            }]
        ),
    ])