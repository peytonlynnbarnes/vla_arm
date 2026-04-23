"""Launch move_group for the SO-101 arm.

Expects arm_gazebo.launch.py to already be running (or started concurrently) so
that /joint_states is published and the follow_joint_trajectory action servers
for arm_controller and gripper_controller are up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arm_description_pkg = get_package_share_directory('arm_description')
    arm_moveit_config_pkg = get_package_share_directory('arm_moveit_config')

    urdf_xacro = os.path.join(arm_description_pkg, 'description', 'so101.urdf.xacro')
    srdf_file = os.path.join(arm_moveit_config_pkg, 'config', 'so101.srdf')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name='so101_new_calib', package_name='arm_moveit_config')
        .robot_description(file_path=urdf_xacro)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=os.path.join(arm_moveit_config_pkg, 'config', 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(arm_moveit_config_pkg, 'config', 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(arm_moveit_config_pkg, 'config', 'moveit_controllers.yaml'))
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
            default_planning_pipeline='ompl',
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        move_group,
    ])
