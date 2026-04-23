"""One-shot launcher: brings up Gazebo + arm + controllers + move_group.

Starts arm_gazebo.launch.py, waits 8s for controllers, then launches move_group.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    arm_description_pkg = get_package_share_directory('arm_description')
    arm_moveit_config_pkg = get_package_share_directory('arm_moveit_config')

    render_engine = LaunchConfiguration('render_engine')
    moveit_start_delay = LaunchConfiguration('moveit_start_delay')

    declare_render_engine = DeclareLaunchArgument(
        'render_engine', default_value='ogre2')
    declare_moveit_delay = DeclareLaunchArgument(
        'moveit_start_delay', default_value='10.0',
        description='Seconds to wait after arm_gazebo before bringing up move_group.')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arm_description_pkg, 'launch', 'arm_gazebo.launch.py'])
        ),
        launch_arguments={'render_engine': render_engine}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arm_moveit_config_pkg, 'launch', 'arm_moveit.launch.py'])
        ),
    )

    delayed_moveit = TimerAction(period=10.0, actions=[moveit])

    return LaunchDescription([
        declare_render_engine,
        declare_moveit_delay,
        gazebo,
        delayed_moveit,
    ])
