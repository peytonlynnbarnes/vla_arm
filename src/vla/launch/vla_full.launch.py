#!/usr/bin/env python3
"""
One-shot bring-up: Gazebo world + SO-101 + controllers + VLA pipeline.

This wraps `arm_description/arm_gazebo.launch.py` and `vla/vla.launch.py`
into a single launch. It also sets PYTHONPATH so the VLA nodes can import
`json_numpy` from the project's .venv (system python3 does not have it).

Usage:
    ros2 launch vla vla_full.launch.py

    # Optional overrides:
    ros2 launch vla vla_full.launch.py \
        venv_site_packages:=/path/to/some/other/venv/lib/python3.12/site-packages \
        vla_start_delay:=20.0 \
        render_engine:=ogre

Assumes the OpenVLA inference server is reachable at http://127.0.0.1:8000/act
(typically via an SSH tunnel). The VLA nodes will block on POSTs until it
responds, but the sim itself will come up regardless.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declared args -----------------------------------------------------------

    declare_venv = DeclareLaunchArgument(
        'venv_site_packages',
        default_value='/home/peyton-peyton/vla_arm/.venv/lib/python3.12/site-packages',
        description=(
            'Path to the project venv site-packages. Prepended to PYTHONPATH so '
            'the VLA nodes can import json_numpy (which is not in system python3).'
        ),
    )

    declare_delay = DeclareLaunchArgument(
        'vla_start_delay',
        default_value='15.0',
        description=(
            'Seconds to wait after Gazebo launch before starting the VLA pipeline. '
            'Gives gz_sim + gz_ros2_control time to spawn the robot and activate '
            'both controllers (arm_gazebo.launch.py has its own 6 s internal delay).'
        ),
    )

    declare_render_engine = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre2',
        description='Forwarded to arm_gazebo.launch.py (ogre=CPU, ogre2=GPU).',
    )

    # Environment -------------------------------------------------------------

    # Prepend the venv site-packages to PYTHONPATH. SetEnvironmentVariable
    # propagates to all Nodes/IncludeLaunchDescriptions in this LaunchDescription.
    venv_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[
            LaunchConfiguration('venv_site_packages'),
            ':',
            EnvironmentVariable('PYTHONPATH', default_value=''),
        ],
    )

    # Included launches -------------------------------------------------------

    arm_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_description'),
                'launch',
                'arm_gazebo.launch.py',
            ])
        ),
        launch_arguments={
            'render_engine': LaunchConfiguration('render_engine'),
        }.items(),
    )

    vla_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vla'),
                'launch',
                'vla.launch.py',
            ])
        ),
    )

    # Delay the VLA pipeline until the sim + controllers are up.
    delayed_vla = TimerAction(
        period=LaunchConfiguration('vla_start_delay'),
        actions=[vla_launch],
    )

    return LaunchDescription([
        declare_venv,
        declare_delay,
        declare_render_engine,
        venv_pythonpath,
        arm_gazebo_launch,
        delayed_vla,
    ])
