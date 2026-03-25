import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'arm_description'

    # --- Robot State Publisher (your URDF/xacro loader) ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp2.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- Launch Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ])
    )

    # --- Spawn robot into Gazebo ---
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'so101_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_robot
    ])