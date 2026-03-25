from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('arm_description')

    urdf_file = os.path.join(pkg_path, 'description', 'so101.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        # Publishes robot model
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # GUI to move joints manually
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])