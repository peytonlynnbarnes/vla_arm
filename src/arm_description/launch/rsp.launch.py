import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_description')
    default_xacro_file = os.path.join(pkg_share, 'description', 'so101.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = LaunchConfiguration('xacro_file')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ]),
        value_type=str
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_xacro_file = DeclareLaunchArgument(
        'xacro_file',
        default_value=default_xacro_file,
        description='Absolute path to robot xacro file'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_xacro_file,
        robot_state_publisher_node,
    ])