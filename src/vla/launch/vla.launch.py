from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla',
            executable='openvla_client.py',
            name='openvla_client',
            output='screen',
            parameters=['config/vla_params.yaml']
        ),
        Node(
            package='vla',
            executable='action_bridge.py',
            name='action_bridge',
            output='screen',
            parameters=['config/vla_params.yaml']
        )
    ])