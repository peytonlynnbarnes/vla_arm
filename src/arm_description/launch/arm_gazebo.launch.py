import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'arm_description'
    pkg_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(pkg_share, 'description', 'so101.urdf.xacro')
    urdf_file = '/tmp/so101.urdf'

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )

    generate_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'so101_arm',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        generate_urdf,
        rsp,
        gazebo,
        spawn_robot
    ])