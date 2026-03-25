import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'arm_description'
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    render_engine = LaunchConfiguration('render_engine')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    xacro_file = os.path.join(pkg_share, 'description', 'so101.urdf.xacro')
    urdf_file = '/tmp/so101.urdf'

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    )

    declare_render_engine = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        description='Gazebo render engine (ogre or ogre2)'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='so101_arm',
        description='Entity name for the spawned robot'
    )

    declare_x = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Spawn X position'
    )

    declare_y = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Spawn Y position'
    )

    declare_z = DeclareLaunchArgument(
        'z',
        default_value='0.2',
        description='Spawn Z position'
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':',
            os.path.dirname(pkg_share)  # <-- THIS is the fix
        ]
    )

    generate_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [
                '-r ',
                '--render-engine ', render_engine,
                ' ',
                world
            ]
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', urdf_file,
            '-x', x,
            '-y', y,
            '-z', z
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_render_engine,
        declare_robot_name,
        declare_x,
        declare_y,
        declare_z,
        gz_resource_path,
        generate_urdf,
        rsp,
        gazebo,
        delayed_spawn,
    ])