import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('finenav_simulator')

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose localization chain: fast_lio or gazebo_odom (legacy aliases ground_truth/static now fall back to gazebo_odom)'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='flat_small_scale_maze.world',
        description='World file name under finenav_simulator/resource/worlds'
    )

    world_sdf_path = PathJoinSubstitution([
        pkg,
        'resource',
        'worlds',
        LaunchConfiguration('world_name'),
    ])
    ign_config_path = os.path.join(pkg, 'resource', 'ign', 'gui.config')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "core", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    include_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "core", "robot.launch.py")
        ),
        launch_arguments={
            'lio_type': LaunchConfiguration('lio_type'),
            'world_name': LaunchConfiguration('world_name'),
        }.items(),
    )

    pointcloud_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "core", "pointcloud_complete_launch.py")
                ),
                launch_arguments={
                    'lio_type': LaunchConfiguration('lio_type'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        declare_lio_type,
        declare_world_name,
        gazebo_launch,
        include_robot_launch,
        pointcloud_launch
    ])
