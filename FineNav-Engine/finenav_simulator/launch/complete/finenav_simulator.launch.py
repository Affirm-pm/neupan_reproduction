#  Copyright (c) 2025.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sim_dir = get_package_share_directory('finenav_simulator')
    launch_dir = os.path.join(sim_dir, 'launch', 'complete')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

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

    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='false',
        description='Enable publishing map to odom TF transform'
    )

    # Gazebo simulation and point-cloud pipeline
    simulator_infrastructure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulator_infrastructure.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_type': LaunchConfiguration('lio_type'),
            'world_name': LaunchConfiguration('world_name'),
        }.items()
    )

    # FAST-LIO and Gazebo odometry support
    slam_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_and_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_type': LaunchConfiguration('lio_type'),
            'publish_map_tf': LaunchConfiguration('publish_map_tf'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_world_name)
    ld.add_action(declare_publish_map_tf)
    ld.add_action(simulator_infrastructure)
    ld.add_action(slam_and_localization)

    return ld
