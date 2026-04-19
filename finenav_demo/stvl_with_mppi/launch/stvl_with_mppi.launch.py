#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator')
    finenav_simulator_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'finenav_simulator.launch.py'
    )

    start_finenav_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(finenav_simulator_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'lio_type': 'static',
            'world_name': 'flat_small_scale_maze.world',
            'publish_map_tf': 'true',
        }.items()
    )

    start_hello_finenav = Node(
        package='finenav_demo',
        executable='hello_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(start_finenav_simulator)
    ld.add_action(start_hello_finenav)

    return ld