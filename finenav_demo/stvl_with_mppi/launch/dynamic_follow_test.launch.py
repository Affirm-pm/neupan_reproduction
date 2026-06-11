#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo')
    mppi_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'mppi_params.yaml'
    )
    finenav_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'finenav_params.yaml'
    )

    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator')
    finenav_simulator_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'finenav_simulator.launch.py'
    )

    # 目标运动参数
    circle_radius = LaunchConfiguration('circle_radius', default='1.0')
    circle_period = LaunchConfiguration('circle_period', default='100.0')
    follow_distance = LaunchConfiguration('follow_distance', default='0.2')

    circle_radius_arg = DeclareLaunchArgument(
        'circle_radius', default_value='1.0',
        description='Radius of target circular trajectory [m]'
    )
    circle_period_arg = DeclareLaunchArgument(
        'circle_period', default_value='100.0',
        description='Period of target circular motion [s]'
    )
    follow_distance_arg = DeclareLaunchArgument(
        'follow_distance', default_value='0.2',
        description='Distance to stay behind the target [m]'
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
            mppi_params_file,
            finenav_params_file,
        ]
    )

    start_dynamic_follow_test = Node(
        package='finenav_demo',
        executable='dynamic_follow_test_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'circle_radius': circle_radius},
            {'circle_period': circle_period},
            {'follow_distance': follow_distance},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(circle_radius_arg)
    ld.add_action(circle_period_arg)
    ld.add_action(follow_distance_arg)
    ld.add_action(start_finenav_simulator)
    ld.add_action(start_hello_finenav)
    ld.add_action(start_dynamic_follow_test)

    return ld
