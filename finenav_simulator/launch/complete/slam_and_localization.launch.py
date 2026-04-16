# Copyright (c) 2024.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('finenav_simulator'), 'config')

    fast_lio_condition = LaunchConfigurationEquals('lio_type', 'fast_lio')
    
    # static 模式下不需要 map 到 odom 的静态 TF
    dynamic_mode_condition = LaunchConfigurationEquals('navigation_mode', 'dynamic')

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

    declare_nav_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='dynamic',
        description='Choose the mode of navigation: dynamic or static'
    )

    declare_map_save = DeclareLaunchArgument(
        'map_save',
        default_value='false',
        description='Enable octomap saving after navigation'
    )

    declare_map_load = DeclareLaunchArgument(
        'map_load',
        default_value='false',
        description='Enable octomap loading a map before navigation'
    )

    declare_enable_octomap = DeclareLaunchArgument(
        'enable_octomap',
        default_value='true',
        description='Enable octomap server when running the fast_lio pipeline'
    )

    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='false',
        description='Enable publishing map to odom TF transform'
    )

    fast_lio_config = os.path.join(config_dir, 'fastlio_gazebo_config.yaml')
    fast_lio_params = [
        fast_lio_config,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
    ]

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=fast_lio_params,
        output='screen',
        condition=fast_lio_condition
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='both',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'latch': False,
            'occupancy_min_z': 0.15,
            'occupancy_max_z': 7.45,
            'sensor_model.max_range': 7.0,
            'point_cloud_max_z': 7.45,
            'filter_ground_plane': False,
            'sensor_height': 0.40,
            'r_min': 0.2,
            'r_max': 7.0,
            'n_bins': 120,
            'n_segments': 360,
            'max_dist_to_line': 0.05,
            'min_slope': 0.0,
            'max_slope': 0.3,
            'max_fit_error': 0.05,
            'long_threshold': 1.0,
            'max_long_height': 0.1,
            'max_start_height': 0.2,
            'line_search_angle': 0.1,
            'visualize': False,
        }],
        remappings=[('/cloud_in', '/cloud_registered')],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('lio_type'), "' == 'fast_lio' and '",
                LaunchConfiguration('enable_octomap'), "' == 'true'"
            ])
        ),
    )

    map_to_odom_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('lio_type'), "' == 'fast_lio' and '",
                LaunchConfiguration('navigation_mode'), "' == 'dynamic' and '",
                LaunchConfiguration('publish_map_tf'), "' == 'true'"
            ])
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_map_save)
    ld.add_action(declare_map_load)
    ld.add_action(declare_enable_octomap)
    ld.add_action(declare_publish_map_tf)
    ld.add_action(declare_nav_mode)
    ld.add_action(map_to_odom_static_node)
    ld.add_action(fast_lio_node)
    ld.add_action(octomap_server_node)

    return ld
