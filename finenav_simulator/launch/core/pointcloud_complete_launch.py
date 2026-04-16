import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    build_tf_args,
    generate_robot_sdf_from_xmacro,
    load_robot_sdf_config_from_string,
)


def generate_launch_description():
    pkg = get_package_share_directory("finenav_simulator")
    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xmacro")
    robot_xml = generate_robot_sdf_from_xmacro(sdf_xmacro_path)
    sdf_config = load_robot_sdf_config_from_string(robot_xml)
    robot_name = "fine_robot"

    declare_lio_type = DeclareLaunchArgument(
        "lio_type",
        default_value="fast_lio",
        description="Choose localization chain: fast_lio or gazebo_odom",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    gpu_lidar_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gpu_lidar_frame_alias',
        arguments=build_tf_args(
            'lidar_frame',
            f"{robot_name}/{sdf_config['lidar_parent_link']}/{sdf_config['lidar_sensor_name']}",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    converter_node = Node(
        package='finenav_simulator',
        executable='point_cloud_converter_node',
        name='point_cloud_converter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'pcd_topic': '/livox/lidar'},
            {'output_topic': '/cloud_for_lio'},
            {'output_frame_id': 'lidar_frame'},
            {'vertical_samples': sdf_config['lidar_vertical_samples']},
            {'horizontal_samples': sdf_config['lidar_horizontal_samples']},
            {'vertical_min_angle': sdf_config['lidar_vertical_min_angle']},
            {'vertical_max_angle': sdf_config['lidar_vertical_max_angle']},
            {'horizontal_min_angle': sdf_config['lidar_horizontal_min_angle']},
            {'horizontal_max_angle': sdf_config['lidar_horizontal_max_angle']},
            {'scan_period': sdf_config['lidar_scan_period']},
            {'synthesize_time': True},
        ]
    )

    return LaunchDescription([
        declare_lio_type,
        declare_use_sim_time,
        gpu_lidar_frame_alias,
        converter_node,
    ])
