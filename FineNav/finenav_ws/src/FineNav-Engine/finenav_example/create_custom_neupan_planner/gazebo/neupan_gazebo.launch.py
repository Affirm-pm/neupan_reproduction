# SPDX-License-Identifier: GPL-3.0-or-later
"""NeuPAN wiring around the existing FineNav simulator, robot and worlds."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def setup(context):
    example = Path(get_package_share_directory('finenav_example'))
    sim = Path(get_package_share_directory('finenav_simulator'))
    assets = example/'create_custom_neupan_planner/gazebo'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    actions = [
        SetLaunchConfiguration('gazebo_extra_args', '-r' if gui else '-r -s --headless-rendering'),
        SetLaunchConfiguration('enable_keyboard_teleop', 'false'),
        # Gazebo's truth odometry tracks the model origin. The original model's
        # ground-level base_link is 0.75 m below that origin.
        SetLaunchConfiguration('ground_truth_base_frame', 'base_origin'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/complete/finenav_simulator.launch.py')),
            launch_arguments={'lio_type': 'gazebo_odom', 'use_sim_time': 'true',
                              'world_name': LaunchConfiguration('world_name'), 'publish_map_tf': 'true'}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher', name='neupan_model_origin_offset',
             arguments=['--x', '0', '--y', '0', '--z', '-0.75', '--frame-id', 'base_origin',
                        '--child-frame-id', 'base_link'], parameters=[{'use_sim_time': True}]),
        Node(package='finenav_example', executable='neupan_gazebo_adapter.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='finenav_example', executable='hello_finenav', output='screen',
             remappings=[('/cmd_vel', '/neupan/cmd_vel')],
             parameters=[str(example/'getting_started/params/sim_localizer.yaml'),
                         ParameterFile(str(assets/'neupan_gazebo.yaml'), allow_substs=True)]),
    ]
    for process in actions[-2:]:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='NeuPAN adapter or engine exited'))])))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(assets/'navigation.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    # Paths for private engine nodes must go in a parameter file: flat launch
    # parameter dictionaries target hello_finenav rather than my_layer.
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='flat_small_scale_maze.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
