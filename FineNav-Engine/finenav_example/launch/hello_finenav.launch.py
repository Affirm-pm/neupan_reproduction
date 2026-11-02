from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = Path(get_package_share_directory("finenav_example"))
    simulator_dir = Path(get_package_share_directory("finenav_simulator"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="flat_small_scale_maze.world",
                description="World file under finenav_simulator/resource/worlds",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(example_dir / "config" / "hello_finenav.yaml"),
                description="FineNav-Engine runtime parameters",
            ),
            DeclareLaunchArgument(
                "neupan_params_file",
                default_value=str(example_dir / "neupan_sim.yaml"),
                description="NEUPAN planner and obstacle-source parameter overrides",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(example_dir / "neupan_sim.rviz"),
                description="RViz configuration for NEUPAN paths and obstacle points",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the example displays and goal tools",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(simulator_dir / "launch" / "complete" / "finenav_simulator.launch.py")
                ),
                launch_arguments={
                    "world_name": LaunchConfiguration("world_name"),
                    "lio_type": "gazebo_odom",
                    "use_sim_time": "true",
                    "publish_map_tf": "true",
                }.items(),
            ),
            Node(
                package="finenav_example",
                executable="hello_finenav",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    LaunchConfiguration("neupan_params_file"),
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            )
        ]
    )
