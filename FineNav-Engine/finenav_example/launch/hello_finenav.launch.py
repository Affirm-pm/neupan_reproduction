from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    example_dir = Path(get_package_share_directory("finenav_example"))
    simulator_dir = Path(get_package_share_directory("finenav_simulator"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_global_planner", default_value="true",
                                  description="Prepend the official FineNav A* planner to NeuPAN"),
            DeclareLaunchArgument("publish_global_map", default_value="true",
                                  description="Publish the selected prior world as the static navigation map"),
            DeclareLaunchArgument("global_map_topic", default_value="/global_map",
                                  description="World-frame OccupancyGrid (reliable, transient local)"),
            DeclareLaunchArgument(
                "world_name",
                default_value="flat_small_scale_maze.world",
                description="World file under finenav_simulator/resource/worlds",
            ),
            DeclareLaunchArgument(
                "global_map_world_name",
                default_value=LaunchConfiguration("world_name"),
                description="Prior world used for the static map, independently of the Gazebo scene",
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
                executable="maze_global_map.py",
                output="screen",
                parameters=[{"use_sim_time": True,
                             "world_name": LaunchConfiguration("global_map_world_name"),
                             "map_topic": LaunchConfiguration("global_map_topic")}],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("use_global_planner"), "'.lower() == 'true' and '",
                    LaunchConfiguration("publish_global_map"), "'.lower() == 'true'"
                ])),
            ),
            Node(
                package="finenav_example",
                executable="hello_finenav",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    LaunchConfiguration("neupan_params_file"),
                    {"use_sim_time": True,
                     "use_global_planner": ParameterValue(LaunchConfiguration("use_global_planner"), value_type=bool),
                     "global_grid_world_view.map_topic": LaunchConfiguration("global_map_topic")},
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
