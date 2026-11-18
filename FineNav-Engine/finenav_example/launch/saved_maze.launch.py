"""Restore obstacles in Gazebo; discover them through live lidar, using the original maze as prior."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = Path(get_package_share_directory('finenav_example')) / 'launch/hello_finenav.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments={
            'world_name': 'maze_with_obstacles.sdf',
            'global_map_world_name': 'flat_small_scale_maze.world',
        }.items(),
    )])
