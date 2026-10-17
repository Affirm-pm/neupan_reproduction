#!/usr/bin/env python3
"""Launch bag eval for FineNavLocalizer.

Usage:
    ros2 launch finenav_localizer eval.launch.py bag:=test_bag1
    ros2 launch finenav_localizer eval.launch.py bag:=test_bag2
    ros2 launch finenav_localizer eval.launch.py bag:=test_bag3
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("finenav_localizer")

    # ── Launch arguments ────────────────────────────────────────────────────
    bag_arg = DeclareLaunchArgument(
        "bag", default_value="test_bag1",
        description="Bag name (subdirectory under share/finenav_localizer/eval/)")
    bag_name = LaunchConfiguration("bag")

    # ── Paths ───────────────────────────────────────────────────────────────
    config_path   = os.path.join(pkg_share, "eval", "config", "eval_params.yaml")
    record_script = os.path.join(pkg_share, "eval", "record_eval.py")
    bag_path      = PathJoinSubstitution([pkg_share, "eval", bag_name])

    # Build the record_eval command via bash -c so LaunchConfiguration
    # substitutions are resolved by the shell string, not passed as raw tuples.
    record_cmd = PythonExpression([
        '"python3 ', record_script,
        ' --ros-args -p output_csv:=/tmp/localizer_', bag_name, '.csv',
        ' -r \\"~/nav_state:=/localizer_eval_node/finenav_localizer/nav_state\\""',
    ])

    # ── 1. FineNavLocalizer (C++) ───────────────────────────────────────────
    # Use ExecuteProcess instead of Node action — Node injects __node remapping
    # which would rename the internal finenav_localizer sub-node.
    localizer_node = ExecuteProcess(
        cmd=["ros2", "run", "finenav_localizer", "finenav_localizer_node",
             "--ros-args", "--params-file", config_path],
        output="screen",
    )

    # ── 2. Record-eval node (Python) ────────────────────────────────────────
    record_node = ExecuteProcess(
        cmd=["bash", "-c", record_cmd],
        output="screen",
    )

    # ── 3. Bag playback ─────────────────────────────────────────────────────
    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "1000", "-r", "1.0"],
        output="screen",
    )

    # When the bag finishes (or is interrupted), shut down everything cleanly
    bag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play,
            on_exit=[Shutdown()],
        )
    )

    return LaunchDescription([
        bag_arg,
        localizer_node,
        record_node,
        bag_play,
        bag_exit,
    ])
