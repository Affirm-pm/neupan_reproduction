"""Runs each headless driver in this directory against the node it checks.

colcon test picks this up through add_launch_test. Each driver is a plain
rclpy script that exits 0 on PASS; this file only starts the node with the
parameters and remaps the driver expects and checks the driver's exit code.
"""
import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts

HERE = os.path.dirname(os.path.abspath(__file__))


def planner(config, model):
    share = get_package_share_directory('neupan_cpp_ros')
    return launch_ros.actions.Node(
        package='neupan_cpp_ros', executable='neupan_node', output='screen',
        parameters=[{
            'config_file': os.path.join(share, 'config', config),
            'dune_checkpoint': os.path.join(share, 'models', model),
        }],
        # The drivers command /cmd_vel and send RViz-style /goal_pose goals.
        remappings=[('neupan_cmd_vel', 'cmd_vel'),
                    ('neupan_goal', 'goal_pose')])


def astar():
    return launch_ros.actions.Node(
        package='neupan_cpp_ros', executable='astar_global_node',
        output='screen')


SCENARIOS = {
    'smoke_test.py': lambda: planner('planner.yaml', 'diff_default.bin'),
    'sentry_smoke_test.py': lambda: planner('sentry_diff.yaml', 'diff_sentry.bin'),
    'closed_loop_test.py': lambda: planner('sentry_diff.yaml', 'diff_sentry.bin'),
    'path_overshoot_test.py': lambda: planner('sentry_diff.yaml', 'diff_sentry.bin'),
    'astar_test.py': astar,
}


@launch_testing.parametrize('driver', list(SCENARIOS))
def generate_test_description(driver):
    driver_proc = launch.actions.ExecuteProcess(
        cmd=[sys.executable, os.path.join(HERE, driver)], output='screen')
    return launch.LaunchDescription([
        SCENARIOS[driver](),
        driver_proc,
        launch_testing.actions.ReadyToTest(),
    ]), {'driver_proc': driver_proc}


class TestDriver(unittest.TestCase):

    def test_driver_finishes(self, proc_info, driver_proc):
        proc_info.assertWaitForShutdown(process=driver_proc, timeout=90)


@launch_testing.post_shutdown_test()
class TestDriverResult(unittest.TestCase):

    def test_driver_passed(self, proc_info, driver_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=driver_proc)
