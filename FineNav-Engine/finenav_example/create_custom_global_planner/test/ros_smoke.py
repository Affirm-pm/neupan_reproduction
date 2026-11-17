#!/usr/bin/python3
"""Real engine/plugin communication test with synthetic odometry and lidar.

Run after sourcing install/setup.bash. No Gazebo or physical robot is used.
The subprocesses run on an isolated ROS domain. This is not a Gazebo drive test.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '117'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/finenav-global-smoke-ros')

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray


def main():
    rclpy.init()
    node = rclpy.create_node('global_planner_smoke')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    mp = node.create_publisher(OccupancyGrid, '/global_map', qos)
    op = node.create_publisher(Odometry, '/odom', 10)
    cp = node.create_publisher(PointCloud2, '/cloud_for_lio', 10)
    gp = node.create_publisher(PoseStamped, '/goal_pose', 10)
    commands, paths = [], []
    node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((time.monotonic(), m)), 10)
    node.create_subscription(MarkerArray, '/finenav_global_path', paths.append, 10)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.transform.rotation.w = 1.
    tf.sendTransform(transform)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.1
    grid.info.width = grid.info.height = 100
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 10000
    for y in range(65):
        for x in range(48, 53):
            grid.data[y*100+x] = 100
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.pose.pose.position.x, odom.pose.pose.position.y = 2., 4.
    odom.pose.pose.orientation.w = 1.
    obstacles = [(4.8, y*0.1, 0.4) for y in range(65)]

    def publish_inputs():
        stamp = node.get_clock().now().to_msg()
        odom.header.stamp = grid.header.stamp = stamp
        op.publish(odom)
        mp.publish(grid)
        cp.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='map'), obstacles))

    node.create_timer(0.05, publish_inputs)
    share = Path(get_package_share_directory('finenav_example'))
    binary = Path(get_package_prefix('finenav_example')) / 'lib/finenav_example/hello_finenav'
    fd, filename = tempfile.mkstemp(prefix='finenav-global-smoke-', suffix='.log')
    os.close(fd)
    log_path = Path(filename)
    with log_path.open('w') as log:
        engine = subprocess.Popen([str(binary), '--ros-args',
            '--params-file', str(share / 'config/hello_finenav.yaml'),
            '--params-file', str(share / 'neupan_sim.yaml'),
            '-p', 'use_sim_time:=false'], stdout=log, stderr=subprocess.STDOUT)
        try:
            def spin_for(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    if engine.poll() is not None:
                        raise AssertionError(f'Engine exited: {engine.returncode}; {log_path}')
                    rclpy.spin_once(node, timeout_sec=0.05)

            spin_for(3)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = node.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = 8., 4.
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(.7), math.cos(.7)
            gp.publish(goal)
            spin_for(4)
            assert paths, f'No global path; see {log_path}'
            points = [p for msg in paths for marker in msg.markers for p in marker.points]
            assert max(p.y for p in points) > 7., 'Route did not go around the wall'
            assert any(abs(m.linear.x) + abs(m.angular.z) > .01 for _, m in commands), \
                f'NeuPAN did not produce a command; see {log_path}'
            print(f'PASS: official A* detour published ({len(paths)} updates), NeuPAN generated motion')
            route = [p for marker in paths[-1].markers for p in marker.points]
            obstacle = next(p for p in route if math.hypot(p.x - 2., p.y - 4.) > 1.8)
            original_obstacles = list(obstacles)
            obstacles.extend((obstacle.x + dx*.1, obstacle.y + dy*.1, .4)
                             for dx in range(-2, 3) for dy in range(-2, 3))
            offset = len(paths)
            spin_for(2.5)
            detours = [[p for marker in msg.markers for p in marker.points] for msg in paths[offset:]]
            assert any(route and min(math.hypot(p.x-obstacle.x, p.y-obstacle.y)
                                     for p in route) > .7 for route in detours), \
                f'Live cloud obstacle did not change the route on an unchanged map; {log_path}'
            print('PASS: new lidar obstacle replanned the route without changing the map or goal')
            obstacles[:] = original_obstacles
            offset = len(paths)
            spin_for(4.5)  # Wait past the live-cell persistence interval.
            cleared = [p for msg in paths[offset:] for marker in msg.markers for p in marker.points]
            assert any(math.hypot(p.x-obstacle.x, p.y-obstacle.y) < .3 for p in cleared), \
                'Removed live obstacle remained in the global map'
            print('PASS: removing the obstacle cleared the temporary overlay')
            grid.data = [100] * 10000
            changed = time.monotonic()
            spin_for(4)
            after = [m for t, m in commands if t > changed + 1.5]
            assert commands[-1][0] > changed, 'No command after map became blocked'
            assert abs(commands[-1][1].linear.x) + abs(commands[-1][1].angular.z) < 1e-6, \
                'Last command is not a stop'
            assert all(abs(m.linear.x) + abs(m.angular.z) < 1e-6 for m in after), \
                'Motion continued after global planning failure'
            print(f'PASS: blocked-map update stopped the control chain; log: {log_path}')
        finally:
            engine.send_signal(signal.SIGINT)
            try:
                engine.wait(timeout=8)
            except subprocess.TimeoutExpired:
                engine.kill()
                engine.wait()
            node.destroy_node()
            rclpy.shutdown()
    assert engine.returncode == 0, f"Unclean engine shutdown ({engine.returncode}): {log_path}"


if __name__ == '__main__':
    main()
