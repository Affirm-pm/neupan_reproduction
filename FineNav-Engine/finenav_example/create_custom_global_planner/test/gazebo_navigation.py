#!/usr/bin/python3
"""Headless Gazebo acceptance test; never publishes synthetic sensor data.

Requires the built workspace to be sourced. Uses separate ROS/Gazebo domains,
starts and stops only its own launch process group, and records a JSON trace.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=180., help='Wall seconds per goal')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    output = args.output_dir or Path(tempfile.mkdtemp(prefix='finenav-gazebo-'))
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = '119'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['IGN_PARTITION'] = f'finenav_acceptance_{os.getpid()}'
    os.environ['GZ_PARTITION'] = os.environ['IGN_PARTITION']
    os.environ['ROS_LOG_DIR'] = str(output / 'ros')
    os.environ['PATH'] = '/usr/bin:/bin:/opt/ros/humble/bin:' + os.environ['PATH']
    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Pose, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray
    from finenav_interfaces.action import NavigateToPose

    rclpy.init()
    node = rclpy.create_node('gazebo_navigation_acceptance')
    latest = {'odom': None, 'cloud': 0, 'cmd': None}
    trace = {'samples': [], 'paths': [], 'goals': [], 'passed': False}

    def yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def odom_cb(msg):
        latest['odom'] = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not trace['samples'] or stamp - trace['samples'][-1]['t'] >= .1:
            p = msg.pose.pose
            trace['samples'].append({'t': stamp, 'x': p.position.x, 'y': p.position.y,
                                     'yaw': yaw(p.orientation), 'v': msg.twist.twist.linear.x})

    def path_cb(msg):
        points = [p for marker in msg.markers for p in marker.points]
        if points:
            trace['paths'].append([[p.x, p.y] for p in points])

    node.create_subscription(Odometry, '/odom', odom_cb, 10)
    node.create_subscription(PointCloud2, '/cloud_for_lio',
        lambda msg: latest.update(cloud=latest['cloud'] + 1), qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda msg: latest.update(cmd=msg), 10)
    node.create_subscription(MarkerArray, '/finenav_global_path', path_cb, 10)
    client = ActionClient(node, NavigateToPose, '/hello_finenav/navigate_to_pose')
    print(f'Artifacts: {output}', flush=True)
    with (output / 'launch.log').open('w') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'finenav_example', 'hello_finenav.launch.py',
            'use_rviz:=false', 'gazebo_gui:=false', 'start_paused:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            def spin_until(predicate, timeout, description):
                deadline, next_report = time.monotonic() + timeout, time.monotonic() + 10
                while not predicate():
                    if launch.poll() is not None:
                        raise RuntimeError(f'Launch exited ({launch.returncode}); see {output / "launch.log"}')
                    if time.monotonic() > deadline:
                        raise TimeoutError(description)
                    rclpy.spin_once(node, timeout_sec=.05)
                    if time.monotonic() >= next_report:
                        print(description, trace['samples'][-1:] or latest['cloud'], flush=True)
                        (output / 'trace.json').write_text(json.dumps(trace, indent=2))
                        next_report += 10

            spin_until(lambda: latest['odom'] and latest['cloud'] >= 10 and client.server_is_ready(),
                       75, 'Waiting for Gazebo odometry/lidar/action server')
            # Let the spawned robot settle on the floor before issuing the first goal.
            settle = time.monotonic() + 2
            spin_until(lambda: time.monotonic() > settle, 5, 'Settling robot')
            for name, gx, gy, gyaw in [('nearby', 2., 3., math.pi/2), ('wall_detour', 5., 6., 0.)]:
                goal = NavigateToPose.Goal()
                pose = Pose()
                pose.position.x, pose.position.y = gx, gy
                pose.orientation.z, pose.orientation.w = math.sin(gyaw/2), math.cos(gyaw/2)
                goal.poses = [pose]
                goal.behavior_tree = 'GettingStartedNavigate'
                record = {'name': name, 'target': [gx, gy, gyaw],
                          'path_offset': len(trace['paths']), 'sample_offset': len(trace['samples'])}
                trace['goals'].append(record)
                accepted = client.send_goal_async(goal)
                spin_until(accepted.done, 10, 'Waiting for goal acceptance')
                handle = accepted.result()
                assert handle.accepted, 'Navigation goal rejected'
                result = handle.get_result_async()
                spin_until(result.done, args.timeout, f'Navigating {name}')
                record['status'] = result.result().status
                assert record['status'] == GoalStatus.STATUS_SUCCEEDED, f'{name}: navigation failed with status {record["status"]}'
                stop_deadline = time.monotonic() + 1
                spin_until(lambda: time.monotonic() > stop_deadline, 3, 'Waiting for stop')
                actual = latest['odom'].pose.pose
                error = math.hypot(actual.position.x-gx, actual.position.y-gy)
                yaw_error = abs(math.atan2(math.sin(yaw(actual.orientation)-gyaw),
                                           math.cos(yaw(actual.orientation)-gyaw)))
                record.update(position_error=error, yaw_error=yaw_error,
                              final_pose=[actual.position.x, actual.position.y, yaw(actual.orientation)])
                assert error < .25 and yaw_error < .1, f'Final pose outside tolerances: {record}'
                assert latest['cmd'] and abs(latest['cmd'].linear.x) + abs(latest['cmd'].angular.z) < 1e-6
                twist = latest['odom'].twist.twist
                record['final_twist'] = [twist.linear.x, twist.angular.z]
                assert abs(twist.linear.x) < .02 and abs(twist.angular.z) < .03, \
                    f'Robot still moving after stop: {record["final_twist"]}'
                paths = trace['paths'][record['path_offset']:]
                assert any(math.hypot(path[-1][0] - gx, path[-1][1] - gy) < 1e-5
                           for path in paths), 'No global route ending at this goal'
                if name == 'wall_detour':
                    crossing = [p for p in trace['samples'][record['sample_offset']:]
                                if 3.8 <= p['y'] <= 4.2]
                    assert crossing and all(p['x'] < 2.5 for p in crossing), \
                        'Robot did not pass to the left of the wall at y=4, x=[3,7]'
                print(f'PASS {name}: position error={error:.3f} m, yaw error={yaw_error:.3f} rad', flush=True)
        finally:
            (output / 'trace.json').write_text(json.dumps(trace, indent=2))
            # ros2 launch forwards SIGINT to its children; signalling the whole
            # group here would deliver it twice and interrupt Python cleanup.
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    lines = (output / 'launch.log').read_text().splitlines()
    assert any('[hello_finenav-' in line and 'process has finished cleanly' in line
               for line in lines), 'FineNav engine did not exit cleanly'
    trace['passed'] = True
    (output / 'trace.json').write_text(json.dumps(trace, indent=2))
    print(f'PASS Gazebo closed-loop navigation: {output}', flush=True)


if __name__ == '__main__':
    main()
