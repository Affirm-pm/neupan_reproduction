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
    parser.add_argument('--live-obstacle', action='store_true',
                        help='Insert a box on an active route using Gazebo; require a detour and arrival')
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
    import numpy as np
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Pose, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray
    from finenav_interfaces.action import NavigateToPose

    rclpy.init()
    node = rclpy.create_node('gazebo_navigation_acceptance')
    latest = {'odom': None, 'cloud': 0, 'cmd': None}
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    cloud_snapshot = {}
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
    def cloud_cb(msg):
        latest['cloud'] += 1
        if not args.live_obstacle:
            return
        try:
            t = tf_buffer.lookup_transform('map', msg.header.frame_id,
                                           rclpy.time.Time.from_msg(msg.header.stamp)).transform
            points = np.array([tuple(p) for p in read_points(msg, field_names=('x', 'y', 'z'),
                                                           skip_nans=True)], dtype=float).reshape(-1, 3)
            q = np.array([t.rotation.x, t.rotation.y, t.rotation.z])
            points += 2*np.cross(q, np.cross(q, points) + t.rotation.w*points)
            points += [t.translation.x, t.translation.y, t.translation.z]
            points = points[(points[:, 2] >= .15) & (points[:, 2] <= 2.)].tolist()
            cloud_snapshot.update(t=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                  points=points)
        except Exception:
            pass  # Only diagnostic capture; navigation uses its own timestamped TF.

    node.create_subscription(PointCloud2, '/cloud_for_lio', cloud_cb, qos_profile_sensor_data)
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
            # The simulator model origin is 0.75 m above base_link. The odometry
            # publisher must apply that offset before labelling its pose base_link.
            assert abs(latest['odom'].pose.pose.position.z) < .08, \
                'Ground-truth odometry is not at base_link height; lidar floor filtering is invalid'
            goals = [('live_obstacle', 6., 2., 0.)] if args.live_obstacle else [
                ('nearby', 2., 3., math.pi/2), ('wall_detour', 5., 6., 0.)]
            for name, gx, gy, gyaw in goals:
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
                if args.live_obstacle:
                    spin_until(lambda: len(trace['paths']) > record['path_offset'], 10,
                               'Waiting for the initial unobstructed path')
                    original = trace['paths'][-1]
                    assert any(abs(x-3.8) < .2 and abs(y-2.) < .2 for x, y in original), \
                        'Initial path does not pass through the insertion point'
                    sdf = ('<sdf version="1.7"><model name="live_route_box"><static>true</static>'
                           '<pose>3.8 2 0.5 0 0 0</pose><link name="box">'
                           '<collision name="collision"><geometry><box><size>0.6 0.6 1</size>'
                           '</box></geometry></collision><visual name="visual"><geometry><box>'
                           '<size>0.6 0.6 1</size></box></geometry></visual></link></model></sdf>')
                    response = subprocess.run(['ign', 'service', '-s', '/world/default/create',
                        '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '5000', '--req', 'sdf: ' + json.dumps(sdf)],
                        capture_output=True, text=True, timeout=10)
                    assert response.returncode == 0 and 'true' in response.stdout, \
                        f'Gazebo box insertion failed: {response.stdout} {response.stderr}'
                    record['obstacle'] = [3.8, 2., .6, .6]
                    record['insertion_path_offset'] = len(trace['paths'])
                    print('Inserted a 0.6 m box into the active path at (3.8, 2)', flush=True)
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
                if args.live_obstacle:
                    reroutes = trace['paths'][record['insertion_path_offset']:]
                    assert any(path and min(math.hypot(x-3.8, y-2.) for x, y in path) > .7
                               for path in reroutes), 'No global route around the inserted box'
                    samples = trace['samples'][record['sample_offset']:]
                    clearance = min(math.hypot(max(abs(p['x']-3.8)-.3, 0),
                                               max(abs(p['y']-2.)-.3, 0)) for p in samples)
                    record['box_clearance'] = clearance
                    assert clearance > .3, f'Robot passed too close to the box: {clearance}'
                print(f'PASS {name}: position error={error:.3f} m, yaw error={yaw_error:.3f} rad', flush=True)
        finally:
            (output / 'trace.json').write_text(json.dumps(trace, indent=2))
            if cloud_snapshot:
                (output / 'last_cloud.json').write_text(json.dumps(cloud_snapshot))
            # ros2 launch forwards SIGINT to its children; signalling the whole
            # group here would deliver it twice and interrupt Python cleanup.
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            # Ignition's Ruby launcher can exit while its headless server is
            # still alive. Reap remaining processes in this test's own group.
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            # Some Ignition versions detach the server into another group.
            # Match this run's unique partition before terminating any orphan.
            partition = ('IGN_PARTITION=' + os.environ['IGN_PARTITION']).encode()
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():
                    continue
                try:
                    argv = (proc / 'cmdline').read_bytes()
                    env = (proc / 'environ').read_bytes().split(b'\0')
                    if partition in env and b'gazebo' in argv:
                        os.kill(int(proc.name), signal.SIGKILL)
                except (FileNotFoundError, PermissionError, ProcessLookupError):
                    pass
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
