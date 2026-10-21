#!/usr/bin/python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the running original FineNav maze; uses actual Gazebo odometry/clouds."""
import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from finenav_interfaces.action import NavigateToPose


def heading(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=4.0)
    parser.add_argument('--y', type=float, default=2.0)
    parser.add_argument('--yaw', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--blocked', action='store_true', help='Observe a wall goal for 15 seconds, then cancel')
    parser.add_argument('--output', type=Path, default=Path('/tmp/finenav-original-navigation.json'))
    args = parser.parse_args()
    # The flat maze is composed of these axis-aligned wall rectangles.
    sim = Path(get_package_share_directory('finenav_simulator'))
    walls = []
    for part in ET.parse(sim/'resource/model/smaze2d/model.sdf').getroot().findall('model/include'):
        pose = [float(x) for x in part.findtext('pose').split()]
        model = part.findtext('uri').removeprefix('model://')
        dims = ET.parse(sim/f'resource/model/{model}/model.sdf').getroot().findtext('model/link/collision/geometry/box/size')
        length, width, _ = map(float, dims.split())
        if abs(math.sin(pose[5])) > 0.5:
            length, width = width, length
        walls.append((pose[0], pose[1], length/2, width/2))
    rclpy.init()
    node = Node('neupan_original_sim_test')
    counts = dict(odom=0, raw_cloud=0, aligned_cloud=0, wall_overlaps=0)
    odom = None
    commands = []
    last = {}
    def on_odom(msg):
        nonlocal odom
        odom = msg
        counts['odom'] += 1
        last['odom'] = time.monotonic()
        p=msg.pose.pose.position
        a=heading(msg.pose.pose.orientation)
        c,s=math.cos(a),math.sin(a)
        for x,y,hx,hy in walls:
            dx,dy=p.x-x,p.y-y
            # Separating-axis check, expanded robot envelope by 5 cm.
            if (abs(dx)<hx+0.40*abs(c)+0.35*abs(s) and
                abs(dy)<hy+0.40*abs(s)+0.35*abs(c) and
                abs(dx*c+dy*s)<0.40+hx*abs(c)+hy*abs(s) and
                abs(-dx*s+dy*c)<0.35+hx*abs(s)+hy*abs(c)):
                counts['wall_overlaps'] += 1
    def cloud(msg, name):
        if name == 'aligned_cloud':
            assert msg.header.frame_id == 'map'
        if msg.width*msg.height:
            counts[name] += 1
            last[name] = time.monotonic()
    node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/livox/lidar',lambda m: cloud(m,'raw_cloud'),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/aligned_cloud',lambda m: cloud(m,'aligned_cloud'),qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel',lambda m: commands.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def wait(predicate, timeout):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:
                raise RuntimeError('Timed out waiting for sensors/action/stop')
            rclpy.spin_once(node,timeout_sec=0.05)
    report=dict(passed=False,goal=[args.x,args.y,args.yaw],world='flat_small_scale_maze.world',blocked=args.blocked)
    client=ActionClient(node,NavigateToPose,'/hello_finenav/navigate_to_pose')
    handle=None
    try:
        wait(lambda: all(counts[k]>=10 for k in ('odom','raw_cloud','aligned_cloud')),30)
        assert client.wait_for_server(timeout_sec=10)
        goal=NavigateToPose.Goal()
        p=Pose()
        p.position.x,p.position.y=args.x,args.y
        p.orientation.z,p.orientation.w=math.sin(args.yaw/2),math.cos(args.yaw/2)
        goal.poses=[p]
        goal.behavior_tree='NeuPANNavigate'
        sent=client.send_goal_async(goal)
        wait(sent.done,10)
        handle=sent.result()
        assert handle.accepted
        result=handle.get_result_async()
        if args.blocked:
            until=time.monotonic()+15
            wait(lambda: time.monotonic()>until,20)
            assert not result.done(), 'Wall goal falsely succeeded'
            cancelled=handle.cancel_goal_async()
            wait(cancelled.done,5)
            wait(result.done,5)
        else:
            wait(result.done,args.timeout)
            assert result.result().status==4, 'Navigation action did not succeed'
        report['action_status']=result.result().status
        until=time.monotonic()+2
        wait(lambda: time.monotonic()>until,5)
        p=odom.pose.pose.position
        yaw=heading(odom.pose.pose.orientation)
        report.update(final_pose=[p.x,p.y,yaw],xy_error_m=math.hypot(p.x-args.x,p.y-args.y),
                      yaw_error_rad=abs(math.atan2(math.sin(yaw-args.yaw),math.cos(yaw-args.yaw))))
        recent=[(v,w) for t,v,w in commands if time.monotonic()-t<1]
        assert recent and all(abs(v)<1e-6 and abs(w)<1e-6 for v,w in recent), 'Nonzero final command'
        assert abs(odom.twist.twist.linear.x)<0.03 and abs(odom.twist.twist.angular.z)<0.05
        assert counts['wall_overlaps']==0, 'Robot envelope violated 5 cm wall clearance'
        assert all(time.monotonic()-last[k]<1 for k in ('odom','raw_cloud','aligned_cloud'))
        if not args.blocked:
            assert any(abs(v)>0.02 for _,v,_ in commands), 'No drive command observed'
        if not args.blocked:
            assert report['xy_error_m']<0.10 and report['yaw_error_rad']<0.10
        report['motion_and_clearance_checks_passed']=True
        if args.blocked:
            report['cancel_contract_passed']=report['action_status']==5
            assert report['cancel_contract_passed'], 'Engine cancellation did not return STATUS_CANCELED (5)'
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        if handle is not None:
            cancel=handle.cancel_goal_async()
            try: wait(cancel.done,3)
            except Exception: pass
        raise
    finally:
        report['counts']=counts
        if odom is not None:
            p=odom.pose.pose.position
            report.setdefault('final_pose',[p.x,p.y,heading(odom.pose.pose.orientation)])
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
