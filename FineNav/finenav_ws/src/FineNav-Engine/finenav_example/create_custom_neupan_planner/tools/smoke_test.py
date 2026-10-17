#!/usr/bin/env python3
"""Isolated ROS 2 closed-loop + watchdog test; never uses the real driver."""
import os
os.environ.setdefault('ROS_DOMAIN_ID','187')
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ.setdefault('ROS_LOG_DIR','/tmp/finenav_neupan_ros_logs')
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import PointCloud2, PointField
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    rclpy.init()
    node=rclpy.create_node('neupan_integration_test')
    pose_pub=node.create_publisher(PoseWithCovarianceStamped,'/localization_pose',10)
    cloud_pub=node.create_publisher(PointCloud2,'/aligned_cloud',10)
    goal_pub=node.create_publisher(PoseStamped,'/goal_pose',10)
    state=[0.0,0.0,0.0]
    commands=[]
    cmd=[0.0,0.0]
    def on_cmd(msg):
        cmd[:]=[msg.linear.x,msg.angular.z]
        commands.append((time.monotonic(),*cmd))
    sub=node.create_subscription(Twist,'/cmd_vel',on_cmd,10)
    share=Path(get_package_share_directory('finenav_example'))
    assets=share/'create_custom_neupan_planner'
    log=open('/tmp/neupan_engine_smoke.log','w')
    proc=subprocess.Popen([str(Path(get_package_prefix('finenav_example'))/'lib/finenav_example/hello_finenav'),
        '--ros-args','--params-file',str(share/'getting_started/params/sim_localizer.yaml'),
        '--params-file',str(assets/'neupan_sim.yaml')],stdout=log,stderr=subprocess.STDOUT)
    report={}
    def run(seconds, cloud=True, pose=True, obstacle=False, move=False, frame='map'):
        start=time.monotonic(); last=start
        while time.monotonic()-start<seconds:
            assert proc.poll() is None, 'Engine exited; see /tmp/neupan_engine_smoke.log'
            now=time.monotonic();dt=min(now-last,0.1);last=now
            if move:
                state[0]+=cmd[0]*math.cos(state[2])*dt
                state[1]+=cmd[0]*math.sin(state[2])*dt
                state[2]+=cmd[1]*dt
            stamp=node.get_clock().now().to_msg()
            if pose:
                p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map'
                p.pose.pose.position.x=state[0];p.pose.pose.position.y=state[1]
                p.pose.pose.orientation.z=math.sin(state[2]/2);p.pose.pose.orientation.w=math.cos(state[2]/2)
                pose_pub.publish(p)
            if cloud:
                c=PointCloud2();c.header.stamp=stamp;c.header.frame_id=frame
                pts=[(state[0]+4.0, state[1]+3.0,0.2)]
                if obstacle: pts.append((state[0]+0.25*math.cos(state[2]),state[1]+0.25*math.sin(state[2]),0.2))
                c.height=1;c.width=len(pts);c.point_step=12;c.row_step=12*c.width
                c.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
                c.data=b''.join(struct.pack('<fff',*pt) for pt in pts)
                cloud_pub.publish(c)
            rclpy.spin_once(node,timeout_sec=0.02)
            time.sleep(0.02)
    def stopped(label):
        recent=[x for x in commands if x[0]>time.monotonic()-0.5]
        assert recent and all(abs(x[1])<1e-6 and abs(x[2])<1e-6 for x in recent), (label,recent)
        report[label]='pass'
    def moving(label):
        recent=[x for x in commands if x[0]>time.monotonic()-1.0]
        assert any(abs(x[1])>0.005 for x in recent), (label,recent)
        assert all(abs(x[1])<=0.30001 and abs(x[2])<=0.60001 for x in recent)
        report[label]='pass'
    try:
        run(2,cloud=False)
        goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=node.get_clock().now().to_msg()
        goal.pose.position.x=1.0;goal.pose.orientation.z=math.sin(0.5);goal.pose.orientation.w=math.cos(0.5)
        goal_pub.publish(goal)
        run(1.5,cloud=False);stopped('no_cloud_stop')
        run(2);moving('open_space_motion')
        run(1.5,obstacle=True);stopped('near_obstacle_stop')
        run(2);moving('obstacle_removed_resume')
        run(1.5,cloud=False);stopped('stale_cloud_stop')
        run(2);moving('cloud_resumed')
        run(1.5,pose=False);stopped('stale_pose_stop')
        run(2);moving('pose_resumed')
        run(1.5,frame='wrong_frame');stopped('wrong_cloud_frame_stop')
        run(18,move=True)
        dist=math.hypot(state[0]-1,state[1]);angle=abs(math.atan2(math.sin(state[2]-1),math.cos(state[2]-1)))
        assert dist<0.09 and angle<0.09, (state,dist,angle)
        assert abs(cmd[0])<1e-6 and abs(cmd[1])<1e-6,cmd
        report['closed_loop_goal_and_yaw']='pass';report['final_pose']=state
        report['xy_error_m']=dist;report['yaw_error_rad']=angle
    finally:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
        report['engine_shutdown_returncode']=proc.returncode
        log.close();node.destroy_node();rclpy.shutdown()
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__': main()
