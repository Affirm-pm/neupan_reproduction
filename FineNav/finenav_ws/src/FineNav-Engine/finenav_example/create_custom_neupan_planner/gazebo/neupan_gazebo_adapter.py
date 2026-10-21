#!/usr/bin/python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Convert the original simulator's base_link cloud and TF to NeuPAN inputs."""
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Adapter(Node):
    def __init__(self):
        super().__init__('neupan_gazebo_adapter')
        self.buffer = Buffer(cache_time=Duration(seconds=5), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pending = deque(maxlen=10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization_pose', 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, '/simulation/measured_twist', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aligned_cloud', qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command = Twist()
        self.last_cloud = self.last_odom = self.last_cmd = -math.inf
        self.create_subscription(PointCloud2, '/cloud_for_lio', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/neupan/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().error('Expected original simulator odom frame', throttle_duration_sec=5)
            return
        # Existing gazebo_odom mode publishes identity map -> odom. Account for
        # the model-origin offset including any roll/pitch before forcing 2D.
        pose = PoseWithCovarianceStamped()
        pose.header = Header(stamp=msg.header.stamp, frame_id='map')
        pose.pose = msg.pose
        offset = rotation(msg.pose.pose.orientation) @ np.array([0.0, 0.0, -0.75])
        pose.pose.pose.position.x += float(offset[0])
        pose.pose.pose.position.y += float(offset[1])
        pose.pose.pose.position.z += float(offset[2])
        self.pose_pub.publish(pose)
        velocity = TwistWithCovarianceStamped()
        velocity.header = Header(stamp=msg.header.stamp, frame_id='base_link')
        velocity.twist = msg.twist
        self.twist_pub.publish(velocity)
        self.last_odom = time.monotonic()

    def on_cloud(self, msg):
        if msg.header.frame_id != 'base_link':
            self.get_logger().error('Expected /cloud_for_lio in base_link', throttle_duration_sec=5)
            return
        self.pending.append((time.monotonic(), msg))

    def on_cmd(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.command = Twist()
            self.command.linear.x = max(-0.3, min(0.3, msg.linear.x))
            self.command.angular.z = max(-0.6, min(0.6, msg.angular.z))
            self.last_cmd = time.monotonic()
        else:
            self.last_cmd = -math.inf

    def tick(self):
        now = time.monotonic()
        while self.pending:
            received, msg = self.pending[0]
            if now-received > 0.5:
                self.pending.popleft()
                continue
            try:
                tf = self.buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except TransformException:
                break  # Wait for odometry covering this scan, never use the latest TF.
            self.pending.popleft()
            points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz = np.column_stack([points[name].reshape(-1) for name in ('x', 'y', 'z')])
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            # Reject self returns using the original robot's wheel envelope.
            xyz = xyz[(np.abs(xyz[:, 0]) > 0.36) | (np.abs(xyz[:, 1]) > 0.31) | (xyz[:, 2] > 0.65)]
            t = tf.transform.translation
            xyz = xyz @ rotation(tf.transform.rotation).T + [t.x, t.y, t.z]
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=msg.header.stamp, frame_id='map'), xyz))
            if len(xyz):
                self.last_cloud = now
        fresh = now-self.last_cmd < 0.4 and now-self.last_odom < 0.5 and now-self.last_cloud < 0.5
        self.cmd_pub.publish(self.command if fresh else Twist())


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
