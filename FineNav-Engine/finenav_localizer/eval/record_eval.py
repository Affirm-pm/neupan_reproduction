"""Subscribe /finenav_localizer/nav_state + /tf + /Odometry → CSV."""

import csv, math, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage


def q_mul(a, b):
    """quaternion multiply (x,y,z,w) * (x,y,z,w) → (x,y,z,w)."""
    return (a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1],
            a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0],
            a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3],
            a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2])


def q_rot(q, v):
    """rotate vector v by quaternion q. q=(x,y,z,w), v=(x,y,z)."""
    qv = (v[0], v[1], v[2], 0)
    qi = (-q[0], -q[1], -q[2], q[3])  # conjugate
    r = q_mul(q_mul(q, qv), qi)
    return (r[0], r[1], r[2])


def yaw(q):
    return math.atan2(2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))


class Recorder(Node):
    def __init__(self):
        super().__init__("record_eval")
        path = self.declare_parameter("output_csv", "/tmp/localizer_eval.csv").value
        self._f = open(path, "w")
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "gt_x", "gt_y", "gt_yaw",
                          "est_x", "est_y", "est_z", "est_yaw",
                          "est_vx", "est_vy", "est_vz", "est_wz",
                          "odom_x", "odom_y", "odom_z", "odom_yaw",
                          "odom_vx", "odom_vy", "odom_vz", "odom_wz"])
        nav_topic = self.declare_parameter("nav_topic", "~/nav_state").value
        self.create_subscription(Odometry, nav_topic, self._cb, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.create_subscription(Odometry, "/Odometry", self._on_odom, 10)

        pose_topic = self.declare_parameter("pose_topic", "localization_pose").value
        if pose_topic:
            self.create_subscription(PoseWithCovarianceStamped,
                                     pose_topic, self._on_pose, 10)

        self._gt = None
        self._odom = None  # (x, y, z, yaw, vx, vy, vz, wz)
        self._pose_obs = []  # (t, x, y, z, yaw)

    def _on_tf(self, msg):
        for tx in msg.transforms:
            if tx.header.frame_id == "map" and tx.child_frame_id == "odom":
                self._A = tx.transform
            if tx.header.frame_id == "odom" and tx.child_frame_id == "base_link":
                self._B = tx.transform
        if hasattr(self, "_A") and hasattr(self, "_B"):
            A, B = self._A, self._B
            qA = (A.rotation.x, A.rotation.y, A.rotation.z, A.rotation.w)
            qB = (B.rotation.x, B.rotation.y, B.rotation.z, B.rotation.w)
            q = q_mul(qA, qB)
            v = q_rot(qA, (B.translation.x, B.translation.y, B.translation.z))
            self._gt = (A.translation.x + v[0], A.translation.y + v[1], yaw(q))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self._odom = (p.x, p.y, p.z,
                      yaw((q.x, q.y, q.z, q.w)),
                      t.linear.x, t.linear.y, t.linear.z,
                      t.angular.z)

    def _on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._pose_obs.append((t, p.x, p.y, p.z,
                               yaw((q.x, q.y, q.z, q.w))))

    def _cb(self, msg):
        gt = (math.nan, math.nan, math.nan) if self._gt is None else self._gt
        odom = (math.nan,) * 8 if self._odom is None else self._odom
        p, t = msg.pose.pose, msg.twist.twist
        self._w.writerow([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            *gt,
            p.position.x, p.position.y, p.position.z,
            yaw((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)),
            t.linear.x, t.linear.y, t.linear.z, t.angular.z,
            *odom,
        ])


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._f.close()

    # Write accumulated pose observations to a companion CSV
    if node._pose_obs:
        pose_path = node._f.name.replace(".csv", "_pose.csv")
        with open(pose_path, "w") as fp:
            w = csv.writer(fp)
            w.writerow(["t", "x", "y", "z", "yaw"])
            w.writerows(node._pose_obs)
        print(f"[record] {len(node._pose_obs)} pose obs → {pose_path}")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except RuntimeError:
        pass


if __name__ == "__main__":
    main()
