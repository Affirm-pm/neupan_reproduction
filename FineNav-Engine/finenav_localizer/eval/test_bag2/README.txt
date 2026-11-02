test_bag2 — FineNavLocalizer bag-replay test data (high-speed motion)
======================================================================

Bag info
--------
  Duration  : 57.3 s
  Messages  : 22996
  Size      : 12 MiB
  Storage   : sqlite3 (rosbag2 v5)

Topics
------
  /Odometry            nav_msgs/msg/Odometry              11470 msgs (~200 Hz)
  /tf                  tf2_msgs/msg/TFMessage              11498 msgs (~200 Hz)
  /localization_pose   geometry_msgs/msg/PoseWithCovarianceStamped   28 msgs (~0.5 Hz)

Characteristics
---------------
  — 200 Hz high-rate body-frame odometry for fast, responsive dead-reckoning.
  — Rapid motion profile: high-speed translation + rotation.
  — Sparse absolute-pose corrections from GICP (only ~28 frames over 57 s).
  — Exercises odometry integration between sparse absolute-pose updates.
  — Tests state estimation under sustained high dynamics.
