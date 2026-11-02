test_bag1 — FineNavLocalizer bag-replay test data (normal conditions)
======================================================================

Bag info
--------
  Duration  : 60.0 s
  Start     : 2026-07-13 16:38:32
  Messages  : 5391
  Size      : 1.4 MiB
  Storage   : sqlite3 (rosbag2 v5)

Topics
------
  /tf                  tf2_msgs/msg/TFMessage    4189 msgs (~70 Hz)
  /localization_pose   geometry_msgs/msg/PoseWithCovarianceStamped  601 msgs (~10 Hz)
  /Odometry            nav_msgs/msg/Odometry      601 msgs (~10 Hz)

Data sources
------------
  /Odometry            — 10 Hz body-frame odometry from FAST-LIO.
  /localization_pose   — 10 Hz absolute pose from GICP relocalisation.
                         The stored messages include covariance, as declared
                         in metadata.yaml; no PoseStamped conversion is required.
  /tf                  — map -> base_link transform at ~70 Hz, used as
                         ground truth reference.

