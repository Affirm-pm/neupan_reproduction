test_bag3 — FineNavLocalizer bag-replay test data (extreme / stress)
=====================================================================

Bag info
--------
  Duration  : 82.2 s
  Start     : 2026-05-08 16:03:10
  Messages  : 5741
  Size      : 1.5 MiB
  Storage   : sqlite3 (rosbag2 v5)

Topics
------
  /tf_static           tf2_msgs/msg/TFMessage       2 msgs
  /tf                  tf2_msgs/msg/TFMessage    4897 msgs (~60 Hz)
  /localization_pose   geometry_msgs/msg/PoseStamped  26 msgs (~0.32 Hz)
  /Odometry            nav_msgs/msg/Odometry      816 msgs (~10 Hz)

Data sources
------------
  /Odometry            — 10 Hz body-frame odometry from FAST-LIO.
  /localization_pose   — ~0.32 Hz absolute pose from GICP relocalisation.
                         Extremely sparse: most tracking relies on odometry
                         dead-reckoning.
  /tf                  — map -> base_link transform at ~60 Hz, used as
                         ground truth reference.

Known stress events
-------------------
  1. Dual-sensor outage (t~44.7-45.9 s)
     Odometry drops for ~1.2 s inside a longer Pose gap (4.8 s, t~42.3-47.1).
     Filter dead-reckons on stale velocity -- covariance inflates.

  2. Extended Pose-only gap (t~54.2-63.8 s, ~9.5 s)
     Single longest GICP outage.  Odometry runs normally throughout but
     without an absolute reference the filter gradually drifts.

  3. Dual-sensor outage (t~68.4-70.5 s)
     Odometry drops for ~2.1 s inside another Pose gap (4.8 s, t~66.2-71.0).
     Severe: both sensors offline simultaneously for over 2 seconds.

  4. GICP poses diverge from TF ground truth (t~66-74 s)
     t~66.2 s : GICP yaw ~ -0.30 rad, TF yaw ~ -0.61 rad   (delta ~ 18 deg)
     t~71.0 s : GICP yaw ~  1.54 rad, TF yaw ~ -1.51 rad   (delta ~175 deg -- nearly pi off)
     t~73.4 s : GICP yaw ~  1.54 rad, TF yaw ~ -1.95 rad   (delta ~160 deg -- still pi-off)
     GICP likely suffers a tracking failure (pi-ambiguity) during this window.
