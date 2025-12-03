std_msgs/Header header
uint32 valid_fields      # 掩码: 1=POSE, 2=VEL
geometry_msgs/Pose[] poses
geometry_msgs/Twist[] twists
geometry_msgs/Accel[] accels