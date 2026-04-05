# Dedicated GNSS input for map/odom fusion.
#
# header.stamp carries the NavSatFix sample time that should be used for
# GNSS quality gating. When has_odom=true, odom is the synchronized GNSS
# absolute pose for the same fix epoch.

std_msgs/Header header
bool has_odom
nav_msgs/Odometry odom
int8 fix_status
