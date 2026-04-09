source /opt/ros/jazzy/setup.bash
. install/setup.bash

wait_for_topic_once() {
  local topic="$1"
  shift || true
  local extra=("$@")
  echo "[wait] waiting for first message on ${topic} ..."
  until timeout 3s ros2 topic echo "${topic}" --once "${extra[@]}" >/dev/null 2>&1; do
    sleep 1
  done
  echo "[wait] ${topic} is available"
}

# === Static TFs (sensor extrinsics) ===
gnome-terminal -- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x -0.00 --y 0 --z -0.0 \
  --roll 0 --pitch 0 --yaw 0.0 \
  --frame-id base_link --child-frame-id imu

gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x -1.98 --y 0.79 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gnss_antenna


# === Launch localization stack ===
gnome-terminal -- ros2 launch pure_odometry_bringup odometry_container.launch.py use_gnss:=false use_sim_time:=true
sleep 8s

# === Play rosbag first ===
# gnome-terminal -- ros2 bag play rosbag/my_bag_mcap-20260122T043704Z-1-001/my_bag_mcap/ \
#   --remap /imu/imu0/data_raw/corrected:=/imu -r 1.0 --clock



gnome-terminal -- ros2 run rosbag2_transport player --ros-args \
  -p storage.uri:=rosbag/rosbag2_2026_04_08-22_52_42 \
  -p storage.storage_id:=mcap \
  -p play.rate:=1.0 \
  -p play.start_paused:=false \
  -p play.clock_publish_frequency:=40.0 \
  --remap /livox/lidar:=/converted_points \
  --remap /livox/imu:=/imu

  # -p play.playback_duration.sec:=176 \
  # -p play.playback_duration.nsec:=0 \



# Wait until the scan matcher actually has input data before triggering GI.
wait_for_topic_once /localization/points_undistorted --qos-profile sensor_data
# wait_for_topic_once /localization/gyro_lidar_odom /

# Optional: record outputs
gnome-terminal -- ros2 bag record /localization/gyro_lidar_odom /localization/lidar_degeneracy_debug

QT_QPA_PLATFORM=xcb ros2 run rqt_robot_monitor rqt_robot_monitor --force-discover
