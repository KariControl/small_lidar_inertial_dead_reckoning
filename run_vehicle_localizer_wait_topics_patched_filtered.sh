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

IMU_PARAM=$(ros2 pkg prefix pure_imu_undistortion)/share/pure_imu_undistortion/param/param_vel.yaml
ODOM_PARAM=$(ros2 pkg prefix pure_lidar_gyro_odometer)/share/pure_lidar_gyro_odometer/param/param_vel.yaml
GNSS_FUSION_PARAM=$(ros2 pkg prefix pure_gnss_map_odom_fusion)/share/pure_gnss_map_odom_fusion/param/param.yaml

# === Static TFs (sensor extrinsics) ===
gnome-terminal -- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link velodyne
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z -0.1048 \
  --roll 3.14159 --pitch 0 --yaw 0.0 \
  --frame-id base_link --child-frame-id imu

gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x -1.98 --y 0.79 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gnss_antenna

# === Launch localization stack ===
gnome-terminal -- ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=true use_sim_time:=true \
  imu_param:=${IMU_PARAM} \
  odom_param:=${ODOM_PARAM} \
  gnss_fusion_param:=${GNSS_FUSION_PARAM}
sleep 8s

# === Play rosbag first ===
gnome-terminal -- ros2 run rosbag2_transport player --ros-args \
  -p storage.uri:=rosbag/rosbag2_2025_02_20-16_06_35_fukushi_back \
  -p storage.storage_id:=mcap \
  -p play.rate:=1.0 \
  -p play.start_paused:=false \
  -p play.clock_publish_frequency:=40.0 \
  -p play.playback_duration.sec:=176 \
  -p play.playback_duration.nsec:=0 \
  --remap /velodyne_points:=/converted_points \
  --remap /imu/data_raw:=/imu

wait_for_topic_once /localization/points_undistorted --qos-profile sensor_data
wait_for_topic_once /localization/gyro_lidar_odom

gnome-terminal -- ros2 bag record \
  /localization/gyro_lidar_odom \
  /localization/gyro_lidar_odom_filtered \
  /localization/lidar_degenerate/debug \
  /localization/ekf_odom \
  /localization/gnss_odometry \
  /localization/gnss_fusion_input

QT_QPA_PLATFORM=xcb ros2 run rqt_robot_monitor rqt_robot_monitor --force-discover
