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

IMU_PARAM=$(ros2 pkg prefix pure_imu_undistortion)/share/pure_imu_undistortion/param/param_xt.yaml
ODOM_PARAM=$(ros2 pkg prefix pure_lidar_gyro_odometer)/share/pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml
GNSS_FUSION_PARAM=$(ros2 pkg prefix pure_gnss_map_odom_fusion)/share/pure_gnss_map_odom_fusion/param/param.yaml

# === Static TFs (sensor extrinsics) ===
gnome-terminal -- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar/0
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x -0.100 --y 0 --z -0.0936 \
  --roll 0 --pitch 0 --yaw 0.0 \
  --frame-id base_link --child-frame-id imu

gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x -1.98 --y 0.79 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gnss_antenna

# === Launch localization stack ===
gnome-terminal -- ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=false use_sim_time:=true \
  imu_param:=${IMU_PARAM} \
  odom_param:=${ODOM_PARAM} \
  gnss_fusion_param:=${GNSS_FUSION_PARAM}
sleep 8s

# === Play rosbag first ===
gnome-terminal -- ros2 run rosbag2_transport player --ros-args \
  -p storage.uri:=rosbag/output \
  -p storage.storage_id:=mcap \
  -p play.rate:=1.0 \
  -p play.start_paused:=false \
  -p play.clock_publish_frequency:=40.0 \
  --remap /pandar_points_ex:=/converted_points \
  --remap /sensor/imu/data_raw:=/imu

wait_for_topic_once /localization/points_undistorted --qos-profile sensor_data

gnome-terminal -- ros2 bag record \
  /localization/gyro_lidar_odom \
  /localization/gyro_lidar_odom_filtered \
  /localization/lidar_degenerate \
  /localization/lidar_degeneracy_debug

QT_QPA_PLATFORM=xcb ros2 run rqt_robot_monitor rqt_robot_monitor --force-discover
