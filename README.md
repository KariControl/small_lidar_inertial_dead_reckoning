# Pure odometry subset

This workspace contains only the packages required to run the IMU undistorter and the scan-to-scan lidar/gyro odometer extracted from `src48`.

Included packages:
- `pure_imu_undistortion`
- `pure_lidar_gyro_odometer`
- `pure_odometry_bringup`
- `small_gicp`
- `ndt_omp`

## What changed in `pure_lidar_gyro_odometer`

`updateMiniSmootherLocked()` now has two optional factors:
- `lidar_odom.smoother.zupt.*`: zero-velocity update (ZUPT) factor applied only when the interval is classified as stationary.
- `lidar_odom.smoother.nhc.*`: midpoint non-holonomic constraint (NHC) on lateral motion, with a Huber-style robust scale.

Both are disabled by default in `pure_lidar_gyro_odometer/param/param.yaml`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch

Composable container:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py
```

Standalone nodes:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py
```

Default parameter files:
- `pure_imu_undistortion/param/param.yaml`
- `pure_lidar_gyro_odometer/param/param.yaml`

You can override them from launch:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  imu_param:=/path/to/imu.yaml \
  odom_param:=/path/to/gyro_odom.yaml
```

## Docker

A best-effort Dockerfile is included at `docker/Dockerfile`.

```bash
docker build -t pure-odometry -f docker/Dockerfile .
docker run --rm -it --net=host pure-odometry
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/pure_odometry_ws/install/setup.bash
```

## Notes

- The default odometry backend is `SMALL_GICP`.
- `wheel_speed.use=true` still disables the lidar odometry path in the current node implementation.
- This bundle was prepared by extraction and code editing only; it was not compiled in this environment because ROS 2 is not installed here.
