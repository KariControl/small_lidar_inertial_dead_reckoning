source /opt/ros/jazzy/setup.bash
rm -r install/ build/
#colcon build --packages-up-to gnss_interface
#colcon build --packages-up-to motion_judge

#colcon build --packages-up-to yaw_estimator
#colcon build --packages-up-to velocity_estimator
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#colcon build --symlink-install
