from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("pure_imu_undistortion")
    params = os.path.join(pkg, "param", "param.yaml")

    return LaunchDescription([
        Node(
            package="pure_imu_undistortion",
            executable="imu_undistorter_node",
            name="imu_undistorter",
            output="screen",
            parameters=[params],
            remappings=[
                ("diagnostics", "/localization/deskew_diagnostics"),
            ],
        )
    ])
