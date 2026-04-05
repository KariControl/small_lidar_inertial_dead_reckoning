from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_imu_param = os.path.join(
        get_package_share_directory("pure_imu_undistortion"), "param", "param.yaml"
    )
    default_odom_param = os.path.join(
        get_package_share_directory("pure_lidar_gyro_odometer"), "param", "param.yaml"
    )
    default_gnss_param = os.path.join(
        get_package_share_directory("pure_gnss_conversion"), "param", "param.yaml"
    )
    default_gnss_fusion_param = os.path.join(
        get_package_share_directory("pure_gnss_map_odom_fusion"), "param", "param.yaml"
    )
    diag_agg_param = os.path.join(
        get_package_share_directory("pure_odometry_bringup"), "config", "diagnostic_aggregator.yaml"
    )

    imu_param = LaunchConfiguration("imu_param")
    odom_param = LaunchConfiguration("odom_param")
    gnss_param = LaunchConfiguration("gnss_param")
    gnss_fusion_param = LaunchConfiguration("gnss_fusion_param")

    use_gnss = LaunchConfiguration("use_gnss")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    gnss_fix_topic = LaunchConfiguration("gnss_fix_topic")
    gnss_fix_velocity_topic = LaunchConfiguration("gnss_fix_velocity_topic")
    gnss_global_pose_topic = LaunchConfiguration("gnss_global_pose_topic")
    gnss_odom_topic = LaunchConfiguration("gnss_odom_topic")
    gnss_fusion_input_topic = LaunchConfiguration("gnss_fusion_input_topic")

    imu_undistorter = ComposableNode(
        package="pure_imu_undistortion",
        plugin="pure_imu_undistortion::ImuUndistorter",
        name="imu_undistorter",
        parameters=[imu_param, {"use_sim_time": use_sim_time}],
    )

    gyro_odometer = ComposableNode(
        package="pure_lidar_gyro_odometer",
        plugin="pure_gyro_odometer::GyroOdometerNode",
        name="gyro_odometer",
        parameters=[odom_param, {"use_sim_time": use_sim_time}],
    )

    container = ComposableNodeContainer(
        name="pure_odometry_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[imu_undistorter, gyro_odometer],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    gnss_conversion = Node(
        condition=IfCondition(use_gnss),
        package="pure_gnss_conversion",
        executable="pure_gnss_conversion",
        name="gnss_conversion",
        output="screen",
        parameters=[gnss_param, {"use_sim_time": use_sim_time}],
        remappings=[
            ("fix", gnss_fix_topic),
            ("fix_velocity", gnss_fix_velocity_topic),
            ("global_pose", gnss_global_pose_topic),
            ("gnss_odometry", gnss_odom_topic),
            ("gnss_fusion_input", gnss_fusion_input_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    gnss_map_odom_fusion = Node(
        condition=IfCondition(use_gnss),
        package="pure_gnss_map_odom_fusion",
        executable="pure_gnss_map_odom_fusion_node",
        name="gnss_map_odom_fusion",
        output="screen",
        parameters=[
            gnss_fusion_param,
            {
                "use_sim_time": use_sim_time,
                "gnss_input_topic": gnss_fusion_input_topic,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[diag_agg_param, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument("imu_param", default_value=default_imu_param),
        DeclareLaunchArgument("odom_param", default_value=default_odom_param),
        DeclareLaunchArgument("gnss_param", default_value=default_gnss_param),
        DeclareLaunchArgument("gnss_fusion_param", default_value=default_gnss_fusion_param),
        DeclareLaunchArgument("use_gnss", default_value="false"),
        DeclareLaunchArgument("gnss_fix_topic", default_value="/ublox_gps_node/fix"),
        DeclareLaunchArgument("gnss_fix_velocity_topic", default_value="/ublox_gps_node/fix_velocity"),
        DeclareLaunchArgument("gnss_global_pose_topic", default_value="/localization/global_pose"),
        DeclareLaunchArgument("gnss_odom_topic", default_value="/localization/gnss_odometry"),
        DeclareLaunchArgument("gnss_fusion_input_topic", default_value="/localization/gnss_fusion_input"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        container,
        gnss_conversion,
        gnss_map_odom_fusion,
        diagnostic_aggregator_node,
    ])
