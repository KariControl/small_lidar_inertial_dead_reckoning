import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    fix_topic = LaunchConfiguration('fix_topic')
    fix_velocity_topic = LaunchConfiguration('fix_velocity_topic')
    global_pose_topic = LaunchConfiguration('global_pose_topic')
    gnss_odom_topic = LaunchConfiguration('gnss_odom_topic')
    gnss_fusion_input_topic = LaunchConfiguration('gnss_fusion_input_topic')
    antenna_frame_id = LaunchConfiguration('antenna_frame_id')
    initial_heading_deg = LaunchConfiguration('initial_heading_deg')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory('pure_gnss_conversion'),
                    'param',
                    'param.yaml',
                )
            )
        ],
        description='pure_gnss_conversion parameter file path',
    )

    declare_fix_topic = DeclareLaunchArgument(
        'fix_topic',
        default_value='/ublox_gps_node/fix',
        description='NavSatFix topic',
    )

    declare_fix_velocity_topic = DeclareLaunchArgument(
        'fix_velocity_topic',
        default_value='/ublox_gps_node/fix_velocity',
        description='GNSS Doppler velocity topic',
    )

    declare_global_pose_topic = DeclareLaunchArgument(
        'global_pose_topic',
        default_value='/localization/global_pose',
        description='Projected GNSS pose output topic',
    )

    declare_gnss_odom_topic = DeclareLaunchArgument(
        'gnss_odom_topic',
        default_value='/localization/gnss_odometry',
        description='Projected GNSS odometry output topic',
    )

    declare_gnss_fusion_input_topic = DeclareLaunchArgument(
        'gnss_fusion_input_topic',
        default_value='/localization/gnss_fusion_input',
        description='Dedicated GNSS input topic for map/odom fusion',
    )

    declare_antenna_frame_id = DeclareLaunchArgument(
        'antenna_frame_id',
        default_value='gnss_antenna',
        description='GNSS antenna frame id used for TF-based lever-arm correction',
    )

    declare_initial_heading_deg = DeclareLaunchArgument(
        'initial_heading_deg',
        default_value='0.0',
        description='Initial GNSS heading [deg] used until a valid Doppler heading is available',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    node = Node(
        package='pure_gnss_conversion',
        executable='pure_gnss_conversion',
        name='gnss_conversion',
        parameters=[params_file, {'antenna_frame_id': antenna_frame_id, 'initial_heading_deg': initial_heading_deg, 'use_sim_time': use_sim_time}],
        remappings=[
            ('fix', fix_topic),
            ('fix_velocity', fix_velocity_topic),
            ('global_pose', global_pose_topic),
            ('gnss_odometry', gnss_odom_topic),
            ('gnss_fusion_input', gnss_fusion_input_topic),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_params_file,
        declare_fix_topic,
        declare_fix_velocity_topic,
        declare_global_pose_topic,
        declare_gnss_odom_topic,
        declare_gnss_fusion_input_topic,
        declare_antenna_frame_id,
        declare_initial_heading_deg,
        declare_use_sim_time,
        node,
    ])
