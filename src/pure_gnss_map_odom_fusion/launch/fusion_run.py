import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    gnss_input_topic = LaunchConfiguration('gnss_input_topic')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory('pure_gnss_map_odom_fusion'),
                    'param',
                    'param.yaml',
                )
            )
        ],
        description='pure_gnss_map_odom_fusion parameter file path',
    )

    declare_gnss_input_topic = DeclareLaunchArgument(
        'gnss_input_topic',
        default_value='/localization/gnss_fusion_input',
        description='Dedicated GNSS fusion input topic',
    )

    node = Node(
        package='pure_gnss_map_odom_fusion',
        executable='pure_gnss_map_odom_fusion_node',
        name='gnss_map_odom_fusion',
        parameters=[params_file, {'gnss_input_topic': gnss_input_topic}],
        output='screen',
    )

    return LaunchDescription([
        declare_params_file,
        declare_gnss_input_topic,
        node,
    ])
