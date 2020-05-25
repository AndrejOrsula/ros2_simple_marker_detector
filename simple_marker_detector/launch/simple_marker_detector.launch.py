"""Launch pupil centre localisation"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    config_simple_marker_detector = LaunchConfiguration('config_simple_marker_detector', default=os.path.join(get_package_share_directory(
        'simple_marker_detector'), 'config', 'simple_marker_detector.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_simple_marker_detector',
            default_value=config_simple_marker_detector,
            description='Path to config for pupil centre localisation'),

        Node(
            package='simple_marker_detector',
            node_executable='simple_marker_detector',
            node_name='simple_marker_detector',
            node_namespace='',
            output='screen',
            parameters=[config_simple_marker_detector],
            remappings=[('camera/color/image_raw', 'camera/color/image_raw'),
                        ('camera/aligned_depth_to_color/image_raw',
                         'camera/aligned_depth_to_color/image_raw'),
                        ('camera/aligned_depth_to_color/camera_info',
                         'camera/aligned_depth_to_color/camera_info'),
                        ('marker_centre', 'marker_centre')],
        ),
    ])
