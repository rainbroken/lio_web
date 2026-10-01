from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory('lio_web')) / 'config' / 'web.yaml'
    with config_file.open(encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['lio_web_server']['ros__parameters']

    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value=str(defaults['pointcloud_topic'])),
        DeclareLaunchArgument('fallback_pointcloud_topic', default_value=str(defaults['fallback_pointcloud_topic'])),
        DeclareLaunchArgument('odometry_topic', default_value=str(defaults['odometry_topic'])),
        DeclareLaunchArgument('camera_topic', default_value=str(defaults['camera_topic'])),
        DeclareLaunchArgument('camera_rate_hz', default_value=str(defaults['camera_rate_hz'])),
        DeclareLaunchArgument('camera_max_width', default_value=str(defaults['camera_max_width'])),
        DeclareLaunchArgument('camera_jpeg_quality', default_value=str(defaults['camera_jpeg_quality'])),
        DeclareLaunchArgument('trajectory_max_points', default_value=str(defaults['trajectory_max_points'])),
        DeclareLaunchArgument('bind_host', default_value=str(defaults['bind_host'])),
        DeclareLaunchArgument('port', default_value=str(defaults['port'])),
        DeclareLaunchArgument('max_points', default_value=str(defaults['max_points'])),
        DeclareLaunchArgument('rate_hz', default_value=str(defaults['rate_hz'])),
        DeclareLaunchArgument('voxel_size', default_value=str(defaults['voxel_size'])),
        Node(
            package='lio_web', executable='lio_web_server', name='lio_web_server',
            output='screen', parameters=[str(config_file), {
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'fallback_pointcloud_topic': LaunchConfiguration('fallback_pointcloud_topic'),
                'odometry_topic': LaunchConfiguration('odometry_topic'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_rate_hz': ParameterValue(LaunchConfiguration('camera_rate_hz'), value_type=float),
                'camera_max_width': ParameterValue(LaunchConfiguration('camera_max_width'), value_type=int),
                'camera_jpeg_quality': ParameterValue(LaunchConfiguration('camera_jpeg_quality'), value_type=int),
                'trajectory_max_points': ParameterValue(
                    LaunchConfiguration('trajectory_max_points'), value_type=int),
                'bind_host': LaunchConfiguration('bind_host'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'max_points': ParameterValue(LaunchConfiguration('max_points'), value_type=int),
                'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
                'voxel_size': ParameterValue(LaunchConfiguration('voxel_size'), value_type=float),
            }],
        ),
    ])
