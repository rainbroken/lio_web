from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value='/lio/cloud_registered'),
        DeclareLaunchArgument('bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('max_points', default_value='20000'),
        DeclareLaunchArgument('rate_hz', default_value='5.0'),
        DeclareLaunchArgument('voxel_size', default_value='0.15'),
        Node(
            package='lio_web', executable='lio_web_server', name='lio_web_server',
            output='screen', parameters=[{
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'bind_host': LaunchConfiguration('bind_host'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'max_points': ParameterValue(LaunchConfiguration('max_points'), value_type=int),
                'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
                'voxel_size': ParameterValue(LaunchConfiguration('voxel_size'), value_type=float),
            }],
        ),
    ])
