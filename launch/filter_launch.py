import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('input_topic', default_value='input_point_cloud', description='Input point cloud topic'),
        DeclareLaunchArgument('output_topic', default_value='filtered_point_cloud', description='Filtered point cloud topic'),
        DeclareLaunchArgument('filter_radius', default_value='0.2', description='Filtering radius (meters)'),
        DeclareLaunchArgument('filter_intensity_threshold', default_value='1.0', description='Filtering intensity threshold'),

        # Start the PointCloudFilter node with parameters
        Node(
            package='your_package_name',
            executable='pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
            parameters=[{
                'input_topic': 'input_point_cloud',
                'output_topic': 'filtered_point_cloud',
                'filter_radius': 0.2,
                'filter_intensity_threshold': 1.0
            }],
            remappings=[('/input', 'input_point_cloud'), ('/output', 'filtered_point_cloud')]
        ),
    ])
