from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pcl_filter_project'),
        'config',
        'params.yaml'
        )
        
    return LaunchDescription([
        Node(
            package='pcl_filter_project',
            executable='pointcloud_filter_node',
            parameters=[config],
            name='pointcloud_filter_node',
            output='screen',
        ),
    ])
