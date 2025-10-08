from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ouster_http_node',
            executable='ouster_http_node_exe',
            name='ouster_http_node',
            parameters=[{'url': 'http://169.254.50.111/api/v1/time'}],
            output='screen'
        )
    ])
