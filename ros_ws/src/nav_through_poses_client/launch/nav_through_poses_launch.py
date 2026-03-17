from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_through_poses_client',
            executable='nav_through_poses_node',
            name='nav_through_poses_client',
            output='screen',
            parameters=[],
        ),
    ])
