from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rm_serial = Node(
        package="my_serial_py",
        executable="serial_node",
        name="serial",
        output='screen',
        # arguments=['--ros-args', '--log-level', 'debug']
    )
    return LaunchDescription(
        [
            rm_serial
        ]
    )