from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='colibus_simulator',
            executable='vehicle_control_node',
            output='screen'
        )
    ])
