from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='operation_mode_manager',
            executable='operation_mode_manager_node',
            namespace="system",
            name='operation_mode_manager',
            output='screen',
            remappings=[
            ('~/output/operation_mode_state', '/system/operation_mode_state'),
            ],
        )
    ])
