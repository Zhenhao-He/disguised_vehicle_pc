from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoware_state',
            executable='autoware_state_exe',
            namespace="system",
            name='autoware_state',
            output='screen',
            remappings=[
            ('~/input/localization_state', '/system/localization_state'),
            ('~/input/planning_state', '/system/planning_state'),
            ('~/input/control_state', '/system/control_state'),
            ('~/input/operation_state', '/system/operation_mode_state'),

            ('~/output/autoware_state', '/system/autoware_state'),
            ],
        )
    ])
