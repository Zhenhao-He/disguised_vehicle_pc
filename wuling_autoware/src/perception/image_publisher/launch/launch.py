from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_publisher',
            executable='image_publisher',
            namespace="perception",
            name='image_publisher',
            output='screen',
            remappings=[                          
                ('~/output/bev_image', '/perception/bev_image'),
                ]
        ),
    ])
