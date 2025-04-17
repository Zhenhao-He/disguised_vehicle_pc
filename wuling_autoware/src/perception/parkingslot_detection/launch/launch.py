from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
         Node(
            package='parkingslot_detection',
            executable='parkingslot_detection',
            namespace="perception",
            name='parkingslot_detection',
            output='screen',
            remappings=[            
                ('~/input/bev_image', '/perception/bev_image'),
                
                ('~/output/parking_points_info', '/perception/parking_points_info'),
                ('~/output/parking_slot_info', '/perception/parking_slot_info'),
                ('~/output/parking_slot_image', '/perception/parking_slot_image'),
            ]
            
        )
    ])
