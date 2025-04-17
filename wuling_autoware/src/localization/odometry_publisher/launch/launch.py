from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_publisher',
            executable='odometry_publisher_node',
            name='odometry_publisher',
            output='screen',
            remappings=[          
                ('~/input/position', '/localization/position'),
                ('~/input/velocity_state', '/vehicleInterface/velocity_state'),

                ('~/output/kinematic_state', '/localization/kinematic_state'),
    
                ]
        ),
    ])
