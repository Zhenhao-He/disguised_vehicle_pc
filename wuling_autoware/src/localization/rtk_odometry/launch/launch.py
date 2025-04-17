from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtk_odometry',
            executable='rtk_odometry_node',
            name='rtk_odometry',
            output='screen',
            remappings=[          
                ('~/input/UTM', '/localization/UTM'),

                ('~/output/position', '/localization/position'),
                #('~/output/base_link_odometry_', '/localization/base_link_odometry_'),
                #('~/output/rtk_odometry_', '/localization/rtk_odometry_'),
    
                ]
        ),
    ])
