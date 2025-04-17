from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            remappings=[          
                ('~/input/driven_left_velocity', '/vehicleInterface/driven_left_velocity'),
                ('~/input/driven_right_velocity', '/vehicleInterface/driven_right_velocity'),
                ('~/input/driven_left_pulse', '/vehicleInterface/driven_left_pulse'),
                ('~/input/driven_right_pulse', '/vehicleInterface/driven_right_pulse'),

                ('~/output/position', '/localization/position'),
    
                ]
        ),
    ])
