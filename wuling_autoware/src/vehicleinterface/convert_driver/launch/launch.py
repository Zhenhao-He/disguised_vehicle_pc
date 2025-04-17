from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='convert_driver',
            executable='convert_driver_node',
            namespace="vehicleInterface",
            name='convert_driver',
            output='screen',
            remappings=[
            ('~/input/command/control_cmd', '/control/command/control_cmd'),
            ('~/input/command/gear_cmd', '/control/command/gear_cmd'),

            ('~/output/steer_state', '/vehicleInterface/steer_state'),
            ('~/output/gear_state', '/vehicleInterface/gear_state'),
            ('~/output/acceleration_state', '/vehicleInterface/acceleration_state'),

            #以下发布给定位模块
            ('~/output/velocity_state', '/vehicleInterface/velocity_state'),

            ('~/output/driven_left_velocity', '/vehicleInterface/driven_left_velocity'),
            ('~/output/driven_right_velocity', '/vehicleInterface/driven_right_velocity'),
            ('~/output/no_driven_left_velocity', '/vehicleInterface/no_driven_left_velocity'),
            ('~/output/no_driven_right_velocity', '/vehicleInterface/no_driven_right_velocity'),

            ('~/output/driven_left_pulse', '/vehicleInterface/driven_left_pulse'),
            ('~/output/driven_right_pulse', '/vehicleInterface/driven_right_pulse'),
            ('~/output/no_driven_left_pulse', '/vehicleInterface/no_driven_left_pulse'),
            ('~/output/no_driven_right_pulse', '/vehicleInterface/no_driven_right_pulse'),
            ],
        )
    ])
