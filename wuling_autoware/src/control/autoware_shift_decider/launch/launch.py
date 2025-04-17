from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the parameter file paths
    shift_decider_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_shift_decider'),
        'config',
        'shift_decider.param.yaml'
    ])

    # Define the node with remappings and parameters
    autoware_shift_decider_node = Node(
        package='autoware_shift_decider',
        executable='autoware_shift_decider',
        namespace="control",
        name='autoware_shift_decider',
        output='screen',
        remappings=[
            ('~/input/control_cmd', '/control/command/control_cmd'),
            ('~/input/autoware_state', '/system/autoware_state'),
            ('~/input/current_gear', '/vehicleInterface/gear_state'),
            
            ('~/output/gear_cmd', '/control/command/gear_cmd'),
        ],
        parameters=[shift_decider_param_path,
                    ],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        autoware_shift_decider_node,
    ])