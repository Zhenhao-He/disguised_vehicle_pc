from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the parameter file paths
    control_validator_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_control_validator'),
        'config',
        'control_validator.param.yaml'
    ])
    vehicle_info_param_file_path = PathJoinSubstitution([
        FindPackageShare('autoware_vehicle_info_utils'),
        'config',
        'vehicle_info.param.yaml'
    ])
    remappings = [
        ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
        ("~/input/predicted_trajectory", "/control/lateral/predicted_trajectory"),
        ("~/input/kinematics", "/localization/kinematic_state"),
        ("~/output/validation_status", "/control/validation_status"),
        ("~/output/control_state", "/system/control_state"),
        #debug_marker
        ("~/output/marker", "/control/control_validator/marker"),
        ("~/output/virtual_wall", "/control/control_validator/virtual_wall"),
    ]
    # Define the node with remappings and parameters
    control_validator_node = Node(
        package='autoware_control_validator',
        executable='autoware_control_validator_node',
        namespace="control",
        name='control_validator',
        output='screen',
        remappings=remappings,
        parameters=[control_validator_param_path,
                    vehicle_info_param_file_path,
                    ],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        control_validator_node,
    ])