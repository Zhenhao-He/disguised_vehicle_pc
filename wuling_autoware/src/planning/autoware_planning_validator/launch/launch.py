from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_info_param_file_path = PathJoinSubstitution([
        FindPackageShare('autoware_vehicle_info_utils'),
        'config',
        'vehicle_info.param.yaml'
    ])
    
    planning_validator_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_planning_validator'),
        'config',
        'planning_validator.param.yaml'
    ])
    remappings = [
        ("~/input/trajectory", "/planning/motion_velocity_smoother/trajectory"),
        ("~/input/kinematics", "/localization/kinematic_state"),
        ("~/input/is_completed", "/planning/is_completed"),

        ("~/output/trajectory", "/planning/scenario_planning/trajectory"),
        ("~/output/markers", "/planning/planning_validator/markers"),
        ("~/output/planning_validator/processing_time_ms", "/planning/planning_validator/processing_time_ms"),
        ("~/output/validation_status", "/planning/validation_status"),
        ("~/output/planning_state", "/system/planning_state"),###增加--用于系统验证模块是否正常
        #debug_marker
        ("~/output/marker", "/planning/planning_validator/marker"),
        ("~/output/virtual_wall", "/planning/planning_validator/virtual_wall"), 
    ]
    # Define the node with remappings and parameters
    planning_validator_node = Node(
        package='autoware_planning_validator',
        executable='planning_validator_node',
        namespace="planning",
        name='planning_validator',
        output='screen',
        remappings=remappings,
        parameters=[vehicle_info_param_file_path,
                    planning_validator_param_path,
                    ],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        planning_validator_node,
    ])