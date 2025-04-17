from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the parameter file paths
    nearest_search_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_trajectory_follower_node'),
        'config',
        'nearest_search.param.yaml'
    ])

    trajectory_follower_node_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_trajectory_follower_node'),
        'config',
        'trajectory_follower_node.param.yaml'
    ])

    lon_controller_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_trajectory_follower_node'),
        'config',
        'pid.param.yaml'
    ])

    lat_controller_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_trajectory_follower_node'),
        'config',
        'mpc.param.yaml'
    ])

    vehicle_param_file = PathJoinSubstitution([
        FindPackageShare('autoware_trajectory_follower_node'),
        'config',
        'vehicle_info.param.yaml'
    ])
    # Define the node with remappings and parameters
    trajectory_follower_node = Node(
        package='autoware_trajectory_follower_node',
        executable='controller_node_exe',
        namespace="control",
        name='trajectory_follower',
        output='screen',
        remappings=[
            ('~/input/reference_trajectory', '/planning/scenario_planning/trajectory'),
            ('~/input/current_odometry', '/localization/kinematic_state'),
            ('~/input/current_accel', '/vehicleInterface/acceleration_state'),
            ('~/input/current_steering', '/vehicleInterface/steer_state'),
            ('~/input/current_operation_mode', '/system/operation_mode/state'),

            ('~/output/control_cmd', '/control/command/control_cmd'),
            ('~/output/lateral/processing_time_ms', '/control/lateral/processing_time_ms'),
            ('~/output/longitudinal/processing_time_ms', '/control/longitudinal/processing_time_ms'),
            ('~/output/debug_marker', '/control/trajectory_follower/debug_marker'),
            #位于mpc
            ('~/output/predicted_trajectory', '/control/lateral/predicted_trajectory'),
            ('~/output/lateral_diagnostic', '/control/lateral/diagnostic'),
            ('~/output/estimated_steer_offset', '/control/lateral/estimated_steer_offset'),
            #位于pid
            ('~/output/slope_angle', '/control/longitudinal/slope_angle'),
            ('~/output/longitudinal_diagnostic', '/control/longitudinal/diagnostic'),
            ('~/output/stop_reason', '/control/longitudinal/stop_reason'),
            
        ],
        parameters=[{'lateral_controller_mode': 'mpc'},
                    {'longitudinal_controller_mode': 'pid'},
                    nearest_search_param_path, 
                    trajectory_follower_node_param_path,
                    lon_controller_param_path,
                    lat_controller_param_path,
                    vehicle_param_file],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        trajectory_follower_node,
    ])