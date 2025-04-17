from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define the parameter file paths
    param_file_path = PathJoinSubstitution([
        FindPackageShare('autoware_freespace_planner'),
        'config',
        'freespace_planner.param.yaml'
    ])

    vehicle_info_param_file_path = PathJoinSubstitution([
        FindPackageShare('autoware_vehicle_info_utils'),
        'config',
        'vehicle_info.param.yaml'
    ])

    # Define the node with remappings and parameters
    freespace_planner_node = Node(
        package='autoware_freespace_planner',
        executable='freespace_planner',
        namespace="planning",
        name='freespace_planner',
        output='screen',
        remappings=[
            ('~/input/goal_pose', '/planning/goal_pose'),
            ('~/input/occupancy_grid', '/planning/occupancy_grid'),
            ('~/input/odometry', '/localization/kinematic_state'),

            ('~/output/trajectory', '/planning/parking/trajectory'),
            ('~/output/trajectory_array', '/planning/trajectory_array'),
            ('~/output/partial_trajectory_array', '/planning/partial_trajectory_array'),
            ('~/output/is_completed', '/planning/is_completed'),
        ],
        parameters=[param_file_path, vehicle_info_param_file_path],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        freespace_planner_node,
    ])
