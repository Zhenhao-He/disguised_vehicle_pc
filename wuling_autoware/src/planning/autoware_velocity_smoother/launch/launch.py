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
    common_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_velocity_smoother'),
        'config',
        'default_common.param.yaml'
    ])
    nearest_search_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_velocity_smoother'),
        'config',
        'nearest_search.param.yaml'
    ])
    param_path = PathJoinSubstitution([
        FindPackageShare('autoware_velocity_smoother'),
        'config',
        'default_velocity_smoother.param.yaml'
    ])
    # Define the algorithm type variable
    algorithm_type = 'JerkFiltered'
                    # option: JerkFiltered
                    #         L2
                    #         Linf(Unstable)
                    #         Analytical
    velocity_smoother_param_path = PathJoinSubstitution([
        FindPackageShare('autoware_velocity_smoother'),
        'config',
        f'{algorithm_type}.param.yaml'  # Use the algorithm_type variable here
    ])

    remappings=[
        ('~/input/trajectory', '/planning/parking/trajectory'),
        ('~/input/odometry', '/localization/kinematic_state'),

        ('~/debug/processing_time_detail_ms', '/planning/motion_velocity_smoother/processing_time_detail_ms'),
        ('~/output/trajectory', '/planning/motion_velocity_smoother/trajectory'),
        ]
    # Define the node with remappings and parameters
    motion_velocity_smoother_node = Node(
        package='autoware_velocity_smoother',
        executable='velocity_smoother_node',
        namespace="planning",
        name='motion_velocity_smoother',
        output='screen',
        remappings=remappings,
        parameters=[{'algorithm_type': algorithm_type},
                    vehicle_info_param_file_path,
                    common_param_path, 
                    nearest_search_param_path,
                    param_path,
                    velocity_smoother_param_path,
                    ],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        motion_velocity_smoother_node,
    ])
