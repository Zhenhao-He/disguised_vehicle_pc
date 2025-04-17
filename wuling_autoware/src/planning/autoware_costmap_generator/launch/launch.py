from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Correctly construct the parameter file path
    param_file_path = PathJoinSubstitution([
        FindPackageShare('autoware_costmap_generator'),
        'config',
        'costmap_generator.param.yaml'
    ])

    # Define the node with remappings and parameters
    costmap_generator_node = Node(
        package='autoware_costmap_generator',
        executable='costmap_generator',
        namespace="planning",
        name='costmap_generator',
        output='screen',
        remappings=[
            ('~/input/objects', '/perception/object_recognition/objects'),
            ('~/input/points_no_ground', '/perception/obstacle_segmentation/pointcloud'),
            ('~/input/vector_map', '/map/vector_map'),
            ('~/input/scenario', '/planning/scenario_planning/scenario'),
            
            ('~/output/grid_map', '/planning/grid_map'),
            ('~/output/occupancy_grid', '/planning/occupancy_grid'),
        ],
        parameters=[param_file_path],
    )

    # Return the LaunchDescription
    return LaunchDescription([
        costmap_generator_node,
    ])
