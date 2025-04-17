import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Corrected import
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch wgs_to_utm_mgrs_node
        Node(
            package='wgs_to_utm_mgrs',
            executable='wgs_to_utm_mgrs_node',
            name='wgs_to_utm_mgrs',
            output='screen',
            remappings=[          
                ('~/input/chcnav/devpvt', '/localization/chcnav/devpvt'),

                ('~/output/UTM', '/localization/UTM'),
                ('~/output/MGRS', '/localization/MGRS'),
    
                ]
        ),
    ])
