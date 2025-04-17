import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
def generate_launch_description():
    rviz_config=get_package_share_directory('rviz_launch')+'/rviz/our_rviz2.rviz'
    return LaunchDescription([
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config]),
    ])

