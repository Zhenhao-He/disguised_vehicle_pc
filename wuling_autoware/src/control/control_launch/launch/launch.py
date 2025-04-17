import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
def generate_launch_description():
    return LaunchDescription([
        # 横纵向控制
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_trajectory_follower_node'), 'launch', 'launch.py'
                )
            ),
        ),
        # 速度+路径偏差验证
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_control_validator'), 'launch', 'launch.py'
                )
            ),
        ),
        # 档位控制
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_shift_decider'), 'launch', 'launch.py'
                )
            ),
        ),
    ])
