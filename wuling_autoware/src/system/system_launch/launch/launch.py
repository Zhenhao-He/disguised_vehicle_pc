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
        # 自动或手动模式，切换成自动模式才能下发控制指令
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('operation_mode_manager'), 'launch', 'launch.py'
                )
            ),
        ),
    ])

