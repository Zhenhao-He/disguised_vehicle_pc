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
        # 发布全景拼接图片
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('image_publisher'), 'launch', 'launch.py'
                )
            ),
        ),

        # 停车位检测
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('parkingslot_detection'), 'launch', 'launch.py'
                )
            ),
        ),

    ])
