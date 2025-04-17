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
        # 设置map和base_link坐标系之间的初始关系
        # 发布虚拟车辆实时位姿,相对于base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('simple_planning_simulator'), 'launch', 'launch.py'
                )
            ),
        ),
    ])

