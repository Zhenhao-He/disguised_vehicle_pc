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
    	#轮速计
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('wheel_odometry'), 'launch', 'launch.py'
                )
            ),
        ),
        #发布相对起点的位姿
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('odometry_publisher'), 'launch', 'launch.py'
                )
            ),
        ),
    ])

