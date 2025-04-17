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
        # 生成成本地图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_costmap_generator'), 'launch', 'launch.py'
                )
            ),
        ),
        
        # 发布终点位置，相对于base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('goal_publisher'), 'launch', 'launch.py'
                )
            ),
        ),
        
        # 混合A星规划器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_freespace_planner'), 'launch', 'launch.py'
                )
            ),
        ),
        # 速度平滑器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_velocity_smoother'), 'launch', 'launch.py'
                )
            ),
        ),
        # 规划验证
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_planning_validator'), 'launch', 'launch.py'
                )
            ),
        ),
    ])
