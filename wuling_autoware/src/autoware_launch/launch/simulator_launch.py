import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
def generate_launch_description():
    rviz_config=get_package_share_directory('autoware_launch')+'/rviz/our_rviz2.rviz'
    return LaunchDescription([
        # 感知系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('perception_launch'), 'launch', 'simulator_launch.py'
                )
            ),
        ),
        # 决策规划系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('planning_launch'), 'launch', 'launch.py'
                )
            ),
        ),
        # 控制系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('control_launch'), 'launch', 'launch.py'
                )
            ),
        ),
        # 虚拟动力学仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('simple_planning_simulator'), 'launch', 'launch.py'
                )
            ),
        ),
        #设置为 DRIVING 才能允许换挡
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('autoware_state'), 'launch', 'launch.py'
                )
            ),
        ),
        #Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config]),
    ])

