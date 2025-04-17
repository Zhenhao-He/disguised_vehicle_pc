from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_publisher',  # 功能包名称
            executable='goal_publisher_node',  # 可执行文件名称
            namespace="planning",
            name='goal_publisher',  # 节点名称
            output='screen',  # 输出到屏幕
            remappings=[
                ('~/input/UTM', '/localization/UTM'),

                ('~/output/goal_pose', '/planning/goal_pose'),      
            ]
        )
    ])
