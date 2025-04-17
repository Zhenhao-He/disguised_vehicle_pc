import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Corrected import
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch HcCgiProtocolProcessNode
        Node(
            package='chcnav',
            executable='HcCgiProtocolProcessNode',
            name='hc_topic_driver',
            output='screen',
            remappings=[          
                ('~/input/chcnav/hc_sentence', '/localization/chcnav/hc_sentence'),

                ('~/output/chcnav/devpvt', '/localization/chcnav/devpvt'),
                ('~/output/chcnav/devimu', '/localization/chcnav/devimu'),
                ]
        ),
        
        # Launch HcMsgParserLaunchNode
        Node(
            package='chcnav',
            executable='HcMsgParserLaunchNode',
            name='rs232',
            output='screen',
            parameters=[{
                'type': 'serial',
                'rate': 1000,
                'port': '/dev/ttyUSB0',
                'baudrate': 460800
            }],
            remappings=[          
                ('~/output/chcnav/nmea_sentence', '/localization/chcnav/nmea_sentence'),
                ('~/output/chcnav/hc_sentence', '/localization/chcnav/hc_sentence'),
                ]
        ),
    ])
