from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
         package = 'chcnav',
         executable = 'HcCgiProtocolProcessNode',
         name = 'hc_topic_driver',
         output = 'screen',
      ),
      Node(
         package = 'chcnav',
         executable = 'HcMsgParserLaunchNode',
         name = 'tcp_7532',
         output = 'screen',
         parameters =[
            {"type": "tcp"},
            #节点每秒解析最大协议数量
            {"rate": 1000},
            #ip 地址
            {"host": "192.168.200.1"},
            #端口号
            {"port": 7532},
         ]
      ),
      Node(
         package = 'chcnav',
         executable = 'TimeUniformityNode',
         name = 'time_uniformity_node',
         output = 'screen',
      ),
   ])

