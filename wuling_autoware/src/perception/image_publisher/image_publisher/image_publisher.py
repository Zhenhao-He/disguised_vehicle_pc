import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import re
import glob
import ament_index_python.packages

package_name = 'image_publisher'

# 获取功能包路径
ROOT_path = ament_index_python.packages.get_package_share_directory(package_name)

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # 创建图像发布者，发布到 'image_topic' 话题
        self.publisher = self.create_publisher(Image, '~/output/bev_image', 10)

        # 创建定时器，以10Hz的频率发布图像
        self.timer = self.create_timer(0.1, self.publish_image)

        # 初始化 CvBridge，用于将 OpenCV 图像转换为 ROS 2 图像消息
        self.bridge = CvBridge()

        # 设置图片文件夹路径
        self.image_folder = os.path.join(ROOT_path, 'parking_test_images')
        self.image_paths = glob.glob(os.path.join(self.image_folder, '*.jpg'))

        # 根据文件名中的数字部分排序
        self.image_paths.sort(key=self.extract_number)

        # 检查是否有图片
        if not self.image_paths:
            self.get_logger().error(f"No images found in folder: {self.image_folder}")
            self.image_paths = []

        # 当前发布图片的索引
        self.current_index = 0

    def extract_number(self, file_path):
        """从文件名中提取数字部分"""
        file_name = os.path.basename(file_path)
        match = re.search(r'img_(\d+)', file_name)
        if match:
            return int(match.group(1))  # 提取数字并转换为整数
        return 0  # 如果未匹配到数字，则返回 0 作为默认值

    def publish_image(self):
        if not self.image_paths:
            return  # 如果没有图片，退出

        # 加载当前索引对应的图片
        image_path = self.image_paths[self.current_index]
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        # 将 OpenCV 图像转换为 ROS 图像消息
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            # 发布图像
            self.publisher.publish(ros_image)
            self.get_logger().info(f'Publishing image: {image_path}')
        except Exception as e:
            self.get_logger().error(f"Error in publishing image: {e}")

        # 更新索引，循环发送
        self.current_index = (self.current_index + 1) % len(self.image_paths)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()