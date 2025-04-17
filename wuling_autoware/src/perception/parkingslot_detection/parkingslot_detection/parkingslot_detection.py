import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node

import rclpy
from rclpy.clock import Clock
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import math
import cv2 as cv
import numpy as np
import torch
from torchvision.transforms import ToTensor
import config
import json
from data import get_predicted_points, pair_marking_points, calc_point_squre_dist, pass_through_third_point
from model import DirectionalPointDetector
from util import Timer
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode
from pathlib import Path

import glob

import time
from pathlib import Path
import ament_index_python.packages
from geometry_msgs.msg import Point
from parking_slot_detection_msgs.msg import ParkingSlotInfo,ParkingPoint, ParkingPointsList
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

package_name = 'parkingslot_detection'
M_PI=3.14159265358979323846
# 获取功能包路径
ROOT_path = ament_index_python.packages.get_package_share_directory(package_name)

def plot_points(image, pred_points):
    """Plot marking points on the image."""
    if not pred_points:
        return
    height = image.shape[0]
    width = image.shape[1]
    for index ,marking_points in enumerate(pred_points):
        for marking_point in marking_points:
            p0_x = (marking_point[..., 0] + marking_point[..., 2]) / 1280
            p0_y = (marking_point[..., 1] + marking_point[..., 3]) / 1280
            p0_x =  width * p0_x.item()
            p0_y =  height * p0_y.item()
            sin_value = marking_point[7].item()
            cos_value = marking_point[6].item()
            direction = math.atan2(sin_value, cos_value)
            cos_val = math.cos(direction)
            sin_val = math.sin(direction)  
            p1_x = p0_x + 50*cos_val
            p1_y = p0_y + 50*sin_val
            p2_x = p0_x - 50*sin_val
            p2_y = p0_y + 50*cos_val
            p3_x = p0_x + 50*sin_val
            p3_y = p0_y - 50*cos_val
            p0_x = int(round(p0_x))
            p0_y = int(round(p0_y))
            p1_x = int(round(p1_x))
            p1_y = int(round(p1_y))
            p2_x = int(round(p2_x))
            p2_y = int(round(p2_y))
            #print("p0_x:",p0_x,"p0_y:",p0_y,"p1_x:",p1_x,"p1_y:",p1_y,"direction:",direction)
            cv.line(image, (p0_x, p0_y), (p1_x, p1_y), (0, 0, 255), 2)#分隔线-角点中心忘里面指向
            cv.putText(image, str(marking_point[4].item()), (p0_x, p0_y),
                        cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))

            if marking_point[5] > 0.5:
                cv.line(image, (p0_x, p0_y), (p2_x, p2_y), (0, 0, 255), 2)
            else:
                p3_x = int(round(p3_x))
                p3_y = int(round(p3_y))
                cv.line(image, (p2_x, p2_y), (p3_x, p3_y), (0, 0, 255), 2)

def plot_slots(image, pred_points, slots):
    """Plot parking slots on the image."""
    if not pred_points or not slots:
        return
    # marking_points = list(list(zip(*pred_points))[1])
    height = image.shape[0]
    width = image.shape[1]
    car_center = np.array([width / 2, height / 2])  # 自车在图片中的中心坐标
    vehicle_forward = np.array([0, -1])  # 自车前向方向设为垂直向上
    slot_coordinates = []
    for index ,marking_points in enumerate(pred_points):
        for slot in slots:
            point_a = marking_points[slot[0]]
            point_b = marking_points[slot[1]]
            point_a_x = (point_a[..., 0] + point_a[..., 2]) / 1280
            point_a_y = (point_a[..., 1] + point_a[..., 3]) / 1280
            point_b_x = (point_b[..., 0] + point_b[..., 2]) / 1280
            point_b_y = (point_b[..., 1] + point_b[..., 3]) / 1280
            p0_x = width * point_a_x.item() - 0.5
            p0_y = height * point_a_y.item() - 0.5
            p1_x = width * point_b_x.item() - 0.5
            p1_y = height * point_b_y.item() - 0.5
            vec = np.array([p1_x - p0_x, p1_y - p0_y])
            vec = vec / np.linalg.norm(vec)
            distance = calc_point_squre_dist(point_a, point_b)
            if config.VSLOT_MIN_DIST <= distance <= config.VSLOT_MAX_DIST:
                separating_length = config.LONG_SEPARATOR_LENGTH
            elif config.HSLOT_MIN_DIST <= distance <= config.HSLOT_MAX_DIST:
                separating_length = config.SHORT_SEPARATOR_LENGTH
            p2_x = p0_x + height * separating_length * vec[1]
            p2_y = p0_y - width * separating_length * vec[0]
            p3_x = p1_x + height * separating_length * vec[1]
            p3_y = p1_y - width * separating_length * vec[0]
            p0_x = int(round(p0_x))
            p0_y = int(round(p0_y))
            p1_x = int(round(p1_x))
            p1_y = int(round(p1_y))
            p2_x = int(round(p2_x))
            p2_y = int(round(p2_y))
            p3_x = int(round(p3_x))
            p3_y = int(round(p3_y))
            # Round all points to integers for pixel coordinates
            p0 = (int(round(p0_x)), int(round(p0_y)))
            p1 = (int(round(p1_x)), int(round(p1_y)))
            p2 = (int(round(p2_x)), int(round(p2_y)))
            p3 = (int(round(p3_x)), int(round(p3_y)))
            # Calculate the center point of the slot
            center_x = int(round((p0[0] + p1[0] + p2[0] + p3[0]) / 4))
            center_y = int(round((p0[1] + p1[1] + p2[1] + p3[1]) / 4))
            center = (center_x, center_y)

            point_a_direction = math.atan2(point_a[7].item(), point_a[6].item())
            point_b_direction = math.atan2(point_b[7].item(), point_b[6].item())
            angle_deg=((point_a_direction+point_b_direction)/2)/M_PI*180

            slot_coordinates.append({
                "corners": [p0, p1, p2, p3],
                "center": center,
                "heading_angle": angle_deg  # 航向角
            })
            cv.line(image, (p0_x, p0_y), (p1_x, p1_y), (255, 0, 0), 2)
            cv.line(image, (p0_x, p0_y), (p2_x, p2_y), (255, 0, 0), 2)
            cv.line(image, (p1_x, p1_y), (p3_x, p3_y), (255, 0, 0), 2)
    return slot_coordinates

def preprocess_image(image):
    """Preprocess numpy image to torch tensor."""
    if image.shape[0] != 640 or image.shape[1] != 640:
        image = cv.resize(image, (640, 640))
    return torch.unsqueeze(ToTensor()(image), 0)


def detect_marking_points(detector, image, thresh, device):
    """Given image read from opencv, return detected marking points."""
    prediction = detector(preprocess_image(image).to(device))
    return get_predicted_points(prediction[0], thresh)



def inference_slots(pred_points):
    """Inference slots based on marking points."""
    #基于标记点推理停车位
    for index ,marking_points in enumerate(pred_points):
        num_detected = len(marking_points)
        slots = []
        for i in range(num_detected - 1):
            for j in range(i + 1, num_detected):
                point_i = marking_points[i]
                point_j = marking_points[j]
                # Step 1: length filtration.
                distance = calc_point_squre_dist(point_i, point_j)
                if not (config.VSLOT_MIN_DIST <= distance <= config.VSLOT_MAX_DIST
                        or config.HSLOT_MIN_DIST <= distance <= config.HSLOT_MAX_DIST):
                    continue
                # Step 2: pass through filtration.
                if pass_through_third_point(marking_points, i, j):
                    continue
                result = pair_marking_points(point_i, point_j)
                if result == 1:
                    slots.append((i, j))
                elif result == -1:
                    slots.append((j, i))
        return slots


class ParkingSpaceDetectionNode(Node):
    def __init__(self):
        super().__init__('image_publisher')
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # 创建图像订阅者，接收 'camera/image' 话题的图像
        self.subscription = self.create_subscription(
            Image,
            '~/input/bev_image',
            self.image_callback,
            10
        )

        self.parkingpoints_pub = self.create_publisher(ParkingPointsList, '~/output/parking_points_info', 10)
        
        self.parkingslot_pub = self.create_publisher(ParkingSlotInfo, '~/output/parking_slot_info', 10)

        self.image_publisher = self.create_publisher(Image, '~/output/parking_slot_image', 10)
        
        # 初始化 CvBridge 用于图像转换
        self.bridge = CvBridge()
        # 加载模型
        self.device = self.select_device()
        self.model = self.load_model()

    def select_device(self):
        """选择设备（CPU 或 GPU）"""
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        return device

    def load_model(self):
        """加载预训练的模型"""
        weight_path = os.path.join(ROOT_path, 'ckpt/epoch17.pt')
        model = DetectMultiBackend(weight_path, device=self.device, dnn=False, data=None, fp16=False)
        return model

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式的图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # 获取当前时间戳
            timestamp1 = time.time()

            # 进行推理检测
            marking_points, slot_coordinates, processed_image = self.inference_detector(cv_image)

            #发布角点信息
            self.publish_parking_points_info(processed_image,marking_points)
            # 发布车位信息
            self.publish_parking_slot_info(slot_coordinates)

            # 发布处理后的图像
            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.image_publisher.publish(processed_image_msg)

            #cv_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            #self.image_publisher.publish(cv_image_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def inference_detector(self, cv_image):
        """推理方向标记点检测器"""
        im = preprocess_image(cv_image).to(self.device)
        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred[0][1], conf_thres=0.25, iou_thres=0.45)
        plot_points(cv_image, pred)
        slots = inference_slots(pred)
        slot_coordinates = plot_slots(cv_image, pred, slots)
        return pred, slot_coordinates, cv_image
    def publish_parking_points_info(self,image,pred_points):
        """Plot marking points on the image."""
        if not pred_points:
            return
        height = image.shape[0]
        width = image.shape[1]
        parking_points_list = ParkingPointsList()


        # 使用Clock来获取时间
        clock = Clock()
        current_time = clock.now()  # 获取当前时间戳（已经是Time类型）
        # 创建消息header并设置时间戳
        header = Header()
        header.stamp = current_time.to_msg()  # 使用 .to_msg() 方法转换为 Time 消息类型
        parking_points_list.header = header
        for index ,marking_points in enumerate(pred_points):
            for marking_point in marking_points:
                #角点中心坐标
                p0_x = (marking_point[..., 0] + marking_point[..., 2]) / 1280
                p0_y = (marking_point[..., 1] + marking_point[..., 3]) / 1280
                p0_x =  width * p0_x.item()
                p0_y =  height * p0_y.item()
                #角点的角度
                sin_value = marking_point[7].item()
                cos_value = marking_point[6].item()
                direction = math.atan2(sin_value, cos_value)
                #置信度
                confidence=marking_point[4].item()

                # 创建ParkingPoint消息
                parking_point_msg = ParkingPoint()
                parking_point_msg.x = p0_x
                parking_point_msg.y = p0_y
                parking_point_msg.angle = direction
                parking_point_msg.confidence = confidence
                
                # 将停车点信息添加到列表
                parking_points_list.points.append(parking_point_msg)
         # 发布消息
        self.parkingpoints_pub.publish(parking_points_list)

    def publish_parking_slot_info(self, slot_coordinates):
        """发布车位信息"""
        heading_angles = []
        centers = []
        for slot in slot_coordinates:
            #print("车位信息:",slot)
            heading_angles.append(slot['heading_angle'])  # 添加方向角度
            centers.append(slot['center'])  # 添加中心点

        msg = ParkingSlotInfo()
        # 使用Clock来获取时间
        clock = Clock()
        current_time = clock.now()  # 获取当前时间戳（已经是Time类型）
        # 创建消息header并设置时间戳
        header = Header()
        header.stamp = current_time.to_msg()  # 使用 .to_msg() 方法转换为 Time 消息类型
        msg.header = header
        msg.heading_angle = heading_angles
        msg.center = [Point(x=float(center[0]), y=float(center[1]), z=0.0) for center in centers]
        self.parkingslot_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




