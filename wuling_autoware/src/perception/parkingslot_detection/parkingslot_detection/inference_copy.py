"""Inference demo of directional point detector."""
import math
import cv2 as cv
import numpy as np
import torch
from torchvision.transforms import ToTensor
import config
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
import sys
import os
import glob
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLO root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
# def plot_points(image, pred_points):
#     """Plot marking points on the image."""
#     if not pred_points:
#         return
#     height = image.shape[0]
#     width = image.shape[1]
#     for confidence, marking_point in pred_points:
#         p0_x = width * marking_point.x - 0.5
#         p0_y = height * marking_point.y - 0.5
#         cos_val = math.cos(marking_point.direction)
#         sin_val = math.sin(marking_point.direction)
#         p1_x = p0_x + 50*cos_val
#         p1_y = p0_y + 50*sin_val
#         p2_x = p0_x - 50*sin_val
#         p2_y = p0_y + 50*cos_val
#         p3_x = p0_x + 50*sin_val
#         p3_y = p0_y - 50*cos_val
#         p0_x = int(round(p0_x))
#         p0_y = int(round(p0_y))
#         p1_x = int(round(p1_x))
#         p1_y = int(round(p1_y))
#         p2_x = int(round(p2_x))
#         p2_y = int(round(p2_y))
#         cv.line(image, (p0_x, p0_y), (p1_x, p1_y), (0, 0, 255), 2)
#         cv.putText(image, str(confidence), (p0_x, p0_y),
#                    cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
#         if marking_point.shape > 0.5:
#             cv.line(image, (p0_x, p0_y), (p2_x, p2_y), (0, 0, 255), 2)
#         else:
#             p3_x = int(round(p3_x))
#             p3_y = int(round(p3_y))
#             cv.line(image, (p2_x, p2_y), (p3_x, p3_y), (0, 0, 255), 2)

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
            cv.line(image, (p0_x, p0_y), (p1_x, p1_y), (0, 0, 255), 2)
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
            cv.line(image, (p0_x, p0_y), (p1_x, p1_y), (255, 0, 0), 2)
            cv.line(image, (p0_x, p0_y), (p2_x, p2_y), (255, 0, 0), 2)
            cv.line(image, (p1_x, p1_y), (p3_x, p3_y), (255, 0, 0), 2)


def preprocess_image(image):
    """Preprocess numpy image to torch tensor."""
    if image.shape[0] != 640 or image.shape[1] != 640:
        image = cv.resize(image, (640, 640))
    return torch.unsqueeze(ToTensor()(image), 0)


def detect_marking_points(detector, image, thresh, device):
    """Given image read from opencv, return detected marking points."""
    prediction = detector(preprocess_image(image).to(device))
    return get_predicted_points(prediction[0], thresh)


# def inference_slots(marking_points):
#     """Inference slots based on marking points."""
#     #基于标记点推理停车位
#     num_detected = len(marking_points)
#     slots = []
#     for i in range(num_detected - 1):
#         for j in range(i + 1, num_detected):
#             point_i = marking_points[i]
#             point_j = marking_points[j]
#             # Step 1: length filtration.
#             distance = calc_point_squre_dist(point_i, point_j)
#             if not (config.VSLOT_MIN_DIST <= distance <= config.VSLOT_MAX_DIST
#                     or config.HSLOT_MIN_DIST <= distance <= config.HSLOT_MAX_DIST):
#                 continue
#             # Step 2: pass through filtration.
#             if pass_through_third_point(marking_points, i, j):
#                 continue
#             result = pair_marking_points(point_i, point_j)
#             if result == 1:
#                 slots.append((i, j))
#             elif result == -1:
#                 slots.append((j, i))
#     return slots



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

def detect_image(weight, device, args):
    model = DetectMultiBackend(weight, device=device, dnn=False, data=None, fp16=False)
    stride, names, pt = model.stride, model.names, model.pt
    folder_path="/home/parking/DMPR-PS1.0/result"
    image_files = glob.glob(f"{folder_path}/*.png")
    for image_file in image_files:
        # image_file = '/home/parking/DMPR-PS1.0/util/0001--.jpg'
        # image_file = '/home/parking/parking_dataset/train/train/p2_img99_0174_90.jpg'
        image = cv.imread(image_file)
        # image = cv.imread("/home/parking/DMPR-PS1.0/result/bev_360_view_171.png")
        # cv.imshow('demo', image)
        # cv.waitKey(1)
        # im = torch.from_numpy(image).to(model.device)
        im = preprocess_image(image).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        # im /= 255  # 0 - 255 to 0.0 - 1.0
        # npimage = im[0].cpu().numpy()
        # maxValue=npimage.max()
        # npimage=npimage*255/maxValue
        # mat=np.uint8(npimage)
        # mat=mat.transpose(1,2,0)
        # mat=cv.cvtColor(mat,cv.COLOR_RGB2BGR)
        # cv.imshow("img",mat)
        # cv.waitKey(0)
        timer = Timer()
        # image_file = '0001--.jpg'
        pred = model(im, augment=False, visualize=False)
        pred = pred[0][1]
        # pred , pred_orien = pred.split((6,2),1)
        timer.tic()
        pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False, max_det=1000)
        plot_points(image, pred)
        slots = inference_slots(pred)
        plot_slots(image, pred, slots)
        # cv.imshow('demo', image)
        # cv.waitKey(0)
        image_name = Path(image_file).name
        save_path = "/home/parking/DMPR-PS1.0/result2"
        save_path = f'{save_path}/{image_name}'
        if args.save:
            # cv.imwrite('/home/parking/DMPR-PS1.0/save3.jpg', image, [int(cv.IMWRITE_JPEG_QUALITY), 100])
            cv.imwrite(save_path, image)
    
    


def inference_detector(args):
    """Inference demo of directional point detector."""
    args.cuda = not args.disable_cuda and torch.cuda.is_available()
    device = torch.device('cuda:' + str(args.gpu_id) if args.cuda else 'cpu')
    print("args.gpu_id",args.gpu_id)
    torch.set_grad_enabled(False)

    weight = ROOT / 'ckpt'/'weights'/'epoch17.pt'
    device = select_device(device)
    detect_image(weight, device, args)


if __name__ == '__main__':
    inference_detector(config.get_parser_for_inference().parse_args())
