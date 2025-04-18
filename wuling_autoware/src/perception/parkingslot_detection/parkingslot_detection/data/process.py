"""Defines related function to process defined data structure."""
import math
import numpy as np
import torch
import config
from data.struct import MarkingPoint, detemine_point_shape


def non_maximum_suppression(pred_points):
    """Perform non-maxmum suppression on marking points."""
    #对标记点进行非极大值抑制
    suppressed = [False] * len(pred_points)
    #print(len(pred_points))
    #print(pred_points)
    for i in range(len(pred_points) - 1):
        for j in range(i + 1, len(pred_points)):
            i_x = pred_points[i][1].x
            i_y = pred_points[i][1].y
            j_x = pred_points[j][1].x
            j_y = pred_points[j][1].y
            # 0.0625 = 1 / 16
            if abs(j_x - i_x) < 0.0625 and abs(j_y - i_y) < 0.0625:
                idx = i if pred_points[i][0] < pred_points[j][0] else j
                suppressed[idx] = True
    if any(suppressed):
        unsupres_pred_points = []
        for i, supres in enumerate(suppressed):
            if not supres:
                unsupres_pred_points.append(pred_points[i])
        return unsupres_pred_points
    #print("sssssssssssssssss",suppressed)
    return pred_points


def get_predicted_points(prediction, thresh):
    """Get marking points from one predicted feature map."""
    assert isinstance(prediction, torch.Tensor)
    predicted_points = []
    prediction = prediction.detach().cpu().numpy()
    for i in range(prediction.shape[1]):
        for j in range(prediction.shape[2]):
            if prediction[0, i, j] >= thresh:#判断每个方格的置信度是否满足要求
                xval = (j + prediction[2, i, j]) / prediction.shape[2]
                yval = (i + prediction[3, i, j]) / prediction.shape[1]#还原成归一化后的标记点
                if not (config.BOUNDARY_THRESH <= xval <= 1-config.BOUNDARY_THRESH
                        and config.BOUNDARY_THRESH <= yval <= 1-config.BOUNDARY_THRESH):
                    continue
                cos_value = prediction[4, i, j]
                sin_value = prediction[5, i, j]
                direction = math.atan2(sin_value, cos_value)
                marking_point = MarkingPoint(
                    xval, yval, direction, prediction[1, i, j])
                predicted_points.append((prediction[0, i, j], marking_point))
                #print(predicted_points)
    return non_maximum_suppression(predicted_points)


def pass_through_third_point(marking_points, i, j):
    """See whether the line between two points pass through a third point."""
    x_1 = (marking_points[i][..., 0] + marking_points[i][..., 2]) / 1280
    y_1 = (marking_points[i][..., 1] + marking_points[i][..., 3]) / 1280
    x_2 = (marking_points[j][..., 0] + marking_points[j][..., 2]) / 1280
    y_2 = (marking_points[j][..., 1] + marking_points[j][..., 3]) / 1280
    for point_idx, point in enumerate(marking_points):
        if point_idx == i or point_idx == j:
            continue
        x_0 = (point[..., 0] + point[..., 2]) / 1280
        y_0 = (point[..., 1] + point[..., 3]) / 1280
        vec1 = np.array([x_0.item() - x_1.item(), y_0.item() - y_1.item()])
        vec2 = np.array([x_2.item() - x_0.item(), y_2.item() - y_0.item()])
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        if np.dot(vec1, vec2) > config.SLOT_SUPPRESSION_DOT_PRODUCT_THRESH:
            return True
    return False


def pair_marking_points(point_a, point_b):
    """See whether two marking points form a slot."""
    vector_ab = np.array([((point_b[..., 0] + point_b[..., 2]) / 1280).item() - ((point_a[..., 0] + point_a[..., 2]) / 1280).item(), ((point_b[..., 1] + point_b[..., 3]) / 1280).item() - ((point_a[..., 1] + point_a[..., 3]) / 1280).item()])
    vector_ab = vector_ab / np.linalg.norm(vector_ab)
    point_shape_a = detemine_point_shape(point_a, vector_ab)
    point_shape_b = detemine_point_shape(point_b, -vector_ab)
    if point_shape_a.value == 0 or point_shape_b.value == 0:
        return 0
    if point_shape_a.value == 3 and point_shape_b.value == 3:
        return 0
    if point_shape_a.value > 3 and point_shape_b.value > 3:
        return 0
    if point_shape_a.value < 3 and point_shape_b.value < 3:
        return 0
    if point_shape_a.value != 3:
        if point_shape_a.value > 3:
            return 1
        if point_shape_a.value < 3:
            return -1
    if point_shape_a.value == 3:
        if point_shape_b.value < 3:
            return 1
        if point_shape_b.value > 3:
            return -1
