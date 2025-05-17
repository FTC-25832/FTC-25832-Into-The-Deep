# Actual code in pipe 1

import cv2
import numpy as np
import time
import math

last_valid_angle = 0
smoothed_angle = 0
alpha = 0.2
AREA_RANGE = [7000, 34000]
# ROI is left bottom corner of the image
ROI = [0, 200, 360, 280]  # x, y, w, h

def separate_contours(mask):
    # 使用形态学操作增强分离效果
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3)

    # 确保背景是黑色的，前景是白色的
    sure_bg = cv2.dilate(mask, kernel, iterations=3)

    # 距离变换
    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist_transform, 0.5*dist_transform.max(), 255, 0)

    # 寻找不确定区域
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # 标记连通区域
    _, markers = cv2.connectedComponents(sure_fg)

    # 为分水岭算法添加标记
    markers = markers + 1
    markers[unknown == 255] = 0

    # 转换为BGR图像用于分水岭
    color_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(color_mask, markers)

    # 创建分离后的轮廓列表
    separated_contours = []
    for marker in np.unique(markers):
        if marker > 1:  # 忽略背景和边界
            # 为每个标记区域创建掩码
            temp_mask = np.zeros(mask.shape, dtype="uint8")
            temp_mask[markers == marker] = 255

            # 找到轮廓
            cnts, _ = cv2.findContours(temp_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts) > 0:
                separated_contours.append(cnts[0])

    return separated_contours

def runPipeline(image, llrobot):
    global last_valid_angle, smoothed_angle, alpha

    # 裁剪ROI区域并调整大小
    cutted = image[ROI[1]:ROI[1] + ROI[3], ROI[0]:ROI[0] + ROI[2]]
    cutted = cv2.resize(cutted, (320, 240))
    cutted = cv2.GaussianBlur(cutted, (5, 5), 0)

    hsv = cv2.cvtColor(cutted, cv2.COLOR_BGR2HSV)

    # 绘制ROI区域
    cv2.rectangle(
        image,
        (ROI[0], ROI[1]),
        (ROI[0] + ROI[2], ROI[1] + ROI[3]),
        (255, 0, 0),
        2,
    )

    # 根据颜色选择阈值
    if llrobot[0] == 0:  # blue
        lower = np.array([100, 150, 0])
        upper = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif llrobot[0] == 1:  # red
        lower = np.array([0, 150, 0])
        upper = np.array([10, 255, 255])
        lower2 = np.array([170, 150, 0])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)
    elif llrobot[0] == 2:  # yellow
        lower = np.array([20, 150, 50])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        return [], image, [0] * 8

    # 形态学操作
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 分离相邻的方块
    separated_contours = separate_contours(mask)

    largest_contours = []
    current_angle = 0
    detection_flag = 0
    angles = []

    if len(separated_contours) > 0:
        # 按面积排序并保留前两个最大的轮廓
        separated_contours = sorted(separated_contours, key=cv2.contourArea, reverse=True)[:2]

        for contour in separated_contours:
            area = cv2.contourArea(contour)
            if AREA_RANGE[0] < area < AREA_RANGE[1]:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) >= 4 and len(approx) <= 6:
                    largest_contours.append(contour)

        if len(largest_contours) > 0:
            detection_flag = 1
            for contour in largest_contours:
                try:
                    # 调整ROI偏移
                    contour[:, :, 0] += ROI[0]
                    contour[:, :, 1] += ROI[1]

                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    # 绘制轮廓和边界框
                    cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                    cv2.drawContours(image, [box], -1, (0, 0, 255), 2)

                    (x, y), (width, height), rect_angle = rect

                    if width < height:
                        width, height = height, width
                        rect_angle += 90

                    angle = rect_angle
                    if angle < -90:
                        angle += 180
                    if angle > 90:
                        angle -= 180

                    angles.append(angle)

                except cv2.error as e:
                    print(f"OpenCV error: {e}")

            # 计算平均角度
            if angles:
                current_angle = np.mean(angles)
                smoothed_angle = alpha * current_angle + (1 - alpha) * smoothed_angle
                last_valid_angle = smoothed_angle

    llpython = [
        detection_flag,
        smoothed_angle if detection_flag else last_valid_angle,
        0,
        0,
        0,
        0,
        0,
        0,
    ]

    cv2.putText(
        image,
        f"Smoothed: {llpython[1]:.1f}°",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )

    return largest_contours, image, llpython