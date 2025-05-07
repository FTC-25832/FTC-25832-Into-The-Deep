import cv2
import numpy as np
import time
import math


last_valid_angle = 0
smoothed_angle = 0
alpha = 0.2
AREA_RANGE = [7000, 20000]
# ROI is left bottom corner of the image
ROI = [0, 240, 320, 240]  # x, y, w, h


def separate_touching_contours(contour, min_area_ratio=0.15):
    x, y, w, h = cv2.boundingRect(contour)
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

    original_area = cv2.contourArea(contour)
    max_contours = []
    max_count = 1

    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 3)

    for threshold in np.linspace(0.1, 0.9, 9):
        _, thresh = cv2.threshold(
            dist_transform, threshold * dist_transform.max(), 255, 0
        )
        thresh = np.uint8(thresh)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        valid_contours = [
            c for c in contours if cv2.contourArea(c) > original_area * min_area_ratio
        ]

        if len(valid_contours) > max_count:
            max_count = len(valid_contours)
            max_contours = valid_contours

    if max_contours:
        return [c + [x, y] for c in max_contours]
    return [contour]


# 0 for blue, 1 for red, 2 for yellow
def runPipeline(image, llrobot):
    llrobot[0] = 1
    global last_valid_angle, smoothed_angle, alpha

    # let cutted be the ROI of the image, but with the same size as the original image
    cutted = image[ROI[1] : ROI[1] + ROI[3], ROI[0] : ROI[0] + ROI[2]]
    cutted = cv2.resize(cutted, (320, 240))
    cutted = cv2.GaussianBlur(cutted, (5, 5), 0)

    hsv = cv2.cvtColor(cutted, cv2.COLOR_BGR2HSV)

    # draw ROI
    cv2.rectangle(
        image,
        (ROI[0], ROI[1]),
        (ROI[0] + ROI[2], ROI[1] + ROI[3]),
        (255, 0, 0),
        2,
    )

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

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Apply separate_touching_contours to each contour that exceeds a certain area threshold
    separated_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > AREA_RANGE[0]:  # Only process potentially valid contours
            separated = separate_touching_contours(contour)
            separated_contours.extend(separated)
        else:
            separated_contours.append(contour)

    largest_contour = []
    largest_area = 0
    current_angle = 0
    detection_flag = 0

    if len(separated_contours) > 0:
        for contour in separated_contours:
            area = cv2.contourArea(contour)
            if area > largest_area and AREA_RANGE[0] < area < AREA_RANGE[1]:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) >= 4 and len(approx) <= 6:
                    largest_contour = contour
                    largest_area = area

        if largest_contour is not None and len(largest_contour) >= 4:
            try:
                # adjust for ROI
                largest_contour[:, :, 0] += ROI[0]
                largest_contour[:, :, 1] += ROI[1]
                rect = cv2.minAreaRect(np.array(largest_contour))
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(image, [largest_contour], 0, (0, 255, 0), 2)
                cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

                (x, y), (width, height), rect_angle = rect

                if width < height:
                    width, height = height, width
                    rect_angle += 90

                current_angle = rect_angle
                if current_angle < -90:
                    current_angle += 180
                if current_angle > 90:
                    current_angle -= 180

                global smoothed_angle
                smoothed_angle = alpha * current_angle + (1 - alpha) * smoothed_angle

                last_valid_angle = smoothed_angle
                detection_flag = 1

            except cv2.error as e:
                print(f"OpenCV error: {e}")
                current_angle = last_valid_angle

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
    return largest_contour, image, llpython
