import cv2
import numpy as np
import time
import math


last_valid_angle = 0
smoothed_angle = 0
alpha = 0.2


def runPipeline(image, llrobot):
    global last_valid_angle, smoothed_angle, alpha

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)

    contours, _ = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    largest_contour = []
    largest_area = 0
    current_angle = 0
    detection_flag = 0

    if len(contours) > 0:

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > largest_area and area > 100:

                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) >= 4 and len(approx) <= 6:
                    largest_contour = contour
                    largest_area = area

        if largest_contour is not None and len(largest_contour) >= 4:
            try:

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