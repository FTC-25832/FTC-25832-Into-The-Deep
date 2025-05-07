import cv2
import numpy as np
import time
import math

# Constants for angle smoothing
SMOOTHING_FACTOR = 0.1
MINIMUM_CONTOUR_AREA = 100

# Color detection ranges in HSV
BLUE_HSV_LOWER = np.array([100, 150, 50])
BLUE_HSV_UPPER = np.array([130, 255, 255])
RED_HSV_LOWER1 = np.array([0, 150, 50])
RED_HSV_UPPER1 = np.array([10, 255, 255])
RED_HSV_LOWER2 = np.array([170, 150, 50])
RED_HSV_UPPER2 = np.array([180, 255, 255])
YELLOW_HSV_LOWER = np.array([20, 150, 50])
YELLOW_HSV_UPPER = np.array([30, 255, 255])

# Morphological operation parameters
KERNEL_SIZE = 5
ERODE_ITERATIONS = 1
DILATE_ITERATIONS = 2

# Contour approximation parameters
EPSILON_FACTOR = 0.04
MIN_VERTICES = 4
MAX_VERTICES = 6

# Separation parameters
MIN_AREA_RATIO = 0.15
MIN_ASPECT_RATIO = 1.5
MAX_ASPECT_RATIO = 6.0
MIN_BRIGHTNESS_THRESHOLD = 50


def process_color(frame, mask):
    try:
        kernel = np.ones((5, 5), np.uint8)
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
        gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray_masked, (3, 3), 0)

        sobelx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=1)
        sobely = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=1)

        magnitude = np.sqrt(sobelx**2 + sobely**2)
        magnitude = np.uint8(magnitude * 255 / np.max(magnitude))

        _, edges = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        edges = cv2.bitwise_not(edges)
        edges = cv2.bitwise_and(edges, edges, mask=mask)
        edges = cv2.GaussianBlur(edges, (3, 3), 0)

        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return contours, hierarchy, gray_masked
    except Exception as e:
        print(f"Process color error: {str(e)}")
        return [], None, None


def separate_touching_contours(contour, min_area_ratio=MIN_AREA_RATIO):
    try:
        x, y, w, h = cv2.boundingRect(contour)
        mask = np.zeros((h, w), dtype=np.uint8)
        shifted_contour = contour - np.array([x, y])
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
                c
                for c in contours
                if cv2.contourArea(c) > original_area * min_area_ratio
            ]

            if len(valid_contours) > max_count:
                max_count = len(valid_contours)
                max_contours = valid_contours

        if max_contours:
            return [c + np.array([x, y]) for c in max_contours]
        return [contour]
    except Exception as e:
        print(f"Separation error: {str(e)}")
        return [contour]


def calculate_angle(contour):
    try:
        if len(contour) < 5:
            return 0
        (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
        return angle
    except Exception as e:
        print(f"Angle calculation error: {str(e)}")
        return 0


def draw_info(image, color, angle, center, index, area):
    try:
        cv2.putText(
            image,
            f"#{index}: {color}",
            (center[0] - 40, center[1] - 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            image,
            f"Angle: {angle:.2f}",
            (center[0] - 40, center[1] - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            image,
            f"Area: {area:.2f}",
            (center[0] - 40, center[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
        cv2.circle(image, center, 5, (255, 0, 0), -1)
        cv2.line(
            image,
            center,
            (
                int(center[0] + 50 * math.cos(math.radians(90 - angle))),
                int(center[1] - 50 * math.sin(math.radians(90 - angle))),
            ),
            (255, 0, 0),
            2,
        )
    except Exception as e:
        print(f"Drawing error: {str(e)}")


# Global variables for angle smoothing
last_valid_angle = 0
smoothed_angle = 0


def runPipeline(image, llrobot):
    global last_valid_angle, smoothed_angle

    try:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_denoised = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Create masks for each color
        blue_mask = cv2.inRange(hsv_denoised, BLUE_HSV_LOWER, BLUE_HSV_UPPER)
        red_mask1 = cv2.inRange(hsv_denoised, RED_HSV_LOWER1, RED_HSV_UPPER1)
        red_mask2 = cv2.inRange(hsv_denoised, RED_HSV_LOWER2, RED_HSV_UPPER2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        yellow_mask = cv2.inRange(hsv_denoised, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)

        # Combine all masks
        combined_mask = cv2.bitwise_or(blue_mask, cv2.bitwise_or(red_mask, yellow_mask))
        combined_mask = cv2.erode(combined_mask, np.ones((3, 3), np.uint8))

        contours, hierarchy, gray_masked = process_color(image, combined_mask)

        detection_flag = 0
        current_angle = last_valid_angle
        result_contour = np.array([])
        valid_contours = []

        if len(contours) > 0:
            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
                    continue

                rect = cv2.minAreaRect(contour)
                width = max(rect[1])
                height = min(rect[1])
                if width == 0 or height == 0:
                    continue

                aspect_ratio = width / height
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    continue

                separated_contours = separate_touching_contours(contour)
                for sep_contour in separated_contours:
                    mask = np.zeros(gray_masked.shape, dtype=np.uint8)
                    cv2.drawContours(mask, [sep_contour], -1, 255, -1)

                    if cv2.mean(gray_masked, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
                        continue

                    if len(sep_contour) >= 4:
                        M = cv2.moments(sep_contour)
                        if M["m00"] == 0:
                            continue

                        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        angle = calculate_angle(sep_contour)
                        area = cv2.contourArea(sep_contour)

                        valid_contours.append(
                            {
                                "contour": sep_contour,
                                "center": center,
                                "angle": angle,
                                "area": area,
                                "index": i,
                            }
                        )

                        # (x, y), (width, height), rect_angle = rect

                        # if width < height:
                        #     width, height = height, width
                        #     rect_angle += 90

                        # angle = rect_angle

                        # if angle < -90:
                        #     angle += 180

                        # if angle > 90:
                        #     angle -= 180

                        detection_flag = 1
                        # smoothed_angle = SMOOTHING_FACTOR * angle + (1 - SMOOTHING_FACTOR) * smoothed_angle
                        last_valid_angle = angle
                        detection_flag = 1
                        result_contour = sep_contour

        for contour_info in valid_contours:
            cv2.drawContours(image, [contour_info["contour"]], -1, (255, 0, 0), 2)
            draw_info(
                image,
                "Blue",
                contour_info["angle"],
                contour_info["center"],
                contour_info["index"] + 1,
                contour_info["area"],
            )

        # Get center coordinates for the largest contour
        center_x = 0
        center_y = 0
        if valid_contours:
            # Sort contours by area and get the largest one
            largest_contour = max(valid_contours, key=lambda x: x["area"])
            center_x = largest_contour["center"][0]
            center_y = largest_contour["center"][1]

        llpython = [
            detection_flag,
            smoothed_angle if detection_flag else last_valid_angle,
            center_x,  # 添加中心点x坐标
            center_y,  # 添加中心点y坐标
            0,
            0,
            0,
            0,
        ]

        return result_contour, image, llpython

    except Exception as e:
        print(f"Error in runPipeline: {str(e)}")
        return np.array([]), image, [0, 0, 0, 0, 0, 0, 0, 0]
