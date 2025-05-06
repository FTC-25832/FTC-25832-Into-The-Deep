import cv2
import numpy as np
import time
import math

# Constants for angle smoothing
SMOOTHING_FACTOR = 0.1
MINIMUM_CONTOUR_AREA = 100

# ROI parameters
ROI_X = 100  # X coordinate of top-left corner
ROI_Y = 100  # Y coordinate of top-left corner
ROI_WIDTH = 400  # Width of ROI
ROI_HEIGHT = 300  # Height of ROI

# Color detection ranges for HSV
BLUE_HSV_LOWER = np.array([100, 150, 50])
BLUE_HSV_UPPER = np.array([130, 255, 255])

YELLOW_HSV_LOWER = np.array([20, 100, 100])
YELLOW_HSV_UPPER = np.array([30, 255, 255])

# Red has two ranges in HSV space
RED_HSV_LOWER1 = np.array([0, 100, 100])
RED_HSV_UPPER1 = np.array([10, 255, 255])
RED_HSV_LOWER2 = np.array([160, 100, 100])
RED_HSV_UPPER2 = np.array([180, 255, 255])

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
        # Define color-specific BGR values
        color_bgr = {"Blue": (255, 0, 0), "Yellow": (0, 255, 255), "Red": (0, 0, 255)}
        bgr = color_bgr.get(color, (255, 0, 0))  # Default to blue if color not found

        cv2.putText(
            image,
            f"#{index}: {color}",
            (center[0] - 40, center[1] - 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bgr,
            2,
        )
        cv2.putText(
            image,
            f"Angle: {angle:.2f}",
            (center[0] - 40, center[1] - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bgr,
            2,
        )
        cv2.putText(
            image,
            f"Area: {area:.2f}",
            (center[0] - 40, center[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            bgr,
            2,
        )
        cv2.circle(image, center, 5, bgr, -1)
        cv2.line(
            image,
            center,
            (
                int(center[0] + 50 * math.cos(math.radians(90 - angle))),
                int(center[1] - 50 * math.sin(math.radians(90 - angle))),
            ),
            bgr,
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
        # Draw ROI rectangle
        cv2.rectangle(
            image,
            (ROI_X, ROI_Y),
            (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT),
            (0, 255, 0),
            2,
        )

        # Create ROI mask
        roi_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        roi_mask[ROI_Y : ROI_Y + ROI_HEIGHT, ROI_X : ROI_X + ROI_WIDTH] = 255

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_denoised = cv2.GaussianBlur(hsv, (5, 5), 0)

        # Process each color within ROI
        blue_mask = cv2.inRange(hsv_denoised, BLUE_HSV_LOWER, BLUE_HSV_UPPER) & roi_mask
        yellow_mask = (
            cv2.inRange(hsv_denoised, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER) & roi_mask
        )
        red_mask1 = cv2.inRange(hsv_denoised, RED_HSV_LOWER1, RED_HSV_UPPER1) & roi_mask
        red_mask2 = cv2.inRange(hsv_denoised, RED_HSV_LOWER2, RED_HSV_UPPER2) & roi_mask
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Process each color
        blue_contours, blue_hierarchy, blue_gray = process_color(image, blue_mask)
        yellow_contours, yellow_hierarchy, yellow_gray = process_color(
            image, yellow_mask
        )
        red_contours, red_hierarchy, red_gray = process_color(image, red_mask)

        # Combine contours with color information
        all_contours = []
        if len(blue_contours) > 0:
            all_contours.extend(
                [{"contour": c, "color": "Blue"} for c in blue_contours]
            )
        if len(yellow_contours) > 0:
            all_contours.extend(
                [{"contour": c, "color": "Yellow"} for c in yellow_contours]
            )
        if len(red_contours) > 0:
            all_contours.extend([{"contour": c, "color": "Red"} for c in red_contours])

        detection_flag = 0
        current_angle = last_valid_angle
        result_contour = np.array([])
        valid_contours = []

        if len(all_contours) > 0:
            for i, contour_info in enumerate(all_contours):
                contour = contour_info["contour"]
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

                color = contour_info["color"]
                gray_img = (
                    blue_gray
                    if color == "Blue"
                    else yellow_gray if color == "Yellow" else red_gray
                )

                separated_contours = separate_touching_contours(contour)
                for sep_contour in separated_contours:
                    mask = np.zeros(gray_img.shape, dtype=np.uint8)
                    cv2.drawContours(mask, [sep_contour], -1, 255, -1)

                    if cv2.mean(gray_img, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
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
                                "color": contour_info["color"],
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
            # Draw contours with color-specific colors
            color_bgr = {
                "Blue": (255, 0, 0),
                "Yellow": (0, 255, 255),
                "Red": (0, 0, 255),
            }
            cv2.drawContours(
                image,
                [contour_info["contour"]],
                -1,
                color_bgr[contour_info["color"]],
                2,
            )
            draw_info(
                image,
                contour_info["color"],
                contour_info["angle"],
                contour_info["center"],
                contour_info["index"] + 1,
                contour_info["area"],
            )

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

        return result_contour, image, llpython

    except Exception as e:
        print(f"Error in runPipeline: {str(e)}")
        return np.array([]), image, [0, 0, 0, 0, 0, 0, 0, 0]
