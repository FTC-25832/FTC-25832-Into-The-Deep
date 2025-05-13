import cv2
import numpy as np
import math
import time
from collections import defaultdict

# Constants for filtering contours
SMALL_CONTOUR_AREA = 300

# Minimum average brightness threshold (0-255)
MIN_BRIGHTNESS_THRESHOLD = 50

# Color detection ranges for blue in HSV
HSV_BLUE_RANGE = ([80, 60, 100], [110, 255, 255])

# Edge detection parameters - initial values
BLUR_SIZE = 17
SOBEL_KERNEL = 3

# Aspect ratio range for contour filtering
MIN_ASPECT_RATIO = 1.5  # Minimum width/height ratio
MAX_ASPECT_RATIO = 6.0  # Maximum width/height ratio

def calculate_angle(contour):
    if len(contour) < 5:
        return 0
    (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
    return angle

def draw_info(image, color, angle, center, index, area):
    cv2.putText(image, f"#{index}: {color}", (center[0] - 40, center[1] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.putText(image, f"Angle: {angle:.2f}", (center[0] - 40, center[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.putText(image, f"Area: {area:.2f}", (center[0] - 40, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.circle(image, center, 5, (255, 0, 0), -1)
    cv2.line(image, center, (int(center[0] + 50 * math.cos(math.radians(90 - angle))), 
                             int(center[1] - 50 * math.sin(math.radians(90 - angle)))), (255, 0, 0), 2)

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
        _, thresh = cv2.threshold(dist_transform, threshold * dist_transform.max(), 255, 0)
        thresh = np.uint8(thresh)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > original_area * min_area_ratio]

        if len(valid_contours) > max_count:
            max_count = len(valid_contours)
            max_contours = valid_contours

    if max_contours:
        return [c + [x, y] for c in max_contours]
    return [contour]

def pipeline_debug_return(frame):
    return None, None, None, True, frame

def process_color(frame, mask):
    try:
        debug_info = None
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

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours, hierarchy, gray_masked, False, debug_info

    except Exception as e:
        print(f"Process color error: {str(e)}")
        return [], None, None, False, None

def debug_return(frame):
    return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]

def runPipeline(frame, llrobot):
    try:
        # Initialize Limelight-style output
        llpython = [0, 0, 0, 0, 0, 0, 0, 0]
        largest_contour = np.array([[]])
        largest_area = 0
        
        # Convert to HSV and denoise
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_denoised = cv2.GaussianBlur(hsv, (5, 5), 0)
        hsv_denoised = hsv

        # Create mask for blue
        blue_mask = cv2.inRange(hsv_denoised, np.array(HSV_BLUE_RANGE[0]), np.array(HSV_BLUE_RANGE[1]))
        blue_mask = cv2.erode(blue_mask, np.ones((3, 3), np.uint8))

        # Process blue color
        blue_contours, blue_hierarchy, blue_gray, isDebug, debug_info = process_color(frame, blue_mask)
        if isDebug:
            return debug_return(debug_info)

        # Create a copy for contour visualization
        contour_frame = frame.copy()
        valid_contours = []

        for i, contour in enumerate(blue_contours):
            if cv2.contourArea(contour) < SMALL_CONTOUR_AREA:
                continue

            # Check aspect ratio using minAreaRect
            rect = cv2.minAreaRect(contour)
            width = max(rect[1])
            height = min(rect[1])
            if width == 0 or height == 0:
                continue
                
            aspect_ratio = width / height
            if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                continue

            for sep_contour in separate_touching_contours(contour):
                mask = np.zeros(blue_gray.shape, dtype=np.uint8)
                cv2.drawContours(mask, [sep_contour], -1, 255, -1)

                if cv2.mean(blue_gray, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
                    continue

                M = cv2.moments(sep_contour)
                if M["m00"] == 0:
                    continue

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                angle = calculate_angle(sep_contour)
                area = cv2.contourArea(sep_contour)

                # Store valid contour info
                valid_contours.append({
                    'contour': sep_contour,
                    'center': center,
                    'angle': angle,
                    'area': area,
                    'index': i
                })

                # Update llpython and largest_contour if this is the largest valid contour
                if area > largest_area:
                    largest_area = area
                    largest_contour = sep_contour
                    llpython = [1, center[0], center[1], angle, len(blue_contours), 0, 0, 0]

        # Draw all valid contours and their info
        for contour_info in valid_contours:
            cv2.drawContours(frame, [contour_info['contour']], -1, (255, 0, 0), 2)
            draw_info(frame, "Blue", contour_info['angle'], contour_info['center'], 
                     contour_info['index'] + 1, contour_info['area'])

        return largest_contour, frame, llpython

    except Exception as e:
        print(f"Error in runPipeline: {str(e)}")
        return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]
