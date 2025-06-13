import cv2
import numpy as np
import math

# ===================== CONFIGURATION VARIABLES =====================
# Color detection ranges for blue in HSV
BLUE_HSV_LOWER = np.array([95, 150, 0])
BLUE_HSV_UPPER = np.array([120, 255, 255])

# Image preprocessing parameters
GAUSSIAN_BLUR_HSV_KERNEL = (5, 5)  # Kernel size for HSV Gaussian blur
GAUSSIAN_BLUR_GRAY_KERNEL = (3, 3)  # Kernel size for grayscale Gaussian blur
FINAL_BLUR_KERNEL = (3, 3)  # Kernel size for final edge smoothing

# Morphological operation parameters
ERODE_KERNEL = np.ones((3, 3), np.uint8)  # Kernel for erosion
ERODE_ITERATIONS = 1  # Iterations for erosion
DILATE_KERNEL = np.ones((3, 3), np.uint8)  # Kernel for dilation
DILATE_ITERATIONS = 2  # Iterations for dilation
EDGE_CLOSE_KERNEL = np.ones((5, 5), np.uint8)  # Kernel for morphological closing

# Edge detection parameters
SOBEL_THRESHOLD = 50  # Threshold for Sobel edge detection
SOBEL_KSIZE = 1  # Kernel size for Sobel operator

# Contour filtering parameters
MINIMUM_CONTOUR_AREA = 100  # Minimum area for valid contours
EPSILON_FACTOR = 0.1  # Factor for contour approximation
MIN_VERTICES = 4  # Minimum number of vertices for valid contours
MAX_VERTICES = 6  # Maximum number of vertices for valid contours

# Object separation parameters
MIN_AREA_RATIO = 0.15  # Minimum area ratio for separated contours
MIN_ASPECT_RATIO = 1.5  # Minimum aspect ratio for valid contours
MAX_ASPECT_RATIO = 6.0  # Maximum aspect ratio for valid contours
MIN_BRIGHTNESS_THRESHOLD = 50  # Minimum brightness for valid contours
DISTANCE_TRANSFORM_METHOD = cv2.DIST_L2  # Distance transform method
DISTANCE_TRANSFORM_MASK_SIZE = 3  # Mask size for distance transform
SEPARATION_THRESHOLDS = np.linspace(0.1, 0.9, 9)  # Thresholds for separation

# Angle smoothing parameters
SMOOTHING_FACTOR = 0.1  # Factor for angle smoothing

# Drawing parameters
TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX  # Font for text
TEXT_SCALE = 0.5  # Font scale
TEXT_THICKNESS = 2  # Text thickness
TEXT_COLOR = (255, 0, 0)  # Text color (BGR)
CONTOUR_COLOR = (255, 0, 0)  # Contour color (BGR)
CONTOUR_THICKNESS = 2  # Contour line thickness
CENTER_RADIUS = 5  # Radius for center point
DIRECTION_LINE_LENGTH = 50  # Length of direction line

# Global variables for angle tracking
last_valid_angle = 0
smoothed_angle = 0


def step_1_thresholding(image):
    """
    Step 1: Convert to HSV and apply color thresholding to isolate blue regions.
    Returns a mask.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_denoised = cv2.GaussianBlur(hsv, GAUSSIAN_BLUR_HSV_KERNEL, 0)
    blue_mask = cv2.inRange(hsv_denoised, BLUE_HSV_LOWER, BLUE_HSV_UPPER)
    blue_mask = cv2.erode(blue_mask, ERODE_KERNEL, iterations=ERODE_ITERATIONS)
    return blue_mask


def step_2_masking(frame, mask):
    """
    Step 2: Apply the mask to the original image and prepare for edge detection.
    Returns masked image and grayscale version.
    """
    masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
    gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_masked, GAUSSIAN_BLUR_GRAY_KERNEL, 0)
    return masked_frame, gray_masked, blurred


def step_3_edge_detection(blurred_image, mask):
    """
    Step 3: Detect edges using Sobel operator.
    Returns edges.
    """
    # Apply Sobel edge detection
    sobelx = cv2.Sobel(blurred_image, cv2.CV_32F, 1, 0, ksize=SOBEL_KSIZE)
    sobely = cv2.Sobel(blurred_image, cv2.CV_32F, 0, 1, ksize=SOBEL_KSIZE)

    # Calculate magnitude and normalize
    magnitude = np.sqrt(sobelx**2 + sobely**2)
    magnitude = (
        np.uint8(magnitude * 255 / np.max(magnitude))
        if np.max(magnitude) > 0
        else np.zeros_like(magnitude, dtype=np.uint8)
    )

    # Threshold to get binary edges
    _, edges = cv2.threshold(magnitude, SOBEL_THRESHOLD, 255, cv2.THRESH_BINARY)
    return edges


def step_4_morphology(edges, mask):
    """
    Step 4: Apply morphological operations to enhance edges.
    Returns processed edges.
    """
    # Close gaps in edges
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, EDGE_CLOSE_KERNEL)

    # Dilate to improve connectivity
    edges = cv2.dilate(edges, DILATE_KERNEL, iterations=DILATE_ITERATIONS)

    # Invert and apply original mask to keep only relevant edges
    edges = cv2.bitwise_not(edges)
    edges = cv2.bitwise_and(edges, edges, mask=mask)

    # Final smoothing
    edges = cv2.GaussianBlur(edges, FINAL_BLUR_KERNEL, 0)
    return edges


def step_5_contours(edges):
    """
    Step 5: Find contours in the processed edges.
    Returns contours and hierarchy.
    """
    try:
        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return contours, hierarchy
    except Exception as e:
        print(f"Error finding contours: {str(e)}")
        return [], None


def separate_touching_contours(contour, min_area_ratio=MIN_AREA_RATIO):
    """
    Helper function to separate touching contours using distance transform.
    Returns list of separated contours.
    """
    try:
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return [contour]

        mask = np.zeros((h, w), dtype=np.uint8)
        shifted_contour = contour - np.array([x, y])
        cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

        original_area = cv2.contourArea(contour)
        max_contours = []
        max_count = 1

        dist_transform = cv2.distanceTransform(
            mask, DISTANCE_TRANSFORM_METHOD, DISTANCE_TRANSFORM_MASK_SIZE
        )

        for threshold in SEPARATION_THRESHOLDS:
            _, thresh = cv2.threshold(
                dist_transform, threshold * dist_transform.max(), 255, 0
            )
            thresh = np.uint8(thresh)

            cnts, _ = cv2.findContours(
                thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            valid_contours = [
                c for c in cnts if cv2.contourArea(c) > original_area * min_area_ratio
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
    """
    Helper function to calculate the angle of a contour using ellipse fitting.
    Returns angle in degrees.
    """
    try:
        if len(contour) < 5:
            return 0
        (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
        return angle
    except Exception as e:
        print(f"Angle calculation error: {str(e)}")
        return 0


def step_6_process_contours(contours, gray_masked):
    """
    Step 6: Process contours to filter, separate touching objects, calculate angles.
    Returns list of valid contour information.
    """
    valid_contours = []

    for i, contour in enumerate(contours):
        # Skip small contours
        if cv2.contourArea(contour) < MINIMUM_CONTOUR_AREA:
            continue

        # Check aspect ratio
        rect = cv2.minAreaRect(contour)
        width = max(rect[1])
        height = min(rect[1])

        if width == 0 or height == 0:
            continue

        aspect_ratio = width / height
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue

        # Process potential touching objects
        separated_contours = separate_touching_contours(contour)

        for sep_contour in separated_contours:
            # Create mask for the contour
            mask = np.zeros(gray_masked.shape, dtype=np.uint8)
            cv2.drawContours(mask, [sep_contour], -1, 255, -1)

            # Skip if brightness is too low
            if cv2.mean(gray_masked, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
                continue

            if len(sep_contour) >= MIN_VERTICES:
                # Calculate center using moments
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

    return valid_contours


def draw_info(image, contour_info):
    """
    Helper function to draw information about contours on the image.
    """
    color = "Blue"
    angle = contour_info["angle"]
    center = contour_info["center"]
    index = contour_info["index"] + 1
    area = contour_info["area"]

    try:
        cv2.putText(
            image,
            f"#{index}: {color}",
            (center[0] - 40, center[1] - 60),
            TEXT_FONT,
            TEXT_SCALE,
            TEXT_COLOR,
            TEXT_THICKNESS,
        )
        cv2.putText(
            image,
            f"Angle: {angle:.2f}",
            (center[0] - 40, center[1] - 40),
            TEXT_FONT,
            TEXT_SCALE,
            TEXT_COLOR,
            TEXT_THICKNESS,
        )
        cv2.putText(
            image,
            f"Area: {area:.2f}",
            (center[0] - 40, center[1] - 20),
            TEXT_FONT,
            TEXT_SCALE,
            TEXT_COLOR,
            TEXT_THICKNESS,
        )
        cv2.circle(image, center, CENTER_RADIUS, TEXT_COLOR, -1)
        cv2.line(
            image,
            center,
            (
                int(
                    center[0]
                    + DIRECTION_LINE_LENGTH * math.cos(math.radians(90 - angle))
                ),
                int(
                    center[1]
                    - DIRECTION_LINE_LENGTH * math.sin(math.radians(90 - angle))
                ),
            ),
            TEXT_COLOR,
            TEXT_THICKNESS,
        )
    except Exception as e:
        print(f"Drawing error: {str(e)}")


def runPipeline(image, llrobot):
    """
    Main pipeline that processes an image to detect blue objects.
    Returns contours, processed image, and llpython data.
    """
    global last_valid_angle, smoothed_angle

    try:
        # ========== Step 1: Color thresholding ==========
        blue_mask = step_1_thresholding(image)

        # ========== Step 2: Apply mask to original image ==========
        masked_image, gray_masked, blurred = step_2_masking(image, blue_mask)
        # Uncomment to debug step 2
        # return np.array([]), masked_image, [0, 0, 0, 0, 0, 0, 0, 0]

        # ========== Step 3: Edge detection ==========
        edges = step_3_edge_detection(blurred, blue_mask)
        edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # Uncomment to debug step 3
        # return np.array([]), edges_rgb, [0, 0, 0, 0, 0, 0, 0, 0]

        # ========== Step 4: Morphological operations ==========
        processed_edges = step_4_morphology(edges, blue_mask)
        processed_edges_rgb = cv2.cvtColor(processed_edges, cv2.COLOR_GRAY2BGR)
        # Uncomment to debug step 4
        # return np.array([]), processed_edges_rgb, [0, 0, 0, 0, 0, 0, 0, 0]

        # ========== Step 5: Find contours ==========
        contours, hierarchy = step_5_contours(processed_edges)

        # ========== Step 6: Process and analyze contours ==========
        valid_contours = step_6_process_contours(contours, gray_masked)

        # Initialize output values
        detection_flag = 0
        current_angle = last_valid_angle
        result_contour = np.array([])

        # Process detected contours
        if valid_contours:
            detection_flag = 1
            # Use the first valid contour as the result
            result_contour = valid_contours[0]["contour"]
            current_angle = valid_contours[0]["angle"]
            last_valid_angle = current_angle

            # Apply smoothing to angle
            smoothed_angle = (
                SMOOTHING_FACTOR * current_angle
                + (1 - SMOOTHING_FACTOR) * smoothed_angle
            )

        # Draw contours and information on image
        result_image = image.copy()
        for contour_info in valid_contours:
            cv2.drawContours(
                result_image,
                [contour_info["contour"]],
                -1,
                CONTOUR_COLOR,
                CONTOUR_THICKNESS,
            )
            draw_info(result_image, contour_info)

        # Prepare output data
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

        return result_contour, result_image, llpython

    except Exception as e:
        print(f"Error in runPipeline: {str(e)}")
        return np.array([]), image, [0, 0, 0, 0, 0, 0, 0, 0]
