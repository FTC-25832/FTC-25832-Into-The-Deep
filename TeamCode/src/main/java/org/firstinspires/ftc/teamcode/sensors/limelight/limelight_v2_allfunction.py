import cv2
import numpy as np

# ===================== CONFIGURATION VARIABLES =====================
# Color detection ranges for blue in HSV
LOWER_BLUE_HSV = np.array([95, 150, 0])  # Lower bound for blue in HSV
UPPER_BLUE_HSV = np.array([120, 255, 255])  # Upper bound for blue in HSV

# Image preprocessing parameters
GAUSSIANBLUR_KERNEL_SIZE = (7, 7)  # Kernel size for HSV Gaussian blur
MEDIAN_BLUR_KERNEL_SIZE = 3  # Kernel size for median blur

# Edge detection parameters
CANNY_LOWER_THRESHOLD = 50  # Lower threshold for Canny edge detection
CANNY_UPPER_THRESHOLD = 110  # Upper threshold for Canny edge detection

# Morphological operation parameters
MORPHCLOSE_KERNAL_SIZE = (3, 3)  # Kernel for morphological closing
MORPHCLOSE_ITERATIONS = 3  # Iterations for morphological closing
MORPHOPEN_KERNAL_SIZE = (3, 3)  # Kernel for morphological opening
MORPHOPEN_ITERATIONS = 2  # Iterations for morphological opening

# Contour filtering parameters
MINIMUM_CONTOUR_AREA = 100  # Minimum area for valid contours
EPSILON_FACTOR = 0.1  # Factor for contour approximation
MIN_VERTICES = 4  # Minimum number of vertices for valid contours
MAX_VERTICES = 6  # Maximum number of vertices for valid contours

# Object separation parameters
MIN_AREA_RATIO = 0.15  # Minimum area ratio for separated contours
MIN_ASPECT_RATIO = 1.5  # Minimum aspect ratio for valid contours
MAX_ASPECT_RATIO = 6  # Maximum aspect ratio for valid contours
MIN_BRIGHTNESS_THRESHOLD = 50  # Minimum brightness for valid contours
DISTANCE_TRANSFORM_METHOD = cv2.DIST_L2  # Distance transform method
DISTANCE_TRANSFORM_MASK_SIZE = 5  # Mask size for distance transform
SEPARATION_THRESHOLDS = np.linspace(0.1, 0.9, 20)  # Thresholds for separation

# Camera intrinsic matrix from your calibration data
camera_matrix = np.array(
    [[1221.445, 0.000, 637.226], [0.000, 1223.398, 502.549], [0.000, 0.000, 1.000]],
    dtype=np.float32,
)

# Distortion coefficients (K1, K2, P1, P2, K3)
dist_coeffs = np.array(
    [0.177168, -0.457341, 0.000360, 0.002753, 0.178259], dtype=np.float32
)

# Image dimensions (640x480)
h, w = 480, 640

# Hough Transform parameters
HOUGH_RHO = 1  # Distance resolution in pixels
HOUGH_THETA = np.pi / 180  # Angle resolution in radians
HOUGH_THRESHOLD = 50  # Minimum number of votes (intersections)
HOUGH_MIN_LINE_LENGTH = 20  # Minimum length of line
HOUGH_MAX_LINE_GAP = (
    10  # Maximum gap between line segments to treat them as a single line
)


def step_1_thresholding(image):
    """
    Step 1: Apply color thresholding to isolate blue regions in the image.
    return a mask
    """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, LOWER_BLUE_HSV, UPPER_BLUE_HSV)
    return mask


def step_2_masking(image, mask):
    """
    Step 2: Apply the mask to the original image to isolate blue regions.
    return masked image
    """
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    blurred = cv2.medianBlur(masked_image, MEDIAN_BLUR_KERNEL_SIZE)
    return blurred


def connect_edges_with_hough(edges):
    """
    使用霍夫变换连接断裂的边缘
    """
    # 使用概率霍夫变换检测直线
    lines = cv2.HoughLinesP(
        edges,
        HOUGH_RHO,
        HOUGH_THETA,
        HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LINE_LENGTH,
        maxLineGap=HOUGH_MAX_LINE_GAP,
    )

    if lines is None:
        return edges

    # 创建新的边缘图像
    connected_edges = np.zeros_like(edges)

    # 绘制检测到的直线
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(connected_edges, (x1, y1), (x2, y2), 255, 1)

    # 合并原始边缘和连接的边缘
    combined_edges = cv2.bitwise_or(edges, connected_edges)

    return combined_edges


def step_3_detecting_canny_edge(image):
    """
    Step 3: Creating a wireframe of the target objects using edge detection.
    return edges
    """
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, GAUSSIANBLUR_KERNEL_SIZE, 0)
    edges = cv2.Canny(blurred_image, CANNY_LOWER_THRESHOLD, CANNY_UPPER_THRESHOLD)

    # 使用霍夫变换连接断裂的边缘
    connected_edges = connect_edges_with_hough(edges)

    return connected_edges


def step_3_detecting_sobel_edge(image):
    """
    Step 3: Creating a wireframe of the target objects using Sobel edge detection.
    return edges
    """
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, GAUSSIANBLUR_KERNEL_SIZE, 0)
    sobelx = cv2.Sobel(blurred_image, cv2.CV_32F, 1, 0, ksize=1)
    sobely = cv2.Sobel(blurred_image, cv2.CV_32F, 0, 1, ksize=1)
    magnitude = np.sqrt(sobelx**2 + sobely**2)
    magnitude = np.uint8(magnitude * 255 / np.max(magnitude))
    _, edges = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)
    return edges


def step_4_morphology(edges):
    """
    Step 4: Dilate the edges to enhance the visibility of contours.
    return dilated edges
    """
    kernel = np.ones(MORPHCLOSE_KERNAL_SIZE, np.uint8)
    closed = cv2.morphologyEx(
        edges, cv2.MORPH_DILATE, kernel, iterations=MORPHCLOSE_ITERATIONS
    )
    kernel = np.ones(MORPHOPEN_KERNAL_SIZE, np.uint8)
    opened = cv2.morphologyEx(
        closed, cv2.MORPH_ERODE, kernel, iterations=MORPHOPEN_ITERATIONS
    )
    return opened


def separate_touching_contours(contour, min_area_ratio=MIN_AREA_RATIO):
    """
    使用距离变换分离接触的轮廓。
    返回分离后的轮廓列表。
    """
    try:
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return [contour]

        # 创建掩码
        mask = np.zeros((h, w), dtype=np.uint8)
        shifted_contour = contour - np.array([x, y])
        cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

        original_area = cv2.contourArea(contour)
        max_contours = []
        max_count = 1

        # 应用距离变换
        dist_transform = cv2.distanceTransform(
            mask, DISTANCE_TRANSFORM_METHOD, DISTANCE_TRANSFORM_MASK_SIZE
        )

        # 使用不同的阈值尝试分离
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
        print(f"分离错误: {str(e)}")
        return [contour]


def fit_rectangle(contour):
    """
    将轮廓拟合成长方形，返回拟合后的四个角点坐标。
    返回格式：((x1,y1), (x2,y2), (x3,y3), (x4,y4))
    """
    try:
        # 计算最小外接矩形
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 计算中心点
        center = rect[0]

        # 计算长和宽
        width = rect[1][0]
        height = rect[1][1]

        # 计算角度
        angle = rect[2]

        # 确保宽度大于高度
        if width < height:
            width, height = height, width
            angle = angle + 90

        # 计算四个角点
        angle_rad = np.deg2rad(angle)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)

        # 计算矩形的四个角点
        half_width = width / 2
        half_height = height / 2

        # 计算四个角点的相对坐标
        corners = np.array(
            [
                [-half_width, -half_height],
                [half_width, -half_height],
                [half_width, half_height],
                [-half_width, half_height],
            ]
        )

        # 旋转角点
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        rotated_corners = np.dot(corners, rotation_matrix.T)

        # 平移到中心点
        final_corners = rotated_corners + center

        return final_corners.astype(np.int32)

    except Exception as e:
        print(f"长方形拟合错误: {str(e)}")
        return None


def step_5_contours(edges):
    """
    第5步：查找并分离接触的轮廓，返回最大的长方形轮廓。
    返回：最大长方形轮廓的四个角点坐标和原始轮廓
    """
    try:
        # 确保输入是二值图像
        if len(edges.shape) > 2:
            edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)

        # 查找初始轮廓
        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 存储所有有效的轮廓和它们的长方形拟合
        valid_contours = []
        max_area = 0
        best_rect = None
        best_contour = None

        # 处理每个轮廓
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < MINIMUM_CONTOUR_AREA:
                continue

            # 尝试分离接触的轮廓
            separated_contours = separate_touching_contours(contour)

            for sep_contour in separated_contours:
                # 计算轮廓的近似多边形
                epsilon = EPSILON_FACTOR * cv2.arcLength(sep_contour, True)
                approx = cv2.approxPolyDP(sep_contour, epsilon, True)

                # 检查顶点数量
                if not (MIN_VERTICES <= len(approx) <= MAX_VERTICES):
                    continue

                # 计算最小外接矩形
                rect = cv2.minAreaRect(sep_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # 计算长宽比
                width = rect[1][0]
                height = rect[1][1]
                aspect_ratio = max(width, height) / (min(width, height) + 1e-6)

                # 检查长宽比
                if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO):
                    continue

                # 拟合长方形
                rectangle = fit_rectangle(sep_contour)
                if rectangle is not None:
                    # 计算长方形面积
                    rect_area = width * height

                    # 更新最大面积的长方形
                    if rect_area > max_area:
                        max_area = rect_area
                        best_rect = rectangle
                        best_contour = sep_contour

                valid_contours.append(sep_contour)

        return best_rect, best_contour, valid_contours

    except Exception as e:
        print(f"轮廓检测错误: {str(e)}")
        return None, None, []


def runPipeline(image, llrobot):
    contours = []
    llpython = [0] * 8
    # return contours, image, llpython

    # ========== Step 1-2: Color thresholding to isolate blue regions ==========
    mask = step_1_thresholding(image)
    masked_image = step_2_masking(image, mask)  # step1-2 result
    # return contours, masked_image, llpython  # step2 result

    # ========== Step 3: Edge detection using Canny and Hough Transform ==========
    edges = step_3_detecting_canny_edge(masked_image)
    edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    # return contours, edges_rgb, llpython  # step3 result

    edges_combined = cv2.addWeighted(masked_image, 1, edges_rgb, 0.5, 0)  # Combine
    # return contours, edges_combined, llpython

    # ========== Step 4: Morphological operations to enhance contours ==========
    processed_edges = step_4_morphology(edges)

    processed_edges_rgb = cv2.cvtColor(
        processed_edges, cv2.COLOR_GRAY2BGR
    )  # step4 result
    # return contours, processed_edges_rgb, llpython

    processed_edges_combined = cv2.addWeighted(
        masked_image, 1, processed_edges_rgb, 0.5, 0
    )  # step4 Combine
    # return contours, processed_edges_combined, llpython

    # ========== Step 5: Find contours and get the largest rectangle ==========
    best_rect, best_contour, all_contours = step_5_contours(processed_edges)

    # 绘制所有检测到的轮廓
    image_with_contours = processed_edges_combined.copy()
    cv2.drawContours(image_with_contours, all_contours, -1, (0, 255, 0), 2)

    # 绘制最大的长方形
    if best_rect is not None:
        cv2.drawContours(image_with_contours, [best_rect], 0, (0, 0, 255), 2)

        # 计算中心点
        center = np.mean(best_rect, axis=0).astype(np.int32)
        cv2.circle(image_with_contours, tuple(center), 5, (0, 0, 255), -1)

        # 更新llpython数据
        llpython[0] = 1  # 检测标志
        llpython[1] = center[0]  # 中心点x坐标
        llpython[2] = center[1]  # 中心点y坐标

        # 返回长方形的角点坐标和原始轮廓
        return best_contour, image_with_contours, llpython

    # 如果没有检测到长方形，返回空数组
    return np.array([]), image_with_contours, llpython  # Final result with contours
