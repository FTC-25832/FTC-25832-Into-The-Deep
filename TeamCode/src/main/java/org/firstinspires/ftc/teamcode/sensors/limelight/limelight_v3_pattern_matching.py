# Limelight长方体目标检测程序 - 从文件读取特征数据
# 作者: pubuyun
# 日期: 2025-06-08

import cv2
import numpy as np
import math
import json
import os

# ==========================================
# 特征数据加载函数
# ==========================================

# 全局变量用于存储解码后的特征
front_keypoints = None
front_descriptors = None
side_keypoints = None
side_descriptors = None
sides_loaded = False  # 是否加载了侧面特征
features_initialized = False  # 是否初始化了特征


def load_features_from_file(filename):
    """
    从JSON文件加载特征数据

    Args:
        filename: JSON文件路径

    Returns:
        keypoints, descriptors: OpenCV关键点和描述子
    """
    try:
        with open(filename, "r", encoding="utf-8") as f:
            features_dict = json.load(f)

        # 恢复关键点
        keypoints = []
        for kp_dict in features_dict["keypoints"]:
            kp = cv2.KeyPoint(
                x=kp_dict["x"],
                y=kp_dict["y"],
                size=kp_dict["size"],
                angle=kp_dict["angle"],
                response=kp_dict["response"],
                octave=kp_dict["octave"],
                class_id=kp_dict["class_id"],
            )
            keypoints.append(kp)

        # 恢复描述子
        descriptors = None
        if features_dict["descriptors"] is not None:
            descriptors = np.array(features_dict["descriptors"], dtype=np.uint8)

        return keypoints, descriptors
    except Exception as e:
        print(f"加载特征文件失败: {e}")
        return [], None


def initialize_features():
    """初始化特征数据，从文件加载"""
    global front_keypoints, front_descriptors, side_keypoints, side_descriptors, features_initialized, sides_loaded

    if not features_initialized:
        # 尝试加载正面特征
        front_file = "front.json"
        if os.path.exists(front_file):
            front_keypoints, front_descriptors = load_features_from_file(front_file)
            print(f"已加载正面特征: {len(front_keypoints)} 个关键点")

        # 尝试加载侧面特征
        side_file = "side.json"
        if os.path.exists(side_file):
            side_keypoints, side_descriptors = load_features_from_file(side_file)
            print(f"已加载侧面特征: {len(side_keypoints)} 个关键点")
            sides_loaded = True

        features_initialized = True


# ==========================================
# 长方体检测函数
# ==========================================


def match_template(
    image_keypoints,
    image_descriptors,
    template_keypoints,
    template_descriptors,
    min_match_count=200,
):
    """
    匹配模板特征与图像特征

    Returns:
        成功匹配返回 (True, H, good_matches, mask)，失败返回 (False, None, None, None)
    """
    if template_descriptors is None or image_descriptors is None:
        return False, None, None, None

    if (
        len(template_keypoints) < min_match_count
        or len(image_keypoints) < min_match_count
    ):
        return False, None, None, None

    # 创建特征匹配器
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # 匹配特征
    matches = matcher.match(template_descriptors, image_descriptors)

    # 按距离排序
    matches = sorted(matches, key=lambda x: x.distance)

    # 筛选好的匹配
    good_matches = matches[:min_match_count] if len(matches) >= min_match_count else []

    if len(good_matches) >= min_match_count:
        # 提取匹配点坐标
        src_pts = np.float32(
            [template_keypoints[m.queryIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)
        dst_pts = np.float32(
            [image_keypoints[m.trainIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)

        # 使用RANSAC计算单应性矩阵
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if H is not None:
            return True, H, good_matches, mask

    return False, None, None, None


def detect_box_face(
    image,
    image_keypoints,
    image_descriptors,
    template_keypoints,
    template_descriptors,
    face_name="face",
    color=(0, 255, 0),
    min_match_count=10,
    template_size=(200, 100),
):
    """
    检测长方体的一个面

    Args:
        image: 输入图像
        image_keypoints, image_descriptors: 图像的特征
        template_keypoints, template_descriptors: 模板的特征
        face_name: 面的名称
        color: 显示颜色
        min_match_count: 最少匹配点数
        template_size: 模板尺寸 (宽,高)

    Returns:
        检测成功返回(dict, 标注后图像)，失败返回(None, 原图)
    """
    annotated_image = np.copy(image)

    # 匹配模板
    success, H, good_matches, mask = match_template(
        image_keypoints,
        image_descriptors,
        template_keypoints,
        template_descriptors,
        min_match_count,
    )

    if success:
        # 创建模板矩形
        w, h = template_size
        corners = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(
            -1, 1, 2
        )

        # 变换矩形角点到目标图像中
        transformed_corners = cv2.perspectiveTransform(corners, H)
        contour = np.int32(transformed_corners)

        # 计算中心位置
        center_x = np.mean(transformed_corners[:, 0, 0])
        center_y = np.mean(transformed_corners[:, 0, 1])

        # 计算旋转角度(yaw)
        dx = transformed_corners[3, 0, 0] - transformed_corners[0, 0, 0]
        dy = transformed_corners[3, 0, 1] - transformed_corners[0, 0, 1]
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)

        # 在图像上绘制轮廓
        cv2.polylines(annotated_image, [contour], True, color, 2)

        # 绘制中心点
        center_point = (int(center_x), int(center_y))
        cv2.circle(annotated_image, center_point, 5, (0, 0, 255), -1)

        # 绘制方向指示线
        line_length = 30
        end_x = int(center_x + line_length * math.cos(angle_rad))
        end_y = int(center_y + line_length * math.sin(angle_rad))
        cv2.line(annotated_image, center_point, (end_x, end_y), (255, 0, 0), 2)

        # 添加面的名称和角度文本
        cv2.putText(
            annotated_image,
            f"{face_name}: {angle_deg:.1f}°",
            (int(center_x) + 10, int(center_y) + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            2,
        )

        # 计算置信度
        inlier_count = np.sum(mask) if mask is not None else len(good_matches)
        confidence = min(100, inlier_count * 5)

        # 返回检测结果
        result = {
            "face": face_name,
            "contour": contour,
            "position": (center_x, center_y),
            "rotation": angle_deg,
            "confidence": confidence,
            "match_count": len(good_matches),
            "width": max(
                abs(transformed_corners[0, 0, 0] - transformed_corners[2, 0, 0]),
                abs(transformed_corners[1, 0, 0] - transformed_corners[3, 0, 0]),
            ),
            "height": max(
                abs(transformed_corners[0, 0, 1] - transformed_corners[2, 0, 1]),
                abs(transformed_corners[1, 0, 1] - transformed_corners[3, 0, 1]),
            ),
        }

        return result, annotated_image

    return None, annotated_image


# ==========================================
# 完整长方体姿态估计
# ==========================================


def estimate_box_pose(detections):
    """
    基于检测到的各个面估计长方体的完整位姿

    Args:
        detections: 检测到的各个面信息

    Returns:
        pose_info: 长方体姿态信息
    """
    if not detections:
        return None

    # 如果只有一个面被检测到，直接返回其信息
    if len(detections) == 1:
        det = detections[0]
        return {
            "position": det["position"],
            "yaw": det["rotation"],
            "confidence": det["confidence"],
            "visible_faces": [det["face"]],
        }

    # 如果检测到多个面，使用最高置信度的面作为主要参考
    detections.sort(key=lambda x: x["confidence"], reverse=True)
    primary_det = detections[0]

    # 收集可见面
    visible_faces = [det["face"] for det in detections]

    # 基于多个面的信息，可以更准确地估计姿态
    # 这里使用简化计算，实际应用中可能需要更复杂的3D姿态估计
    position_x = np.mean([det["position"][0] for det in detections])
    position_y = np.mean([det["position"][1] for det in detections])

    # 使用主要面的旋转作为yaw角度
    yaw = primary_det["rotation"]

    # 置信度使用最高值
    confidence = primary_det["confidence"]

    return {
        "position": (position_x, position_y),
        "yaw": yaw,
        "confidence": confidence,
        "visible_faces": visible_faces,
    }


# ==========================================
# Limelight主处理Pipeline
# ==========================================


def runPipeline(image, llrobot):
    """
    Limelight主处理Pipeline函数

    Args:
        image: 输入图像
        llrobot: 机器人数据

    Returns:
        contour: 检测到的最大轮廓
        annotated_image: 带注释的图像
        llpython: 包含位置和旋转信息的8元素数组
    """
    # 初始化输出
    largestContour = []
    llpython = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始化8元素数组

    # 确保特征已加载
    initialize_features()

    # 复制图像用于标注
    annotated_image = np.copy(image)

    # 转换为灰度图像并检测特征
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 创建ORB检测器
    orb = cv2.ORB_create(
        nfeatures=500,
        scaleFactor=1.2,
        nlevels=8,
        edgeThreshold=31,
        firstLevel=0,
        WTA_K=2,
        patchSize=31,
        fastThreshold=20,
    )

    # 检测图像特征
    image_keypoints, image_descriptors = orb.detectAndCompute(gray, None)

    # 存储所有检测结果
    detections = []

    # 检测正面
    if front_keypoints and len(front_keypoints) > 0:
        front_result, annotated_image = detect_box_face(
            image,
            image_keypoints,
            image_descriptors,
            front_keypoints,
            front_descriptors,
            face_name="Front",
            color=(0, 255, 0),
            template_size=(142, 145),  # 预估的正面尺寸
        )
        if front_result:
            detections.append(front_result)

    # 检测侧面 (如果有侧面特征)
    if sides_loaded and side_keypoints and len(side_keypoints) > 0:
        side_result, annotated_image = detect_box_face(
            annotated_image,
            image_keypoints,
            image_descriptors,
            side_keypoints,
            side_descriptors,
            face_name="Side",
            color=(0, 200, 200),
            template_size=(372, 1590),  # 预估的侧面尺寸
        )
        if side_result:
            detections.append(side_result)

    # 基于检测到的面估计长方体姿态
    pose = estimate_box_pose(detections)

    # 如果成功估计姿态，填充输出数组
    if pose:
        # 使用最高置信度的检测结果作为轮廓输出
        largestContour = detections[0]["contour"]

        # 姿态信息
        center_x, center_y = pose["position"]
        yaw = pose["yaw"]
        confidence = pose["confidence"]

        # 绘制长方体中心和姿态
        cv2.putText(
            annotated_image,
            f"Box: Yaw={yaw:.1f}°",
            (int(center_x) - 50, int(center_y) - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

        # 填充输出数组:
        # [0] = x位置(归一化到-1到1，其中0是中心)
        # [1] = y位置(归一化到-1到1，其中0是中心)
        # [2] = 旋转角度(度)
        # [3] = 检测到的面数量
        # [4] = 检测到的物体宽度
        # [5] = 置信度(0-100)
        # [6-7] = 保留用于附加数据

        # 归一化坐标到-1到1范围
        img_height, img_width = image.shape[:2]
        norm_x = (center_x - img_width / 2) / (img_width / 2)
        norm_y = (center_y - img_height / 2) / (img_height / 2)

        # 估算物体宽度 (取最高置信度的检测结果)
        width = detections[0]["width"]
        height = detections[0]["height"]

        # 填充输出数组
        llpython[0] = float(norm_x)
        llpython[1] = float(norm_y)
        llpython[2] = float(yaw)
        llpython[3] = float(len(detections))  # 检测到的面数量
        llpython[4] = float(width)
        llpython[5] = float(confidence)

        # 如果检测到多个面，额外输出第二个面的信息
        if len(detections) > 1:
            # 将第二个面的角度输出到第7个元素
            llpython[6] = float(detections[1]["rotation"])
            llpython[7] = float(detections[1]["confidence"])

    return largestContour, annotated_image, llpython
