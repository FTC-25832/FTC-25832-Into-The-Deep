import cv2
import numpy as np
import base64
import json
import os


def extract_orb_features(image_path, nfeatures=500):
    """
    从图像中提取ORB特征

    Args:
        image_path: 图像路径
        nfeatures: 要提取的特征点数量

    Returns:
        keypoints_dict: 关键点列表，每个点包含位置、大小和角度信息
        descriptors: 描述子数组
    """
    # 读取图像
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"无法读取图像: {image_path}")
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # 创建ORB特征检测器
    orb = cv2.ORB_create(nfeatures=nfeatures, scaleFactor=1.2, edgeThreshold=5)

    # 检测关键点和计算描述子
    keypoints, descriptors = orb.detectAndCompute(img, None)

    # 转换关键点为可序列化的字典
    keypoints_dict = []
    for kp in keypoints:
        keypoints_dict.append(
            {
                "x": float(kp.pt[0]),
                "y": float(kp.pt[1]),
                "size": float(kp.size),
                "angle": float(kp.angle),
                "response": float(kp.response),
                "octave": int(kp.octave),
                "class_id": int(kp.class_id),
            }
        )

    return keypoints_dict, descriptors


def convert_features_to_code_string(
    keypoints, descriptors, var_name="TEMPLATE_FEATURES"
):
    """
    将特征转换为可嵌入代码的字符串

    Args:
        keypoints: 关键点字典列表
        descriptors: 描述子数组
        var_name: 在代码中使用的变量名

    Returns:
        code_string: 可嵌入Python代码的字符串
    """
    # 将关键点和描述子转换为JSON
    features_dict = {
        "keypoints": keypoints,
        "descriptors": descriptors.tolist() if descriptors is not None else None,
    }

    # 将JSON转换为字符串
    json_str = json.dumps(features_dict)

    # 使用base64编码以减小大小并避免特殊字符问题
    encoded_bytes = base64.b64encode(json_str.encode("utf-8"))
    encoded_str = encoded_bytes.decode("utf-8")

    # 创建变量赋值语句
    code_string = f'{var_name} = """{encoded_str}"""\n'

    # 添加解码代码
    code_string += f"""
import base64
import json
import numpy as np
import cv2


# 解码恢复特征数据的函数
def decode():

    # 解码base64字符串
    decoded_bytes = base64.b64decode({var_name})
    features_dict = json.loads(decoded_bytes)

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
    descriptors = np.array(features_dict["descriptors"], dtype=np.uint8)

    return keypoints, descriptors


def save_features_to_file(keypoints, descriptors, output_path):
    keypoints_list = []
    for kp in keypoints:
        keypoints_list.append(
            {{
                "x": float(kp.pt[0]),
                "y": float(kp.pt[1]),
                "size": float(kp.size),
                "angle": float(kp.angle),
                "response": float(kp.response),
                "octave": int(kp.octave),
                "class_id": int(kp.class_id),
            }}
        )

    # 创建特征字典
    features_dict = {{"keypoints": keypoints_list, "descriptors": descriptors.tolist()}}

    # 将特征保存为JSON
    with open(output_path, "w") as f:
        json.dump(features_dict, f)

    return True


keypoints, descriptors = decode()

save_features_to_file(keypoints, descriptors, "{var_name}.json")


def runpipeline(image, llrobot):
    return np.array([]), image, [0, 0, 0, 0, 0, 0, 0, 0]

"""

    return code_string


def generate_feature_code_file(
    image_path, output_code_path, var_name="TEMPLATE_FEATURES", nfeatures=500
):
    """
    生成包含嵌入式特征数据的Python代码文件

    Args:
        image_path: 输入图像路径
        output_code_path: 输出Python代码文件路径
        var_name: 在代码中使用的特征变量名
        nfeatures: 要提取的特征点数量
    """
    # 提取特征
    keypoints, descriptors = extract_orb_features(image_path, nfeatures)

    # 转换为代码字符串
    code_string = convert_features_to_code_string(keypoints, descriptors, var_name)

    # 写入文件
    with open(output_code_path, "w", encoding="utf-8") as f:
        f.write(code_string)

    print(f"特征代码已保存到 {output_code_path}")
    print(f"特征数据大小: 约 {len(code_string)/1024:.2f} KB")


# 使用示例
if __name__ == "__main__":
    import sys

    # add current path
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    # 替换为你的模板图像路径
    image_path = "side.jpg"
    output_code_path = "side_features.py"

    generate_feature_code_file(
        image_path=image_path,
        output_code_path=output_code_path,
        var_name="side",
        nfeatures=500,
    )
