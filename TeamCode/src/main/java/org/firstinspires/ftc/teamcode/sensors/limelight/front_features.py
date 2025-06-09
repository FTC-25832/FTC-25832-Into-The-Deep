front = """eyJrZXlwb2ludHMiOiBbeyJ4IjogMjM0LjAsICJ5IjogNi4wLCAic2l6ZSI6IDMxLjAsICJhbmdsZSI6IDk3Ljg5ODYxMjk3NjA3NDIyLCAicmVzcG9uc2UiOiAxLjUyNjQ5NjI3MTI1MzU2MzVlLTA1LCAib2N0YXZlIjogMCwgImNsYXNzX2lkIjogLTF9XSwgImRlc2NyaXB0b3JzIjogW1syMDMsIDIyMiwgMTI3LCAyNTIsIDEyNSwgMjEzLCAyNDksIDExOSwgMjA1LCA5NCwgMjQ0LCAzOSwgMTcyLCAxNTgsIDE4MiwgMTAxLCAyMzgsIDE3OSwgMTU5LCAxMzMsIDE2NCwgMTc1LCA1MCwgMjIyLCAyNDcsIDEyMywgMTU0LCAxMzksIDIzLCAxNDAsIDM3LCAyNTVdXX0="""

import base64
import json
import numpy as np
import cv2


# 解码恢复特征数据的函数
def decode():

    # 解码base64字符串
    decoded_bytes = base64.b64decode(front)
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

    # 创建特征字典
    features_dict = {"keypoints": keypoints_list, "descriptors": descriptors.tolist()}

    # 将特征保存为JSON
    with open(output_path, "w") as f:
        json.dump(features_dict, f)

    return True


keypoints, descriptors = decode()

save_features_to_file(keypoints, descriptors, "front.json")


def runpipeline(image, llrobot):
    return np.array([]), image, [0, 0, 0, 0, 0, 0, 0, 0]
