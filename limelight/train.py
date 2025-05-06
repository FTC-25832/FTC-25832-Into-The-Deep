from ultralytics import YOLO

def train_yolo_pose():
    # 加载YOLOv8n-pose模型
    model = YOLO('yolov8n-pose.pt')  # 从预训练的姿态检测模型开始
    
    # 训练模型
    results = model.train(
        data='data.yaml',           # 数据集配置文件路径
        epochs=100,                 # 训练轮数
        imgsz=640,                 # 输入图像大小
        batch=16,                  # 批次大小
        device='0',                # 使用GPU设备0
        patience=50,               # 早停patience
        save=True,                 # 保存模型
        project='runs/train',      # 保存结果的项目文件夹
        name='yolov8-pose',       # 实验名称
        exist_ok=True,            # 允许覆盖已存在的实验文件夹
        pretrained=True,          # 使用预训练权重
        optimizer='AdamW',        # 使用AdamW优化器
        lr0=0.001,                # 初始学习率
        lrf=0.01,                 # 最终学习率 = lr0 * lrf
        momentum=0.937,           # SGD动量
        weight_decay=0.0005,      # 权重衰减系数
        warmup_epochs=3.0,        # 预热轮数
        warmup_momentum=0.8,      # 预热动量
        warmup_bias_lr=0.1,       # 预热偏置学习率
        box=7.5,                  # 边界框损失权重
        cls=0.5,                  # 类别损失权重
        dfl=1.5,                  # DFL损失权重
        pose=12.0,                # 姿态损失权重
        kobj=2.0,                 # 关键点目标损失权重
        label_smoothing=0.0,      # 标签平滑
        nbs=64,                   # 标称批次大小
        overlap_mask=True,        # 掩码重叠
        mask_ratio=4,             # 掩码下采样比例
        dropout=0.0,              # 使用dropout正则化
        val=True,                 # 验证评估
    )
    
    return results

if __name__ == '__main__':
    results = train_yolo_pose()
    print("Training completed. Results:", results)
