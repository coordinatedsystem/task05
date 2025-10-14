# split_dataset.py
import os
import shutil
import random
from pathlib import Path

def split_rm_armor_dataset(
    src_root: str,
    dst_root: str,
    train_ratio: float = 0.8,
    seed: int = 42
):  
    """
    自动划分 RoboMaster 装甲板数据集为 train/val
    
    Args:
        src_root: 原始数据根目录（包含 1/, 2/, ..., 9neg/）
        dst_root: 输出目录（将创建 train/ 和 val/）
        train_ratio: 训练集比例（默认 0.8）
        seed: 随机种子（保证可复现）
    """
    random.seed(seed)
    src_path = Path(src_root)
    dst_path = Path(dst_root)
    
    # 创建目标目录
    (dst_path / 'train').mkdir(parents=True, exist_ok=True)
    (dst_path / 'val').mkdir(parents=True, exist_ok=True)
    
    # 定义类别映射
    class_mapping = {
        '1': '1',
        '2': '2',
        '3': '3',
        '4': '4',
        '5': '5',
        '6outpost': '6',
        '7guard': '7',
        '8base': '8',
        '9neg': 'negative'
    }
    
    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff'}
    
    print("开始划分数据集...")
    
    for src_folder_name, dst_class_name in class_mapping.items():
        src_folder = src_path / src_folder_name
        if not src_folder.exists():
            print(f"⚠️ 警告：文件夹不存在 {src_folder}")
            continue
        
        # 获取所有图像文件
        image_files = [
            f for f in src_folder.iterdir()
            if f.is_file() and f.suffix.lower() in image_extensions
        ]
        
        if len(image_files) == 0:
            print(f"⚠️ 警告：{src_folder} 中没有图像文件")
            continue
        
        # 打乱顺序
        random.shuffle(image_files)
        
        # 划分训练集和验证集
        split_idx = int(len(image_files) * train_ratio)
        train_files = image_files[:split_idx]
        val_files = image_files[split_idx:]
        
        # 创建目标类别目录
        (dst_path / 'train' / dst_class_name).mkdir(exist_ok=True)
        (dst_path / 'val' / dst_class_name).mkdir(exist_ok=True)
        
        # 复制文件
        for f in train_files:
            shutil.copy(f, dst_path / 'train' / dst_class_name / f.name)
        
        for f in val_files:
            shutil.copy(f, dst_path / 'val' / dst_class_name / f.name)
        
        print(f" {src_folder_name} → {dst_class_name}: {len(train_files)} train, {len(val_files)} val")
    
    print(f"\n 数据集划分完成！")
    print(f"   训练集路径: {dst_path / 'train'}")
    print(f"   验证集路径: {dst_path / 'val'}")


# ======================
# 使用示例
# ======================
if __name__ == "__main__":
    # 请修改以下路径为您的实际路径
    SOURCE_ROOT = "/home/coordsys/zbx/MVSROS_project/MVS_ROS2/hik_camera/digit_recognition/train/datasets"         # 原始数据根目录
    DEST_ROOT = "/home/coordsys/zbx/MVSROS_project/MVS_ROS2/hik_camera/digit_recognition/train/splited_data"   # 输出目录（自动创建）
    
    split_rm_armor_dataset(
        src_root=SOURCE_ROOT,
        dst_root=DEST_ROOT,
        train_ratio=0.8,
        seed=42
    )