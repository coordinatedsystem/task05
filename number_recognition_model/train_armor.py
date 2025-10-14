# train_armor.py
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
import torchvision.transforms as T
import torchvision.datasets as datasets
from tqdm import tqdm
import os
import numpy as np
import cv2


# ================ 配置 ================
DATA_ROOT = "/home/coordsys/zbx/MVSROS_project/MVS_ROS2/hik_camera/number_recognition_model/train/splited_data"
MODEL_SAVE_PATH = "./models"
NUM_CLASSES = 9  # 1,2,3,4,5,6,7,8,negative → 9 类
BATCH_SIZE = 32
LEARNING_RATE = 0.001
NUM_EPOCHS = 20
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

os.makedirs(MODEL_SAVE_PATH, exist_ok=True)


# ================ 自定义变换：固定阈值二值化 ================
class BinarizeFixed(object):
    """
    使用固定阈值进行二值化（与C++推理一致）
    输入: PIL Image (灰度)
    输出: torch.Tensor (1, H, W), 值为 0.0 或 1.0
    """
    def __init__(self, threshold=100/255.0):
        self.threshold = threshold  # 190 对应 0.745

    def __call__(self, img):
        # 转为 numpy array
        img_np = np.array(img).astype(np.float32) / 255.0  # [0,255] -> [0.0,1.0]

        # 固定阈值二值化: > threshold ? 1.0 : 0.0
        binary = (img_np > self.threshold).astype(np.float32)

        # 转为 torch.Tensor 并增加通道维度
        tensor = torch.from_numpy(binary).unsqueeze(0)  # (H, W) -> (1, H, W)

        return tensor


# ================ 添加轻微噪声（模拟边缘抖动）================
class AddNoise(object):
    """添加轻微高斯噪声，防止过拟合，模拟二值图边缘抖动"""
    def __init__(self, std=0.03):
        self.std = std

    def __call__(self, tensor):
        noise = torch.randn(tensor.size()) * self.std
        return torch.clamp(tensor + noise, 0.0, 1.0)


# ================ 数据预处理 ================
transform = T.Compose([
    T.Resize((28, 20), interpolation=T.InterpolationMode.NEAREST),  # H=28, W=20
    T.Grayscale(num_output_channels=1),
    BinarizeFixed(threshold=190/255.0),     # ✅ 使用固定阈值 190
    AddNoise(std=0.03),
])

val_transform = transform  # 验证集同样处理


# ================ 加载数据集 ================
train_dataset = datasets.ImageFolder(
    root=os.path.join(DATA_ROOT, 'train'),
    transform=transform
)

val_dataset = datasets.ImageFolder(
    root=os.path.join(DATA_ROOT, 'val'),
    transform=val_transform
)

train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

print(f"✅ 训练集样本数: {len(train_dataset)}")
print(f"✅ 验证集样本数: {len(val_dataset)}")
print(f"✅ 类别: {train_dataset.classes}")


# ================ 定义适配 20x28 的 CNN 模型 ================
class ArmorNetV2(nn.Module):
    def __init__(self, num_classes=9):
        super(ArmorNetV2, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=3, padding=1),  # (1,28,20)
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),  # → (32,14,10)

            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),  # → (64,7,5)
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64 * 7 * 5, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        return self.classifier(x)


model = ArmorNetV2(num_classes=NUM_CLASSES).to(DEVICE)
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)


# ================ 训练循环 ================
best_val_acc = 0.0

for epoch in range(NUM_EPOCHS):
    model.train()
    running_loss = 0.0
    correct = 0
    total = 0

    for images, labels in tqdm(train_loader, desc=f"Epoch {epoch+1}/{NUM_EPOCHS}"):
        images, labels = images.to(DEVICE), labels.to(DEVICE)

        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item()
        _, predicted = outputs.max(1)
        total += labels.size(0)
        correct += predicted.eq(labels).sum().item()

    train_acc = 100. * correct / total

    # 验证阶段
    model.eval()
    val_correct = 0
    val_total = 0
    with torch.no_grad():
        for images, labels in val_loader:
            images, labels = images.to(DEVICE), labels.to(DEVICE)
            outputs = model(images)
            _, predicted = outputs.max(1)
            val_total += labels.size(0)
            val_correct += predicted.eq(labels).sum().item()

    val_acc = 100. * val_correct / val_total

    print(f"Train Loss: {running_loss:.3f} | Train Acc: {train_acc:.2f}% | Val Acc: {val_acc:.2f}%")

    # 保存最佳模型
    if val_acc > best_val_acc:
        best_val_acc = val_acc

        # 保存 state_dict
        torch.save(model.state_dict(), os.path.join(MODEL_SAVE_PATH, "best_armor_model.pth"))

        # 保存 TorchScript 模型 (.pt)，注意输入尺寸也变了！
        model.eval()
        example_input = torch.rand(1, 1, 28, 20).to(DEVICE)  # ✅ 改为 (1,28,20)
        traced_script_module = torch.jit.trace(model, example_input)
        pt_model_path = os.path.join(MODEL_SAVE_PATH, "best_armor_model.pt")
        traced_script_module.save(pt_model_path)

        print(f"⭐ 新最佳模型保存为 .pt 格式: {pt_model_path}")
        print(f"   验证准确率: {val_acc:.2f}%")

print(f"✅ 训练完成！最终最佳验证准确率: {best_val_acc:.2f}%")