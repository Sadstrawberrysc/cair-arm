import torch
import os
from torch.utils.data import DataLoader
import segmentation_models_pytorch as smp
from dataset import MySegmentationDataset

# --- 1. 配置参数 ---
DATA_DIR = './'
x_train_dir = os.path.join(DATA_DIR, 'images')
y_train_dir = os.path.join(DATA_DIR, 'masks')

ENCODER = 'resnet34'
ENCODER_WEIGHTS = 'imagenet'
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# ### 【变化】定义类别数 ###
# 背景(0) + 类别1(1) + 类别2(2) = 3类
N_CLASSES = 3 

# --- 2. 准备模型 ---
model = smp.Unet(
    encoder_name=ENCODER, 
    encoder_weights=ENCODER_WEIGHTS, 
    # ### 【变化】输出通道设为 3 ###
    classes=N_CLASSES, 
    # ### 【变化】多分类通常输出 Logits (原始值)，由 Loss 函数处理 Softmax
    activation=None, 
)

# --- 3. 准备数据 ---

# 因为不再使用额外的预处理函数，这里直接传入路径即可
# ⚠️ 注意：这要求你的 dataset.py 里必须自己处理了 [转置 HWC->CHW] 和 [归一化 /255.0]
train_dataset = MySegmentationDataset(
    x_train_dir, 
    y_train_dir
)

# batch_size 如果显存不够就改小 (比如 4 或 2)
train_loader = DataLoader(train_dataset, batch_size=8, shuffle=True, num_workers=0)

# --- 4. 定义损失函数和优化器 ---

# ### 【变化】使用交叉熵损失 (专门用于多分类 0,1,2...) ###
loss_fn = torch.nn.CrossEntropyLoss()

optimizer = torch.optim.Adam([ 
    dict(params=model.parameters(), lr=0.0001),
])

# --- 5. 训练循环 ---
model.to(DEVICE)
model.train()

print(f"开始训练... 设备: {DEVICE}, 类别数: {N_CLASSES}")

for epoch in range(20): 
    for i, (images, masks) in enumerate(train_loader):
        images = images.to(DEVICE)
        masks = masks.to(DEVICE) # 这里的 masks 应该是 long 类型, 形状 [Batch, H, W]
        
        optimizer.zero_grad()
        
        # Forward
        outputs = model(images) # 输出形状 [Batch, 3, H, W]
        
        # Calculate Loss
        # CrossEntropyLoss 要求: 
        # input=(Batch, C, H, W), target=(Batch, H, W) 且 target 是 long 类型
        loss = loss_fn(outputs, masks)
        
        # Backward
        loss.backward()
        optimizer.step()
        
        if i % 10 == 0:
            print(f"Epoch {epoch}, Step {i}, Loss: {loss.item():.4f}")

    # 保存模型
    torch.save(model.state_dict(), './latest_model.pth')

print("训练完成！")