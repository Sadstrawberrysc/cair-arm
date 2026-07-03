import os
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset as BaseDataset

class MySegmentationDataset(BaseDataset):
    def __init__(self, images_dir, masks_dir, augmentation=None, preprocessing=None):
        self.ids = os.listdir(images_dir)
        # 排序确保一一对应
        self.ids.sort() 
        self.images_fps = [os.path.join(images_dir, image_id) for image_id in self.ids]

        # Mask 路径
        self.masks_fps = []
        for image_id in self.ids:
            # 1. 去掉图片后缀 (变成 "car")
            filename_no_ext = os.path.splitext(image_id)[0]
            # 2. 加上 mask 后缀 (变成 "car_mask.png")
            mask_name = filename_no_ext + '_mask.png'
            # 3. 拼路径
            self.masks_fps.append(os.path.join(masks_dir, mask_name))

    
    def __getitem__(self, i):
        # 1. 读取图像
        image = cv2.imread(self.images_fps[i])
        if image is None:
            print(f"❌ 严重错误！无法读取图片，请检查路径: {self.images_fps[i]}")
            # 为了防止程序直接崩掉看不清报错，这里可以抛出一个更清晰的异常
            raise ValueError(f"图片读取失败: {self.images_fps[i]}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 2. 读取 Mask
        # flag=0 读取为灰度，此时像素值就是 0, 1, 2
        mask = cv2.imread(self.masks_fps[i], 0) 

        # ### 【关键变化】手动转换为 Tensor ###
        # 图片：转为 Float, 形状变 (3, H, W)
        image = image.transpose(2, 0, 1).astype('float32') / 255.0
        
        # Mask：必须是 Long (整型), 形状保持 (H, W), 不要加通道维度！
        # 严禁除以 255，否则 1 和 2 会变成小数
        mask = mask.astype('long') 
            
        return torch.from_numpy(image), torch.from_numpy(mask)
        
    def __len__(self):
        return len(self.ids)
