import os
import json
import torch
from torch.utils.data import Dataset
from PIL import Image
from torchvision import transforms

class CarotidDataset(Dataset):
    def __init__(self, root_dir, split='train', transform=None):
        """
        Args:
            root_dir (str): 数据集根目录 (例如 './carotid_dataset_v1')
            split (str): 'train', 'val', 或 'test'
            transform (callable): 图像预处理
        """
        self.root_dir = os.path.join(root_dir, split)
        self.transform = transform
        self.samples = [] # 存储 (image_path, label_int)

        self._load_data()

    def _load_data(self):
        # 遍历 split 目录下的所有序列文件夹
        if not os.path.exists(self.root_dir):
            print(f"警告: 找不到目录 {self.root_dir}")
            return

        seq_dirs = [d for d in os.listdir(self.root_dir) 
                    if os.path.isdir(os.path.join(self.root_dir, d))]
        
        for seq_name in seq_dirs:
            seq_path = os.path.join(self.root_dir, seq_name)
            json_path = os.path.join(seq_path, 'label_carotid.json')
            
            # 读取 Label JSON
            try:
                with open(json_path, 'r', encoding='utf-8') as f:
                    label_data = json.load(f)
                    # 假设我们只需要文件名和标签对应关系
                    # 也可以直接用 trigger_start/end 算，但用 frames 列表更直接
                    frames_info = label_data.get('frames', [])
                    
                    for item in frames_info:
                        img_name = item['name']
                        label = int(item['label']) # 0, 1, 2
                        
                        img_path = os.path.join(seq_path, 'images', img_name)
                        if os.path.exists(img_path):
                            self.samples.append((img_path, label))
            except Exception as e:
                print(f"Error loading {seq_name}: {e}")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, label = self.samples[idx]
        
        # 打开图像 (转为 RGB，防止部分单通道图报错)
        image = Image.open(img_path).convert('RGB')
        
        if self.transform:
            image = self.transform(image)
            
        return image, label

    # 用于计算类别权重
    def get_class_counts(self):
        counts = {0: 0, 1: 0, 2: 0}
        for _, label in self.samples:
            counts[label] += 1
        return counts