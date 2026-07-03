import torch
import torch.nn as nn
import torch.nn.functional as F
import math

# [在此处插入你的 PoseHead4 类代码，稍微修改 Head 部分]
class PoseHead4(nn.Module):
    def __init__(self, in_dim=768, hidden_dim=256, out_dim=3,  # <--- 修改默认 out_dim 为 3
                 dropout=0.2, **kwargs):
        super().__init__()
        # ... (前面的初始化代码保持不变) ...
        # 为了节省篇幅，省略中间重复部分，只展示修改的 Head
        
        self.use_spatial_attn = kwargs.get('use_spatial_attn', False)
        self.patch_drop_prob = float(kwargs.get('patch_drop_prob', 0.1))
        self.use_grid = kwargs.get('use_grid', True)
        self.use_dilation = kwargs.get('use_dilation', True)
        self.use_coord = kwargs.get('use_coord', True)
        num_heads = kwargs.get('num_heads', 4)

        self.cls_norm = nn.LayerNorm(in_dim)
        self.patch_norm = nn.LayerNorm(in_dim)
        self.cls_proj = nn.Linear(in_dim, hidden_dim)
        self.patch_proj = nn.Linear(in_dim, hidden_dim)

        # ... (中间的 attention, grid 逻辑保持完全一致) ...
        # (请确保复制你原始代码中 _init_weights, _make_grid 等所有辅助函数)
        
        # 假设前面所有层都初始化好了 (attn, coord_proj, etc.)
        # 这里为了代码运行完整性，简写一下结构，实际请保留你的完整代码
        if self.use_spatial_attn:
             self.attn_ln = nn.LayerNorm(hidden_dim)
             self.attn = nn.MultiheadAttention(hidden_dim, num_heads, dropout=dropout, batch_first=True)
        
        if self.use_grid:
            in_ch = hidden_dim + (2 if self.use_coord else 0)
            self.coord_proj = nn.Conv2d(in_ch, hidden_dim, 1)
            self.dw3 = nn.Conv2d(hidden_dim, hidden_dim, 3, 1, 1, groups=hidden_dim)
            if self.use_dilation:
                self.dw3_dil = nn.Conv2d(hidden_dim, hidden_dim, 3, 2, 2, dilation=2, groups=hidden_dim)
            self.pw1 = nn.Conv2d(hidden_dim, hidden_dim, 1)
            self.spatial_act = nn.GELU()
            self.spatial_drop = nn.Dropout(dropout)
            self.register_buffer('grid_xy', self._make_grid(14, 14), persistent=False)

        self.pool_reduce = nn.Linear(2 * hidden_dim, hidden_dim)
        self.fuse = nn.Linear(2 * hidden_dim, hidden_dim)

        # --- [关键修改] ---
        # 你的原始代码用了 Sigmoid，这对多分类互斥任务是不对的。
        # 这里我们去掉 Sigmoid，直接输出 Logits，配合 CrossEntropyLoss 使用。
        self.head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, out_dim) 
            # <--- 移除了 nn.Sigmoid()
        )
        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None: nn.init.zeros_(m.bias)
            elif isinstance(m, nn.LayerNorm):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, nonlinearity='linear')
                if m.bias is not None: nn.init.zeros_(m.bias)
    
    def _make_grid(self, H, W):
        y, x = torch.meshgrid(torch.linspace(-1, 1, H), torch.linspace(-1, 1, W), indexing='ij')
        return torch.stack([x, y], dim=0).unsqueeze(0)

    # ... Forward 函数保持不变，确保最后 return out ...
    def forward(self, features, x=None):
        # (请粘贴你原始的 forward 代码)
        # 简略版：
        B, T, C = features.shape
        # ... logic ...
        # 为了演示，假设经过一系列复杂计算得到了 z
        # 这里模拟一下流程，实际请使用你的完整逻辑
        # 重点是 input shape 必须是 (B, 197, 768)
        
        # 模拟 Head 输出
        # 在真实代码中，请用你原来的 logic 计算 patch_z 和 cls_z
        # 这里仅为示意 wrapper 怎么写
        return self.head(torch.randn(B, 256).to(features.device)) # Replace with real logic

# --- 完整模型包装器 ---
class CarotidModel(nn.Module):
    def __init__(self, backbone_name='facebook/vit-mae-base', num_classes=3):
        super().__init__()
        
        # 1. 加载 Backbone (使用 HuggingFace Transformers 库为例)
        # 如果你有本地 .pth 权重，可以用 timm 或 torch.load 加载
        from transformers import ViTMAEModel
        print(f"Loading Backbone: {backbone_name} ...")
        self.backbone = ViTMAEModel.from_pretrained(backbone_name)
        
        # 冻结 Backbone? (建议先冻结，训练几个 epoch 后再解冻，或者只训练最后几层)
        # for param in self.backbone.parameters():
        #     param.requires_grad = False
            
        # 2. 加载 PoseHead
        # MAE Base 输出通常是 768 维，14x14 patches + 1 CLS = 197 tokens
        self.head = PoseHead4(
            in_dim=768, 
            hidden_dim=256, 
            out_dim=num_classes, # 3类
            use_spatial_attn=True,
            use_grid=True
        )

    def forward(self, x):
        # x: [Batch, 3, 224, 224]
        
        # Backbone Forward
        outputs = self.backbone(x)
        
        # last_hidden_state shape: [Batch, 197, 768]
        features = outputs.last_hidden_state
        
        # Head Forward
        logits = self.head(features) # [Batch, 3]
        
        return logits