import math
import torch
import torch.nn as nn
import torch.nn.functional as F
import warnings
from torch.utils.checkpoint import checkpoint

import torch
import torch.nn as nn
import torch.nn.functional as F

import torch.nn as nn
from torch.nn.parameter import UninitializedParameter



class RESNetHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.2, hidden_dim=512):
        super(RESNetHead, self).__init__()
        # 输入层归一化
        self.bn1 = nn.BatchNorm1d(in_dim)
        
        # 第一个残差块：in_dim -> hidden_dim
        self.fc1 = nn.Linear(in_dim, hidden_dim)
        self.bn2 = nn.BatchNorm1d(hidden_dim)
        self.dropout1 = nn.Dropout(dropout)
        
        # 第二个残差块：hidden_dim -> hidden_dim
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.bn3 = nn.BatchNorm1d(hidden_dim)
        self.dropout2 = nn.Dropout(dropout)
        
        # 用于跳连的投影层（当输入维度不等于隐藏维度时）
        self.shortcut_proj = nn.Linear(in_dim, hidden_dim) if in_dim != hidden_dim else None
        
        # 输出层
        self.fc3 = nn.Linear(hidden_dim, out_dim)
        
        # 权重初始化
        nn.init.normal_(self.fc1.weight, std=0.01)
        nn.init.constant_(self.fc1.bias, 0)
        nn.init.normal_(self.fc2.weight, std=0.01)
        nn.init.constant_(self.fc2.bias, 0)
        nn.init.normal_(self.fc3.weight, std=0.01)
        nn.init.constant_(self.fc3.bias, 0)
        if self.shortcut_proj:
            nn.init.normal_(self.shortcut_proj.weight, std=0.01)
            nn.init.constant_(self.shortcut_proj.bias, 0)

    def forward(self, x):
        # x shape: [Batch, 768]
        identity_input = x
        
        # 输入层：bn
        x = self.bn1(x)
        
        # 第一个残差块
        # 投影输入用于跳连
        if self.shortcut_proj:
            shortcut = self.shortcut_proj(identity_input)
        else:
            shortcut = identity_input
        
        x = self.fc1(x)
        x = self.bn2(x)
        x = F.relu(x)
        
        # 第二个残差块 + 跳连
        identity = x
        x = self.fc2(x)
        x = self.bn3(x)
        x = x + identity  # 残差连接
        x = F.relu(x)
        x = self.dropout2(x)
        
        # 输出层
        x = self.fc3(x)
        return x

class PhaseHead(nn.Module):
    def __init__(self, in_dim=768, out_dim=3, dropout=0.5):
        super(PhaseHead, self).__init__()
        self.bn = nn.BatchNorm1d(in_dim)
        self.dropout = nn.Dropout(dropout)
        self.fc = nn.Linear(in_dim, out_dim)

    def forward(self, x):
        x = self.bn(x)
        x = self.dropout(x)
        x = self.fc(x)
        return x

class PoseHead(nn.Module):
    def __init__(
        self,
        in_dim=768,
        hidden_dim=256,
        out_dim=2,
        use_spatial_attn=False, 
        num_heads=4,
        patch_drop_prob=0.1,
        dropout=0.2,
        use_grid=True, 
        use_dilation=True, 
        use_coord=True 
    ):
        super().__init__()
        self.use_spatial_attn = use_spatial_attn
        self.patch_drop_prob = float(patch_drop_prob)
        self.use_grid = use_grid
        self.use_dilation = use_dilation
        self.use_coord = use_coord

        self.cls_norm = nn.LayerNorm(in_dim)
        self.patch_norm = nn.LayerNorm(in_dim)
        self.cls_proj = nn.Linear(in_dim, hidden_dim)
        self.patch_proj = nn.Linear(in_dim, hidden_dim)

        if use_spatial_attn:
            self.attn_ln = nn.LayerNorm(hidden_dim)
            self.attn = nn.MultiheadAttention(
                embed_dim=hidden_dim, num_heads=num_heads,
                dropout=dropout, batch_first=True
            )

        if use_grid:
            in_ch_spatial = hidden_dim + (2 if use_coord else 0)
            self.coord_proj = nn.Conv2d(in_ch_spatial, hidden_dim, kernel_size=1, bias=True)

            self.dw3 = nn.Conv2d(hidden_dim, hidden_dim, kernel_size=3, padding=1,
                                  groups=hidden_dim, bias=True)          
            if use_dilation:
                self.dw3_dil = nn.Conv2d(hidden_dim, hidden_dim, kernel_size=3,
                                          padding=2, dilation=2,
                                          groups=hidden_dim, bias=True)     
            self.pw1 = nn.Conv2d(hidden_dim, hidden_dim, kernel_size=1, bias=True) 
            self.spatial_act = nn.GELU()
            self.spatial_drop = nn.Dropout(dropout)

        self.pool_reduce = nn.Linear(2 * hidden_dim, hidden_dim)
        self.fuse = nn.Linear(2 * hidden_dim, hidden_dim)
        self.head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, out_dim),
            nn.Sigmoid()
        )

        self._init_weights()
        if use_grid:
            self.register_buffer('grid_xy', self._make_grid(14, 14), persistent=False)

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None: nn.init.zeros_(m.bias)
            elif isinstance(m, nn.LayerNorm):
                nn.init.ones_(m.weight); nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, nonlinearity='linear')
                if m.bias is not None: nn.init.zeros_(m.bias)

    def _make_grid(self, H, W):
        y, x = torch.meshgrid(
            torch.linspace(-1, 1, H),
            torch.linspace(-1, 1, W),
            indexing='ij'
        )
        return torch.stack([x, y], dim=0).unsqueeze(0)

    def forward(self, features, x=None):

        B, T, C = features.shape
        has_cls = (T == 197)

        if has_cls:
            cls_feat = x[:, 0, :]
            patches  = features[:, 1:, :]
        else:
            cls_feat = None
            patches  = features

        cls_z = None
        if cls_feat is not None:
            cls_z = self.cls_proj(self.cls_norm(cls_feat)) 
        p = self.patch_proj(self.patch_norm(patches)) 


        key_padding_mask = None
        if self.training and self.patch_drop_prob > 0.0 and p.size(1) > 0:
            keep = torch.bernoulli(
                p.new_full((B, p.size(1)), 1.0 - self.patch_drop_prob)
            )  
            empty = keep.sum(dim=1) == 0
            if empty.any():
                rand_idx = torch.randint(0, p.size(1), (empty.sum().item(),), device=p.device)
                keep[empty] = 0.0
                keep[empty, rand_idx] = 1.0
            p = p * keep.unsqueeze(-1)
            key_padding_mask = (keep < 0.5)

        if self.use_spatial_attn and p.size(1) > 0:
            q = self.attn_ln(p)
            attn_out, _ = self.attn(q, q, q, key_padding_mask=key_padding_mask)
            p = p + attn_out

        P = p.size(1)
        Hside = int(math.isqrt(P))
        use_grid_now = (self.use_grid and Hside * Hside == P)

        if use_grid_now:
            fmap = p.view(B, Hside, Hside, -1).permute(0, 3, 1, 2).contiguous() 

            if key_padding_mask is not None:
                keep_hw = (~key_padding_mask).float().view(B, 1, Hside, Hside) 
                fmap = fmap * keep_hw

            if self.use_coord:
                grid = self.grid_xy.to(fmap.dtype).expand(B, -1, Hside, Hside) 
                fmap = torch.cat([fmap, grid], dim=1)                           

            fmap = self.coord_proj(fmap)                                         

            z = self.dw3(fmap)
            if self.use_dilation:
                z = z + self.dw3_dil(fmap)
            z = self.pw1(z)
            z = self.spatial_act(z)
            z = self.spatial_drop(z)

            gap = F.adaptive_avg_pool2d(z, 1).flatten(1)                         
            gmp = F.adaptive_max_pool2d(z, 1).flatten(1)                         
            patch_z = self.pool_reduce(torch.cat([gap, gmp], dim=1))             

        else:
            if key_padding_mask is not None:
                keep_float = (~key_padding_mask).float()
                denom = keep_float.sum(dim=1, keepdim=True).clamp_min(1.0)
                gap = (p * keep_float.unsqueeze(-1)).sum(dim=1) / denom          
            else:
                gap = p.mean(dim=1)                                              
            gmp = p.max(dim=1).values
            patch_z = self.pool_reduce(torch.cat([gap, gmp], dim=1))             

        z = patch_z if cls_z is None else self.fuse(torch.cat([cls_z, patch_z], dim=1))  

        out = self.head(z) 
        return out

class PoseClsHead(nn.Module):
    def __init__(self,
                 in_dim=768,
                 hidden_dim=256,
                 num_classes=2,
                 binary_mode="ce",   # "ce" 或 "bce"
                 dropout=0.1,
                 pool_type="mean"):   # "cls" 或 "mean"
        super().__init__()
        assert num_classes >= 2 or (num_classes == 2)
        if num_classes == 2:
            assert binary_mode in ("ce", "bce")

        self.num_classes = num_classes
        self.binary_mode = binary_mode
        self.out_dim = 1 if (num_classes == 2 and binary_mode == "bce") else num_classes
        self.pool_type = pool_type

        self.mlp = nn.Sequential(
            nn.LayerNorm(in_dim),
            nn.Linear(in_dim, hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, self.out_dim),
        )

        self.init_weights()


    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.LayerNorm):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)


    def pool_tokens(self, tokens):
        if tokens.dim() == 2:
            return tokens

        B, T, C = tokens.shape
        if self.pool_type == "cls" and T >= 1:
            return tokens[:, 0, :]
        else:
            return tokens.mean(dim=1)


    def forward(self, tokens, apply_activation: bool = False):
        z = self.pool_tokens(tokens)  
        logits = self.mlp(z)          

        if not apply_activation:
            return logits

        if self.num_classes == 2 and self.binary_mode == "bce":
            return torch.sigmoid(logits)          
        else:
            return torch.softmax(logits, dim=-1)  




class PoseClsHead2(nn.Module):
    def __init__(self,
                 in_dim=768,
                 hidden_dim=256,
                 num_classes=2,
                 binary_mode="ce",     
                 use_attn=False,
                 num_heads=4,
                 dropout=0.1,
                 patch_drop_prob=0.0,   
                 cls_boost=1.5):        
        super().__init__()
        assert num_classes >= 2 or (num_classes == 2)
        if num_classes == 2:
            assert binary_mode in ("ce", "bce")

        self.num_classes   = num_classes
        self.binary_mode   = binary_mode
        self.out_dim       = 1 if (num_classes == 2 and binary_mode == "bce") else num_classes
        self.use_attn      = use_attn
        self.num_heads     = num_heads
        self.patch_drop_prob = float(patch_drop_prob)
        self.cls_boost     = float(cls_boost)

        self.tok_norm  = nn.LayerNorm(in_dim)
        self.tok_proj  = nn.Linear(in_dim, hidden_dim)
        self.dropout   = nn.Dropout(dropout)

        if use_attn:
            self.attn_ln = nn.LayerNorm(hidden_dim)
            self.attn = nn.MultiheadAttention(embed_dim=hidden_dim,
                                                 num_heads=num_heads,
                                                 dropout=dropout,
                                                 batch_first=True)

        self.score_mlp = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.GELU(),
            nn.Linear(hidden_dim // 2, 1)
        )

        # 门控融合
        self.fuse_gate = nn.Sequential(
            nn.LayerNorm(2 * hidden_dim),
            nn.Linear(2 * hidden_dim, 1)
        )
        self.fuse_proj = nn.Linear(2 * hidden_dim, hidden_dim)

        # 分类头
        self.head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, self.out_dim)
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

    def forward(self, tokens, apply_activation: bool = False):
        B, T, C = tokens.shape
        assert T == 197, f"expected 197 tokens (incl. CLS), got {T}"

        x = self.tok_proj(self.tok_norm(tokens))   
        x = self.dropout(x)

        # 随机丢 patch token
        key_padding_mask = None
        if self.training and self.patch_drop_prob > 0.0 and T > 1:
            keep = torch.bernoulli(
                x.new_full((B, T-1), 1.0 - self.patch_drop_prob)
            )  # [B,196]

            empty = keep.sum(dim=1) == 0
            if empty.any():
                ridx = torch.randint(0, T-1, (int(empty.sum().item()),), device=x.device)
                keep[empty] = 0.0
                keep[empty, ridx] = 1.0
            keep = torch.cat([x.new_ones(B,1), keep], dim=1)  
            key_padding_mask = (keep < 0.5)                   
            x = x * keep.unsqueeze(-1)


        if self.use_attn:
            q = self.attn_ln(x)
            attn_out, _ = self.attn(q, q, q, key_padding_mask=key_padding_mask)
            x = x + attn_out


        scores = self.score_mlp(x).squeeze(-1)  # [B,197]
        if self.cls_boost != 1.0:
            scores[:, 0] = scores[:, 0] + math.log(self.cls_boost)  
        if key_padding_mask is not None:
            scores = scores.masked_fill(key_padding_mask, float("-inf"))
        w = F.softmax(scores, dim=1)            
        pool = (w.unsqueeze(-1) * x).sum(dim=1) 

        cls = x[:, 0, :]                       


        fuse_in = torch.cat([cls, pool], dim=1)  
        g = torch.sigmoid(self.fuse_gate(fuse_in))
        z = g * cls + (1.0 - g) * pool            
        z = self.fuse_proj(torch.cat([cls, z], dim=1)) 

        logits = self.head(z) 

        if not apply_activation:
            return logits


        if self.num_classes == 2 and self.binary_mode == "bce":
            return torch.sigmoid(logits)  
        else:
            return torch.softmax(logits, dim=1)  

