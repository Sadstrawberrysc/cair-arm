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


class PoseFeatureFusion(nn.Module):
    def __init__(self, embed_dim = 768, num_heads = 8):
        super().__init__()
        self.pose_embed = nn.Linear(9, embed_dim)
        self.cross_attn = nn.MultiheadAttention(embed_dim, num_heads, batch_first=True)
        self.ln = nn.LayerNorm(embed_dim)

    @staticmethod
    def rot_to_6d(R):
        if R.dim() == 3:
            mat = R
        elif R.dim() == 2:
            mat = R.view(-1, 3, 3)
        else:
            raise ValueError("R must be (B,3,3) or (B,9)")
        return mat[:, :, :2].reshape(mat.size(0), 6)    

    def forward(self, a, t, R):

        orig_shape = a.shape 
        need_reshape_back = False

        if a.dim() == 4: 
            B, C, H, W = a.shape
            patch = a.flatten(2).transpose(1, 2)
            need_reshape_back = True
        elif a.dim() == 3: 
            patch = a
        else:
            raise ValueError("a must be (B,C,H,W) or (B,N,C)")

        pose_tok = self.pose_embed(
            torch.cat([t, self.rot_to_6d(R)], dim=1)
        ).unsqueeze(1)
        attn_out, _ = self.cross_attn(patch, pose_tok, pose_tok)
        fused = self.ln(patch + attn_out)

        if need_reshape_back:
            fused = fused.transpose(1, 2).view(orig_shape)

        return fused


def build_encoder(embed_dim=768, num_heads=8, depth=6, ffn_ratio=2):
    layer = nn.TransformerEncoderLayer(
        d_model=embed_dim,
        nhead=num_heads,
        dim_feedforward=int(embed_dim * ffn_ratio),
        dropout=0.2,
        activation="gelu",
        batch_first=True,
    )
    return nn.TransformerEncoder(layer, num_layers=depth)


class PoseViTCore(nn.Module):
    def __init__(
        self,
        in_dim = 768, cmid = 512, Fusion_numheads = 8,
        Trans_numheads = 8, Trans_enc_depth = 4, ffn_ratio = 1,
    ):
        super().__init__()

        if cmid is None or cmid == in_dim:
            self.reduce = nn.Identity()
            self.mid_dim = in_dim
        else:
            self.reduce = nn.Linear(in_dim, cmid)
            self.mid_dim = cmid

        self.fusion = PoseFeatureFusion(self.mid_dim, Fusion_numheads)
        self.encoder = build_encoder(
            embed_dim=self.mid_dim,
            num_heads=Trans_numheads,
            depth=Trans_enc_depth,
            ffn_ratio=ffn_ratio,
        )

        if self.mid_dim == in_dim:
            self.expand = nn.Identity()
        else:
            self.expand = nn.Linear(self.mid_dim, in_dim)

    def forward(self, f1, t, R):
        original_is_map = f1.dim() == 4
        if original_is_map:
            B, C_in, H, W = f1.shape
            f1 = f1.flatten(2).transpose(1, 2)   
        else:
            B, N, C_in = f1.shape

        tokens = self.reduce(f1)                 

        tokens = self.fusion(tokens, t, R)        
        tokens = self.encoder(tokens)             

        tokens = self.expand(tokens)

        if original_is_map:
            tokens = tokens.transpose(1, 2).view(B, C_in, H, W)
        return tokens



class ResMLPBlock(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.fc1 = nn.Linear(dim, dim)
        self.bn1 = nn.BatchNorm1d(dim)
        self.fc2 = nn.Linear(dim, dim)
        self.bn2 = nn.BatchNorm1d(dim)
        self.act = nn.ReLU(inplace=True)

    def forward(self, x):
        identity = x
        out = self.act(self.bn1(self.fc1(x)))
        out = self.bn2(self.fc2(out))
        return self.act(out + identity)


class MultiScalePool(nn.Module):
    def __init__(self, scales=(1,), include_cls=False):
        super().__init__()
        self.scales = scales
        self.include_cls = include_cls

    def forward(self, x):                         
        cls_vec = None

        if x.dim() == 3:
            B, N, C = x.shape
            Hp = int(math.sqrt(N - 1))
            if Hp * Hp == N - 1:
                cls_vec, patch = x[:, :1], x[:, 1:]              
                x = patch.transpose(1, 2).view(B, C, Hp, Hp)     
            else:
                warnings.warn(f" use tokenmean", RuntimeWarning)
                pooled = x.mean(dim=1)                           
                return torch.cat([pooled, cls_vec.squeeze(1)], dim=1) \
                       if (self.include_cls and cls_vec is not None) else pooled

        feats = []
        for s in self.scales:
            pool = F.adaptive_avg_pool2d(x, s)                   
            feats.append(pool.flatten(1))                        

        if self.include_cls and cls_vec is not None:
            feats.append(cls_vec.squeeze(1))                     

        return torch.cat(feats, dim=1) 


class GuidanceHead(nn.Module):
    def __init__(self,
                 feat_dim=768,
                 query_dim=32,
                 hidden_dim=256,
                 out_dim=6,
                 head_type="mlp",               
                 num_layers=3,
                 scales=(1,2,4)):               
        super().__init__()
        assert head_type in ("mlp", "resnet")
        self.head_type = head_type
        self.query_dim = query_dim

        self.pool = MultiScalePool(scales)
        scale_factor = sum(s*s for s in scales) 
        pooled_dim = feat_dim * scale_factor 

        if head_type == "mlp":
            def make_mlp(in_dim):
                layers = []
                for i in range(num_layers):
                    inp = in_dim if i == 0 else hidden_dim
                    layers += [nn.Linear(inp, hidden_dim), nn.ReLU(inplace=True)]
                    layers += [nn.Dropout(0.1)]

                layers += [nn.Linear(hidden_dim, hidden_dim),
                        nn.ReLU(inplace=True)]             
                return nn.Sequential(*layers)
            self.mlp_noq = make_mlp(pooled_dim)
            self.mlp_q = make_mlp(pooled_dim + query_dim)

        else:  # resnet style
            self.fc_init = nn.Linear(pooled_dim, hidden_dim)
            self.fc_init_q = nn.Linear(pooled_dim + query_dim,   hidden_dim)

            self.blocks_noq = nn.Sequential(
                *[ResMLPBlock(hidden_dim) for _ in range(num_layers)]
            )
            self.blocks_q = nn.Sequential(
                *[ResMLPBlock(hidden_dim) for _ in range(num_layers)]
            )
        self.fc_out = nn.Linear(hidden_dim, out_dim)


    def forward(self, feat, qi=None):
        x = self.pool(feat) 

        if self.head_type == "mlp":
            if qi is None:
                x = self.mlp_noq(x)
            else:
                if qi.size(1) != self.query_dim:
                    raise ValueError(f"qi dim must be {self.query_dim}")
                x = self.mlp_q(torch.cat([x, qi], dim=1))

        else:                       
            if qi is None:
                x = F.relu(self.fc_init(x), inplace=True)
                x = self.blocks_noq(x)
            else:
                if qi.size(1) != self.query_dim:
                    raise ValueError(f"qi dim must be {self.query_dim}")
                x = torch.cat([x, qi], dim=1)
                x = F.relu(self.fc_init_q(x), inplace=True)
                x = self.blocks_q(x)

        return self.fc_out(x)                 




def get_2d_sincos_pos_embed(embed_dim, h, w, temperature = 10000):

    assert embed_dim % 2 == 0, "embed_dim 须是偶数a"
    y_embed = torch.arange(h, dtype=torch.float32)
    x_embed = torch.arange(w, dtype=torch.float32)

    grid_y, grid_x = torch.meshgrid(y_embed, x_embed, indexing='ij')
    grid_y = grid_y.reshape(-1)
    grid_x = grid_x.reshape(-1)

    def _sincos(pos, dim):
        assert dim % 2 == 0
        omega = torch.arange(dim // 2, dtype=torch.float32)
        omega = 1 / (temperature ** (omega / (dim // 2)))
        out = pos[:, None] * omega[None, :]
        sin, cos = torch.sin(out), torch.cos(out)
        return torch.cat([sin, cos], dim=1)

    emb_y = _sincos(grid_y, embed_dim // 2)
    emb_x = _sincos(grid_x, embed_dim // 2)
    pos_embed = torch.cat([emb_y, emb_x], dim=1)
    return pos_embed



class TransformerPoseHead(nn.Module):

    def __init__(self, in_dim=768, hidden=256, num_levels=1, H=14, W=14,
        out_dim=6, nhead=8, depth=2, use_level_embed=False,
        use_pos_embed=False,
    ):
        super().__init__()
        self.num_levels = num_levels
        self.H = H
        self.W = W
        self.hidden = hidden
        self.use_level_embed = use_level_embed
        self.use_pos_embed = use_pos_embed

        self.projs = nn.ModuleList([
            nn.Conv2d(in_dim, hidden, kernel_size=1)
            for _ in range(num_levels)
        ])

        if use_level_embed:
            self.level_embed = nn.Parameter(torch.zeros(num_levels, hidden))
            nn.init.trunc_normal_(self.level_embed, std=0.02)
        else:
            self.register_parameter("level_embed", None)

        if use_pos_embed:
            pos = get_2d_sincos_pos_embed(hidden, H, W)
            self.register_buffer("pos_embed", pos[None, ...], persistent=False)
        else:
            self.register_buffer("pos_embed", None)

        self.reg_token = nn.Parameter(torch.zeros(1, 1, hidden))
        nn.init.trunc_normal_(self.reg_token, std=0.02)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden,
            nhead=nhead,
            dim_feedforward=hidden * 1,
            batch_first=True,
            activation='gelu'
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=depth)

        self.norm = nn.LayerNorm(hidden)
        self.fc_out = nn.Linear(hidden, out_dim)

    def forward(self, feats):

        assert len(feats) == self.num_levels, f"input features: {len(feats)} != num_levels {self.num_levels}"
        B = feats[0].shape[0]

        tokens = []
        for i, (proj, f) in enumerate(zip(self.projs, feats)):
            x = proj(f)
            t = x.flatten(2).transpose(1, 2)

            if self.use_level_embed:
                t = t + self.level_embed[i].view(1, 1, -1)

            tokens.append(t)

        seq = torch.cat(tokens, dim=1)

        if self.use_pos_embed:
            pos = self.pos_embed.repeat(1, self.num_levels, 1)
            seq = seq + pos

        # 拼接回归 
        reg = self.reg_token.expand(B, -1, -1)
        seq = torch.cat([reg, seq], dim=1)

        # Transformer
        seq = self.encoder(seq)
        reg_out = self.norm(seq[:, 0])

        return self.fc_out(reg_out)



class LitePoseHead(nn.Module):
    def __init__(
        self,
        in_dim=768,
        hidden=64,
        num_levels=2,
        out_dim=6,
        spatial_pool="avg",           
        level_fuse="mean",      
        use_level_embed=False,
        drop=0.2,
    ):
        super().__init__()
        self.num_levels = num_levels
        self.hidden = hidden
        self.spatial_pool = spatial_pool
        self.level_fuse = level_fuse
        self.use_level_embed = use_level_embed

        self.projs = nn.ModuleList([
            nn.Conv2d(in_dim, hidden, kernel_size=1) for _ in range(num_levels)
        ])

        if use_level_embed:
            self.level_embed = nn.Parameter(torch.zeros(num_levels, hidden))
            nn.init.trunc_normal_(self.level_embed, std=0.02)
        else:
            self.register_parameter("level_embed", None)

        if spatial_pool == "attn":
            self.spatial_attn = nn.Conv2d(hidden, 1, kernel_size=1)
        elif spatial_pool == "avgmax":
            self.reduce = nn.Linear(2 * hidden, hidden)

        if level_fuse == "attn":
            self.level_score = nn.Sequential(
                nn.LayerNorm(hidden),
                nn.Linear(hidden, hidden // 2),
                nn.GELU(),
                nn.Linear(hidden // 2, 1)
            )

        self.head = nn.Sequential(
            nn.LayerNorm(hidden),
            nn.Linear(hidden, hidden),
            nn.GELU(),
            nn.Dropout(drop),
            nn.Linear(hidden, out_dim),
        )

    def _spatial_descriptor(self, x, lvl_idx=None):
        if self.use_level_embed:
            x = x + self.level_embed[lvl_idx].view(1, -1, 1, 1)

        if self.spatial_pool == "avg":
            desc = x.mean(dim=(2, 3))  # GAP
            return desc

        if self.spatial_pool == "maxpool":
            gmp = F.adaptive_max_pool2d(x, 1).flatten(1)
            return gmp

        elif self.spatial_pool == "avgmax":
            gap = x.mean(dim=(2, 3))
            gmp = F.adaptive_max_pool2d(x, 1).flatten(1)
            desc = torch.cat([gap, gmp], dim=1)        
            return self.reduce(desc)                   

        elif self.spatial_pool == "attn":
            a = self.spatial_attn(x)                    
            a = a.flatten(2)                            
            a = torch.softmax(a, dim=-1).view(a.size(0), 1, *x.shape[-2:])
            desc = (x * a).sum(dim=(2, 3))              
            return desc

        else:
            raise ValueError(f"Unknown spatial_pool: {self.spatial_pool}")

    def forward(self, feats):

        assert len(feats) == self.num_levels, f"got {len(feats)} feats, expected {self.num_levels}"
        B = feats[0].shape[0]

        descs = []
        for i, (proj, f) in enumerate(zip(self.projs, feats)):
            x = proj(f)                                     
            d = self._spatial_descriptor(x, lvl_idx=i)      
            descs.append(d)

        D = torch.stack(descs, dim=1)                       

        if self.level_fuse == "mean":
            z = D.mean(dim=1)                              
        else:  # attn
            score = self.level_score(D)                     
            attn = torch.softmax(score, dim=1)              
            z = (attn * D).sum(dim=1)         

        return self.head(z) 



class PoseHead2(nn.Module):
    def __init__(
        self,
        in_dim=768,
        num_levels=1,
        per_level=256,              
        fuse_mid=256,              
        out_dim=3,                 
        use_dilation=True,         
        dilation=2,
        use_large_kernel=False,    
        k_large=5,                  
        use_spatial_attn=False,     
        use_bn=True,                
        dropout=0.2,
    ):
        super().__init__()
        self.num_levels = num_levels
        self.per_level = per_level
        self.use_spatial_attn = use_spatial_attn
        self.use_bn = use_bn

        
        self.projs = nn.ModuleList([
            nn.Conv2d(in_dim, per_level, kernel_size=1, bias=not use_bn)
            for _ in range(num_levels)
        ])
        self.proj_bns = nn.ModuleList([
            nn.BatchNorm2d(per_level) if use_bn else nn.Identity()
            for _ in range(num_levels)
        ])

        in_ch = num_levels * per_level

        k1, d1, p1 = (k_large, 1, k_large // 2) if use_large_kernel else (3, 1, 1)
        self.conv1 = nn.Conv2d(in_ch, fuse_mid, kernel_size=k1, padding=p1, dilation=d1, bias=not use_bn)
        self.bn1 = nn.BatchNorm2d(fuse_mid) if use_bn else nn.Identity()

        if use_dilation and not use_large_kernel:
            k2, d2, p2 = 3, dilation, dilation
        else:
            k2, d2, p2 = 3, 1, 1

        self.conv2 = nn.Conv2d(fuse_mid, per_level, kernel_size=k2, padding=p2, dilation=d2, bias=not use_bn)
        self.bn2 = nn.BatchNorm2d(per_level) if use_bn else nn.Identity()

        if use_spatial_attn:
            self.spatial_attn = nn.Conv2d(2, 1, kernel_size=7, padding=3)

        self.pool_reduce = nn.Linear(2 * per_level, per_level)

        self.head = nn.Sequential(
            nn.LayerNorm(per_level),
            nn.Linear(per_level, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            # nn.Linear(128, 128),
            # nn.ReLU(inplace=True),
            # nn.Dropout(dropout),
            nn.Linear(64, out_dim),
            nn.Sigmoid(),
        )

        self.init_weights()

    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)

    def spatial_attention(self, x):
        avg_map = torch.mean(x, dim=1, keepdim=True)
        max_map, _ = torch.max(x, dim=1, keepdim=True)
        a = torch.cat([avg_map, max_map], dim=1)
        a = torch.sigmoid(self.spatial_attn(a))
        return x * a

    def forward(self, feats):

        assert len(feats) == self.num_levels, f"got {len(feats)} feats, expected {self.num_levels}"
        xs = []
        for f, proj, bn in zip(feats, self.projs, self.proj_bns):
            x = proj(f)
            x = bn(x)
            x = F.relu(x, inplace=True)
            xs.append(x)

        x = torch.cat(xs, dim=1)
        x = self.conv1(x)
        x = self.bn1(x)
        x = F.relu(x, inplace=True)

        # x = self.conv2(x)
        # x = self.bn2(x); x = F.relu(x, inplace=True)

        if self.use_spatial_attn:
            x = self.spatial_attention(x)  

        gap = F.adaptive_avg_pool2d(x, 1).flatten(1)  
        gmp = F.adaptive_max_pool2d(x, 1).flatten(1)
        z = torch.cat([gap, gmp], dim=1) 
        z = self.pool_reduce(z)

        out = self.head(z)
        return out



class PoseHead3(nn.Module):
    def __init__(
        self,
        in_dim=768,            
        hidden_dim=256,        
        out_dim=2,             
        use_spatial_attn=False, 
        num_heads=4,
        patch_drop_prob=0.15,  
        dropout=0.2,
    ):
        super().__init__()
        self.use_spatial_attn = use_spatial_attn
        self.patch_drop_prob = float(patch_drop_prob)

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

        self.pool_reduce = nn.Linear(2 * hidden_dim, hidden_dim)

        self.fuse = nn.Linear(2 * hidden_dim, hidden_dim)

        self.head = nn.Sequential(
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, out_dim),
        )

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.LayerNorm):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)

    def forward(self, features, x=None):

        B, T, C = features.shape
        has_cls = (T == 197)

        if has_cls:
            cls_feat = x[:, 0, :] 
            patches = features[:, 1:, :]    
        else:
            cls_feat = None
            patches = features          

        cls_z = None
        if cls_feat is not None:
            cls_z = self.cls_proj(cls_feat) 

        p = self.patch_proj(patches)    

        key_padding_mask = None 
        if self.training and self.patch_drop_prob > 0.0 and p.size(1) > 0:
            keep = torch.bernoulli(
                p.new_full((B, p.size(1)), 1.0 - self.patch_drop_prob)
            )  # 1=keep, 0=drop

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

        if p.size(1) > 0:
            if key_padding_mask is not None:
                keep_bool = ~key_padding_mask         
                keep_float = keep_bool.float()
                denom = keep_float.sum(dim=1, keepdim=True).clamp_min(1.0)
                gap = (p * keep_float.unsqueeze(-1)).sum(dim=1) / denom 
            else:
                gap = p.mean(dim=1)                     
            gmp = p.max(dim=1).values                  
            patch_z = self.pool_reduce(torch.cat([gap, gmp], dim=1)) 
        else:
            patch_z = torch.zeros(B, self.pool_reduce.out_features, device=features.device)

        if cls_z is None:
            z = patch_z
        else:
            z = self.fuse(torch.cat([cls_z, patch_z], dim=1)) 

        out = self.head(z) 
        return out



class PoseHead4(nn.Module):
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
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)
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




class DeepSet2Image(nn.Module):
    def __init__(self, in_ch=768, hidden=128, out_ch=64, pool="max", use_attn=False):
        super().__init__()

        self.phi = nn.Sequential(
            nn.Conv2d(in_ch, hidden, 3, padding=1), nn.GELU(),
            nn.Conv2d(hidden, hidden, 3, padding=1), nn.GELU(),
        )
        self.rho = nn.Sequential(
            nn.Conv2d(hidden, hidden, 3, padding=1), nn.GELU(),
            nn.Conv2d(hidden, out_ch, 1),
        )

        self.pool = pool           
        self.use_attn = use_attn
        if use_attn:
            self.gate = nn.Conv2d(hidden, 1, 1) 

    def forward(self, x, mask = None):

        assert x.dim() == 4, f"expect [N,C,H,W], got {tuple(x.shape)}"
        N, C, H, W = x.shape
        feats = self.phi(x)  

        if mask is not None:
            if mask.dim() == 1:
                mask = mask.view(N, 1, 1)
            feats = feats * mask            

        if self.use_attn:
            logits = self.gate(feats)      
            if mask is not None:
                logits = logits + (mask - 1) * 1e9 
            w = torch.softmax(logits, dim=0)
            agg = (feats * w).sum(dim=0)
        else:
            if self.pool == "mean":
                if mask is None:
                    agg = feats.mean(dim=0)
                else:
                    denom = mask.sum(dim=0).clamp_min(1e-6)
                    agg = feats.sum(dim=0) / denom
            elif self.pool == "sum":
                agg = feats.sum(dim=0) if mask is None else (feats.sum(dim=0))
            elif self.pool == "max":
                agg = feats.max(dim=0).values
            else:
                raise ValueError(f"unknown pool: {self.pool}")

        y = self.rho(agg.unsqueeze(0)).squeeze(0) 
        return y




class PoseClsHead(nn.Module):
    def __init__(
        self,
        in_dim=768,
        num_levels=1,
        per_level=256,
        fuse_mid=256,
        num_classes=2,
        binary_mode="ce",  
        use_dilation=True,
        dilation=2,
        use_large_kernel=False,
        k_large=5,
        use_spatial_attn=False,
        use_bn=True,
        dropout=0.1,
    ):
        super().__init__()
        assert num_levels >= 1
        assert num_classes >= 2 or (num_classes == 2)
        if num_classes == 2:
            assert binary_mode in ("ce", "bce"), "binary_mode should be 'ce' or 'bce'"

        self.num_levels = num_levels
        self.per_level = per_level
        self.use_spatial_attn = use_spatial_attn
        self.use_bn = use_bn
        self.num_classes = num_classes
        self.binary_mode = binary_mode

        if self.num_classes == 2 and self.binary_mode == "bce":
            self.out_dim = 1
        else:
            self.out_dim = self.num_classes

        self.projs = nn.ModuleList([
            nn.Conv2d(in_dim, per_level, kernel_size=1, bias=not use_bn)
            for _ in range(num_levels)
        ])
        self.proj_bns = nn.ModuleList([
            nn.BatchNorm2d(per_level) if use_bn else nn.Identity()
            for _ in range(num_levels)
        ])

        in_ch = num_levels * per_level
        k1, d1, p1 = (k_large, 1, k_large // 2) if use_large_kernel else (3, 1, 1)
        self.conv1 = nn.Conv2d(in_ch, fuse_mid, kernel_size=k1, padding=p1, dilation=d1, bias=not use_bn)
        self.bn1 = nn.BatchNorm2d(fuse_mid) if use_bn else nn.Identity()

        if use_dilation and not use_large_kernel:
            k2, d2, p2 = 3, dilation, dilation
        else:
            k2, d2, p2 = 3, 1, 1
        self.conv2 = nn.Conv2d(fuse_mid, per_level, kernel_size=k2, padding=p2, dilation=d2, bias=not use_bn)
        self.bn2 = nn.BatchNorm2d(per_level) if use_bn else nn.Identity()

        if use_spatial_attn:
            self.spatial_attn = nn.Conv2d(2, 1, kernel_size=7, padding=3)

        self.pool_reduce = nn.Linear(2 * per_level, per_level)

        self.head = nn.Sequential(
            nn.LayerNorm(per_level),
            nn.Linear(per_level, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, self.out_dim),
        )

        self.init_weights()

    def init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)

    def spatial_attention(self, x):
        avg_map = torch.mean(x, dim=1, keepdim=True)
        max_map, _ = torch.max(x, dim=1, keepdim=True)
        a = torch.cat([avg_map, max_map], dim=1)
        a = torch.sigmoid(self.spatial_attn(a))
        return x * a

    def forward(self, feats, apply_activation: bool = False):
        assert len(feats) == self.num_levels, f"got {len(feats)} feats, expected {self.num_levels}"

        xs = []
        for f, proj, bn in zip(feats, self.projs, self.proj_bns):
            x = proj(f)
            x = bn(x)
            x = F.relu(x, inplace=True)
            xs.append(x)

        x = torch.cat(xs, dim=1)
        x = self.conv1(x)
        x = self.bn1(x)
        x = F.relu(x, inplace=True)
        
        # x = self.conv2(x); x = self.bn2(x); x = F.relu(x, inplace=True)

        if self.use_spatial_attn:
            x = self.spatial_attention(x)

        gap = F.adaptive_avg_pool2d(x, 1).flatten(1)
        gmp = F.adaptive_max_pool2d(x, 1).flatten(1)
        z = torch.cat([gap, gmp], dim=1)
        z = self.pool_reduce(z)

        logits = self.head(z)

        if not apply_activation:
            return logits

        if self.num_classes == 2 and self.binary_mode == "bce":
            return torch.sigmoid(logits)
        else:
            return torch.softmax(logits, dim=1)







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





class PoseHeadFlex(nn.Module):

    def __init__(
        self,
        in_dim=768,
        per_level=256,              
        fuse_mid=256,               
        out_dim=3,                 
        use_spatial_attn=False,     
        use_large_kernel=False,     
        k_large=5,
        use_dilation=True,
        dilation=2,
        use_bn=True,                
        token_pool='auto',          
                                    
        final_activation='sigmoid', 
        dropout=0.1,
        use_map_fuse_for_cnn=True,  
        enable_conv2=False          
    ):
        super().__init__()
        self.in_dim = in_dim
        self.per_level = per_level
        self.fuse_mid = fuse_mid
        self.out_dim = out_dim
        self.use_spatial_attn = use_spatial_attn
        self.use_large_kernel = use_large_kernel
        self.k_large = k_large
        self.use_dilation = use_dilation
        self.dilation = dilation
        self.use_bn = use_bn
        self.token_pool = token_pool
        self.final_activation = final_activation
        self.use_map_fuse_for_cnn = use_map_fuse_for_cnn
        self.enable_conv2 = enable_conv2

        self.proj_cnn = nn.Conv2d(in_dim, per_level, kernel_size=1, bias=not use_bn)
        self.bn_cnn   = nn.BatchNorm2d(per_level) if use_bn else nn.Identity()

        self.ln_in_seq = nn.LayerNorm(in_dim)
        self.proj_seq  = nn.Linear(in_dim, per_level, bias=True)  # 用 LN 替代 BN
        self.ln_seq    = nn.LayerNorm(per_level)

        if use_spatial_attn:
            self.spatial_attn = nn.Conv2d(2, 1, kernel_size=7, padding=3)


        k1, d1, p1 = (k_large, 1, k_large // 2) if use_large_kernel else (3, 1, 1)
        self.conv1 = nn.LazyConv2d(self.fuse_mid, kernel_size=k1, padding=p1, dilation=d1, bias=not use_bn)
        self.bn1_2d = nn.BatchNorm2d(self.fuse_mid) if use_bn else nn.Identity()

        if self.use_dilation and (not self.use_large_kernel):
            k2, d2, p2 = 3, dilation, dilation
        else:
            k2, d2, p2 = 3, 1, 1
        self.conv2 = nn.Conv2d(self.fuse_mid, self.per_level, kernel_size=k2, padding=p2, dilation=d2, bias=not use_bn)
        self.bn2_2d = nn.BatchNorm2d(self.per_level) if use_bn else nn.Identity()

        self.pool_reduce = nn.Linear(2 * per_level, per_level)

        self.fuse_fc   = nn.LazyLinear(fuse_mid, bias=not use_bn)
        self.fuse_bn1d = nn.BatchNorm1d(fuse_mid) if use_bn else nn.Identity()

        self.token_attn = nn.Sequential(
            nn.Linear(per_level, per_level // 2),
            nn.Tanh(),
            nn.Linear(per_level // 2, 1)
        )

        self.map_to_fuse = nn.Linear(per_level, fuse_mid)

        head = [
            nn.LayerNorm(fuse_mid),
            nn.Linear(fuse_mid, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, out_dim)
        ]
        self.head = nn.Sequential(*head)

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            w = getattr(m, "weight", None)
            if isinstance(w, UninitializedParameter):
                continue

            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

            elif isinstance(m, nn.Linear):
                nn.init.trunc_normal_(m.weight, std=0.02)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

            elif isinstance(m, (nn.BatchNorm1d, nn.BatchNorm2d)):
                nn.init.ones_(m.weight)
                nn.init.zeros_(m.bias)


    def _spatial_attention(self, x):
        avg_map = torch.mean(x, dim=1, keepdim=True)
        max_map, _ = torch.max(x, dim=1, keepdim=True)
        a = torch.cat([avg_map, max_map], dim=1)
        a = torch.sigmoid(self.spatial_attn(a))
        return x * a

    @staticmethod
    def _detect_has_cls_token(seq_len):
        s = int(math.sqrt(max(seq_len - 1, 0)))
        return (s * s == seq_len - 1)

    def _pool_tokens(self, t, has_cls):

        B, N, C = t.shape
        if self.token_pool in ('auto', 'cls'):
            if (self.token_pool == 'cls' and has_cls) or (self.token_pool == 'auto' and has_cls):
                return t[:, 0, :]  # CLS
            tokens = t[:, 1:, :] if has_cls else t
            return tokens.mean(dim=1)

        elif self.token_pool == 'mean':
            tokens = t[:, 1:, :] if has_cls else t
            return tokens.mean(dim=1)

        elif self.token_pool == 'meanmax':
            tokens = t[:, 1:, :] if has_cls else t
            gap = tokens.mean(dim=1)
            gmp = tokens.amax(dim=1)
            return torch.cat([gap, gmp], dim=1)

        elif self.token_pool == 'attn':
            tokens = t[:, 1:, :] if has_cls else t 
            score = self.token_attn(tokens).squeeze(-1) 
            attn  = torch.softmax(score, dim=1).unsqueeze(-1)
            return (tokens * attn).sum(dim=1) 

        else:
            tokens = t[:, 1:, :] if has_cls else t
            return tokens.mean(dim=1)

    def forward(self, feats):
        if isinstance(feats, torch.Tensor):
            feats = [feats]

        assert isinstance(feats, (list, tuple)) and len(feats) >= 1, "feats shoud be Tensor 或 list with something"

        is_all_cnn = all(f.dim() == 4 for f in feats)
        is_all_seq = all(f.dim() == 3 for f in feats)

        if is_all_cnn and self.use_map_fuse_for_cnn:
            xs = []
            for f in feats:
                x = self.proj_cnn(f)   
                x = self.bn_cnn(x)
                x = F.relu(x, inplace=True)
                if self.use_spatial_attn:
                    x = self._spatial_attention(x)
                xs.append(x)

            x_cat = torch.cat(xs, dim=1) 
            y = self.conv1(x_cat) 
            y = self.bn1_2d(y)
            y = F.relu(y, inplace=True)

            if self.enable_conv2:
                y = self.conv2(y)
                y = self.bn2_2d(y)
                y = F.relu(y, inplace=True)

            gap = F.adaptive_avg_pool2d(y, 1).flatten(1)  
            gmp = F.adaptive_max_pool2d(y, 1).flatten(1)


            if gap.shape[1] == self.fuse_mid:
                z = torch.cat([gap, gmp], dim=1)               
                fused = z.view(z.size(0), 2, -1).mean(dim=1)  
                fused = self.fuse_bn1d(fused)
                fused = F.relu(fused, inplace=True)
            else:
                z2 = torch.cat([gap, gmp], dim=1)             
                z2 = self.pool_reduce(z2)                      
                fused = self.map_to_fuse(z2)                   

            out = self.head(fused)                             
            if self.final_activation == 'sigmoid':
                out = torch.sigmoid(out)
            elif self.final_activation == 'tanh':
                out = torch.tanh(out)
            return out

        vecs = []
        for f in feats:
            if f.dim() == 4:
                x = self.proj_cnn(f)
                x = self.bn_cnn(x)
                x = F.relu(x, inplace=True)
                if self.use_spatial_attn:
                    x = self._spatial_attention(x)
                gap = F.adaptive_avg_pool2d(x, 1).flatten(1)
                gmp = F.adaptive_max_pool2d(x, 1).flatten(1)
                z = torch.cat([gap, gmp], dim=1)                # [B, 2*per_level]
                z = self.pool_reduce(z)                         # [B, per_level]
                vecs.append(z)

            elif f.dim() == 3:
                B, N, C = f.shape
                x = self.ln_in_seq(f)
                x = self.proj_seq(x)                            
                x = self.ln_seq(x)
                has_cls = self._detect_has_cls_token(N)

                v = self._pool_tokens(x, has_cls=has_cls)      
                if v.dim() == 2 and v.shape[1] == 2 * self.per_level:
                    v = self.pool_reduce(v)                    
                vecs.append(v)
            else:
                raise ValueError(f"不支持的特征维度: {f.dim()}")

        H = torch.cat(vecs, dim=1)                               # [B, L*per_level]
        fused = self.fuse_fc(H)                                  
        fused = self.fuse_bn1d(fused)
        fused = F.relu(fused, inplace=True)

        out = self.head(fused)                                   # [B, out_dim]
        if self.final_activation == 'sigmoid':
            out = torch.sigmoid(out)
        elif self.final_activation == 'tanh':
            out = torch.tanh(out)
        return out



class PoseHead5(nn.Module):
    def __init__(
        self,
        in_dim=768,
        hidden_dim=256,
        out_dim=2,
        dropout=0.1,
        use_sigmoid=True,
        pool_type="mean",   # "cls" or "mean"
    ):
        super().__init__()
        self.use_sigmoid = use_sigmoid
        self.pool_type = pool_type

        self.mlp = nn.Sequential(
            nn.LayerNorm(in_dim),
            nn.Linear(in_dim, hidden_dim),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, out_dim),
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

    def pool_tokens(self, feats, x_raw=None):
        if feats.dim() == 2:
            return feats

        B, T, C = feats.shape
        if self.pool_type == "cls" and T >= 1:
            if x_raw is not None and x_raw.dim() == 3 and x_raw.size(1) >= 1:
                return x_raw[:, 0, :]
            return feats[:, 0, :]
        else:
            return feats.mean(dim=1)

    def forward(self, features, x=None):
        vec = self.pool_tokens(features, x_raw=x)
        out = self.mlp(vec)
        if self.use_sigmoid:
            out = torch.sigmoid(out)
        return out






class PoseClsHead3(nn.Module):
    def __init__(self,
                 in_dim=768,
                 hidden_dim=256,
                 num_classes=2,
                 binary_mode="ce",   # "ce" 或 "bce"
                 dropout=0.1,
                 pool_type="cls"):   # "cls" 或 "mean"
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






