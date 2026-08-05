import torch

import torch
import numpy as np
from pathlib import Path

_NAME2IDX = {'x':0, 'y':1, 'z':2}

def _canonical_index(index, dim_size, names_map=None):
    if index is None:
        return slice(None)

    if callable(index):
        index = index(dim_size)

    if isinstance(index, str):
        # 例如 'xz' -> [0, 2]
        mp = _NAME2IDX if names_map is None else names_map
        idxs = [mp[ch] for ch in index.lower()]
        return torch.as_tensor(idxs, dtype=torch.long)

    if isinstance(index, slice):
        return index

    if isinstance(index, (list, tuple, np.ndarray, torch.Tensor)):
        idx = torch.as_tensor(index, dtype=torch.long)
        return idx

    raise TypeError(f"Unsupported index type: {type(index)}")


def index_along_axis(t, index, axis):

    if isinstance(index, slice):
        sl = [slice(None)] * t.ndim
        sl[axis] = index
        return t[tuple(sl)]
    else:
        if not torch.is_tensor(index):
            index = torch.as_tensor(index, dtype=torch.long, device=t.device)
        tt = torch.movedim(t, axis, 0)
        tt = torch.index_select(tt, 0, index.to(tt.device))
        tt = torch.movedim(tt, 0, axis)
        return tt


def reshape_vector_for_axis(vec, like, axis):
    if vec.ndim != 1:
        return vec
    shape = [1] * like.ndim
    shape[axis] = vec.numel()
    return vec.view(*shape)




def pick_stats_tensor(stats_dict, name, like_tensor, feat_axis = -1, index=None, strict_len = False):

    t = stats_dict.get(name, None)
    if t is None:
        return None

    t = t.to(device=like_tensor.device, dtype=like_tensor.dtype)

    tgt_len = like_tensor.size(feat_axis)
    idx = _canonical_index(index, dim_size=t.size(-1) if t.ndim > 0 else 1)

    if t.ndim == 1:
        t = t[idx]
        if t.numel() != tgt_len:
            if strict_len:
                raise ValueError(f"stats length {t.numel()} != target length {tgt_len}")
            t = t[:tgt_len]
        t = reshape_vector_for_axis(t, like_tensor, feat_axis)
    else:
        t = index_along_axis(t, idx, feat_axis)
        if t.size(feat_axis) != tgt_len:
            if strict_len:
                raise ValueError(f"stats size along axis {feat_axis} = {t.size(feat_axis)} "
                                 f"!= target {tgt_len}")
            t = index_along_axis(t, slice(0, tgt_len), feat_axis)

    return t


def _load_stats(pt):
    pt = Path(pt)
    js = pt.with_suffix(".json")
    if pt.exists():
        return torch.load(pt, map_location="cpu")
    if js.exists():
        import json
        with open(js) as f: jd = json.load(f)

        
        return {k: torch.tensor(v) for k, v in jd.items()}
    print(f"[cannot find: {pt}")
    exit()

def build_inv_normalizer_from_dataset(key, stats_path, mode = "zscore", pad_ratio = 0.05, feat_axis = -1, index=None):
    
    stats = _load_stats(stats_path)

    def inv(y_norm):

        y = y_norm.detach()

        mean = pick_stats_tensor(stats, f"{key}_mean", y, feat_axis, index)
        std  = pick_stats_tensor(stats, f"{key}_std",  y, feat_axis, index)
        vmin = pick_stats_tensor(stats, f"{key}_min",  y, feat_axis, index)
        vrng = pick_stats_tensor(stats, f"{key}_rng",  y, feat_axis, index)
        mid  = pick_stats_tensor(stats, f"{key}_mid",  y, feat_axis, index)
        half = pick_stats_tensor(stats, f"{key}_half", y, feat_axis, index)
        mabs = pick_stats_tensor(stats, f"{key}_maxabs", y, feat_axis, index)

        if mode == "zscore":
            std  = (std  if std  is not None else reshape_vector_for_axis(torch.ones(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)).clamp_min(1e-12)
            mean =  mean if mean is not None else reshape_vector_for_axis(torch.zeros(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)
            return y * std + mean

        elif mode in ("minmax", "minmax_pad"):
            vmin = vmin if vmin is not None else reshape_vector_for_axis(torch.zeros(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)
            vrng = (vrng if vrng is not None else reshape_vector_for_axis(torch.ones(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)).clamp_min(1e-12)
            if mode == "minmax_pad":
                pad = vrng * pad_ratio
                vmin = vmin - pad
                vrng = vrng + 2 * pad
            return y * vrng + vmin

        elif mode in ("sym_minmax", "sym_minmax_pad"):
            if mid is None or half is None:
                vmin = vmin if vmin is not None else reshape_vector_for_axis(torch.zeros(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)
                vrng = (vrng if vrng is not None else reshape_vector_for_axis(torch.ones(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)).clamp_min(1e-12)
                mid  = vmin + 0.5 * vrng
                half = 0.5 * vrng
            if mode == "sym_minmax_pad":
                half = half * (1 + 2 * pad_ratio)
            return y * half + mid

        elif mode == "maxabs":
            if mabs is None:
                vmin = vmin if vmin is not None else reshape_vector_for_axis(torch.zeros(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis)
                vrng = (vrng if vrng is not None else reshape_vector_for_axis(torch.ones(y.size(feat_axis), device=y.device, dtype=y.dtype), y, feat_axis))
                vmax = vmin + vrng
                mabs = torch.maximum(vmin.abs(), vmax.abs()).clamp_min(1e-12)
            return y * mabs

        else:
            return y

    return inv
