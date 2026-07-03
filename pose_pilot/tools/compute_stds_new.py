import os
import json
import argparse
import time
from collections import defaultdict
import sys

import torch
from torch.utils.data import DataLoader

# 处理路径导入
cur_dir = os.path.dirname(__file__)
root_dir = os.path.abspath(os.path.join(cur_dir, ".."))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from utils import MyCarotidDataset

# 仅保留 1E 逻辑需要的键
ALL_KEYS = ("T", "Tend", "dT_1e")


@torch.no_grad()
def run_stats(
    ds_kwargs,
    root,
    batch=32,
    workers=8,
    device="cpu",
):
    ds_kwargs = dict(ds_kwargs)   
    ds_kwargs["normalize"] = False
    ds = MyCarotidDataset(**ds_kwargs)

    if hasattr(ds, "use_stats") and isinstance(ds.use_stats, dict):
        for k in ds.use_stats:
            ds.use_stats[k] = False

    # =========================================================================
    # [新增] 自定义 collate_fn 解决图像尺寸不一致问题
    # =========================================================================
    def mixed_size_collate(batch):
        """
        batch: list of tuples, e.g. [(img1, t1, ...), (img2, t2, ...)]
        解决：图像尺寸不一致时 stack 会报错，而统计脚本不需要图像，因此将图像保持为 list，向量进行 stack。
        """
        # 转置: [(img1, img2), (t1, t2), ...]
        transposed = list(zip(*batch))
        
        output = []
        for samples in transposed:
            first_elem = samples[0]
            
            if isinstance(first_elem, torch.Tensor):
                # 如果是图像 (维度通常为3: C, H, W)，不进行 stack，避免尺寸不一报错
                if first_elem.ndim == 3:
                    output.append(list(samples)) 
                else:
                    # 如果是向量 (维度通常为1: [3])，必须 Stack 以便后续计算统计量
                    output.append(torch.stack(samples))
            else:
                # 其他类型保持列表
                output.append(list(samples))
        return output
    # =========================================================================

    # 在 DataLoader 中使用自定义的 collate_fn
    dl = DataLoader(
        ds, 
        batch_size=batch, 
        num_workers=workers, 
        shuffle=False,
        collate_fn=mixed_size_collate  # <--- 关键修改
    )

    acc = defaultdict(lambda: {
        "n": 0,
        "mean": torch.zeros(3, dtype=torch.float64, device=device),
        "M2": torch.zeros(3, dtype=torch.float64, device=device),
        "min": torch.full((3,),  float("inf"),  dtype=torch.float64, device=device),
        "max": torch.full((3,), -float("inf"),  dtype=torch.float64, device=device),
    })

    def update(key, tensor):
        """
        tensor: [..., 3]，可包含 batch 维
        """
        t = tensor.to(device, torch.float64).view(-1, 3)
        st = acc[key]
        st["min"] = torch.minimum(st["min"], t.min(dim=0).values)
        st["max"] = torch.maximum(st["max"], t.max(dim=0).values)

        # Welford 算法更新均值和方差
        for v in t:
            st["n"] += 1
            delta = v - st["mean"]
            st["mean"] += delta / st["n"]
            st["M2"]   += delta * (v - st["mean"])

    for batch_tup in dl:
        if not isinstance(batch_tup, (list, tuple)):
            raise RuntimeError("Unexpected dataloader output type.")
        L = len(batch_tup)

        # 这里解包时，图像对应的变量（即 _）会变成一个 List[Tensor]，
        # 而不是 stack 后的 Tensor，但这不影响后续逻辑，因为你没有使用图像。
        if L == 9:
            # (img_i, img_e, T_i, T_e, R_i, R_e, dT_1e, dR_1e, e1e)
            _, _, T_i, T_e, _, _, dT_1e, _, _ = batch_tup
        elif L == 8:
            # (img_i,      T_i, T_e, R_i, R_e, dT_1e, dR_1e, e1e)
            _, T_i, T_e, _, _, dT_1e, _, _ = batch_tup
        else:
            raise RuntimeError(f"Unexpected batch tuple length {L}. Expect 8 or 9.")

        vals = {
            "T": T_i,
            "Tend": T_e,
            "dT_1e": dT_1e,
        }

        for k in ALL_KEYS:
            v = vals[k]
            update(k, v)

    stats = {}
    for k, s in acc.items():
        var = s["M2"] / max(s["n"] - 1, 1)
        std = torch.sqrt(var + 1e-12)
        rng = (s["max"] - s["min"]).clamp_min(1e-12)

        mid = (s["max"] + s["min"]) / 2
        half = (s["max"] - s["min"]) / 2
        half = half.clamp_min(1e-12)
        maxabs = torch.maximum(s["max"].abs(), s["min"].abs()).clamp_min(1e-12)

        stats[f"{k}_mean"] = s["mean"].float().cpu()
        stats[f"{k}_std"] = std.float().cpu()
        stats[f"{k}_min"] = s["min"].float().cpu()
        stats[f"{k}_rng"] = rng.float().cpu()
        stats[f"{k}_mid"] = mid.float().cpu()
        stats[f"{k}_half"] = half.float().cpu()
        stats[f"{k}_maxabs"] = maxabs.float().cpu()

        print(f"{k:6} | n={s['n']:<7d} | "
              f"mean {s['mean'].cpu().numpy()} | std {std.cpu().numpy()} | "
              f"min {s['min'].cpu().numpy()} | max {s['max'].cpu().numpy()}")

    os.makedirs(root, exist_ok=True)
    out_pt = os.path.join(root, "norm_stats_all.pt")
    torch.save(stats, out_pt)
    with open(os.path.splitext(out_pt)[0] + ".json", "w") as f:
        json.dump({k: v.tolist() for k, v in stats.items()}, f, indent=2)
    print("save to:", out_pt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/home/mingcong/project/us_carotid/pose_pilot/datasets/pose_data/")
    ap.add_argument("--split", default="train")
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--gpu", action="store_true")

    ap.add_argument("--image-interval", type=int, default=1)
    ap.add_argument("--include-last",   action="store_true", default=False)
    ap.add_argument("--return-end-image", action="store_true", default=False)

    ap.add_argument("--include-neg", choices=["none", "xy", "all"], default="xy", help="xy=仅 (x,y) 取负, all=整向量取负")
    ap.add_argument("--neg-keys", nargs="+", default=None, help=f"哪些键参与 -T 统计，默认全部：{', '.join(ALL_KEYS)}")

    args = ap.parse_args()

    ds_kwargs = dict(
        root_dir = args.root,
        split = args.split,
        image_interval = args.image_interval,
        transform = None,
        normalize = False,               
        return_end_image = args.return_end_image,
        include_last = args.include_last,
        flip_expand=True,
    )

    # neg_keys 这里的逻辑在你原始代码中未使用到 update 里的逻辑，
    # 如果需要用到翻转统计，请确保你的 dataset 确实返回了翻转后的数据，或者在这里进行处理。
    # 按照目前代码逻辑，这里仅做参数保留。
    neg_keys = tuple(args.neg_keys) if args.neg_keys is not None else ALL_KEYS

    device = "cuda" if args.gpu and torch.cuda.is_available() else "cpu"
    a = time.time()
    run_stats(
        ds_kwargs=ds_kwargs,
        root=args.root,
        batch=args.batch,
        workers=args.workers,
        device=device,
    )
    b = time.time()
    print(f"[done] elapsed: {b - a:.2f}s")


if __name__ == "__main__":
    main()
