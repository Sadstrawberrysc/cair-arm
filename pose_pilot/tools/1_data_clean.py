import os, re, shutil, argparse
from pathlib import Path
from datetime import datetime
import numpy as np

_TS_RE = re.compile(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}$")
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")

def ts_str_to_ms(ts: str) -> int:
    """
    兼容两种格式：
    1. 标准格式: "2025-06-25 17:41:22.951"
    2. 紧凑格式 (你的图片名): "20250625_174122951"
    """
    ts = ts.strip()
    try:
        # 情况 A: 标准格式 (txt 文件常见)
        dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        try:
            # 情况 B: 紧凑格式 YYYYMMDD_HHMMSSmmm (图片文件名)
            # Python 的 %f 期望 6 位微秒，你的数据只有 3 位毫秒，所以必须手动处理
            if "_" in ts and len(ts.split("_")[-1]) > 6: 
                # 处理类似 20250625_174122951
                parts = ts.split("_")
                date_time_part = parts[0] + parts[1][:6] # 20250625 + 174122
                ms_part = parts[1][6:] # 951
                
                dt = datetime.strptime(date_time_part, "%Y%m%d%H%M%S")
                # 补全毫秒
                dt = dt.replace(microsecond=int(ms_part) * 1000)
            else:
                # 简单的备用尝试
                dt = datetime.strptime(ts, "%Y%m%d_%H%M%S%f")
        except ValueError:
            raise ValueError(f"无法解析的时间格式: {ts}")

    return int(dt.timestamp() * 1000)

def read_txt(txt_path: Path):
    ts_list, vec_list, raw_lines = [], [], []
    
    with open(txt_path, "r") as f:
        lines = f.readlines()
        
    for i, line in enumerate(lines):
        line = line.strip()
        if not line: continue
        
        parts = line.split()
        
        # 你的 txt 数据可能有两种情况：
        # 情况 1: "2025-06-25 17:41:20 ..." (日期 时间 分开) -> 数据从第2列开始
        # 情况 2: "20250625_174122951 ..." (紧凑时间戳) -> 数据从第1列开始
        
        try:
            # 尝试解析前两个元素组合 (情况 1)
            if len(parts) > 2 and "-" in parts[0]:
                ts_str = f"{parts[0]} {parts[1]}"
                nums_start_idx = 2
            else:
                # 尝试解析第一个元素 (情况 2)
                ts_str = parts[0]
                nums_start_idx = 1
            
            # 解析时间
            ts_ms = ts_str_to_ms(ts_str)
            
            # 解析后面的 18 个数字 (处理可能的逗号)
            vec_strs = parts[nums_start_idx:]
            # 有时候数据中间混有逗号，先合并再替换再拆分，或者直接逐个替换
            clean_vec = []
            for s in vec_strs:
                s = s.replace(",", "")
                if s: clean_vec.append(float(s))
            
            # 只有当找到至少 12 个位姿数据时才算有效行
            if len(clean_vec) >= 12:
                ts_list.append(ts_ms)
                vec_list.append(clean_vec) # 这里存 list，最后转 numpy
                raw_lines.append(line)
                
        except (ValueError, IndexError):
            # 只有在调试时取消注释，查看哪些行被跳过了
            # print(f"Skipping line {i} in {txt_path.name}: format error")
            continue

    return (
        np.asarray(ts_list, dtype=np.int64),
        np.asarray(vec_list, dtype=np.float32),
        raw_lines,
    )

def nearest_idx(ts_arr, target_ms):
    return np.abs(ts_arr - target_ms).argmin()

def process_seq(seq_dir: Path, dst_dir: Path):
    imgs = sorted([p for p in seq_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS])
    if not imgs:
        return 0, 0
    txt_files = [p for p in seq_dir.iterdir() if p.suffix.lower() == ".txt"]
    if len(txt_files) != 1:
        return 0, 0
        
    # 读取 txt
    ts_arr, vecs, raw_lines = read_txt(txt_files[0])

    # === 【关键修改】如果 txt 没读出任何数据，直接返回，不要往下跑 ===
    if len(ts_arr) == 0:
        print(f"[Warn] Skipped {seq_dir.name}: txt file is empty or format invalid.")
        return 0, 0
    # ==========================================================

    dst_dir.mkdir(parents=True, exist_ok=True)
    new_txt_lines = []
    kept, removed = 0, 0
    last_pose12 = None

    for img_path in imgs:
        try:
            # 解析图片名的时间
            ts_ms = ts_str_to_ms(img_path.stem)
            
            # 找最近邻 (现在 ts_arr 保证不为空，这里就不会报错了)
            idx = nearest_idx(ts_arr, ts_ms)
            
            pose12 = vecs[idx][:12]

            if last_pose12 is not None and np.allclose(pose12, last_pose12, atol=1e-6):
                removed += 1
                continue
            last_pose12 = pose12.copy()
            kept += 1
            shutil.copy2(img_path, dst_dir / img_path.name)
            new_txt_lines.append(raw_lines[idx])
            
        except ValueError as e:
            print(f"Skipping image {img_path.name}: {e}")
            continue

    if new_txt_lines:
        with open(dst_dir / txt_files[0].name, "w") as f:
            f.write("\n".join(new_txt_lines) + "\n")
            
    return kept, removed

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--src_root", default='/home/mingcong/project/embodied/us_proj/data0625/data/', help="原始数据根目录")
    parser.add_argument("--dst_root", default='/home/mingcong/project/us_carotid/pose_pilot/raw_data/data_angle_new/data0625/', help="清洗后数据根目录")
    args = parser.parse_args()

    src_root = Path(args.src_root).resolve()
    dst_root = Path(args.dst_root).resolve()
    total_kept = total_removed = 0

    for seq_dir in [p for p in src_root.rglob("*") if p.is_dir()]:
        img_exist = any(f.suffix.lower() in IMAGE_EXTS for f in seq_dir.iterdir())
        txt_count = sum(1 for f in seq_dir.iterdir() if f.suffix.lower() == ".txt")
        if not img_exist or txt_count != 1:
            continue                                   # 跳过无效目录

        rel_path = seq_dir.relative_to(src_root)
        kept, removed = process_seq(seq_dir, dst_root / rel_path)
        total_kept   += kept
        total_removed+= removed
        if kept or removed:
            print(f"[{rel_path}] kept {kept:4d} | removed {removed:4d}")

    print(f"\nFinished. kept={total_kept}  removed={total_removed}")

if __name__ == "__main__":
    main()
