import re, argparse, itertools
from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")        
import matplotlib.pyplot as plt

_pattern = re.compile(r"Epoch:(\d+)/\d+,\s*Loss:([\d.]+)")
def parse_log(path: Path):
    ep, ls = [], []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = _pattern.search(line)
            if m:
                ep.append(int(m.group(1)))
                ls.append(float(m.group(2)))
    if not ep:
        raise ValueError(f"{path} 中未找到 Epoch/Loss 记录")
    return pd.Series(ls, index=ep)  

def main():
    parser = argparse.ArgumentParser(
        description="Draw Hiera‑MAE training Loss curves")
    parser.add_argument("--logs", nargs="+",default=['/home/jiuan/Downloads/HY0714/With300wPretrain.txt', '/home/jiuan/Downloads/HY0714/Without300wPretrain.txt'], help="一个或多个训练日志 txt 路径")
    parser.add_argument("-o", "--output", default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/carotid_copilot/visual/simmimloss.png',
                        help="输出图片路径")
    args = parser.parse_args()

    colors = itertools.cycle(
        ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
         "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
         "#bcbd22", "#17becf"]
    )
    plt.figure(figsize=(9, 6))
    series_list, labels = [], []

    for log_path in args.logs:
        p = Path(log_path).expanduser().resolve()
        s = parse_log(p)
        series_list.append(s)

        min_epoch = s.idxmin()
        min_loss  = s[min_epoch]
        color     = next(colors)
        plt.plot(s.index, s.values, label=f"{p.stem} (min={min_loss:.4f})",
                 linewidth=2, color=color)

        plt.scatter(min_epoch, min_loss, color=color, marker="o", zorder=5)
        plt.annotate(f" {min_loss:.4f}@{min_epoch}",
                     xy=(min_epoch, min_loss),
                     xytext=(5, -15), textcoords="offset points",
                     fontsize=10, color=color)
        labels.append((p.stem, min_loss, min_epoch, color))

    if len(series_list) == 2:
        (name0, min0, _, col0), (name1, min1, _, col1) = labels
        if min0 < min1:
            s_a, s_b, col_a = series_list[0], series_list[1], col0
            thresh          = min1
        else:
            s_a, s_b, col_a = series_list[1], series_list[0], col1
            thresh          = min0
        cross_epoch = next(ep for ep, l in s_a.items() if l < thresh)
        cross_loss  = s_a[cross_epoch]

        plt.scatter(cross_epoch, cross_loss,
                    color=col_a, marker="D", s=60, zorder=6)
        plt.annotate(f"{cross_loss:.4f} @ {cross_epoch} epoch",
                     xy=(cross_epoch, cross_loss),
                     xytext=(5, 10), textcoords="offset points",
                     fontsize=10, color=col_a)

    plt.title("SIMMIM", fontsize=18)
    plt.xlabel("Epoch", fontsize=14)
    plt.ylabel("Loss",  fontsize=14)
    plt.tick_params(axis="both", labelsize=12)
    plt.legend(fontsize=12)
    plt.grid(alpha=0.3)
    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=300)
        print(f"已保存图像 -> {args.output}")
    else:
        plt.show()

if __name__ == "__main__":
    main()


