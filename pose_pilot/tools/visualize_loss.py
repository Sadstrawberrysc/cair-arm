import argparse, itertools, textwrap
from pathlib import Path
from typing import Dict

import pandas as pd
import matplotlib
matplotlib.use("Agg")  
import matplotlib.pyplot as plt


def plot_metrics(
        dfs,
        x_col,
        y_cols,
        figsize=(8, 5),
        out_dir=None):

    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    markers = "o^sD*vx+"
    marker_cyc = itertools.cycle(markers)

    for y in y_cols:
        fig, ax = plt.subplots(figsize=figsize)

        for label, df in dfs.items():
            if y not in df.columns:
                print(f"⚠ {label} 缺列 {y}，跳过")
                continue

            idx_min = df[y].idxmin()
            x_min, y_min = df.loc[idx_min, [x_col, y]]
            label_with_min = f"{label}  (min={y_min:.3f})"

            m = next(marker_cyc)
            ax.plot(df[x_col], df[y],
                    label=label_with_min,
                    marker=m, markevery=8, ms=5, lw=1.5)

            ax.scatter([x_min], [y_min],
                       color=ax.lines[-1].get_color(),
                       s=50, zorder=3)

        ax.set_title(textwrap.fill(y, 30), fontsize=16)
        ax.set_xlabel(x_col)
        ax.set_ylabel(y)
        ax.tick_params(axis='both', which='major', labelsize=16)
        ax.legend(fontsize=16, ncol=2, frameon=False)
        ax.grid(alpha=.3, linestyle='--')
        fig.tight_layout()

        if out_dir:
            png_path = out_dir / f"{y}.png"
            fig.savefig(png_path, dpi=300)     
            print(f"saved  {png_path}")
        else:
            plt.show()

        plt.close(fig)



def parse_args():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="指标对比图")
    p.add_argument('--csvs', nargs='+', default=['/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swinHY_freeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swinHY_nofreeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swinHY5_freeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swinHY5_nofreeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swintinny_freeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/swintinny_nofreeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/mae_freeze/train_log.csv',
                                               '/home/jiuan/AAAworkspace/Myproject/slicelocalization/runs/runs1/mae_nofreeze/train_log.csv'],help='要读取的 CSV 路径，支持通配符 *.csv')
    
    p.add_argument('-x', '--x_col', default='epoch', help='横轴列名')
    p.add_argument('-y', '--y_cols', nargs='+', default=['train_total', 'val_total', 'train_r', 'val_r'], help='要绘制的指标列')
    p.add_argument('--figsize', type=float, nargs=2, default=(12, 7), metavar=('W', 'H'), help='单张图尺寸')
    p.add_argument('-o', '--out_dir', type=Path, default='/home/jiuan/AAAworkspace/Myproject/slicelocalization/carotid_copilot/visual/runs250625', help='若指定,PNG 输出目录；否则只弹窗显示')
    return p.parse_args()


def main():
    args = parse_args()

    dfs: Dict[str, pd.DataFrame] = {}
    for p in map(Path, args.csvs):
        if not p.is_file():
            print(f"找不到 {p}")
            continue
        df = pd.read_csv(p, header=None)
        df.columns = ['epoch',
                      'train_total', 'train_t', 'train_r',
                      'val_total', 'val_t', 'val_r']
        
        run_name = p.parent.name        
        dfs[run_name] = df


    if not dfs:
        raise SystemExit("没读到CSV, he退")

    plot_metrics(dfs,
                 x_col=args.x_col,
                 y_cols=args.y_cols,
                 figsize=tuple(args.figsize),
                 out_dir=args.out_dir)


if __name__ == "__main__":
    main()



