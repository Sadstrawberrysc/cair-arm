import os, re
from datetime import datetime
import numpy as np
import matplotlib
matplotlib.use("Agg")  
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
from scipy.spatial.transform import Rotation as R
from sklearn.decomposition import PCA
from scipy.spatial import cKDTree


_TS_RE = re.compile(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}$")
def ts_str_to_ms(ts: str) -> int:
    return int(datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f").timestamp() * 1e3)

def read_txt(txt_path: str):
    ts_abs, vecs, raw_lines_valid = [], [], []
    with open(txt_path, "r") as f:
        for ln in f:
            parts = ln.strip().split(maxsplit=2)
            if len(parts) < 3:
                continue
            date, time, rest = parts
            ts_str = f"{date} {time}"
            if not _TS_RE.match(ts_str):
                continue
            nums = re.sub(",", " ", rest).split()
            if len(nums) != 18:
                continue
            ts_abs.append(ts_str_to_ms(ts_str))
            vecs.append([float(x) for x in nums])
            raw_lines_valid.append(ln.rstrip("\n"))
    if not ts_abs:
        raise ValueError(f"{txt_path} has no valid timestamp lines")
    base_ms = ts_abs[0]
    ts_rel = np.asarray(ts_abs, np.int64) - base_ms 
    vecs = np.asarray(vecs, np.float32)
    vecs[:, :3] *= 100.0 
    return ts_rel, vecs, raw_lines_valid

def split_pose(v):
    T = v[0:3]
    R = v[3:12].reshape(3, 3)
    return T, R

def geodesic_angle(R, R_ref):
    Rt = np.matmul(R.transpose(0, 2, 1), R_ref)
    trace = np.trace(Rt, axis1=-2, axis2=-1)
    cos_th = np.clip((trace - 1) / 2, -1.0, 1.0)
    return np.arccos(cos_th) 


def rotmats_to_rotvecs(R_all):
    return R.from_matrix(R_all).as_rotvec()

def rotmats_to_logvecs(R_all, R_ref=None):
    if R_ref is None:
        R_ref = R_all[-1]
    R_rel = R.from_matrix(R_ref.T @ R_all)
    return R_rel.as_rotvec() 

def quat_continuous(R_all):
    q = R.from_matrix(R_all).as_quat() 
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0.0:
            q[i] *= -1 
    return q


def plot_rotvec_space(rotvecs, idx_keep=None, save_path="rotvec_space.png", title="Rotation distribution",
                      mode="density", 
                      subsample_ratio=0.05,
                      dpi=300):

    N = len(rotvecs)
    if mode == "subsample":
        keep_all = np.random.choice(N, max(1, int(N * subsample_ratio)), replace=False)
        rot_all = rotvecs[keep_all]
        colors = "steelblue"
        sizes = 2
    elif mode == "density":
        tree = cKDTree(rotvecs)
        dist,_ = tree.query(rotvecs, k=6) 
        rho = 1.0 / (dist[:, 1:].mean(axis=1)+1e-6) 
        colors = rho
        rot_all= rotvecs
        sizes  = 2
    else:  
        rot_all= rotvecs
        colors = "grey"
        sizes = 2

    fig = plt.figure(figsize=(8, 6))
    ax  = fig.add_subplot(111, projection="3d")
    sc  = ax.scatter(rot_all[:,0], rot_all[:,1], rot_all[:,2],
                     s=sizes, c=colors, cmap="viridis", alpha=0.7,
                     label="all frames")

    if mode == "density":
        cbar = fig.colorbar(sc, ax=ax, shrink=0.6)
        cbar.set_label("local density (1/avg kNN dist)")

    if idx_keep is not None and len(idx_keep):
        idx_keep = np.asarray(idx_keep,int)
        ax.scatter(rotvecs[idx_keep,0], rotvecs[idx_keep,1], rotvecs[idx_keep,2],
                   s=10, marker="x", color="red", label="sampled",alpha=0.6, depthshade=False)

    ax.set_xlabel(r"$v_x$")
    ax.set_ylabel(r"$v_y$")
    ax.set_zlabel(r"$v_z$")
    # ax.set_title(title + f"\nmode: {mode}")
    ax.set_title(title)

    ax.legend(loc="best")
    plt.tight_layout()
    os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
    plt.savefig(save_path, dpi=dpi)
    plt.close(fig)
    print(f"[Saved] {save_path}")


# def plot_rotvec_space(rotvecs, idx_keep = None, save_path = "rotvec_space.png", title = "Rotation distribution", dpi = 300):
#     fig = plt.figure(figsize=(8, 6))
#     ax  = fig.add_subplot(111, projection="3d")

#     ax.scatter(rotvecs[:, 0], rotvecs[:, 1], rotvecs[:, 2], s=1, alpha=0.4, label="all frames")

#     if idx_keep is not None and len(idx_keep):
#         idx_keep = np.asarray(idx_keep, int)
#         ax.scatter(rotvecs[idx_keep, 0], rotvecs[idx_keep, 1], rotvecs[idx_keep, 2],
#                    s=20, marker="*", color="black", label="sampled", depthshade=False)

#     ax.set_xlabel(r"$v_x$")
#     ax.set_ylabel(r"$v_y$")
#     ax.set_zlabel(r"$v_z$")
#     ax.set_title(title)
#     ax.legend(loc="best")
#     plt.tight_layout()

#     os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
#     plt.savefig(save_path, dpi=dpi)
#     plt.close(fig)
#     print(f"[Saved] {save_path}")


def downsample_pose(in_txt, out_txt, n_keep = 20, angle_step_deg = 1.0):
    ts_rel, vecs, raw_lines = read_txt(in_txt)
    N = len(raw_lines)
    if N == 0:
        raise RuntimeError("have no data")    

    R_all = np.array([split_pose(v)[1] for v in vecs], dtype=np.float64)

    dists = geodesic_angle(R_all, R_all[-1])
    D_total = float(dists[0]) 
    D_deg   = np.degrees(D_total) 

    if n_keep > 0:                         
        targets = np.linspace(0.0, D_total, n_keep)
    else:                                  
        targets_deg = np.arange(0.0, D_deg + 1e-6, angle_step_deg)
        targets = np.radians(targets_deg)

    idx_keep = np.unique([np.abs(dists - t).argmin() for t in targets])
    idx_keep.sort()                        

    print(f"total {N},keep {len(idx_keep)}", f"({'manual' if n_keep>0 else f'step:  {angle_step_deg}° '})")

    q_cont = quat_continuous(R_all)
    xyz = q_cont[:, :3]
    plot_rotvec_space(xyz, idx_keep, '/home/jiuan/AAAworkspace/Myproject/slicelocalization/carotid_copilot/visual/ds_quat.png', title="Before vs. FPS", mode="subsample", subsample_ratio=0.2, )


    out_dir = os.path.dirname(out_txt)
    if out_dir and out_dir != '' and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    with open(out_txt, "w") as f:
        for idx in idx_keep:
            f.write(raw_lines[idx] + "\n")

    print(f"save to: {out_txt}")


def downsample_pose_fps(in_txt, out_txt, n_keep = 20, angle_step_deg = 1.0):

    ts_rel, vecs, raw_lines = read_txt(in_txt)
    N = len(raw_lines)
    if N == 0:
        raise RuntimeError("have no data")

    R_all = np.array([split_pose(v)[1] for v in vecs], dtype=np.float64)

    if n_keep <= 0:
        D_total = geodesic_angle(R_all[[0]], R_all[-1])[0]
        D_deg   = np.degrees(D_total)
        n_keep  = int(D_deg // angle_step_deg) + 1 
        n_keep  = min(n_keep, N) 
        print(f"total angle≈{D_deg:.2f}°, angle_step={angle_step_deg}° , " f"n_keep={n_keep}")

    if n_keep > N:
        raise ValueError(f"n_keep={n_keep} cannot over {N}")

    selected = [N - 1] 
    dist_min = geodesic_angle(R_all, R_all[selected[0]])

    for _ in range(1, n_keep):
        idx = int(dist_min.argmax())
        selected.append(idx)
        dist_new = geodesic_angle(R_all, R_all[idx])
        dist_min = np.minimum(dist_min, dist_new)

    selected = np.sort(np.array(selected, int))

    print(f"[FPS] origin {N}, keep {len(selected)}。")



    q_cont = quat_continuous(R_all)
    xyz = q_cont[:, :3]
    plot_rotvec_space(xyz, selected, '/home/jiuan/AAAworkspace/Myproject/slicelocalization/carotid_copilot/visual/fps_quat.png', title="Before vs. FPS", mode="subsample", subsample_ratio=0.2, )

    logvecs = rotmats_to_rotvecs(R_all)
    plot_rotvec_space(logvecs, selected, '/home/jiuan/AAAworkspace/Myproject/slicelocalization/carotid_copilot/visual/fps_logv.png', title="Before vs. FPS", mode="subsample", subsample_ratio=0.05, )

    out_dir = os.path.dirname(out_txt)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    with open(out_txt, "w") as f:
        for idx in selected:
            f.write(raw_lines[idx] + "\n")
    print(f"save to: {out_txt}")




if __name__ == "__main__":

    downsample_pose(
        in_txt  = "/home/jiuan/AAAworkspace/Myproject/slicelocalization/EchoWorld-main/origindata/carotid_artery_0611/data_clean/train/202506111458/202506111458.txt",
        out_txt = "pose_down.txt",
        n_keep  = 0
    )

    # downsample_pose_fps(
    #     in_txt  = "/home/jiuan/AAAworkspace/Myproject/slicelocalization/EchoWorld-main/origindata/carotid_artery_0611/data_clean/train/202506111458/202506111458.txt",
    #     out_txt = "pose_down_fps.txt",
    #     n_keep  = 0
    # )
