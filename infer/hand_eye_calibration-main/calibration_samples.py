"""标定数据校验和机器人位姿转换（内部模块）。"""
import numpy as np
import yaml

def load_samples(path):
    images = list(path.glob("*.jpg"))
    if not images:
        raise ValueError(f"目录中没有 JPG 图像：{path}；请检查是否选中了旧的空目录")
    if any(not f.stem.isdigit() or int(f.stem) < 1 for f in images):
        raise ValueError("图像必须以 1.jpg、2.jpg 等正整数编号，编号对应 poses.txt 行号")
    pose_path = path / "poses.txt"
    if not pose_path.is_file():
        raise ValueError(f"缺少机器人位姿文件：{pose_path}")
    poses = np.loadtxt(pose_path, delimiter=",", ndmin=2)
    if poses.shape[1] != 6 or not np.isfinite(poses).all():
        raise ValueError("poses.txt 每行必须包含六个有限数值 x,y,z,rx,ry,rz（m、rad）")
    images.sort(key=lambda f: int(f.stem))
    ids = [int(f.stem) for f in images]
    if ids != list(range(1, len(poses) + 1)):
        raise ValueError("图片编号与位姿行数不一致；需要从 1 开始连续编号且一图一行，不能猜测配对")
    return images, poses


def load_board(config_path):
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get("checkerboard_args"), dict):
        raise ValueError("配置缺少 checkerboard_args")
    board = config["checkerboard_args"]
    xx, yy, length = board.get("XX"), board.get("YY"), board.get("L")
    if type(xx) is not int or type(yy) is not int or xx < 2 or yy < 2:
        raise ValueError("XX、YY 必须是大于等于 2 的内部角点数")
    if isinstance(length, bool) or not isinstance(length, (int, float)) or not np.isfinite(length) or length <= 0:
        raise ValueError("L 必须为正的有限格长，单位米")
    return xx, yy, float(length)


def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz@Ry@Rx

    return R


def pose_to_homogeneous_matrix(pose):

    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)

    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]

    return H

