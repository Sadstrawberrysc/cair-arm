# 数字人单点到 RM75 Probe TCP

## 一次启动完整流程

完成本轮现场检查后，在 `camera` 环境中从仓库根目录运行（默认真实运动，速度3）：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py
```

先打开数字人窗口；按 `s` 保存，按 `q` 退出后，入口检查本次文件已更新，读取第一行法向量，
读取第一个采样点，即画面最下面的红点（像素 y 最大；由相机保存前排序）。用新外参计算 `p_base = R_base_camera @ p_camera + t_base_camera`，
然后调用 `arm_probe_pose --target-pose-m-rad ... --controller-movej-p`，自动传入速度3和双执行参数，
通过控制器 `movej_p` 执行单次初始位姿移动。此流程仅负责到达扫描初始位姿，不是生产扫描控制。
相机保存的是 Z≤0 的朝相机外法向，入口取反后只用外参旋转：
`n_inward_base = -R_base_camera @ normalize(n_camera)`，不加平移。
用户确认阵列长边沿 Tool-Y，初始图像要求短轴切面。直接构造目标坐标系：
`Z = normalize(n_inward_base)`，`t_surface = normalize(t - dot(t,Z)*Z)`，
`X = -t_surface`，`Y = Z × X`，得到右手旋转矩阵 `[X,Y,Z]`。
因此 Tool-YZ 成像平面垂直于血管投影切向，+Z 朝内，-X 沿采样方向。
不再生成长轴姿态或叠加固定 -90° 旋转；轴符号保留原最终朝向以避免额外翻转。
切向取第一点指向后续首个非重复点（距离大于1微米）的方向，按当前保存顺序由画面底部向后续点。
方向只用外参旋转，不加平移；完整姿态不再依赖当前探头姿态。程序将旋转矩阵转换为 ZYX
欧拉角，通过完整六维探头位姿参数交给 Robot，后者仍负责 TCP 到 ArmTip 换算。
至少需要两个不同采样点；点不足、全部重复或归一化切向投影长度小于0.001 时拒绝运动。
这是局部采样连线的切向近似，依赖深度质量及采样顺序；像素 y 排序不保证弯曲或分支血管的拓扑顺序。
入口对采样点添加固定50 mm外法向偏置：`p_target = p_base - 0.050 * n_inward_base`。
此偏置施加在 Probe TCP 目标上，不是 Base-Z 固定偏移，也不修改188 mm工具长度；
Robot 随后执行已有 TCP 到 ArmTip 换算，不重复添加偏置。短轴姿态保持不变。
日志分别打印表面采样点、偏置距离和最终目标；这是终点几何距离，不保证整条 MoveJ 轨迹保持该距离。
法向来自局部深度平面，不保证解剖方向或估计精度；
新姿态需先 dry-run 并现场核对方向和短轴图像；几何对齐不代表已验证超声成像效果。
MoveJ 途中不保证直线或始终保持该朝向。

若只需读取状态并预览目标（不检查控制器逆解或可达性），运行：

```bash
python infer/hand_eye_calibration-main/run_probe_target.py --dry-run
```

无参数命令在本轮保存并退出数字人后会进入真实 MoveJ_P 流程，直接把换算后的 ArmTip 位姿交给控制器。
此模式不执行本地 ServoJ 迭代 IK、模型一致性、3° 关节裕量、奇异性、关节变化幅度和关节路径预检查，
也没有预期目标关节角可用于最终关节误差检查。控制器保护不等同于原本的这些本地检查。
仍检查标定链、启动状态、命令返回结果和最终 Probe TCP 位置/姿态误差；失败请求停止，不重试运动。
独立调用 `arm_probe_pose` 不加 `--controller-movej-p` 时仍使用原有本地规划模式。
`Ctrl+C` 中断流程；数字人异常退出、未保存或文件非法时不会调用 Robot。不自动重试运动。
数字人目前也支持 Esc 保存退出，此入口会把它视为有效的新快照。

需要构建 Robot 时：

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_MAINTENANCE_TOOLS=ON
cmake --build infer/Robot/build --target arm_probe_pose -j2
```

当前外参保存在同目录 `d455_to_rm75_base.json`，定义 `T_base_camera`：把 D455 彩色光学
坐标（米）映射到 RM75 Base 坐标（米）。

先转换数字人保存的单点：

```bash
source /home/cair-jacen/anaconda3/etc/profile.d/conda.sh
conda activate camera
cd /home/cair-jacen/uspilot_ctrl-main/infer/hand_eye_calibration-main
python transform_point.py --camera-point-m X Y Z
```

复制输出的 Base 位置，回到仓库根目录做 Robot dry-run：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/build/arm_probe_pose \
  --target-position-base-m "BASE_X,BASE_Y,BASE_Z" \
  --tool-calibration infer/Robot/build/rm75_force_calibration.json \
  --velocity 1
```

上述手动命令未传入法向，仍保留原行为：Robot 读取启动时的 ArmTip 状态，通过 `CalibratedFrameChain` 取得当前 Probe TCP 姿态，
打印完整 `Base→Probe_TCP [x,y,z,rx,ry,rz]` 并运行 IK。单点模式不添加50 mm偏置。

当前新外参由用户确认，但尚未记录独立验证结果；转换输出不是运动授权。真机执行仍需
目标预览、已知点和 Base 方向现场核对、整段净空确认及明确授权。
初次验证采用悬空、速度1、小位移，并核对实际到位误差；离线检查不代表真机已验收。
