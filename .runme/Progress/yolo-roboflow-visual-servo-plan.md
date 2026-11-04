# 自动扫描点定位与腕部视觉伺服：计划及历史

制定：2026-09-24；本文件保留早期方案与验证沿革，当前实现见
[腕部架构](../../infer/Camera_wrist/ARCHITECTURE.md)、[detector 架构](../../infer/Camera_wrist/detector/ARCHITECTURE.md)，
操作见 [腕部手册](../run/camera-wrist-operations.md)。旧目录、已移除工具和历史数值不作为当前启动指令。

## 当前实现与剩余验收

Roboflow Object Detection 的唯一合格框中心作为 `scan_start`，相机以同一 RGB-D 源帧初始化 LocalTracker，
赶帧到新鲜观测后经 Redis 交给 Robot。模型归入 `Camera_wrist/detector/`，相机 / worker 分进程；
默认模型、阈值和缓存见配置源码。`--manual` 保留人工选点，两种模式均连接 Redis。

2026-10-12 起模型模式在新鲜 Robot Hold、完整 RGB-D、有效新目标和回传确认后自动请求首次跟随；
历史“必须首次人工 b”和 YOLO-Pose 训练路线不代表当前行为。
未完成：独立目标真值、多主体定位误差、真实连续融合 / 恢复、外参 / 时序 / 悬停净空；接触力控交接未实现。
最新验证见 [腕部进展](camera-wrist-progress.md)和 [模型进展](carotid-begin-progress.md)。

## 目标与数据原则

- 定位一个选定侧颈部体表初始扫描点 `scan_start`；只使用真实腕部 RGB-D、相机内外参、实测状态和任务配置，不使用数字人衍生信息。
- 用户要求不放可见标记；可见线索是下颌和颈部边界，不要求锁骨。真实目标必须入镜，不能用画面内替代点冒充视野外起点。
- 标签来自统一协议的专家超声示教 / 独立测量及相机配准，保留来源、时间和误差；探头遮挡、移开后回弹与人体运动需计入对应误差。
- 同一受试者 / 会话不跨 train、val、test；独立测试主体留出。模型预测、几何有效和用户目视确认不能替代独立真值。
- 原始 RGB-D、标签证据、权重、私有凭据和运行报告留在受控本机；API Key 不写仓库。上传前核对授权与工作区可见性。

## 历史数据与验证

### 2026-09-25：P001 左侧首批采集

Gemini `CV2L360000HZ`，Close Range、视差 256，RGB / 深度 640×480@30。
会话 `20260925_153731_8ff02ad8`：30 组、59.03 s，间隔 2.035～2.036 s，帧龄 34.30～37.06 ms，
彩深差最大 0.169 ms；全图有效深度比例 60.81%～85.75%，不代表目标点有效。

首图草稿 `ff3f827350d040ffb5fe79f7b2e8d105`：像素 `(392,468)`，相机点约
`(8.223,27.508,44.100)` mm，法向约 `(-0.00495,-0.06180,-0.99808)`，平面 RMS 0.248 mm、内点 1.0。
当时因 40 px 固定边距拒绝；用户随后确认真实起点在视野外，已追加 out_of_view 草稿修订并拒绝导出。
其余 29 张未标注，不能当负样本。当前已取消固定 40 px 门，保留实际纹理 / 深度质量门。

当时 P0 工具 collect / annotate / check / report / export 保存 RGB-D、单点证据和主体划分；
未记录机器人反馈或自动示教配准。最初 14 项、加入报告后 20 项离线测试通过，仅采相机，未连 / 移动 Robot。
报告曾位于 `infer/Carotid_begin/reports/`，旧入口和目录不再维护。

### 2026-09-28：v4 单图与输出规则变更

`ultralytics` 中 Inference 1.7.2 对 `data/pictures/1.png`（848×530）加载 v4，初次使用 CPU provider。
旧框下边中点 `(490,529)` 触及底行，被当时 40 px 边距拒绝。
用户随后改为唯一框中心：v4 类别 carotid、置信度 0.8667103052，框 `[441,279,539,529]`，中心 `(490,404)`。
用户目视确认与 P0 重合，本机无独立真值坐标；单图没有同帧深度 / 内参，不能据此评价三维误差或实时跟踪。

当时无 Docker，camera Python 3.8、carotid Python 3.10，GPU 为 RTX 4080 Laptop 12 GB；
独立 `local_infer.py` 入口的 help / 20 项离线回归通过。后续 worker 运行环境与 GPU 实测见模型进展。

### 2026-09-28：同步 RGB-D 会话回放

会话 `20260928_142615_003bdcb2`，P001 左侧 10 组，完整性 10/10；彩深差最大约 1.03 ms，
帧龄最大约 34.52 ms，有效深度比例约 0.67～0.86，尚无逐帧独立真值。
当时 replay_session 只加载一次模型并输出候选 / 几何 / 纹理；用户运行 v1 后 6 帧唯一候选且几何通过，4 帧多框拒绝。
被接受框底均为 y=480；推理中位 136.0 ms、P95 167.3 ms，帧龄加推理 P95 约 200.7 ms，未含传输 / 跟踪。
v1 未保留多框数值，工具随后补齐全部检测框；历史报告为 `capture_audit_20260928_142615_003bdcb2.json` 等。

### 2026-09-28：真实视频只读预览与难例

旧 `live_preview.py` / `camera_stream.py` 经私有 socketpair 做只读窗口预览，09-30 归并到 click_follow，旧文件已移除。
20 帧抽查：1 帧候选、6 帧过期、13 帧无唯一框（11 帧零框、2 帧双框）；7 个预测框都触底。
推理中位 161.4 ms，最终帧龄中位 223.7 ms；26 项离线测试通过，直接窗口有界显示 5 帧正常退出。
未做 LK / Robot 连续验证，也无独立 P0 真值。

长记录 2250 帧：1184 帧无原始框、236 帧唯一框置信度 <0.5；692 个候选中 621 个贴底。
229 帧程序通过、405 帧过期、1616 帧拒绝，推理中位 163.1 ms、最终帧龄 222.5 ms。
日志未存图 / 真值，不是模型精度。随后增加 s 键 RGB-D 难例保存、主体留出导出和独立复核点误差统计。

### 2026-09-28：用户自采与训练交接

用户报告自采约 50 张并在 Roboflow 训练；当时本机未取得文件、划分、标注和结果，不与 P001 的 30 组混计。
提供的 [v4 页面](https://app.roboflow.com/songjiatong/carotid-detect-c9gi9/4?openModelEval=true) 为账号页面，URL 不能证明模型类型、训练完成或评估指标。
交接需记录项目 / 模型 / 数据版本、框或关键点的目标定义、主体划分、预测样例及本地部署格式，离线复核后再联调。
旧 ANNOTATION / DEPLOYMENT / RETRAINING 文档已移除；当前模型更换方法见[模型进展](carotid-begin-progress.md#更换模型)。

## 早期工作包（历史方案）

原 YOLO-Pose 方案仅预测一个自定义关键点；COCO 人体关键点不能直接提供颈动脉起点。
当时建议 Keypoint Detection 类别 `carotid_scan_region`、`kpt_shape: [1,3]`，
实例框由已配准起点邻域生成，不手绘任意大小颈部框。当前使用检测框中心，不能混用任务标签 / 导出格式。

| 阶段 | 工作与交付 | 原估算 |
| --- | --- | --- |
| P0 目标与测量 | 20～30 组试标、明确可见性 / 侧别 / 真值 / 误差预算；多候选或无效深度拒绝，不自动换目标 | 2～4 天 |
| P1 数据与标注 | 按主体采集不同姿态 / 光照 / 遮挡 / 无目标场景；100～200 张首批双人复核；冻结 split、规范、数据版本和校验值 | 1～3 周 |
| P2 训练与评估 | 固定环境、权重、seed、imgsz、epochs、batch 和增强；对独立主体报告像素 / 三维误差、漏检、错接管及延迟，与人工基线比较 | 3～7 天 |
| P3 三维适配 | 模型常驻，绑定源 RGB-D 与时间；反投影、稳健法向、纹理 / 质量 / 时效门；输出可回放的 v1 观测 | 1～2 周 |
| P4 会话联调 | 离线 → 隔离 Redis 仿真 → 实机只读 → 悬停 → 低速跟随；记录重启、断流、遮挡、恢复与停止结果 | 1～2 周以上 |

估算以设备、人手和算力可用为前提，不承诺上线日期。原建议测试门为像素误差 P95≤8 px、
错选可执行目标 / 无目标误触发均≤0.5%，必须按实际误差预算确认，报告分母 / 置信区间；样本不足标待验证。

原训练示例为 yolo11n-pose；需按安装版本核对任务和许可。模型只输出二维候选，RGB-D 求三维点 / 法向，
Robot 插值 `T_base_armtip(t)×T_armtip_camera`，生成 `surface+0.050×normal` 悬停目标和连续姿态。
不能伪造 feature_inliers 或用旧检测帧冒充新观测。独立记录法向误差、P95 / P99 延迟、间距、丢失恢复和错误目标接管。
早期“人工确认首次开始”已更新；当前自动开始和恢复仍须通过会话、质量、目标回执和 Robot 规划门。

## 参考资料

- [Ultralytics Pose](https://docs.ultralytics.com/tasks/pose/)、[数据格式](https://docs.ultralytics.com/datasets/pose/)、[训练](https://docs.ultralytics.com/zh/modes/train/)、[Python 推理 / 导出](https://docs.ultralytics.com/zh/usage/python/)。
- [Roboflow 文档](https://docs.roboflow.com/)、[数据集版本](https://docs.roboflow.com/datasets/versions/dataset-versions)、[创建版本](https://docs.roboflow.com/datasets/versions/dataset-versions/create-a-dataset-version)。
- 仓库依据：[click_follow.py](../../infer/Camera_wrist/click_follow.py)、[LocalTracker](../../infer/Camera_wrist/local_tracker.py)、[模型进展](carotid-begin-progress.md)。
