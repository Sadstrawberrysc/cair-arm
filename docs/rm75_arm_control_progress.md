# Realman RM75 机械臂控制进度记录

## 当前目标

- 只保留 RM75 机械臂控制主线。
- 以 `main_rm75` 作为主程序。
- 主程序源码为 `infer/Robot/src/arm_servoj_line_test.cpp`。
- 控制方式为固定周期 streaming ServoJ。

## 当前保留入口

- `main_rm75`：主控制程序，支持相对位置运动和绝对 6D 位姿控制。
- `arm_read_state`：读取七轴关节、末端位置和末端姿态。
- `arm_preset_pose`：按七轴关节角移动到安全姿态，用于脱离奇异点或回到准备位。

## 已删除/移除的测试入口

- `arm_servoj_delta`：单关节 ServoJ 小角度测试入口，已完成验证后移除。
- `arm_servoj_pose`：临时 6D ServoJ 入口，已并入 `main_rm75`。
- `arm_jacobian_6d_pose.cpp`：旧 6D 测试主程序，已被 `arm_servoj_line_test.cpp` 取代。
- `build_arm_servoj_delta.sh`、`build_arm_servoj_pose.sh`、`build_arm_servoj_line_test.sh`：临时构建脚本已移除，统一使用 `build_main_rm75.sh`。

## 主程序能力

`main_rm75` 当前支持两类控制：

1. 相对位置运动，姿态保持当前值：

```bash
./main_rm75 \
  --delta-cm "0,0,20" \
  --execute
```

2. 绝对 6D 位姿控制，同时给 position 和 rotation：

```bash
./main_rm75 \
  --target-position-cm "40,20,45" \
  --target-rotation-deg "96,71,97" \
  --execute
```

也支持只给位置或只给姿态：

```bash
./main_rm75 --target-position-cm "40,20,45" --execute
./main_rm75 --target-rotation-deg "96,71,97" --execute
```

未给出的目标字段默认使用当前位姿。

## 当前默认参数

- 控制器地址：`192.168.50.254:8080`
- ServoJ 周期：`period_ms=20`
- 反馈读取：`feedback_every=10`
- 终点追踪：`final_hold_ms=2000`
- 末端主轨迹速度上限：`max_tcp_speed_cm_s=5`
- 最大总位置变化：`max_total_delta_cm=100`
- 最大姿态变化：`max_rotation_delta_deg=180`
- 关节速度限幅：使用 RM75 官方最大关节角速度
  - J1/J2：`180 deg/s`
  - J3-J7：`225 deg/s`
- 位姿插值：默认 `linear`
- S 曲线插值：可通过 `--interpolation s-curve` 显式启用
- 关节加速度限幅：默认关闭，`max_joint_accel_deg_s2=0`
- 关节加速度限幅可通过 `--max-joint-accel-deg-s2` 显式启用
- 关节限位预警：`joint_limit_warning_deg=10`
- 关节限位停止：`joint_limit_stop_deg=3`
- 奇异点预警：`singularity_warning_deg=5`

## 技术路径总结

1. 先完成 Realman TCP JSON 通信。
2. 将原 6 轴接口适配为 RM75 七轴接口。
3. 验证七轴 `MoveJ()` 基础运动。
4. 使用 RM75 官方 MDH 参数建立 FK 和 `6x7` Jacobian。
5. 用真机数据验证 FK 与控制器末端位置一致。
6. 用阻尼伪逆 Jacobian 实现数值 IK。
7. 先完成位置闭环，再完成姿态闭环，最后合成 6D 位姿控制。
8. 将控制方式从分段 ServoJ 改为固定周期 streaming ServoJ。
9. 在主程序中加入：
   - 终点 hold 追踪
   - 反馈校正
   - 关节限位保护
   - RM75 官方奇异点保护
   - 位置和姿态 S 曲线插值
   - 关节加速度限制

## 当前结论

- `main_rm75` 已经可以作为当前 RM75 控制主入口。
- 真机反馈显示首次 S 曲线版本反而更晃，原因判断为：
  - 五次 S 曲线中段峰值速度是平均速度的 `1.875` 倍，原估时没有计入该峰值，导致中段实际速度偏激进。
  - 反馈校正后清零上一周期关节增量，会在反馈点制造重复起步感。
- 已修正并回退默认策略：
  - 默认恢复到此前较稳定的线性插值。
  - 默认关闭额外关节加速度限幅，避免外层限幅与控制器内部 ServoJ 滤波耦合振荡。
  - 保留 S 曲线插值作为可选调参项：`--interpolation s-curve`。
  - 保留关节加速度限幅作为可选调参项：`--max-joint-accel-deg-s2 N`。
  - 估时按 S 曲线峰值速度重新计算，保证峰值 TCP 速度不超过 `max_tcp_speed_cm_s`。
  - 反馈校正后保留上一周期关节增量历史，避免周期性速度突变。
- 相对位置 streaming ServoJ 已在真机上取得较好结果：
  - 终点误差可到约 `0.0036 cm`
  - 横向误差约 `0.07 cm`
  - 周期丢失通常为 `0-2`
- 旧 `arm_jacobian_6d_pose.cpp` 直接追目标误差时，大位姿目标误差较大；因此当前主程序改用 `arm_servoj_line_test.cpp` 的连续轨迹 + 终点追踪逻辑。

## 常用命令

重新编译主程序：

```bash
cd /home/cair-jacen/uspilot_ctrl-main
infer/Robot/tools/build_main_rm75.sh
cmake -S infer/Robot -B infer/Robot/build
cmake --build infer/Robot/build --target main_rm75 -j2
```

读取当前状态：

```bash
cd /home/cair-jacen/uspilot_ctrl-main/infer/Robot/build
./arm_read_state --repeat 1
```

移动到安全关节姿态：

```bash
./arm_preset_pose \
  --target-deg "10,-20,30,80,-50,50,70" \
  --allow-multistep \
  --max-joint-delta-deg 15 \
  --execute
```

主程序 dry-run：

```bash
./main_rm75 \
  --target-position-cm "40,20,45" \
  --target-rotation-deg "96,71,97"
```

确认安全后执行：

```bash
./main_rm75 \
  --target-position-cm "40,20,45" \
  --target-rotation-deg "96,71,97" \
  --execute
```

## 风险/注意事项

- 大范围目标仍可能触发关节限位、奇异点或不可达问题。
- 如果出现 `Target joint Jx exceeds RM75 joint limits`，不要强行放开，应换姿态或拆小目标。
- 如果出现 `rm75_singularity_warning`，说明当前或规划目标靠近 RM75 官方奇异位形。
- `final_hold_ms` 会增加总运动时间，但能显著降低终点误差。
- 所有大位移或大姿态变化都应先 dry-run，再 `--execute`。
