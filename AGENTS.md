# USPilot Control

在线超声推理通过 Redis `127.0.0.1:7777` 向 RM75 发送控制意图。设备控制使用另一实例 `127.0.0.1:6379`。模块入口、协议和坐标约定见[系统架构](ARCHITECTURE.md)。

## 真机操作

- 无参数 `main_rm75`、腕部 `start_robot.sh` 和一键扫描都会驱动真机。按[系统运行手册](.runme/run/system-operations.md)逐个运行单元；禁止 Run All。
- Collision 监测默认只读，一键扫描的 `--collision` 模式允许按 E 执行。执行前使用固定快照检查路径，运动中新增或移动的障碍物不会触发相机避障停止。真机验收待完成，见[执行说明](infer/Collision/EXECUTION.md)。

## 环境与验证入口

在仓库根目录构建 Robot 并运行离线测试。涉及 `arm_probe_pose` 或标定工具时，将 `BUILD_MAINTENANCE_TOOLS` 设为 `ON` 以启用相应目标和测试。

```bash
cmake -S infer/Robot -B infer/Robot/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_MAINTENANCE_TOOLS=OFF
cmake --build infer/Robot/build --target main_rm75 robot_offline_tests
ctest --test-dir infer/Robot/build --output-on-failure
```

## 仓库维护

- 修改前后检查 `git status --short`，保留已有未提交改动。提交时排除构建产物、运行日志、权重和本机临时文件。
- 接口或启动方式变化时，直接替换对应文档中的旧说明。操作和进展见 [Runme 索引](.runme/README.md)。跨模块接口和长期设计取舍的原因按日期追加到 [DECISIONS.md](DECISIONS.md)。
- 默认用中文交付，分别说明实际执行的离线验证和真机验收情况。
