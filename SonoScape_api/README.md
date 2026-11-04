# SonoScape 超声设备接口

[UltrasoundClient](sonoscape_client/client.py) 通过 TCP 控制超声主机；
[redis_service.py](redis_service.py) 将 UI 的 Redis 请求转为设备调用。
该接口独立于 RM75 的 `robot:command:channel` 运动闭环。

## 入口与连接

| 文件 / 接口 | 用途 |
| --- | --- |
| `sonoscape_client/` | 真实设备客户端、协议常量与异常 |
| `sonoscape_fake/`、`main_fake.py` | 本地模拟 / 演示 |
| `redis_service.py` | Redis List `sonoscape:cmd` → 设备调用 → 结果键 |
| `redis_client_demo.py` | Redis 请求示例 |
| `main.py`、`read_status.py` | 设备演示 / 状态读取；运行前核对脚本地址与实际调用 |

客户端构造为 `UltrasoundClient(host, port=6061, timeout=5)`，显式 `connect()` / `disconnect()`，
或使用 `with UltrasoundClient(...) as client` 自动连接和关闭。服务脚本当前配置
`192.168.1.100:6061`，Redis 默认 `localhost:6379`，运行前按实际设备调整。

## Redis 请求

生产者向 `sonoscape:cmd` List 推入 JSON，例如：

```json
{"id":"example-request","command":"set_scan_depth(depth_mm=35)"}
```

服务按 id 写 `sonoscape:result:<id>`；成功结果包含 `status=success` 和 data，成功路径 TTL 为 60 秒，
异常返回 `status=error` / error，当前异常路径未设置 TTL。UI 客户端等待结果后删除键。
服务启动会清空旧命令队列；它执行设备操作，不是只读监视器。

## 查询方法

| 方法 | 返回 / 单位 |
| --- | --- |
| `get_status()` | 状态字典：模式、探头、冻结 / B 模式、fps、增益、功率、频率、深度和图像位置 / 尺寸 |
| `get_disk_space()` | 剩余空间整数，GB |
| `get_patient_name()` / `get_patient_id()` | 姓名 / ID 字符串，无值为空字符串 |
| `get_patient_scan_time()` | 扫描时间，`datetime` |
| `get_dicom_send_status(patient_id)` | `exists`、成功 / 失败计数；存在时含 `id_returned`，ID 长度最多 22 字节 |
| `get_zoom_factor()` | 浮点放大倍数 |

`get_status()` 的键包括 `scan_mode`、`probe_active`、`is_b_mode`、`is_frozen`、`is_new_patient_ui`、
`fps`、`d_g`、`gain_gn`、`i_p`、`power_pwr`、`frequency_frq`、`depth_d_mm`、`micro_imaging_us`、
`image_start_x_percent`、`image_start_y_percent`、`image_width_px`、`image_height_px`。

## 控制方法

通常成功返回 `True`，另有明确返回值的方法见表。

| 方法 | 参数 / 行为 |
| --- | --- |
| `control_s_marker(show)` / `control_ruler(show)` | bool，显示 / 隐藏标记或标尺 |
| `set_scan_mode(mode)` | `ScanMode.FUNDAMENTAL` / `HARMONIC` |
| `set_scan_depth(depth_mm)` | 毫米，例如 35；内部转整数，旧 `depth_cm` 参数已不适用 |
| `set_image_type(image_type)` | `ImageType.POST_PROCESSED` / `PRE_PROCESSED` |
| `start_image_acquisition(params)` | 协议采集参数，如 LT、RT、LH、RH |
| `abort_image_acquisition()` | 丢弃当前采集 |
| `end_image_acquisition()` | 结束采集并冻结 |
| `control_freeze(freeze)` | True 冻结，False 解冻 |
| `save_and_send_image()` | 保存 / DICOM 发送，返回 `success` 与 `dicom_servers` |
| `new_patient()` | 打开新患者界面 |
| `set_b_gain(gain)` | 整数 0～999 |
| `robot_start(bodypart)` | 向 Redis 7777 的旧 `robot_control` 发布请求；当前 RM75 主链路不消费此通道，返回成功不代表运动 |

## 错误与验证边界

设备拒绝返回 `UltrasoundCommandError`，帧头 / 尾 / checksum 等失败返回 `UltrasoundProtocolError`，
建连 / 握手失败返回 `UltrasoundConnectionError`；参数校验可抛 `ValueError`，收发仍可能抛原始 socket 异常。
实际帧实现见 [protocol.py](sonoscape_client/protocol.py)。本说明核对源码，不代表本次已连接设备验收。
