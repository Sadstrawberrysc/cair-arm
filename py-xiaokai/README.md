# py-xiaokai 语音与 UI 客户端

[English](README.en.md)

本目录基于 [py-xiaozhi](https://github.com/huangjunsen0406/py-xiaozhi)，其语音客户端源自
[xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)。保留 GUI / CLI、音频、唤醒、WebSocket / MQTT、MCP 和 IoT 扩展，
并接入 USPilot 超声设备交互。上游通用功能和安装说明见[项目文档](https://huangjunsen0406.github.io/py-xiaozhi/)。

## 本仓库连接

| 入口 | 职责 |
| --- | --- |
| [main.py](main.py) | 参数解析、初始化、激活与应用生命周期 |
| [src/application.py](src/application.py) | 语音 / UI 编排与超声 Redis 客户端 |
| [src/redis_client.py](src/redis_client.py) | Redis 6379 的 `sonoscape:cmd` 请求与结果等待 |
| [src/roboredis_arm.py](src/roboredis_arm.py) | `device:arm:*` 接口；当前 RM75 生产主链路没有对应消费者 |
| [src/audio_codecs/](src/audio_codecs/)、[audio_processing/](src/audio_processing/) | 音频编解码、AEC、VAD 与唤醒 |
| [src/protocols/](src/protocols/)、[mcp/](src/mcp/)、[iot/](src/iot/) | 语音服务协议、工具与设备扩展 |
| [src/views/](src/views/)、[display/](src/display/)、[utils/](src/utils/) | UI、显示与配置 / 日志 |

超声请求链路为本客户端 → Redis 6379 → [SonoScape 服务](../SonoScape_api/README.md) → 超声主机。
`UltrasoundRedisMockClient` 类名虽含 Mock，实际会向 Redis 发请求；不能据此视为离线模拟。
RM75 超声运动主链路使用 Redis 7777 的 `robot:command:channel`，见 [系统架构](../ARCHITECTURE.md)。

## 运行

在本目录、独立 Python 环境中安装 [requirements.txt](requirements.txt)，使用麦克风 / 扬声器和所需服务。
上游标注 Python 3.9～3.12；本机兼容性按实际依赖验证，不与 Robot / 相机环境混装。

```bash
cd py-xiaokai
pip install -r requirements.txt
python main.py                         # 默认 GUI / WebSocket
python main.py --mode cli
python main.py --mode chatgui
python main.py --protocol mqtt
```

`--skip-activation` 仅用于调试。唤醒需本地 Sherpa-ONNX 模型，摄像头功能需设备 / OpenCV，在线语音和工具需对应网络服务。
GUI、依赖安装或 Redis 发布成功均不代表设备控制 / 机器人运动验收。

## 开发与上游资源

采用 asyncio；配置集中由 ConfigManager 管理。新增 MCP 工具放 `src/mcp/tools/`，IoT 设备继承 Thing，
协议和显示扩展沿用对应抽象。格式化使用 `./format_code.sh`；修改后运行相关检查并更新文档。

- [上游文档](https://huangjunsen0406.github.io/py-xiaozhi/)、[发布](https://github.com/huangjunsen0406/py-xiaozhi/releases/latest)、[Gitee](https://gitee.com/huang-jun-sen/py-xiaozhi)
- [演示](https://www.bilibili.com/video/BV1HmPjeSED2/)、[入门视频](https://www.bilibili.com/video/BV1dWQhYEEmq/)、[赞助者与支持](https://huangjunsen0406.github.io/py-xiaozhi/sponsors/)
- [MIT License](LICENSE)

## 上游贡献者

保留原项目鸣谢，排名不分先后：

[Xiaoxia](https://github.com/78)
[zhh827](https://github.com/zhh827)
[四博智联-李洪刚](https://github.com/SmartArduino)
[HonestQiao](https://github.com/HonestQiao)
[vonweller](https://github.com/vonweller)
[孙卫公](https://space.bilibili.com/416954647)
[isamu2025](https://github.com/isamu2025)
[Rain120](https://github.com/Rain120)
[kejily](https://github.com/kejily)
[电波bilibili君](https://space.bilibili.com/119751)
[赛搏智能](https://shop115087494.m.taobao.com/?refer=https%3A%2F%2Fm.tb.cn%2F&ut_sk=1.WMelxbgDQWkDAJ1Rq9Pn7DCD_21380790_1757337352472.Copy.shop&suid=0E25E948-651D-46E0-8E89-5C8CB03B4F56&shop_navi=shopindex&sourceType=shop&shareUniqueId=33038752403&un=d22c5ceda82844ab8bd7bab98ffeb263&share_crt_v=1&un_site=0&spm=a2159r.13376460.0.0&sp_tk=dkRKUjRKUWo2ZHY%3D&bc_fl_src=share-1041250486811064-2-1&cpp=1&shareurl=true&short_name=h.SaBKVHytsCKIPNS&bxsign=scdGtSe264e_qkFQBh0rXCkF-Mrb_s6t35EnpVBBU5dsrd-J24c-_rn_PhJiXRk0hg2hjGoAm0L7j2UQg27OIH_6gZkbhKDyLziD2cy4pDf8sC3KmqrF55TXP3USZaPTw_-&app=weixin)
