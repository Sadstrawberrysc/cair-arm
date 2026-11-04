# py-xiaokai Voice and UI Client

[简体中文](README.md)

This directory is based on [py-xiaozhi](https://github.com/huangjunsen0406/py-xiaozhi), a Python client
ported from [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32). It retains voice, GUI / CLI,
wake words, WebSocket / MQTT, MCP and IoT extensions, with USPilot ultrasound integration.
See the [upstream documentation](https://huangjunsen0406.github.io/py-xiaozhi/) for general setup.

## Repository integration

| Entry | Purpose |
| --- | --- |
| [main.py](main.py) | CLI options, initialization, activation and application lifecycle |
| [src/application.py](src/application.py) | Voice / UI orchestration and ultrasound Redis client |
| [src/redis_client.py](src/redis_client.py) | `sonoscape:cmd` requests and result polling on Redis 6379 |
| [src/roboredis_arm.py](src/roboredis_arm.py) | `device:arm:*` interfaces; the current RM75 production service has no consumer |
| [src/audio_codecs/](src/audio_codecs/), [audio_processing/](src/audio_processing/) | Audio codecs, AEC, VAD and wake words |
| [src/protocols/](src/protocols/), [mcp/](src/mcp/), [iot/](src/iot/) | Service protocols, tools and devices |
| [src/views/](src/views/), [display/](src/display/), [utils/](src/utils/) | UI, display, configuration and logging |

Ultrasound commands flow through Redis 6379 to the [SonoScape service](../SonoScape_api/README.md).
Despite its name, `UltrasoundRedisMockClient` sends real Redis requests. RM75 motion uses Redis 7777
and `robot:command:channel`; see the [system architecture](../ARCHITECTURE.md).

## Run

Use a separate Python environment with [requirements.txt](requirements.txt), audio devices and required services.
Upstream lists Python 3.9–3.12; validate the actual dependency set locally and keep it separate from camera / Robot environments.

```bash
cd py-xiaokai
pip install -r requirements.txt
python main.py                         # GUI / WebSocket by default
python main.py --mode cli
python main.py --mode chatgui
python main.py --protocol mqtt
```

`--skip-activation` is for debugging. Wake words require local Sherpa-ONNX models; camera tools need
OpenCV and a camera; online services require network access. Successful startup or publication does not verify device control.

## Development and upstream resources

Use asyncio and ConfigManager. Add MCP tools under `src/mcp/tools/`, extend Thing for IoT devices,
and use the protocol / display abstractions. Run `./format_code.sh`, relevant checks, and update affected documentation.

- [Docs](https://huangjunsen0406.github.io/py-xiaozhi/), [releases](https://github.com/huangjunsen0406/py-xiaozhi/releases/latest), [Gitee](https://gitee.com/huang-jun-sen/py-xiaozhi)
- [Demo](https://www.bilibili.com/video/BV1HmPjeSED2/), [tutorial](https://www.bilibili.com/video/BV1dWQhYEEmq/), [sponsors](https://huangjunsen0406.github.io/py-xiaozhi/sponsors/)
- [MIT License](LICENSE)

## Upstream contributors

Original acknowledgments, in no particular order:

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
