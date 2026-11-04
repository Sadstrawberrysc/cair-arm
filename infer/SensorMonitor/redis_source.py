"""Read Robot sensor telemetry in SI units without opening its serial device."""

import json
import math
import time

CHANNEL = "robot:sensor:v1"


def decode_sample(payload, now_ms=None):
    """Return raw Sensor-frame wrench and identity; reject invalid/stale telemetry."""
    message = json.loads(payload)
    if not isinstance(message, dict) or type(message.get("version")) is not int or message["version"] != 1:
        raise ValueError("不是 sensor v1 遥测")
    if message.get("units") != {"force": "N", "torque": "N*m", "length": "m"}:
        raise ValueError("遥测单位不匹配")
    published = message.get("published_unix_ms")
    now_ms = time.time_ns() // 1_000_000 if now_ms is None else now_ms
    if type(published) is not int or not -1000 <= now_ms - published <= 1000:
        raise ValueError("遥测发布时间无效或已过期")
    sensor = message.get("sensor")
    if not isinstance(sensor, dict):
        raise ValueError("缺少传感器状态")
    if (message.get("valid") is not True or sensor.get("stale") is not False
            or sensor.get("io_status") != "streaming" or sensor.get("io_error") != 0):
        raise ValueError("Robot 力数据无效、陈旧或采集未就绪")
    available = sensor.get("checksum_available", True)
    if type(available) is not bool or (available and sensor.get("checksum_valid") is not True):
        raise ValueError("传感器校验未通过")
    wrench = message.get("raw_wrench_sensor")
    if not isinstance(wrench, dict):
        raise ValueError("缺少 Sensor 原始六轴数据")
    force, torque = wrench.get("force_n"), wrench.get("torque_nm")
    if (not isinstance(force, list) or not isinstance(torque, list)
            or len(force) != 3 or len(torque) != 3):
        raise ValueError("六轴数据形状不匹配")
    values = force + torque
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError("六轴数据包含无效数值")
    sequence, timestamp = message.get("sequence"), message.get("timestamp_monotonic_ns")
    if type(sequence) is not int or sequence <= 0 or type(timestamp) is not int or timestamp <= 0:
        raise ValueError("采样序号或时间无效")
    return tuple(values), sequence, timestamp


class RedisSource:
    """One subscription per session; disconnects stop acquisition, never switch to serial."""

    def __init__(self, host, port):
        import redis

        self.client = redis.Redis(host=host, port=port, socket_connect_timeout=0.5,
                                  socket_timeout=0.5)
        self.subscription = self.client.pubsub(ignore_subscribe_messages=True)
        try:
            self.subscription.subscribe(CHANNEL)
        except Exception:
            self.close()
            raise

    def read(self, timeout=0.1):
        message = self.subscription.get_message(timeout=timeout)
        return message["data"] if message and message["type"] == "message" else None

    def close(self):
        try:
            self.subscription.close()
        finally:
            self.client.close()
