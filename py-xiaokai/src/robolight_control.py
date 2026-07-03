import serial
import threading
import time

class LightController:
    def __init__(self, port="/dev/ttyUSB1", baudrate=9600, timeout=3):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.last_status = None
        self._lock = threading.Lock()

        # 对齐 UI 所需的可读属性（与 UI 字段名一致）
        self.power_on = False          # True/False
        self.spot_size = 2             # 1..3
        self.color_temp = 3            # 1..5
        self.illumination = 5          # 1..10
        self.mode = "INIT"             # ENDO/R9/INIT/DEPTH

        # 兼容旧的“current_”缓存（内部控制用）
        self.current_mode = "INIT"
        self.current_color_temp = 3
        self.current_illumination = 5

        # 启动即初始化目标状态（同步执行）
        # 如果担心阻塞，可改为开启线程异步执行
        try:
            self._init_default_state()
        except Exception as e:
            print(f"Init sequence failed: {e}")

    # ---------------- 内部：初始化序列 ----------------
    def _init_default_state(self):
        # 按需求：关、模式INIT、光斑2、色温3(4000K)、照度5
        steps = [
            ("turn_off", lambda: self.turn_off()),
            ("set_mode_INIT", lambda: self.set_mode("INIT")),
            ("set_spot_2", lambda: self.set_spot_size(2)),
            ("set_ct_3", lambda: self.set_color_temp(3)),
            ("set_illum_5", lambda: self.set_illumination(5)),
        ]
        for name, fn in steps:
            ok, info = self._safe_apply(name, fn)
            # 可以根据需要决定是否在失败时中断
            # if not ok: break
            time.sleep(0.08)  # 适当间隔，避免设备处理不过来

    def _safe_apply(self, name, fn):
        try:
            ok, info = fn()
            print(f"[init] {name}: ok={ok}, info={info}")
            return ok, info
        except Exception as e:
            print(f"[init] {name}: exception={e}")
            return False, {"ok": False, "error": str(e)}

    # ---------------- 下行命令与解析 ----------------
    def _send_cmd(self, cmd_type: int, cmd_param: int, wait_reply=True):
        data = [0xFA, 0x51, cmd_type, cmd_param] + [0x00] * 11 + [0xAF]
        pkt = bytes(data)
        print(f"Sending Command: {[hex(x) for x in data]}")

        try:
            with self._lock:
                self.ser.write(pkt)
                print(f"Command Sent Successfully: {pkt.hex()}")
        except Exception as e:
            print(f"Serial Write Exception: {e}")
            return False, {"ok": False, "action": f"send(type={cmd_type:#04x},param={cmd_param})", "msg": f"serial write error: {e}"}

        if not wait_reply:
            return True, {"ok": True, "action": f"send(type={cmd_type:#04x},param={cmd_param})", "msg": "sent"}

        try:
            with self._lock:
                buf = b""
                for _ in range(4):
                    chunk = self.ser.read(16 - len(buf))
                    if chunk:
                        buf += chunk
                    if len(buf) >= 16:
                        break
                    if len(chunk) == 0:
                        break
            if len(buf) < 16:
                print(f"No/short reply (len={len(buf)}): {buf.hex() if buf else 'None'}")
                return True, {"ok": True, "action": "recv", "msg": "no reply (acceptable)"}

            print(f"Received Reply: {buf.hex()}")
            status = self._parse_reply(buf)
            self.last_status = status
            if isinstance(status, dict) and status.get("error"):
                print(f"Reply parse error: {status['error']}")
                return True, {"ok": False, "action": "parse_reply", "msg": status.get("error", "parse error"), "raw": status}
            return True, {"ok": True, "action": "parse_reply", "msg": "ok", "raw": status}
        except Exception as e:
            print(f"Serial Read/Parse Exception: {e}")
            return True, {"ok": False, "action": "read/parse", "msg": str(e)}

    # === Control Commands ===
    def turn_on(self):
        print("Turning on the light")
        ok, info = self._send_cmd(cmd_type=0x01, cmd_param=0x01)
        if ok:
            self.power_on = True
            info = info or {}
            info.update({"operation": "turn_on"})
        return ok, info

    def turn_off(self):
        print("Turning off the light")
        ok, info = self._send_cmd(cmd_type=0x01, cmd_param=0x02)
        if ok:
            self.power_on = False
            info = info or {}
            info.update({"operation": "turn_off"})
        return ok, info

    def set_spot_size(self, level: int):
        assert 1 <= level <= 3
        if self.current_mode.upper() in ("ENDO", "R9", "DEPTH"):
            msg = f"blocked: mode={self.current_mode} prohibits spot adjust"
            print(msg)
            return False, {"ok": False, "operation": "set_spot", "msg": msg}
        ok, info = self._send_cmd(cmd_type=0x02, cmd_param=level)
        if ok:
            self.spot_size = level
            info = info or {}
            info.update({"operation": "set_spot", "level": level})
        return ok, info

    def set_color_temp(self, level: int):
        assert 1 <= level <= 5
        if self.current_mode.upper() in ("ENDO", "R9"):
            msg = f"blocked: mode={self.current_mode} prohibits color temp adjust"
            print(msg)
            return False, {"ok": False, "operation": "set_color_temp", "msg": msg}
        ok, info = self._send_cmd(cmd_type=0x03, cmd_param=level)
        if ok:
            self.current_color_temp = level
            self.color_temp = level
            info = info or {}
            info.update({"operation": "set_color_temp", "level": level})
        return ok, info

    def increase_color_temp(self):
        lvl = getattr(self, 'current_color_temp', 3)
        return self.set_color_temp(min(5, lvl + 1))

    def decrease_color_temp(self):
        lvl = getattr(self, 'current_color_temp', 3)
        return self.set_color_temp(max(1, lvl - 1))

    def set_illumination(self, level: int):
        assert 1 <= level <= 10
        if self.current_mode.upper() in ("ENDO",):
            msg = f"blocked: mode={self.current_mode} prohibits illumination adjust"
            print(msg)
            return False, {"ok": False, "operation": "set_illumination", "msg": msg}
        ok, info = self._send_cmd(cmd_type=0x04, cmd_param=level)
        if ok:
            self.current_illumination = level
            self.illumination = level
            info = info or {}
            info.update({"operation": "set_illumination", "level": level})
        return ok, info

    def increase_illumination(self):
        lvl = getattr(self, 'current_illumination', 5)
        return self.set_illumination(min(10, lvl + 1))

    def decrease_illumination(self):
        lvl = getattr(self, 'current_illumination', 5)
        return self.set_illumination(max(1, lvl - 1))

    def set_mode(self, mode: str):
        mode_map = {
            "ENDO": 0x01,
            "R9": 0x02,
            "INIT": 0x03,
            "DEPTH": 0x06,
        }
        mode = mode.upper()
        code = mode_map.get(mode)
        if code is None:
            raise ValueError("不支持的模式！支持的模式: ENDO, R9, INIT, DEPTH")
        ok, info = self._send_cmd(cmd_type=0x05, cmd_param=code)
        if ok:
            self.current_mode = mode
            self.mode = mode
            info = info or {}
            info.update({"operation": "set_mode", "mode": mode})
        return ok, info

    def _parse_reply(self, reply: bytes):
        if len(reply) != 16:
            return {"error": "Data length error", "raw": reply.hex()}
        if reply[0] != 0xFA or reply[-1] != 0xAF:
            return {"error": "Header/Footer mismatch", "raw": reply.hex()}

        dev = reply[1]
        body = reply[1:-1].hex()

        info = {
            "check": "OK",
            "raw": reply.hex(),
            "body": body
        }

        if dev != 0x51:
            info["warn"] = f"unexpected dev byte: {dev:#04x}"

        # 可选：基于你的回传格式解析5个关键位（如已确认字节位置，可取消下面注释）
        try:
            pwr = reply[3]        # 0x01 开 / 0x02 关
            spot = reply[4]       # 1..3
            ct = reply[5]         # 1..5
            illum = reply[6]      # 1..10
            mode_byte = reply[7]  # 0x01 ENDO 0x02 R9 0x03 INIT 0x06 DEPTH
            mode_rev = {0x01: "ENDO", 0x02: "R9", 0x03: "INIT", 0x06: "DEPTH"}
            self.power_on = (pwr == 0x01)
            self.spot_size = int(spot)
            self.color_temp = int(ct)
            self.current_color_temp = int(ct)
            self.illumination = int(illum)
            self.current_illumination = int(illum)
            self.mode = mode_rev.get(mode_byte, self.mode)
            self.current_mode = self.mode
            info["parsed"] = {
                "power": "ON" if self.power_on else "OFF",
                "spot": self.spot_size,
                "color_temp_level": self.color_temp,
                "illumination": self.illumination,
                "mode": self.mode,
            }
        except Exception as _:
            pass

        return info

    def query_status(self):
        print("Querying current status")
        return self.last_status

    def close(self):
        with self._lock:
            self.ser.close()