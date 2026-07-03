#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import json
import re
import ast
import requests
from datetime import datetime
from typing import Any, Dict, Union, Tuple, List

# ========== 可选硬件控制器导入：失败则回退到 Dummy ==========
RealLightController = None
try:
    from light_control import LightController as RealLightController  # 可能不存在
except Exception:
    RealLightController = None

# 机械臂 Redis 发布器
ArmCommandPublisher = None
try:
    from redis_arm import ArmCommandPublisher  # 可能不存在或连接失败
except Exception:
    ArmCommandPublisher = None

class DummyLightController:
    """
    用于离线/无硬件调试的假控制器。
    提供与 LightController 相同的方法签名，返回模拟结果，不访问串口。
    """
    def __init__(self, port: str = None):
        self.port = port or "DUMMY"
        self.power = False
        self.power_on = False  # 与 UI Dummy 对齐的兼容字段
        self.illumination = 5         # 1..10
        self.color_temp = 3           # 1..5 (3500..4500K 映射)
        self.spot_size = 2            # 1..3
        self.mode = "INIT"            # INIT/ENDO/R9/DEPTH
        self.current_spot = self.spot_size

    # 开关
    def turn_on(self):
        self.power = True
        self.power_on = True
        return True, {"mock": True, "power": True, "msg": "turned on"}

    def turn_off(self):
        self.power = False
        self.power_on = False
        return True, {"mock": True, "power": False, "msg": "turned off"}

    # 亮度 1..10
    def set_illumination(self, level: int):
        if not 1 <= level <= 10:
            return False, {"error": "illumination range 1..10"}
        self.illumination = level
        return True, {"mock": True, "illumination": self.illumination}

    def increase_illumination(self):
        if self.illumination >= 10:
            return False, {"error": "already max"}
        self.illumination += 1
        return True, {"mock": True, "illumination": self.illumination}

    def decrease_illumination(self):
        if self.illumination <= 1:
            return False, {"error": "already min"}
        self.illumination -= 1
        return True, {"mock": True, "illumination": self.illumination}

    # 色温档位 1..5
    def set_color_temp(self, level: int):
        if not 1 <= level <= 5:
            return False, {"error": "color_temp range 1..5"}
        self.color_temp = level
        return True, {"mock": True, "color_temp": self.color_temp}

    def increase_color_temp(self):
        if self.color_temp >= 5:
            return False, {"error": "already max"}
        self.color_temp += 1
        return True, {"mock": True, "color_temp": self.color_temp}

    def decrease_color_temp(self):
        if self.color_temp <= 1:
            return False, {"error": "already min"}
        self.color_temp -= 1
        return True, {"mock": True, "color_temp": self.color_temp}

    # 光斑 1..3
    def set_spot_size(self, level: int):
        if not 1 <= level <= 3:
            return False, {"error": "spot_size range 1..3"}
        self.spot_size = level
        self.current_spot = level
        return True, {"mock": True, "spot_size": self.spot_size}

    # 模式
    def set_mode(self, mode: str):
        mode = (mode or "").upper()
        if mode not in ("INIT", "ENDO", "R9", "DEPTH"):
            raise ValueError("invalid mode")
        self.mode = mode
        return True, {"mock": True, "mode": self.mode}

    def close(self):
        return True

# ========== 配置 ==========
OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "mirrorwanswer_ollama"
DEFAULT_OPTIONS = {"temperature": 0.0}



def find_json_like(text: str):
    """
    在文本中查找第一个 \json{...}（或 \\json{{...}} 等）并提取内部内容（含最外层大括号）。
    返回提取的字符串（包括最外层 { }），或 None。
    """
    m = re.search(r'\\+json', text)
    if not m:
        return None
    start = m.end()
    # 找第一个 '{' 作为开始
    brace_pos = text.find('{', start)
    if brace_pos == -1:
        return None

    # 从 brace_pos 开始做手动配对，找到匹配的右大括号位置
    i = brace_pos
    depth = 0
    while i < len(text):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return text[brace_pos:i+1]
        i += 1
    return None

def parse_json_like_string(s: str):
    """
    将类似 {'command_type': 'switch', ...} 或 {"command_type": "switch", ...} 转为 dict。
    优先尝试 json.loads，然后尝试 ast.literal_eval（兼容单引号）。
    返回 dict 或 None。
    """
    if s is None:
        return None
    # 清理外层重复大括号（例如 "{{...}}")
    while s.startswith("{{") and s.endswith("}}"):
        s = s[1:-1]

    # 尝试 json.loads
    try:
        return json.loads(s)
    except Exception:
        pass

    # 尝试 ast.literal_eval（可以解析单引号的字面量）
    try:
        return ast.literal_eval(s)
    except Exception:
        pass

    # 尝试移除末尾逗号并重试
    s_clean = re.sub(r",\s*}", "}", s)
    try:
        return json.loads(s_clean)
    except Exception:
        pass
    try:
        return ast.literal_eval(s_clean)
    except Exception:
        pass

    return None


# ========== 工具函数：模型通讯与输出解析 ==========
# ...existing code...
def parse_output_field(record: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
    """
    尝试将模型输出解析为 dict 并保证返回值为 dict：
    - 若 record 已经是 dict 则直接返回；
    - 使用 find_json_like + parse_json_like_string 优先解析 \json{...} 样式的内容；
    - 回退到 ast.literal_eval / json.loads 的尝试；
    - 最终回退为 {"raw": 原始字符串}，保证返回类型始终为 dict。
    """
    if isinstance(record, dict):
        return record

    output = str(record or "")

    # 优先使用 find_json_like + parse_json_like_string（支持多层大括号、单引号等）
    s = find_json_like(output)
    if s:
        parsed = parse_json_like_string(s)
        if isinstance(parsed, dict):
            return parsed

    # 若上面未能解析，尝试直接从 \json{...} 提取内部文本并用多种方式解析
    m = re.search(r'\\+json\{(.*)\}', output, flags=re.DOTALL)
    if m:
        inner = m.group(1).strip()
        try:
            parsed = ast.literal_eval(inner)
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            pass
        try:
            approx_json = re.sub(r"'", '"', inner)
            parsed = json.loads(approx_json)
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            pass

    # 最后尝试直接解析整个输出字符串（容错处理）
    parsed = parse_json_like_string(output)
    if isinstance(parsed, dict):
        return parsed

    # 保证返回 dict：将原始文本包裹返回
    return {"raw": output}
# ...existing code...

def call_ollama(prompt: str,
                model: str = MODEL_NAME,
                options: Dict[str, Any] = None,
                stream: bool = False,
                url: str = OLLAMA_URL) -> Dict[str, Any]:
    if options is None:
        options = DEFAULT_OPTIONS
    payload = {"model": model, "options": options, "prompt": prompt, "stream": stream}
    try:
        resp = requests.post(url, json=payload, timeout=120)
        resp.raise_for_status()
    except requests.RequestException as e:
        return {"success": False, "error": f"HTTP error: {e}"}
    try:
        data = resp.json()
    except ValueError:
        return {"success": False, "error": f"Invalid JSON response: {resp.text[:500]}"}
    response_text = data.get("response", "")
    return {"success": True, "raw": data, "text": response_text}

def format_parsed(parsed: Union[str, Dict[str, Any]]) -> Tuple[str, str]:
    if isinstance(parsed, dict):
        try:
            return "dict", json.dumps(parsed, ensure_ascii=False, indent=2, sort_keys=True)
        except Exception:
            return "str", str(parsed)
    return "str", str(parsed)

def print_separator():
    print("=" * 80)

# ========== 无影灯/机械臂参数与工具 ==========
def _kelvin_to_ct_level(k: int) -> int:
    mapping = {3500: 1, 3800: 2, 4000: 3, 4200: 4, 4500: 5}
    if k not in mapping:
        raise ValueError("色温仅支持 3500/3800/4000/4200/4500")
    return mapping[k]

def _facula_mode_to_level(mode: str) -> int:
    m = (mode or "").strip().lower()
    mapping = {"level_1": 1, "level_2": 2, "level_3": 3}
    if m not in mapping:
        raise ValueError("光斑模式仅支持 level_1/level_2/level_3")
    return mapping[m]

def normalize_bodypart_name(name: str) -> str:
    n = (name or "").strip().lower()
    arm_map = {
        "left_upper_arm": "left_arm",
        "left_lower_arm": "left_arm",
        "left_hand": "left_arm",
        "right_upper_arm": "right_arm",
        "right_lower_arm": "right_arm",
        "right_hand": "right_arm",
        "left_thigh": "left_leg",
        "left_foot": "left_leg",
        "right_thigh": "right_leg",
        "right_foot": "right_leg",
        "head": "head",
        "chest": "chest",
        "abdomen": "abdomen"
    }
    return arm_map.get(n, n)

def default_arm_angles() -> List[float]:
    return [0, -45, 0, -90, 0, 0]

def load_bodypart_presets(path: str = "init_part.json") -> Dict[str, List[float]]:
    presets: Dict[str, List[float]] = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            # 支持一行一个 JSON 或整个文件 JSON
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except Exception:
                    try:
                        f.seek(0)
                        obj_all = json.load(f)
                        if isinstance(obj_all, dict):
                            for k, v in obj_all.items():
                                presets[str(k).strip().lower()] = [float(x) for x in v]
                        elif isinstance(obj_all, list):
                            for it in obj_all:
                                part = (it.get("part") or "").strip().lower()
                                joint = it.get("joint")
                                if not part or joint is None:
                                    continue
                                if isinstance(joint, str):
                                    try:
                                        joint = json.loads(joint)
                                    except Exception:
                                        try:
                                            joint = json.loads(joint.replace("'", '"'))
                                        except Exception:
                                            continue
                                presets[part] = [float(x) for x in list(joint)]
                        return presets
                    except Exception:
                        continue
                part = (obj.get("part") or "").strip().lower()
                joint = obj.get("joint")
                if not part or joint is None:
                    continue
                if isinstance(joint, str):
                    try:
                        joint = json.loads(joint)
                    except Exception:
                        try:
                            joint = json.loads(joint.replace("'", '"'))
                        except Exception:
                            continue
                try:
                    joint = [float(x) for x in list(joint)]
                except Exception:
                    continue
                presets[part] = joint
    except FileNotFoundError:
        pass
    return presets

# ========== 指令到 控制器/机械臂 的映射与执行 ==========
def execute_command(cmd: Dict[str, Any], lc, arm_pub=None, presets: Dict[str, List[float]] = None) -> Dict[str, Any]:
    """
    合并 UI 的 execute_command 能力，但不依赖 lc._ui_owner：
    - switch/lamp_type/brightness/color_temp/facula/mode_switch 与原来一致
    - position/robot/bodypart 直接使用传入的 arm_pub 与 presets
    """
    if not isinstance(cmd, dict):
        return {"ok": False, "error": "解析结果不是 dict"}

    command_type = cmd.get("command_type")
    action = (cmd.get("action") or "").strip().lower()
    params = cmd.get("parameters") or {}

    try:
        # 开关
        if command_type == "switch":
            if action == "on":
                ok, info = lc.turn_on()
                return {"ok": ok, "operation": "turn_on", "detail": info}
            elif action == "off":
                ok, info = lc.turn_off()
                return {"ok": ok, "operation": "turn_off", "detail": info}
            else:
                return {"ok": False, "error": "switch.action 仅支持 on/off"}

        # 灯型（未实现）
        elif command_type == "lamp_type":
            return {"ok": False, "error": "lamp_type 暂未实现硬件映射"}

        # 亮度
        elif command_type == "brightness":
            if action == "set_level":
                level = int(params.get("level"))
                if not 1 <= level <= 10:
                    return {"ok": False, "error": "亮度 level 范围 1..10"}
                ok, info = lc.set_illumination(level)
                return {"ok": ok, "operation": "set_illumination", "detail": info}
            elif action == "increase":
                steps = int(params.get("steps", 1))
                if not 1 <= steps <= 9:
                    return {"ok": False, "error": "increase.steps 范围 1..9"}
                ok, info = True, {"steps": 0, "results": []}
                for _ in range(steps):
                    ok_step, info_step = lc.increase_illumination()
                    info["results"].append(info_step)
                    if not ok_step:
                        ok = False
                        break
                    info["steps"] += 1
                return {"ok": ok, "operation": "increase_illumination", "detail": info}
            elif action == "decrease":
                steps = int(params.get("steps", 1))
                if not 1 <= steps <= 9:
                    return {"ok": False, "error": "decrease.steps 范围 1..9"}
                ok, info = True, {"steps": 0, "results": []}
                for _ in range(steps):
                    ok_step, info_step = lc.decrease_illumination()
                    info["results"].append(info_step)
                    if not ok_step:
                        ok = False
                        break
                    info["steps"] += 1
                return {"ok": ok, "operation": "decrease_illumination", "detail": info}
            else:
                return {"ok": False, "error": "brightness.action 仅支持 set_level/increase/decrease"}

        # 色温
        elif command_type == "color_temp":
            if action == "set":
                k = int(params.get("kelvin"))
                level = _kelvin_to_ct_level(k)
                ok, info = lc.set_color_temp(level)
                return {"ok": ok, "operation": "set_color_temp", "detail": info}
            elif action == "increase":
                ok, info = lc.increase_color_temp()
                return {"ok": ok, "operation": "increase_color_temp", "detail": info}
            elif action == "decrease":
                ok, info = lc.decrease_color_temp()
                return {"ok": ok, "operation": "decrease_color_temp", "detail": info}
            else:
                return {"ok": False, "error": "color_temp.action 仅支持 set/increase/decrease"}

        # 位置 => 机械臂移动（通过 Redis 发布器）
        elif command_type == "position":
            if arm_pub is None:
                return {"ok": False, "error": "机械臂发布器未初始化"}
            if action == "move":
                direction = (params.get("direction") or "").strip().lower()
                if direction not in {"left", "right", "up", "down"}:
                    return {"ok": False, "error": "position.move.direction 仅支持 left/right/up/down"}
                ok = arm_pub.send_direction(direction)
                return {"ok": bool(ok), "operation": "arm_move_direction", "detail": {"direction": direction}}
            elif action == "reset":
                angles = default_arm_angles()
                ok = arm_pub.publish_joint_angles(angles)
                return {"ok": bool(ok), "operation": "arm_reset", "detail": {"angles": angles}}
            else:
                return {"ok": False, "error": "position.action 仅支持 move/reset"}

        # 光斑
        elif command_type == "facula":
            if action == "set":
                mode = params.get("mode")
                level = _facula_mode_to_level(mode)
                ok, info = lc.set_spot_size(level)
                if ok:
                    setattr(lc, "current_spot", level)
                return {"ok": ok, "operation": "set_spot", "detail": info}
            elif action == "increase":
                current_spot = getattr(lc, "current_spot", 2)
                target = min(3, current_spot + 1)
                ok, info = lc.set_spot_size(target)
                if ok:
                    setattr(lc, "current_spot", target)
                return {"ok": ok, "operation": "increase_spot", "detail": info}
            elif action == "decrease":
                current_spot = getattr(lc, "current_spot", 2)
                target = max(1, current_spot - 1)
                ok, info = lc.set_spot_size(target)
                if ok:
                    setattr(lc, "current_spot", target)
                return {"ok": ok, "operation": "decrease_spot", "detail": info}
            else:
                return {"ok": False, "error": "facula.action 仅支持 set/increase/decrease"}

        # 模式切换
        elif command_type == "mode_switch":
            if action != "set":
                return {"ok": False, "error": "mode_switch.action 仅支持 set"}
            mode = (params.get("mode") or "").upper()
            ok, info = lc.set_mode(mode)
            return {"ok": ok, "operation": "set_mode", "detail": info}

        # robot 拖拽
        elif command_type == "robot":
            if arm_pub is None:
                return {"ok": False, "error": "机械臂发布器未初始化"}
            state = (params.get("state") or "").strip().lower()
            # if state != "drag":
            #     return {"ok": False, "error": "robot.parameters.state 仅支持 drag"}
            if action == "start":
                ok = arm_pub.publish_drag("enable")
                return {"ok": bool(ok), "operation": "arm_drag_start", "detail": {}}
            elif action == "stop":
                ok = arm_pub.publish_drag("disable")
                return {"ok": bool(ok), "operation": "arm_drag_stop", "detail": {}}
            else:
                return {"ok": False, "error": "robot.action 仅支持 start/stop"}

        # 部位预置
        elif command_type == "bodypart":
            if arm_pub is None:
                return {"ok": False, "error": "机械臂发布器未初始化"}
            target_raw = (params.get("target") or "").strip().lower()
            target = normalize_bodypart_name(target_raw)
            presets = presets or {}
            if target not in presets:
                return {"ok": False, "error": f"未找到预置部位: {target}"}
            angles = presets[target]
            ok = arm_pub.publish_joint_angles(angles)
            return {"ok": bool(ok), "operation": "arm_move_preset", "detail": {"target": target, "angles": angles}}

        # light_point 未实现
        elif command_type == "light_point":
            return {"ok": False, "error": "light_point 暂未实现硬件映射"}

        else:
            return {"ok": False, "error": f"未知的 command_type: {command_type}"}

    except AssertionError as e:
        return {"ok": False, "error": f"参数断言失败: {str(e)}"}
    except ValueError as e:
        return {"ok": False, "error": f"参数错误: {str(e)}"}
    except Exception as e:
        return {"ok": False, "error": f"执行异常: {str(e)}"}

# ========== 主程序：对话循环 + 指令执行 ==========
def main():
    print_separator()
    print("本地 Ollama 终端对话（按 Ctrl+C 退出，或输入 'exit'/'quit'/'q'/空行）")
    print(f"模型: {MODEL_NAME}  接口: {OLLAMA_URL}")
    print(f"默认参数: {json.dumps(DEFAULT_OPTIONS, ensure_ascii=False)}")
    print_separator()

    # 初始化控制器：优先真实 LightController，失败则使用 Dummy
    if RealLightController is not None:
        try:
            light = RealLightController(port="/dev/ttyUSBlight")
            print("[Light] 已使用真实 LightController。")
        except Exception as e:
            print(f"[Light] 初始化真实 LightController 失败: {e}")
            print("[Light] 切换到 DummyLightController（不触发串口）。")
            light = DummyLightController()
    else:
        print("[Light] 未找到 light_control 模块，使用 DummyLightController（不触发串口）。")
        light = DummyLightController()

    # 机械臂 Redis 发布器
    arm_pub = None
    arm_connected = False
    arm_error = ""
    if ArmCommandPublisher is not None:
        try:
            arm_pub = ArmCommandPublisher(source="cli_console")
            ok, err = arm_pub.ping()
            arm_connected = bool(ok)
            arm_error = "" if ok else (err or "ping failed")
        except Exception as e:
            arm_connected = False
            arm_error = str(e)
    else:
        arm_error = "未找到 redis_arm.ArmCommandPublisher"

    if arm_connected:
        print("[Arm] Redis 已连接。")
    else:
        print(f"[Arm] Redis 未连接: {arm_error}")

    # 加载部位预置
    presets = load_bodypart_presets("init_part.json")
    if presets:
        print(f"[Preset] 已加载 {len(presets)} 个部位预置。")
    else:
        print("[Preset] 未加载到部位预置（bodypart 命令将返回错误）。")

    history = []

    while True:
        try:
            user_input = input("\n你 > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已退出。")
            break

        if user_input.lower() in ("exit", "quit", "q") or user_input == "":
            print("已退出。")
            break

        # 构造提示（简化：不注入系统状态；模型需输出 \json{...} 的结构化命令）
        prompt = f"用户: {user_input}\n助手:"

        result = call_ollama(prompt, model=MODEL_NAME, options=DEFAULT_OPTIONS, stream=False, url=OLLAMA_URL)
        ts = datetime.now().strftime("%H:%M:%S")

        if not result.get("success"):
            print(f"[{ts}] 请求失败: {result.get('error')}")
            continue

        model_text = result.get("text", "")

        print(f"\n[{ts}] 模型原始输出:")
        print(model_text if model_text else "(空)")

        parsed = parse_output_field(model_text)

        if isinstance(parsed, dict) and parsed.get("command_type"):
            # 若命令涉及机械臂而未连接，给出显式提示，但仍调用以得到一致返回
            cmd_type = str(parsed.get("command_type"))
            if cmd_type in ("position", "robot", "bodypart") and not arm_connected:
                print("[警告] 机械臂 Redis 未连接，该命令可能执行失败。")

            exec_result = execute_command(parsed, light, arm_pub=arm_pub, presets=presets)
            print("\n执行结果:")
            print(json.dumps(exec_result, ensure_ascii=False, indent=2))
        else:
            print("\n解析结果:")
            reply_type, reply_pretty = format_parsed(parsed)
            if reply_type == "dict":
                print("COMMAND(看起来是 dict，但缺少 command_type):")
                print(reply_pretty)
            else:
                print("CHAT:")
                print(reply_pretty)

        history.append(("user", user_input))
        history.append(("assistant", model_text))

    if light is not None:
        try:
            light.close()
        except Exception:
            pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已退出。")