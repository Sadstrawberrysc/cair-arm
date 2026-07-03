import requests
import json
import re
import ast

# Ollama 本地 API 地址
OLLAMA_API_URL = "http://localhost:11434/api/chat"

# 初始手术室状态
state = {
    "surgery_type": "神经外科",
    "lamp_power": "关",          # '开' 或 '关'
    "lamp_brightness": 0,       # 0-10
    "lamp_color_temp": 0,       # 3500/3800/4000/4200/4500 或 其他允许值
    "lamp_shadows": 0,          # 0-10（映射 facula levels）
    "lamp_mode": "INIT",        # ENDO/R9/NAVSTICK/DEPTH/INIT
    "lamp_type": "main_lamp",   # main_lamp / auxiliary_lamp
    "position": "default",      # left/right/up/down/default
    "robot_state": None,        # e.g. "drag" or None
    "light_points": {},         # 保存的光点，key=point number, value=description
    "last_bodypart": None       # 最近设置的身体部位
}

# 允许的 command_type 集合（来自你的约束）
ALLOWED_COMMAND_TYPES = {
    "switch",
    "lamp_type",
    "brightness",
    "color_temp",
    "position",
    "facula",
    "mode_switch",
    "robot",
    "light_point",
    "bodypart"
}

# allowed values
ALLOWED_COLOR_KELVIN = {3500, 3800, 4000, 4200, 4500}
ALLOWED_MODES = {"ENDO", "R9", "NAVSTICK", "DEPTH", "INIT"}
ALLOWED_POS_DIRECTIONS = {"left", "right", "up", "down"}
ALLOWED_FACULA_MODES = {"level_1", "level_2", "level_3"}
ALLOWED_LIGHT_POINT_ACTIONS = {"save", "adjust", "light_from_to"}
ALLOWED_BRIGHTNESS_ACTIONS = {"increase", "decrease", "set_level","set"}
ALLOWED_COLOR_ACTIONS = {"increase", "decrease", "set"}
ALLOWED_SWITCH_ACTIONS = {"on", "off", "open", "close", "开启", "关"}
ALLOWED_ROBOT_ACTIONS = {"start", "stop"}

def build_system_prompt(st):
    """根据当前状态生成 system prompt"""
    return (
        "你是小凯医生，一名专业的手术无影灯控制助手。"
        "参考当前无影灯的状态做出回答: "
        f"现在正在进行{st['surgery_type']}手术，"
        f"无影灯{st['lamp_power']}，亮度设置为{st['lamp_brightness']}，"
        f"色温设置为{st['lamp_color_temp']}K，光斑阴影设置为{st['lamp_shadows']}，"
        f"处于{st['lamp_mode']}模式, 照射部位为{st['position']}"
        f"机器人状态为{st['robot_state']}。"
        "Tips:"
        "1. command_type只能是以下几种之一："
        f"{list(ALLOWED_COMMAND_TYPES)}"
        "2. 如果涉及到身体部位时，'command_type'设置为'bodypart'"
        "3. 回复指令时， 先简短文字回答，再附上\\json{{}};"
        "4. 回答问题时，专业且简洁。"
    )

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

def validate_command(cmd: dict):
    """
    严格校验 parsed command 是否满足你列出的每一种 command_type 的格式与取值。
    返回 (valid: bool, reason: str)
    """
    if not isinstance(cmd, dict):
        return False, "命令不是字典结构"

    ctype = cmd.get("command_type")
    if ctype not in ALLOWED_COMMAND_TYPES:
        return False, f"未知或不允许的 command_type: {ctype}"

    action = cmd.get("action")
    params = cmd.get("parameters") or {}

    # 校验每个类型的结构与取值
    if ctype == "switch":
        # action 必须存在且为 on/off 等
        if not isinstance(action, str) or action.lower() not in ALLOWED_SWITCH_ACTIONS:
            return False, "switch 的 action 必须是 'on'/'off' 等"
        # parameters.target 可选，但若存在必须是指定集合
        if "target" in params:
            if params["target"] not in {"all", "main_lamp", "auxiliary_lamp"}:
                return False, "switch.parameters.target 必须为 'all'/'main_lamp'/'auxiliary_lamp'"
        return True, "ok"

    if ctype == "lamp_type":
        if action != "set":
            return False, "lamp_type 的 action 必须是 'set'"
        tgt = params.get("target")
        if tgt not in {"auxiliary_lamp", "main_lamp"}:
            return False, "lamp_type.parameters.target 必须为 'auxiliary_lamp' 或 'main_lamp'"
        return True, "ok"

    if ctype == "brightness":
        if action not in ALLOWED_BRIGHTNESS_ACTIONS:
            return False, f"brightness 的 action 必须在 {ALLOWED_BRIGHTNESS_ACTIONS}"
        # set_level 要求 level
        if action == "set_level" or action == "set":
            if "level" not in params:
                return False, "brightness set_level 需要 parameters.level"
            try:
                lv = int(params["level"])
                if not (1 <= lv <= 10):
                    return False, "brightness.level 必须在 1-10"
            except Exception:
                return False, "brightness.level 需要是整数"
        else:
            # increase/decrease 可选 steps
            if "steps" in params:
                try:
                    stp = int(params["steps"])
                    if not (1 <= stp <= 9):
                        return False, "brightness.steps 必须在 1-9"
                except Exception:
                    return False, "brightness.steps 必须是整数"
        return True, "ok"

    if ctype == "color_temp":
        if action not in ALLOWED_COLOR_ACTIONS:
            return False, f"color_temp action 必须在 {ALLOWED_COLOR_ACTIONS}"
        if action == "set":
            try:
                k = int(list(params.values())[0])
                if k not in ALLOWED_COLOR_KELVIN:
                    return False, f"color_temp.kelvin 必须在 {sorted(ALLOWED_COLOR_KELVIN)}"
            except Exception:
                return False, "color_temp.kelvin 必须为整数"
        return True, "ok"

    if ctype == "position":
        if action not in {"move", "reset"}:
            return False, "position action 必须是 'move' 或 'reset'"
        if action == "move":
            if "direction" not in params or params["direction"] not in ALLOWED_POS_DIRECTIONS:
                return False, f"position.move 需要 parameters.direction 在 {ALLOWED_POS_DIRECTIONS}"
        if action == "reset":
            # reset 可含 position:'default'
            if "position" in params and params["position"] != "default":
                return False, "position.reset 的 parameters.position 必须是 'default'（若存在）"
        return True, "ok"

    if ctype == "facula":
        if action not in {"increase", "decrease", "set"}:
            return False, "facula action 必须是 increase/decrease/set"
        if action == "set":
            if "mode" not in params or params["mode"] not in ALLOWED_FACULA_MODES:
                return False, f"facula.set 需要 parameters.mode 在 {ALLOWED_FACULA_MODES}"
        return True, "ok"

    if ctype == "mode_switch":
        if action != "set":
            return False, "mode_switch 的 action 必须是 'set'"
        if "mode" not in params or params["mode"] not in ALLOWED_MODES:
            return False, f"mode_switch.parameters.mode 必须在 {ALLOWED_MODES}"
        return True, "ok"

    if ctype == "robot":
        if action not in ALLOWED_ROBOT_ACTIONS:
            return False, f"robot action 必须在 {ALLOWED_ROBOT_ACTIONS}"
        # parameters.state 可选，但若存在必须为字符串
        if "state" in params and not isinstance(params["state"], str):
            return False, "robot.parameters.state 必须是字符串"
        return True, "ok"

    if ctype == "light_point":
        if action not in ALLOWED_LIGHT_POINT_ACTIONS:
            return False, f"light_point action 必须在 {ALLOWED_LIGHT_POINT_ACTIONS}"
        if action in {"save", "adjust"}:
            if "point" not in params:
                return False, f"{action} 需要 parameters.point (1-10)"
            try:
                p = int(params["point"])
                if not (1 <= p <= 10):
                    return False, "light_point.point 必须在 1-10"
            except Exception:
                return False, "light_point.point 必须为整数"
            if action == "adjust" and "target" not in params:
                return False, "light_point.adjust 需要 parameters.target (previous 等)"
        if action == "light_from_to":
            if "from_point" not in params or "to_point" not in params:
                return False, "light_from_to 需要 from_point 和 to_point"
            try:
                f = int(params["from_point"]); t = int(params["to_point"])
                if not (1 <= f <= 10 and 1 <= t <= 10):
                    return False, "from_point/to_point 必须在 1-10"
            except Exception:
                return False, "from_point/to_point 必须为整数"
        return True, "ok"

    if ctype == "bodypart":
        if action != "set":
            return False, "bodypart action 必须是 'set'"
        tgt = params.get("target")
        if not isinstance(tgt, str):
            tgt = params.get("bodypart", "")
        allowed_bodyparts = {
            "neck","chest","abdomen","left_upper_arm","left_lower_arm","left_hand",
            "right_upper_arm","right_lower_arm","right_hand","left_thigh","left_foot",
            "right_thigh","right_foot"
        }
        if tgt not in allowed_bodyparts:
            return False, f"bodypart.parameters.target 必须在 {sorted(allowed_bodyparts)}"
        return True, "ok"

    return False, "未处理的 command_type"

def clamp_int(value, lo, hi):
    try:
        v = int(value)
    except Exception:
        return None
    return max(lo, min(hi, v))

def apply_command(cmd: dict, st: dict):
    """
    在保证 validate_command 通过的前提下应用命令，修改 state 并返回 (success, message)
    """
    ctype = cmd.get("command_type")
    action = cmd.get("action")
    params = cmd.get("parameters") or {}

    if ctype == "switch":
        a = str(action).lower()
        if a in {"on", "open", "开启"}:
            target = params.get("target", "all")
            if target == "all":
                st["lamp_power"] = "开"
            elif target == "main_lamp":
                # 对主灯单独处理（此处只是记录 lamp_type 或可扩展）
                st["lamp_power"] = "开"
                st["lamp_type"] = "main_lamp"
            else:
                st["lamp_power"] = "开"
                st["lamp_type"] = "auxiliary_lamp"
            return True, f"已打开 ({target})"
        else:
            # off
            target = params.get("target", "all")
            st["lamp_power"] = "关"
            return True, f"已关闭 ({target})"

    if ctype == "lamp_type":
        tgt = params["target"]
        st["lamp_type"] = tgt
        return True, f"灯具类型已设置为 {tgt}"

    if ctype == "brightness":
        if action == "set_level" or action == "set":
            lv = clamp_int(params["level"], 1, 10)
            st["lamp_brightness"] = lv
            return True, f"亮度已设置为 {lv}"
        elif action == "increase":
            steps = int(params.get("steps", 1))
            st["lamp_brightness"] = clamp_int(st.get("lamp_brightness", 0) + steps, 0, 10)
            return True, f"亮度增加 {steps}，当前 {st['lamp_brightness']}"
        else:  # decrease
            steps = int(params.get("steps", 1))
            st["lamp_brightness"] = clamp_int(st.get("lamp_brightness", 0) - steps, 0, 10)
            return True, f"亮度减少 {steps}，当前 {st['lamp_brightness']}"

    if ctype == "color_temp":
        if action == "set":
            k = int(list(params.values())[0])
            st["lamp_color_temp"] = k
            return True, f"色温已设置为 {k}K"
        elif action == "increase":
            # 如果当前是允许集合中的值，向上取下一个允许值；否则直接设置为 max 值
            cur = st.get("lamp_color_temp", 0)
            sorted_vals = sorted(ALLOWED_COLOR_KELVIN)
            for v in sorted_vals:
                if v > cur:
                    st["lamp_color_temp"] = v
                    return True, f"色温增加到 {v}K"
            st["lamp_color_temp"] = sorted_vals[-1]
            return True, f"色温设置为最高 {sorted_vals[-1]}K"
        else:  # decrease
            cur = st.get("lamp_color_temp", 0)
            sorted_vals = sorted(ALLOWED_COLOR_KELVIN)
            for v in reversed(sorted_vals):
                if v < cur:
                    st["lamp_color_temp"] = v
                    return True, f"色温降低到 {v}K"
            st["lamp_color_temp"] = sorted_vals[0]
            return True, f"色温设置为最低 {sorted_vals[0]}K"

    if ctype == "position":
        if action == "move":
            d = params["direction"]
            st["position"] = d
            return True, f"位置向 {d} 移动"
        else:  # reset
            st["position"] = "default"
            return True, "位置已复位到 default"

    if ctype == "facula":
        if action == "set":
            mode = params["mode"]
            # 将 level 映射为 lamp_shadows 数值：level_1->3, level_2->6, level_3->9
            mapping = {"level_1": 3, "level_2": 6, "level_3": 9}
            st["lamp_shadows"] = mapping.get(mode, st["lamp_shadows"])
            return True, f"光斑模式设置为 {mode}，阴影值 {st['lamp_shadows']}"
        elif action == "increase":
            st["lamp_shadows"] = clamp_int(st["lamp_shadows"] + 1, 0, 10)
            return True, f"光斑增大，阴影 {st['lamp_shadows']}"
        else:
            st["lamp_shadows"] = clamp_int(st["lamp_shadows"] - 1, 0, 10)
            return True, f"光斑减小，阴影 {st['lamp_shadows']}"

    if ctype == "mode_switch":
        m = params["mode"]
        st["lamp_mode"] = m
        return True, f"模式已切换为 {m}"

    if ctype == "robot":
        if action == "start":
            st["robot_state"] = params.get("state", "started")
            return True, f"机器人已启动 ({st['robot_state']})"
        else:
            st["robot_state"] = "stopped"
            return True, "机器人已停止"

    if ctype == "light_point":
        if action == "save":
            p = int(params["point"])
            st["light_points"][p] = {"saved": True}
            return True, f"光点 {p} 已保存"
        if action == "adjust":
            p = int(params["point"])
            tgt = params.get("target", "previous")
            st["light_points"][p] = {"adjusted_to": tgt}
            return True, f"光点 {p} 已调整到 {tgt}"
        if action == "light_from_to":
            f = int(params["from_point"]); t = int(params["to_point"])
            st["light_points"]["last_move"] = {"from": f, "to": t}
            return True, f"光点从 {f} 移动到 {t}"

    if ctype == "bodypart":
        try:
            tgt = params["target"]
        except:
            tgt = params["bodypart"]
        st["last_bodypart"] = tgt
        return True, f"已设关注部位：{tgt}"

    return False, "未实现的命令类型"

def extract_and_apply_from_model_reply(reply_text: str, st: dict):
    """
    从模型回复中提取 json 指令（如果有），解析、校验并应用到状态 st。
    返回 (applied: bool, message: str, parsed_command: dict or None)
    """
    jtext = find_json_like(reply_text)
    if not jtext:
        return False, "回复中未找到 json 指令。", None

    parsed = parse_json_like_string(jtext)
    if parsed is None:
        return False, f"提取到的 json 字符串但解析失败: {jtext}", None

    # 校验
    valid, reason = validate_command(parsed)
    if not valid:
        return False, f"命令校验失败: {reason}", parsed

    # 应用
    ok, msg = apply_command(parsed, st)
    return ok, msg, parsed

def chat_with_ollama(user_input: str, model_name="mirrorwanswer_ollama:latest"):
    """
    发送用户输入到 Ollama 模型并返回模型原始回复文本。
    同时会使用当前 state 生成 system prompt（确保上下文一致）。
    """
    print(build_system_prompt(state))
    payload = {
        "model": model_name,
        "messages": [
            {"role": "system", "content": build_system_prompt(state)},
            {"role": "user", "content": user_input}
        ],
        "temperature": 0,
        "repeat_penalty": 1.0,
        "top_p": 0.0,
        "top_k": 1,
        "stream": False
    }

    try:
        response = requests.post(OLLAMA_API_URL, json=payload, timeout=120)
        response.raise_for_status()
        data = response.json()
        reply = data.get("message", {}).get("content", "")
        return reply.strip()
    except requests.RequestException as e:
        return f"[Error communicating with Ollama API: {e}]"
    except json.JSONDecodeError:
        return "[Error parsing Ollama response]"

if __name__ == "__main__":
    print("🪞 MirrorWAnswer (Ollama Chat Interface) with strict command schema")
    print("Type 'exit' to quit.\n")

    while True:
        user_input = input("You: ").strip()
        if user_input.lower() in {"exit", "quit"}:
            print("Goodbye!")
            break

        model_reply = chat_with_ollama(user_input)
        print(f"MirrorWAnswer: {model_reply}\n")

        applied, msg, parsed = extract_and_apply_from_model_reply(model_reply, state)
        if applied:
            print(f"[指令已应用] {msg}")
        else:
            print(f"[未应用] {msg}")
        if parsed:
            print(f"[解析到的命令] {parsed}")

        # 把最新状态打印出来，便于查看
        print("\n当前灯状态：")
        for k, v in state.items():
            print(f"  {k}: {v}")
        print("\n" + "-"*40 + "\n")
