import stat
from typing import Any, Optional
import re
from src.constants.constants import AbortReason, DeviceState
from src.utils.logging_config import get_logger
from src.plugins.base import Plugin
import os
import sys
# 相对路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

logger = get_logger(__name__)

class UIPlugin(Plugin):
    """UI 插件 - 管理 CLI/GUI 显示"""

    name = "ui"

    # 设备状态文本映射
    STATE_TEXT_MAP = {
        DeviceState.IDLE: "待命",
        DeviceState.LISTENING: "聆听中...",
        DeviceState.SPEAKING: "说话中...",
    }

    def __init__(self, mode: Optional[str] = None, redis_publisher = None,
                 light = None, arm_pub = None, presets = None, arm_connected = False) -> None:
        super().__init__()
        self.app = None
        self.mode = (mode or "cli").lower()
        self.display = None
        self._is_gui = False
        self.is_first = True
        self.redis_publisher = redis_publisher
        self.light = light
        self.arm_pub = arm_pub
        self.presets = presets
        self.arm_connected = arm_connected
        # 聊天消息缓冲：[{role: \"user\"/\"assistant\", \"text\": str, \"type\": \"chat\"/\"llmcmd\"}]
        self._chat_messages = []

    async def setup(self, app: Any) -> None:
        """
        初始化 UI 插件.
        """
        self.app = app

        # 创建对应的 display 实例
        self.display = self._create_display()

        # 禁用应用内控制台输入
        if hasattr(app, "use_console_input"):
            app.use_console_input = False

    def _create_display(self):
        """
        根据模式创建 display 实例.
        """
        if self.mode == "gui":
            from src.display.gui_display import GuiDisplay

            self._is_gui = True
            return GuiDisplay()

        elif self.mode == "chatgui":
            from src.display.chat_gui_display import GuiDisplay

            self._is_gui = True
            return GuiDisplay()
        else:
            from src.display.cli_display import CliDisplay

            self._is_gui = False
            return CliDisplay()

    def validate_command_type(self,s):
        """检测字符串中 'command_type' 是否在允许的集合中"""
        if s is None:
            return None
        allowed_types = {
            "switch",
            "lamp_type",
            "brightness",
            "color_temp",
            "position",
            "facula",
            "mode_switch",
            "robot",
            "light_point",
            "bodypart",
        }

        # 提取类似 'command_type':'xxx' 或 "command_type":"xxx"
        pattern = r"['\"]command_type['\"]\s*:\s*['\"]([a-zA-Z_]+)['\"]"
        matches = re.findall(pattern, s)

        if not matches:
            return None

        # 检查所有匹配项
        invalid = [m for m in matches if m not in allowed_types]
        if invalid:
            return None
        else:
            return s

    async def start(self) -> None:
        """
        启动 UI 显示.
        """
        if not self.display:
            return

        # 绑定回调
        await self._setup_callbacks()
        await self.app.protocol.send_last_status_update(self.app.last_status)


        # 启动显示
        self.app.spawn(self.display.start(), name=f"ui:{self.mode}:start")

    async def _setup_callbacks(self) -> None:
        """
        设置 display 回调.
        """
        if self._is_gui:
            # GUI 需要调度到异步任务
            callbacks = {
                "press_callback": self._wrap_callback(self._press),
                "release_callback": self._wrap_callback(self._release),
                "auto_callback": self._wrap_callback(self._auto_toggle),
                "abort_callback": self._wrap_callback(self._abort),
                "send_text_callback": self._send_text,
            }
        else:
            # CLI 直接传递协程函数
            callbacks = {
                "auto_callback": self._auto_toggle,
                "abort_callback": self._abort,
                "send_text_callback": self._send_text,
            }

        await self.display.set_callbacks(**callbacks)

    def _wrap_callback(self, coro_func):
        """
        包装协程函数为可调度的 lambda.
        """
        return lambda: self.app.spawn(coro_func(), name="ui:callback")

    async def on_incoming_json(self, message: Any) -> None:
        """
        处理传入的 JSON 消息.

        - stt: 视为用户输入，使用右侧气泡展示
        - tts: 视为助手回复，使用左侧气泡展示
        - llmcmd: 工具调用结果，直接文本展示，不包裹气泡
        """
        if not self.display or not isinstance(message, dict):
            return

        import json

        msg_type = message.get("type")
        print(f"\n[LLM消息] {json.dumps(message, ensure_ascii=False, indent=2)}\n")

        # tts/stt：语音输入输出 -> 聊天气泡
        if msg_type in ("tts", "stt"):
            text = (message.get("text") or "").strip()
            if text:
                # 保持原有行为：更新 display 文本，兼容 CLI/旧 GUI
                await self.display.update_text(text)
                # 新行为：追加到聊天消息列表
                role = "assistant" if msg_type == "tts" else "user"
                self._append_chat_message(role=role, text=text, msg_type="chat")

        # llm：仅更新表情
        elif msg_type == "llm":
            emotion = message.get("emotion")
            if emotion:
                await self.display.update_emotion(emotion)

        # llmcmd：工具调用结果
        elif msg_type == "llmcmd":
            model_text = (message.get("text") or "").strip()
            if model_text:
                print(f"Redis to send: {model_text}")
                # if self.validate_command_type(llmcmd):
                return_value = self.redis_publisher.send_raw_command(model_text)
                self.app.last_status = return_value.get('data')
                result_tools = re.search(r'已执行: (.*)', self.app.last_status)
                if len(result_tools.groups()) > 0:
                    if len(result_tools.groups()[0]) > 2:
                        result_text = result_tools.groups()[0]
                print(f"result_text: {result_text}")
                self.app.last_status = self.app.last_status.replace(result_text, '[]]')
                await self.app.protocol.send_last_status_update(self.app.last_status)

                # parsed = parse_output_field(model_text.replace("\n","")[1:-1])
                # print(f"Parsed command: {parsed}")
                # print(f"type: {type(parsed)}")
                # print(f"command_type: {parsed.get('command_type') if isinstance(parsed, dict) else 'N/A'}")


                if result_text and self.mode == "chatgui":
                    if "success" in result_text.lower():
                        msg_type = "llmcmd_ok"
                    elif "error" in result_text.lower():
                        msg_type = "llmcmd_error"
                    else:
                        msg_type = "llmcmd"

                    self._append_chat_message(
                        role="assistant", text=result_text, msg_type=msg_type
                )

                # 聊天窗口中展示三种类型：普通/ok/error
                # 第一行：原始 llmcmd 文本（始终灰色）
                # 非 chatgui 模式：沿用原有文本更新逻辑
                if "null" not in model_text:
                    await self.display.update_text(model_text)
                    self._append_chat_message(
                        role="assistant", text=model_text, msg_type="llmcmd"
                    )
                # print(f"self.chat_messages: {self._chat_messages}")


    def _append_chat_message(self, role: str, text: str, msg_type: str = "chat") -> None:
        """
        维护聊天消息队列，并通知显示层更新。

        role: \"user\" | \"assistant\"
        msg_type: \"chat\"（气泡）| \"llmcmd\"（工具结果，裸文本）
        """
        text = (text or "").strip()
        if not text:
            return

        # 限制消息数量，避免无限增长导致 UI 卡顿
        max_messages = 100
        self._chat_messages.append(
            {
                "role": role,
                "text": text,
                "type": msg_type,
            }
        )
        if len(self._chat_messages) > max_messages:
            self._chat_messages = self._chat_messages[-max_messages:]

        # 显示层如果实现了 update_chat，则推送整个列表
        display = self.display
        if display and hasattr(display, "update_chat"):
            # 使用应用的任务调度，避免阻塞当前协程
            try:
                self.app.spawn(
                    display.update_chat(list(self._chat_messages)),
                    name="ui:update_chat",
                )
            except Exception:
                # 不影响主流程
                logger.exception("更新聊天消息到显示层时出错")


    async def on_device_state_changed(self, state: Any) -> None:
        """
        设备状态变化处理.
        """
        if not self.display:
            return

        # 跳过首次调用
        if self.is_first:
            self.is_first = False
            return

        # 更新表情和状态
        await self.display.update_emotion("neutral")
        if status_text := self.STATE_TEXT_MAP.get(state):
            await self.display.update_status(status_text, True)

    async def shutdown(self) -> None:
        """
        清理 UI 资源，关闭窗口.
        """
        if self.display:
            await self.display.close()
            self.display = None

    # ===== 回调函数 =====

    async def _send_text(self, text: str):
        """
        发送文本到服务端.
        """
        if self.app.device_state == DeviceState.SPEAKING:
            audio_plugin = self.app.plugins.get_plugin("audio")
            if audio_plugin:
                await audio_plugin.codec.clear_audio_queue()
            await self.app.abort_speaking(None)
        if await self.app.connect_protocol():
            await self.app.protocol.send_wake_word_detected(text)

    async def _press(self):
        """
        手动模式：按下开始录音.
        """
        await self.app.start_listening_manual()

    async def _release(self):
        """
        手动模式：释放停止录音.
        """
        await self.app.stop_listening_manual()

    async def _auto_toggle(self):
        """
        自动模式切换.
        """
        await self.app.start_auto_conversation()

    async def _abort(self):
        """
        中断对话.
        """
        await self.app.abort_speaking(AbortReason.USER_INTERRUPTION)
