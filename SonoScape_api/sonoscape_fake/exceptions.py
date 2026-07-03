# ==============================================================================
# 文件: ultrasound_client/exceptions.py
# 描述: 为客户端定义自定义异常。
# ==============================================================================
class UltrasoundError(Exception):
    """超声客户端所有异常的基类。"""
    pass

class UltrasoundConnectionError(UltrasoundError):
    """当发生套接字连接、超时或握手问题时引发。"""
    pass

class UltrasoundProtocolError(UltrasoundError):
    """当发生协议违规时引发，例如校验和错误或帧格式无效。"""
    pass

class UltrasoundCommandError(UltrasoundError):
    """当设备报告命令执行失败时引发（例如，响应字节为 0xFF）。"""
    def __init__(self, message, command_byte, response_frame):
        full_message = f"{message} (命令: {command_byte:#04x})"
        super().__init__(full_message)
        self.command_byte = command_byte
        self.response_frame = response_frame