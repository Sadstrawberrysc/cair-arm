# ==============================================================================
# 文件: ultrasound_client/protocol.py
# 描述: 定义协议常量、命令代码和枚举。
# ==============================================================================
from enum import Enum

# --- 帧常量 ---
FRAME_HEADER_STATUS = 0xB6
FRAME_HEADER_CONTROL = 0xB7
FRAME_TAIL = 0xBB
FRAME_SIZE = 30
HANDSHAKE_PACKET = b'\xB6\xF1\xF9\xC8'

# --- 命令代码 ---
CMD_GET_STATUS = 0x33
CMD_CONTROL_S_MARKER = 0x01
CMD_CONTROL_RULER = 0x02
CMD_GET_DISK_SPACE = 0x03
CMD_SET_SCAN_MODE = 0x04
CMD_GET_PATIENT_NAME = 0x05
CMD_GET_PATIENT_ID = 0x06
CMD_GET_SCAN_TIME = 0x07
CMD_SET_SCAN_DEPTH = 0x08
CMD_SET_IMAGE_TYPE = 0x09
CMD_START_ACQUISITION = 0x0A
CMD_ABORT_ACQUISITION = 0x0B
CMD_END_ACQUISITION = 0x0C
CMD_CONTROL_FREEZE = 0x0D
CMD_SAVE_SEND_IMAGE = 0x0E
CMD_NEW_PATIENT = 0x10
CMD_GET_DICOM_STATUS = 0x11
CMD_GET_ZOOM = 0x12
CMD_SET_B_GAIN = 0x13

# --- 为提高可读性使用枚举 ---
class ScanMode(Enum):
    FUNDAMENTAL = b'FMODE'  # 基波
    HARMONIC = b'HMODE'    # 谐波

class ImageType(Enum):
    POST_PROCESSED = b'SIA'  # 后处理图像
    PRE_PROCESSED = b'SIB'   # 未经后处理的图像