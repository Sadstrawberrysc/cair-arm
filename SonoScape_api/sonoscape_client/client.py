# ==============================================================================
# 文件: ultrasound_client/client.py
# 描述: 包含核心的 UltrasoundClient 类。
# ==============================================================================
import socket
import struct
import json
import redis
from datetime import datetime
from .protocol import *
from .exceptions import *

class UltrasoundClient:
    """一个通过TCP与超声主机通信的客户端，实现了所有指定的协议功能。"""

    def __init__(self, host, port=6061, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        """连接到超声主机并执行握手。"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            self._handshake()
        except socket.timeout:
            self.disconnect()
            raise UltrasoundConnectionError(f"连接超时 ({self.timeout}s)。")
        except socket.error as e:
            self.disconnect()
            raise UltrasoundConnectionError(f"连接失败: {e}")

    def _handshake(self):
        """执行协议握手。"""
        self.sock.sendall(HANDSHAKE_PACKET)
        response = self.sock.recv(len(HANDSHAKE_PACKET))
        if response != HANDSHAKE_PACKET:
            raise UltrasoundConnectionError("握手失败：响应不正确。")

    def disconnect(self):
        """断开与超声主机的连接。"""
        if self.sock:
            self.sock.close()
            self.sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    @staticmethod
    def _calculate_checksum(payload_bytes):
        """计算校验和 (Byte1到Byte27相加，取低八位)。"""
        return sum(payload_bytes) & 0xFF

    def _build_frame(self, command_byte, data=b'', header=FRAME_HEADER_CONTROL, tail=FRAME_TAIL):
        """构建一个30字节的数据帧。"""
        payload = bytearray(27)
        payload[0] = command_byte
        payload[1:1 + len(data)] = data
        checksum = self._calculate_checksum(payload)
        return bytes([header]) + payload + bytes([checksum, tail])

    def _send_and_receive(self, frame):
        """发送帧并接收一个完整的响应帧。"""
        if not self.sock:
            raise UltrasoundConnectionError("客户端未连接。")
        self.sock.sendall(frame)
        response = self.sock.recv(FRAME_SIZE)
        if len(response) < FRAME_SIZE:
            raise UltrasoundProtocolError(f"接收到的数据不完整 (收到 {len(response)} 字节, 需要 {FRAME_SIZE} 字节)。")
        return response

    def _validate_response(self, frame, expected_header, expected_cmd):
        """验证响应帧的头部、尾部和校验和。"""
        header, tail = frame[0], frame[29]
        if header != expected_header or tail != FRAME_TAIL:
            raise UltrasoundProtocolError(f"无效的帧头/帧尾。收到 H:{header:#04x} T:{tail:#04x}")
        
        payload = frame[1:28]
        received_checksum = frame[28]
        calculated_checksum = self._calculate_checksum(payload)
        if received_checksum != calculated_checksum:
            raise UltrasoundProtocolError("响应校验和错误。")
        
        if frame[1] != expected_cmd:
            raise UltrasoundProtocolError(f"响应命令不匹配 (需要: {expected_cmd:#04x}, 收到: {frame[1]:#04x})。")

    def _execute_control_command(self, command_byte, data, success_byte_index):
        """执行一个通用的控制命令并检查其成功状态。"""
        frame = self._build_frame(command_byte, data, header=FRAME_HEADER_CONTROL)
        print(frame)
        response = self._send_and_receive(frame)
        print(response)

        self._validate_response(response, FRAME_HEADER_CONTROL, command_byte)


        
        if response[success_byte_index] == 0xFF:
            raise UltrasoundCommandError("设备报告命令失败", command_byte, response)
        
        return response

    # --- API 方法 ---

    def get_status(self):
        """获取完整的设备状态。"""
        frame = self._build_frame(CMD_GET_STATUS, header=FRAME_HEADER_STATUS)
        response = self._send_and_receive(frame)
        self._validate_response(response, FRAME_HEADER_STATUS, CMD_GET_STATUS)
        
        return {
            "scan_mode": '基波' if chr(response[2]) == 'F' else '谐波',
            "probe_active": '激活' if chr(response[3]) == 'U' else '未激活',
            "is_b_mode": '是' if chr(response[4]) == 'B' else '否',
            "is_frozen": '冻结' if chr(response[5]) == 'F' else '解冻',
            "is_new_patient_ui": '是' if chr(response[6]) == 'P' else '否',
            "fps": f"{response[7]}.{response[8]}",
            "d_g": f"D={(response[9] << 8) + response[10]}, G={response[11]}",
            "gain_gn": response[12],
            "i_p": f"I={response[13]}, P={response[14]}",
            "power_pwr": response[15],
            "frequency_frq": f"第 {response[16]} 档",
            "depth_d_mm": response[17],
            "micro_imaging_us": response[18],
            "image_start_x_percent": round(((response[19] << 8) + response[20]) / 100, 2),
            "image_start_y_percent": round(((response[21] << 8) + response[22]) / 100, 2),
            "image_width_px": (response[23] << 8) + response[24],
            "image_height_px": (response[25] << 8) + response[26],
        }

    def control_s_marker(self, show: bool):
        """控制'S'标记的显示或隐藏。"""
        data = b'SSHOW' if show else b'SHIDE'
        self._execute_control_command(CMD_CONTROL_S_MARKER, data, success_byte_index=7)
        return True

    def control_ruler(self, show: bool):
        """控制图像标尺（刻度线）的显示或隐藏。"""
        data = b'MSHOW' if show else b'MHIDE'
        self._execute_control_command(CMD_CONTROL_RULER, data, success_byte_index=7)
        return True

    def get_disk_space(self):
        """获取剩余硬盘容量（以G为单位）。"""
        response = self._execute_control_command(CMD_GET_DISK_SPACE, b'GDISK', success_byte_index=6)
        size_str = response[2:6].decode('ascii')
        return int(size_str)

    def set_scan_mode(self, mode: ScanMode):
        """设置扫描模式（基波或谐波）。"""
        self._execute_control_command(CMD_SET_SCAN_MODE, mode.value, success_byte_index=7)
        return True

    def get_patient_name(self):
        """获取当前患者的姓名。"""
        response = self._send_and_receive(self._build_frame(CMD_GET_PATIENT_NAME, b'NAME'))
        self._validate_response(response, FRAME_HEADER_CONTROL, CMD_GET_PATIENT_NAME)
        
        length = response[2]
        if length == 0xFF:
            raise UltrasoundCommandError("获取患者姓名失败", CMD_GET_PATIENT_NAME, response)
        if length == 0:
            return ""
        return response[3:3 + length].decode('utf-8', errors='ignore')

    def get_patient_id(self):
        """获取当前患者的ID。"""
        response = self._send_and_receive(self._build_frame(CMD_GET_PATIENT_ID, b'PID'))
        self._validate_response(response, FRAME_HEADER_CONTROL, CMD_GET_PATIENT_ID)

        length = response[2]
        if length == 0xFF:
            raise UltrasoundCommandError("获取患者ID失败", CMD_GET_PATIENT_ID, response)
        if length == 0:
            return ""
        return response[3:3 + length].decode('ascii', errors='ignore')

    def get_patient_scan_time(self):
        """获取患者扫描时间。"""
        response = self._execute_control_command(CMD_GET_SCAN_TIME, b'DATE', success_byte_index=21)
        
        time_str = (
            f"{response[2:6].decode('ascii')}-"  # Year
            f"{response[7:9].decode('ascii')}-"  # Month
            f"{response[10:12].decode('ascii')} " # Day
            f"{response[13:15].decode('ascii')}:" # Hour
            f"{response[16:18].decode('ascii')}:" # Minute
            f"{response[19:21].decode('ascii')}"  # Second
        )
        return datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S')

    # def set_scan_depth(self, depth_cm: float):
    #     """设置扫描深度。"""
    #     depth_mm_str = str(int(depth_cm * 10)).zfill(2)
    #     data = b'D' + depth_mm_str.encode('ascii')
    #     self._execute_control_command(CMD_SET_SCAN_DEPTH, data, success_byte_index=5)
    #     return 
        
    def set_scan_depth(self, depth_mm: float):
        """设置扫描深度。"""
        depth_mm_str = str(int(depth_mm)).zfill(2)
        data = b'D' + depth_mm_str.encode('ascii')
        self._execute_control_command(CMD_SET_SCAN_DEPTH, data, success_byte_index=5)
        return True

    def set_image_type(self, image_type: ImageType):
        """设置显示的图像类型（处理前/后）。"""
        self._execute_control_command(CMD_SET_IMAGE_TYPE, image_type.value, success_byte_index=5)
        return True

    def start_image_acquisition(self, params: str):
        """开始采集图像。 params: e.g., 'SLT', 'SRH'"""
        data = b'S' + params.encode('ascii')
        self._execute_control_command(CMD_START_ACQUISITION, data, success_byte_index=5)
        return True

    def abort_image_acquisition(self):
        """中止图像采集。"""
        self._execute_control_command(CMD_ABORT_ACQUISITION, b'ABORT', success_byte_index=7)
        return True

    def end_image_acquisition(self):
        """结束图像采集。"""
        self._execute_control_command(CMD_END_ACQUISITION, b'EF', success_byte_index=4)
        return True

    def control_freeze(self, freeze: bool):
        """控制图像冻结或解冻。"""
        data = b'DOFZ' if freeze else b'UNFZ'
        self._execute_control_command(CMD_CONTROL_FREEZE, data, success_byte_index=6)
        return True

    def save_and_send_image(self):
        """控制主机保存并发送图像。"""
        response = self._execute_control_command(CMD_SAVE_SEND_IMAGE, b'FND', success_byte_index=6)
        # 协议描述'FNDA'，但长度为3，故使用'FND'
        num_servers = int(chr(response[5]))
        return {"success": True, "dicom_servers": num_servers}

    def new_patient(self):
        """跳转到新建患者界面。"""
        self._execute_control_command(CMD_NEW_PATIENT, b'PEE', success_byte_index=5)
        return True

    def get_dicom_send_status(self, patient_id: str):
        """根据患者ID获取DICOM发送的成功/失败计数。"""
        id_bytes = patient_id.encode('ascii')
        id_len = len(id_bytes)
        if id_len > 22:
            raise ValueError("患者ID过长。")
        
        data = b'DN' + bytes([id_len]) + id_bytes
        response = self._send_and_receive(self._build_frame(CMD_GET_DICOM_STATUS, data))
        self._validate_response(response, FRAME_HEADER_CONTROL, CMD_GET_DICOM_STATUS)

        if response[5] == 0x01: # ID不存在
            return {"exists": False, "success_count": 0, "fail_count": 0}
        
        return {
            "exists": True,
            "success_count": response[2],
            "fail_count": response[3],
            "id_returned": response[6:6+response[4]].decode('ascii')
        }

    def get_zoom_factor(self):
        """获取图像放大倍数。"""
        response = self._execute_control_command(CMD_GET_ZOOM, b'ZOOM', success_byte_index=6)
        integer_part = (response[2] << 8) + response[3]
        decimal_part = (response[4] << 8) + response[5]
        return float(f"{integer_part}.{decimal_part}")

    def set_b_gain(self, gain: int):
        """设置B模式增益值。"""
        if not 0 <= gain <= 999:
            raise ValueError("增益值必须在 0 和 999 之间。")
        gain_str = str(gain).zfill(3)
        data = b'BGS' + gain_str.encode('ascii')
        print(data)
        # 协议文档在成功字节索引上存在矛盾。假设为Byte8。
        self._execute_control_command(CMD_SET_B_GAIN, data, success_byte_index=8)
        return True

    def robot_start(self, bodypart: str):
        """发送机械臂控制命令到Redis。"""
        print(f"机械臂扫描，部位: {bodypart}")
        try:
            r = redis.Redis(host='localhost', port=7777, db=0)
            message = {
                "command": "robot",
                "action": "move",
                "parameter": bodypart
            }
            r.publish('robot_control', json.dumps(message, ensure_ascii=False))
            return True
        except Exception as e:
            print(f"Failed to publish robot command: {e}")
            return False