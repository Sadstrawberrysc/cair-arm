# ==============================================================================
# 文件: sonoscape_fake/client.py
# 描述: 模拟 UltrasoundClient 类，用于测试或无设备时的回退。
# ==============================================================================
import time
import json
import redis
from datetime import datetime
from .protocol import ScanMode, ImageType

class UltrasoundClient:
    """一个模拟的超声客户端，不进行实际的网络通信。"""

    def __init__(self, host, port=6061, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.is_connected = False
        
        # 初始化系统状态变量
        self.system_state = {
            "scan_mode": '基波',
            "probe_active": '激活',
            "is_b_mode": '是',
            "is_frozen": '解冻',
            "is_new_patient_ui": '否',
            "fps": "30.0",
            "d_g": "D=150, G=80",
            "gain_gn": 100,
            "i_p": "I=5, P=10",
            "power_pwr": 80,
            "frequency_frq": "第 2 档",
            "depth_d_mm": 150,
            "micro_imaging_us": 0,
            "image_start_x_percent": 0.0,
            "image_start_y_percent": 0.0,
            "image_width_px": 800,
            "image_height_px": 600,
        }

        print(f"[FakeClient] 初始化连接到 {host}:{port} (模拟)")

    def connect(self):
        print(f"[FakeClient] 正在连接 {self.host}:{self.port}...")
        time.sleep(0.5) # 模拟网络延迟
        self.is_connected = True
        print("[FakeClient] 连接成功。")

    def disconnect(self):
        if self.is_connected:
            print("[FakeClient] 断开连接。")
            self.is_connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def _check_connection(self):
        if not self.is_connected:
            raise Exception("[FakeClient] 未连接")

    # --- API 方法 ---

    def get_status(self):
        self._check_connection()
        print("[FakeClient] 获取设备状态")
        return self.system_state

    def control_s_marker(self, show: bool):
        self._check_connection()
        print(f"[FakeClient] 控制S标记: {'显示' if show else '隐藏'}")
        return True

    def control_ruler(self, show: bool):
        self._check_connection()
        print(f"[FakeClient] 控制标尺: {'显示' if show else '隐藏'}")
        return True

    def get_disk_space(self):
        self._check_connection()
        print("[FakeClient] 获取硬盘容量")
        return 500  # GB

    def set_scan_mode(self, mode: ScanMode):
        self._check_connection()
        # ScanMode.FUNDAMENTAL -> '基波', ScanMode.HARMONIC -> '谐波'
        mode_val = '基波'
        if mode == ScanMode.HARMONIC:
             mode_val = '谐波'
        self.system_state['scan_mode'] = mode_val
        print(f"[FakeClient] 设置扫描模式: {mode} -> {mode_val}")
        return True

    def get_patient_name(self):
        self._check_connection()
        print("[FakeClient] 获取患者姓名")
        return "Fake Patient"

    def get_patient_id(self):
        self._check_connection()
        print("[FakeClient] 获取患者ID")
        return "FAKE12345"

    def get_patient_scan_time(self):
        self._check_connection()
        print("[FakeClient] 获取扫描时间")
        return datetime.now()

    def set_scan_depth(self, depth_mm: float):
        self._check_connection()
        self.system_state['depth_d_mm'] = depth_mm
        # 假设 d_g 格式为 "D=150, G=80"
        self.system_state['d_g'] = f"D={self.system_state['depth_d_mm']}, G={self.system_state['gain_gn']}"
        print(f"[FakeClient] 设置扫描深度: {depth_mm} mm")
        return True

    def set_image_type(self, image_type: ImageType):
        self._check_connection()
        print(f"[FakeClient] 设置图像类型: {image_type}")
        return True

    def start_image_acquisition(self, params: str):
        self._check_connection()
        print(f"[FakeClient] 开始采集: {params}")
        return True

    def abort_image_acquisition(self):
        self._check_connection()
        print("[FakeClient] 中止采集")
        return True

    def end_image_acquisition(self):
        self._check_connection()
        print("[FakeClient] 结束采集")
        return True

    def control_freeze(self, freeze: bool):
        self._check_connection()
        val = '冻结' if freeze else '解冻'
        self.system_state['is_frozen'] = val
        print(f"[FakeClient] 图片冻结控制: {val}")
        return True

    def save_and_send_image(self):
        self._check_connection()
        print("[FakeClient] 保存并发送图像")
        return {"success": True, "dicom_servers": 1}

    def new_patient(self):
        self._check_connection()
        print("[FakeClient] 新建患者")
        return True

    def get_dicom_send_status(self, patient_id: str):
        self._check_connection()
        print(f"[FakeClient] 获取DICOM发送状态 (ID: {patient_id})")
        return {
            "exists": True,
            "success_count": 10,
            "fail_count": 0,
            "id_returned": patient_id
        }

    def get_zoom_factor(self):
        self._check_connection()
        print("[FakeClient] 获取放大倍数")
        return 1.2

    def set_b_gain(self, gain: int):
        self._check_connection()
        self.system_state['gain_gn'] = gain
        # 假设 d_g 格式为 "D=150, G=80"
        self.system_state['d_g'] = f"D={self.system_state['depth_d_mm']}, G={self.system_state['gain_gn']}"
        print(f"[FakeClient] 设置增益: {gain}")
        return True

    def robot_start(self,bodypart:str):
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