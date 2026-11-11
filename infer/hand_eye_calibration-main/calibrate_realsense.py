"""全局 RealSense 相机标定入口：采集后求解 Camera → Base。"""
from calibration import run

if __name__ == '__main__':
    run('realsense')
