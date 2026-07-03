# ==============================================================================
# 文件: main.py
# 描述: 演示如何使用 UltrasoundClient 与真实的超声设备通信。
# ==============================================================================
import time

from sonoscape_client import *

def run_client_demo(host, port):
    """运行客户端演示，调用所有API功能。"""
    print("\n--- 开始客户端演示 ---\n")
    try:
        # 使用 'with' 语句确保连接在使用后自动关闭
        with UltrasoundClient(host, port) as client:
            print(f"客户端已成功连接到 {host}:{port}。")

            # # 1. 获取设备状态
            # print("\n1. 获取设备状态...")
            # status = client.get_status()
            # print(status)
            # print(f"   -> 状态: 扫描模式={status['scan_mode']}, 冻结={status['is_frozen']}")

            # # 2. 控制S标记
            # print("\n2. 控制S标记...")
            # client.control_s_marker(show=True)
            # print("   -> 显示S标记: 成功")
            # time.sleep(3) # 等待设备响应
            # client.control_s_marker(show=False)
            # print("   -> 隐藏S标记: 成功")
            
            # # 3. 控制图像标尺
            # print("\n3. 控制图像标尺...")
            # client.control_ruler(show=True)
            # print("   -> 显示标尺: 成功")
            # time.sleep(3) # 等待设备响应
            # client.control_ruler(show=False)
            # print("   -> 隐藏标尺: 成功")

            # # 4. 获取硬盘容量
            # print("\n4. 获取硬盘容量...")
            # space = client.get_disk_space()
            # print(f"   -> 剩余空间: {space} GB")
            
            # # 5. 设置扫描模式 PS: 如果当前是基频模式，再设置为基频模式会报错，同理谐波模式
            # print("\n5. 设置扫描模式...")
            # client.set_scan_mode(ScanMode.FUNDAMENTAL)
            # print(f"   -> 设置为基频模式 ({ScanMode.FUNDAMENTAL.name}): 成功")
            # time.sleep(3) # 等待设备响应
            # client.set_scan_mode(ScanMode.HARMONIC)
            # print(f"   -> 设置为谐波模式 ({ScanMode.HARMONIC.name}): 成功")


            # # 6. 获取患者姓名
            # print("\n6. 获取患者姓名...")
            # name = client.get_patient_name()
            # print(f"   -> 患者姓名: {name}")

            # # 7. 获取患者ID
            # print("\n7. 获取患者ID...")
            # pid = client.get_patient_id()
            # print(f"   -> 患者ID: {pid}")

            # # 8. 获取扫描时间
            # print("\n8. 获取扫描时间...")
            # scan_time = client.get_patient_scan_time()
            # print(f"   -> 扫描时间: {scan_time}")

            # 9. 设置扫描深度
            # print("\n9. 设置扫描深度...")
            # client.set_scan_depth(35)
            # print("   -> 设置深度为 3.5cm: 成功")

            # # 10. 设置图像类型
            # print("\n10. 设置图像类型...")
            # client.set_image_type(ImageType.POST_PROCESSED)
            # print(f"   -> 设置为后处理图像 ({ImageType.POST_PROCESSED.name}): 成功")
            # time.sleep(3) # 等待设备响应
            # client.set_image_type(ImageType.PRE_PROCESSED)
            # print(f"   -> 设置为前处理图像 ({ImageType.PRE_PROCESSED.name}): 成功")

            # # 11. 采集控制 L/R, H/T: H 代表 Harmonic（谐波）
            # print("\n11. 采集控制...")
            # client.start_image_acquisition("LH")
            # print("   -> 开始采集: 成功")
            # time.sleep(5) # 模拟采集
            # # client.end_image_acquisition()
            # # print("   -> 结束采集: 成功")
            # client.abort_image_acquisition()
            # print("   -> 中止采集: 成功")

            # # 12. 冻结/解冻
            # print("\n12. 冻结/解冻...")
            # client.control_freeze(freeze=True)
            # print("   -> 冻结图像: 成功")
            # time.sleep(3)
            # client.control_freeze(freeze=False)
            # print("   -> 解冻图像: 成功")

            # # 13. 保存和发送
            # print("\n13. 保存和发送...")
            # result = client.save_and_send_image()
            # print(f"   -> 保存和发送结果: {result}")

            # # 14. 新建患者
            # print("\n14. 新建患者...")
            # client.new_patient()
            # print("   -> 跳转新建患者界面: 成功")
            
            # # 15. 获取DICOM发送状态
            # print("\n15. 获取DICOM发送状态...")
            # if pid: # 仅当获取到ID时才查询
            #     dicom_status = client.get_dicom_send_status(pid)
            #     print(f"   -> ID '{pid}' 的DICOM状态: {dicom_status}")
            # else:
            #     print("   -> 无患者ID，跳过查询。")

            # # 16. 获取放大倍数
            # print("\n16. 获取放大倍数...")
            # zoom = client.get_zoom_factor()
            # print(f"   -> 放大倍数: {zoom}")

            # # 17. 设置B模式增益 目前存在问题
            # print("\n17. 设置B模式增益...")
            # client.set_b_gain(100)
            # print("   -> 设置增益为 150: 成功")

    except Exception as e:
        print(f"\n!!! 发生未知错误: {e}")

    print("\n--- 客户端演示结束 ---\n")


if __name__ == "__main__":
    ULTRASOUND_HOST_IP = "192.168.1.100"
    ULTRASOUND_PORT = 6061

    run_client_demo(ULTRASOUND_HOST_IP, ULTRASOUND_PORT)

