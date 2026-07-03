import time
import json
import datetime
import os
from sonoscape_client import UltrasoundClient

# 配置参数
ULTRASOUND_HOST_IP = "192.168.1.100"
ULTRASOUND_PORT = 6061
TARGET_HZ = 1000
BATCH_SIZE = 100  # 每采集 500 条数据写入一次硬盘 (约每5秒写一次)

def get_log_filename():
    """生成唯一的文件名"""
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"./log/device_log_{timestamp_str}.jsonl" # 注意后缀改为 .jsonl

def flush_to_disk(filename, buffer_data):
    """将缓冲区数据追加写入文件"""
    if not buffer_data:
        return
    
    try:
        with open(filename, 'a', encoding='utf-8') as f:
            for record in buffer_data:
                # 将字典转换为 json 字符串并写入，末尾加换行符
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
    except Exception as e:
        print(f"\n!!! 写入硬盘失败: {e}")

def record_data_loop(host, port):
    interval = 1.0 / TARGET_HZ
    
    # 1. 在循环开始前就确定文件名
    filename = get_log_filename()
    abs_path = os.path.abspath(filename)
    
    # 内存缓冲区
    data_buffer = []
    total_count = 0
    
    print(f"--- 开始连接设备 {host}:{port} ---")
    print(f"--- 数据将实时写入: {filename} ---")
    print(f"--- 缓存策略: 每 {BATCH_SIZE} 条写入一次 ---")

    try:
        with UltrasoundClient(host, port) as client:
            print("设备连接成功，开始采集...")
            
            while True:
                loop_start_time = time.perf_counter()

                # --- 数据采集区域 ---
                local_time = datetime.datetime.now().isoformat()
                
                try:
                    device_status = client.get_status()
                except Exception:
                    device_status = None # 简化错误处理，保持数据整洁

                # try:
                #     scan_time = client.get_patient_scan_time()
                # except Exception:
                #     scan_time = None

                record = {
                    "timestamp": local_time,
                    # "scan_time": scan_time,
                    "device_status": device_status
                }
                # -------------------

                # 加入缓冲区
                data_buffer.append(record)
                total_count += 1

                # 检查是否达到写入阈值
                if len(data_buffer) >= BATCH_SIZE:
                    flush_to_disk(filename, data_buffer)
                    data_buffer.clear() # 清空缓冲区，释放内存
                    print(f"\r已保存记录: {total_count} 条...", end="")

                # 频率控制
                elapsed = time.perf_counter() - loop_start_time
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n!!! 用户停止采集 !!!")
    except Exception as e:
        print(f"\n!!! 发生错误: {e}")
    finally:
        # 退出前，将缓冲区剩余的数据写入
        if data_buffer:
            print(f"\n正在保存剩余的 {len(data_buffer)} 条数据...")
            flush_to_disk(filename, data_buffer)
        print(f"采集结束。总计保存: {total_count} 条。")
        print(f"文件路径: {abs_path}")

if __name__ == "__main__":
    record_data_loop(ULTRASOUND_HOST_IP, ULTRASOUND_PORT)
