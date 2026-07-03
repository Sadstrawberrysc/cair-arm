#!/usr/bin/env python3
"""
测试PyAudio播放功能
"""
import pyaudio
import numpy as np
import time
import sys
import os
from contextlib import contextmanager
import ctypes

def test_pyaudio():
    """测试PyAudio是否正常工作"""
    try:
        # 初始化PyAudio，使用上下文管理器抑制错误
        # with no_alsa_err():
        p = pyaudio.PyAudio()
        print("PyAudio初始化成功")

        # 列出音频设备
        print("\n=== 音频设备列表 ===")
        output_devices = []
        default_output_index = -1
        
        try:
            default_info = p.get_default_output_device_info()
            default_output_index = default_info['index']
            print(f"默认输出设备索引: {default_output_index} ({default_info['name']})")
        except:
            print("无法获取默认输出设备信息")

        for i in range(p.get_device_count()):
            try:
                info = p.get_device_info_by_index(i)
                if info['maxOutputChannels'] > 0:
                    output_devices.append(info)
                    mark = "*" if i == default_output_index else " "
                    print(f"{mark} 设备 {i}: {info['name']} (输出通道: {info['maxOutputChannels']}, 采样率: {int(info['defaultSampleRate'])})")
            except Exception as e:
                print(f"  设备 {i}: 获取信息失败 - {e}")

        # 生成测试音频 - 440Hz正弦波
        sample_rate = 48000
        duration = 0.5 # 每个设备播放0.5秒
        frequency = 440

        samples = int(sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        wave = np.sin(2 * np.pi * frequency * t)
        # 转换为16bit PCM
        audio_data = (wave * 32767).astype(np.int16)
        
        print("\n=== 开始播放测试 ===")
        
        # 优先测试默认设备
        devices_to_test = []
        # 先放默认设备
        if default_output_index >= 0:
             # Find the info for default
             for info in output_devices:
                 if info['index'] == default_output_index:
                     devices_to_test.append(info)
                     break
        
        # 再放其他设备
        for info in output_devices:
            if info['index'] != default_output_index:
                devices_to_test.append(info)

        if not devices_to_test:
            print("没有找到输出设备！")
            return False

        for info in devices_to_test:
            device_index = info['index']
            print(f"\n正在尝试设备 {device_index}: {info['name']} ...")
            
            try:
                stream = p.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=sample_rate,
                    output=True,
                    output_device_index=device_index,
                    frames_per_buffer=1024,
                )

                # 分块播放
                chunk_size = 1024
                for i in range(0, len(audio_data), chunk_size):
                    chunk = audio_data[i : i + chunk_size]
                    if len(chunk) < chunk_size:
                         chunk = np.concatenate([chunk, np.zeros(chunk_size - len(chunk), dtype=np.int16)])
                    stream.write(chunk.tobytes())

                stream.stop_stream()
                stream.close()
                print(f"设备 {device_index} 播放完成")
                input("按回车继续测试下一个设备...")
                time.sleep(0.2) 
            except Exception as e:
                print(f"设备 {device_index} 播放失败: {e}")

        p.terminate()

        print("\nPyAudio全流程测试完成")
        return True

    except Exception as e:
        print(f"PyAudio总体测试失败: {e}")
        return False


if __name__ == "__main__":
    success = test_pyaudio()
    exit(0 if success else 1)
