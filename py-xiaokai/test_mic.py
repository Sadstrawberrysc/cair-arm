#!/usr/bin/env python3
"""
测试PyAudio录音功能
"""
import pyaudio
import numpy as np
import time
import wave
import sys
import ctypes
from contextlib import contextmanager

# 定义错误处理上下文管理器以抑制ALSA错误
# ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
# def py_error_handler(filename, line, function, err, fmt):
#     pass
# c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
# @contextmanager
# def no_alsa_err():
#     asound = ctypes.cdll.LoadLibrary('libasound.so.2')
#     asound.snd_lib_error_set_handler(c_error_handler)
#     try:
#         yield
#     finally:
#         asound.snd_lib_error_set_handler(None)

def test_microphone():
    """测试麦克风是否正常工作"""
    try:
        # 初始化PyAudio
        # with no_alsa_err():
        p = pyaudio.PyAudio()
        
        print("PyAudio初始化成功")

        # 列出音频设备
        print("\n=== 音频输入设备列表 ===")
        input_devices = []
        default_input_index = -1
        
        try:
            default_info = p.get_default_input_device_info()
            default_input_index = default_info['index']
            print(f"默认输入设备索引: {default_input_index} ({default_info['name']})")
        except:
            print("无法获取默认输入设备信息")

        for i in range(p.get_device_count()):
            try:
                info = p.get_device_info_by_index(i)
                if info['maxInputChannels'] > 0:
                    input_devices.append(info)
                    mark = "*" if i == default_input_index else " "
                    print(f"{mark} 设备 {i}: {info['name']} (输入通道: {info['maxInputChannels']}, 采样率: {int(info['defaultSampleRate'])})")
            except Exception as e:
                print(f"  设备 {i}: 获取信息失败 - {e}")

        if not input_devices:
            print("没有找到输入设备！")
            return False

        # 选择要测试的设备
        devices_to_test = []
        if default_input_index >= 0:
             for info in input_devices:
                 if info['index'] == default_input_index:
                     devices_to_test.append(info)
                     break
        
        for info in input_devices:
            if info['index'] != default_input_index:
                devices_to_test.append(info)

        print("\n=== 开始录音测试 ===")
        print("将对每个设备录音3秒，然后回放...")

        # sample_rate = 44100 # 大多数设备支持
        # sample_rate = 48000 
        chunk = 1024
        record_seconds = 3

        for info in devices_to_test:
            device_index = info['index']
            # 使用设备建议的采样率，如果获取失败则默认44100
            current_sample_rate = int(info.get('defaultSampleRate', 44100))
            
            print(f"\n--------------------------------------------------")
            print(f"正在测试输入设备 {device_index}: {info['name']} (使用采样率: {current_sample_rate})...")
            
            try:
                # 尝试打开流
                stream = p.open(format=pyaudio.paInt16,
                                channels=1,
                                rate=current_sample_rate,
                                input=True,
                                input_device_index=device_index,
                                frames_per_buffer=chunk)

                print(f"* 正在录音 {record_seconds} 秒... (请说话)")
                frames = []

                for _ in range(0, int(current_sample_rate / chunk * record_seconds)):
                    data = stream.read(chunk, exception_on_overflow=False)
                    frames.append(data)

                print("* 录音结束")
                stream.stop_stream()
                stream.close()

                # 计算音量验证是否有声音输入
                audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
                volume = np.abs(audio_data).mean()
                print(f"* 平均音量: {volume:.2f}")

                if volume < 10:
                    print("! 警告: 录到的声音非常小，可能麦克风没有工作或静音了")

                # 保存录音文件
                filename = f"test_mic_{device_index}.wav"
                print(f"* 正在保存录音到 {filename} ...")
                try:
                    wf = wave.open(filename, 'wb')
                    wf.setnchannels(1)
                    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
                    wf.setframerate(current_sample_rate)
                    wf.writeframes(b''.join(frames))
                    wf.close()
                    print(f"* 文件保存成功，请在外部播放器中检查 {filename}")
                except Exception as save_err:
                    print(f"保存文件失败: {save_err}")
                
                # 回放
                print("* 正在回放录到的声音...")
                try:
                    # 使用与录音相同的采样率进行回放
                    playback_stream = p.open(format=pyaudio.paInt16,
                                            channels=1,
                                            rate=current_sample_rate,
                                            output=True,
                                            frames_per_buffer=chunk)
                    
                    playback_stream.write(b''.join(frames))
                    playback_stream.stop_stream()
                    playback_stream.close()
                    print("* 回放结束")
                except Exception as playback_err:
                    print(f"回放失败 (不用担心，重点是录音已经完成): {playback_err}")

                choice = input("是否听到刚录制的声音? (y/n/q退出): ").strip().lower()
                if choice == 'y':
                    print(f"恭喜! 设备 {device_index} 工作正常。")
                    # break # 如果只需找一个，可以break
                elif choice == 'q':
                    break

            except Exception as e:
                print(f"设备 {device_index} 测试失败: {e}")

        p.terminate()
        print("\nPyAudio录音测试结束")
        return True

    except Exception as e:
        print(f"PyAudio总体测试失败: {e}")
        return False

if __name__ == "__main__":
    test_microphone()
