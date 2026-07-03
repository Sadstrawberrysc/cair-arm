#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import sounddevice as sd
import time
import sys

# 配置
TONE_FREQ = 440.0
TONE_DURATION = 0.8   # 每次播放时长（秒）
REPEATS = 1           # 每个设备播放次数（通常 1 次即可）
FADE_MS = 20          # 淡入淡出毫秒
MAX_DEVICES_TO_TRY = None  # None = 尝试所有输出设备；设置整数可限制尝试数量

def make_tone(sr, duration=TONE_DURATION, freq=TONE_FREQ, amplitude=0.3, fade_ms=FADE_MS):
    t = np.linspace(0, duration, int(sr * duration), endpoint=False)
    tone = np.sin(2 * np.pi * freq * t)
    fade_samples = max(1, int(sr * fade_ms / 1000.0))
    window = np.ones_like(tone)
    window[:fade_samples] = np.linspace(0.0, 1.0, fade_samples)
    window[-fade_samples:] = np.linspace(1.0, 0.0, fade_samples)
    return (amplitude * tone * window).astype('float32')

def list_output_devices():
    try:
        devs = sd.query_devices()
    except Exception as e:
        print("无法查询设备:", e)
        return []
    outs = []
    print("\n=== 输出设备列表 ===")
    for i, d in enumerate(devs):
        out_ch = d.get("max_output_channels", 0)
        name = d.get("name", "<unknown>")
        dsr = d.get("default_samplerate", "N/A")
        if out_ch > 0:
            outs.append((i, name, int(dsr) if dsr != "N/A" else 44100, out_ch))
            print(f"[{i}] {name}   out_channels={out_ch}  default_sr={dsr}")
    if not outs:
        print("未找到任何具有输出通道的设备。")
    return outs

def try_play_on_device(dev_idx, sr, channels=1):
    tone = make_tone(sr)
    try:
        # 尝试用 OutputStream 打开（validate）
        with sd.OutputStream(device=dev_idx, samplerate=sr, channels=channels, dtype='float32'):
            pass
    except Exception as e:
        return False, f"无法打开 OutputStream (channels={channels}): {repr(e)}"

    # 尝试播放（使用 sd.play）
    try:
        for _ in range(REPEATS):
            sd.play(tone, samplerate=sr, device=dev_idx)
            sd.wait()  # 等待播放完成
            time.sleep(0.05)
        return True, "播放成功 (no exception)"
    except Exception as e:
        return False, f"播放失败: {repr(e)}"

def main():
    print("开始遍历所有输出设备并播放测试音（440Hz）。\n")
    outs = list_output_devices()
    if not outs:
        sys.exit(1)

    if MAX_DEVICES_TO_TRY is not None:
        outs = outs[:MAX_DEVICES_TO_TRY]

    results = []  # (idx, name, sr, channels, open_success, play_success, message, user_heard_bool)

    print("\n说明：每次播放后你可以按回车并输入 y 表示“我听到声音”，否则直接回车表示未听到。")
    print("如果你想自动化检测并不人工确认，运行脚本并在提示处按回车即可（不会记录人工确认）。\n")

    for idx, name, dsr, out_ch in outs:
        print("------------------------------------------------------------")
        print(f"设备 {idx}: {name}  (默认采样率={dsr}, 输出通道={out_ch})")
        open_ok = False
        play_ok = False
        messages = []

        # 优先尝试单声道，如果设备只支持 2 通道也会尝试 channels=2
        channels_to_try = [1]
        if out_ch >= 2:
            channels_to_try.append(2)

        for ch in channels_to_try:
            ok, msg = try_play_on_device(idx, dsr, channels=ch)
            messages.append(f"channels={ch}: {msg}")
            if ok:
                open_ok = True
                play_ok = True
                used_channels = ch
                break
            else:
                # 如果打开失败或播放失败，继续尝试下一个 ch
                continue

        if not open_ok:
            # 可能是采样率不兼容，尝试 44100 采样率（常见回退）
            if dsr != 44100:
                print("  尝试使用 44100 作为采样率重试...")
                for ch in channels_to_try:
                    ok, msg = try_play_on_device(idx, 44100, channels=ch)
                    messages.append(f"sr=44100,ch={ch}: {msg}")
                    if ok:
                        open_ok = True
                        play_ok = True
                        used_channels = ch
                        dsr = 44100
                        break

        # 打印尝试结果
        for m in messages:
            print("  -", m)

        user_heard = None
        # 交互确认步骤（可跳过）
        try:
            inp = input("你听到声音了吗？(y/N, 直接回车视为 N；输入 q 退出脚本): ").strip().lower()
            if inp == 'q':
                print("用户中断。")
                break
            user_heard = (inp == 'y')
        except KeyboardInterrupt:
            print("\n用户键盘中断，退出。")
            break

        results.append((idx, name, dsr, out_ch, open_ok, play_ok, " | ".join(messages), user_heard))

    # 汇总结果
    print("\n\n================ 遍历汇总 ================\n")
    heard_any = False
    for r in results:
        idx, name, dsr, out_ch, open_ok, play_ok, msg, user_heard = r
        status = []
        status.append("open_ok" if open_ok else "open_fail")
        status.append("play_ok" if play_ok else "play_fail")
        heard = "heard=Y" if user_heard else ("heard=N" if user_heard is not None else "heard=?")
        print(f"[{idx}] {name}  sr={dsr} out_ch={out_ch}  -> {' '.join(status)}  {heard}")
        if user_heard:
            heard_any = True

    if heard_any:
        print("\n你确认至少有一个设备播放出了声音。请把你确认听到的设备索引记录下来，例如 device=3，用于后续播放。")
    else:
        print("\n没有设备被人工确认听到。返回的 open/play 异常信息可用于进一步排查（驱动/路由/权限等）。")
    print("\n脚本结束。")

if __name__ == "__main__":
    main()
