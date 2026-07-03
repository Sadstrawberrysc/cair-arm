import redis
import json
import time
import socket
import re
from datetime import datetime

try:
    from sonoscape_client import UltrasoundClient as RealClient, ScanMode, ImageType
    from sonoscape_fake import UltrasoundClient as FakeClient
except ImportError as e:
    print(f"Error importing modules: {e}")
    # 为了防止ImportError导致脚本直接退出无法演示，定义mock类
    class RealClient:
        def __init__(self, *args, **kwargs): pass
        def connect(self): pass
        def disconnect(self): pass
    
    class FakeClient:
        def __init__(self, *args, **kwargs): pass
        def connect(self): pass
        def disconnect(self): pass
        def get_status(self): return {}
    
    # ScanMode/ImageType 定义mock
    class ScanMode:
        FUNDAMENTAL = 'FUNDAMENTAL'
        HARMONIC = 'HARMONIC'
    class ImageType:
        POST_PROCESSED = 'POST_PROCESSED'
        PRE_PROCESSED = 'PRE_PROCESSED'
    
    print("Warning: Using fallback mocks due to import error.")

def get_client(host, port):
    try:
        print(f"尝试连接真实设备 {host}:{port} ...")
        # 短超时，避免阻塞太久
        client = RealClient(host, port, timeout=2)
        client.connect()
        print(">>> 成功连接到 真实 设备。")
        return client
    except Exception as e:
        print(f"!!! 连接真实设备失败: {e}")
        print(">>> 切换到 模拟 (Fake) 模式。")
        client = FakeClient(host, port)
        client.connect()
        return client

def convert_args(kwargs):
    """(Legacy) 转换参数中的特定字符串为枚举类型。"""
    if "mode" in kwargs:
        val = kwargs["mode"]
        if isinstance(val, str):
            if "FUNDAMENTAL" in val:
                kwargs["mode"] = ScanMode.FUNDAMENTAL
            elif "HARMONIC" in val:
                kwargs["mode"] = ScanMode.HARMONIC
    
    if "image_type" in kwargs:
        val = kwargs["image_type"]
        if isinstance(val, str):
            if "POST" in val:
                kwargs["image_type"] = ImageType.POST_PROCESSED
            elif "PRE" in val:
                kwargs["image_type"] = ImageType.PRE_PROCESSED
    
    return kwargs

def parse_value(v):
    """解析字符串值为正确的Python类型"""
    if not isinstance(v, str):
        return v
    v = v.strip()
    if v == 'True': return True
    if v == 'False': return False
    if v.isdigit(): return int(v)
    try:
        return float(v)
    except ValueError:
        pass
    
    if v.startswith("'") and v.endswith("'"): return v[1:-1]
    if v.startswith('"') and v.endswith('"'): return v[1:-1]
    
    if "ScanMode." in v:
        if "HARMONIC" in v: return ScanMode.HARMONIC
        if "FUNDAMENTAL" in v: return ScanMode.FUNDAMENTAL
    
    if "ImageType." in v:
        if "POST" in v: return ImageType.POST_PROCESSED
        if "PRE" in v: return ImageType.PRE_PROCESSED
        
    return v

def execute_command_string(client, command_string):
    """
    解析并执行命令字符串，例如 "func1(a=1), func2(b=ScanMode.X)"
    返回执行日志列表
    """
    # 匹配 format: name(args)
    # 简单的正则，可能不支持嵌套括号
    calls = re.findall(r'(\w+)\((.*?)\)', command_string)
    
    executed_logs = []
    
    for func_name, args_str in calls:
        if not hasattr(client, func_name):
            print(f"Error: 方法 '{func_name}' 不存在")
            executed_logs.append({"tool": f"{func_name}({args_str})", "result": f"Error: Method not found"})
            continue
        
        args_str = args_str.strip()
        kwargs = {}
        if args_str:
            # 简单按逗号分割参数
            # 注意：如果字符串参数中包含逗号，这里会出问题。假设输入是干净的。
            # 为了更好的鲁棒性，如果有嵌套括号或逗号在字符串里，这里会fail。
            parts = args_str.split(',')
            for part in parts:
                if '=' in part:
                    k, v = part.split('=', 1)
                    kwargs[k.strip()] = parse_value(v)
        
        try:
            func = getattr(client, func_name)
            result = func(**kwargs)
            
            # 构造工具调用记录
            # 保持原始调用字符串的形式方便记录
            call_signature = f"{func_name}({args_str})"
            executed_logs.append({"tool": call_signature, "result": result})
            print(f"-> 执行 {call_signature}: {result}")
            
        except Exception as e:
            print(f"!!! 执行 {func_name} 出错: {e}")
            executed_logs.append({"tool": f"{func_name}({args_str})", "result": f"Error: {str(e)}"})
            
    return executed_logs

def run_redis_service(host, port, redis_host="localhost", redis_port=6379):
    try:
        r = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        r.ping() # 检测Redis连接
    except Exception as e:
        print(f"无法连接Redis: {e}")
        return

    client = get_client(host, port)
    
    # 清空旧命令
    r.delete("sonoscape:cmd")
    print("\n------------------------------------------------")
    print("Redis 服务已启动。监听队列: 'sonoscape:cmd' ...")
    print("------------------------------------------------")
    
    try:
        while True:
            # 阻塞读取命令
            # 返回 tuple: (key, value)
            res = r.blpop("sonoscape:cmd")
            if not res:
                continue
            
            _, message = res
            req_id = None
            
            try:
                # 尝试解析JSON
                data = None
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    data = {"command": message}

                executed_logs = []
                
                if isinstance(data, dict):
                    req_id = data.get("id")
                    
                    if "command" in data:
                        # 新模式：直接传递命令字符串
                        command_str = data["command"]
                        executed_logs = execute_command_string(client, command_str)
                    
                    elif "cmd" in data:
                        # 兼容旧模式：cmd + kwargs
                        func_name = data.get("cmd")
                        kwargs = data.get("kwargs", {})
                        
                        kwargs = convert_args(kwargs)
                        if hasattr(client, func_name):
                            func = getattr(client, func_name)
                            try:
                                result = func(**kwargs)
                            except Exception as e:
                                result = f"Error: {str(e)}"
                            
                            # 构造 args string for log
                            args_repr = ", ".join([f"{k}={v}" for k,v in kwargs.items()])
                            executed_logs.append({"tool": f"{func_name}({args_repr})", "result": result})
                            print(f"-> 执行 {func_name}: {result}")
                        else:
                             executed_logs.append({"tool": func_name, "result": "Error: Method not found"})
                
                # 获取最新状态
                current_status = client.get_status()
                
                # 构造最终返回字符串
                # [当前状态: {status}, 已执行: [logs]]
                final_resp_str = f"[当前状态: {json.dumps(current_status, ensure_ascii=False)}, 已执行: {json.dumps(executed_logs, ensure_ascii=False)}]"
                # final_resp_str = f"[当前状态: {json.dumps(current_status, ensure_ascii=False)}, 已执行: []]"

                
                print(f"-> 返回数据: {final_resp_str[:100]}...")

                # 返回结果
                if req_id:
                    # 依然包裹在JSON中返回，status为success，data为那个大字符串
                    resp = {"status": "success", "data": final_resp_str}
                    r.set(f"sonoscape:result:{req_id}", json.dumps(resp, ensure_ascii=False))
                    r.expire(f"sonoscape:result:{req_id}", 60)
                    
            except Exception as e:
                print(f"!!! 执行出错: {e}")
                if req_id:
                    r.set(f"sonoscape:result:{req_id}", json.dumps({"status": "error", "error": str(e)}))

    except KeyboardInterrupt:
        print("\n正在停止服务...")
    finally:
        client.disconnect()
        print("服务已停止。")

if __name__ == "__main__":
    # 配置
    US_HOST = "192.168.1.100"
    US_PORT = 6061
    
    run_redis_service(US_HOST, US_PORT)
