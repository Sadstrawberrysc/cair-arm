import redis
import json
import uuid
import time

class UltrasoundRedisMockClient:
    def __init__(self, host='localhost', port=6379):
        self.r = redis.Redis(host=host, port=port, decode_responses=True)

    def send_raw_command(self, command_string):
        req_id = str(uuid.uuid4())
        payload = {
            "command": command_string,
            "id": req_id
        }
        
        try:
            self.r.rpush("sonoscape:cmd", json.dumps(payload))
            print(f">>> 发送命令串: {command_string}")
        except redis.ConnectionError:
            return {"status": "error", "error": "无法连接到Redis"}
            
        return self._wait_for_response(req_id)

    def _wait_for_response(self, req_id):
        result_key = f"sonoscape:result:{req_id}"
        timeout = 5.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            resp = self.r.get(result_key)
            if resp:
                self.r.delete(result_key)
                return json.loads(resp)
            time.sleep(0.05)
            
        return {"status": "error", "error": "等待响应超时"}

    def call_method(self, method_name, **kwargs):
        """(兼容旧方法) 发送调用请求并等待结果"""
        # 为了演示新的字符串解析能力，我们将 call_method 也转换为字符串形式发送
        # 构建参数字符串
        args_parts = []
        for k, v in kwargs.items():
            if isinstance(v, str):
                # 检查是否是ScanMode/ImageType的字符串表示，如果是则不加引号
                if "ScanMode." in v or "ImageType." in v:
                    args_parts.append(f"{k}={v}")
                else:
                    args_parts.append(f"{k}='{v}'")
            else:
                args_parts.append(f"{k}={v}")
        
        args_str = ", ".join(args_parts)
        command_str = f"{method_name}({args_str})"
        return self.send_raw_command(command_str)

if __name__ == "__main__":
    client = UltrasoundRedisMockClient()

    res = client.send_raw_command("control_freeze(freeze=True)")
    print(f"<<< 响应: {res.get('data')}\n")

    res = client.send_raw_command("set_b_gain(gain=150)")
    print(f"<<< 响应: {res.get('data')}\n")

    res = client.send_raw_command("set_scan_depth(depth_mm=20)")
    print(f"<<< 响应: {res.get('data')}\n")
    # res = client.send_raw_command("robot_start(bodypart='thyroid')")
    
