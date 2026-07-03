# ==============================================================================
# 文件: redis_client_demo.py
# 描述: 演示如何通过Redis发送命令给 redis_service.py 来控制超声。
# ==============================================================================
import redis
import json
import uuid
import time

class UltrasoundRedisMockClient:
    """
    这是一个包装类，用于方便地向Redis服务发送命令。
    """
    def __init__(self, redis_host='localhost', redis_port=6379):
        self.r = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)

    def send_raw_command(self, command_string):
        """发送原始命令字符串，例如 'func1(), func2()'"""
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

    res = client.send_raw_command("set_scan_depth(depth_mm=6)")
    print(res)
    print(f"<<< 响应: {res.get('data')}\n")
    
    # print(">>> 测试混合调用:")
    # res = client.send_raw_command("control_ruler(show=True), get_disk_space()")
    # print(f"<<< 响应: {res.get('data')}\n")
    # print(res)
    
    print("--- 演示结束 ---")

