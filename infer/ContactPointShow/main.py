import redis
import json
import threading
import time
import numpy as np
import pyvista as pv

# --- 配置参数 ---
REDIS_HOST = 'localhost'
REDIS_PORT = 7777
REDIS_CHANNEL = 'sensor_data'

STL_FILE_PATH = "./model/Lprobe-show.STL" 
SCALING_FACTOR = 100

class ContactSensingApp:
    def __init__(self):
        # 共享数据：当前接触点坐标 [x, y, z]
        self.contact_point = np.array([0.0, 0.0, 0.0])
        self.running = True
        self.logic_flag = True

    def redis_worker_thread(self):
        """后台线程：连接 Redis 并订阅数据"""
        try:
            r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)
            pubsub = r.pubsub()
            pubsub.subscribe(REDIS_CHANNEL)
            print(f"Connected to Redis. Listening on channel: '{REDIS_CHANNEL}'")

            for message in pubsub.listen():
                if not self.running:
                    break
                
                if message['type'] == 'message':
                    raw_data = message['data']
                    try:
                        data_array = json.loads(raw_data)
                        if len(data_array) == 4:
                            self.process_logic(data_array)
                    except Exception as e:
                        print(f"Error processing data: {e}")

        except Exception as e:
            print(f"Redis Thread Error: {e}")

    def process_logic(self, data):
        d0, d1, d2, d3 = data
        if d3 == 1.0:
            self.logic_flag = False
        
        if self.logic_flag:
            if 0.180 < d2 < 0.188:
                self.contact_point = np.array([d0, d1, d2])
            else:
                if -7 < d3 < -1:
                    self.contact_point = np.array([
                        0.5 * d0,
                        0.5 * d1,
                        0.185 + 0.02 * d2
                    ])
                else:
                    self.contact_point = np.array([0.0, 0.0, 0.0])
        else:
            self.contact_point = np.array([d0, d1, d2])
            self.logic_flag = True

    def start_visualization(self):
        """主线程：PyVista 渲染循环"""
        
        try:
            mesh = pv.read(STL_FILE_PATH)
            mesh.scale([SCALING_FACTOR, SCALING_FACTOR, SCALING_FACTOR], inplace=True)
        except Exception:
            print("Warning: STL file not found. Using Cube.")
            mesh = pv.Cube()

        plotter = pv.Plotter(title="Realtime Contact Sensing (Redis)")
        plotter.add_mesh(mesh, color="gray", opacity=0.5, show_edges=False)
        plotter.add_axes()
        plotter.show_grid()

        sphere_mesh = pv.Sphere(radius=0.1, center=(0, 0, 0)) 
        
        self.sphere_actor = plotter.add_mesh(sphere_mesh, color="red")

        plotter.camera.position = (8.0, 0.0, 25.0)
        plotter.camera.focal_point = (-35.0, 0.0, -10.0)
        plotter.camera.up = (0.0, 0.0, -1.0)
        
        def update_scene(step):
            cp = self.contact_point
            new_pos = cp * SCALING_FACTOR
            self.sphere_actor.position = new_pos.tolist()
            
        # 启动 Redis 监听线程
        t = threading.Thread(target=self.redis_worker_thread, daemon=True)
        t.start()

        # 设置定时器，注意 duration 单位是毫秒
        plotter.add_timer_event(max_steps=100000000, duration=10, callback=update_scene)
        
        print("Visualization started...")
        plotter.show()
        
        self.running = False

if __name__ == "__main__":
    app = ContactSensingApp()
    app.start_visualization()