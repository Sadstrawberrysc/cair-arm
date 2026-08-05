# 远程相机监控记录

本文记录在远程主机 `10.21.11.40` 上搭建机械臂远程相机监控的过程，包括普通 UVC 相机的 `motion` 视频流方案，以及 Daheng 工业相机的 Galaxy SDK 安装和验证状态。

## 目标

- 相机连接在远程主机上。
- 远程主机运行 `motion`，将相机画面发布到 `localhost:8081`。
- 本地电脑通过 SSH 隧道访问远程视频流。
- 本地浏览器打开 `http://localhost:18081/` 查看画面。

## 当前 Daheng 相机状态

- 系统已识别 Daheng USB3 工业相机：`Daheng Imaging MER2-503-36U3M`。
- USB ID：`2ba2:4d55`。
- 相机序列号：`FCK25120545`。
- SDK 枚举名：`MER2-503-36U3M(FCK25120545)`。
- 已安装 Daheng Galaxy Linux x86 SDK：`Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251`。
- 已在 conda `camera` 环境安装 Galaxy Python SDK / `gxipy`：`2.6.2606.9081`。
- `camera` 环境 Python 版本：`3.8.16`。
- 已成功抓取单帧测试图像：`/tmp/daheng_test_frame.png`。
- 测试图像尺寸：`2448 x 2048`。

检查 USB 是否识别 Daheng 相机：

```bash
sudo lsusb | grep -i daheng
```

预期结果类似：

```text
Bus 004 Device 002: ID 2ba2:4d55 Daheng Imaging MER2-503-36U3M
```

使用 `gxipy` 枚举相机：

```bash
/home/cair-jacen/anaconda3/envs/camera/bin/python -c 'import gxipy as gx; dm=gx.DeviceManager(); n, info=dm.update_device_list(); print("devices", n); print(info)'
```

预期能看到：

```text
devices 1
MER2-503-36U3M(FCK25120545)
```

注意：Daheng 工业相机不是普通 UVC 摄像头，通常不会直接作为 `/dev/video0` 给 `motion` 使用。它需要通过 Galaxy SDK / `gxipy` 读取画面；若要浏览器远程监控，需要后续编写一个基于 `gxipy` 的 MJPEG/HTTP 视频流服务，或把 SDK 图像转接成虚拟 V4L2 设备。

## Daheng SDK 安装记录

官方下载页面：

```text
https://www.daheng-imaging.com/downloads/softwares/
```

已使用的安装包：

```text
Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251.zip
Galaxy_Linux_Python_2.6.2606.9081.zip
```

底层 SDK 安装方式：

```bash
unzip Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251.zip
cd Galaxy_Linux-x86_Gige-U3_32bits-64bits_2.6.2606.9251
sudo bash Galaxy_camera.run
```

安装器提示配置完整生效需要重启系统，或者至少重新插拔 Daheng USB3 相机。

Python SDK 安装方式：

```bash
unzip Galaxy_Linux_Python_2.6.2606.9081.zip
cd Galaxy_Linux_Python_2.6.2606.9081/api
sudo /home/cair-jacen/anaconda3/envs/camera/bin/python setup.py install
```

验证 `gxipy` 版本：

```bash
/home/cair-jacen/anaconda3/envs/camera/bin/python -c 'import gxipy; print(gxipy.__version__)'
```

预期结果：

```text
2.6.2606.9081
```

抓取一帧测试图像：

```bash
/home/cair-jacen/anaconda3/envs/camera/bin/python -c 'import gxipy as gx; from PIL import Image; dm=gx.DeviceManager(); n, info=dm.update_device_list(); print("devices", n); cam=dm.open_device_by_index(1); cam.TriggerMode.set(gx.GxSwitchEntry.OFF); cam.ExposureTime.set(10000); cam.Gain.set(10.0); cam.stream_on(); raw=cam.data_stream[0].get_image(); arr=raw.get_numpy_array() if raw else None; print("frame", raw.get_frame_id() if raw else None, "size", (raw.get_width(), raw.get_height()) if raw else None); Image.fromarray(arr, "L").save("/tmp/daheng_test_frame.png") if arr is not None else None; cam.stream_off(); cam.close_device()'
```

已验证结果：

```text
devices 1
frame 0 size (2448, 2048)
```

## 重新启动 Daheng 监控

Daheng 工业相机不能直接用 `motion` 读取。重新启动 Daheng 监控时，先停掉 `motion`，再启动 `gxipy` 的 MJPEG 服务。

以下命令在远程主机执行：

```bash
sudo systemctl stop motion
grep 'PORT =' ~/daheng_mjpeg_server.py
```

如果端口不是 `8081`，改成 `8081`：

```bash
sed -i 's/PORT = .*/PORT = 8081/' ~/daheng_mjpeg_server.py
```

启动 Daheng MJPEG 服务：

```bash
/home/cair-jacen/anaconda3/envs/camera/bin/python ~/daheng_mjpeg_server.py
```

看到下面输出，并且终端停住不返回，说明服务正在运行。这个远程终端不要关闭。

```text
Daheng MJPEG server: http://127.0.0.1:8081/
```

如果启动时报端口占用：

```text
OSError: [Errno 98] Address already in use
```

先确认占用 `8081` 的进程：

```bash
ss -ltnp | grep 8081
```

通常是 `motion` 还在运行，执行：

```bash
sudo systemctl stop motion
```

然后重新启动 Daheng 服务：

```bash
/home/cair-jacen/anaconda3/envs/camera/bin/python ~/daheng_mjpeg_server.py
```

远程主机另开一个终端测试视频流：

```bash
curl -v --max-time 5 http://localhost:8081/
```

如果看到类似内容，说明远程 Daheng 视频流正常：

```text
HTTP/1.0 200 OK
Content-Type: multipart/x-mixed-replace
```

以下命令在本地电脑执行：

```bash
ssh -N -L 18081:localhost:8081 cair-jacen@10.21.11.40
```

保持本地 SSH 隧道终端不要关闭，然后在本地浏览器打开：

```text
http://localhost:18081/
```

如果本地 `18081` 被占用，只换本地端口，远程端口仍然保持 `8081`：

```bash
ssh -N -L 18082:localhost:8081 cair-jacen@10.21.11.40
```

浏览器打开：

```text
http://localhost:18082/
```

## 远程主机：检查相机

以下命令在远程主机执行。

```bash
ls /dev/video*
```

正常情况下应看到类似：

```text
/dev/video0
```

查看相机支持的分辨率和帧率：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

如果没有 `v4l2-ctl`：

```bash
sudo apt update
sudo apt install v4l-utils
```

## 远程主机：安装 motion

```bash
sudo apt update
sudo apt install motion
```

## 远程主机：配置 motion

编辑配置文件：

```bash
sudo nano /etc/motion/motion.conf
```

关键配置建议如下：

```conf
daemon on
videodevice /dev/video0
width 1280
height 720
framerate 20

stream_localhost off
stream_port 8081

webcontrol_localhost off
webcontrol_port 8080

log_file /var/log/motion/motion.log
target_dir /var/lib/motion
```

也可以直接用命令修改常用项：

```bash
sudo cp /etc/motion/motion.conf /etc/motion/motion.conf.codex.bak
sudo sed -i 's/^daemon .*/daemon on/' /etc/motion/motion.conf
sudo sed -i 's/^videodevice .*/videodevice \/dev\/video0/' /etc/motion/motion.conf
sudo sed -i 's/^width .*/width 1280/' /etc/motion/motion.conf
sudo sed -i 's/^height .*/height 720/' /etc/motion/motion.conf
sudo sed -i 's/^framerate .*/framerate 20/' /etc/motion/motion.conf
sudo sed -i 's/^stream_localhost .*/stream_localhost off/' /etc/motion/motion.conf
sudo sed -i 's/^stream_port .*/stream_port 8081/' /etc/motion/motion.conf
sudo sed -i 's/^webcontrol_localhost .*/webcontrol_localhost off/' /etc/motion/motion.conf
sudo sed -i 's/^webcontrol_port .*/webcontrol_port 8080/' /etc/motion/motion.conf
```

当前 `/etc/motion/motion.conf` 已写入的关键项：

```conf
daemon on
videodevice /dev/video0
width 1280
height 720
framerate 20

stream_localhost off
stream_port 8081

webcontrol_localhost off
webcontrol_port 8080
```

已创建过备份：

```text
/etc/motion/motion.conf.codex.bak
```

## 远程主机：修复权限

如果出现下面错误：

```text
cannot create log file /var/log/motion/motion.log: Permission denied
```

执行：

```bash
sudo mkdir -p /var/log/motion /var/lib/motion
sudo touch /var/log/motion/motion.log
sudo chown -R motion:motion /var/log/motion /var/lib/motion
sudo chmod 755 /var/log/motion
sudo chmod 755 /var/lib/motion
sudo chmod 644 /var/log/motion/motion.log
```

## 远程主机：启动服务

```bash
sudo systemctl enable motion
sudo systemctl restart motion
sudo systemctl status motion --no-pager
```

如果状态显示 `active (running)`，说明服务已经启动。

## 远程主机：测试视频流

不要使用 `curl -I` 判断视频流是否正常，因为 `motion` 的 MJPEG 流可能不正确响应 `HEAD` 请求。

使用：

```bash
curl -v --max-time 5 http://localhost:8081/
```

如果看到类似内容，说明远程相机流正常：

```text
HTTP/1.1 200 OK
Content-Type: multipart/x-mixed-replace
```

后面出现二进制输出警告是正常的：

```text
Warning: Binary output can mess up your terminal.
```

## 本地电脑：建立 SSH 隧道

以下命令在本地电脑执行，不是在远程 SSH 终端里执行。

```bash
ssh -N -L 18081:localhost:8081 cair-jacen@10.21.11.40
```

这个终端会停住，没有输出是正常的。保持它不要关闭。

然后在本地电脑浏览器打开：

```text
http://localhost:18081/
```

如果本地 `18081` 被占用，换成：

```bash
ssh -N -L 18082:localhost:8081 cair-jacen@10.21.11.40
```

浏览器打开：

```text
http://localhost:18082/
```

## 同时转发控制页和视频流

如果也想访问 `motion` 控制页面，可以在本地电脑执行：

```bash
ssh -N -L 18080:localhost:8080 -L 18081:localhost:8081 cair-jacen@10.21.11.40
```

本地浏览器访问：

```text
http://localhost:18080/
http://localhost:18081/
```

## 调低分辨率并提高帧率

推荐配置：

```bash
sudo sed -i 's/^width .*/width 640/' /etc/motion/motion.conf
sudo sed -i 's/^height .*/height 480/' /etc/motion/motion.conf
sudo sed -i 's/^framerate .*/framerate 30/' /etc/motion/motion.conf
sudo systemctl restart motion
```

如果希望更低像素、更低延迟：

```bash
sudo sed -i 's/^width .*/width 640/' /etc/motion/motion.conf
sudo sed -i 's/^height .*/height 360/' /etc/motion/motion.conf
sudo sed -i 's/^framerate .*/framerate 30/' /etc/motion/motion.conf
sudo systemctl restart motion
```

如果相机支持 60 fps，可以尝试：

```bash
sudo sed -i 's/^width .*/width 424/' /etc/motion/motion.conf
sudo sed -i 's/^height .*/height 240/' /etc/motion/motion.conf
sudo sed -i 's/^framerate .*/framerate 60/' /etc/motion/motion.conf
sudo systemctl restart motion
```

修改后再次测试：

```bash
curl -v --max-time 5 http://localhost:8081/
```

## 常见问题排查

查看服务完整状态：

```bash
sudo systemctl status motion --no-pager -l
```

查看 `motion` 日志：

```bash
tail -120 /var/log/motion/motion.log
```

检查配置关键项：

```bash
grep -nE '^(daemon|videodevice|width|height|framerate|stream_localhost|stream_port|webcontrol_localhost|webcontrol_port|log_file|target_dir)' /etc/motion/motion.conf
```

检查是否有进程：

```bash
ps -ef | grep '[m]otion'
```

检查视频流端口是否可访问：

```bash
curl -v --max-time 5 http://localhost:8081/
```

检查本地端口占用。如果本地执行 SSH 隧道时出现：

```text
bind [127.0.0.1]:8081: Address already in use
```

说明本地端口被占用，换成本地端口 `18081` 或 `18082`：

```bash
ssh -N -L 18081:localhost:8081 cair-jacen@10.21.11.40
```

## 当前已验证结论

- 远程主机 `motion` 可以成功启动为 `active (running)`。
- 远程主机 `curl -v --max-time 5 http://localhost:8081/` 已返回：

```text
HTTP/1.1 200 OK
Content-Type: multipart/x-mixed-replace; boundary=BoundaryString
```

- 因此远程视频流已经正常。
- 本地查看画面时，只需要保持 SSH 隧道运行，并在本地浏览器访问 `http://localhost:18081/`。
