# Ultrasound Client API 文档

本文档详细说明了 `UltrasoundClient` 类中每个公共方法的用途、参数和返回值。

## 通用行为

-   所有“设置”或“控制”型方法在成功时返回 `True`。
-   如果设备响应表示命令失败（例如，通过返回 `0xFF` 状态字节），所有方法都会引发 `UltrasoundCommandError` 异常。
-   如果在通信过程中发生协议错误（如校验和不匹配、帧格式错误），方法会引发 `UltrasoundProtocolError`。
-   如果在网络层面发生问题（如连接超时、无法连接），方法会引发 `UltrasoundConnectionError`。

---

### `get_status()`

获取设备的完整实时状态。

-   **参数**: 无
-   **成功返回值**: 一个包含设备状态的字典。
    ```python
    {
        "scan_mode": "谐波",
        "probe_active": "激活",
        "is_b_mode": "是",
        "is_frozen": "解冻",
        "is_new_patient_ui": "否",
        "fps": "30.12",
        "d_g": "D=300, G=6",
        "gain_gn": 128,
        "i_p": "I=1, P=2",
        "power_pwr": 100,
        "frequency_frq": "第 1 档",
        "depth_d_mm": 35,
        "micro_imaging_us": 0,
        "image_start_x_percent": 12.54,
        "image_start_y_percent": 12.54,
        "image_width_px": 720,
        "image_height_px": 540,
    }
    ```

### `control_s_marker(show: bool)`

控制屏幕左上角“S”标记的显示或隐藏。

-   **参数**:
    -   `show` (`bool`): `True` 表示显示，`False` 表示隐藏。
-   **成功返回值**: `True`

### `control_ruler(show: bool)`

控制图像标尺（刻度线）的显示或隐藏。

-   **参数**:
    -   `show` (`bool`): `True` 表示显示，`False` 表示隐藏。
-   **成功返回值**: `True`

### `get_disk_space()`

获取超声主机用于存储图像的硬盘剩余容量。

-   **参数**: 无
-   **成功返回值**: 一个整数，表示剩余容量（单位：GB）。例如：`512`。

### `set_scan_mode(mode: ScanMode)`

设置扫描模式（基波或谐波）。

-   **参数**:
    -   `mode` (`ScanMode`): 使用 `ScanMode.FUNDAMENTAL`（基波）或 `ScanMode.HARMONIC`（谐波）。
-   **成功返回值**: `True`

### `get_patient_name()`

获取当前扫描患者的姓名。

-   **参数**: 无
-   **成功返回值**: 一个字符串，表示患者姓名。如果不存在，则返回空字符串 `""`。例如：`"ZhangSan"`。

### `get_patient_id()`

获取当前扫描患者的ID。

-   **参数**: 无
-   **成功返回值**: 一个字符串，表示患者ID。如果不存在，则返回空字符串 `""`。例如：`"PID12345"`。

### `get_patient_scan_time()`

获取当前患者的扫描时间。

-   **参数**: 无
-   **成功返回值**: 一个 Python 的 `datetime` 对象。例如：`datetime.datetime(2025, 11, 6, 10, 30, 0)`。

### `set_scan_depth(depth_cm: float)`

设置B模式下的扫描深度。

-   **参数**:
    -   `depth_cm` (`float`): 扫描深度，单位为厘米（cm）。例如：`3.5`。
-   **成功返回值**: `True`

### `set_image_type(image_type: ImageType)`

设置超声显示器显示的图像类型（处理前或处理后）。

-   **参数**:
    -   `image_type` (`ImageType`): 使用 `ImageType.POST_PROCESSED` (后处理) 或 `ImageType.PRE_PROCESSED` (处理前)。
-   **成功返回值**: `True`

### `start_image_acquisition(params: str)`

命令超声主机开始采集图像。

-   **参数**:
    -   `params` (`str`): 根据协议定义的参数，例如 `'LT'`, `'RT'`, `'LH'`, `'RH'`。
-   **成功返回值**: `True`

### `abort_image_acquisition()`

命令超声主机中止（丢弃）当前正在进行的图像采集。

-   **参数**: 无
-   **成功返回值**: `True`

### `end_image_acquisition()`

命令超声主机结束图像采集，此时图像将被冻结。

-   **参数**: 无
-   **成功返回值**: `True`

### `control_freeze(freeze: bool)`

控制图像的冻结或解冻。

-   **参数**:
    -   `freeze` (`bool`): `True` 表示冻结，`False` 表示解冻。
-   **成功返回值**: `True`

### `save_and_send_image()`

命令超声主机保存当前图像并通过DICOM发送。

-   **参数**: 无
-   **成功返回值**: 一个字典，包含成功状态和配置的DICOM服务器数量。
    ```python
    {"success": True, "dicom_servers": 1}
    ```

### `new_patient()`

命令超声主机跳转到新建患者界面。

-   **参数**: 无
-   **成功返回值**: `True`

### `get_dicom_send_status(patient_id: str)`

根据患者ID查询其关联图像的DICOM发送成功和失败的计数。

-   **参数**:
    -   `patient_id` (`str`): 要查询的患者ID。
-   **成功返回值**: 一个包含查询结果的字典。
    -   如果ID存在:
        ```python
        {
            "exists": True,
            "success_count": 15,
            "fail_count": 2,
            "id_returned": "PID12345"
        }
        ```
    -   如果ID不存在:
        ```python
        {"exists": False, "success_count": 0, "fail_count": 0}
        ```

### `get_zoom_factor()`

获取当前图像的放大倍数。

-   **参数**: 无
-   **成功返回值**: 一个浮点数，表示放大倍数。例如：`1.5`。

### `set_b_gain(gain: int)`

设置B模式的扫查增益值。

-   **参数**:
    -   `gain` (`int`): 增益值，范围为 0 到 999。
-   **成功返回值**: `True`
