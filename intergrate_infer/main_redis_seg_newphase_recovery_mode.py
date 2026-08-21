import os
import json
import time
import threading
import subprocess
import sys
import uuid
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import segmentation_models_pytorch as smp
import redis
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
import torch.backends.cudnn as cudnn
from torch.backends.cuda import sdp_kernel
from collections import deque

from utils import rot6d_to_matrix
from models import (
    models_mae,
    SwinTransformer,
    SwinTransformerV2,
    PoseHead,
)
from utils import (
    build_inv_normalizer_from_dataset,
)
from ultralytics import YOLO
from infer_realtime import (
    SingleConvNeXtClassifier,
    build_transform as build_convnext_transform,
    get_state_dict as get_convnext_state_dict,
    preprocess_frame as preprocess_convnext_frame,
)

# Global variables for thread communication
redis_client = None
redis_pubsub = None
command_publisher = None
camera = None
inference_running = True
trigger_inference = False  # Flag to trigger inference manually

# Phase Label Map
PHASE_LABELS = {0: 'Scan', 1: 'Trigger', 2: 'Action'}
CONVNEXT_LABELS = {0: 'pre', 1: 'in', 2: 'after', 3: 'brench', 4: 'rota'}
# The latest classifier has five outputs, while the RM75 Redis protocol and
# controller intentionally keep the existing three-phase state machine.
CONVNEXT_TO_ROBOT_PHASE = {0: 0, 1: 0, 2: 0, 3: 1, 4: 2}
COMMAND_CHANNEL = "robot:command:channel"
STATUS_CHANNEL = "robot:status:channel"

# Start from the historical 640x600 ultrasound ROI, then remove its fixed
# black side bars: 22 pixels on the left and 21 pixels on the right. Express
# the combined crop directly in raw 1920x1080 capture coordinates so YOLO,
# segmentation, pose/angle and phase inference all receive the same 640x557
# image, with no vertical flip.
ULTRASOUND_ROI = (160, 800, 532, 1089)
ULTRASOUND_FRAME_HEIGHT = 640
ULTRASOUND_FRAME_WIDTH = 557


def crop_ultrasound_frame(frame_bgr):
    if frame_bgr is None or frame_bgr.ndim != 3:
        raise ValueError("camera frame must be a non-empty HxWxC image")

    top, bottom, left, right = ULTRASOUND_ROI
    height, width = frame_bgr.shape[:2]
    if height < bottom or width < right:
        raise ValueError(
            f"camera frame {width}x{height} is smaller than the required "
            f"{right}x{bottom} ROI boundary"
        )

    cropped = frame_bgr[top:bottom, left:right]
    if cropped.shape[:2] != (
        ULTRASOUND_FRAME_HEIGHT,
        ULTRASOUND_FRAME_WIDTH,
    ):
        raise RuntimeError(f"unexpected cropped frame shape: {cropped.shape}")
    return cropped


def gate_visual_command_for_scan(
    y_value,
    rz_value,
    phase_idx,
    phase_confidence,
    recovery_mode,
    mask_lr_majority,
    scan_enabled,
):
    """b 后即可进行 Tool-Y 居中；m 仅额外放行 Tool-X 与 Tool-RZ。"""
    if scan_enabled:
        return (
            float(y_value),
            float(rz_value),
            int(phase_idx),
            float(phase_confidence),
            bool(recovery_mode),
            int(mask_lr_majority),
        )
    # phase=-1 不会触发 RM75 的 Tool-X/Trigger/RZ 状态机，但非零 y 会
    # 在接触与力门满足后直接进入 Tool-Y 对齐。
    return float(y_value), 0.0, -1, 0.0, False, 0


def gate_terminate_for_scan(auto_terminate, manual_terminate, scan_enabled):
    """模型自动结束只在 m 放行后生效；手动 t 始终保留。"""
    return bool(manual_terminate or (scan_enabled and auto_terminate))


def normal_termination_frame_ready(
    latest_phase_idx,
    rz_value,
    yolo_valid,
    seg_valid,
    vessel_pixels,
    pre_rotation_vessel_pixels,
    vessel_pixel_growth_ratio,
    rz_tolerance_deg,
):
    """Return whether one frame satisfies every automatic finish condition."""
    return bool(
        int(latest_phase_idx) == 4
        and (bool(yolo_valid) or bool(seg_valid))
        and int(pre_rotation_vessel_pixels) > 0
        and int(vessel_pixels)
            > int(pre_rotation_vessel_pixels)
                * float(vessel_pixel_growth_ratio)
        and np.isfinite(rz_value)
        and abs(float(rz_value)) < float(rz_tolerance_deg)
    )


class VisualYConditioner:
    """将逐帧视觉横向误差变成稳定的 Tool-Y 命令输入。"""

    def __init__(self, deadband_m, lowpass_alpha, confirmation_frames):
        if deadband_m < 0.0:
            raise ValueError("visual Y deadband must be non-negative")
        if not 0.0 < lowpass_alpha <= 1.0:
            raise ValueError("visual Y low-pass alpha must be in (0, 1]")
        if confirmation_frames < 1:
            raise ValueError("visual Y confirmation frames must be positive")
        self.deadband_m = float(deadband_m)
        self.lowpass_alpha = float(lowpass_alpha)
        self.confirmation_frames = int(confirmation_frames)
        self.reset()

    def reset(self):
        self._filtered_y_m = 0.0
        self._confirmed_sign = 0
        self._candidate_sign = 0
        self._candidate_count = 0

    def update(self, raw_y_m):
        """死区清零；换向时先连续确认，再对同向误差做 EMA 低通。"""
        raw_y_m = float(raw_y_m)
        if not np.isfinite(raw_y_m) or abs(raw_y_m) <= self.deadband_m:
            self.reset()
            return 0.0

        sign = 1 if raw_y_m > 0.0 else -1
        if sign != self._confirmed_sign:
            if sign == self._candidate_sign:
                self._candidate_count += 1
            else:
                self._candidate_sign = sign
                self._candidate_count = 1

            # 尚未确认方向时发布零，防止单帧噪声让机械臂反向。
            if self._candidate_count < self.confirmation_frames:
                self._filtered_y_m = 0.0
                return 0.0

            self._confirmed_sign = sign
            self._candidate_sign = sign
            self._candidate_count = self.confirmation_frames
            self._filtered_y_m = raw_y_m
        else:
            self._candidate_sign = sign
            self._candidate_count = self.confirmation_frames
            self._filtered_y_m = (
                self.lowpass_alpha * raw_y_m
                + (1.0 - self.lowpass_alpha) * self._filtered_y_m
            )

        return (self._filtered_y_m
                if abs(self._filtered_y_m) > self.deadband_m else 0.0)


def colorize_mask(mask, color_map):
    color_mask = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for class_id, color in enumerate(color_map):
        color_mask[mask == class_id] = color
    return color_mask

class VisionCommandPublisher:
    """与 main_fast.py 一致的 Redis v1 会话和递增序号发布器。"""

    def __init__(self, client, channel=COMMAND_CHANNEL, session_id=None):
        self.client = client
        self.channel = channel
        self.session_id = session_id or str(uuid.uuid4())
        self.sequence = 0
        # RM75 的 command_stale_ms 为 500 ms。推理、画面冻结或按键等待
        # 期间仍需重发最后一条命令，不能让机器人因没有新 Redis 消息进入 Hold。
        self._lock = threading.Lock()
        self._last_command = None
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None
        self._heartbeat_period_s = 0.2

    def publish(
        self,
        y_value,
        rz_value,
        terminate_value,
        recovery_mode_value,
        phase_idx=None,
        phase_confidence=0.0,
        robot_action_state="idle",
        mask_lr_majority=0,
        heartbeat=False,
    ):
        with self._lock:
            self.sequence += 1
            command_data = {
                "version": 1,
                "session_id": self.session_id,
                "sequence": self.sequence,
                "timestamp_unix_ms": int(time.time() * 1000),
                "parameters": {
                    "y": float(y_value),
                    "rz": float(rz_value),
                },
                "terminate": bool(terminate_value),
                "recovery_mode": bool(recovery_mode_value),
                "phase_idx": int(phase_idx) if phase_idx is not None else -1,
                "phase_confidence": float(phase_confidence),
                "action_state": robot_action_state == "moving",
                "mask_lr_majority": int(mask_lr_majority),
            }
            try:
                self.client.publish(self.channel, json.dumps(command_data))
            except Exception as error:
                print(f"[Python] Redis publish error: {error}")
                return None
            self._last_command = command_data
        if not heartbeat:
            print(
                "[Python] Published v1 command - "
                f"session: {self.session_id[:8]}, sequence: {command_data['sequence']}, "
                f"action: {int(command_data['action_state'])}, "
                f"phase: {command_data['phase_idx']}, "
                f"confidence: {command_data['phase_confidence']:.3f}, "
                f"y: {float(y_value):.4f} m, rz: {float(rz_value):.3f} deg, "
                f"terminate: {bool(terminate_value)}, "
                f"recovery: {bool(recovery_mode_value)}, "
                f"mask_side: {int(mask_lr_majority)}"
            )
        return command_data

    def start_heartbeat(self):
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        self._heartbeat_stop.clear()

        def heartbeat_loop():
            while not self._heartbeat_stop.wait(self._heartbeat_period_s):
                with self._lock:
                    command = dict(self._last_command) if self._last_command else None
                if command is None:
                    continue
                self.publish(
                    command["parameters"]["y"],
                    command["parameters"]["rz"],
                    command["terminate"],
                    command["recovery_mode"],
                    phase_idx=command["phase_idx"],
                    phase_confidence=command["phase_confidence"],
                    robot_action_state=("moving" if command["action_state"] else "idle"),
                    mask_lr_majority=command["mask_lr_majority"],
                    heartbeat=True,
                )

        self._heartbeat_thread = threading.Thread(
            target=heartbeat_loop, name="redis-command-heartbeat", daemon=True
        )
        self._heartbeat_thread.start()

    def stop_heartbeat(self):
        self._heartbeat_stop.set()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)


def check_command_protocol():
    """不连接 Redis/相机，检查生产 v1 字段和会话递增语义。"""

    class RecordingRedis:
        def __init__(self):
            self.messages = []

        def publish(self, channel, payload):
            self.messages.append((channel, json.loads(payload)))
            return 1

    fake_redis = RecordingRedis()
    publisher = VisionCommandPublisher(
        fake_redis,
        session_id="00000000-0000-0000-0000-000000000001",
    )
    idle = publisher.publish(
        0.0,
        0.0,
        False,
        False,
        phase_idx=-1,
        phase_confidence=0.0,
        robot_action_state="idle",
    )
    axial_values = gate_visual_command_for_scan(
        0.0012,
        -8.5,
        1,
        0.86,
        True,
        -1,
        False,
    )
    axial = publisher.publish(
        axial_values[0],
        axial_values[1],
        False,
        axial_values[4],
        phase_idx=axial_values[2],
        phase_confidence=axial_values[3],
        robot_action_state="moving",
        mask_lr_majority=axial_values[5],
    )
    scan_values = gate_visual_command_for_scan(
        0.0012,
        -8.5,
        2,
        0.86,
        True,
        1,
        True,
    )
    scan = publisher.publish(
        scan_values[0],
        scan_values[1],
        False,
        scan_values[4],
        phase_idx=scan_values[2],
        phase_confidence=scan_values[3],
        robot_action_state="moving",
        mask_lr_majority=scan_values[5],
    )
    if idle is None or axial is None or scan is None:
        raise RuntimeError("command protocol self-check failed to publish")
    if (
        idle["version"] != 1
        or idle["sequence"] != 1
        or idle["action_state"]
        or idle["terminate"]
    ):
        raise RuntimeError("first command is not a valid v1 idle handshake")
    if (
        axial["sequence"] != 2
        or not axial["action_state"]
        or axial["phase_idx"] != -1
        or axial["parameters"] != {"y": 0.0012, "rz": 0.0}
    ):
        raise RuntimeError("b/Z+Y command fields are invalid")
    if (
        scan["sequence"] != 3
        or not scan["action_state"]
        or scan["phase_idx"] != 2
        or scan["phase_confidence"] != 0.86
        or not scan["recovery_mode"]
        or scan["mask_lr_majority"] != 1
    ):
        raise RuntimeError("m/scan command fields are invalid")
    if (
        axial["session_id"] != idle["session_id"]
        or scan["session_id"] != idle["session_id"]
    ):
        raise RuntimeError("session changed without an explicit reset")
    if fake_redis.messages[0][0] != COMMAND_CHANNEL:
        raise RuntimeError("command channel is incorrect")
    print("[Python] Integrated Redis v1 command protocol self-check passed")
    print(json.dumps(scan, indent=2))


def poll_redis_messages(redis_pubsub, latest_status, robot_action_state):
    """非阻塞读取机器人状态，同时保留旧 robot_control 启动消息。"""
    newest_status = latest_status
    next_action_state = robot_action_state
    try:
        while True:
            message = redis_pubsub.get_message(timeout=0.0)
            if message is None:
                break
            if message.get("type") != "message":
                continue
            channel = message.get("channel")
            parsed = json.loads(message["data"])
            if not isinstance(parsed, dict):
                continue
            if channel == STATUS_CHANNEL:
                previous_key = None if newest_status is None else (
                    newest_status.get("session_id"),
                    newest_status.get("producer_sequence"),
                    newest_status.get("state"),
                    newest_status.get("error_code"),
                )
                current_key = (
                    parsed.get("session_id"),
                    parsed.get("producer_sequence"),
                    parsed.get("state"),
                    parsed.get("error_code"),
                )
                newest_status = parsed
                if current_key != previous_key:
                    print(
                        "[Python] Robot status - "
                        f"session: {str(parsed.get('session_id'))[:8]}, "
                        f"ack: {parsed.get('producer_sequence', 0)}, "
                        f"state: {parsed.get('state')}, "
                        f"phase: {parsed.get('phase_idx')}, "
                        f"age_ms: {parsed.get('command_age_ms')}, "
                        f"fault: {parsed.get('error_code')}"
                    )
            elif channel == "robot_control":
                if (
                    parsed.get("command") == "robot"
                    and parsed.get("action") == "move"
                ):
                    bodypart = parsed.get("parameter")
                    print(f"[Python] Received Robot Start for: {bodypart}")
                    next_action_state = "moving"
    except Exception as error:
        print(f"[Python] Redis message error: {error}")
    return newest_status, next_action_state


def draw_robot_status_overlay(
    frame_bgr,
    latest_status,
    publisher,
    robot_action_state,
    scan_enabled,
):
    status = latest_status or {}
    session_matches = status.get("session_id") == publisher.session_id
    command_age_ms = status.get("command_age_ms", 0.0)
    if not isinstance(command_age_ms, (int, float)):
        command_age_ms = 0.0
    lines = [
        (
            f"Redis session={publisher.session_id[:8]} "
            f"sent={publisher.sequence} action={robot_action_state} "
            f"scan={'on' if scan_enabled else 'off'}"
        ),
        (
            f"Robot={status.get('state', 'no-status')} "
            f"ack={status.get('producer_sequence', 0) if session_matches else 0} "
            f"age={command_age_ms:.1f}ms"
        ),
        f"Fault={status.get('error_code') or 'none'}",
    ]
    for index, line in enumerate(lines):
        cv2.putText(
            frame_bgr,
            line,
            (10, 310 + 35 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return frame_bgr

def _normalize_key(k):
    import re
    replacements = [
        (r'^module\.', ''),
        (r'^encoder\.', ''),
        (r'\.mlp\.fc1\.', '.mlp.linear1.'),
        (r'\.mlp\.fc2\.', '.mlp.linear2.'),
        (r'layers\.(\d+)\.', r'layers\1.'),
    ]
    for pat, rep in replacements:
        k = re.sub(pat, rep, k)
    return k

def build_swinmodel(swincheckpoint, device):
    model = SwinTransformer(
        in_chans=3, embed_dim=128, patch_size=(2, 2), window_size=(8, 8),
        depths=(2, 2, 18, 2), num_heads=(4, 8, 16, 32),
        mlp_ratio=4., qkv_bias=True, spatial_dims=2, use_v2=True
    ).to(device).eval()

    ckpt_path = swincheckpoint
    if os.path.isfile(ckpt_path):
        raw = torch.load(ckpt_path, map_location='cpu')
        raw = raw.get('model', raw)
        ckpt = {_normalize_key(k): v for k, v in raw.items()}
        loadable = {k: v for k, v in ckpt.items()
                    if k in model.state_dict() and v.shape == model.state_dict()[k].shape}
        msg = model.load_state_dict(loadable, strict=False)
        print(f"swin pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    else:
        print(f"swin pretrain not found: {ckpt_path}, random init.")
    return model

def build_swinmodel_origin(swinocheckpoint, device):
    swin = SwinTransformerV2(
        img_size=256, patch_size=2, in_chans=3,
        num_classes=0,
        embed_dim=48, depths=(2, 2, 2, 2),
        num_heads=(3, 6, 12, 24),
        window_size=8, drop_path_rate=0.3
    ).to(device).eval()

    ckpt_p = swinocheckpoint
    if os.path.isfile(ckpt_p):
        raw_state = torch.load(ckpt_p, map_location="cpu")
        raw_state = raw_state.get("model", raw_state)
        enc_state = {k: v for k, v in raw_state.items() if not k.startswith("head.")}
        swin.load_state_dict(enc_state, strict=False)
        print("[swino pretrain loaded.")
    else:
        print(f"swino pretrain not found: {ckpt_p}, random init.")
    return swin

def build_maemodel(maecheckpoint, maemodel, device):
    assert hasattr(models_mae, maemodel), f"models_mae.py cannot find '{maemodel}'"
    model = getattr(models_mae, maemodel)(norm_pix_loss=False).to(device).eval()
    ckpt = torch.load(maecheckpoint, map_location='cpu', weights_only=False)
    state_dict = ckpt["model"] if "model" in ckpt else ckpt
    msg = model.load_state_dict(state_dict, strict=False)
    print(f"mae pretrain loaded: missing={len(msg.missing_keys)} unexpected={len(msg.unexpected_keys)}")
    return model

def center_crop_numpy(img, size):
    h, w = img.shape[:2]
    th, tw = size, size
    y1 = max(0, (h - th) // 2)
    x1 = max(0, (w - tw) // 2)
    return img[y1:y1+th, x1:x1+tw, :]

def preprocess_cv2(img_bgr, backbone, size):
    if backbone in ('swin', 'swino', 'transformer'):
        interp = cv2.INTER_CUBIC  
        img_bgr = cv2.resize(img_bgr, (size, size), interpolation=interp)
    else:
        raise ValueError(backbone)

    if backbone == 'transformer':
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img_rgb = (img_rgb - mean) / std
    else:
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

    chw = np.transpose(img_rgb, (0, 1, 2)).transpose(2, 0, 1) 
    tensor = torch.from_numpy(chw).unsqueeze(0)
    return tensor

def extract_embedding(backbone, model, img_tensor, pick_indices):
    if backbone == 'swin':
        feat = model.forward_features(img_tensor, feat_type='pyramid')[-1]
        return None, None, [feat]
    elif backbone == 'swino':
        feat = model.forward_features2(img_tensor)
        return None, None, feat
    elif backbone == 'transformer':
        x, feats, emb = model.forward_encoder2(img_tensor, mask_ratio=0.0, pick_indices=pick_indices)
        return x, feats, emb
    else:
        raise ValueError(backbone)

def init_camera(camera_id, width, height, fps):
    """Initialize USB camera"""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {camera_id}")
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # Verify settings
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
    return cap

def run_yolo_on_frame(frame_bgr, yolo_model, conf_thres=0.5, target_class_id=0, debug=False):
    """
    Online YOLO inference on one frame.
    Returns:
        best_det: dict or None
        target_count: number of boxes with target_class_id
    """
    best_det = None
    target_count = 0
    valid_center_y_min = 200
    valid_center_y_max = 350
    try:
        results = yolo_model.predict(
            source=frame_bgr,
            conf=conf_thres,
            save=False,
            save_txt=False,
            verbose=False
        )

        if len(results) == 0:
            if debug:
                print("[YOLO DEBUG] empty results")
            return None, 0

        if results[0].boxes is None or len(results[0].boxes) == 0:
            if debug:
                print("[YOLO DEBUG] no boxes found")
            return None, 0

        best_conf = -1.0

        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            if debug:
                print(f"[YOLO DEBUG] cls={cls_id}, conf={conf:.3f}, box=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")

            if cls_id != target_class_id:
                continue

            target_count += 1

            if conf > best_conf:
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                if not (valid_center_y_min <= cy <= valid_center_y_max):
                    continue
                best_conf = conf
                best_det = {
                    "x1": int(round(x1)),
                    "y1": int(round(y1)),
                    "x2": int(round(x2)),
                    "y2": int(round(y2)),
                    "cx": int(round(cx)),
                    "cy": int(round(cy)),
                    "conf": conf,
                    "cls_id": cls_id
                }

        if debug:
            print(f"[YOLO DEBUG] target_class_id={target_class_id}, target_count={target_count}, best_det={best_det}")

    except Exception as e:
        print(f"[YOLO] inference error: {e}")

    return best_det, target_count

def draw_valid_center_y_band(img, y_min, y_max):
    """
    Visualize the valid center-y range as two horizontal lines
    and a translucent band between them.
    """
    out = img.copy()
    h, w = out.shape[:2]

    # Safety clamp
    y_min = max(0, min(h - 1, int(y_min)))
    y_max = max(0, min(h - 1, int(y_max)))

    if y_min > y_max:
        y_min, y_max = y_max, y_min

    # Draw translucent band
    overlay = out.copy()
    cv2.rectangle(overlay, (0, y_min), (w - 1, y_max), (0, 255, 0), -1)
    out = cv2.addWeighted(overlay, 0.15, out, 0.85, 0)

    # Draw boundary lines
    cv2.line(out, (0, y_min), (w - 1, y_min), (0, 255, 0), 2)
    cv2.line(out, (0, y_max), (w - 1, y_max), (0, 255, 0), 2)

    # Labels
    # cv2.putText(out, f"valid_center_y_min = {y_min}", (10, max(25, y_min - 10)),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    # cv2.putText(out, f"valid_center_y_max = {y_max}", (10, min(h - 10, y_max + 25)),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    return out

def draw_yolo_overlay(img, det, det_count=0):
    """
    Draw YOLO bbox and center point on image.
    """
    out = img.copy()

    if det is None:
        # cv2.putText(
        #     out, "YOLO: no target", (10, 190),
        #     cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA
        # )
        return out

    x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
    cx, cy = det["cx"], det["cy"]
    conf = det["conf"]

    # Bounding box
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)

    # Center point
    cv2.circle(out, (cx, cy), 5, (255, 0, 255), -1)

    # Crosshair around center
    cv2.line(out, (cx - 12, cy), (cx + 12, cy), (255, 0, 255), 2)
    cv2.line(out, (cx, cy - 12), (cx, cy + 12), (255, 0, 255), 2)

    # Text
    cv2.putText(
        out, f"YOLO conf: {conf:.2f}", (10, 190),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA
    )
    cv2.putText(
        out, f"Center: ({cx}, {cy})", (10, 225),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA
    )
    cv2.putText(
        out, f"Count: {det_count}", (10, 260),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 255), 2, cv2.LINE_AA
    )

    return out

def filter_valid_seg_regions(pred_mask, target_class=2,
                             min_area=10000,
                             valid_center_y_min=200,
                             valid_center_y_max=350):
    """
    Keep only connected components that satisfy:
    1. area > min_area
    2. component center_y in [valid_center_y_min, valid_center_y_max]

    Returns:
        valid_mask: filtered mask
        seg_valid: whether at least one valid component exists
        valid_regions: list of region info
    """
    binary = (pred_mask == target_class).astype(np.uint8)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)

    valid_mask = np.zeros_like(pred_mask, dtype=np.uint8)
    valid_regions = []

    for label_id in range(1, num_labels):  # 0 is background
        area = stats[label_id, cv2.CC_STAT_AREA]
        cx, cy = centroids[label_id]

        if area <= min_area:
            continue

        if not (valid_center_y_min <= cy <= valid_center_y_max):
            continue

        valid_mask[labels == label_id] = target_class
        valid_regions.append({
            "label_id": label_id,
            "area": int(area),
            "cx": float(cx),
            "cy": float(cy)
        })

    seg_valid = len(valid_regions) > 0
    return valid_mask, seg_valid, valid_regions

def get_mask_side_label(mask, target_class=2):
    """
    For one frame, determine whether the valid mask is more on the left or right.
    Return:
        0 -> no valid mask or tie
        1 -> left
        2 -> right
    """
    valid = (mask == target_class)
    if not np.any(valid):
        return 0

    h, w = mask.shape
    mid = w // 2

    left_count = int(np.sum(valid[:, :mid]))
    right_count = int(np.sum(valid[:, mid:]))

    if left_count > right_count:
        return 1
    elif right_count > left_count:
        return 2
    else:
        return 0


def get_majority_side_from_history(side_history):
    """
    side_history contains values in {0,1,2}
    We only count valid votes 1/2.
    Return:
        0 -> no valid vote or tie
        1 -> left majority
        2 -> right majority
    """
    if len(side_history) == 0:
        return 0

    left_votes = sum(1 for x in side_history if x == 1)
    right_votes = sum(1 for x in side_history if x == 2)

    if left_votes > right_votes:
        return 1
    elif right_votes > left_votes:
        return 2
    else:
        return 0


def measure_carotid_short_axis_diameter(carotid_mask, image_height_mm=37.0):
    if carotid_mask is None:
        return None

    mask = ((carotid_mask > 0).astype(np.uint8)) * 255
    if np.count_nonzero(mask) == 0:
        return None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < 10.0:
        return None

    rect = cv2.minAreaRect(largest_contour)
    (cx, cy), (rect_w, rect_h), _ = rect
    if rect_w <= 0 or rect_h <= 0:
        return None

    box = cv2.boxPoints(rect).astype(np.float32)

    def midpoint(p1, p2):
        return 0.5 * (p1 + p2)

    m01 = midpoint(box[0], box[1])
    m23 = midpoint(box[2], box[3])
    m12 = midpoint(box[1], box[2])
    m30 = midpoint(box[3], box[0])

    d_a = float(np.linalg.norm(m01 - m23))
    d_b = float(np.linalg.norm(m12 - m30))

    if d_a <= d_b:
        p1, p2 = m01, m23
        diameter_px = d_a
    else:
        p1, p2 = m12, m30
        diameter_px = d_b

    if diameter_px <= 0:
        return None

    h = carotid_mask.shape[0]
    if h <= 0:
        return None

    mm_per_px = float(image_height_mm) / float(h)
    diameter_mm = float(diameter_px) * mm_per_px

    return {
        "diameter_px": float(diameter_px),
        "diameter_mm": float(diameter_mm),
        "mm_per_px": float(mm_per_px),
        "center": (float(cx), float(cy)),
        "p1": (float(p1[0]), float(p1[1])),
        "p2": (float(p2[0]), float(p2[1])),
        "rect_box": box,
    }


def render_hold_measurement_overlay(frame_bgr, carotid_mask, image_height_mm=37.0):
    vis = frame_bgr.copy()
    measure = measure_carotid_short_axis_diameter(carotid_mask, image_height_mm=image_height_mm)

    if measure is None:
        cv2.putText(
            vis,
            "CCA D: N/A",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return vis, None

    p1 = measure["p1"]
    p2 = measure["p2"]
    x1 = int(np.clip(round(p1[0]), 0, vis.shape[1] - 1))
    y1 = int(np.clip(round(p1[1]), 0, vis.shape[0] - 1))
    x2 = int(np.clip(round(p2[0]), 0, vis.shape[1] - 1))
    y2 = int(np.clip(round(p2[1]), 0, vis.shape[0] - 1))

    rect_box = measure.get("rect_box", None)
    if rect_box is not None:
        rect_box_i = np.round(rect_box).astype(np.int32)
        cv2.polylines(vis, [rect_box_i], isClosed=True, color=(255, 200, 0), thickness=1, lineType=cv2.LINE_AA)

    cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(vis, (x1, y1), 4, (0, 255, 0), -1, cv2.LINE_AA)
    cv2.circle(vis, (x2, y2), 4, (0, 255, 0), -1, cv2.LINE_AA)

    cv2.putText(
        vis,
        f"CCA D: {measure['diameter_mm']:.2f} mm ({measure['diameter_px']:.1f} px)",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return vis, measure


def update_cca_diameter_json(diameter_mm, json_path="./cca.json"):
    try:
        data = {}
        if os.path.exists(json_path):
            with open(json_path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    data = loaded

        data["vessel_cca_diameter_mm"] = f"{float(diameter_mm):.2f}"

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        return True
    except Exception as e:
        print(f"[AutoMeasure] Failed to update {json_path}: {e}")
        return False


def generate_carotid_report(
    image_dir,
    input_file="./cca.json",
    timeout_sec=600,
    deterministic_only=False,
    no_think=False,
    skip_image_llm=False,
    max_new_tokens=None,
    request_timeout_sec=None,
):
    script_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "report_generate",
        "carotid_report_local.py",
    )
    if not os.path.isfile(script_path):
        raise RuntimeError(f"Report script not found: {script_path}")

    cmd = [
        sys.executable,
        script_path,
        "--image-dir",
        image_dir,
        "--input-file",
        input_file,
    ]
    if deterministic_only:
        cmd.append("--deterministic-only")
    if no_think:
        cmd.append("--no-think")
    if skip_image_llm:
        cmd.append("--skip-image-llm")
    if max_new_tokens is not None:
        cmd.extend(["--max-new-tokens", str(int(max_new_tokens))])
    if request_timeout_sec is not None:
        cmd.extend(["--timeout", str(int(request_timeout_sec))])

    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_sec)

    stdout = proc.stdout or ""
    stderr = proc.stderr or ""
    if proc.returncode != 0:
        err_tail = stderr.strip() or stdout.strip() or f"return code={proc.returncode}"
        raise RuntimeError(f"Report generation failed: {err_tail}")

    pdf_path = None
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("[PDF] 已生成："):
            pdf_path = line.split("：", 1)[-1].strip()
            break
    return pdf_path, stdout

def _open_pdf_file(pdf_path):
    if not pdf_path:
        return
    try:
        if not os.path.isfile(pdf_path):
            print(f"[AutoReport] PDF not found: {pdf_path}")
            return
        subprocess.Popen(["xdg-open", pdf_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[AutoReport] Opened PDF: {pdf_path}")
    except Exception as e:
        print(f"[AutoReport] Failed to open PDF: {e}")

def main():
    global redis_client, redis_pubsub, command_publisher
    global camera, inference_running, trigger_inference
    
    # ====================== Configuration Settings ======================
    # Camera settings
    camera_id = 2
    camera_width = 1920
    camera_height = 1080
    camera_fps = 30
    
    # Redis settings
    redis_host = '127.0.0.1'
    redis_port = 7777
    
    # Model settings
    backbone = 'transformer'
    swincheckpoint = '/home/jiuan/AAAworkspace/Myproject/slicelocalization/pretrain_weights/simmim/swin_ckp_final_2.pt'
    swinocheckpoint = '/path/to/swino_pretrain.pth'
    maecheckpoint = 'weights/backbone/carotid_base_checkpoint-500_0721.pth'
    maemodel = 'mae_vit_base_patch16'
    transformer_layers = [9] 
    
    # Head checkpoints
    posehead_ckpt = "weights/pose_head.pth"
    anglehead_ckpt = 'weights/angle_head.pth'
    # anglehead_ckpt = '/home/cair/Projects/ziping/uspilot_ctrl/runs/angle/20260430_110744/weights/best_aug.pth'

    # Latest five-class ConvNeXt phase classifier. Its outputs are mapped to
    # RM75 phases as: pre/in/after -> Scan, brench -> Trigger, rota -> Action.
    phase_classifier_ckpt = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'weights',
        'runs_ConvNeXtBase_single.pth',
    )

    # Segmentation settings
    seg_model_path = 'weights/seg_model.pth' 
    seg_encoder = 'resnet34'
    seg_encoder_weights = 'imagenet'
    seg_classes = 3
    seg_threshold = 0.9 # 增加阈值配置    
    seg_y_threshold = 100 # y轴小于此值的区域不显示分割结果 (像素单位)    
    # 0:Black, 1:Black, 2:Orange
    seg_color_map = [
        [0, 0, 0],       
        [0, 0, 0],   
        [255, 50, 0]    
    ]

    # Phase order settings (Scan -> Trigger -> Action)
    phase_order = [0, 1, 2]
    current_phase_idx = 0  # start from Scan
    phase_candidate_idx = 0
    phase_candidate_count = 0

    # Phase debounce / confidence thresholds
    # Higher patience for Scan to avoid easy rollback
    phase_change_patience = {
        0: 8,  # Scan
        1: 3,  # Trigger
        2: 3,  # Action
    }
    phase_min_prob = {
        0: 0.60,
        1: 0.55,
        2: 0.55,
    }

    # Normal completion requires the latest model's raw class 4 (rota), at
    # least one vessel detector, and a stable RZ residual inside this tolerance.
    termination_rz_tolerance_deg = 12.0
    termination_stable_frames = 20
    termination_stable_count = 0
    termination_vessel_pixel_growth_ratio = 3.0
    pre_rotation_vessel_pixels = 0
    pre_rotation_vessel_pixels_locked = False
    auto_terminate_latched = False
    terminate_command_sequence = 0

    # Tool-Y 视觉误差调理：0.01 mm 居中死区，方向当帧确认；
    # alpha=0.7：当前帧占 70%，上一次滤波结果占 30%。
    visual_y_deadband_m = 0.00001
    visual_y_lowpass_alpha = 0.7
    visual_y_confirmation_frames = 1
    visual_y_conditioner = VisualYConditioner(
        visual_y_deadband_m,
        visual_y_lowpass_alpha,
        visual_y_confirmation_frames,
    )

    # recovery mode settings
    recovery_mode = False
    lost_vessel_count = 0
    found_vessel_count = 0
    mask_side_history = deque(maxlen=30)
    mask_lr_majority = 0
    current_mask_side = 0
    last_valid_seg_mask = None
    # thresholds
    lost_vessel_patience = 20      # 连续丢失多少帧后进入 recovery
    recover_vessel_patience = 5   # 连续检测到多少帧后退出 recovery
    valid_center_y_min = 20
    valid_center_y_max = 500

    # 手动可调节
    min_vessel_pixels = 3000       # 你后面可以再调

    # YOLO settings
    yolo_model_path = "weights/yolo.pt"
    yolo_conf_thres = 0.6
    yolo_target_class_id = 0
    yolo_flag = True

    # Other settings
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Normalization stats path
    stats_path = "weights/norm_stats_all.pt"
    
    # Video saving settings
    output_dir = './output'

    # Auto capture & report settings
    auto_capture_enabled = True
    image_total_height_mm = 37.0

    report_deterministic_only = False
    report_no_think = True
    report_skip_image_llm = False
    report_max_new_tokens = 1024
    report_request_timeout_sec = 45
    
    # ====================== Initialization ======================
    
    cudnn.benchmark = True
    
    # Initialize Redis connection
    try:
        redis_client = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        redis_pubsub = redis_client.pubsub()
        redis_client.ping()
        print(f"[Python] Connected to Redis at {redis_host}:{redis_port}")
    except Exception as e:
        print(f"[Python] Failed to connect to Redis: {e}")
        return
    
    # Initialize camera
    try:
        camera = init_camera(camera_id, camera_width, camera_height, camera_fps)
    except Exception as e:
        print(f"[Python] Failed to initialize camera: {e}")
        return
        
    # Initialize video writer
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    video_filename = os.path.join(output_dir, f"{timestamp}.avi")
    auto_capture_dir = os.path.join(output_dir, f"auto_capture_{timestamp}")
    os.makedirs(auto_capture_dir, exist_ok=True)
    frame_width_out = ULTRASOUND_FRAME_WIDTH
    frame_height_out = ULTRASOUND_FRAME_HEIGHT
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # video_writer = cv2.VideoWriter(video_filename, fourcc, float(camera_fps), (frame_width_out, frame_height_out))
    print(f"Recording video to {video_filename}")

    # Load normalization stats
    inv_dT1e = build_inv_normalizer_from_dataset(
        key="dT_1e", 
        stats_path=stats_path, 
        mode="minmax_pad", 
        pad_ratio=0.05, 
        feat_axis=-1, 
        index=[1]
    )
    # --- Build YOLO Model ---
    print("[Python] Loading YOLO Model...")
    try:
        yolo_model = YOLO(yolo_model_path)
        print("[Python] YOLO model loaded.")
    except Exception as e:
        print(f"[Python] Failed to load YOLO model: {e}")
        yolo_model = None

    img_size = 256 if backbone in ('swin', 'swino') else 224

    # Build backbone
    if backbone == 'swin':
        backbone_model = build_swinmodel(swincheckpoint, device)
    elif backbone == 'swino':
        backbone_model = build_swinmodel_origin(swinocheckpoint, device)
    else:
        backbone_model = build_maemodel(maecheckpoint, maemodel, device)
    
    # --- Build Heads ---
    posehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
    anglehead = PoseHead(in_dim=768, out_dim=1).to(device).eval()
    
    # Build the latest standalone ConvNeXt phase classifier. This replaces the
    # former MAE-feature RESNetHead; the other heads continue using MAE.
    phase_classifier = SingleConvNeXtClassifier(num_classes=5)
    phase_transform = build_convnext_transform(image_size=224)

    # --- Build Segmentation Model ---
    print("[Python] Loading Segmentation Model...")
    try:
        seg_model = smp.Unet(
            encoder_name=seg_encoder, 
            encoder_weights=seg_encoder_weights, 
            classes=seg_classes, 
            activation=None
        ).to(device)
        
        if os.path.exists(seg_model_path):
            seg_model.load_state_dict(torch.load(seg_model_path, map_location='cpu'))
            print("[Python] Segmentation model loaded.")
        else:
            print(f"[Python] Segmentation model not found at {seg_model_path}, using random init.")
        seg_model.eval()
    except Exception as e:
        print(f"[Python] Failed to load segmentation model: {e}")
        seg_model = None

    # --- Load Checkpoints ---
    print("\n[Loading Checkpoints...]")
    
    # Load checkpoints
    posehead_ckpt_data = torch.load(posehead_ckpt, map_location='cpu', weights_only=False)
    anglehead_ckpt_data = torch.load(anglehead_ckpt, map_location='cpu', weights_only=False)
    
    if not os.path.isfile(phase_classifier_ckpt):
        raise FileNotFoundError(
            f"ConvNeXt phase classifier checkpoint not found: {phase_classifier_ckpt}"
        )
    print(f"Loading ConvNeXt phase classifier from: {phase_classifier_ckpt}")
    phase_checkpoint = torch.load(
        phase_classifier_ckpt, map_location='cpu', weights_only=False
    )
    phase_state_dict = get_convnext_state_dict(phase_checkpoint)
    phase_load_result = phase_classifier.load_state_dict(
        phase_state_dict, strict=True
    )
    phase_classifier.to(device).eval()
    print(
        "ConvNeXt phase classifier loaded: "
        f"missing={len(phase_load_result.missing_keys)} "
        f"unexpected={len(phase_load_result.unexpected_keys)}; "
        "mapping=0/1/2->0, 3->1, 4->2"
    )

    if 'posehead' in posehead_ckpt_data:
        missing, unexpected = posehead.load_state_dict(posehead_ckpt_data['posehead'], strict=False)
        print(f"posehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    if 'posehead' in anglehead_ckpt_data:
        missing, unexpected = anglehead.load_state_dict(anglehead_ckpt_data['posehead'], strict=False)
        print(f"anglehead loaded: missing={len(missing)} unexpected={len(unexpected)}")
    print("\n[Python] Starting inference loop...")
    print("Press 'q' to quit, 'b' to trigger inference\n")

    # Subscribe to status channel
    redis_pubsub.subscribe(STATUS_CHANNEL)
    redis_pubsub.subscribe("robot_control")
    command_publisher = VisionCommandPublisher(redis_client)
    initial_idle = command_publisher.publish(
        0.0,
        0.0,
        False,
        False,
        phase_idx=-1,
        phase_confidence=0.0,
        robot_action_state="idle",
        mask_lr_majority=0,
    )
    if initial_idle is None:
        print("[Python] Initial Redis idle handshake failed")
        if camera is not None:
            camera.release()
        redis_pubsub.close()
        cv2.destroyAllWindows()
        return
    print(
        "[Python] New vision session idle handshake sent: "
        f"{command_publisher.session_id}"
    )
    command_publisher.start_heartbeat()
    
    # ====================== Main Inference Loop ======================
    # try:
    terminate_flag_key = False

    robot_action_state = "idle"  # Track robot action state
    # 两级运动使能：b 允许 Tool-Z 接近/力控和 Tool-Y 居中，m 再放行
    # Tool-X 扫描与 Tool-RZ 对齐。
    scan_enabled = False
    latest_robot_status = None
    segmentation_flag = False  # Enable segmentation overlay display
    freeze_on_hold_enabled = True
    is_display_frozen = False
    frozen_display_frame = None
    report_thread = None
    report_generated_once = False
    report_state = {
        "running": False,
        "done": False,
        "success": False,
        "message": "",
        "pdf_path": None,
    }
    report_state_lock = threading.Lock()

    phase_capture_done = {"scan": False, "trigger": False, "hold": False}

    def _set_report_state(**kwargs):
        with report_state_lock:
            report_state.update(kwargs)

    def _report_worker():
        _set_report_state(
            running=True,
            done=False,
            success=False,
            message="Report Generating",
            pdf_path=None,
        )
        try:
            pdf_path, _ = generate_carotid_report(
                image_dir=auto_capture_dir,
                input_file="./cca.json",
                timeout_sec=600,
                deterministic_only=report_deterministic_only,
                no_think=report_no_think,
                skip_image_llm=report_skip_image_llm,
                max_new_tokens=report_max_new_tokens,
                request_timeout_sec=report_request_timeout_sec,
            )
            _set_report_state(
                running=False,
                done=True,
                success=True,
                message="Report Generated",
                pdf_path=pdf_path,
            )
            if pdf_path:
                print(f"[AutoReport] Generated: {pdf_path}")
                _open_pdf_file(pdf_path)
            else:
                print("[AutoReport] Generated (PDF path parsed as empty).")
        except Exception as e:
            _set_report_state(
                running=False,
                done=True,
                success=False,
                message="Report Generation Failed",
                pdf_path=None,
            )
            print(f"[AutoReport] Failed: {e}")

    while inference_running:
        if is_display_frozen and frozen_display_frame is not None:
            # Keep consuming robot status after freezing the completed US
            # image, so the operator can see whether the latched terminate
            # command has actually been acknowledged by this Redis session.
            latest_robot_status, robot_action_state = poll_redis_messages(
                redis_pubsub,
                latest_robot_status,
                robot_action_state,
            )
            frozen_with_ack = frozen_display_frame.copy()
            status = latest_robot_status or {}
            session_matches = (
                status.get("session_id") == command_publisher.session_id
            )
            acknowledged_sequence = (
                status.get("producer_sequence", 0) if session_matches else 0
            )
            terminate_acknowledged = (
                terminate_command_sequence > 0
                and isinstance(acknowledged_sequence, (int, float))
                and acknowledged_sequence >= terminate_command_sequence
            )
            cv2.rectangle(
                frozen_with_ack, (0, 395),
                (frozen_with_ack.shape[1], 445), (0, 0, 0), -1,
            )
            cv2.putText(
                frozen_with_ack,
                (
                    f"Terminate latched=True seq={terminate_command_sequence} "
                    f"ack={terminate_acknowledged} "
                    f"robot={status.get('state', 'no-status')}"
                ),
                (10, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (0, 255, 0) if terminate_acknowledged else (0, 255, 255),
                2, cv2.LINE_AA,
            )
            cv2.imshow('US image', frozen_with_ack)
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("[Python] User requested quit")
                inference_running = False
                break
            continue
        latest_robot_status, robot_action_state = poll_redis_messages(
            redis_pubsub,
            latest_robot_status,
            robot_action_state,
        )

        begin_time = time.time()
        ret, frame = camera.read()
        if not ret:
            print("[Python] Failed to capture frame")
            continue
        
        try:
            frame = crop_ultrasound_frame(frame)
        except (ValueError, RuntimeError) as error:
            print(f"[Python] Invalid camera frame: {error}")
            continue
        # video_writer.write(frame)
        # --- YOLO Inference ---
        yolo_det = None
        yolo_count = 0
        # YOLO remains active for the completion safety gate while inference is
        # running; yolo_flag controls whether its overlay is displayed.
        if yolo_model is not None and (yolo_flag or trigger_inference):
            yolo_det, yolo_count = run_yolo_on_frame(
                frame_bgr=frame,
                yolo_model=yolo_model,
                conf_thres=yolo_conf_thres,
                target_class_id=yolo_target_class_id,
                debug=True
            )
        # --- Segmentation Inference ---
        vessel_found_this_frame = False
        seg_overlay_bgr = frame.copy()

        vessel_pixels = 0
        seg_valid = False
        seg_regions = []
        seg_overlay_bgr = frame.copy()

        need_seg_for_control = trigger_inference
        if seg_model is not None and (segmentation_flag or need_seg_for_control):
            try:
                with torch.no_grad():
                    h, w = frame.shape[:2]
                    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img_tensor_s = img_rgb.astype('float32') / 255.0
                    img_tensor_s = torch.from_numpy(img_tensor_s.transpose(2, 0, 1)).unsqueeze(0).to(device)

                    pad_h = (32 - h % 32) % 32
                    pad_w = (32 - w % 32) % 32
                    if pad_h > 0 or pad_w > 0:
                        img_tensor_s = F.pad(img_tensor_s, (0, pad_w, 0, pad_h), mode='constant', value=0)

                    seg_output = seg_model(img_tensor_s)
                    seg_probs = torch.softmax(seg_output, dim=1)
                    max_vals, pred_mask = torch.max(seg_probs, dim=1)

                    pred_mask = pred_mask.squeeze(0).cpu().numpy()
                    max_vals = max_vals.squeeze(0).cpu().numpy()

                    pred_mask[max_vals < seg_threshold] = 0

                    if pad_h > 0 or pad_w > 0:
                        pred_mask = pred_mask[:h, :w]

                    # Optional: keep your old top-area suppression if needed
                    # pred_mask[:seg_y_threshold, :] = 0

                    # Filter connected components
                    valid_seg_mask, seg_valid, seg_regions = filter_valid_seg_regions(
                        pred_mask,
                        target_class=2,
                        min_area=min_vessel_pixels,
                        valid_center_y_min=valid_center_y_min,
                        valid_center_y_max=valid_center_y_max
                    )
                    # Left-right distribution for current frame
                    current_mask_side = get_mask_side_label(valid_seg_mask, target_class=2)

                    # Save current frame result into history
                    mask_side_history.append(current_mask_side)

                    # Majority over recent 10 frames
                    mask_lr_majority = get_majority_side_from_history(mask_side_history)

                    vessel_pixels = int(np.sum(valid_seg_mask == 2))

                    if seg_valid:
                        last_valid_seg_mask = valid_seg_mask.copy()

                    # Visualization only shows valid regions
                    colored_mask = colorize_mask(valid_seg_mask, seg_color_map)
                    colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
                    seg_overlay_bgr = cv2.addWeighted(frame, 1, colored_mask_bgr, 0.8, 0)

            except Exception as e:
                # print(f"[Seg] inference error: {e}")
                pass

        yolo_valid = (yolo_det is not None)

        ############ try #############
        # frame_has_target = seg_valid
        # frame_lost_target = not seg_valid

        frame_has_target = yolo_valid and seg_valid
        frame_lost_target = (not yolo_valid) and (not seg_valid)


        # if seg_model is not None and segmentation_flag:
        #     try:
        #         with torch.no_grad():
        #             h, w = frame.shape[:2]
        #             # Preprocess
        #             img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #             img_tensor_s = img_rgb.astype('float32') / 255.0
        #             img_tensor_s = torch.from_numpy(img_tensor_s.transpose(2, 0, 1)).unsqueeze(0).to(device)
                    
        #             # Padding
        #             pad_h = (32 - h % 32) % 32
        #             pad_w = (32 - w % 32) % 32
        #             if pad_h > 0 or pad_w > 0:
        #                 img_tensor_s = F.pad(img_tensor_s, (0, pad_w, 0, pad_h), mode='constant', value=0)
                    
        #             # Inference
        #             seg_output = seg_model(img_tensor_s)
                    
        #             # 获取概率
        #             seg_probs = torch.softmax(seg_output, dim=1)
                    
        #             # 获取最大概率值和对应的类别索引
        #             max_vals, pred_mask = torch.max(seg_probs, dim=1)
                    
        #             pred_mask = pred_mask.squeeze(0).cpu().numpy()
        #             max_vals = max_vals.squeeze(0).cpu().numpy()
                    
        #             # 应用阈值过滤：只有当预测概率大于阈值时才保留，否则设为0 (背景)
        #             pred_mask[max_vals < seg_threshold] = 0
                    
        #             # Unpad
        #             if pad_h > 0 or pad_w > 0:
        #                 pred_mask = pred_mask[:h, :w]
                    
        #             # Y-axis Filter
        #             if seg_y_threshold > 0:
        #                 # y轴小于seg_y_threshold的区域设为背景(0)
        #                 pred_mask[:seg_y_threshold, :] = 0
        #             vessel_pixels = np.sum(pred_mask == 2)
        #             vessel_found_this_frame = vessel_pixels > min_vessel_pixels

        #             # Visualize
        #             colored_mask = colorize_mask(pred_mask, seg_color_map)
        #             colored_mask_bgr = cv2.cvtColor(colored_mask, cv2.COLOR_RGB2BGR)
        #             seg_overlay_bgr = cv2.addWeighted(frame, 1, colored_mask_bgr, 0.8, 0)
        #     except Exception as e:
        #         # print(f"Seg inference error: {e}") 
        #         pass
        # seg_overlay_bgr = cv2.resize(seg_overlay_bgr, (800, 600))
        cavana = seg_overlay_bgr.copy()
        # Draw valid y-range band first
        # cavana = draw_valid_center_y_band(cavana, valid_center_y_min, valid_center_y_max)
        # draw yolo before resize
        if yolo_flag:
            cavana = draw_yolo_overlay(cavana, yolo_det, yolo_count)

        cavana = cv2.resize(cavana, (1300, 1300))
        # Display camera feed (basic)

        key = cv2.waitKey(1)
        freeze_request = False
        
        if key == ord('q'):
            print("[Python] User requested quit")
            inference_running = False
            break
        elif key == ord('b'):
            trigger_inference = not trigger_inference
            scan_enabled = False
            termination_stable_count = 0
            pre_rotation_vessel_pixels = 0
            pre_rotation_vessel_pixels_locked = False
            auto_terminate_latched = False
            terminate_command_sequence = 0
            visual_y_conditioner.reset()
            current_phase_idx = 0
            phase_candidate_idx = 0
            phase_candidate_count = 0
            robot_action_state = "moving" if trigger_inference else "idle"
            command_publisher.publish(
                0.0,
                0.0,
                terminate_flag_key,
                False,
                phase_idx=-1,
                phase_confidence=0.0,
                robot_action_state=robot_action_state,
                mask_lr_majority=0,
            )
            print(
                "[Python] Inference/Z control "
                f"{'started' if trigger_inference else 'stopped'}; "
                "Tool-X/RZ remain disabled; Tool-Y centering is enabled"
            )
        elif key == ord('t'):
            terminate_flag_key = not terminate_flag_key
            print(f"[Python] Terminate flag set to {terminate_flag_key}")
        elif key == ord('m'):
            if not trigger_inference:
                print("[Python] Press b before enabling Tool-X scan")
            else:
                scan_enabled = not scan_enabled
                termination_stable_count = 0
                pre_rotation_vessel_pixels = 0
                pre_rotation_vessel_pixels_locked = False
                auto_terminate_latched = False
                terminate_command_sequence = 0
                current_phase_idx = 0
                phase_candidate_idx = 0
                phase_candidate_count = 0
                command_publisher.publish(
                    0.0,
                    0.0,
                    terminate_flag_key,
                    False,
                    phase_idx=0 if scan_enabled else -1,
                    phase_confidence=0.0,
                    robot_action_state="moving",
                    mask_lr_majority=0,
                )
                print(
                    "[Python] Tool-X scan and Tool-RZ "
                    f"{'enabled' if scan_enabled else 'disabled'}"
                )
        elif key == ord('s'):
            segmentation_flag = not segmentation_flag
        elif key == ord('y'):
            yolo_flag = not yolo_flag
            print(f"[Python] YOLO visualization set to {yolo_flag}")
        if trigger_inference:
                img_tensor = preprocess_cv2(frame, backbone, img_size).to(device)
                phase_img_tensor = preprocess_convnext_frame(
                    frame, phase_transform
                ).unsqueeze(0).to(device)
                
                with torch.no_grad():
                    with torch.cuda.amp.autocast(enabled=torch.cuda.is_available()):
                        # 1. Extract Backbone Features
                        # x shape: [1, 197, 768] (Sequence of patches)
                        x, feats, emb = extract_embedding(backbone, backbone_model, img_tensor, pick_indices=transformer_layers)
                        
                        # 2. Existing Heads Inference
                        pose = posehead(feats[-1], x)
                        pred_xy_real = inv_dT1e(pose)
                        angle = anglehead(feats[-1], x) * 90
                        
                        # 3. Latest ConvNeXt five-class phase inference.
                        latest_phase_logits = phase_classifier(phase_img_tensor)
                        latest_phase_probs = torch.softmax(
                            latest_phase_logits, dim=1
                        )
                        latest_phase_idx = int(
                            torch.argmax(latest_phase_probs, dim=1).item()
                        )
                        raw_phase_idx = CONVNEXT_TO_ROBOT_PHASE[
                            latest_phase_idx
                        ]
                        raw_phase_prob = float(
                            latest_phase_probs[0, latest_phase_idx].item()
                        )

                        # Aggregate probabilities only for reporting the
                        # confidence of the currently debounced RM75 phase.
                        phaseprobs = torch.stack(
                            (
                                latest_phase_probs[:, 0:3].sum(dim=1),
                                latest_phase_probs[:, 3],
                                latest_phase_probs[:, 4],
                            ),
                            dim=1,
                        )
                        print(
                            "[ConvNeXtPhase] "
                            f"label={CONVNEXT_LABELS[latest_phase_idx]} "
                            f"class={latest_phase_idx} "
                            f"mapped_phase={raw_phase_idx} "
                            f"confidence={raw_phase_prob:.3f}"
                        )

                        # Enforce phase order: allow rollback, but prevent forward jumps
                        next_allowed = min(current_phase_idx + 1, len(phase_order) - 1)
                        if raw_phase_idx > next_allowed:
                            raw_phase_idx = current_phase_idx

                        # Debounce / confidence gate to avoid frequent rollback to Scan
                        if raw_phase_idx != current_phase_idx:
                            required_prob = phase_min_prob.get(raw_phase_idx, 0.5)
                            required_count = phase_change_patience.get(raw_phase_idx, 3)

                            if raw_phase_prob >= required_prob:
                                if phase_candidate_idx == raw_phase_idx:
                                    phase_candidate_count += 1
                                else:
                                    phase_candidate_idx = raw_phase_idx
                                    phase_candidate_count = 1

                                if phase_candidate_count >= required_count:
                                    current_phase_idx = raw_phase_idx
                                    phase_candidate_count = 0
                            else:
                                phase_candidate_idx = current_phase_idx
                                phase_candidate_count = 0
                        else:
                            phase_candidate_idx = current_phase_idx
                            phase_candidate_count = 0

                        phase_idx = current_phase_idx
                        phase_confidence = float(
                            phaseprobs[0, phase_idx].detach().cpu().item()
                        )
                        
                        phase_str = PHASE_LABELS.get(phase_idx, "Unknown")
                        
                        # Extract Values
                        raw_y_value = float(pred_xy_real[0, 0].cpu().numpy())
                        y_value = visual_y_conditioner.update(raw_y_value)
                        rz_value = float(angle[0, 0].cpu().numpy())

                        # Before the first rotation phase of this m/scan run,
                        # retain the latest non-zero segmentation vessel area.
                        # Lock it when phase 2 begins; later phase rollback must
                        # not move the baseline used by the 3x completion gate.
                        if (
                            scan_enabled
                            and not pre_rotation_vessel_pixels_locked
                        ):
                            if phase_idx < 2 and vessel_pixels > 0:
                                pre_rotation_vessel_pixels = int(vessel_pixels)
                            elif (
                                phase_idx == 2
                                and pre_rotation_vessel_pixels > 0
                            ):
                                pre_rotation_vessel_pixels_locked = True
                                print(
                                    "[AutoTerminate] Pre-rotation vessel "
                                    "pixel baseline locked: "
                                    f"{pre_rotation_vessel_pixels}"
                                )
                        termination_frame_ready = normal_termination_frame_ready(
                            latest_phase_idx=latest_phase_idx,
                            rz_value=rz_value,
                            yolo_valid=yolo_valid,
                            seg_valid=seg_valid,
                            vessel_pixels=vessel_pixels,
                            pre_rotation_vessel_pixels=(
                                pre_rotation_vessel_pixels
                                if pre_rotation_vessel_pixels_locked else 0
                            ),
                            vessel_pixel_growth_ratio=(
                                termination_vessel_pixel_growth_ratio
                            ),
                            rz_tolerance_deg=termination_rz_tolerance_deg,
                        )
                        if scan_enabled:
                            termination_stable_count = (
                                termination_stable_count + 1
                                if termination_frame_ready else 0
                            )
                            if (
                                not auto_terminate_latched
                                and termination_stable_count
                                >= termination_stable_frames
                            ):
                                auto_terminate_latched = True
                                print(
                                    "[AutoTerminate] Completion conditions "
                                    "stable; terminate latched true"
                                )
                        else:
                            termination_stable_count = 0

                        # Automatic completion is only allowed to latch above
                        # while m/scan is enabled. Once latched, both the Redis
                        # command and the US-image UI remain explicitly true.
                        auto_terminate = auto_terminate_latched
                        terminate_value = bool(
                            terminate_flag_key or auto_terminate_latched
                        )
                        # Report/Hold uses exactly the same qualified automatic
                        # completion gate; there is no independent RZ shortcut.
                        is_hold = bool(scan_enabled and auto_terminate)
                        if freeze_on_hold_enabled and is_hold and not is_display_frozen:
                            freeze_request = True

                        if auto_capture_enabled and scan_enabled:
                            if phase_idx == 0 and not phase_capture_done["scan"]:
                                save_path = os.path.join(auto_capture_dir, f"scan_{int(time.time() * 1000)}.png")
                                if cv2.imwrite(save_path, frame):
                                    phase_capture_done["scan"] = True
                                    print(f"[AutoCapture] Saved scan frame: {save_path}")
                            if phase_idx in (1, 2) and not phase_capture_done["trigger"]:
                                save_path = os.path.join(auto_capture_dir, f"trigger_{int(time.time() * 1000)}.png")
                                if cv2.imwrite(save_path, frame):
                                    phase_capture_done["trigger"] = True
                                    print(f"[AutoCapture] Saved trigger frame: {save_path}")
                            if is_hold:
                                mask_for_measure = valid_seg_mask if seg_valid else last_valid_seg_mask
                                save_frame, measure = render_hold_measurement_overlay(
                                    frame,
                                    mask_for_measure,
                                    image_height_mm=image_total_height_mm,
                                )
                                if measure is not None:
                                    update_cca_diameter_json(measure["diameter_mm"], json_path="./cca.json")
                                    print(
                                        f"[AutoMeasure] Hold carotid diameter: "
                                        f"{measure['diameter_mm']:.2f} mm ({measure['diameter_px']:.1f} px)"
                                    )
                                save_path = os.path.join(auto_capture_dir, f"hold_{int(time.time() * 1000)}.png")
                                if cv2.imwrite(save_path, save_frame):
                                    phase_capture_done["hold"] = True
                                    print(f"[AutoCapture] Saved hold frame: {save_path}")


                        if phase_idx == 2:
                            if recovery_mode:
                                # 与原六轴退出语义一致：YOLO 与分割同时有效
                                # 时累计 found；两者同时丢失才清零 found；
                                # 只有一个检测器有效时保留计数，既不累加也不清零。
                                if frame_has_target:
                                    found_vessel_count += 1
                                    lost_vessel_count = 0
                                    if found_vessel_count >= recover_vessel_patience:
                                        recovery_mode = False
                                        found_vessel_count = 0
                                        print(
                                            "[Recovery] Vessel reacquired; "
                                            "leave rotation recovery"
                                        )
                                elif frame_lost_target:
                                    lost_vessel_count += 1
                                    found_vessel_count = 0
                            else:
                                # 未进入恢复时，只有 YOLO 与分割连续同时丢失
                                # 才进入旋转丢失恢复；C++ 在上升沿锁存此时
                                # mask_lr_majority：左侧驱动 -Tool-Y，右侧驱动
                                # +Tool-Y，固定速度 0.002 m/s，同时保持 X/RZ 为零。
                                found_vessel_count = 0
                                if frame_lost_target:
                                    lost_vessel_count += 1
                                    if lost_vessel_count >= lost_vessel_patience:
                                        recovery_mode = True
                                        lost_vessel_count = 0
                                else:
                                    lost_vessel_count = 0
                        else:
                            recovery_mode = False
                            lost_vessel_count = 0
                            found_vessel_count = 0


                        # [Updated Print] Include Phase
                        print(
                            f"[Infer] y_raw: {raw_y_value:.4f}, "
                            f"y_command: {y_value:.4f}, rz: {rz_value:.4f}, "
                            f"phase_raw: {latest_phase_idx}, phase: {phase_idx}, "
                            f"confidence: {phase_confidence:.3f}, "
                            f"finish_stable: {termination_stable_count}/"
                            f"{termination_stable_frames}, "
                            f"recovery_mode: {recovery_mode}"
                        )
                        
                        (
                            command_y,
                            command_rz,
                            command_phase_idx,
                            command_phase_confidence,
                            command_recovery_mode,
                            command_mask_lr_majority,
                        ) = gate_visual_command_for_scan(
                            y_value,
                            rz_value,
                            phase_idx,
                            phase_confidence,
                            recovery_mode,
                            mask_lr_majority,
                            scan_enabled,
                        )

                        # b 只允许 Z；m 开启后才发布 X/Y/RZ 所需的视觉量。
                        published_command = command_publisher.publish(
                            command_y,
                            command_rz,
                            terminate_value,
                            command_recovery_mode,
                            phase_idx=command_phase_idx,
                            phase_confidence=command_phase_confidence,
                            robot_action_state=robot_action_state,
                            mask_lr_majority=command_mask_lr_majority,
                        )
                        if (
                            auto_terminate_latched
                            and published_command is not None
                            and terminate_command_sequence == 0
                        ):
                            terminate_command_sequence = int(
                                published_command["sequence"]
                            )
                    
                    # --- Visualization ---
                    overlay_text = f"y: {y_value:.4f}"
                    phase_text = (
                        f"Phase: {phase_str}({phase_confidence:.2f})"
                    )
                    
                    # Phase Color (Scan:Green, Trigger:Red, Action:Blue)
                    p_color = (0, 255, 0) if phase_idx == 0 else (0, 0, 255) if phase_idx == 1 else (255, 0, 0)
                    
                    

                    cv2.putText(cavana, phase_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, p_color, 2, cv2.LINE_AA)
                    terminate_text = f"Terminate: {terminate_value}"
                    cv2.putText(
                        cavana, terminate_text, (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0) if auto_terminate_latched
                        else (255, 255, 255),
                        2, cv2.LINE_AA,
                    )
                    if y_value > 0.0003:
                        cv2.arrowedLine(cavana, (50, 50), (100, 50), (0, 0, 255), 2)
                        cv2.putText(cavana, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
                    elif y_value < -0.0003:
                        cv2.arrowedLine(cavana, (100, 50), (50, 50), (0, 255, 0), 2)
                        cv2.putText(cavana, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.putText(cavana, "On Track", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(cavana, f"rz: {rz_value:.2f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

                    cv2.putText(cavana, f"Recovery: {recovery_mode}", (10, 190),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)

                    cv2.putText(cavana, f"Vessel pixels: {vessel_pixels}", (10, 230),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

                    cv2.putText(cavana, f"Lost count: {lost_vessel_count}", (10, 270),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                    

        if segmentation_flag:
            cv2.putText(cavana, "Segmentation: ON", (1150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(cavana, "Segmentation: OFF", (1150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        cavana = draw_robot_status_overlay(
            cavana,
            latest_robot_status,
            command_publisher,
            robot_action_state,
            scan_enabled,
        )

        # if yolo_flag:
        #     cv2.putText(cavana, "YOLO: ON", (1150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        # else:
        #     cv2.putText(cavana, "YOLO: OFF", (1150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        if freeze_request and not is_display_frozen:
            frozen_display_frame = cavana.copy()
            is_display_frozen = True
            print("[Python] Hold condition met. Display frozen.")
            if (not report_generated_once) and (report_thread is None or not report_thread.is_alive()):
                print("[AutoReport] Starting report generation...")
                if command_publisher:
                    final_terminate_command = command_publisher.publish(
                        0.0,
                        0.0,
                        True,
                        False,
                        phase_idx=-1,
                        phase_confidence=0.0,
                        robot_action_state="idle",
                        mask_lr_majority=0,
                    )
                    if final_terminate_command is not None:
                        terminate_command_sequence = int(
                            final_terminate_command["sequence"]
                        )
                print("[Python] Sent final terminate command")
                report_generated_once = True
                report_thread = threading.Thread(target=_report_worker, daemon=True)
                report_thread.start()


        cv2.imshow('US image', cavana)
        # cv2.imshow('Segmentation Overlay', seg_overlay_bgr)
            # end_time = time.time()
            # print(f"Time: {end_time - begin_time:.4f}s")
            
    # Send final terminate command
    if command_publisher:
        command_publisher.stop_heartbeat()
        command_publisher.publish(
            0.0,
            0.0,
            True,
            False,
            phase_idx=-1,
            phase_confidence=0.0,
            robot_action_state="idle",
            mask_lr_majority=0,
        )
        print("[Python] Sent final terminate command")
    
    # Unsubscribe
    if redis_pubsub:
        redis_pubsub.unsubscribe()
        redis_pubsub.close()

    if camera is not None: camera.release()
    # if video_writer is not None: video_writer.release()
    cv2.destroyAllWindows()
    print("[Python] Stopped")

if __name__ == "__main__":
    if sys.argv[1:] == ["--check-command-protocol"]:
        check_command_protocol()
    else:
        main()
