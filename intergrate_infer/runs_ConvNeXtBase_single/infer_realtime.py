import argparse
import os
import socket
import struct
import time
import zlib
import re
from collections import deque
from typing import Deque, Dict, List, Tuple

import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision
import torchvision.transforms as T
from PIL import Image

try:
    from mamba_ssm import Mamba2 as Mamba
except ImportError:
    Mamba = None


REFERENCE_IMAGE_NAMES = {
    0: "pre.png",
    1: "in.png",
    2: "after.png",
    3: "brench.png",
    4: "rota.png",
}


class WirelessProbe:
    """Read ultrasound frames from probe stream."""

    def __init__(self, ip: str = "127.0.0.1", port: int = 59999):
        self.ip = ip
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.remaining_data = bytearray()

    def connect(self):
        print(f"[INFO] Connecting to probe at {self.ip}:{self.port} ...")
        self.client_socket.connect((self.ip, self.port))
        print("[INFO] Connected.")
        return self

    def read(self) -> Tuple[bool, np.ndarray]:
        """Return (ok, frame_bgr)."""
        while True:
            offset = 0
            while len(self.remaining_data) - offset >= 8:
                if self.remaining_data[offset] != 0xA5 or self.remaining_data[offset + 1] != 0xA5:
                    self.remaining_data = self.remaining_data[offset + 1 :]
                    offset = 0
                    continue

                msg_len = struct.unpack_from("i", self.remaining_data, offset + 4)[0]

                if len(self.remaining_data) - offset - 8 >= msg_len:
                    message = self.remaining_data[offset + 8 : offset + 8 + msg_len]
                    self.remaining_data = self.remaining_data[offset + 8 + msg_len :]
                    offset = 0

                    try:
                        expected_size = 640 * 480 * 4
                        uncompressed = zlib.decompress(message)
                        if len(uncompressed) != expected_size:
                            continue

                        img_rgba = np.frombuffer(uncompressed, dtype=np.uint8).reshape((480, 640, 4))
                        img_bgr = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)
                        return True, img_bgr
                    except zlib.error:
                        continue
                else:
                    break

            try:
                temp_buffer = self.client_socket.recv(16384)
                if len(temp_buffer) == 0:
                    return False, None
                self.remaining_data.extend(temp_buffer)
            except Exception:
                return False, None

    def release(self):
        try:
            self.client_socket.close()
        except Exception:
            pass


class MambaPlus(nn.Module):
    def __init__(self, d_model: int, share: bool = False):
        super().__init__()
        if Mamba is None:
            raise RuntimeError("mamba_ssm is required for sequence models. Please install it in this environment.")
        self.d_model = d_model
        self.norm1 = nn.LayerNorm(d_model)
        self.mamba1 = Mamba(d_model=d_model)
        self.norm2 = nn.LayerNorm(d_model)
        self.mamba2 = Mamba(d_model=d_model)

        if share:
            self.mamba2.in_proj = self.mamba1.in_proj
            self.mamba2.out_proj = self.mamba1.out_proj
            self.out_fc = nn.Identity()
        else:
            self.out_fc = nn.Linear(d_model, d_model, bias=False)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        _, _, c = x.shape
        assert c == self.d_model
        x1 = self.mamba1(self.norm1(x))
        x2 = self.mamba2(self.norm2(torch.flip(x, dims=[1]))).flip(dims=[1])
        return self.out_fc(x1 + x2)


class MultiMambaPlus(nn.Module):
    def __init__(self, d_model: int, n_layers: int = 1, share: bool = False):
        super().__init__()
        self.d_model = d_model
        self.n_layers = n_layers
        self.share = share
        for idx in range(n_layers):
            setattr(self, f"mamba{idx}", MambaPlus(d_model=d_model, share=share))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        for idx in range(self.n_layers):
            x = getattr(self, f"mamba{idx}")(x) + x
        return x


class SingleConvNeXtClassifier(nn.Module):
    """Matches train_single.py's ConvNeXt-Base classifier."""

    def __init__(self, num_classes: int = 5):
        super().__init__()
        self.backbone = torchvision.models.convnext_base(weights=None)
        self.backbone.classifier[2] = nn.Linear(1024, num_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() == 5:
            x = x[:, -1]
        return self.backbone(x)


class SeqConvNeXtMambaClassifier(nn.Module):
    """Matches train_seq.py's ConvNeXt-Base + Mamba sequence classifier."""

    def __init__(self, num_classes: int = 5):
        super().__init__()
        self.backbone = torchvision.models.convnext_base(weights=None)
        self.mamba = MultiMambaPlus(1024, n_layers=1)
        self.backbone.classifier = nn.Linear(1024, num_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() != 5:
            raise ValueError(f"Sequence model expects [B,T,C,H,W], got shape {tuple(x.shape)}")
        b, t, _, _, _ = x.shape
        xs = []
        for idt in range(t):
            xt = self.backbone.features(x[:, idt])
            xt = self.backbone.avgpool(xt)
            xs.append(xt)
        x = torch.stack(xs, dim=1).view(b, t, -1)
        x = self.mamba(x)[:, -1]
        return self.backbone.classifier(x)


def get_state_dict(ckpt: object) -> Dict[str, torch.Tensor]:
    if isinstance(ckpt, dict) and "model" in ckpt:
        return ckpt["model"]
    if isinstance(ckpt, dict) and "state_dict" in ckpt:
        return ckpt["state_dict"]
    if isinstance(ckpt, dict):
        return ckpt
    raise ValueError("Unsupported checkpoint format. Expected a dict or a dict with key 'model'.")


def infer_model_type(state_dict: Dict[str, torch.Tensor], requested: str) -> str:
    if requested != "auto":
        return requested
    has_mamba = any(key.startswith("mamba.") for key in state_dict)
    return "seq" if has_mamba else "single"


def infer_num_classes(state_dict: Dict[str, torch.Tensor], requested: int) -> int:
    if requested > 0:
        return requested
    for key in ("backbone.classifier.2.weight", "backbone.classifier.weight"):
        if key in state_dict:
            return int(state_dict[key].shape[0])
    return 5


def infer_seq_len(checkpoint: str, requested: int) -> int:
    if requested > 0:
        return requested
    match = re.search(r"seq(\d+)", checkpoint)
    if match:
        return int(match.group(1))
    return 10


def build_model_from_checkpoint(args: argparse.Namespace, device: torch.device) -> Tuple[nn.Module, str, int, int]:
    if not os.path.isfile(args.checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {args.checkpoint}")

    ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    state_dict = get_state_dict(ckpt)
    model_type = infer_model_type(state_dict, args.model_type)
    num_classes = infer_num_classes(state_dict, args.num_classes)
    seq_len = infer_seq_len(args.checkpoint, args.seq_len) if model_type == "seq" else 1

    if model_type == "single":
        model = SingleConvNeXtClassifier(num_classes=num_classes)
    elif model_type == "seq":
        model = SeqConvNeXtMambaClassifier(num_classes=num_classes)
    else:
        raise ValueError("--model-type must be one of: auto, single, seq")

    msg = model.load_state_dict(state_dict, strict=True)
    model.to(device).eval()
    print(f"[INFO] Loaded {model_type} model from {args.checkpoint}")
    print(f"[INFO] load_state_dict: {msg}")
    print(f"[INFO] num_classes={num_classes}, seq_len={seq_len}")
    return model, model_type, num_classes, seq_len


def build_transform(image_size: int = 224) -> T.Compose:
    return T.Compose(
        [
            T.Resize((image_size, image_size)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )


def parse_label_names(label_names: str, num_classes: int) -> List[str]:
    if label_names:
        labels = [name.strip() for name in label_names.split(",") if name.strip()]
        if len(labels) != num_classes:
            raise ValueError(f"label-names count ({len(labels)}) must match num-classes ({num_classes}).")
        return labels
    return [str(i) for i in range(num_classes)]


def preprocess_frame(frame_bgr: np.ndarray, transform: T.Compose) -> torch.Tensor:
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    return transform(pil_img)


def phase_color(phase_idx: int) -> Tuple[int, int, int]:
    palette = [
        (0, 255, 0),
        (0, 165, 255),
        (255, 0, 0),
        (255, 0, 255),
        (0, 255, 255),
        (128, 128, 255),
        (255, 128, 0),
        (0, 128, 255),
    ]
    if phase_idx < 0:
        return (200, 200, 200)
    return palette[phase_idx % len(palette)]


def overlay_label(
    frame: np.ndarray,
    label: str,
    score: float,
    fps: float,
    color: Tuple[int, int, int],
    status: str = "",
) -> np.ndarray:
    display = frame.copy()
    text = f"Pred: {label} ({score:.2f})"
    cv2.putText(display, text, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    cv2.putText(display, f"FPS: {fps:.1f}", (10, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
    if status:
        cv2.putText(display, status, (10, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 220), 2)
    return display


def load_reference_images(resource_dir: str = "resource") -> Dict[int, np.ndarray]:
    references = {}
    for class_idx, filename in REFERENCE_IMAGE_NAMES.items():
        path = os.path.join(resource_dir, filename)
        image = cv2.imread(path)
        if image is None:
            print(f"[WARN] Reference image not found or unreadable: {path}")
            continue
        references[class_idx] = image
    return references


def append_reference_image(display: np.ndarray, references: Dict[int, np.ndarray], pred_idx: int) -> np.ndarray:
    reference = references.get(pred_idx)
    if reference is None:
        return display

    target_h = display.shape[0]
    ref_h, ref_w = reference.shape[:2]
    target_w = max(1, int(round(ref_w * target_h / ref_h)))
    resized_reference = cv2.resize(reference, (target_w, target_h), interpolation=cv2.INTER_AREA)
    return cv2.hconcat([display, resized_reference])


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Realtime inference from wireless ultrasound probe.")
    parser.add_argument("--ip", default="127.0.0.1", help="Probe server IP")
    parser.add_argument("--port", type=int, default=59999, help="Probe server port")
    parser.add_argument("--checkpoint", default="./runs_ConvNeXtBase_single/best_val_acc_model.pth", help="Checkpoint exported by train_single.py or train_seq.py")
    parser.add_argument("--model-type", choices=["auto", "single", "seq"], default="auto", help="Model type. auto detects Mamba keys in checkpoint.")
    parser.add_argument("--seq-len", type=int, default=0, help="Sequence length for train_seq.py models. 0 means infer from checkpoint path, then fallback to 10.")
    parser.add_argument("--num-classes", type=int, default=0, help="Number of output classes. 0 means infer from classifier weight.")
    parser.add_argument("--label-names", default="", help="Comma-separated label names")
    parser.add_argument("--image-size", type=int, default=224, help="Inference input size")
    parser.add_argument("--device", default="", help="cuda, cuda:0, or cpu (auto if empty)")
    parser.add_argument("--skip", type=int, default=0, help="Skip N frames between inference")
    parser.add_argument("--window", default="Carotid Inference", help="OpenCV window title")
    parser.add_argument("--no-display", action="store_true", help="Disable OpenCV display")
    return parser


def main() -> None:
    args = build_argparser().parse_args()

    device = (
        torch.device(args.device)
        if args.device
        else torch.device("cuda" if torch.cuda.is_available() else "cpu")
    )
    torch.backends.cudnn.benchmark = True

    transform = build_transform(args.image_size)

    model, model_type, num_classes, seq_len = build_model_from_checkpoint(args, device)
    labels = parse_label_names(args.label_names, num_classes)

    probe = WirelessProbe(args.ip, args.port)
    try:
        probe.connect()
    except Exception as exc:
        print(f"[ERROR] Failed to connect probe: {exc}")
        return

    if not args.no_display:
        references = load_reference_images()
        cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)
    else:
        references = {}

    frame_idx = 0
    last_time = time.time()
    fps = 0.0
    frame_buffer: Deque[torch.Tensor] = deque(maxlen=seq_len)

    print("[INFO] Press 'q' to quit.")

    try:
        while True:
            ok, frame = probe.read()
            if not ok or frame is None:
                print("[WARN] Stream disconnected or frame read failed.")
                break

            if args.skip > 0 and frame_idx % (args.skip + 1) != 0:
                frame_idx += 1
                if not args.no_display:
                    cv2.imshow(args.window, frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                continue

            with torch.no_grad():
                tensor = preprocess_frame(frame, transform)
                frame_buffer.append(tensor)
                if model_type == "seq" and len(frame_buffer) < seq_len:
                    now = time.time()
                    fps = 1.0 / max(1e-6, now - last_time)
                    last_time = now
                    display = overlay_label(
                        frame,
                        "warming",
                        0.0,
                        fps,
                        (200, 200, 200),
                        status=f"Buffer: {len(frame_buffer)}/{seq_len}",
                    )
                    if not args.no_display:
                        cv2.imshow(args.window, display)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
                    frame_idx += 1
                    continue

                if model_type == "seq":
                    batch = torch.stack(list(frame_buffer), dim=0).unsqueeze(0).to(device)
                else:
                    batch = tensor.unsqueeze(0).to(device)

                with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                    logits = model(batch)
                    probs = torch.softmax(logits, dim=1)
                    score, pred = probs.max(dim=1)

            now = time.time()
            fps = 1.0 / max(1e-6, now - last_time)
            last_time = now

            pred_idx = int(pred.item())
            label = labels[pred_idx] if pred_idx < len(labels) else str(pred_idx)
            color = phase_color(pred_idx)
            status = f"{model_type} model"
            if model_type == "seq":
                status = f"seq model | Buffer: {len(frame_buffer)}/{seq_len}"
            display = overlay_label(frame, label, float(score.item()), fps, color, status=status)
            display = append_reference_image(display, references, pred_idx)

            if not args.no_display:
                cv2.imshow(args.window, display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_idx += 1

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    finally:
        probe.release()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
